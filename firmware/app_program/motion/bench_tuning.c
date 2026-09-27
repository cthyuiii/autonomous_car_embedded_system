/** @file bench_tuning.c
 *
 * @brief Measures the speed loop's step response and the accuracy of
 *        distance moves and turns, for motion/tuning_report.md.
 *
 * Build and flash with `./flash.sh tuning`, battery connected. Two parts,
 * both printing Markdown table rows that paste straight into the report:
 *
 *   1. Step response, wheels OFF the ground. From rest to each speed in
 *      g_step_speeds for BENCH_STEP_MSEC. Per wheel: rise time from 10 to
 *      90 percent of the target, overshoot, and the mean error and ripple
 *      (highest minus lowest) over the last BENCH_SETTLED_MSEC.
 *   2. Accuracy, ON the floor. BENCH_RUNS drives of BENCH_DISTANCE_MM,
 *      then BENCH_RUNS left and BENCH_RUNS right turns of BENCH_TURN_DEG.
 *      Before each, BENCH_PLACE_MSEC to measure the last one and put the
 *      car back on its start mark. Each row is what the encoders say; the
 *      tape measure fills the last column.
 *
 * NOTE: Motion is ticked through every wait, as the car's motion task
 * does, so pulses from lifting and placing the car are taken in while it
 * stands still rather than landing in the next move's first tick.
 *
 * Owner: Buddy 2, motion control. Extend it as you need; nothing else
 * depends on it.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "car_time.h"
#include "motion.h"

#define BENCH_STARTUP_MSEC     2000u
#define BENCH_STEP_MSEC        3000u
#define BENCH_SETTLED_MSEC     1000u   /* Tail of a step counted as settled */
#define BENCH_SPIN_DOWN_MSEC   1500u
#define BENCH_FLOOR_MSEC      15000u   /* To put the car on the floor */
#define BENCH_PLACE_MSEC      15000u   /* To measure and re-place it */
#define BENCH_MOVE_LIMIT_MSEC 10000u   /* A move that runs longer failed */
#define BENCH_COAST_MSEC        500u   /* The coast is part of the result */
#define BENCH_STEPS               3u
#define BENCH_RUNS                3u
#define BENCH_DISTANCE_MM       500u
#define BENCH_TURN_DEG           90u
/* Tenths of a degree per mm of wheel difference, times WHEEL_BASE_MM:
 * 1800 / pi for a heading, 3600 / pi for a turn's arc. */
#define BENCH_HEADING_TENTHS    573
#define BENCH_ARC_TENTHS       1146u

/** One wheel's step response, gathered a sample at a time. */
typedef struct
{
    uint32_t rise_start_msec;   /* First at 10 percent, 0 until then */
    uint32_t rise_end_msec;     /* First at 90 percent, 0 until then */
    int32_t  peak;
    int32_t  sum;               /* Over the settled tail */
    uint32_t samples;
    int32_t  low;
    int32_t  high;
} step_stats_t;

static uint16_t const g_step_speeds[BENCH_STEPS] = { 100u, 200u, 300u };

static uint32_t step_response (uint16_t target);
static void     note_speed (step_stats_t * p_stats, int32_t speed,
                            uint16_t target, uint32_t elapsed_msec);
static void     print_step (uint16_t target, char const * p_wheel,
                            step_stats_t const * p_stats);
static void     accuracy_drive (uint8_t run);
static void     accuracy_turn (bool b_left, uint8_t run);
static bool     run_move (uint32_t * p_left, uint32_t * p_right,
                          uint32_t * p_msec);
static uint32_t pulses_to_milli_mm (uint32_t pulses);
static void     idle (uint32_t msec);
static void     print_noise (char const * p_when);

INT usermain (void)
{
    uint8_t  index       = 0u;
    uint32_t period_msec = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    (void)car_time_start_ticks();
    CAR_LOG(CAR_LOG_INFO, "tuning bench: KP %u KI %u KD %u milli, "
            "straight KP %u, %u pulses per turn\n", MOTION_PID_KP_MILLI,
            MOTION_PID_KI_MILLI, MOTION_PID_KD_MILLI, MOTION_STRAIGHT_KP,
            ENCODER_SLOTS_PER_REV);


    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed\n");
    }

    CAR_LOG(CAR_LOG_INFO, "part 1, step response: wheels OFF the ground\n");
    CAR_LOG(CAR_LOG_INFO, "| Target mm/s | Wheel | Rise 10-90%% ms "
            "| Overshoot %% | Steady error mm/s | Ripple mm/s |\n");
    CAR_LOG(CAR_LOG_INFO, "|---|---|---|---|---|---|\n");

    for (index = 0u; index < BENCH_STEPS; index++)
    {
        period_msec = step_response(g_step_speeds[index]);
        idle(BENCH_SPIN_DOWN_MSEC);
    }

    CAR_LOG(CAR_LOG_INFO, "motion loop ran every %u ms; the PID and the "
            "stall timers count %u\n", period_msec, MOTION_TICK_PERIOD_MSEC);
    print_noise("after the step test");

    CAR_LOG(CAR_LOG_INFO, "part 2, accuracy: put the car ON the floor, "
            "%u s\n", BENCH_FLOOR_MSEC / 1000u);
    idle(BENCH_FLOOR_MSEC);
    CAR_LOG(CAR_LOG_INFO, "| Run | Asked mm | Encoders mm "
            "| Heading deg, + right | Time ms | Tape mm "
            "| Sideways mm, + right |\n");
    CAR_LOG(CAR_LOG_INFO, "|---|---|---|---|---|---|---|\n");

    for (index = 1u; index <= BENCH_RUNS; index++)
    {
        accuracy_drive(index);
    }

    print_noise("after the drives");

    CAR_LOG(CAR_LOG_INFO, "| Run | Asked deg | Encoders deg | Time ms "
            "| Protractor deg |\n");
    CAR_LOG(CAR_LOG_INFO, "|---|---|---|---|---|\n");

    for (index = 1u; index <= BENCH_RUNS; index++)
    {
        accuracy_turn(true, index);
    }

    for (index = 1u; index <= BENCH_RUNS; index++)
    {
        accuracy_turn(false, index);
    }

    CAR_LOG(CAR_LOG_INFO, "tuning bench done\n");
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}

/**
 * @brief Drive both wheels from rest to one speed and print their rows.
 *
 * @param[in] target Speed to step to, mm per second.
 *
 * @return How often the loop really ran, in ms.
 */
static uint32_t step_response (uint16_t target)
{
    step_stats_t left    = { 0 };
    step_stats_t right   = { 0 };
    uint32_t     start   = car_time_msec();
    uint32_t     elapsed = 0u;
    uint32_t     ticks   = 0u;

    left.low   = INT32_MAX;
    left.high  = INT32_MIN;
    right.low  = INT32_MAX;
    right.high = INT32_MIN;

    (void)motion_set_speed(target);
    (void)motion_drive_steer(0);

    while (elapsed < BENCH_STEP_MSEC)
    {
        motion_state_t state = { 0 };

        (void)motion_tick();
        (void)motion_get_state(&state);
        elapsed = car_time_msec() - start;
        note_speed(&left, state.left_mm_per_sec, target, elapsed);
        note_speed(&right, state.right_mm_per_sec, target, elapsed);
        ticks++;
        car_time_wait_tick();
    }

    (void)motion_stop();
    print_step(target, "left", &left);
    print_step(target, "right", &right);

    return elapsed / ticks;
}

/**
 * @brief Take one speed sample into a wheel's step statistics.
 *
 * @param[in,out] p_stats      Statistics for this wheel.
 * @param[in]     speed        Measured speed, mm per second.
 * @param[in]     target       Speed asked for.
 * @param[in]     elapsed_msec Time since the step.
 */
static void note_speed (step_stats_t * p_stats, int32_t speed,
                        uint16_t target, uint32_t elapsed_msec)
{
    /* Casts: a uint16_t target, exact as int32_t, and at most 300 here. */
    if ((0u == p_stats->rise_start_msec) && ((speed * 10) >= (int32_t)target))
    {
        p_stats->rise_start_msec = elapsed_msec;
    }

    /* Casts: as above. */
    if ((0u == p_stats->rise_end_msec)
        && ((speed * 10) >= ((int32_t)target * 9)))
    {
        p_stats->rise_end_msec = elapsed_msec;
    }

    if (speed > p_stats->peak)
    {
        p_stats->peak = speed;
    }

    if (elapsed_msec >= (BENCH_STEP_MSEC - BENCH_SETTLED_MSEC))
    {
        p_stats->sum += speed;
        p_stats->samples++;

        if (speed < p_stats->low)
        {
            p_stats->low = speed;
        }

        if (speed > p_stats->high)
        {
            p_stats->high = speed;
        }
    }
}

/**
 * @brief Print one wheel's step response as a table row.
 *
 * @param[in] target  Speed asked for.
 * @param[in] p_wheel Name for the row.
 * @param[in] p_stats What note_speed() gathered.
 */
static void print_step (uint16_t target, char const * p_wheel,
                        step_stats_t const * p_stats)
{
    int32_t overshoot = 0;
    int32_t mean      = 0;

    /* Casts: the target is a uint16_t, exact as int32_t, and the sample
     * count is at most a few hundred per step. */
    if (p_stats->peak > (int32_t)target)
    {
        overshoot = ((p_stats->peak - (int32_t)target) * 100)
                    / (int32_t)target;
    }

    /* Cast: as above. */
    if (0u != p_stats->samples)
    {
        mean = p_stats->sum / (int32_t)p_stats->samples;
    }

    /* Casts: the target is a uint16_t, exact as int32_t. */
    if (0u == p_stats->rise_end_msec)
    {
        CAR_LOG(CAR_LOG_INFO, "| %u | %s | never reached 90%% | %d | %d "
                "| %d |\n", target, p_wheel, overshoot,
                mean - (int32_t)target, p_stats->high - p_stats->low);
    }
    else
    {
        CAR_LOG(CAR_LOG_INFO, "| %u | %s | %u | %d | %d | %d |\n", target,
                p_wheel, p_stats->rise_end_msec - p_stats->rise_start_msec,
                overshoot, mean - (int32_t)target,
                p_stats->high - p_stats->low);
    }
}

/**
 * @brief One measured straight drive.
 *
 * @param[in] run Run number for the row.
 */
static void accuracy_drive (uint8_t run)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    uint32_t msec  = 0u;

    CAR_LOG(CAR_LOG_INFO, "drive %u in %u s: car on the start mark, facing "
            "along the tape\n", run, BENCH_PLACE_MSEC / 1000u);
    idle(BENCH_PLACE_MSEC);
    (void)motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC);
    (void)motion_move_forward(BENCH_DISTANCE_MM);

    if (run_move(&left, &right, &msec))
    {
        /* Casts: one 500 mm drive is well under 2^31 thousandths of a
         * mm, and the wheel base is a small constant. */
        int32_t left_mmm  = (int32_t)pulses_to_milli_mm(left);
        int32_t right_mmm = (int32_t)pulses_to_milli_mm(right);
        int32_t heading   = ((left_mmm - right_mmm) * BENCH_HEADING_TENTHS)
                            / ((int32_t)WHEEL_BASE_MM * 1000);
        int32_t magnitude = (heading < 0) ? -heading : heading;

        CAR_LOG(CAR_LOG_INFO, "| drive %u | %u | %d | %c%d.%d | %u |  |  |\n",
                run, BENCH_DISTANCE_MM, (left_mmm + right_mmm) / 2000,
                (heading < 0) ? '-' : '+', magnitude / 10, magnitude % 10,
                msec);
    }
}

/**
 * @brief One measured turn on the spot.
 *
 * @param[in] b_left true for a left turn.
 * @param[in] run    Run number for the row.
 */
static void accuracy_turn (bool b_left, uint8_t run)
{
    char const * p_name = b_left ? "left" : "right";
    uint32_t     left   = 0u;
    uint32_t     right  = 0u;
    uint32_t     msec   = 0u;

    CAR_LOG(CAR_LOG_INFO, "turn %s %u in %u s: car on its mark, pointing "
            "along the tape\n", p_name, run, BENCH_PLACE_MSEC / 1000u);
    idle(BENCH_PLACE_MSEC);

    if (b_left)
    {
        (void)motion_turn_left(BENCH_TURN_DEG);
    }
    else
    {
        (void)motion_turn_right(BENCH_TURN_DEG);
    }

    if (run_move(&left, &right, &msec))
    {
        uint32_t arc_mmm = (pulses_to_milli_mm(left)
                            + pulses_to_milli_mm(right)) / 2u;
        uint32_t tenths  = (arc_mmm * BENCH_ARC_TENTHS)
                           / (WHEEL_BASE_MM * 1000u);

        CAR_LOG(CAR_LOG_INFO, "| %s %u | %u | %u.%u | %u |  |\n", p_name,
                run, BENCH_TURN_DEG, tenths / 10u, tenths % 10u, msec);
    }
}

/**
 * @brief Run the move just started to its end, then let the car coast.
 *
 * @param[out] p_left  Left pulses during the move and the coast.
 * @param[out] p_right Right pulses during the move and the coast.
 * @param[out] p_msec  How long the move took, coast excluded.
 *
 * @return false if the move timed out or a wheel stalled.
 */
static bool run_move (uint32_t * p_left, uint32_t * p_right,
                      uint32_t * p_msec)
{
    motion_state_t before  = { 0 };
    motion_state_t after   = { 0 };
    uint32_t       start   = car_time_msec();
    bool           b_fault = false;

    (void)motion_get_state(&before);

    while ((motion_is_busy()) && (!b_fault)
           && ((car_time_msec() - start) < BENCH_MOVE_LIMIT_MSEC))
    {
        b_fault = (CAR_ERR_HARDWARE == motion_tick());
        car_time_wait_tick();
    }

    *p_msec = car_time_msec() - start;
    b_fault = b_fault || (motion_is_busy());
    (void)motion_stop();
    idle(BENCH_COAST_MSEC);
    (void)motion_get_state(&after);
    *p_left  = after.encoder_count_left - before.encoder_count_left;
    *p_right = after.encoder_count_right - before.encoder_count_right;

    if (b_fault)
    {
        CAR_LOG(CAR_LOG_ERROR, "move did not finish: a wheel stalled or "
                "it ran past %u ms\n", BENCH_MOVE_LIMIT_MSEC);
    }

    return !b_fault;
}

/**
 * @brief Distance a wheel covers for a number of pulses.
 *
 * @param[in] pulses Encoder pulses, up to about 22000.
 *
 * @return Thousandths of a millimetre.
 */
static uint32_t pulses_to_milli_mm (uint32_t pulses)
{
    return (pulses * WHEEL_CIRCUMFERENCE_MM * 1000u) / ENCODER_SLOTS_PER_REV;
}

/**
 * @brief Wait, ticking motion as the car's motion task would.
 *
 * @param[in] msec How long.
 */
static void idle (uint32_t msec)
{
    uint32_t start = car_time_msec();

    while ((car_time_msec() - start) < msec)
    {
        (void)motion_tick();
        car_time_wait_tick();
    }
}

/**
 * @brief Print how much encoder noise has been thrown away so far.
 *
 * NOTE: The step test and the drives only ever turn the wheels forward, so
 * any reversal counted by then is noise on phase B, and any glitch is
 * noise on phase A. Noise on one wheel makes its count run ahead, which
 * the straight line correction answers by slowing that wheel: the car then
 * curves toward it while the encoders say it went straight.
 *
 * @param[in] p_when Which point of the run this is.
 */
static void print_noise (char const * p_when)
{
    motion_state_t state = { 0 };

    (void)motion_get_state(&state);
    CAR_LOG(CAR_LOG_INFO, "encoder noise %s: left %u glitches %u reversals, "
            "right %u glitches %u reversals\n", p_when, state.glitches_left,
            state.reversals_left, state.glitches_right,
            state.reversals_right);
}

/*** end of file ***/
