/** @file bench_motion.c
 *
 * @brief Drives each motor on its own, reports which encoders answered,
 *        then drives a fixed distance and prints the pulse counts.
 *
 * Build with `make BENCH=motion` and flash to a board with the motors on
 * M1 and M2. Wheels off the ground. Four phases:
 *
 *   1. Left motor alone, M1, for two seconds.
 *   2. Right motor alone, M2, for two seconds.
 *   3. Both together, so the pair can be compared.
 *   4. A measured drive of BENCH_DISTANCE_MM.
 *
 * Phases 1 and 2 are what isolate a dead motor. If only one of them turns
 * a wheel, the fault is that motor, its screw terminal, or that channel of
 * the board, and the board's own M1A and M2A test buttons will say which.
 *
 * WARNING: The motors take their power from the battery through VIN, not
 * from USB. On USB alone the rail sags as soon as a motor starts and the
 * board resets, which looks like a firmware crash.
 *
 * Owner: Buddy 2, motion control. Extend it as you need; nothing else depends
 * on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "car_time.h"
#include "motion.h"

#define BENCH_DISTANCE_MM     500u
#define BENCH_STARTUP_MSEC   2000u
#define BENCH_SINGLE_MSEC    2000u
#define BENCH_PAUSE_MSEC     1000u
#define BENCH_SPEED_PRINT_MSEC 100u   /* In motion ticks' own counting */
#define BENCH_ONE_WHEEL      1000    /* Steer hard enough to stop the other */

static void run_for (int16_t steer_permille, uint32_t msec,
                     char const * p_what);
static void report_encoders (void);

INT usermain (void)
{
    motion_state_t state = { 0 };

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    (void)car_time_start_ticks();
    CAR_LOG(CAR_LOG_INFO, "motion bench: encoders GP%u GP%u\n",
            ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN);
    CAR_LOG(CAR_LOG_INFO, "wheels off the ground, battery connected\n");

    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed\n");
    }
    else
    {
        (void)motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC);
        CAR_LOG(CAR_LOG_INFO, "target %u mm/s per wheel\n",
                MOTION_DEFAULT_SPEED_MM_PER_SEC);

        run_for(BENCH_ONE_WHEEL, BENCH_SINGLE_MSEC, "left only, M1");
        run_for(-BENCH_ONE_WHEEL, BENCH_SINGLE_MSEC, "right only, M2");
        run_for(0, BENCH_SINGLE_MSEC, "both together");
        report_encoders();

        CAR_LOG(CAR_LOG_INFO, "driving %u mm forward\n", BENCH_DISTANCE_MM);
        (void)motion_move_forward(BENCH_DISTANCE_MM);

        while (motion_is_busy())
        {
            if (CAR_ERR_HARDWARE == motion_tick())
            {
                (void)motion_get_state(&state);
                CAR_LOG(CAR_LOG_ERROR,
                        "a turning wheel stopped pulsing: left %u right %u\n",
                        state.encoder_count_left, state.encoder_count_right);
                break;
            }

            car_time_wait_tick();
        }

        (void)motion_stop();
        (void)motion_get_state(&state);
        CAR_LOG(CAR_LOG_INFO, "final counts: left %u right %u\n",
                state.encoder_count_left, state.encoder_count_right);
    }

    /* The initial task must never return: the kernel shuts down if it does. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}

/**
 * @brief Drive with a fixed steering bias for a while, then stop.
 *
 * @param[in] steer_permille Bias; 1000 stops the right wheel, -1000 the left.
 * @param[in] msec           How long to hold it.
 * @param[in] p_what         Name for the log line.
 */
static void run_for (int16_t steer_permille, uint32_t msec,
                     char const * p_what)
{
    uint32_t elapsed = 0u;

    CAR_LOG(CAR_LOG_INFO, "%s\n", p_what);
    (void)motion_drive_steer(steer_permille);

    for (elapsed = 0u; elapsed < msec; elapsed += MOTION_TICK_PERIOD_MSEC)
    {
        (void)motion_tick();

        /* Live speeds, signed by the way each wheel really turned. Driving
         * forward, a negative wheel has its ENCODER_*_B_FORWARD backwards.
         * A reading that leaps about while the wheel sounds steady is
         * encoder noise; a smooth swing either side of the target is the
         * speed loop hunting. */
        if (0u == (elapsed % BENCH_SPEED_PRINT_MSEC))
        {
            motion_state_t state = { 0 };

            (void)motion_get_state(&state);
            CAR_LOG(CAR_LOG_INFO, "  speed left %d right %d mm/s\n",
                    state.left_mm_per_sec, state.right_mm_per_sec);
        }

        car_time_wait_tick();
    }

    (void)motion_stop();
    (void)tk_dly_tsk(BENCH_PAUSE_MSEC);
}

/**
 * @brief Say which encoders produced a pulse during the phases above.
 *
 * NOTE: An encoder that never pulses is treated as absent, not broken, so
 * the car still drives. This is the only place that says so out loud.
 */
static void report_encoders (void)
{
    motion_state_t state = { 0 };

    if (CAR_OK == motion_get_state(&state))
    {
        CAR_LOG(CAR_LOG_INFO, "encoders seen: left %d right %d\n",
                state.b_left_encoder, state.b_right_encoder);

        if ((!state.b_left_encoder) || (!state.b_right_encoder))
        {
            CAR_LOG(CAR_LOG_ERROR,
                    "a missing encoder reads zero speed, so its wheel is "
                    "driven hard: check its wiring\n");
        }
    }
}

/*** end of file ***/
