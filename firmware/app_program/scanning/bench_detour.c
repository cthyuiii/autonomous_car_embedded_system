/** @file bench_detour.c
 *
 * @brief Drives straight and detours round whatever it meets. No line.
 *
 * Build with `make BENCH=detour`, or `./flash.sh detour`. This is the bench
 * for the question "does the box detour actually go round the obstacle",
 * asked without the line sensors, the barcode, the IMU or the mission state
 * machine being involved at all.
 *
 * It drives straight until the forward ranging sees something inside
 * SCAN_OBSTACLE_RANGE_MM, stops, looks at the three scan angles, prints
 * every reading and the decision made from them, drives the detour leg by
 * leg with the leg number announced, and then resumes straight. Put a box
 * in front of it, off to one side, and watch which way it goes.
 *
 * WARNING: There is no line following here, so nothing brings the car back
 * to anything. It drives in whatever direction the last detour left it
 * pointing. Give it floor space, and keep a hand near the power switch.
 *
 * WARNING: Battery. Both the motors and the servo run from the board's
 * motor rail, so on USB alone the wheels will not turn and the horn will
 * not move however healthy this print looks.
 *
 * NOTE on what to expect. The three angles are SCAN_MIN, SCAN_CENTRE and
 * SCAN_MAX_ANGLE_DEG, 25 degrees apart, which is wider than the sonar's own
 * 15 degree beam. So the left and right readings really are looking
 * somewhere the centre reading is not, which is what lets the planner pick
 * a side. An obstacle dead ahead with clear floor to the left should give a
 * large left reading, a small centre one, and a decision to go left.
 *
 * NOTE: If it always reverses, the two side readings are both coming back
 * under SCAN_CLEARANCE_MIN_MM. Either the obstacle really is wide, or the
 * sonar is seeing the floor or the car's own bodywork at an angle. The
 * printed readings tell you which.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_hw.h"
#include "car_log.h"
#include "car_time.h"
#include "motion.h"
#include "scan.h"

#define BENCH_STARTUP_MSEC     3000u
#define BENCH_SPEED_MM_PER_SEC  120u
#define BENCH_SETTLE_MSEC       500u

static uint16_t const g_angles_deg[SCAN_COARSE_ANGLE_COUNT] =
    SCAN_COARSE_ANGLES_DEG;

static void wait_for_move (void);
static void drive_straight (void);
static void handle_obstacle (uint16_t range_mm);
static void go_round (car_avoid_action_t action);
static bool look_ahead (uint16_t * p_range_mm);
static void report_angles (void);
static bool run_detour (car_avoid_action_t side);
static void retrace (car_avoid_action_t side, uint8_t driven);
static void drive_leg (uint8_t index, car_avoid_action_t side, bool b_undo);

INT usermain (void)
{
    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    (void)car_time_start_ticks();
    CAR_LOG(CAR_LOG_INFO,
            "detour bench: straight until something is within %u mm\n",
            SCAN_OBSTACLE_RANGE_MM);
    CAR_LOG(CAR_LOG_INFO,
            "scan angles %u, %u, %u deg. no line following, give it room\n",
            g_angles_deg[0], g_angles_deg[1], g_angles_deg[2]);
    CAR_LOG(CAR_LOG_INFO, "battery must be on: motors and servo share the "
                          "motor rail\n");

    car_hw_enable_timer();

    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed, nothing will move\n");
    }

    if (CAR_OK != scan_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "scan_init failed, nothing will be seen\n");
    }

    (void)motion_set_speed(BENCH_SPEED_MM_PER_SEC);

    for (;;)
    {
        uint16_t range_mm = SONAR_MAX_RANGE_MM;

        drive_straight();

        if (look_ahead(&range_mm))
        {
            handle_obstacle(range_mm);
        }
    }
}

/**
 * @brief Stop, profile what is ahead and act on the plan, as the car does.
 *
 * @param[in] range_mm How far ahead the obstacle was seen.
 */
static void handle_obstacle (uint16_t range_mm)
{
    car_obstacle_profile_t profile = { 0 };
    car_avoid_action_t     action  = CAR_AVOID_STOP;

    (void)motion_stop();
    CAR_LOG(CAR_LOG_INFO, "\nobstacle at %u mm, stopping to look\n",
            range_mm);
    (void)tk_dly_tsk(BENCH_SETTLE_MSEC);
    report_angles();

    if (CAR_OK != scan_fine(SCAN_MIN_ANGLE_DEG, SCAN_MAX_ANGLE_DEG,
                            &profile))
    {
        CAR_LOG(CAR_LOG_ERROR, "scan_fine refused the arc\n");
    }
    else
    {
        CAR_LOG(CAR_LOG_INFO,
                "profile: valid %u bearing %d closest %u width %u, "
                "clear left %u right %u\n",
                profile.b_is_valid, profile.bearing_deg,
                profile.closest_range_mm, profile.width_mm,
                profile.clearance_left_mm, profile.clearance_right_mm);
        (void)scan_plan_avoidance(&profile, &action);

        switch (action)
        {
            case CAR_AVOID_LEFT:
            case CAR_AVOID_RIGHT:
            case CAR_AVOID_REVERSE:
                /* Sized from this profile, as the car sizes them. */
                (void)scan_detour_plan(&profile);
                go_round(action);
            break;

            case CAR_AVOID_CONTINUE:
                CAR_LOG(CAR_LOG_INFO,
                        "decision: CONTINUE, the fine scan found nothing\n");
            break;

            case CAR_AVOID_STOP:
            default:
                CAR_LOG(CAR_LOG_INFO, "decision: STOP\n");
                (void)motion_stop();
                (void)tk_dly_tsk(BENCH_SETTLE_MSEC);
            break;
        }
    }

    (void)motion_set_speed(BENCH_SPEED_MM_PER_SEC);
}

/**
 * @brief Go round on the planned side, trying the other side each time a
 *        lane turns out to be blocked.
 *
 * @param[in] action CAR_AVOID_LEFT, CAR_AVOID_RIGHT or CAR_AVOID_REVERSE.
 */
static void go_round (car_avoid_action_t action)
{
    car_avoid_action_t side   = action;
    uint8_t            tries  = 0u;
    bool               b_done = false;

    if (CAR_AVOID_REVERSE == action)
    {
        /* Nothing measured clear, but the sonar only sees the mouth of
         * each lane. Back off and drive into one. */
        CAR_LOG(CAR_LOG_INFO, "decision: neither side clears %u mm, "
                "backing off to probe\n", SCAN_CLEARANCE_MIN_MM);
        (void)motion_move_backward(SCAN_REVERSE_MM);
        wait_for_move();
        side = CAR_AVOID_LEFT;
    }
    else
    {
        CAR_LOG(CAR_LOG_INFO, "decision: go %s (action %d)\n",
                (CAR_AVOID_LEFT == action) ? "LEFT" : "RIGHT", action);
    }

    while ((tries < CAR_MAX_DETOUR_ATTEMPTS) && (!b_done))
    {
        b_done = run_detour(side);

        if (!b_done)
        {
            side = (CAR_AVOID_LEFT == side) ? CAR_AVOID_RIGHT
                                            : CAR_AVOID_LEFT;
            tries++;
            CAR_LOG(CAR_LOG_INFO, "back at the start, trying %s\n",
                    (CAR_AVOID_LEFT == side) ? "left" : "right");
        }
    }

    if (!b_done)
    {
        CAR_LOG(CAR_LOG_ERROR, "no lane free after %u tries\n", tries);
        (void)motion_stop();
        (void)tk_dly_tsk(BENCH_SETTLE_MSEC);
    }
}

/**
 * @brief Run motion_tick() until the current move reports itself finished.
 *
 * NOTE: This bench has no motion task, so the PID and the distance
 * integrator only advance while this is running.
 */
static void wait_for_move (void)
{
    do
    {
        (void)motion_tick();
        car_time_wait_tick();
    }
    while (motion_is_busy());
}

/**
 * @brief Keep the wheels turning straight ahead for one kernel tick.
 */
static void drive_straight (void)
{
    (void)motion_drive_steer(0);
    (void)motion_tick();
    car_time_wait_tick();
}

/**
 * @brief One forward ranging.
 *
 * @param[out] p_range_mm What it saw.
 *
 * @return true if that is inside the trigger distance.
 */
static bool look_ahead (uint16_t * p_range_mm)
{
    bool b_blocked = false;

    if (CAR_OK == scan_measure(SCAN_CENTRE_ANGLE_DEG, p_range_mm))
    {
        b_blocked = (*p_range_mm < SCAN_OBSTACLE_RANGE_MM);
    }

    return b_blocked;
}

/**
 * @brief Print what each of the three angles sees, one line each.
 *
 * NOTE: This is the output to read when a decision looks wrong. The
 * planner only ever sees these three numbers.
 */
static void report_angles (void)
{
    uint8_t index = 0u;

    for (index = 0u; index < SCAN_COARSE_ANGLE_COUNT; index++)
    {
        uint16_t range_mm = SONAR_MAX_RANGE_MM;

        (void)scan_measure(g_angles_deg[index], &range_mm);
        CAR_LOG(CAR_LOG_INFO, "  %3u deg  %5u mm  %s\n",
                g_angles_deg[index], range_mm,
                (range_mm < SCAN_OBSTACLE_RANGE_MM) ? "blocked" : "clear");
    }
}

/**
 * @brief Drive one leg of the car's detour, or back to undo it, and wait.
 *
 * @param[in] index  Leg, 0 to SCAN_DETOUR_LEGS - 1.
 * @param[in] side   CAR_AVOID_LEFT or CAR_AVOID_RIGHT, the way round.
 * @param[in] b_undo true to undo the leg instead of driving it.
 */
static void drive_leg (uint8_t index, car_avoid_action_t side, bool b_undo)
{
    car_avoid_action_t action = CAR_AVOID_STOP;
    uint16_t           amount = 0u;
    char const *       p_move = "back";
    char const *       p_unit = "mm";

    (void)scan_detour_get_leg(index, side, b_undo, &action, &amount);

    if (CAR_AVOID_LEFT == action)
    {
        p_move = "left";
        p_unit = "deg";
        (void)motion_turn_left(amount);
    }
    else if (CAR_AVOID_RIGHT == action)
    {
        p_move = "right";
        p_unit = "deg";
        (void)motion_turn_right(amount);
    }
    else if (CAR_AVOID_CONTINUE == action)
    {
        p_move = "forward";
        (void)motion_move_forward(amount);
    }
    else
    {
        (void)motion_move_backward(amount);
    }

    CAR_LOG(CAR_LOG_INFO, "  %s %u/%u  %s %u %s\n",
            b_undo ? "undo" : "leg", index + 1u, SCAN_DETOUR_LEGS, p_move,
            amount, p_unit);
    wait_for_move();
}

/**
 * @brief Undo the legs already driven, newest first.
 *
 * Puts the car back where the detour began, so the other side is tried
 * from the same place rather than from wherever this attempt ran out.
 *
 * @param[in] side   The side this attempt was going round.
 * @param[in] driven How many legs were driven.
 */
static void retrace (car_avoid_action_t side, uint8_t driven)
{
    uint8_t index = driven;

    CAR_LOG(CAR_LOG_INFO, "  backing out %u leg(s)\n", driven);

    while (0u != index)
    {
        index--;
        drive_leg(index, side, true);
    }
}

/**
 * @brief Drive the box, announcing each leg, pinging before each drive.
 *
 * NOTE: The ping before a driving leg is what makes this a probe rather
 * than a guess. The sonar only ever sees the mouth of a lane from the
 * start position; driving into it and looking again is the only way to
 * find out whether it really goes anywhere.
 *
 * @param[in] side CAR_AVOID_LEFT or CAR_AVOID_RIGHT, the way round.
 *
 * @return true if the whole box was driven, false if a leg was blocked
 *         and the car has been returned to where it started.
 */
static bool run_detour (car_avoid_action_t side)
{
    uint8_t index  = 0u;
    bool    b_done = true;

    for (index = 0u; index < SCAN_DETOUR_LEGS; index++)
    {
        car_avoid_action_t action   = CAR_AVOID_STOP;
        uint16_t           amount   = 0u;
        uint16_t           range_mm = SONAR_MAX_RANGE_MM;

        (void)scan_detour_get_leg(index, side, false, &action, &amount);

        if ((CAR_AVOID_CONTINUE == action) && (look_ahead(&range_mm)))
        {
            CAR_LOG(CAR_LOG_INFO,
                    "  leg %u/%u blocked at %u mm, backing out\n",
                    index + 1u, SCAN_DETOUR_LEGS, range_mm);
            retrace(side, index);
            b_done = false;
            break;
        }

        drive_leg(index, side, false);
    }

    if (b_done)
    {
        CAR_LOG(CAR_LOG_INFO, "detour done, resuming straight\n\n");
    }

    return b_done;
}

/*** end of file ***/
