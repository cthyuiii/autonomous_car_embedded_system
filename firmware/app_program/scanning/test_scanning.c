/** @file test_scanning.c
 *
 * @brief Host contract test for the scanning module. No hardware needed.
 *
 * The second half injects ranges through the host hook so the profiler and
 * the planner run on real numbers: a wall across the whole arc must plan a
 * reverse, open floor must plan to continue, and the recovery sequencer
 * must hand out turn and drive steps until told the line is back.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling. Add an assert
 * for every new guarantee and never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"
#include "scan.h"

/* The recovery pattern hands out a turn and a drive for every step. */
#define TEST_RECOVER_CALLS  (2u * SCAN_RECOVER_STEPS)

static void test_api (void);
static void test_servo_band (void);
static void test_wall_and_plan (void);
static void test_recovery (void);
static void test_detour (void);

int main (void)
{
    test_api();
    test_servo_band();
    test_wall_and_plan();
    test_recovery();
    test_detour();

    return 0;
}

/**
 * @brief Every call accepts what it should and refuses what it should not.
 */
static void test_api (void)
{
    uint16_t               range_mm = 0u;
    car_obstacle_profile_t profile  = { 0 };
    car_avoid_action_t     action   = CAR_AVOID_STOP;

    assert(CAR_OK == scan_init());
    assert(CAR_OK == scan_measure(90u, &range_mm));
    assert(CAR_ERR_RANGE == scan_measure(SERVO_TRAVEL_DEG + 1u, &range_mm));
    assert(CAR_ERR_RANGE == scan_measure(90u, NULL));
    assert(CAR_OK == scan_coarse(&profile));
    assert(CAR_ERR_RANGE == scan_coarse(NULL));
    assert(CAR_OK == scan_fine(60u, 120u, &profile));
    assert(CAR_ERR_RANGE == scan_fine(120u, 60u, &profile));
    assert(CAR_OK == scan_plan_avoidance(&profile, &action));
    assert(CAR_ERR_RANGE == scan_plan_avoidance(NULL, &action));
    assert(CAR_ERR_NO_DATA == scan_recover_line());
}

/**
 * @brief The horn never leaves the sweep band, whatever is asked of it.
 *
 * This is the guarantee that keeps a congested mount intact, so it is
 * checked at both extremes and at the centre.
 */
static void test_servo_band (void)
{
    uint16_t               range_mm   = 0u;
    uint16_t               angle_deg  = 0u;
    uint16_t               pulse_usec = 0u;
    car_obstacle_profile_t profile    = { 0 };
    car_avoid_action_t     action     = CAR_AVOID_STOP;

    assert(CAR_OK == scan_measure(0u, &range_mm));
    scan_host_get_servo(&angle_deg, &pulse_usec);
    assert(SCAN_MIN_ANGLE_DEG == angle_deg);
    assert(CAR_OK == scan_measure(SERVO_TRAVEL_DEG, &range_mm));
    scan_host_get_servo(&angle_deg, &pulse_usec);
    assert(SCAN_MAX_ANGLE_DEG == angle_deg);

    /* Pulse width follows the mounted centre, not a 0 to 180 fraction, and
     * stays inside what the servo accepts. */
    assert(CAR_OK == scan_measure(SCAN_CENTRE_ANGLE_DEG, &range_mm));
    scan_host_get_servo(&angle_deg, &pulse_usec);
    assert(SERVO_CENTRE_PULSE_USEC == pulse_usec);
    assert(CAR_OK == scan_measure(SCAN_MAX_ANGLE_DEG, &range_mm));
    scan_host_get_servo(&angle_deg, &pulse_usec);
    assert((SERVO_CENTRE_PULSE_USEC
            + (SCAN_HALF_SWEEP_DEG * SERVO_USEC_PER_DEG)) == pulse_usec);
    assert(pulse_usec >= SERVO_PULSE_MIN_USEC);
    assert(pulse_usec <= SERVO_PULSE_MAX_USEC);

    /* Total travel either side of centre stays within the half sweep. */
    assert((SCAN_MAX_ANGLE_DEG - SCAN_MIN_ANGLE_DEG)
           == (2u * SCAN_HALF_SWEEP_DEG));

    /* A fine scan asked for a wide arc is pulled into the band, not run
     * across angles the servo is never allowed to reach. */
    assert(CAR_OK == scan_fine(0u, SERVO_TRAVEL_DEG, &profile));
    scan_host_get_servo(&angle_deg, &pulse_usec);
    assert(angle_deg >= SCAN_MIN_ANGLE_DEG);
    assert(angle_deg <= SCAN_MAX_ANGLE_DEG);

    /* Open floor: nothing valid, the plan is to continue. */
    assert(false == profile.b_is_valid);
    assert(CAR_OK == scan_plan_avoidance(&profile, &action));
    assert(CAR_AVOID_CONTINUE == action);
}

/**
 * @brief A wall across the whole arc is valid with no clearance, so the
 *        plan is to reverse; clearance on a side makes that side the plan.
 */
static void test_wall_and_plan (void)
{
    car_obstacle_profile_t profile = { 0 };
    car_avoid_action_t     action  = CAR_AVOID_STOP;

    scan_host_inject_range(100u);
    assert(CAR_OK == scan_coarse(&profile));
    assert(true == profile.b_is_valid);
    assert(100u == profile.closest_range_mm);
    assert(CAR_OK == scan_fine(60u, 120u, &profile));
    assert(true == profile.b_is_valid);
    /* Bearing cannot resolve finer than one step, and the step does not
     * always divide the band evenly: at SCAN_HALF_SWEEP_DEG 15 the samples
     * run 75, 82, 89, 96, 103, whose midpoint is 89 rather than 90. So a
     * wall centred on the arc reads within one step of straight ahead, not
     * exactly zero. */
    assert(profile.bearing_deg <= (int16_t)SCAN_FINE_STEP_DEG);
    assert(profile.bearing_deg >= -(int16_t)SCAN_FINE_STEP_DEG);
    assert(profile.width_mm > 0u);
    assert(0u == profile.clearance_left_mm);
    assert(0u == profile.clearance_right_mm);
    assert(CAR_OK == scan_plan_avoidance(&profile, &action));
    assert(CAR_AVOID_REVERSE == action);

    /* Clear on both sides with more room on the left: go left. */
    profile.clearance_left_mm  = 1000u;
    profile.clearance_right_mm = 500u;
    assert(CAR_OK == scan_plan_avoidance(&profile, &action));
    assert(CAR_AVOID_LEFT == action);
    profile.clearance_left_mm = 100u;
    assert(CAR_OK == scan_plan_avoidance(&profile, &action));
    assert(CAR_AVOID_RIGHT == action);
}

/**
 * @brief Recovery hands out a turn, then a drive, until the line is seen,
 *        and gives up once the pattern is exhausted.
 */
static void test_recovery (void)
{
    car_avoid_action_t action = CAR_AVOID_STOP;
    uint16_t           amount = 0u;
    uint32_t           index  = 0u;

    assert(CAR_OK == scan_recover_start(false));
    assert(CAR_ERR_NO_DATA == scan_recover_line());
    assert(CAR_OK == scan_recover_get_step(&action, &amount));
    assert(CAR_AVOID_RIGHT == action);
    assert(SCAN_RECOVER_TURN_DEG == amount);
    assert(CAR_ERR_NO_DATA == scan_recover_line());
    assert(CAR_OK == scan_recover_get_step(&action, &amount));
    assert(CAR_AVOID_CONTINUE == action);
    assert(SCAN_RECOVER_DRIVE_MM == amount);
    assert(CAR_ERR_RANGE == scan_recover_get_step(NULL, &amount));
    assert(CAR_OK == scan_recover_report(true));
    assert(CAR_OK == scan_recover_line());

    /* Never seeing the line exhausts the pattern. */
    assert(CAR_OK == scan_recover_start(true));

    for (index = 0u; index < TEST_RECOVER_CALLS; index++)
    {
        assert(CAR_ERR_NO_DATA == scan_recover_line());
    }

    assert(CAR_ERR_TIMEOUT == scan_recover_line());
}

/**
 * @brief The detour legs turn the right way, undo in reverse, and grow
 *        with the obstacle within their limits.
 */
static void test_detour (void)
{
    car_obstacle_profile_t profile = { 0 };
    car_avoid_action_t     action  = CAR_AVOID_STOP;
    uint16_t               amount  = 0u;

    assert(CAR_OK == scan_detour_plan(NULL));
    assert(CAR_ERR_RANGE == scan_detour_get_leg(SCAN_DETOUR_LEGS,
                                                CAR_AVOID_LEFT, false,
                                                &action, &amount));
    assert(CAR_ERR_RANGE == scan_detour_get_leg(0u, CAR_AVOID_LEFT, false,
                                                NULL, &amount));

    /* Round the left: turn left first, and right to undo it. */
    assert(CAR_OK == scan_detour_get_leg(0u, CAR_AVOID_LEFT, false,
                                         &action, &amount));
    assert((CAR_AVOID_LEFT == action) && (SCAN_DETOUR_TURN_DEG == amount));
    assert(CAR_OK == scan_detour_get_leg(0u, CAR_AVOID_LEFT, true,
                                         &action, &amount));
    assert(CAR_AVOID_RIGHT == action);

    /* Turning back toward the obstacle goes the other way. */
    assert(CAR_OK == scan_detour_get_leg(2u, CAR_AVOID_RIGHT, false,
                                         &action, &amount));
    assert(CAR_AVOID_LEFT == action);

    /* Drives go forward, and back to undo. No width: the fixed legs. */
    assert(CAR_OK == scan_detour_get_leg(1u, CAR_AVOID_LEFT, false,
                                         &action, &amount));
    assert((CAR_AVOID_CONTINUE == action) && (SCAN_DETOUR_SIDE_MM == amount));
    assert(CAR_OK == scan_detour_get_leg(3u, CAR_AVOID_LEFT, true,
                                         &action, &amount));
    assert((CAR_AVOID_REVERSE == action) && (SCAN_DETOUR_DEPTH_MM == amount));

    /* 300 mm wide: a longer sideways leg, and a depth past its limit. */
    profile.width_mm = 300u;
    assert(CAR_OK == scan_detour_plan(&profile));
    assert(CAR_OK == scan_detour_get_leg(5u, CAR_AVOID_LEFT, false,
                                         &action, &amount));
    assert((amount > SCAN_DETOUR_SIDE_MM)
           && (amount < SCAN_DETOUR_SIDE_MAX_MM));
    assert(CAR_OK == scan_detour_get_leg(3u, CAR_AVOID_LEFT, false,
                                         &action, &amount));
    assert(SCAN_DETOUR_DEPTH_MAX_MM == amount);
}

/*** end of file ***/
