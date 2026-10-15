/** @file test_motion.c
 *
 * @brief Host contract test for the motion module. No hardware needed.
 *
 * Every assert states what the implementation must guarantee. They fail
 * today because the stubs return CAR_ERR_NOT_IMPLEMENTED.
 *
 * Owner: Buddy 2, motion control. Add an assert for every new guarantee and
 * never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"
#include "motion.h"

static void test_api (void);
static void test_phase_b (void);
static void test_straight_line (void);
static void test_wheel_shares (void);
static void test_ramp (void);
static void test_steer_from_rest (void);
static void settle (void);

/* One tick's rise of a wheel setpoint, and a speed it reaches in two. */
#define TEST_RAMP_STEP   ((MOTION_ACCEL_MM_PER_SEC2 * MOTION_TICK_PERIOD_MSEC) \
                          / 1000u)
/* Straight moves are tested slowly, so the climb from rest settles well
 * inside MOTION_STALL_FAULT_MSEC of a quiet wheel. Enough ticks to reach
 * the fastest target: the speed plus half of it from the correction. */
#define TEST_SPEED_MM_PER_SEC   60u
#define TEST_SETTLE_TICKS \
    ((((TEST_SPEED_MM_PER_SEC * 3u) / 2u) / TEST_RAMP_STEP) + 2u)

int main (void)
{
    test_api();
    test_phase_b();
    test_straight_line();
    test_wheel_shares();
    test_ramp();
    test_steer_from_rest();

    return 0;
}

/**
 * @brief Every call accepts what it should and refuses what it should not.
 */
static void test_api (void)
{
    motion_state_t state = { 0 };

    assert(CAR_OK == motion_init());
    assert(false == motion_is_busy());
    assert(CAR_OK == motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC));
    assert(CAR_ERR_RANGE == motion_set_speed(MOTION_MAX_SPEED_MM_PER_SEC + 1u));
    assert(CAR_OK == motion_move_forward(100u));
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_stop());
    assert(false == motion_is_busy());
    assert(CAR_OK == motion_move_backward(100u));
    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_turn_left(90u));
    assert(CAR_ERR_RANGE == motion_turn_left(361u));
    assert(CAR_OK == motion_turn_right(90u));
    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_get_state(&state));
    assert(CAR_ERR_RANGE == motion_get_state(NULL));
}

/**
 * @brief Phase B decides which way each wheel really turned.
 */
static void test_phase_b (void)
{
    motion_state_t state = { 0 };

    /* A wheel rolling back down a hump while told to go forward reads as a
     * negative speed and takes progress away, so the move cannot finish
     * on it. */
    assert(CAR_OK == motion_move_forward(100u));
    motion_host_inject_pulses(2u * ENCODER_SLOTS_PER_REV, 0u, true, false);
    assert(CAR_OK == motion_tick());
    assert(true == motion_is_busy());
    assert(CAR_OK == motion_get_state(&state));
    assert(state.left_mm_per_sec < 0);
    assert(0u == state.distance_mm);

    /* In a left turn the left wheel is meant to go backward, so its
     * backward pulses count toward the turn, which then completes. */
    assert(CAR_OK == motion_turn_left(90u));
    motion_host_inject_pulses(2u * ENCODER_SLOTS_PER_REV,
                              2u * ENCODER_SLOTS_PER_REV, true, false);
    assert(CAR_OK == motion_tick());
    assert(false == motion_is_busy());

    /* One stray phase B reading does not reverse a wheel that is plainly
     * going forward. The right encoder produced exactly that on the car. */
    motion_host_inject_pulses(ENCODER_SLOTS_PER_REV, 0u, false, false);
    motion_host_inject_pulses(1u, 0u, true, false);
    assert(CAR_OK == motion_get_state(&state));
    assert(false == state.b_left_backward);
}

/**
 * @brief The wheel that gets ahead on a straight move is slowed and the
 *        other sped up, whichever of the two it is. Read from the
 *        setpoints once the climb from rest has settled.
 */
static void test_straight_line (void)
{
    uint16_t left  = 0u;
    uint16_t right = 0u;

    /* A tick while stopped takes in any pulses injected before, as the
     * motion task's steady ticking does on the car. */
    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_set_speed(TEST_SPEED_MM_PER_SEC));
    assert(CAR_OK == motion_move_forward(1000u));
    motion_host_inject_pulses(40u, 10u, false, false);
    settle();
    motion_host_get_setpoint(&left, &right);
    assert(left < right);

    motion_host_inject_pulses(0u, 60u, false, false);
    settle();
    motion_host_get_setpoint(&left, &right);
    assert(left > right);
    assert(CAR_OK == motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC));
}

/**
 * @brief Equal pulse counts are not equal ground when the tyres differ:
 *        the wheel with the larger WHEEL_*_PERMILLE share covers more per
 *        pulse, reads faster at the same pulse rate, and counts as ahead,
 *        so the other wheel is asked for more.
 */
static void test_wheel_shares (void)
{
    uint16_t       left  = 0u;
    uint16_t       right = 0u;
    motion_state_t state = { 0 };

    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_set_speed(TEST_SPEED_MM_PER_SEC));
    assert(CAR_OK == motion_move_forward(1000u));
    motion_host_inject_pulses(200u, 200u, false, false);
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_get_state(&state));
    settle();
    motion_host_get_setpoint(&left, &right);

#if WHEEL_LEFT_PERMILLE < WHEEL_RIGHT_PERMILLE
    assert(state.left_mm_per_sec < state.right_mm_per_sec);
    assert(left > right);
#elif WHEEL_LEFT_PERMILLE > WHEEL_RIGHT_PERMILLE
    assert(state.left_mm_per_sec > state.right_mm_per_sec);
    assert(left < right);
#else
    assert(state.left_mm_per_sec == state.right_mm_per_sec);
#endif

    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC));
}

/**
 * @brief Tick until a setpoint climbing from rest has reached any target a
 *        straight move can ask for.
 */
static void settle (void)
{
    uint8_t tick = 0u;

    for (tick = 0u; tick < TEST_SETTLE_TICKS; tick++)
    {
        (void)motion_tick();
    }
}

/**
 * @brief Every speed-up is gradual and every slow-down immediate: from
 *        rest the setpoints climb one TEST_RAMP_STEP a tick, and a stop
 *        drops them to zero at once.
 */
static void test_ramp (void)
{
    uint16_t left  = 0u;
    uint16_t right = 0u;

    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC));
    assert(CAR_OK == motion_move_forward(1000u));
    motion_host_get_setpoint(&left, &right);
    assert((0u == left) && (0u == right));

    assert(CAR_OK == motion_tick());
    motion_host_get_setpoint(&left, &right);
    assert((TEST_RAMP_STEP == left) && (TEST_RAMP_STEP == right));

    assert(CAR_OK == motion_tick());
    motion_host_get_setpoint(&left, &right);
    assert((2u * TEST_RAMP_STEP) == left);

    assert(CAR_OK == motion_stop());
    motion_host_get_setpoint(&left, &right);
    assert((0u == left) && (0u == right));
}

/**
 * @brief A steer from rest is felt from the first tick: the outer wheel
 *        climbs and the inner keeps its share, rather than both climbing
 *        together while the car goes straight.
 */
static void test_steer_from_rest (void)
{
    uint16_t left  = 0u;
    uint16_t right = 0u;

    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_drive_steer(300));
    assert(CAR_OK == motion_tick());
    motion_host_get_setpoint(&left, &right);
    assert(left > right);
    assert(CAR_OK == motion_stop());
}

/*** end of file ***/

