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

int main (void)
{
    test_api();
    test_phase_b();
    test_straight_line();

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
     * going forward; the right encoder produced exactly that on the car. */
    motion_host_inject_pulses(ENCODER_SLOTS_PER_REV, 0u, false, false);
    motion_host_inject_pulses(1u, 0u, true, false);
    assert(CAR_OK == motion_get_state(&state));
    assert(false == state.b_left_backward);
}

/**
 * @brief The wheel that gets ahead on a straight move is slowed and the
 *        other sped up, whichever of the two it is.
 */
static void test_straight_line (void)
{
    uint16_t duty_left  = 0u;
    uint16_t duty_right = 0u;

    /* A tick while stopped takes in any pulses injected before, as the
     * motion task's steady ticking does on the car. */
    assert(CAR_OK == motion_stop());
    assert(CAR_OK == motion_tick());
    assert(CAR_OK == motion_move_forward(1000u));
    motion_host_inject_pulses(40u, 10u, false, false);
    assert(CAR_OK == motion_tick());
    motion_host_get_duty(&duty_left, &duty_right);
    assert(duty_left < duty_right);

    motion_host_inject_pulses(0u, 60u, false, false);
    assert(CAR_OK == motion_tick());
    motion_host_get_duty(&duty_left, &duty_right);
    assert(duty_left > duty_right);
}

/*** end of file ***/

