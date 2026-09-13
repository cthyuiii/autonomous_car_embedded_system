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

int main (void)
{
    motion_state_t state = { 0 };
    uint32_t       left  = 0u;
    uint32_t       right = 0u;

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
    assert(CAR_OK == motion_get_encoder_counts(&left, &right));
    assert(CAR_ERR_RANGE == motion_get_encoder_counts(NULL, &right));

    return 0;
}

/*** end of file ***/

