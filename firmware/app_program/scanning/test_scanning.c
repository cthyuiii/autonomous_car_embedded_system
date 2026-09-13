/** @file test_scanning.c
 *
 * @brief Host contract test for the scanning module. No hardware needed.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling. Add an assert
 * for every new guarantee and never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"
#include "scanning.h"

int main (void)
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

    return 0;
}

/*** end of file ***/

