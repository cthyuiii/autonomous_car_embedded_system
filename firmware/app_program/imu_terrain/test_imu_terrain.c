/** @file test_imu_terrain.c
 *
 * @brief Host contract test for the IMU module. No hardware needed.
 *
 * Owner: Buddy 4, IMU based motion and terrain monitoring. Add an assert for
 * every new guarantee and never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "imu_terrain.h"

int main (void)
{
    int16_t            pitch_deg   = 0;
    int16_t            heading_deg = 0;
    int16_t            rate_dps    = 0;
    car_motion_event_t event       = CAR_MOTION_STATIONARY;
    car_hump_t         hump        = { 0 };

    assert(CAR_OK == imu_init());
    assert(CAR_OK == imu_calibrate());
    assert(CAR_OK == imu_update());
    assert(CAR_OK == imu_get_orientation(&pitch_deg, &heading_deg));
    assert(CAR_ERR_RANGE == imu_get_orientation(NULL, &heading_deg));
    assert(CAR_OK == imu_get_event(&event));
    assert(CAR_MOTION_STATIONARY == event);
    assert(CAR_OK == imu_get_turn_rate_dps(&rate_dps));
    assert(false == imu_is_hump_detected());
    assert(CAR_OK == imu_get_peak_hump(&hump));
    assert(0u == hump.peak_height_mm);
    assert(false == imu_is_collision_detected());

    return 0;
}

/*** end of file ***/

