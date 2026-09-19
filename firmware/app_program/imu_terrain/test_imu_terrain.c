/** @file test_imu_terrain.c
 *
 * @brief Host contract test for the IMU module. No hardware needed.
 *
 * The second half injects a nose up gravity vector and feeds distance, so
 * the pitch maths, the hump detector and the height integral are all
 * exercised with numbers that can be checked by hand: 20 degrees of pitch
 * over 100 mm of travel is about 34 mm of climb.
 *
 * Owner: Buddy 4, IMU based motion and terrain monitoring. Add an assert for
 * every new guarantee and never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"
#include "imu_terrain.h"

#define TEST_SETTLE_UPDATES   40u
#define TEST_CLIMB_STEPS      20u
#define TEST_STEP_MM           5u
#define TEST_PITCH_AX        -342   /* sin 20 degrees times one g, negated */
#define TEST_PITCH_AZ         940   /* cos 20 degrees times one g */

int main (void)
{
    int16_t            pitch_deg   = 0;
    int16_t            heading_deg = 0;
    int16_t            rate_dps    = 0;
    car_motion_event_t event       = CAR_MOTION_STATIONARY;
    car_hump_t         hump        = { 0 };
    uint32_t           index       = 0u;
    uint32_t           distance_mm = 0u;
    uint16_t           milli_g     = 0u;
    uint16_t           filtered_milli_g = 0u;

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
    assert(CAR_OK == imu_feed_odometry(0u, 0, 0));

    /* Level reads zero pitch and no collision. */
    assert(0 == pitch_deg);

    /* Tilt the nose up 20 degrees and let the filter settle. */
    imu_host_inject(TEST_PITCH_AX, 0, TEST_PITCH_AZ, 300, 0, 0);

    for (index = 0u; index < TEST_SETTLE_UPDATES; index++)
    {
        assert(CAR_OK == imu_update());
    }

    assert(CAR_OK == imu_get_orientation(&pitch_deg, &heading_deg));
    assert((pitch_deg >= 19) && (pitch_deg <= 21));
    assert(true == imu_is_hump_detected());
    assert(CAR_OK == imu_get_event(&event));
    assert(CAR_MOTION_CLIMBING == event);

    /* Climb 100 mm at that pitch, then level out to close the hump. */
    for (index = 0u; index < TEST_CLIMB_STEPS; index++)
    {
        distance_mm += TEST_STEP_MM;
        assert(CAR_OK == imu_feed_odometry(distance_mm, 0, 0));
        assert(CAR_OK == imu_update());
    }

    imu_host_inject(0, 0, 1000, 300, 0, 0);

    for (index = 0u; index < TEST_SETTLE_UPDATES; index++)
    {
        assert(CAR_OK == imu_update());
    }

    assert(false == imu_is_hump_detected());
    assert(CAR_OK == imu_get_peak_hump(&hump));
    assert((hump.peak_height_mm >= 28u) && (hump.peak_height_mm <= 40u));
    assert(true == hump.b_is_run_peak);

    /* A wheel speed difference reads as a turn rate, clockwise positive. */
    assert(CAR_OK == imu_feed_odometry(distance_mm, 200, 100));
    assert(CAR_OK == imu_update());
    assert(CAR_OK == imu_get_turn_rate_dps(&rate_dps));
    assert(rate_dps > 0);

    /* A hard knock latches once and clears on read. */
    imu_host_inject(0, 0, 3500, 300, 0, 0);
    assert(CAR_OK == imu_update());
    assert(true == imu_is_collision_detected());
    assert(false == imu_is_collision_detected());

    /* The peak keeps the raw knock the filtered magnitude smoothed away,
     * which is what the collision threshold is tuned against. */
    assert(CAR_OK == imu_get_peak_accel_magnitude(&milli_g));
    assert(milli_g >= 3400u);
    assert(CAR_OK == imu_get_accel_magnitude(&filtered_milli_g));
    assert(filtered_milli_g < milli_g);
    assert(CAR_OK == imu_get_peak_accel_magnitude(&milli_g));
    assert(milli_g <= 1000u);

    return 0;
}

/*** end of file ***/
