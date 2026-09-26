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
#include "imu.h"

#define TEST_SETTLE_UPDATES  160u   /* Filter settles, alpha 1/32 */
#define TEST_CLIMB_STEPS      20u
#define TEST_STEP_MM           5u
#define TEST_PITCH_AX        -342   /* sin 20 degrees times one g, negated */
#define TEST_PITCH_AZ         940   /* cos 20 degrees times one g */
#define TEST_GRAVITY_UPDATES  200u   /* For the filter to settle fully */
#define TEST_SMALL_AX        -174   /* sin 10 degrees times one g, negated */
#define TEST_SMALL_AZ         985   /* cos 10 degrees times one g */
#define TEST_SHAKE_NUM          3   /* Shaking reads 3/2 of one g */
#define TEST_SHAKE_DEN          2
#define TEST_HUMP_SHAKE_NUM     7   /* 7/5 of one g: past the flat band, */
#define TEST_HUMP_SHAKE_DEN     5   /* inside the hump band */

static void     test_own_gravity (void);
static void     test_knock (void);
static uint32_t test_second_hump (uint32_t distance_mm);
static uint32_t test_bridged_hump (uint32_t distance_mm);
static uint32_t test_curving_hump (uint32_t distance_mm);
static void     inject_forward (int32_t forward_mg, int32_t up_mg);
static void     settle (void);
static uint32_t climb (uint32_t distance_mm);

int main (void)
{
    int16_t            pitch_deg   = 0;
    int16_t            heading_deg = 0;
    int16_t            rate_dps    = 0;
    car_motion_event_t event       = CAR_MOTION_STATIONARY;
    uint16_t           hump        = 0u;
    uint32_t           index       = 0u;
    uint32_t           distance_mm = 0u;
    uint16_t           count       = 0u;
    uint16_t           last        = 0u;

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
    assert(0u == hump);
    assert(false == imu_is_collision_detected());
    assert(CAR_OK == imu_feed_odometry(0u, 0, 0));

    /* Level reads zero pitch and no collision. */
    assert(0 == pitch_deg);

    /* Tilt the nose up 20 degrees and let the filter settle. The tilt goes
     * on whichever axis the car's config says points forward. */
    imu_host_inject((0u == IMU_FORWARD_AXIS)
                        ? (int16_t)(TEST_PITCH_AX * IMU_PITCH_SIGN) : 0,
                    (1u == IMU_FORWARD_AXIS)
                        ? (int16_t)(TEST_PITCH_AX * IMU_PITCH_SIGN) : 0,
                    TEST_PITCH_AZ, 300, 0, 0);

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
    assert((hump >= 28u) && (hump <= 40u));
    assert(CAR_OK == imu_get_hump_count(&count));
    assert(1u == count);
    assert(CAR_OK == imu_get_last_hump(&last));
    assert(last == hump);

    /* A wheel speed difference reads as a turn rate, clockwise positive. */
    assert(CAR_OK == imu_feed_odometry(distance_mm, 200, 100));
    assert(CAR_OK == imu_update());
    assert(CAR_OK == imu_get_turn_rate_dps(&rate_dps));
    assert(rate_dps > 0);

    test_knock();
    distance_mm = test_second_hump(distance_mm);
    distance_mm = test_bridged_hump(distance_mm);
    (void)test_curving_hump(distance_mm);
    test_own_gravity();

    return 0;
}

/**
 * @brief A hard knock latches once and clears on read, and the peak keeps
 *        the raw knock the filtered magnitude smoothed away, which is what
 *        the collision threshold is tuned against.
 */
static void test_knock (void)
{
    uint16_t milli_g          = 0u;
    uint16_t filtered_milli_g = 0u;

    imu_host_inject(0, 0, 3500, 300, 0, 0);
    assert(CAR_OK == imu_update());
    assert(true == imu_is_collision_detected());
    assert(false == imu_is_collision_detected());

    assert(CAR_OK == imu_get_peak_accel_magnitude(&milli_g));
    assert(milli_g >= 3400u);
    assert(CAR_OK == imu_get_accel_magnitude(&filtered_milli_g));
    assert(filtered_milli_g < milli_g);
    assert(CAR_OK == imu_get_peak_accel_magnitude(&milli_g));
    assert(milli_g <= 1000u);
}

/**
 * @brief A lower hump is counted and kept as the last one, and leaves the
 *        run's peak alone.
 *
 * @param[in] distance_mm Odometry so far.
 *
 * @return Odometry afterwards.
 */
static uint32_t test_second_hump (uint32_t distance_mm)
{
    uint16_t   peak  = 0u;
    uint16_t   last  = 0u;
    uint16_t   count = 0u;

    inject_forward(0, 1000);
    settle();
    assert(CAR_OK == imu_get_peak_hump(&peak));

    /* 100 mm at 10 degrees rises about 17 mm. */
    inject_forward(TEST_SMALL_AX, TEST_SMALL_AZ);
    settle();
    distance_mm = climb(distance_mm);
    inject_forward(0, 1000);
    settle();

    assert(CAR_OK == imu_get_hump_count(&count));
    assert(2u == count);
    assert(CAR_OK == imu_get_last_hump(&last));
    assert((last >= 14u) && (last <= 21u));
    assert(last < peak);
    assert(CAR_OK == imu_get_peak_hump(&last));
    assert(peak == last);

    return distance_mm;
}

/**
 * @brief Pitch frozen at the foot of a hump still counts the climb made
 *        while it was frozen.
 *
 * The car starts shaking on the flat, so pitch freezes at 0, then climbs
 * 100 mm at 20 degrees while still shaking, then steadies and climbs 100 mm
 * more. The frozen stretch is bridged at the mean of 0 and 20 degrees,
 * about 17 mm, on top of the 34 mm of the steady climb. Counting at the
 * frozen pitch would give 34.
 *
 * @param[in] distance_mm Odometry so far.
 *
 * @return Odometry afterwards.
 */
static uint32_t test_bridged_hump (uint32_t distance_mm)
{
    uint16_t   peak  = 0u;
    uint16_t   last  = 0u;
    uint16_t   count = 0u;
    uint32_t   index = 0u;

    inject_forward(0, (1000 * TEST_SHAKE_NUM) / TEST_SHAKE_DEN);

    for (index = 0u; imu_is_pitch_trusted(); index++)
    {
        assert(index < TEST_SETTLE_UPDATES);
        assert(CAR_OK == imu_update());
    }

    inject_forward((TEST_PITCH_AX * TEST_SHAKE_NUM) / TEST_SHAKE_DEN,
                   (TEST_PITCH_AZ * TEST_SHAKE_NUM) / TEST_SHAKE_DEN);
    distance_mm = climb(distance_mm);
    settle();
    assert(false == imu_is_pitch_trusted());

    inject_forward(TEST_PITCH_AX, TEST_PITCH_AZ);
    settle();
    assert(true == imu_is_pitch_trusted());
    distance_mm = climb(distance_mm);
    inject_forward(0, 1000);
    settle();

    assert(CAR_OK == imu_get_hump_count(&count));
    assert(3u == count);
    assert(CAR_OK == imu_get_last_hump(&last));
    assert((last >= 45u) && (last <= 58u));
    assert(CAR_OK == imu_get_peak_hump(&peak));
    assert(peak == last);

    return distance_mm;
}

/**
 * @brief On a hump, pitch stays trusted through shaking that would freeze
 *        it on the flat, so a slope that eases off is followed.
 *
 * 100 mm at 20 degrees, then 100 mm at 10 degrees while shaking at 1.4 g,
 * is about 34 + 17 = 51 mm. A pitch frozen near 20 degrees for the second
 * stretch would count about 68.
 *
 * @param[in] distance_mm Odometry so far.
 *
 * @return Odometry afterwards.
 */
static uint32_t test_curving_hump (uint32_t distance_mm)
{
    uint16_t   last  = 0u;
    uint16_t   count = 0u;

    inject_forward(0, 1000);
    settle();
    inject_forward(TEST_PITCH_AX, TEST_PITCH_AZ);
    settle();
    distance_mm = climb(distance_mm);

    inject_forward((TEST_SMALL_AX * TEST_HUMP_SHAKE_NUM) / TEST_HUMP_SHAKE_DEN,
                   (TEST_SMALL_AZ * TEST_HUMP_SHAKE_NUM) / TEST_HUMP_SHAKE_DEN);
    settle();
    assert(true == imu_is_pitch_trusted());
    distance_mm = climb(distance_mm);
    inject_forward(0, 1000);
    settle();

    assert(CAR_OK == imu_get_hump_count(&count));
    assert(4u == count);
    assert(CAR_OK == imu_get_last_hump(&last));
    assert((last >= 45u) && (last <= 57u));

    /* The same shaking on the flat freezes pitch. */
    inject_forward(0, (1000 * TEST_HUMP_SHAKE_NUM) / TEST_HUMP_SHAKE_DEN);
    settle();
    assert(false == imu_is_pitch_trusted());

    return distance_mm;
}

/**
 * @brief Inject an acceleration along the configured forward axis.
 *
 * @param[in] forward_mg Forward axis, negative for nose up with sign +1.
 * @param[in] up_mg      Vertical axis.
 */
static void inject_forward (int32_t forward_mg, int32_t up_mg)
{
    /* Casts: the test injects at most one and a half g, well inside
     * int16_t. */
    int16_t forward = (int16_t)(forward_mg * IMU_PITCH_SIGN);
    int16_t upward  = (int16_t)up_mg;

    imu_host_inject((0u == IMU_FORWARD_AXIS) ? forward : 0,
                    (1u == IMU_FORWARD_AXIS) ? forward : 0,
                    upward, 300, 0, 0);
}

/**
 * @brief Let the filters settle with the car standing still.
 */
static void settle (void)
{
    uint32_t index = 0u;

    for (index = 0u; index < TEST_SETTLE_UPDATES; index++)
    {
        assert(CAR_OK == imu_update());
    }
}

/**
 * @brief Drive TEST_CLIMB_STEPS of TEST_STEP_MM at whatever is injected.
 *
 * @param[in] distance_mm Odometry so far.
 *
 * @return Odometry afterwards.
 */
static uint32_t climb (uint32_t distance_mm)
{
    uint32_t index = 0u;

    for (index = 0u; index < TEST_CLIMB_STEPS; index++)
    {
        distance_mm += TEST_STEP_MM;
        assert(CAR_OK == imu_feed_odometry(distance_mm, 0, 0));
        assert(CAR_OK == imu_update());
    }

    return distance_mm;
}

/**
 * @brief A module that reads one g as 800 mg, level and still, is
 *        calibrated to its own one g, so the ground reads smooth and pitch
 *        is trusted.
 */
static void test_own_gravity (void)
{
    uint8_t index = 0u;

    imu_host_inject(0, 0, 800, 300, 0, 0);
    assert(CAR_OK == imu_calibrate());

    for (index = 0u; index < TEST_GRAVITY_UPDATES; index++)
    {
        assert(CAR_OK == imu_update());
    }

    assert(true == imu_is_terrain_stable());
    assert(true == imu_is_pitch_trusted());
}

/*** end of file ***/
