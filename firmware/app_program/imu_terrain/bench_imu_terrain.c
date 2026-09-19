/** @file bench_imu_terrain.c
 *
 * @brief Streams pitch, heading, event and the numbers behind them.
 *
 * Build with `make BENCH=imu_terrain` and flash to a board with the
 * LSM303DLHC on Grove 3. Tilt it by hand and watch pitch follow. Rotate it
 * and watch heading wrap at 360.
 *
 * NOTE on the `mag` and `ok` columns, which explain a pitch that wanders.
 * An accelerometer measures the direction of apparent gravity, and it
 * cannot tell a tilt from a push: vibration and acceleration move the
 * vector exactly the way a slope does. `mag` is the length of the filtered
 * vector in milli g and should sit near 1000 when the car is still. `ok` is
 * 1 while `mag` is close enough to one g for the tilt maths to mean
 * anything, and pitch is frozen at its last value whenever `ok` is 0.
 *
 * So a running motor showing `mag` swinging away from 1000 and `ok 0` is
 * the sensor working correctly, not a fault. If `ok` stays 0 with the car
 * standing still, either the mount is picking up vibration or
 * IMU_PITCH_TRUST_BAND_MILLI_G is too tight.
 *
 * Owner: Buddy 4, IMU based motion and terrain monitoring. Extend it as you
 * need; nothing else depends on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "imu_terrain.h"
#include "motion.h"

/* Set to 1 to drive straight while sampling. Hump HEIGHT is the integral
 * of sin(pitch) over ground distance, so it stays zero on a bench that
 * never moves however clearly the pitch reads. Driving is the only way to
 * measure it. WARNING: needs the battery and floor space, and the car
 * drives until you switch it off. */
#define BENCH_DRIVE            0

static char const * event_name (car_motion_event_t event);

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_DRIVE_MM_PER_SEC 250u
#define BENCH_PRINT_EVERY    10u

INT usermain (void)
{
    uint32_t sample_count = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "imu bench: hold it still and level to calibrate\n");

    if (CAR_OK != imu_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "imu_init failed, check Grove 3 wiring\n");
    }
    else if (CAR_OK != imu_calibrate())
    {
        CAR_LOG(CAR_LOG_ERROR, "imu_calibrate failed, was it moving?\n");
    }
    else
    {
        CAR_LOG(CAR_LOG_INFO,
                "ready. mag near %u and ok 1 means pitch can be believed\n",
                1000u);
    }

#if BENCH_DRIVE
    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed, it will not drive\n");
    }

    (void)motion_set_speed(BENCH_DRIVE_MM_PER_SEC);
    (void)motion_drive_steer(0);
    CAR_LOG(CAR_LOG_INFO,
            "driving at %u mm/s. peak is the hump height integral\n",
            BENCH_DRIVE_MM_PER_SEC);
#endif

    for (;;)
    {
        int16_t            pitch_deg   = 0;
        int16_t            heading_deg = 0;
        int16_t            rate_dps    = 0;
        uint16_t           milli_g     = 0u;
        uint16_t           peak_milli_g = 0u;
        car_motion_event_t event       = CAR_MOTION_STATIONARY;
        car_hump_t         hump          = { 0 };
        uint16_t           rough_milli_g = 0u;

#if BENCH_DRIVE
        {
            motion_state_t motion = { 0 };

            (void)motion_tick();

            if (CAR_OK == motion_get_state(&motion))
            {
                /* Hump height is the integral of sin(pitch) over ground
                 * distance, so without this feed it stays zero however
                 * clearly the pitch moves. */
                (void)imu_feed_odometry(motion.distance_mm,
                                        motion.left_mm_per_sec,
                                        motion.right_mm_per_sec);
            }
        }
#endif

        (void)imu_update();

        if (0u == (sample_count % BENCH_PRINT_EVERY))
        {
            (void)imu_get_orientation(&pitch_deg, &heading_deg);
            (void)imu_get_event(&event);
            (void)imu_get_turn_rate_dps(&rate_dps);
            (void)imu_get_accel_magnitude(&milli_g);
            (void)imu_get_peak_accel_magnitude(&peak_milli_g);
            (void)imu_get_peak_hump(&hump);
            (void)imu_get_terrain_roughness(&rough_milli_g);

            /* One line per deliverable, in the order Buddy 4 owns them:
             * calibration, tilt, hump, peak, motion class, collision,
             * turn rate, and the terrain summary. */
            CAR_LOG(CAR_LOG_INFO,
                    "cal %d | pitch %d ok %d mag %u raw %u | "
                    "hump %d peak %u mm | "
                    "event %s | hit %d | rate %d dps | heading %d | "
                    "terrain %s rough %u\n",
                    imu_is_calibrated(),
                    pitch_deg, imu_is_pitch_trusted(), milli_g, peak_milli_g,
                    imu_is_hump_detected(), hump.peak_height_mm,
                    event_name(event),
                    imu_is_collision_detected(),
                    rate_dps, heading_deg,
                    imu_is_terrain_stable() ? "STABLE" : "ROUGH",
                    rough_milli_g);
        }

        sample_count++;
        (void)tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }
}

/**
 * @brief Name for a motion class, so the log reads without the header.
 *
 * @param[in] event What imu_get_event() reported.
 *
 * @return A short constant string.
 */
static char const * event_name (car_motion_event_t event)
{
    char const * p_name = "?";

    switch (event)
    {
        case CAR_MOTION_STATIONARY:   p_name = "STILL";   break;
        case CAR_MOTION_ACCELERATING: p_name = "ACCEL";   break;
        case CAR_MOTION_TURNING:      p_name = "TURN";    break;
        case CAR_MOTION_CLIMBING:     p_name = "CLIMB";   break;
        case CAR_MOTION_DESCENDING:   p_name = "DESCEND"; break;
        case CAR_MOTION_IMPACT:       p_name = "IMPACT";  break;
        default:                                          break;
    }

    return p_name;
}

/*** end of file ***/
