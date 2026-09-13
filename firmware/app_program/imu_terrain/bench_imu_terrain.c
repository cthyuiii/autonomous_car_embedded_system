/** @file bench_imu_terrain.c
 *
 * @brief Streams pitch, heading, event and hump state for calibration.
 *
 * Build with `make BENCH=imu_terrain` and flash to a Pico with the
 * LSM303DLHC on I2C. Tilt the board by hand and watch pitch follow. Rotate
 * it and watch heading wrap at 360. Then mount it on the car, run the
 * motors with the car held still, and watch how far heading moves with no
 * rotation. That number is the magnetometer's motor disturbance.
 *
 * Owner: Buddy 4, IMU based motion and terrain monitoring. Extend it as you
 * need; nothing else depends on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "imu_terrain.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_PRINT_EVERY    10u

INT usermain (void)
{
    uint32_t sample_count = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "imu bench\n");

    if ((CAR_OK != imu_init()) || (CAR_OK != imu_calibrate()))
    {
        CAR_LOG(CAR_LOG_ERROR, "imu init or calibrate failed\n");
    }

    for (;;)
    {
        int16_t            pitch_deg   = 0;
        int16_t            heading_deg = 0;
        car_motion_event_t event       = CAR_MOTION_STATIONARY;

        (void)imu_update();

        if (0u == (sample_count % BENCH_PRINT_EVERY))
        {
            (void)imu_get_orientation(&pitch_deg, &heading_deg);
            (void)imu_get_event(&event);
            CAR_LOG(CAR_LOG_INFO, "pitch %d heading %d event %d hump %d\n",
                    pitch_deg, heading_deg, event, imu_is_hump_detected());
        }

        sample_count++;
        (void)tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }
}

/*** end of file ***/

