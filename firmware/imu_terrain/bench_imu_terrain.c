/** @file bench_imu_terrain.c
 *
 * @brief Streams pitch, heading, event and hump state for calibration.
 *
 * Flash this to a Pico with the LSM303DLHC on I2C. Tilt the board by hand
 * and watch pitch follow. Rotate it and watch heading wrap at 360. Then
 * mount it on the car, run the motors, and watch how much heading moves
 * with no rotation. That number is the magnetometer's motor disturbance.
 */

#include <stdio.h>

#include "pico/stdlib.h"

#include "car_config.h"
#include "imu_terrain.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_PRINT_EVERY    10u

int main (void)
{
    uint32_t sample_count = 0u;

    stdio_init_all();
    sleep_ms(BENCH_STARTUP_MSEC);
    printf("imu bench\n");

    if ((CAR_OK != imu_init()) || (CAR_OK != imu_calibrate()))
    {
        printf("imu init or calibrate failed\n");
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
            printf("pitch %d heading %d event %d hump %d\n",
                   pitch_deg, heading_deg, event, imu_is_hump_detected());
        }

        sample_count++;
        sleep_ms(IMU_SAMPLE_PERIOD_MSEC);
    }
}

/*** end of file ***/

