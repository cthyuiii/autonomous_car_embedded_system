/** @file bench_scanning.c
 *
 * @brief Sweeps the servo end to end and prints the range at each step.
 *
 * Flash this to a Pico with the servo and HC-SR04 attached. Place a box at
 * a known distance and bearing and check the printed range and the angle
 * it appears at. Then reduce SERVO_SETTLE_MSEC until readings smear, and
 * back off. That is the settle time to record.
 */

#include <stdio.h>

#include "pico/stdlib.h"

#include "car_config.h"
#include "scanning.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_SWEEP_GAP_MSEC 1000u

int main (void)
{
    stdio_init_all();
    sleep_ms(BENCH_STARTUP_MSEC);
    printf("scan bench: step %u deg, settle %u ms\n",
           SCAN_FINE_STEP_DEG, SERVO_SETTLE_MSEC);

    if (CAR_OK != scan_init())
    {
        printf("scan_init failed\n");
    }

    for (;;)
    {
        uint16_t angle_deg = 0u;

        for (angle_deg = 0u; angle_deg <= SERVO_TRAVEL_DEG;
             angle_deg += SCAN_FINE_STEP_DEG)
        {
            uint16_t     range_mm = 0u;
            car_status_t status   = scan_measure(angle_deg, &range_mm);

            printf("angle %u range %u status %d\n",
                   angle_deg, range_mm, status);
            sleep_ms(SONAR_MIN_CYCLE_MSEC);
        }

        sleep_ms(BENCH_SWEEP_GAP_MSEC);
    }
}

/*** end of file ***/

