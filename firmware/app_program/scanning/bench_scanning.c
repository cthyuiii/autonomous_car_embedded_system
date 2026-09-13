/** @file bench_scanning.c
 *
 * @brief Sweeps the servo end to end and prints the range at each step.
 *
 * Build with `make BENCH=scanning` and flash to a Pico with the servo and
 * HC-SR04 attached. Place a box at a known distance and bearing and check
 * the printed range and the angle it appears at. Then reduce
 * SERVO_SETTLE_MSEC until readings smear, and back off. That is the settle
 * time to record.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "scanning.h"

#define BENCH_STARTUP_MSEC   2000u
#define BENCH_SWEEP_GAP_MSEC 1000u

INT usermain (void)
{
    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "scan bench: step %u deg, settle %u ms\n",
            SCAN_FINE_STEP_DEG, SERVO_SETTLE_MSEC);

    if (CAR_OK != scan_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "scan_init failed\n");
    }

    for (;;)
    {
        uint16_t angle_deg = 0u;

        for (angle_deg = 0u; angle_deg <= SERVO_TRAVEL_DEG;
             angle_deg += SCAN_FINE_STEP_DEG)
        {
            uint16_t     range_mm = 0u;
            car_status_t status   = scan_measure(angle_deg, &range_mm);

            CAR_LOG(CAR_LOG_INFO, "angle %u range %u status %d\n",
                    angle_deg, range_mm, status);
            (void)tk_dly_tsk(SONAR_MIN_CYCLE_MSEC);
        }

        (void)tk_dly_tsk(BENCH_SWEEP_GAP_MSEC);
    }
}

/*** end of file ***/

