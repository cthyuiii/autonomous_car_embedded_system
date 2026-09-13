/** @file bench_line_barcode.c
 *
 * @brief Streams the sensor mask and position error for calibration.
 *
 * Build with `make BENCH=line_barcode` and flash to a Pico with the three
 * IR sensors attached. Slide the car across the line by hand: the mask
 * should walk 001, 011, 010, 110, 100 and the error should change sign at
 * the centre. Any decoded barcode is printed as it completes.
 *
 * Owner: Buddy 3, barcode decoding and IR line following. Extend it as you
 * need; nothing else depends on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "line_barcode.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_PRINT_EVERY    10u

INT usermain (void)
{
    uint32_t sample_count = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "line bench: analog %u\n", LINE_SENSOR_IS_ANALOG);

    if (CAR_OK != line_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "line_init failed\n");
    }

    for (;;)
    {
        int16_t           error   = 0;
        uint8_t           mask    = 0u;
        car_nav_command_t command = CAR_NAV_NONE;

        (void)line_get_position(&error);
        (void)line_get_sensor_mask(&mask);

        if (CAR_OK == barcode_poll(&command))
        {
            CAR_LOG(CAR_LOG_INFO, "barcode command %d\n", command);
        }

        if (0u == (sample_count % BENCH_PRINT_EVERY))
        {
            CAR_LOG(CAR_LOG_INFO, "mask %u%u%u error %d junction %d\n",
                    (mask >> 2u) & 1u, (mask >> 1u) & 1u, mask & 1u,
                    error, line_is_at_junction());
        }

        sample_count++;
        (void)tk_dly_tsk(LINE_SAMPLE_PERIOD_MSEC);
    }
}

/*** end of file ***/

