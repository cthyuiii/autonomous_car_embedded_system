/** @file bench_line_barcode.c
 *
 * @brief Streams the sensor mask and position error for calibration.
 *
 * Flash this to a Pico with the three IR sensors attached and slide the
 * car across the line by hand. The mask should walk 001, 011, 010, 110,
 * 100 and the error should change sign at the centre. Any decoded barcode
 * is printed as it completes.
 */

#include <stdio.h>

#include "pico/stdlib.h"

#include "car_config.h"
#include "line_barcode.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_PRINT_EVERY   20u

int main (void)
{
    uint32_t sample_count = 0u;

    stdio_init_all();
    sleep_ms(BENCH_STARTUP_MSEC);
    printf("line bench: analog %u\n", LINE_SENSOR_IS_ANALOG);

    if (CAR_OK != line_init())
    {
        printf("line_init failed\n");
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
            printf("barcode command %d\n", command);
        }

        if (0u == (sample_count % BENCH_PRINT_EVERY))
        {
            printf("mask %u%u%u error %d junction %d\n",
                   (mask >> 2u) & 1u, (mask >> 1u) & 1u, mask & 1u,
                   error, line_is_at_junction());
        }

        sample_count++;
        sleep_ms(LINE_SAMPLE_PERIOD_MSEC);
    }
}

/*** end of file ***/

