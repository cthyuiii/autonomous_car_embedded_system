/** @file bench_line_barcode.c
 *
 * @brief Streams the sensor mask, the health of each sensor, and the
 *        position error, then prints any barcode it decodes.
 *
 * Build and flash with `./flash.sh line_barcode`, the three IR modules
 * attached. Slide the car sideways across the line by hand: the mask, which
 * prints right, barcode, left, should walk 001, 000, 100 and the error -2,
 * 0, 2. The middle digit is the barcode sensor and never moves on the line.
 *
 * NOTE on health, which is the line to read first. A sensor only counts as
 * working once it has been seen both dark and light, so waving the car over
 * the line proves each one in turn. Until a sensor has shown both levels its
 * health letter stays lower case, and nothing it reports means anything. An
 * unplugged sensor sits at one level forever and never earns its capital.
 *
 * NOTE: mask 1x1 means both line sensors read dark. With working sensors
 * that is a junction. With a disconnected loom it is whatever the pin pull
 * happens to be, which is why the health letters exist: lower case letters
 * and a steady 1x1 is a wiring fault, not a junction.
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
#define BENCH_LEFT_BIT     0x01u
#define BENCH_BARCODE_BIT  0x02u
#define BENCH_RIGHT_BIT    0x04u

static uint8_t  g_last_raw   = 0u;
static uint32_t g_changes[3] = { 0u, 0u, 0u };

static char health_letter (uint8_t health, uint8_t bit, char letter);

INT usermain (void)
{
    uint32_t sample_count = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "line bench: pins L%u B%u R%u, dark is %u\n",
            LINE_SENSOR_LEFT_PIN, LINE_SENSOR_BARCODE_PIN,
            LINE_SENSOR_RIGHT_PIN, LINE_SENSOR_DARK_LEVEL);
    CAR_LOG(CAR_LOG_INFO,
            "wave a hand or the line under each sensor in turn. changes "
            "counts every level flip, right/barcode/left, and the health "
            "letter goes CAPITAL once that sensor has shown both levels\n");

    if (CAR_OK != line_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "line_init failed\n");
    }

    for (;;)
    {
        int16_t           error   = 0;
        uint8_t           mask    = 0u;
        uint8_t           health  = 0u;
        uint8_t           raw     = 0u;
        car_nav_command_t command = CAR_NAV_NONE;

        (void)line_get_position(&error);
        (void)line_get_sensor_mask(&mask);
        (void)line_get_health(&health);
        (void)line_get_raw_levels(&raw);

        /* Count every level change per sensor. A sensor that is wired and
         * pointed at something will tick this up as you move the car over
         * it, whichever way round its output is. One that never ticks has
         * no signal reaching the pin at all. */
        if (raw != g_last_raw)
        {
            uint8_t bit = 0u;

            for (bit = 0u; bit < 3u; bit++)
            {
                if (0u != (((uint8_t)(raw ^ g_last_raw) >> bit) & 1u))
                {
                    g_changes[bit]++;
                }
            }

            g_last_raw = raw;
        }

        if (CAR_OK == barcode_poll(&command))
        {
            CAR_LOG(CAR_LOG_INFO, "barcode command %d\n", command);
        }

        if (0u == (sample_count % BENCH_PRINT_EVERY))
        {
            /* Mask printed right to left so it reads as the car sees the
             * track: right sensor first, then barcode, then left. */
            CAR_LOG(CAR_LOG_INFO,
                    "mask %u%u%u  raw %u%u%u  health %c%c%c  changes "
                    "%u/%u/%u  error %d  junction %d\n",
                    (mask >> 2u) & 1u, (mask >> 1u) & 1u, mask & 1u,
                    (raw >> 2u) & 1u, (raw >> 1u) & 1u, raw & 1u,
                    health_letter(health, BENCH_RIGHT_BIT, 'r'),
                    health_letter(health, BENCH_BARCODE_BIT, 'b'),
                    health_letter(health, BENCH_LEFT_BIT, 'l'),
                    g_changes[2], g_changes[1], g_changes[0],
                    error, line_is_at_junction());
        }

        sample_count++;
        (void)tk_dly_tsk(LINE_SAMPLE_PERIOD_MSEC);
    }
}

/**
 * @brief Upper case the letter once that sensor has proved itself.
 *
 * @param[in] health Working mask from line_get_health().
 * @param[in] bit    Which sensor to test.
 * @param[in] letter Lower case letter for that sensor.
 *
 * @return The letter, upper case if the sensor has shown both levels.
 */
static char health_letter (uint8_t health, uint8_t bit, char letter)
{
    char shown = letter;

    if (0u != (health & bit))
    {
        shown = (char)(letter - ('a' - 'A'));
    }

    return shown;
}

/*** end of file ***/
