/** @file line_barcode.c
 *
 * @brief IR line position, junctions and Code 39 barcode decoding.
 *
 * NOTE: The barcode decoder keeps a shift register of the most recent bar
 * and space widths seen by the centre sensor. Every time a bar ends it
 * tries to read the last 29 elements as start, character, stop. Only a
 * correctly aligned symbol passes both asterisk checks, so no separate
 * quiet zone detection is needed; a run longer than
 * BARCODE_MAX_ELEMENT_MSEC simply empties the register.
 *
 * Owner: Buddy 3, barcode decoding and IR line following. Implement the TODOs
 * in this file. It is yours.
 */

#include "line_barcode.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#else
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "car_hw.h"
#endif

#include "car_config.h"

/* A sensor on GP0 or GP1 takes the UART0 console pins. That is only safe
 * when the console is USB and UART0 is a mirror nobody reads. */
#if !defined(CAR_HOST_TEST) && !defined(TM_CONSOLE_USB_CDC)
#if (LINE_SENSOR_LEFT_PIN < 2u) || (LINE_SENSOR_CENTRE_PIN < 2u) \
    || (LINE_SENSOR_RIGHT_PIN < 2u)
#error "A line sensor on GP0 or GP1 needs the USB console: CONSOLE=usb_cdc"
#endif
#endif

#define LINE_BIT_LEFT            0x01u
#define LINE_BIT_CENTRE          0x02u
#define LINE_BIT_RIGHT           0x04u
#define LINE_BIT_ALL             0x07u

/* Code 39: nine elements per character, three of them wide, and a symbol
 * of three characters with one gap between neighbours. */
#define CODE39_GROUP             9u
#define CODE39_WIDE_COUNT        3u
#define CODE39_START_STOP        '*'
#define BARCODE_ELEMENTS         29u
#define BARCODE_RUNS_MAX         32u
#define BARCODE_TABLE_SIZE       44u

/** One bar or space as timed by the centre sensor. */
typedef struct
{
    uint32_t width_usec;
    bool     b_dark;
} run_t;

/** One Code 39 character: the wide elements as a 9 bit pattern, MSB first. */
typedef struct
{
    char     symbol;
    uint16_t pattern;
} code39_t;

/* The full Code 39 alphabet. Bit 8 is the first bar, bit 0 the last. */
static code39_t const g_code39[BARCODE_TABLE_SIZE] =
{
    { '0', 0x034u }, { '1', 0x121u }, { '2', 0x061u }, { '3', 0x160u },
    { '4', 0x031u }, { '5', 0x130u }, { '6', 0x070u }, { '7', 0x025u },
    { '8', 0x124u }, { '9', 0x064u }, { 'A', 0x109u }, { 'B', 0x049u },
    { 'C', 0x148u }, { 'D', 0x019u }, { 'E', 0x118u }, { 'F', 0x058u },
    { 'G', 0x00Du }, { 'H', 0x10Cu }, { 'I', 0x04Cu }, { 'J', 0x01Cu },
    { 'K', 0x103u }, { 'L', 0x043u }, { 'M', 0x142u }, { 'N', 0x013u },
    { 'O', 0x112u }, { 'P', 0x052u }, { 'Q', 0x007u }, { 'R', 0x106u },
    { 'S', 0x046u }, { 'T', 0x016u }, { 'U', 0x181u }, { 'V', 0x0C1u },
    { 'W', 0x1C0u }, { 'X', 0x091u }, { 'Y', 0x190u }, { 'Z', 0x0D0u },
    { '-', 0x085u }, { '.', 0x184u }, { ' ', 0x0C4u }, { '$', 0x0A8u },
    { '/', 0x0A2u }, { '+', 0x08Au }, { '%', 0x02Au }, { '*', 0x094u },
};

static uint8_t  g_sensor_mask      = 0u;
static uint8_t  g_raw_levels       = 0u;
static uint8_t  g_seen_dark        = 0u;
static uint8_t  g_seen_light       = 0u;
static uint8_t  g_junction_samples = 0u;
static run_t    g_runs[BARCODE_RUNS_MAX];
static uint8_t  g_run_count        = 0u;
static bool     g_run_started      = false;
static bool     g_run_dark         = false;
static uint32_t g_run_start_usec   = 0u;

static uint8_t           read_sensors (void);
static uint8_t           sample_sensors (void);
static uint32_t          clock_usec (void);
static void              push_run (uint32_t width_usec, bool b_dark);
static char              decode_symbol (run_t const * p_runs);
static char              decode_group (uint32_t const * p_widths);
static car_nav_command_t command_for (char symbol);

car_status_t line_init (void)
{
    // TODO: If LINE_SENSOR_IS_ANALOG, tk_opn_dev() the kernel ADC device
    //       (device/adc). Otherwise gpio_set_pin(GPIO_MODE_IN) all three.
    car_status_t status = CAR_ERR_HARDWARE;

    g_sensor_mask      = 0u;
    g_seen_dark        = 0u;
    g_seen_light       = 0u;
    g_junction_samples = 0u;
    g_run_count        = 0u;
    g_run_started      = false;

#if !LINE_SENSOR_IS_ANALOG
#ifndef CAR_HOST_TEST
    car_hw_enable_timer();
#if LINE_SENSOR_PULL_UP
    car_hw_gpio_input_pullup(LINE_SENSOR_LEFT_PIN);
    car_hw_gpio_input_pullup(LINE_SENSOR_CENTRE_PIN);
    car_hw_gpio_input_pullup(LINE_SENSOR_RIGHT_PIN);
#else
    car_hw_gpio_input_pulldown(LINE_SENSOR_LEFT_PIN);
    car_hw_gpio_input_pulldown(LINE_SENSOR_CENTRE_PIN);
    car_hw_gpio_input_pulldown(LINE_SENSOR_RIGHT_PIN);
#endif
#endif
    status = CAR_OK;
#endif

    return status;
}

car_status_t line_calibrate (void)
{
    // TODO: Sample all three sensors, update stored min and max per sensor,
    //       and set the threshold halfway between them.
    g_sensor_mask = sample_sensors();

    return CAR_OK;
}

car_status_t line_get_raw_levels (uint8_t * p_levels)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_levels)
    {
        *p_levels = g_raw_levels;
        status    = CAR_OK;
    }

    return status;
}

car_status_t line_get_health (uint8_t * p_working_mask)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_working_mask)
    {
        *p_working_mask = (uint8_t)(g_seen_dark & g_seen_light & LINE_BIT_ALL);
        status          = CAR_OK;
    }

    return status;
}

car_status_t line_get_position (int16_t * p_error)
{
    // TODO: Read the sensors into g_sensor_mask, then map the mask to a
    //       signed step, or in analog mode compute a weighted centroid.
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_error)
    {
        uint8_t mask = sample_sensors();

        g_sensor_mask = mask;

        if (LINE_BIT_ALL == mask)
        {
            if (g_junction_samples < LINE_JUNCTION_SAMPLES)
            {
                g_junction_samples++;
            }
        }
        else
        {
            g_junction_samples = 0u;
        }

        status = CAR_OK;

        switch (mask)
        {
            case LINE_BIT_CENTRE:
            case LINE_BIT_ALL:
            case (LINE_BIT_LEFT | LINE_BIT_RIGHT):
                *p_error = 0;
            break;

            case (LINE_BIT_CENTRE | LINE_BIT_RIGHT):
                *p_error = 1;
            break;

            case LINE_BIT_RIGHT:
                *p_error = 2;
            break;

            case (LINE_BIT_CENTRE | LINE_BIT_LEFT):
                *p_error = -1;
            break;

            case LINE_BIT_LEFT:
                *p_error = -2;
            break;

            default:
                status = CAR_ERR_NO_DATA;
            break;
        }
    }

    return status;
}

car_status_t line_get_sensor_mask (uint8_t * p_mask)
{
    // TODO: Return CAR_OK once line_get_position() refreshes the mask.
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_mask)
    {
        *p_mask = g_sensor_mask;
        status  = CAR_OK;
    }

    return status;
}

bool line_is_at_junction (void)
{
    // TODO: True when all three mask bits are set for longer than one
    //       sample, to reject a single noisy reading.
    return (g_junction_samples >= LINE_JUNCTION_SAMPLES);
}

car_status_t barcode_poll (car_nav_command_t * p_command)
{
    // TODO: Time each dark and light run in samples, classify each as
    //       narrow or wide, accumulate into a symbol, and map the
    //       finished symbol to a car_nav_command_t.
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_command)
    {
        uint32_t now    = clock_usec();
        bool     b_dark = (0u != (sample_sensors() & LINE_BIT_CENTRE));

        status = CAR_ERR_NO_DATA;

        if (!g_run_started)
        {
            g_run_started    = true;
            g_run_dark       = b_dark;
            g_run_start_usec = now;
        }
        else if (b_dark != g_run_dark)
        {
            uint32_t width     = now - g_run_start_usec;
            bool     b_was_dark = g_run_dark;

            g_run_dark       = b_dark;
            g_run_start_usec = now;

            if (width > (BARCODE_MAX_ELEMENT_MSEC * 1000u))
            {
                /* Far too long for a bar or a gap: the line itself, or open
                 * floor. Whatever came before it cannot be part of a symbol. */
                g_run_count = 0u;
            }
            else
            {
                push_run(width, b_was_dark);

                if (b_was_dark && (g_run_count >= BARCODE_ELEMENTS))
                {
                    char symbol = decode_symbol(
                        &g_runs[g_run_count - BARCODE_ELEMENTS]);

                    if ('\0' != symbol)
                    {
                        car_nav_command_t command = command_for(symbol);

                        g_run_count = 0u;

                        if (CAR_NAV_NONE != command)
                        {
                            *p_command = command;
                            status     = CAR_OK;
                        }
                    }
                }
            }
        }
        else
        {
            /* Same level as before: the current run just grows. */
        }
    }

    return status;
}

/**
 * @brief Read the three sensors and remember which levels each has shown.
 *
 * @return The 3 bit mask, a set bit meaning dark.
 */
static uint8_t sample_sensors (void)
{
    uint8_t mask = read_sensors();

    g_raw_levels = mask;

#if !LINE_SENSOR_DARK_LEVEL
    /* read_sensors() already reports "dark", so the raw level is its
     * inverse whenever dark means a low pin. */
    g_raw_levels = (uint8_t)(LINE_BIT_ALL & ~(uint32_t)mask);
#endif

    g_seen_dark  |= mask;
    g_seen_light |= (uint8_t)(LINE_BIT_ALL & ~(uint32_t)mask);

    return mask;
}

/**
 * @brief Append one run to the shift register, dropping the oldest if full.
 *
 * @param[in] width_usec Run length.
 * @param[in] b_dark     Level of the run.
 */
static void push_run (uint32_t width_usec, bool b_dark)
{
    if (g_run_count >= BARCODE_RUNS_MAX)
    {
        uint8_t index = 0u;

        for (index = 1u; index < BARCODE_RUNS_MAX; index++)
        {
            g_runs[index - 1u] = g_runs[index];
        }

        g_run_count = BARCODE_RUNS_MAX - 1u;
    }

    g_runs[g_run_count].width_usec = width_usec;
    g_runs[g_run_count].b_dark     = b_dark;
    g_run_count++;
}

/**
 * @brief Decode 29 runs as start, character, stop, in either direction.
 *
 * @param[in] p_runs First of BARCODE_ELEMENTS runs, beginning with a bar.
 *
 * @return The data character, or '\0' if the runs are not a valid symbol.
 */
static char decode_symbol (run_t const * p_runs)
{
    uint32_t widths[BARCODE_ELEMENTS];
    uint8_t  index  = 0u;
    char     symbol = '\0';

    for (index = 0u; index < BARCODE_ELEMENTS; index++)
    {
        widths[index] = p_runs[index].width_usec;
    }

    if (p_runs[0].b_dark)
    {
        uint8_t attempt = 0u;

        for (attempt = 0u; (attempt < 2u) && ('\0' == symbol); attempt++)
        {
            char start = decode_group(&widths[0]);
            char data  = decode_group(&widths[CODE39_GROUP + 1u]);
            char stop  = decode_group(&widths[2u * (CODE39_GROUP + 1u)]);

            if ((CODE39_START_STOP == start) && (CODE39_START_STOP == stop))
            {
                symbol = data;
            }
            else
            {
                /* Reverse in place and try again: the car may have crossed
                 * the symbol from its far end. */
                uint8_t low  = 0u;
                uint8_t high = BARCODE_ELEMENTS - 1u;

                while (low < high)
                {
                    uint32_t swap = widths[low];

                    widths[low]  = widths[high];
                    widths[high] = swap;
                    low++;
                    high--;
                }
            }
        }
    }

    return symbol;
}

/**
 * @brief Classify nine element widths and look the pattern up.
 *
 * The three widest elements are wide, the rest narrow. The narrowest wide
 * element must be at least one and a half times the widest narrow one, or
 * the group is noise rather than a character.
 *
 * @param[in] p_widths Nine consecutive run lengths.
 *
 * @return The character, or '\0' if the widths do not form one.
 */
static char decode_group (uint32_t const * p_widths)
{
    uint32_t sorted[CODE39_GROUP];
    uint16_t pattern  = 0u;
    uint8_t  wide     = 0u;
    uint8_t  index    = 0u;
    char     symbol   = '\0';

    for (index = 0u; index < CODE39_GROUP; index++)
    {
        uint8_t slot = index;

        sorted[index] = p_widths[index];

        while ((slot > 0u) && (sorted[slot - 1u] > sorted[slot]))
        {
            uint32_t swap = sorted[slot - 1u];

            sorted[slot - 1u] = sorted[slot];
            sorted[slot]      = swap;
            slot--;
        }
    }

    /* sorted[6] is the smallest of the three wide elements, sorted[5] the
     * largest narrow one. */
    if ((sorted[CODE39_GROUP - CODE39_WIDE_COUNT] * 2u)
        >= (sorted[CODE39_GROUP - CODE39_WIDE_COUNT - 1u] * 3u))
    {
        for (index = 0u; index < CODE39_GROUP; index++)
        {
            pattern = (uint16_t)(pattern << 1u);

            if (p_widths[index] >= sorted[CODE39_GROUP - CODE39_WIDE_COUNT])
            {
                pattern |= 1u;
                wide++;
            }
        }

        if (CODE39_WIDE_COUNT == wide)
        {
            for (index = 0u; index < BARCODE_TABLE_SIZE; index++)
            {
                if (g_code39[index].pattern == pattern)
                {
                    symbol = g_code39[index].symbol;
                }
            }
        }
    }

    return symbol;
}

/**
 * @brief Map a decoded character to the navigation command it carries.
 *
 * @param[in] symbol Code 39 character.
 *
 * @return The command, or CAR_NAV_NONE for a character the course does not use.
 */
static car_nav_command_t command_for (char symbol)
{
    car_nav_command_t command = CAR_NAV_NONE;

    if (BARCODE_CHAR_LEFT == symbol)
    {
        command = CAR_NAV_LEFT;
    }
    else if (BARCODE_CHAR_RIGHT == symbol)
    {
        command = CAR_NAV_RIGHT;
    }
    else if (BARCODE_CHAR_STRAIGHT == symbol)
    {
        command = CAR_NAV_STRAIGHT;
    }
    else if (BARCODE_CHAR_UTURN == symbol)
    {
        command = CAR_NAV_UTURN;
    }
    else
    {
        /* Not a course symbol. */
    }

    return command;
}

#ifdef CAR_HOST_TEST

/* Host fakes: the test injects what the sensors and the clock would say.
 * The default mask is the centre sensor on the line. */
static uint8_t  g_host_mask = LINE_BIT_CENTRE;
static uint32_t g_host_usec = 0u;

void line_host_inject (uint8_t mask, uint32_t now_usec)
{
    g_host_mask = mask;
    g_host_usec = now_usec;
}

static uint8_t read_sensors (void)
{
    return g_host_mask;
}

static uint32_t clock_usec (void)
{
    return g_host_usec;
}

#else /* CAR_HOST_TEST */

/**
 * @brief Sample the three digital sensors into one mask.
 *
 * @return Bit 0 left, bit 1 centre, bit 2 right; a set bit means dark.
 */
static uint8_t read_sensors (void)
{
    uint8_t mask = 0u;

    if (LINE_SENSOR_DARK_LEVEL == gpio_get_val(LINE_SENSOR_LEFT_PIN))
    {
        mask |= LINE_BIT_LEFT;
    }

    if (LINE_SENSOR_DARK_LEVEL == gpio_get_val(LINE_SENSOR_CENTRE_PIN))
    {
        mask |= LINE_BIT_CENTRE;
    }

    if (LINE_SENSOR_DARK_LEVEL == gpio_get_val(LINE_SENSOR_RIGHT_PIN))
    {
        mask |= LINE_BIT_RIGHT;
    }

    return mask;
}

static uint32_t clock_usec (void)
{
    return car_hw_usec();
}

#endif /* CAR_HOST_TEST */

/*** end of file ***/
