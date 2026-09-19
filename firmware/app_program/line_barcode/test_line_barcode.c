/** @file test_line_barcode.c
 *
 * @brief Host contract test for line following and barcode decoding.
 *
 * The second half feeds the decoder a synthetic Code 39 symbol through the
 * host hook, at the course's 3 mm narrow element and a 100 mm per second
 * crossing, in both directions of travel. It is the one check that fails
 * if the classifier, the alphabet table or the alignment logic breaks.
 *
 * Owner: Buddy 3, barcode decoding and IR line following. Add an assert for
 * every new guarantee and never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"
#include "line_barcode.h"

#define TEST_NARROW_USEC   30000u   /* 3 mm at 100 mm per second */
#define TEST_WIDE_USEC     90000u
#define TEST_QUIET_USEC   200000u
#define TEST_ELEMENTS         29u
#define TEST_MASK_CENTRE     0x02u

/* Wide element flags for '*', 'L', '*', nine per character, MSB first. */
static uint16_t const g_test_symbol[3] = { 0x094u, 0x043u, 0x094u };

static void              build_symbol (uint32_t * p_widths);
static car_nav_command_t cross_symbol (uint32_t const * p_widths,
                                       uint32_t * p_now_usec);

int main (void)
{
    int16_t           error   = 0;
    uint8_t           mask    = 0u;
    car_nav_command_t command = CAR_NAV_NONE;
    uint8_t           health  = 0u;
    uint32_t          widths[TEST_ELEMENTS];
    uint32_t          now     = 0u;
    uint8_t           low     = 0u;
    uint8_t           high    = TEST_ELEMENTS - 1u;

    assert(CAR_OK == line_init());
    assert(CAR_OK == line_calibrate());
    assert(CAR_OK == line_get_position(&error));
    assert(CAR_ERR_RANGE == line_get_position(NULL));
    assert(CAR_OK == line_get_sensor_mask(&mask));
    assert(CAR_ERR_RANGE == line_get_sensor_mask(NULL));
    assert(false == line_is_at_junction());
    assert(CAR_ERR_NO_DATA == barcode_poll(&command));
    assert(CAR_ERR_RANGE == line_get_health(NULL));
    assert(CAR_ERR_RANGE == barcode_poll(NULL));
    assert(CAR_NAV_NONE == command);

    /* Health starts empty: nothing has shown both levels yet. */
    assert(CAR_OK == line_get_health(&health));
    assert(0x00u == health);

    /* Position steps follow the mask. Bit 0 is left, bit 2 is right. */
    line_host_inject(0x01u, 0u);
    assert(CAR_OK == line_get_position(&error));
    assert(-2 == error);
    line_host_inject(0x04u, 0u);
    assert(CAR_OK == line_get_position(&error));
    assert(2 == error);
    line_host_inject(0x00u, 0u);
    assert(CAR_ERR_NO_DATA == line_get_position(&error));
    assert(2 == error);

    /* Every sensor has now been seen dark and light, so all three are
     * counted as working. A sensor stuck at one level never would be. */
    assert(CAR_OK == line_get_health(&health));
    assert(0x07u == health);

    /* A junction needs the reading to hold. */
    line_host_inject(0x07u, 0u);
    assert(CAR_OK == line_get_position(&error));
    assert(false == line_is_at_junction());
    assert(CAR_OK == line_get_position(&error));
    assert(true == line_is_at_junction());

    /* Forward crossing of *L* decodes to a left turn. */
    build_symbol(widths);
    assert(CAR_NAV_LEFT == cross_symbol(widths, &now));

    /* The same symbol crossed from the other end decodes the same. */
    while (low < high)
    {
        uint32_t swap = widths[low];

        widths[low]  = widths[high];
        widths[high] = swap;
        low++;
        high--;
    }

    assert(CAR_NAV_LEFT == cross_symbol(widths, &now));

    return 0;
}

/**
 * @brief Expand the three character patterns into 29 element widths.
 *
 * @param[out] p_widths TEST_ELEMENTS entries, bars and spaces alternating.
 */
static void build_symbol (uint32_t * p_widths)
{
    uint8_t character = 0u;
    uint8_t index     = 0u;

    for (character = 0u; character < 3u; character++)
    {
        uint8_t element = 0u;

        for (element = 0u; element < 9u; element++)
        {
            bool b_wide = (0u != (g_test_symbol[character]
                                  & (uint16_t)(0x100u >> element)));

            p_widths[index] = b_wide ? TEST_WIDE_USEC : TEST_NARROW_USEC;
            index++;
        }

        if (character < 2u)
        {
            p_widths[index] = TEST_NARROW_USEC;
            index++;
        }
    }
}

/**
 * @brief Drive the decoder through one symbol preceded by a quiet zone.
 *
 * @param[in]     p_widths   TEST_ELEMENTS element widths, first one a bar.
 * @param[in,out] p_now_usec Simulated clock, advanced as the symbol passes.
 *
 * @return The decoded command, or CAR_NAV_NONE if nothing decoded.
 */
static car_nav_command_t cross_symbol (uint32_t const * p_widths,
                                       uint32_t * p_now_usec)
{
    car_nav_command_t command = CAR_NAV_NONE;
    car_nav_command_t decoded = CAR_NAV_NONE;
    uint8_t           index   = 0u;

    line_host_inject(0u, *p_now_usec);
    (void)barcode_poll(&decoded);
    *p_now_usec += TEST_QUIET_USEC;

    for (index = 0u; index < TEST_ELEMENTS; index++)
    {
        uint8_t mask = (0u == (index % 2u)) ? TEST_MASK_CENTRE : 0u;

        line_host_inject(mask, *p_now_usec);
        assert(CAR_ERR_NO_DATA == barcode_poll(&decoded));
        *p_now_usec += p_widths[index];
    }

    line_host_inject(0u, *p_now_usec);

    if (CAR_OK == barcode_poll(&decoded))
    {
        command = decoded;
    }

    *p_now_usec += TEST_QUIET_USEC;

    return command;
}

/*** end of file ***/
