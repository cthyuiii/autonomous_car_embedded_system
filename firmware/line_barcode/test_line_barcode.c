/** @file test_line_barcode.c
 *
 * @brief Host contract test for line following and barcode decoding.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "line_barcode.h"

int main (void)
{
    int16_t           error   = 0;
    uint8_t           mask    = 0u;
    car_nav_command_t command = CAR_NAV_NONE;

    assert(CAR_OK == line_init());
    assert(CAR_OK == line_calibrate());
    assert(CAR_OK == line_get_position(&error));
    assert(CAR_ERR_RANGE == line_get_position(NULL));
    assert(CAR_OK == line_get_sensor_mask(&mask));
    assert(CAR_ERR_RANGE == line_get_sensor_mask(NULL));
    assert(false == line_is_at_junction());
    assert(CAR_ERR_NO_DATA == barcode_poll(&command));
    assert(CAR_ERR_RANGE == barcode_poll(NULL));
    assert(CAR_NAV_NONE == command);

    return 0;
}

/*** end of file ***/

