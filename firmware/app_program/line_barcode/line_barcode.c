/** @file line_barcode.c
 *
 * @brief IR line position, junctions and barcode decoding.
 */

#include "line_barcode.h"

#include <stddef.h>

#include "car_config.h"

static uint8_t g_sensor_mask = 0u;

car_status_t line_init (void)
{
    // TODO: If LINE_SENSOR_IS_ANALOG, tk_opn_dev() the kernel ADC device
    //       (device/adc). Otherwise gpio_set_pin(GPIO_MODE_IN) all three.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t line_calibrate (void)
{
    // TODO: Sample all three sensors, update stored min and max per sensor,
    //       and set the threshold halfway between them.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t line_get_position (int16_t * p_error)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_error)
    {
        // TODO: Read the sensors into g_sensor_mask, then map the mask to a
        //       signed step, or in analog mode compute a weighted centroid.
        *p_error = 0;
        status   = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

car_status_t line_get_sensor_mask (uint8_t * p_mask)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_mask)
    {
        *p_mask = g_sensor_mask;
        status  = CAR_ERR_NOT_IMPLEMENTED;
    }

    // TODO: Return CAR_OK once line_get_position() refreshes the mask.
    return status;
}

bool line_is_at_junction (void)
{
    // TODO: True when all three mask bits are set for longer than one
    //       sample, to reject a single noisy reading.
    return false;
}

car_status_t barcode_poll (car_nav_command_t * p_command)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_command)
    {
        // TODO: Time each dark and light run in samples, classify each as
        //       narrow or wide, accumulate into a symbol, and map the
        //       finished symbol to a car_nav_command_t.
        status = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

/*** end of file ***/

