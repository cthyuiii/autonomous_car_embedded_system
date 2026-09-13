/** @file line_barcode.h
 *
 * @brief Three sensor IR line following, junction detection and barcodes.
 *
 * NOTE on the sensor choice, set by LINE_SENSOR_IS_ANALOG in car_config.h:
 * The LM393 modules output a digital level thresholded by a trimpot, so
 * three of them give a 3 bit mask and line position is one of a few
 * discrete states. Bare TCRT5000 sensors on the ADC give a proportional
 * error, but the Pico has three ADC pins and GP28 drives the NeoPixel, so
 * only two are free. The digital modules are the path of least resistance.
 * line_get_position() documents which meaning its error carries.
 *
 * NOTE on timing: the track has a 20 mm line, an 18 mm gap between barcode
 * and line, and a barcode about 141 mm long. At speed v mm per second the
 * narrowest bar passes the sensor in width / v seconds, which bounds
 * LINE_SAMPLE_PERIOD_MSEC. Faster driving needs faster sampling.
 */

#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

/**
 * @brief Configure the three sensor pins as inputs, digital or ADC.
 *
 * @return CAR_OK once the pins are configured.
 */
car_status_t line_init (void);

/**
 * @brief Record the sensor reading on both surfaces.
 *
 * Call with the car held over the black line, then again over the light
 * track. Each call samples all three sensors and stores min and max, so the
 * threshold sits between them regardless of ambient light.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if the two surfaces are not separable.
 */
car_status_t line_calibrate (void);

/**
 * @brief Read the signed offset of the line from the sensor centre.
 *
 * Negative means the line is to the left, positive to the right, zero is
 * centred. In digital mode the value is one of a small set of steps. In
 * analog mode it is proportional to the offset in ADC counts.
 *
 * @param[out] p_error Offset, must not be NULL.
 *
 * @return CAR_OK, CAR_ERR_RANGE if NULL, CAR_ERR_NO_DATA if the line is lost.
 */
car_status_t line_get_position (int16_t * p_error);

/**
 * @brief Copy the raw 3 bit sensor mask, bit 0 left, bit 2 right.
 *
 * @param[out] p_mask Destination, must not be NULL.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_mask is NULL.
 */
car_status_t line_get_sensor_mask (uint8_t * p_mask);

/**
 * @brief Report whether all three sensors currently see the line.
 *
 * @return true at a junction or a perpendicular bar.
 */
bool line_is_at_junction (void);

/**
 * @brief Advance the barcode decoder by one sample.
 *
 * Call once per LINE_SAMPLE_PERIOD_MSEC while following the line.
 *
 * @param[out] p_command Decoded command, written only on CAR_OK.
 *
 * @return CAR_OK once a complete symbol has been decoded, CAR_ERR_NO_DATA
 *         while still accumulating bars, CAR_ERR_RANGE if p_command is NULL.
 */
car_status_t barcode_poll (car_nav_command_t * p_command);

#endif /* LINE_BARCODE_H */

/*** end of file ***/

