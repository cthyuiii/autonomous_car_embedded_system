/** @file line_barcode.h
 *
 * @brief Three sensor IR line following, junction detection and barcodes.
 *
 * NOTE on the sensors: three MH-Sensor-Series modules, each an IR pair with
 * an LM393 comparator and a trimpot, so the output is a digital level and
 * line position is one of five discrete steps. LINE_SENSOR_DARK_LEVEL in
 * car_config.h names the level the module outputs over the black line.
 * The analog path in earlier drafts is gone: on this port layout only one
 * sensor sits on an ADC capable pin.
 *
 * NOTE on the barcode: the course symbol is Code 39, one character between
 * a start and a stop asterisk, 29 alternating bars and spaces in all. A
 * 141 mm symbol at a 3 mm narrow element is exactly that. The decoder times
 * every bar and space seen by the centre sensor with the microsecond timer
 * and classifies the three widest of each nine element group as wide, so
 * it does not depend on the driving speed. It also tries the sequence
 * reversed, because the car may cross the symbol from either end.
 *
 * NOTE on timing: sampling at LINE_SAMPLE_PERIOD_MSEC must catch every
 * transition, so the narrowest bar has to outlast one sample period. At
 * 3 mm and 10 ms that caps reading speed near 300 mm per second; the
 * controller slows to CAR_BARCODE_SPEED_MM_PER_SEC while a symbol may be
 * underneath.
 *
 * Owner: Buddy 3, barcode decoding and IR line following. A changed signature
 * here also changes the test, the bench and car_main.c, so agree it with the
 * team first.
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
 * NOTE: With digital modules the threshold lives in each module's trimpot,
 * so this only takes a reading and reports success. Set the trimpots on the
 * bench with the module's own indicator LED.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if the two surfaces are not separable.
 */
car_status_t line_calibrate (void);

/**
 * @brief Read the signed offset of the line from the sensor centre.
 *
 * Negative means the line is to the left, positive to the right, zero is
 * centred. The value is one of -2, -1, 0, 1, 2: the outer sensor alone is
 * two steps, the outer sensor together with the centre is one step.
 *
 * @param[out] p_error Offset, must not be NULL. Left untouched when the
 *                     line is lost, so the caller keeps the last good value.
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
 * @brief Copy the raw pin levels, before the dark level mapping.
 *
 * NOTE: This is the reading to watch when a sensor seems dead. The mask
 * from line_get_sensor_mask() has already been through
 * LINE_SENSOR_DARK_LEVEL, so a module wired the opposite way round shows a
 * mask that never means anything, while these levels still change as the
 * sensor passes over a surface. A level that never moves at all is a
 * wiring or power fault; one that moves but the wrong way means the dark
 * level is set backwards.
 *
 * @param[out] p_levels Bit 0 left, bit 1 centre, bit 2 right, as read.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_levels is NULL.
 */
car_status_t line_get_raw_levels (uint8_t * p_levels);

/**
 * @brief Report which sensors have proved themselves alive.
 *
 * A sensor is counted as working once it has been seen both dark and
 * light since line_init(). One that never changes is either unplugged,
 * wired to the wrong pin, or has its trimpot past the end of its range,
 * and no reading from it means anything.
 *
 * NOTE: This is what separates a real junction from a dead loom. With all
 * three sensors unplugged the mask reads 000 with the configured pull, and
 * every health bit stays clear. With LINE_SENSOR_PULL_UP set the wrong way
 * the mask would instead read 111, which is indistinguishable from a
 * junction, so the health bits are the check that matters.
 *
 * @param[out] p_working_mask Bit 0 left, bit 1 centre, bit 2 right. A set
 *                            bit means that sensor has shown both levels.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_working_mask is NULL.
 */
car_status_t line_get_health (uint8_t * p_working_mask);

/**
 * @brief Report whether all three sensors currently see the line.
 *
 * @return true at a junction or a perpendicular bar, once the reading has
 *         held for LINE_JUNCTION_SAMPLES consecutive samples.
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

#ifdef CAR_HOST_TEST
/**
 * @brief Host test hook: the sensor mask and clock the module reads next.
 *
 * @param[in] mask     3 bit mask, bit 0 left, bit 1 centre, bit 2 right.
 * @param[in] now_usec Microsecond clock value to report.
 */
void line_host_inject (uint8_t mask, uint32_t now_usec);
#endif

#endif /* LINE_BARCODE_H */

/*** end of file ***/
