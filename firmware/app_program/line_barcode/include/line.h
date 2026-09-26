/** @file line.h
 *
 * @brief Two sensor IR line following, junction detection and barcodes.
 *
 * NOTE on the sensors: three MH-Sensor-Series modules, each an IR pair with
 * an LM393 comparator and a trimpot, so the output is a digital level.
 * Left and right straddle the line and give the position; the third sits
 * off to the left and only reads barcodes. LINE_SENSOR_DARK_LEVEL in
 * car_config.h names the level the module outputs over the black line.
 * The analog path in earlier drafts is gone: on this port layout only one
 * sensor sits on an ADC capable pin.
 *
 * NOTE on the barcode: the course symbol is Code 39, one character between
 * a start and a stop asterisk, 29 alternating bars and spaces in all. A
 * 141 mm symbol at a 3 mm narrow element is exactly that. The decoder times
 * every bar and space seen by the barcode sensor with the microsecond timer
 * and classifies the three widest of each nine element group as wide, so
 * it does not depend on the driving speed. It also tries the sequence
 * reversed, because the car may cross the symbol from either end.
 *
 * NOTE on timing: the line task runs every 10 ms, far too slow for a 3 mm
 * bar at driving speed, so the barcode sensor is not read there. A timer
 * alarm interrupt samples it every BARCODE_SAMPLE_USEC and timestamps each
 * change; line_poll_barcode() only decodes what was recorded. The symbol sits
 * beside the line, on the left, under the barcode sensor alone.
 *
 * Owner: Buddy 3, barcode decoding and IR line following. A changed signature
 * here also changes the test, the bench and car_main.c, so agree it with the
 * team first.
 */

#ifndef LINE_H
#define LINE_H

#include <stdbool.h>
#include <stdint.h>

#include "car.h"

/* Sensor mask bits, as line_get_sensor_mask() reports them. */
#define LINE_BIT_LEFT            0x01u
#define LINE_BIT_BARCODE         0x02u
#define LINE_BIT_RIGHT           0x04u
#define LINE_BIT_ALL             0x07u
#define LINE_BITS_LINE           (LINE_BIT_LEFT | LINE_BIT_RIGHT)

/**
 * @brief Configure the three sensor pins as inputs, digital or ADC.
 *
 * @return CAR_OK once the pins are configured.
 */
car_status_t line_init (void);

/**
 * @brief Check every sensor reads floor, steadily, at the start.
 *
 * Call once at start-up with the car in its starting place: the line
 * between the left and right sensors, the barcode sensor over floor. Takes
 * LINE_CALIBRATE_SAMPLES readings and checks every sensor read floor in
 * all of them. Afterwards line_get_sensor_mask() holds the sensors that
 * read dark in any sample.
 *
 * NOTE: With digital modules the threshold lives in each module's trimpot,
 * so software cannot move it. A sensor dark throughout means the car is
 * misplaced or LINE_SENSOR_DARK_LEVEL is wrong for it; one that flickers
 * has its trimpot on the edge of the floor's reflectance, so the first
 * shadow or lighting change would flip it. Set the trimpot until the
 * module's LED is steady over floor and lights on the line.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if any sensor did not read floor.
 */
car_status_t line_calibrate (void);

/**
 * @brief Read the signed offset of the line from the sensor centre.
 *
 * Negative means the line is to the left, positive to the right, zero is
 * centred. The value is -2 with the left sensor over the line, 2 with the
 * right, and 0 with both (a junction) or neither. Neither counts as
 * centred for LINE_CENTRED_HOLD_MSEC after either sensor last saw the
 * line, and as lost after that, because two sensors cannot tell a line
 * between them from no line at all.
 *
 * @param[out] p_error Offset, must not be NULL. Left untouched when the
 *                     line is lost, so the caller keeps the last good value.
 *
 * @return CAR_OK, CAR_ERR_RANGE if NULL, CAR_ERR_NO_DATA if the line is lost.
 */
car_status_t line_get_position (int16_t * p_error);

/**
 * @brief Copy the raw 3 bit sensor mask, bit 0 left, bit 1 barcode, bit 2
 *        right.
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
 * @param[out] p_levels Bit 0 left, bit 1 barcode, bit 2 right, as read.
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
 * junction, so the health bits are the check that matters. The barcode
 * sensor only earns its bit once it has crossed a bar.
 *
 * @param[out] p_working_mask Bit 0 left, bit 1 barcode, bit 2 right. A set
 *                            bit means that sensor has shown both levels.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_working_mask is NULL.
 */
car_status_t line_get_health (uint8_t * p_working_mask);

/**
 * @brief Report whether both line sensors currently see the line.
 *
 * @return true at a junction or a perpendicular bar, once the reading has
 *         held for LINE_JUNCTION_SAMPLES consecutive samples.
 */
bool line_is_at_junction (void);

/**
 * @brief Decode the bar and space edges recorded since the last call.
 *
 * Call once per LINE_SAMPLE_PERIOD_MSEC. Edges after a decoded symbol wait
 * for the next call.
 *
 * @param[out] p_command Decoded command, written only on CAR_OK.
 *
 * @return CAR_OK once a complete symbol has been decoded, CAR_ERR_NO_DATA
 *         while still accumulating bars, CAR_ERR_RANGE if p_command is NULL.
 */
car_status_t line_poll_barcode (car_nav_command_t * p_command);

#ifdef CAR_HOST_TEST
/**
 * @brief Host test hook: the sensor mask and clock the module reads next.
 *
 * @param[in] mask     3 bit mask, bit 0 left, bit 1 barcode, bit 2 right.
 * @param[in] now_usec Microsecond clock value to report.
 */
void line_host_inject (uint8_t mask, uint32_t now_usec);
#endif

#endif /* LINE_H */

/*** end of file ***/
