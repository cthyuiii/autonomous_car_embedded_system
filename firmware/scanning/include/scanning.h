/** @file scanning.h
 *
 * @brief Servo swept HC-SR04 scanning, obstacle profiling and avoidance.
 *
 * WARNING: The HC-SR04 is a 5 V part and its Echo pin outputs 5 V. Pico
 * GPIO is not 5 V tolerant and this board adds no level shifting, so Echo
 * must go through a divider or shifter before SONAR_ECHO_PIN.
 *
 * WARNING: The servo takes its own 5 V supply with ground tied to the Pico.
 * Powering it from the Pico 3V3 pin browns out the board under stall.
 *
 * NOTE on resolution: the HC-SR04 beam is about 15 degrees wide, so two
 * readings closer together than that are not independent. SCAN_FINE_STEP_DEG
 * defaults to the beam width for that reason.
 *
 * NOTE on time: the datasheet requires 60 ms between rangings. A scan
 * costs at least 60 ms per angle plus SERVO_SETTLE_MSEC per move, all spent
 * stationary. Five coarse angles is at least 300 ms before servo travel.
 *
 * Units: angles in degrees where 90 is straight ahead, distances in mm.
 */

#ifndef SCANNING_H
#define SCANNING_H

#include <stdint.h>

#include "car_types.h"

/**
 * @brief Configure the servo PWM and the sonar trigger and echo pins.
 *
 * @return CAR_OK once the servo is centred at 90 degrees.
 */
car_status_t scan_init (void);

/**
 * @brief Point the servo at one angle, settle, and take one ranging.
 *
 * This is the primitive both scans are built from, and what the bench uses.
 * Blocks for SERVO_SETTLE_MSEC plus up to SONAR_ECHO_TIMEOUT_USEC.
 *
 * @param[in]  angle_deg  0 to SERVO_TRAVEL_DEG.
 * @param[out] p_range_mm Distance, SONAR_MAX_RANGE_MM if nothing echoed.
 *
 * @return CAR_OK, CAR_ERR_RANGE for a bad angle or NULL, CAR_ERR_TIMEOUT if
 *         no echo returned.
 */
car_status_t scan_measure (uint16_t angle_deg, uint16_t * p_range_mm);

/**
 * @brief Sweep the SCAN_COARSE_ANGLES_DEG set and find the nearest return.
 *
 * @param[out] p_profile Filled with b_is_valid false if nothing is within
 *                       SCAN_OBSTACLE_RANGE_MM.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_profile is NULL.
 */
car_status_t scan_coarse (car_obstacle_profile_t * p_profile);

/**
 * @brief Sweep a narrow arc in SCAN_FINE_STEP_DEG steps to profile it.
 *
 * Estimates the obstacle's bearing, closest range, width and the free
 * clearance on each side of it.
 *
 * @param[in]  start_deg Arc start, inclusive.
 * @param[in]  end_deg   Arc end, inclusive, must be above start_deg.
 * @param[out] p_profile Result, must not be NULL.
 *
 * @return CAR_OK, or CAR_ERR_RANGE for a bad arc or NULL.
 */
car_status_t scan_fine (uint16_t start_deg, uint16_t end_deg,
                        car_obstacle_profile_t * p_profile);

/**
 * @brief Choose an avoidance action from a profile.
 *
 * NOTE: Pure decision, no hardware. The side with more clearance wins if
 * it exceeds SCAN_CLEARANCE_MIN_MM, otherwise reverse and rescan.
 *
 * @param[in]  p_profile Profile from scan_fine(), must not be NULL.
 * @param[out] p_action  Chosen action.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if either pointer is NULL.
 */
car_status_t scan_plan_avoidance (car_obstacle_profile_t const * p_profile,
                                  car_avoid_action_t * p_action);

/**
 * @brief Run one step of the search that brings the car back to the line.
 *
 * Called repeatedly by the controller in CAR_STATE_RECOVER_LINE until it
 * returns CAR_OK, at which point line following resumes.
 *
 * @return CAR_OK once the line is reacquired, CAR_ERR_NO_DATA while still
 *         searching, CAR_ERR_TIMEOUT if the search pattern is exhausted.
 */
car_status_t scan_recover_line (void);

#endif /* SCANNING_H */

/*** end of file ***/

