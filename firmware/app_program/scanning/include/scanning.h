/** @file scanning.h
 *
 * @brief Servo swept HC-SR04 scanning, obstacle profiling and avoidance.
 *
 * NOTE on power: the HC-SR04 is a 5 V part, but on this car it runs from
 * the Grove port's 3.3 V on purpose. Its echo output then swings to 3.3 V,
 * which the Pico pin accepts directly, so no divider is fitted. The cost is
 * maximum range, which the bench measures. Never move its supply to 5 V
 * without adding the divider back.
 *
 * WARNING: The servo sits on header S1, which the board feeds from its
 * motor rail. On USB power alone that rail sags under stall and resets the
 * Pico, so run the servo and motors from the battery.
 *
 * NOTE on resolution: the HC-SR04 beam is about 15 degrees wide, so two
 * readings closer together than that are not independent. The sweep this
 * car allows, SCAN_HALF_SWEEP_DEG either side of centre, is narrower than
 * one beam, so every angle in it sees very nearly the same thing. Bearing
 * and width are indicative and the clearance beside an obstacle cannot be
 * measured at all. See the README before widening or relying on them.
 *
 * NOTE on time: the datasheet requires 60 ms between rangings. A scan
 * costs at least 60 ms per angle plus SERVO_SETTLE_MSEC per move, all spent
 * stationary. A ranging at the angle the servo already holds skips the
 * settle, which is what makes the forward ping while following the line
 * affordable.
 *
 * WARNING on the first movement: a servo has no position feedback, so the
 * very first pulse throws the horn from wherever it was left to whatever
 * SERVO_CENTRE_PULSE_USEC says. Every angle here is measured from that
 * centre, so nothing afterwards moves it further than the sweep, but the
 * initial jump is only small once that constant matches the mount. Measure
 * it with BENCH=servo.
 *
 * NOTE on recovery: this module owns the search pattern that brings the
 * car back to the line, but it drives nothing and reads no sensor itself.
 * The controller asks for each step, executes it through the motion API,
 * reports what the line sensors saw, and asks again. That keeps the module
 * free of any other module's header, as the layering requires.
 *
 * Units: angles in degrees where 90 is straight ahead and larger angles
 * look to the car's left, distances in mm. Bearings in a profile are
 * relative to straight ahead, positive to the left.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling. A changed
 * signature here also changes the test, the bench and car_main.c, so agree it
 * with the team first.
 */

#ifndef SCANNING_H
#define SCANNING_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

/**
 * @brief Configure the servo PWM and the sonar trigger and echo pins.
 *
 * WARNING: This sends the servo's first pulse, which throws the horn to
 * SERVO_CENTRE_PULSE_USEC from wherever it was resting. Keep fingers and
 * the sonar's cable clear of the horn's path the first time after power on.
 *
 * @return CAR_OK once the servo is centred at SCAN_CENTRE_ANGLE_DEG.
 */
car_status_t scan_init (void);

/**
 * @brief Point the servo at one angle, settle, and take one ranging.
 *
 * This is the primitive both scans are built from, and what the bench uses.
 * Blocks for SERVO_SETTLE_MSEC when the angle changes, waits out the
 * SONAR_MIN_CYCLE_MSEC since the previous ranging, then up to
 * SONAR_ECHO_TIMEOUT_USEC for the echo.
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
 *                       SCAN_OBSTACLE_RANGE_MM. Only bearing and closest
 *                       range are meaningful from a coarse scan.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_profile is NULL.
 */
car_status_t scan_coarse (car_obstacle_profile_t * p_profile);

/**
 * @brief Sweep a narrow arc in SCAN_FINE_STEP_DEG steps to profile it.
 *
 * Estimates the obstacle's bearing, closest range, width and the free
 * clearance on each side of it. A side the arc did not reach past the
 * obstacle reports zero clearance, which the planner treats as blocked.
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
 * @brief Begin a new line search arcing toward one side.
 *
 * @param[in] b_search_left true to arc left, false to arc right. Search
 *                          toward the side the line was last seen on.
 *
 * @return CAR_OK.
 */
car_status_t scan_recover_start (bool b_search_left);

/**
 * @brief Tell the search what the line sensors saw after the last step.
 *
 * @param[in] b_line_seen true if any sensor sees the line.
 *
 * @return CAR_OK.
 */
car_status_t scan_recover_report (bool b_line_seen);

/**
 * @brief Run one step of the search that brings the car back to the line.
 *
 * Called by the controller in CAR_STATE_RECOVER_LINE each time the previous
 * step has finished, until it returns CAR_OK. On CAR_ERR_NO_DATA the next
 * step to execute is available from scan_recover_get_step().
 *
 * @return CAR_OK once the line is reacquired, CAR_ERR_NO_DATA while still
 *         searching, CAR_ERR_TIMEOUT if the search pattern is exhausted.
 */
car_status_t scan_recover_line (void);

/**
 * @brief Read the step scan_recover_line() just issued.
 *
 * @param[out] p_action CAR_AVOID_LEFT or CAR_AVOID_RIGHT with an angle, or
 *                      CAR_AVOID_CONTINUE with a distance.
 * @param[out] p_amount Degrees for a turn, mm for a drive.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if either pointer is NULL.
 */
car_status_t scan_recover_get_step (car_avoid_action_t * p_action,
                                    uint16_t * p_amount);

#ifdef CAR_HOST_TEST
/**
 * @brief Host test hook: the range every ranging reports from now on.
 *
 * @param[in] range_mm Distance to report.
 */
void scan_host_inject_range (uint16_t range_mm);

/**
 * @brief Host test hook: the angle and pulse the servo was last sent to.
 *
 * @param[out] p_angle_deg  Last commanded angle, after clamping.
 * @param[out] p_pulse_usec Pulse width that angle produced.
 */
void scan_host_get_servo (uint16_t * p_angle_deg, uint16_t * p_pulse_usec);
#endif

#endif /* SCANNING_H */

/*** end of file ***/
