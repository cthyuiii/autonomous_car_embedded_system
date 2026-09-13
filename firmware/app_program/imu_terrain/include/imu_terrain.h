/** @file imu_terrain.h
 *
 * @brief LSM303DLHC based tilt, hump, motion event and collision sensing.
 *
 * WARNING: The LSM303DLHC is an accelerometer plus magnetometer. It has no
 * gyroscope. Nothing in this module may integrate an angular rate, because
 * there is none to integrate. Every function below states where its number
 * really comes from.
 *
 * Units: angles in degrees, rates in degrees per second, acceleration in
 * milli g, heights in mm, time in milliseconds.
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

/**
 * @brief Bring up I2C and configure both sensors at their sample rates.
 *
 * @return CAR_OK if both devices answer, CAR_ERR_HARDWARE otherwise.
 */
car_status_t imu_init (void);

/**
 * @brief Capture the level reference and the magnetometer offsets.
 *
 * NOTE: Run this with the car level and the motors running, because the
 * motor fields shift the magnetometer's hard iron offset. A calibration
 * taken with motors off is wrong the moment the car moves.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if the readings are implausible.
 */
car_status_t imu_calibrate (void);

/**
 * @brief Sample both sensors and update the filters.
 *
 * Call every IMU_SAMPLE_PERIOD_MSEC. Everything else in this module reads
 * the state this produces, so nothing changes between calls.
 *
 * @return CAR_OK, or CAR_ERR_HARDWARE on an I2C failure.
 */
car_status_t imu_update (void);

/**
 * @brief Copy the current pitch and heading.
 *
 * NOTE: Pitch comes from the gravity vector, which is only valid when the
 * car is not accelerating hard. Heading is the tilt compensated
 * magnetometer angle from magnetic north, corrupted near the motors.
 *
 * @param[out] p_pitch_deg   Nose up positive.
 * @param[out] p_heading_deg 0 to 359 clockwise from magnetic north.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if either pointer is NULL.
 */
car_status_t imu_get_orientation (int16_t * p_pitch_deg,
                                  int16_t * p_heading_deg);

/**
 * @brief Classify the current motion from the filtered acceleration.
 *
 * @param[out] p_event One of car_motion_event_t.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_event is NULL.
 */
car_status_t imu_get_event (car_motion_event_t * p_event);

/**
 * @brief Estimate the yaw rate.
 *
 * WARNING: There is no gyroscope. IMU_TURN_RATE_FROM_ENCODERS selects the
 * source. From encoders, rate is the wheel speed difference over
 * WHEEL_BASE_MM, which is clean but blind to wheel slip. From the
 * magnetometer, rate is the heading derivative, which sees slip but is
 * noisy and disturbed by the motors. Both doors are left open.
 *
 * @param[out] p_rate_dps Clockwise positive.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_rate_dps is NULL.
 */
car_status_t imu_get_turn_rate_dps (int16_t * p_rate_dps);

/**
 * @brief Report whether the car is currently on a hump.
 *
 * @return true while pitch exceeds IMU_HUMP_PITCH_THRESHOLD_DEG.
 */
bool imu_is_hump_detected (void);

/**
 * @brief Copy the highest hump seen so far this run.
 *
 * NOTE: Height is not measured, it is estimated. Double integrating the
 * vertical acceleration drifts far more than a hump is tall in the two
 * seconds it takes to cross one. Instead, height is the integral of
 * sin(pitch) over distance travelled, with distance from the encoders.
 *
 * @param[out] p_hump Destination, must not be NULL.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_hump is NULL.
 */
car_status_t imu_get_peak_hump (car_hump_t * p_hump);

/**
 * @brief Report whether an impact was seen since the last imu_update().
 *
 * @return true once per impact above IMU_COLLISION_THRESHOLD_MILLI_G.
 */
bool imu_is_collision_detected (void);

#endif /* IMU_TERRAIN_H */

/*** end of file ***/

