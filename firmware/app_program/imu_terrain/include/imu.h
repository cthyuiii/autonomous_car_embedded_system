/** @file imu.h
 *
 * @brief LSM303DLHC based tilt, hump, motion event and collision sensing.
 *
 * WARNING: The LSM303DLHC is an accelerometer plus magnetometer. It has no
 * gyroscope. Nothing in this module may integrate an angular rate, because
 * there is none to integrate. Every function below states where its number
 * really comes from.
 *
 * NOTE: Wheel data never enters this module through another module's
 * header. The controller reads the motion snapshot and hands the numbers
 * across with imu_feed_odometry(), once per sample period, so this module
 * has one caller and no dependency on motion.
 *
 * Units: angles in degrees, rates in degrees per second, acceleration in
 * milli g, heights in mm, time in milliseconds.
 *
 * Owner: Buddy 4, IMU based motion and terrain monitoring. A changed signature
 * here also changes the test, the bench and car_main.c, so agree it with the
 * team first.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "car.h"

/**
 * @brief Bring up I2C and configure both sensors at their sample rates.
 *
 * NOTE: The kernel's I2C unit 0 driver routes the bus to GP8 and GP9 by
 * default, which are motor pins on this board. This moves it to the Grove
 * port pins named in car_config.h before touching either sensor.
 *
 * @return CAR_OK if both devices answer, CAR_ERR_HARDWARE otherwise.
 */
car_status_t imu_init (void);

/**
 * @brief Capture the level reference and seed the magnetometer offsets.
 *
 * Averages IMU_CALIBRATION_SAMPLES readings with the car standing still
 * and level. Pitch is reported relative to that reference, so a slightly
 * tilted mount reads zero on the flat.
 *
 * NOTE: The magnetometer's hard iron offset is not captured here. It is
 * tracked from the running minimum and maximum of each axis as the car
 * turns, which is what makes it valid with the motors running. Do a full
 * turn on the floor before trusting the heading.
 *
 * WARNING: Call this before any task starts calling imu_update(). It reads
 * the bus itself and the driver serialises nothing between tasks.
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
 * @brief Hand the latest wheel data across from the motion snapshot.
 *
 * Distance drives the hump height integral, the two speeds drive the
 * encoder based turn rate. Call before imu_update() each period.
 *
 * @param[in] distance_mm      Total ground distance travelled so far.
 * @param[in] left_mm_per_sec  Left wheel speed, negative in reverse.
 * @param[in] right_mm_per_sec Right wheel speed, negative in reverse.
 *
 * @return CAR_OK.
 */
car_status_t imu_feed_odometry (uint32_t distance_mm, int16_t left_mm_per_sec,
                                int16_t right_mm_per_sec);

/**
 * @brief Copy the current pitch and heading.
 *
 * NOTE: Pitch comes from the gravity vector, so it is only valid while
 * the car is neither accelerating nor vibrating. An accelerometer cannot
 * tell a tilt from a push: both move the apparent gravity vector the same
 * way. Pitch is therefore held at its last value whenever the measured
 * acceleration strays more than IMU_PITCH_TRUST_BAND_MILLI_G from one g,
 * which is what a running motor does. imu_is_pitch_trusted() reports
 * whether the number currently means anything.
 *
 * NOTE: Heading is the magnetometer angle from magnetic north, corrupted
 * near the motors and not tilt compensated, because pitch on the course
 * stays small enough not to matter.
 *
 * @param[out] p_pitch_deg   Nose up positive.
 * @param[out] p_heading_deg 0 to 359 clockwise from magnetic north.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if either pointer is NULL.
 */
car_status_t imu_get_orientation (int16_t * p_pitch_deg,
                                  int16_t * p_heading_deg);

/**
 * @brief Report whether the current pitch reading can be believed.
 *
 * @return true while the acceleration magnitude sits within
 *         IMU_PITCH_TRUST_BAND_MILLI_G of one g, so the vector really is
 *         gravity and nothing else.
 */
bool imu_is_pitch_trusted (void);

/**
 * @brief Copy the filtered acceleration magnitude, for diagnostics.
 *
 * NOTE: One g, near 1000, means the car is still and level enough for the
 * tilt maths. A number that wanders while the car sits still is vibration,
 * and is the reason pitch would otherwise wander with it.
 *
 * @param[out] p_milli_g Magnitude of the filtered acceleration vector.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_milli_g is NULL.
 */
car_status_t imu_get_accel_magnitude (uint16_t * p_milli_g);

/**
 * @brief Largest raw acceleration magnitude since the last call.
 *
 * imu_get_accel_magnitude() reports the filtered vector, which is what the
 * trust gate needs and which smooths a knock almost entirely away. The
 * collision test runs on the raw sample instead, so this is the number
 * IMU_COLLISION_THRESHOLD_MILLI_G is really compared against, and the only
 * one worth watching while tapping the bumper. Reading it clears the peak
 * back to one g.
 *
 * @param[out] p_milli_g Peak raw magnitude, near one g if nothing happened.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_milli_g is NULL.
 */
car_status_t imu_get_peak_accel_magnitude (uint16_t * p_milli_g);

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
 * WARNING: There is no gyroscope. Rate is the wheel speed difference over
 * WHEEL_BASE_MM, fed in by imu_feed_odometry(), which is clean but blind
 * to wheel slip. The magnetometer's heading is too disturbed by the
 * motors to differentiate.
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
 * sin(pitch) over distance travelled while climbing, with distance from
 * the encoders through imu_feed_odometry().
 *
 * @param[out] p_height_mm Destination, must not be NULL. 0 before the
 *                         first hump.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_height_mm is NULL.
 */
car_status_t imu_get_peak_hump (uint16_t * p_height_mm);

/**
 * @brief Copy the height of the most recent hump, whatever its height.
 *
 * @param[out] p_height_mm Destination, must not be NULL. 0 before the
 *                         first hump.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_height_mm is NULL.
 */
car_status_t imu_get_last_hump (uint16_t * p_height_mm);

/**
 * @brief Report how many humps the car has crossed this run.
 *
 * @param[out] p_count Destination, must not be NULL.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_count is NULL.
 */
car_status_t imu_get_hump_count (uint16_t * p_count);

/**
 * @brief Report whether an impact was seen since the last imu_update().
 *
 * @return true once per impact above IMU_COLLISION_THRESHOLD_MILLI_G.
 */
bool imu_is_collision_detected (void);

/**
 * @brief Whether imu_calibrate() has succeeded since boot.
 *
 * NOTE: Pitch is measured against the level reference that calibration
 * takes, so every angle this module reports is meaningless until this is
 * true. The bench prints it for exactly that reason.
 *
 * @return true once a level reference has been captured.
 */
bool imu_is_calibrated (void);

/**
 * @brief How rough the ground is, as a smoothed departure from one g.
 *
 * NOTE: This is the same measurement the pitch trust gate uses, read as a
 * property of the terrain rather than of the reading. Near zero on a
 * smooth floor, because gravity is then the only force acting; every
 * bump, slip and rattle adds to it.
 *
 * @param[out] p_milli_g Filtered absolute deviation from one g.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_milli_g is NULL.
 */
car_status_t imu_get_terrain_roughness (uint16_t * p_milli_g);

/**
 * @brief Whether the ground is smooth enough to trust the car's footing.
 *
 * @return true while roughness is below IMU_TERRAIN_ROUGH_MILLI_G.
 */
bool imu_is_terrain_stable (void);

#ifdef CAR_HOST_TEST
/**
 * @brief Host test hook: the raw readings the next imu_update() sees.
 *
 * @param[in] ax_mg X acceleration, forward axis.
 * @param[in] ay_mg Y acceleration.
 * @param[in] az_mg Z acceleration, 1000 when level and still.
 * @param[in] mag_x    Magnetometer X, raw counts.
 * @param[in] mag_y    Magnetometer Y, raw counts.
 * @param[in] mag_z    Magnetometer Z, raw counts.
 */
void imu_host_inject (int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
                      int16_t mag_x, int16_t mag_y, int16_t mag_z);
#endif

#endif /* IMU_H */

/*** end of file ***/
