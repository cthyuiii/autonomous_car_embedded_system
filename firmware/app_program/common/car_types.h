/** @file car_types.h
 *
 * @brief Every type that crosses a subsystem boundary.
 *
 * NOTE: If a type is only used inside one module it does not belong here.
 * Keeping this file small keeps the five subsystems loosely coupled.
 *
 * Owner: the team. Change only by agreement, since every module and every test
 * depends on it.
 */

#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/** Return status shared by every public function in the firmware. */
typedef enum
{
    CAR_OK = 0,
    CAR_ERR_TIMEOUT,
    CAR_ERR_RANGE,
    CAR_ERR_HARDWARE,
    CAR_ERR_NO_DATA,
    CAR_ERR_NOT_IMPLEMENTED
} car_status_t;

/** Navigation command decoded from a barcode or received over MQTT. */
typedef enum
{
    CAR_NAV_NONE = 0,
    CAR_NAV_LEFT,
    CAR_NAV_RIGHT,
    CAR_NAV_STRAIGHT,
    CAR_NAV_UTURN
} car_nav_command_t;

/** Motion class reported by the IMU module. */
typedef enum
{
    CAR_MOTION_STATIONARY = 0,
    CAR_MOTION_ACCELERATING,
    CAR_MOTION_TURNING,
    CAR_MOTION_CLIMBING,
    CAR_MOTION_DESCENDING,
    CAR_MOTION_IMPACT
} car_motion_event_t;

/** Action chosen by the obstacle avoidance planner. */
typedef enum
{
    CAR_AVOID_STOP = 0,
    CAR_AVOID_CONTINUE,
    CAR_AVOID_LEFT,
    CAR_AVOID_RIGHT,
    CAR_AVOID_REVERSE
} car_avoid_action_t;

/** Top level mission state owned by the vehicle controller. */
typedef enum
{
    CAR_STATE_INIT = 0,
    CAR_STATE_FOLLOW_LINE,
    CAR_STATE_DECODE_BARCODE,
    CAR_STATE_EXECUTE_TURN,
    CAR_STATE_AVOID_OBSTACLE,
    CAR_STATE_RECOVER_LINE,
    CAR_STATE_HALTED
} car_mission_state_t;

/** One hump measurement. Height is estimated, not measured directly. */
typedef struct
{
    uint16_t peak_height_mm;
    uint32_t timestamp_msec;
    bool     b_is_run_peak;
} car_hump_t;

/** Result of an ultrasonic scan, all distances in mm from the sensor. */
typedef struct
{
    int16_t  bearing_deg;
    uint16_t closest_range_mm;
    uint16_t width_mm;
    uint16_t clearance_left_mm;
    uint16_t clearance_right_mm;
    bool     b_is_valid;
} car_obstacle_profile_t;

/** Snapshot published over MQTT. Filled by the vehicle controller. */
typedef struct
{
    car_mission_state_t    mission_state;
    uint16_t               speed_mm_per_sec;
    uint32_t               encoder_count_left;
    uint32_t               encoder_count_right;
    uint8_t                line_sensor_mask;
    car_nav_command_t      last_nav_command;
    uint16_t               peak_hump_height_mm;
    car_obstacle_profile_t last_obstacle;
    uint32_t               total_distance_mm;
} car_telemetry_t;

#endif /* CAR_TYPES_H */

/*** end of file ***/

