/** @file car.h
 *
 * @brief Every type that crosses a subsystem boundary.
 *
 * NOTE: If a type is only used inside one module it does not belong here.
 * Keeping this file small keeps the five subsystems loosely coupled.
 *
 * Owner: the team. Change only by agreement, since every module and every test
 * depends on it.
 */

#ifndef CAR_H
#define CAR_H

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
    CAR_STATE_HALTED,
    /* Appended so the numbers above stay put: they are published over
     * MQTT and printed on the console, and renumbering them would
     * silently change what every recorded log means. */
    CAR_STATE_COLLISION
} car_mission_state_t;

/** What the car is doing right now, published as text over MQTT.
 *  Appended to only, like car_mission_state_t. */
typedef enum
{
    CAR_ACTION_STARTING = 0,
    CAR_ACTION_FOLLOWING,
    CAR_ACTION_ACQUIRING,           /* Turning back onto the line */
    CAR_ACTION_LINE_LOST,
    CAR_ACTION_READING_BARCODE,
    CAR_ACTION_AT_JUNCTION,         /* Stopped, waiting for a command */
    CAR_ACTION_TURNING_LEFT,
    CAR_ACTION_TURNING_RIGHT,
    CAR_ACTION_U_TURN,
    CAR_ACTION_SCANNING,            /* Profiling an obstacle */
    CAR_ACTION_DETOUR,              /* Driving the box round it */
    CAR_ACTION_REVERSING,           /* Backing off to probe a lane */
    CAR_ACTION_BACKING_OUT,         /* Undoing legs of a blocked lane */
    CAR_ACTION_SEARCHING,           /* Searching for the line */
    CAR_ACTION_BACKING_OFF,         /* After a hit or a stall */
    CAR_ACTION_HALTED
} car_action_t;

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
    /* IMU block. Everything Buddy 4 reports, so the terrain analysis can
     * be read off the broker instead of only off the bench console. */
    int16_t                pitch_deg;
    bool                   b_pitch_trusted;
    int16_t                heading_deg;
    int16_t                turn_rate_dps;
    car_motion_event_t     motion_event;
    bool                   b_collision;
    bool                   b_terrain_stable;
    uint16_t               terrain_rough_milli_g;
    /* Appended for the action and terrain messages. */
    car_action_t           action;
    uint16_t               accel_milli_g;
    bool                   b_on_hump;
    uint16_t               hump_count;
    uint16_t               last_hump_height_mm;
} car_telemetry_t;

#endif /* CAR_H */

/*** end of file ***/

