/** @file car_main.c
 *
 * @brief Vehicle controller. Owns mission state and drives every subsystem.
 *
 * This is the only file that includes all five subsystem headers. Anything
 * one subsystem needs from another flows through here, or through the
 * types in car_types.h, never by one module including another.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"

#include "car_config.h"
#include "car_log.h"
#include "car_types.h"
#include "comms.h"
#include "imu_terrain.h"
#include "line_barcode.h"
#include "motion.h"
#include "scanning.h"

static car_mission_state_t g_state     = CAR_STATE_INIT;
static car_telemetry_t     g_telemetry = { 0 };

static void handle_remote_command (car_nav_command_t command);
static void run_state_machine (void);
static void publish_telemetry (void);

int main (void)
{
    uint32_t last_telemetry_msec = 0u;

    stdio_init_all();
    car_log_write(CAR_LOG_INFO, "car firmware starting\n");

    /* Bring up every subsystem. Any failure halts before the wheels move. */
    if ((CAR_OK != motion_init()) || (CAR_OK != line_init())
        || (CAR_OK != imu_init()) || (CAR_OK != scan_init())
        || (CAR_OK != comms_init()))
    {
        car_log_write(CAR_LOG_ERROR, "subsystem init failed, halting\n");
        g_state = CAR_STATE_HALTED;
    }

    (void)comms_set_command_handler(handle_remote_command);

    for (;;)
    {
        uint32_t now_msec = to_ms_since_boot(get_absolute_time());

        /* Periodic work that runs in every state. */
        (void)motion_tick();
        (void)imu_update();
        (void)comms_poll();

        run_state_machine();

        if ((now_msec - last_telemetry_msec) >= CAR_TELEMETRY_PERIOD_MSEC)
        {
            publish_telemetry();
            last_telemetry_msec = now_msec;
        }

        sleep_ms(CAR_MAIN_LOOP_PERIOD_MSEC);
    }
}

/**
 * @brief Receive a navigation command from MQTT.
 *
 * @param[in] command The decoded command.
 */
static void handle_remote_command (car_nav_command_t command)
{
    // TODO: Queue the command so run_state_machine() can act on it from
    //       CAR_STATE_FOLLOW_LINE. Do not call motion from here.
    g_telemetry.last_nav_command = command;
}

/**
 * @brief Advance the mission by one step. Called every main loop tick.
 */
static void run_state_machine (void)
{
    switch (g_state)
    {
        case CAR_STATE_INIT:
            // TODO: Go to FOLLOW_LINE once line_calibrate() and
            //       imu_calibrate() both return CAR_OK.
        break;

        case CAR_STATE_FOLLOW_LINE:
            // TODO: line_get_position() into a steering correction each
            //       tick. Go to DECODE_BARCODE when barcode_poll() returns
            //       CAR_OK, to AVOID_OBSTACLE when scan_coarse() reports a
            //       valid profile, to HALTED on imu_is_collision_detected().
        break;

        case CAR_STATE_DECODE_BARCODE:
            // TODO: Store the command in g_telemetry.last_nav_command and go
            //       to EXECUTE_TURN, or back to FOLLOW_LINE for STRAIGHT.
        break;

        case CAR_STATE_EXECUTE_TURN:
            // TODO: Issue motion_turn_left(), motion_turn_right() or a
            //       180 degree turn once, then go to FOLLOW_LINE when
            //       motion_is_busy() goes false.
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            // TODO: scan_fine() around the coarse bearing, store the
            //       profile in g_telemetry.last_obstacle, then act on
            //       scan_plan_avoidance() and go to RECOVER_LINE.
        break;

        case CAR_STATE_RECOVER_LINE:
            // TODO: Call scan_recover_line() each tick. Go to FOLLOW_LINE on
            //       CAR_OK, to HALTED on CAR_ERR_TIMEOUT.
        break;

        case CAR_STATE_HALTED:
            (void)motion_stop();
        break;

        default:
            g_state = CAR_STATE_HALTED;
        break;
    }
}

/**
 * @brief Gather one snapshot from every subsystem and publish it.
 */
static void publish_telemetry (void)
{
    motion_state_t motion = { 0 };
    car_hump_t     hump   = { 0 };

    (void)motion_get_state(&motion);
    (void)imu_get_peak_hump(&hump);
    (void)line_get_sensor_mask(&g_telemetry.line_sensor_mask);

    g_telemetry.mission_state       = g_state;
    g_telemetry.speed_mm_per_sec    = motion.speed_mm_per_sec;
    g_telemetry.encoder_count_left  = motion.encoder_count_left;
    g_telemetry.encoder_count_right = motion.encoder_count_right;
    g_telemetry.total_distance_mm   = motion.distance_mm;
    g_telemetry.peak_hump_height_mm = hump.peak_height_mm;

    (void)comms_publish_telemetry(&g_telemetry);
}

/*** end of file ***/

