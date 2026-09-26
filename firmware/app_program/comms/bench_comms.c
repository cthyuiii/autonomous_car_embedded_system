/** @file bench_comms.c
 *
 * @brief Connects, then publishes the car's three messages on the car's
 *        timing and answers commands.
 *
 * Build and flash with `./flash.sh --wifi comms` to a Pico W with the broker
 * reachable, then watch every topic from a laptop:
 *
 *     mosquitto_sub -h <broker ip> -t 'car/#' -v
 *
 * - car/telemetry every CAR_TELEMETRY_PERIOD_MSEC (200 ms)
 * - car/heartbeat every COMMS_HEARTBEAT_PERIOD_MSEC (1 s)
 * - car/terrain every CAR_TERRAIN_PERIOD_MSEC (2 s)
 *
 * Only the radio is fitted, so the readings in them stay at zero. What
 * moves is the command path: `mosquitto_pub -h <broker ip> -t car/command
 * -m L` shows up in the next telemetry as `nav` 1 with `action`
 * "turning left", and R, S and U do the same for their commands.
 *
 * Owner: Buddy 1, WiFi communication, command and telemetry. Extend it as you
 * need; nothing else depends on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "car_time.h"
#include "comms.h"

#define BENCH_STARTUP_MSEC 2000u

/* Written only from comms_poll(), on this one task, so no lock. */
static car_telemetry_t g_telemetry =
{
    .action           = CAR_ACTION_HALTED,
    .b_terrain_stable = true,
};

static void answer_command (car_nav_command_t command);

INT usermain (void)
{
    uint32_t last_telemetry_msec = 0u;
    uint32_t last_heartbeat_msec = 0u;
    uint32_t last_terrain_msec   = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "comms bench: broker %s\n", COMMS_MQTT_BROKER_HOST);

    (void)comms_set_command_handler(answer_command);

    if (CAR_OK != comms_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "comms_init failed\n");
    }

    for (;;)
    {
        /* Timed off the clock: a kernel delay runs a tick long, so adding
         * up nominal delays ran every message at half its rate. */
        uint32_t now = car_time_msec();

        (void)comms_poll();

        if ((now - last_telemetry_msec) >= CAR_TELEMETRY_PERIOD_MSEC)
        {
            (void)comms_publish_telemetry(&g_telemetry);
            last_telemetry_msec = now;
        }

        if ((now - last_heartbeat_msec) >= COMMS_HEARTBEAT_PERIOD_MSEC)
        {
            CAR_LOG(CAR_LOG_INFO, "connected %d\n", comms_is_connected());
            (void)comms_publish_heartbeat();
            last_heartbeat_msec = now;
        }

        if ((now - last_terrain_msec) >= CAR_TERRAIN_PERIOD_MSEC)
        {
            (void)comms_publish_terrain(&g_telemetry);
            last_terrain_msec = now;
        }

        (void)tk_dly_tsk(COMMS_POLL_PERIOD_MSEC);
    }
}

/**
 * @brief Put a received command into the telemetry, with its action.
 *
 * @param[in] command The decoded command.
 */
static void answer_command (car_nav_command_t command)
{
    CAR_LOG(CAR_LOG_INFO, "command %d\n", command);
    g_telemetry.last_nav_command = command;

    switch (command)
    {
        case CAR_NAV_LEFT:
            g_telemetry.action = CAR_ACTION_TURNING_LEFT;
        break;

        case CAR_NAV_RIGHT:
            g_telemetry.action = CAR_ACTION_TURNING_RIGHT;
        break;

        case CAR_NAV_STRAIGHT:
            g_telemetry.action = CAR_ACTION_FOLLOWING;
        break;

        case CAR_NAV_UTURN:
            g_telemetry.action = CAR_ACTION_U_TURN;
        break;

        default:
            g_telemetry.action = CAR_ACTION_HALTED;
        break;
    }
}

/*** end of file ***/
