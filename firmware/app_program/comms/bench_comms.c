/** @file bench_comms.c
 *
 * @brief Connects, then heartbeats forever and echoes received commands.
 *
 * Build with `make BENCH=comms WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1
 * WIFI_DHCP=1` and flash to a Pico W with the broker reachable. Watch the
 * serial output for connection state, and send commands to
 * COMMS_TOPIC_COMMAND from a laptop MQTT client to see them echoed.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "comms.h"

#define BENCH_STARTUP_MSEC 2000u

static void print_command (car_nav_command_t command);

INT usermain (void)
{
    uint32_t elapsed_msec = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "comms bench: broker %s\n", COMMS_MQTT_BROKER_HOST);

    (void)comms_set_command_handler(print_command);

    if (CAR_OK != comms_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "comms_init failed\n");
    }

    for (;;)
    {
        (void)comms_poll();

        if (elapsed_msec >= COMMS_HEARTBEAT_PERIOD_MSEC)
        {
            CAR_LOG(CAR_LOG_INFO, "connected %d\n", comms_is_connected());
            (void)comms_publish_heartbeat();
            elapsed_msec = 0u;
        }

        (void)tk_dly_tsk(COMMS_POLL_PERIOD_MSEC);
        elapsed_msec += COMMS_POLL_PERIOD_MSEC;
    }
}

static void print_command (car_nav_command_t command)
{
    CAR_LOG(CAR_LOG_INFO, "command %d\n", command);
}

/*** end of file ***/

