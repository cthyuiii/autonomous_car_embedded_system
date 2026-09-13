/** @file bench_comms.c
 *
 * @brief Connects, then heartbeats forever and echoes received commands.
 *
 * Flash this to a Pico W with the broker reachable. Watch the serial
 * output for connection state and use an MQTT client on a laptop to send
 * commands to COMMS_TOPIC_COMMAND and see them echoed back.
 */

#include <stdio.h>

#include "pico/stdlib.h"

#include "car_config.h"
#include "comms.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_POLL_MSEC      10u

static void print_command (car_nav_command_t command);

int main (void)
{
    uint32_t elapsed_msec = 0u;

    stdio_init_all();
    sleep_ms(BENCH_STARTUP_MSEC);
    printf("comms bench: broker %s\n", COMMS_MQTT_BROKER_HOST);

    (void)comms_set_command_handler(print_command);

    if (CAR_OK != comms_init())
    {
        printf("comms_init failed\n");
    }

    for (;;)
    {
        (void)comms_poll();

        if (elapsed_msec >= COMMS_HEARTBEAT_PERIOD_MSEC)
        {
            printf("connected %d\n", comms_is_connected());
            (void)comms_publish_heartbeat();
            elapsed_msec = 0u;
        }

        sleep_ms(BENCH_POLL_MSEC);
        elapsed_msec += BENCH_POLL_MSEC;
    }
}

static void print_command (car_nav_command_t command)
{
    printf("command %d\n", command);
}

/*** end of file ***/

