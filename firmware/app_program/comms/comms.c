/** @file comms.c
 *
 * @brief WiFi and MQTT client on the kernel's CYW43 service and lwIP.
 *
 * NOTE: Build with WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 and,
 * for MQTT, the WIFI_TCP=1 profile chain. lwIP runs NO_SYS=1, so every
 * call into this module must come from one task. The MQTT client is
 * lwIP's own apps/mqtt raw API client from the Pico SDK's lwIP tree.
 */

#include "comms.h"

#include <stddef.h>

#include "car_config.h"

static comms_command_handler_t g_p_command_handler = NULL;
static bool                    g_b_connected       = false;

car_status_t comms_init (void)
{
    // TODO: Bring up the port's cyw43_utk service, which reads the SSID
    //       and password from config/wifi_credentials.h, and start the
    //       join with COMMS_WIFI_TIMEOUT_MSEC.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t comms_poll (void)
{
    // TODO: Service lwIP, advance the connect state machine, open the
    //       MQTT session to COMMS_MQTT_BROKER_HOST once the netif is up,
    //       subscribe to COMMS_TOPIC_COMMAND, and dispatch any received
    //       payload to g_p_command_handler.
    return CAR_ERR_NOT_IMPLEMENTED;
}

bool comms_is_connected (void)
{
    return g_b_connected;
}

car_status_t comms_publish_telemetry (car_telemetry_t const * p_telemetry)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_telemetry)
    {
        // TODO: Serialise every field to one JSON line and publish on
        //       COMMS_TOPIC_TELEMETRY. Document the JSON keys in README.md.
        status = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

car_status_t comms_set_command_handler (comms_command_handler_t p_handler)
{
    g_p_command_handler = p_handler;

    // TODO: Return CAR_OK once comms_poll() actually invokes the handler.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t comms_publish_heartbeat (void)
{
    // TODO: Publish uptime and connection state on COMMS_TOPIC_HEARTBEAT.
    return CAR_ERR_NOT_IMPLEMENTED;
}

/*** end of file ***/

