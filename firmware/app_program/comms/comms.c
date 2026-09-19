/** @file comms.c
 *
 * @brief WiFi and MQTT client on the kernel's CYW43 service and lwIP.
 *
 * NOTE: Build with WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1
 * WIFI_MQTT=1. lwIP runs NO_SYS=1 on the radio owner task, so nothing here
 * calls lwIP. Messages cross to that task through the ring buffers in
 * lib/libnet/lwip/lwip_utk_mqtt.h, and every call into this module must
 * still come from one task, the comms task, because the rings are single
 * producer and single consumer.
 *
 * NOTE: Without the WIFI_MQTT profile this module reports itself absent
 * from comms_init(), which the controller treats as a car without a radio
 * rather than a fault.
 *
 * Owner: Buddy 1, WiFi communication, command and telemetry. Implement the
 * TODOs in this file. It is yours.
 */

#include "comms.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#include <stdio.h>
#else
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_hw.h"
#if defined(TM_WIFI_MQTT) && TM_WIFI_MQTT
#include "lwip_utk_mqtt.h"
#define COMMS_HAS_RADIO 1
#endif
#endif

#include "car_config.h"

#ifndef COMMS_HAS_RADIO
#define COMMS_HAS_RADIO 0
#endif

#define COMMS_TOPIC_MAX_BYTES 32u

static comms_command_handler_t g_p_command_handler = NULL;
static bool                    g_b_connected       = false;
static char                    g_payload[COMMS_PAYLOAD_MAX_BYTES];
#ifdef CAR_HOST_TEST
static uint16_t                g_host_length       = 0u;
#endif

static uint16_t          format_telemetry (car_telemetry_t const * p_telemetry);
static uint16_t          format_heartbeat (void);
#if defined(CAR_HOST_TEST) || COMMS_HAS_RADIO
static car_nav_command_t parse_command (char const * p_text, uint16_t length);
#endif
static car_status_t      send (char const * p_topic, uint16_t length);
static uint32_t          uptime_msec (void);

car_status_t comms_init (void)
{
    // TODO: Bring up the port's cyw43_utk service, which reads the SSID
    //       and password from config/wifi_credentials.h, and start the
    //       join with COMMS_WIFI_TIMEOUT_MSEC.
    car_status_t status = CAR_ERR_NOT_IMPLEMENTED;

    g_b_connected = false;

#ifdef CAR_HOST_TEST
    status = CAR_OK;
#elif COMMS_HAS_RADIO
    {
        /* The radio service itself was started by the kernel at boot and
         * joins with the credentials from config/wifi_credentials.h. All
         * that is left is telling the MQTT phase where the broker is. */
        T_LWIP_UTK_MQTT_CONFIG config =
        {
            .host              = COMMS_MQTT_BROKER_HOST,
            .port              = COMMS_MQTT_BROKER_PORT,
            .client_id         = COMMS_MQTT_CLIENT_ID,
            .subscribe_topic   = COMMS_TOPIC_COMMAND,
            .keepalive_seconds = COMMS_MQTT_KEEPALIVE_SECONDS,
            .reconnect_ms      = COMMS_RECONNECT_BACKOFF_MSEC,
        };

        lwip_utk_mqtt_configure(&config);
        status = CAR_OK;
    }
#endif

    return status;
}

car_status_t comms_poll (void)
{
    // TODO: Service lwIP, advance the connect state machine, open the
    //       MQTT session to COMMS_MQTT_BROKER_HOST once the netif is up,
    //       subscribe to COMMS_TOPIC_COMMAND, and dispatch any received
    //       payload to g_p_command_handler.
#ifdef CAR_HOST_TEST
    /* The host has a broker that is always there, and delivers whatever
     * the test injected. */
    g_b_connected = true;

    if (0u != g_host_length)
    {
        car_nav_command_t command = parse_command(g_payload, g_host_length);

        g_host_length = 0u;

        if ((CAR_NAV_NONE != command) && (NULL != g_p_command_handler))
        {
            g_p_command_handler(command);
        }
    }
#elif COMMS_HAS_RADIO
    {
        T_LWIP_UTK_MQTT_STATUS status = { 0 };
        char                   topic[COMMS_TOPIC_MAX_BYTES];
        uint16_t               length = 0u;

        lwip_utk_mqtt_get_status(&status);
        g_b_connected = (0u != status.connected);

        while (0 != lwip_utk_mqtt_receive(topic, (uint16_t)sizeof(topic),
                                          g_payload,
                                          (uint16_t)sizeof(g_payload),
                                          &length))
        {
            car_nav_command_t command = parse_command(g_payload, length);

            if ((CAR_NAV_NONE != command) && (NULL != g_p_command_handler))
            {
                g_p_command_handler(command);
            }
        }
    }
#else
    g_b_connected = false;
#endif

    return CAR_OK;
}

bool comms_is_connected (void)
{
    return g_b_connected;
}

car_status_t comms_publish_telemetry (car_telemetry_t const * p_telemetry)
{
    // TODO: Serialise every field to one JSON line and publish on
    //       COMMS_TOPIC_TELEMETRY. Document the JSON keys in README.md.
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_telemetry)
    {
        status = send(COMMS_TOPIC_TELEMETRY, format_telemetry(p_telemetry));
    }

    return status;
}

car_status_t comms_set_command_handler (comms_command_handler_t p_handler)
{
    // TODO: Return CAR_OK once comms_poll() actually invokes the handler.
    g_p_command_handler = p_handler;

    return CAR_OK;
}

car_status_t comms_publish_heartbeat (void)
{
    // TODO: Publish uptime and connection state on COMMS_TOPIC_HEARTBEAT.
    return send(COMMS_TOPIC_HEARTBEAT, format_heartbeat());
}

/**
 * @brief Hand one formatted payload to the transport.
 *
 * @param[in] p_topic Topic string.
 * @param[in] length  Bytes of g_payload to send.
 *
 * @return CAR_OK if queued, CAR_ERR_TIMEOUT if there is no session.
 */
static car_status_t send (char const * p_topic, uint16_t length)
{
    car_status_t status = CAR_ERR_TIMEOUT;

    if (g_b_connected && (0u != length))
    {
#ifdef CAR_HOST_TEST
        (void)p_topic;
        status = CAR_OK;
#elif COMMS_HAS_RADIO
        if (LWIP_UTK_MQTT_QUEUED == lwip_utk_mqtt_publish(p_topic, g_payload,
                                                          length))
        {
            status = CAR_OK;
        }
#else
        (void)p_topic;
#endif
    }

    return status;
}

#if defined(CAR_HOST_TEST) || COMMS_HAS_RADIO
/**
 * @brief Read a navigation command out of a received payload.
 *
 * The first letter decides: L, R, S or U in either case. Anything else is
 * ignored, so a stray message cannot steer the car.
 *
 * @param[in] p_text Payload bytes, not terminated.
 * @param[in] length Payload length.
 *
 * @return The command, or CAR_NAV_NONE.
 */
static car_nav_command_t parse_command (char const * p_text, uint16_t length)
{
    car_nav_command_t command = CAR_NAV_NONE;
    uint16_t          index   = 0u;

    while ((index < length) && (CAR_NAV_NONE == command))
    {
        char letter = p_text[index];

        if ((letter >= 'a') && (letter <= 'z'))
        {
            letter = (char)(letter - ('a' - 'A'));
        }

        if ('L' == letter)
        {
            command = CAR_NAV_LEFT;
        }
        else if ('R' == letter)
        {
            command = CAR_NAV_RIGHT;
        }
        else if ('S' == letter)
        {
            command = CAR_NAV_STRAIGHT;
        }
        else if ('U' == letter)
        {
            command = CAR_NAV_UTURN;
        }
        else
        {
            /* Skip punctuation, quotes and braces around the letter. */
        }

        index++;
    }

    return command;
}
#endif /* CAR_HOST_TEST || COMMS_HAS_RADIO */

#ifdef CAR_HOST_TEST
#define COMMS_FORMAT(...) snprintf(g_payload, sizeof(g_payload), __VA_ARGS__)
#else
/* The kernel's formatter has no length argument. Every format below is
 * bounded by construction: the widest field set is under 200 bytes. */
#define COMMS_FORMAT(...) tm_sprintf((UB *)g_payload, (UB const *)__VA_ARGS__)
#endif

/**
 * @brief Write one telemetry snapshot as a JSON object into g_payload.
 *
 * Keys are documented in comms/README.md.
 *
 * @param[in] p_telemetry Snapshot to serialise.
 *
 * @return Bytes written, 0 if it did not fit.
 */
static uint16_t format_telemetry (car_telemetry_t const * p_telemetry)
{
    int written = COMMS_FORMAT(
        "{\"state\":%u,\"speed\":%u,\"encl\":%u,\"encr\":%u,\"mask\":%u,"
        "\"nav\":%u,\"hump\":%u,\"dist\":%u,\"obst\":{\"valid\":%u,"
        "\"bearing\":%d,\"range\":%u,\"width\":%u,\"left\":%u,"
        "\"right\":%u},\"imu\":{\"pitch\":%d,\"ok\":%u,\"head\":%d,"
        "\"rate\":%d,\"event\":%u,\"hit\":%u,\"stable\":%u,"
        "\"rough\":%u}}",
        (unsigned)p_telemetry->mission_state,
        (unsigned)p_telemetry->speed_mm_per_sec,
        (unsigned)p_telemetry->encoder_count_left,
        (unsigned)p_telemetry->encoder_count_right,
        (unsigned)p_telemetry->line_sensor_mask,
        (unsigned)p_telemetry->last_nav_command,
        (unsigned)p_telemetry->peak_hump_height_mm,
        (unsigned)p_telemetry->total_distance_mm,
        (unsigned)p_telemetry->last_obstacle.b_is_valid,
        (int)p_telemetry->last_obstacle.bearing_deg,
        (unsigned)p_telemetry->last_obstacle.closest_range_mm,
        (unsigned)p_telemetry->last_obstacle.width_mm,
        (unsigned)p_telemetry->last_obstacle.clearance_left_mm,
        (unsigned)p_telemetry->last_obstacle.clearance_right_mm,
        (int)p_telemetry->pitch_deg,
        (unsigned)p_telemetry->b_pitch_trusted,
        (int)p_telemetry->heading_deg,
        (int)p_telemetry->turn_rate_dps,
        (unsigned)p_telemetry->motion_event,
        (unsigned)p_telemetry->b_collision,
        (unsigned)p_telemetry->b_terrain_stable,
        (unsigned)p_telemetry->terrain_rough_milli_g);

    if ((written < 0) || (written >= (int)sizeof(g_payload)))
    {
        written = 0;
    }

    return (uint16_t)written;
}

/**
 * @brief Write the heartbeat JSON object into g_payload.
 *
 * @return Bytes written.
 */
static uint16_t format_heartbeat (void)
{
    int written = COMMS_FORMAT("{\"uptime_ms\":%u,\"connected\":%u}",
                               (unsigned)uptime_msec(),
                               (unsigned)g_b_connected);

    if ((written < 0) || (written >= (int)sizeof(g_payload)))
    {
        written = 0;
    }

    return (uint16_t)written;
}

#ifdef CAR_HOST_TEST

void comms_host_inject (char const * p_text, uint16_t length)
{
    uint16_t index = 0u;

    if (length > (uint16_t)sizeof(g_payload))
    {
        length = (uint16_t)sizeof(g_payload);
    }

    for (index = 0u; index < length; index++)
    {
        g_payload[index] = p_text[index];
    }

    g_host_length = length;
}

static uint32_t uptime_msec (void)
{
    static uint32_t fake_msec = 0u;

    fake_msec += COMMS_POLL_PERIOD_MSEC;

    return fake_msec;
}

#else /* CAR_HOST_TEST */

static uint32_t uptime_msec (void)
{
    return car_hw_msec();
}

#endif /* CAR_HOST_TEST */

/*** end of file ***/
