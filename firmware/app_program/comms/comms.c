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
 * Owner: Buddy 1, WiFi communication, command and telemetry. This file is
 * yours.
 */

#include "comms.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#include <stdio.h>
#else
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_time.h"
#if defined(TM_WIFI_MQTT) && TM_WIFI_MQTT
#include "lwip_utk_mqtt.h"
#define COMMS_HAS_RADIO 1
#endif
#endif

#include "car_config.h"
#include "car_log.h"

#ifndef COMMS_HAS_RADIO
#define COMMS_HAS_RADIO 0
#endif

#if COMMS_HAS_RADIO && (COMMS_PAYLOAD_MAX_BYTES > LWIP_UTK_MQTT_TX_PAYLOAD)
#error "Every formatted message must fit one MQTT ring slot"
#endif

#define COMMS_TOPIC_MAX_BYTES 32u

/* Names published for car_action_t and car_motion_event_t, in order. */
static char const * const g_action_names[CAR_ACTION_HALTED + 1u] =
{
    "starting", "following line", "turning onto line", "line lost",
    "reading barcode", "stopped at junction", "turning left",
    "turning right", "u-turn", "scanning obstacle", "going round obstacle",
    "reversing to probe", "backing out of lane", "searching for line",
    "backing off after hit", "halted"
};
static char const * const g_event_names[CAR_MOTION_IMPACT + 1u] =
{
    "STILL", "ACCEL", "TURN", "CLIMB", "DESCEND", "IMPACT"
};

static comms_command_handler_t gp_command_handler  = NULL;
static bool                    gb_connected        = false;
static uint32_t                g_sessions          = 0u;   // MQTT connects
static char                    g_payload[COMMS_PAYLOAD_MAX_BYTES];
#ifdef CAR_HOST_TEST
static uint16_t                g_host_length       = 0u;
#endif

static uint16_t          format_telemetry (car_telemetry_t const * p_telemetry);
static uint16_t          format_heartbeat (void);
static uint16_t          format_terrain (car_telemetry_t const * p_telemetry);
static uint16_t          fitted (int32_t written);
static char const *      action_name (car_action_t action);
static char const *      event_name (car_motion_event_t event);
#if defined(CAR_HOST_TEST) || COMMS_HAS_RADIO
static car_nav_command_t parse_command (char const * p_text, uint16_t length);
#endif
static car_status_t      send (char const * p_topic, uint16_t length);
static uint32_t          uptime_msec (void);

car_status_t comms_init (void)
{
    car_status_t status = CAR_ERR_NOT_IMPLEMENTED;

    gb_connected = false;

#ifdef CAR_HOST_TEST
    status = CAR_OK;
#elif COMMS_HAS_RADIO
    {
        /* The radio service itself was started by the kernel at boot and
         * joins with the credentials from config/wifi_credentials.h. All
         * that is left is telling the MQTT phase where the broker is. */
        lwip_utk_mqtt_config_t config =
        {
            .host              = COMMS_MQTT_BROKER_HOST,
            .port              = COMMS_MQTT_BROKER_PORT,
            .client_id         = COMMS_MQTT_CLIENT_ID,
            .subscribe_topic   = COMMS_TOPIC_COMMAND,
            .log_topic         = COMMS_TOPIC_LOG,
            .keepalive_seconds = COMMS_MQTT_KEEPALIVE_SECONDS,
            .reconnect_ms      = COMMS_RECONNECT_BACKOFF_MSEC,
        };
        lwip_utk_mqtt_status_t now = { 0 };

        /* The log mirror may have started it already, and configuring it
         * again would drop a session in progress. */
        lwip_utk_mqtt_get_status(&now);

        if (!now.b_configured)
        {
            lwip_utk_mqtt_configure(&config);
        }

        status = CAR_OK;
    }
#endif

    return status;
}

car_status_t comms_poll (void)
{
#ifdef CAR_HOST_TEST
    /* The host has a broker that is always there, and delivers whatever
     * the test injected. */
    gb_connected = true;
    g_sessions   = 1u;

    if (0u != g_host_length)
    {
        car_nav_command_t command = parse_command(g_payload, g_host_length);

        g_host_length = 0u;

        if ((CAR_NAV_NONE != command) && (NULL != gp_command_handler))
        {
            gp_command_handler(command);
        }
    }
#elif COMMS_HAS_RADIO
    {
        lwip_utk_mqtt_status_t status = { 0 };
        char                   topic[COMMS_TOPIC_MAX_BYTES];
        uint16_t               length = 0u;

        lwip_utk_mqtt_get_status(&status);

        if (status.b_connected != gb_connected)
        {
            /* A session count above 1 means the link came back. */
            CAR_LOG(CAR_LOG_INFO, "mqtt %s, session %u\n",
                    status.b_connected ? "connected" : "disconnected",
                    status.sessions);
        }

        gb_connected = status.b_connected;
        g_sessions   = status.sessions;

        /* Casts: both buffers are under 65536 bytes. */
        while (0 != lwip_utk_mqtt_receive(topic, (uint16_t)sizeof(topic),
                                          g_payload,
                                          (uint16_t)sizeof(g_payload),
                                          &length))
        {
            car_nav_command_t command = parse_command(g_payload, length);

            if ((CAR_NAV_NONE != command) && (NULL != gp_command_handler))
            {
                gp_command_handler(command);
            }
        }
    }
#else
    gb_connected = false;
#endif

    return CAR_OK;
}

bool comms_is_connected (void)
{
    return gb_connected;
}

car_status_t comms_publish_telemetry (car_telemetry_t const * p_telemetry)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_telemetry)
    {
        status = send(COMMS_TOPIC_TELEMETRY, format_telemetry(p_telemetry));
    }

    return status;
}

car_status_t comms_publish_terrain (car_telemetry_t const * p_telemetry)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_telemetry)
    {
        status = send(COMMS_TOPIC_TERRAIN, format_terrain(p_telemetry));
    }

    return status;
}

car_status_t comms_set_command_handler (comms_command_handler_t p_handler)
{
    gp_command_handler  = p_handler;

    return CAR_OK;
}

car_status_t comms_publish_heartbeat (void)
{
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

    if (gb_connected && (0u != length))
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
            /* Cast: a to z minus 32 is A to Z, still a char. */
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

/* Barr C deviation 6.3.a, for the same reason as CAR_LOG: neither
 * formatter below has a va_list form to forward a function's arguments to.
 */
#ifdef CAR_HOST_TEST
#define COMMS_FORMAT(...) snprintf(g_payload, sizeof(g_payload), __VA_ARGS__)
#else
/* Barr C deviation 6.3.a, as above. The kernel's formatter has no length
 * argument. Every format below is bounded by construction: the widest,
 * telemetry with every field at its widest, is checked against
 * COMMS_PAYLOAD_MAX_BYTES by test_comms.c. */
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
    /* The last barcode as a one letter string, empty before the first. */
    char const barcode[2] = { p_telemetry->last_barcode, '\0' };

    /* Casts: each value becomes the 32 bit type its %u or %d reads, which
     * holds every value of the narrower field it comes from. */
    int32_t written = COMMS_FORMAT(
        "{\"state\":%u,\"action\":\"%s\",\"speed\":%u,\"encl\":%u,"
        "\"encr\":%u,\"mask\":%u,\"nav\":%u,\"barcode\":\"%s\","
        "\"barcodes\":%u,\"hump\":%u,\"dist\":%u,"
        "\"obst\":{\"valid\":%u,"
        "\"bearing\":%d,\"range\":%u,\"width\":%u,\"left\":%u,"
        "\"right\":%u},\"imu\":{\"pitch\":%d,\"ok\":%u,\"head\":%d,"
        "\"rate\":%d,\"event\":%u,\"hit\":%u,\"stable\":%u,"
        "\"rough\":%u}}",
        (uint32_t)p_telemetry->mission_state,
        action_name(p_telemetry->action),
        (uint32_t)p_telemetry->speed_mm_per_sec,
        (uint32_t)p_telemetry->encoder_count_left,
        (uint32_t)p_telemetry->encoder_count_right,
        (uint32_t)p_telemetry->line_sensor_mask,
        (uint32_t)p_telemetry->last_nav_command,
        barcode,
        (uint32_t)p_telemetry->barcode_count,
        (uint32_t)p_telemetry->peak_hump_height_mm,
        (uint32_t)p_telemetry->total_distance_mm,
        (uint32_t)p_telemetry->last_obstacle.b_is_valid,
        (int32_t)p_telemetry->last_obstacle.bearing_deg,
        (uint32_t)p_telemetry->last_obstacle.closest_range_mm,
        (uint32_t)p_telemetry->last_obstacle.width_mm,
        (uint32_t)p_telemetry->last_obstacle.clearance_left_mm,
        (uint32_t)p_telemetry->last_obstacle.clearance_right_mm,
        (int32_t)p_telemetry->pitch_deg,
        (uint32_t)p_telemetry->b_pitch_trusted,
        (int32_t)p_telemetry->heading_deg,
        (int32_t)p_telemetry->turn_rate_dps,
        (uint32_t)p_telemetry->motion_event,
        (uint32_t)p_telemetry->b_collision,
        (uint32_t)p_telemetry->b_terrain_stable,
        (uint32_t)p_telemetry->terrain_rough_milli_g);

    return fitted(written);
}

/**
 * @brief Write the heartbeat JSON object into g_payload.
 *
 * @return Bytes written.
 */
static uint16_t format_heartbeat (void)
{
    /* Casts: to the 32 bit type %u reads; the values fit it. */
    int32_t written = COMMS_FORMAT(
        "{\"uptime_ms\":%u,\"connected\":%u,\"sessions\":%u}",
        (uint32_t)uptime_msec(), (uint32_t)gb_connected, g_sessions);

    return fitted(written);
}

/**
 * @brief Write the terrain analysis status as a JSON object into g_payload.
 *
 * The IMU half of the snapshot, with the motion event by name and the
 * humps counted, for imu_terrain/terrain_report.md. Keys are documented
 * in comms/README.md.
 *
 * @param[in] p_telemetry Snapshot to serialise.
 *
 * @return Bytes written, 0 if it did not fit.
 */
static uint16_t format_terrain (car_telemetry_t const * p_telemetry)
{
    /* Casts: each value becomes the 32 bit type its %u or %d reads, which
     * holds every value of the narrower field it comes from. */
    int32_t written = COMMS_FORMAT(
        "{\"pitch\":%d,\"ok\":%u,\"mag\":%u,\"rough\":%u,"
        "\"terrain\":\"%s\",\"event\":\"%s\",\"on_hump\":%u,"
        "\"humps\":%u,\"last_mm\":%u,\"peak_mm\":%u}",
        (int32_t)p_telemetry->pitch_deg,
        (uint32_t)p_telemetry->b_pitch_trusted,
        (uint32_t)p_telemetry->accel_milli_g,
        (uint32_t)p_telemetry->terrain_rough_milli_g,
        p_telemetry->b_terrain_stable ? "STABLE" : "ROUGH",
        event_name(p_telemetry->motion_event),
        (uint32_t)p_telemetry->b_on_hump,
        (uint32_t)p_telemetry->hump_count,
        (uint32_t)p_telemetry->last_hump_height_mm,
        (uint32_t)p_telemetry->peak_hump_height_mm);

    return fitted(written);
}

/**
 * @brief Turn a formatter's result into a length to send.
 *
 * @param[in] written What the formatter returned.
 *
 * @return written, or 0 if it failed or did not fit g_payload.
 */
static uint16_t fitted (int32_t written)
{
    uint16_t length = 0u;

    /* Casts: the buffer is under 2^15 bytes, and a written count below
     * it fits a uint16_t. */
    if ((written > 0) && (written < (int32_t)sizeof(g_payload)))
    {
        length = (uint16_t)written;
    }

    return length;
}

/**
 * @brief Name of an action, as published.
 *
 * @param[in] action What the car is doing.
 *
 * @return A constant string.
 */
static char const * action_name (car_action_t action)
{
    return (action <= CAR_ACTION_HALTED) ? g_action_names[action]
                                         : "unknown";
}

/**
 * @brief Name of a motion event, as published.
 *
 * @param[in] event The IMU's motion class.
 *
 * @return A constant string.
 */
static char const * event_name (car_motion_event_t event)
{
    return (event <= CAR_MOTION_IMPACT) ? g_event_names[event] : "unknown";
}

#ifdef CAR_HOST_TEST

void comms_host_inject (char const * p_text, uint16_t length)
{
    uint16_t index = 0u;

    /* Casts: the buffer is under 65536 bytes. */
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
    return car_time_msec();
}

#endif /* CAR_HOST_TEST */

/*** end of file ***/
