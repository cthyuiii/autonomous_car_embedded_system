/** @file comms.h
 *
 * @brief WiFi bring up, MQTT publish and subscribe, heartbeat and recovery.
 *
 * All calls are non blocking. Call comms_poll() from the main loop; it owns
 * reconnection, so the rest of the firmware never waits on the network.
 * Topic names and timeouts are the COMMS_* constants in car_config.h.
 *
 * Owner: Buddy 1, WiFi communication, command and telemetry. A changed
 * signature here also changes the test, the bench and car_main.c, so agree it
 * with the team first.
 */

#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>

#include "car.h"

/** Called from comms_poll() when a command arrives on COMMS_TOPIC_COMMAND. */
typedef void (*comms_command_handler_t)(car_nav_command_t command);

/** Something that happened, sent once on COMMS_TOPIC_EVENT. */
typedef enum
{
    COMMS_EVENT_ACTION = 0,     /* The action changed */
    COMMS_EVENT_BARCODE,        /* A barcode was decoded */
    COMMS_EVENT_HUMP,           /* A hump was crossed and measured */
    COMMS_EVENT_HIT,            /* A collision or a stalled wheel */
    COMMS_EVENT_OBSTACLE        /* An obstacle scan finished */
} comms_event_t;

/**
 * @brief Start WiFi association and the MQTT client without blocking.
 *
 * WARNING: Credentials come from COMMS_WIFI_SSID and COMMS_WIFI_PASSWORD,
 * which are placeholders. Real values must never be committed.
 *
 * @return CAR_OK once the radio is initialised. Connection completes later.
 */
car_status_t comms_init (void);

/**
 * @brief Service the network stack and reconnect after any drop.
 *
 * NOTE: Backs off by COMMS_RECONNECT_BACKOFF_MSEC between attempts so a
 * dead broker does not starve the control loop.
 *
 * @return CAR_OK, or CAR_ERR_HARDWARE if the radio itself has failed.
 */
car_status_t comms_poll (void);

/**
 * @brief Report whether the MQTT session is currently up.
 *
 * @return true only when both WiFi and the broker session are connected.
 */
bool comms_is_connected (void);

/**
 * @brief Publish one telemetry snapshot on COMMS_TOPIC_TELEMETRY.
 *
 * @param[in] p_telemetry Snapshot to serialise, must not be NULL.
 *
 * @return CAR_OK if queued, CAR_ERR_TIMEOUT if not connected.
 */
car_status_t comms_publish_telemetry (car_telemetry_t const * p_telemetry);

/**
 * @brief Publish the terrain analysis status on COMMS_TOPIC_TERRAIN.
 *
 * The IMU half of the snapshot, with the motion event by name and the
 * humps counted, for imu_terrain/terrain_report.md.
 *
 * @param[in] p_telemetry Snapshot to serialise, must not be NULL.
 *
 * @return CAR_OK if queued, CAR_ERR_TIMEOUT if not connected, or
 *         CAR_ERR_RANGE if p_telemetry is NULL.
 */
car_status_t comms_publish_terrain (car_telemetry_t const * p_telemetry);

/**
 * @brief Publish one event on COMMS_TOPIC_EVENT, with the fields of the
 *        snapshot that belong to it.
 *
 * @param[in] p_telemetry Snapshot taken after the event, must not be NULL.
 * @param[in] event       What happened.
 *
 * @return CAR_OK if queued, CAR_ERR_TIMEOUT if not connected, or
 *         CAR_ERR_RANGE if p_telemetry is NULL or the event is unknown.
 */
car_status_t comms_publish_event (car_telemetry_t const * p_telemetry,
                                  comms_event_t event);

/**
 * @brief Register the function that receives remote navigation commands.
 *
 * @param[in] p_handler Callback, or NULL to stop receiving commands.
 *
 * @return CAR_OK.
 */
car_status_t comms_set_command_handler (comms_command_handler_t p_handler);

/**
 * @brief Publish a liveness message on COMMS_TOPIC_HEARTBEAT.
 *
 * @return CAR_OK if queued, CAR_ERR_TIMEOUT if not connected.
 */
car_status_t comms_publish_heartbeat (void);

#ifdef CAR_HOST_TEST
/**
 * @brief Host test hook: a payload the next comms_poll() receives.
 *
 * @param[in] p_text Payload bytes, need not be terminated.
 * @param[in] length Payload length.
 */
void comms_host_inject (char const * p_text, uint16_t length);
#endif

#endif /* COMMS_H */

/*** end of file ***/

