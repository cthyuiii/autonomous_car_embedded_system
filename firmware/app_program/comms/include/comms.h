/** @file comms.h
 *
 * @brief WiFi bring up, MQTT publish and subscribe, heartbeat and recovery.
 *
 * All calls are non blocking. Call comms_poll() from the main loop; it owns
 * reconnection, so the rest of the firmware never waits on the network.
 * Topic names and timeouts are the COMMS_* constants in car_config.h.
 */

#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>

#include "car_types.h"

/** Called from comms_poll() when a command arrives on COMMS_TOPIC_COMMAND. */
typedef void (*comms_command_handler_t)(car_nav_command_t command);

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

#endif /* COMMS_H */

/*** end of file ***/

