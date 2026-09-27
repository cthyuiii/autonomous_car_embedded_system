/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file lwip_utk_mqtt.h
 *
 * @brief MQTT client phase for the NO_SYS lwIP owner task.
 *
 * lwIP and this client run only on the radio owner task. Application tasks
 * never call lwIP; they hand messages across two small rings guarded by the
 * port's spinlock, and the owner task drains them from lwip_utk_mqtt_poll().
 * Received publishes travel the other way through the second ring.
 */

#ifndef LWIP_UTK_MQTT_H
#define LWIP_UTK_MQTT_H

#include <stdbool.h>
#include <stdint.h>

#define LWIP_UTK_MQTT_HOST_LEN      16u   /* Dotted quad plus terminator */
#define LWIP_UTK_MQTT_ID_LEN        24u
#define LWIP_UTK_MQTT_TOPIC_LEN     32u
#define LWIP_UTK_MQTT_TX_PAYLOAD   480u   /* COMMS_PAYLOAD_MAX_BYTES */
#define LWIP_UTK_MQTT_RX_PAYLOAD    64u
#define LWIP_UTK_MQTT_TX_SLOTS       8u   /* Room for a burst of log lines */
#define LWIP_UTK_MQTT_RX_SLOTS       4u
#define LWIP_UTK_MQTT_LOG_BYTES   8192u   /* Log held while offline, 2^n */

/* lwip_utk_mqtt_publish() results. */
#define LWIP_UTK_MQTT_QUEUED         0
#define LWIP_UTK_MQTT_NOT_CONNECTED (-1)
#define LWIP_UTK_MQTT_FULL          (-2)
#define LWIP_UTK_MQTT_TOO_LONG      (-3)

struct netif;

/** Broker and session settings, copied in by lwip_utk_mqtt_configure(). */
typedef struct
{
    char     host[LWIP_UTK_MQTT_HOST_LEN];
    uint16_t port;
    char     client_id[LWIP_UTK_MQTT_ID_LEN];
    char     subscribe_topic[LWIP_UTK_MQTT_TOPIC_LEN];
    char     log_topic[LWIP_UTK_MQTT_TOPIC_LEN];   /* "" for no log */
    uint16_t keepalive_seconds;
    uint32_t reconnect_ms;
} lwip_utk_mqtt_config_t;

/** What the client has done so far, for any task to read. */
typedef struct
{
    bool     b_configured;
    bool     b_connected;
    bool     b_subscribed;
    int32_t  connect_result;
    uint32_t connect_attempts;
    uint32_t sessions;              /* Connects the broker accepted */
    uint32_t published;
    uint32_t dropped;
    uint32_t received;
    uint32_t log_lines_lost;        /* Log lines that found it full */
} lwip_utk_mqtt_status_t;

/**
 * @brief Set the broker and session. Any task, before the first poll.
 *
 * A host that does not parse as a dotted quad leaves the client
 * unconfigured, which the status shows.
 *
 * @param[in] p_config Settings to copy.
 */
void lwip_utk_mqtt_configure (lwip_utk_mqtt_config_t const * p_config);

/**
 * @brief Queue one publish. Any task.
 *
 * Copies the payload into a ring slot; the owner task sends it on its next
 * poll.
 *
 * @param[in] p_topic   Topic, NUL terminated.
 * @param[in] p_payload Payload bytes, may be NULL when length is 0.
 * @param[in] length    Payload length.
 *
 * @return One of the LWIP_UTK_MQTT_* results.
 */
int32_t lwip_utk_mqtt_publish (char const * p_topic, void const * p_payload,
                               uint16_t length);

/**
 * @brief Add text to the log stream. Any task.
 *
 * Unlike a publish, the text is kept while there is no connection and sent
 * in order once there is one, several lines to a message, so an outage
 * delays the log instead of cutting a hole in it. A line that does not fit
 * in the LWIP_UTK_MQTT_LOG_BYTES held is lost whole, and counted.
 *
 * @param[in] p_text Text, usually one line ending in a newline.
 * @param[in] length Bytes of p_text.
 */
void lwip_utk_mqtt_log (char const * p_text, uint16_t length);

/**
 * @brief Pop one received publish, oldest first. Any task.
 *
 * @param[out] p_topic      Topic, truncated to topic_size.
 * @param[in]  topic_size   Room at p_topic.
 * @param[out] p_payload    Payload, truncated to payload_size.
 * @param[in]  payload_size Room at p_payload.
 * @param[out] p_length     Bytes copied to p_payload.
 *
 * @return 1 when a message was copied out, 0 when the ring is empty.
 */
int32_t lwip_utk_mqtt_receive (char * p_topic, uint16_t topic_size,
                               void * p_payload, uint16_t payload_size,
                               uint16_t * p_length);

/**
 * @brief Connect, reconnect and send what is queued. Owner task only.
 *
 * @param[in] p_netif The station interface.
 */
void lwip_utk_mqtt_poll (struct netif * p_netif);

/**
 * @brief Copy out the client's status. Any task.
 *
 * @param[out] p_status Where to copy it.
 */
void lwip_utk_mqtt_get_status (lwip_utk_mqtt_status_t * p_status);

#endif /* LWIP_UTK_MQTT_H */

/*** end of file ***/
