/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* MQTT client phase for the NO_SYS lwIP owner task.
 *
 * lwIP and this client run only on the radio-owner task.  Application tasks
 * never call lwIP; they hand messages across two small rings guarded by the
 * port's spinlock, and the owner task drains them from lwip_utk_mqtt_poll().
 * Received publishes travel the other way through the second ring. */

#ifndef LWIP_UTK_MQTT_H
#define LWIP_UTK_MQTT_H

#include <stdint.h>

#define LWIP_UTK_MQTT_HOST_LEN      16U   /* dotted quad plus terminator */
#define LWIP_UTK_MQTT_ID_LEN        24U
#define LWIP_UTK_MQTT_TOPIC_LEN     32U
#define LWIP_UTK_MQTT_TX_PAYLOAD   256U
#define LWIP_UTK_MQTT_RX_PAYLOAD    64U
#define LWIP_UTK_MQTT_TX_SLOTS       4U
#define LWIP_UTK_MQTT_RX_SLOTS       4U

/* lwip_utk_mqtt_publish() results. */
#define LWIP_UTK_MQTT_QUEUED         0
#define LWIP_UTK_MQTT_NOT_CONNECTED (-1)
#define LWIP_UTK_MQTT_FULL          (-2)
#define LWIP_UTK_MQTT_TOO_LONG      (-3)

struct netif;

typedef struct {
    char host[LWIP_UTK_MQTT_HOST_LEN];
    uint16_t port;
    char client_id[LWIP_UTK_MQTT_ID_LEN];
    char subscribe_topic[LWIP_UTK_MQTT_TOPIC_LEN];
    uint16_t keepalive_seconds;
    uint32_t reconnect_ms;
} T_LWIP_UTK_MQTT_CONFIG;

typedef struct {
    uint32_t configured;
    uint32_t connected;
    uint32_t subscribed;
    int32_t connect_result;
    uint32_t connect_attempts;
    uint32_t published;
    uint32_t dropped;
    uint32_t received;
} T_LWIP_UTK_MQTT_STATUS;

/* Any task, before the first poll.  A host that does not parse as a dotted
   quad leaves the client unconfigured, visible in the status. */
void lwip_utk_mqtt_configure(const T_LWIP_UTK_MQTT_CONFIG *config);

/* Any task.  Copies the payload into a ring slot; the owner task sends it
   on its next poll.  Returns one of the LWIP_UTK_MQTT_* results. */
int32_t lwip_utk_mqtt_publish(const char *topic, const void *payload,
                              uint16_t length);

/* Any task.  Pops one received publish, oldest first.  Returns 1 when a
   message was copied out, 0 when the ring is empty. */
int32_t lwip_utk_mqtt_receive(char *topic, uint16_t topic_size, void *payload,
                              uint16_t payload_size, uint16_t *length);

/* Owner task only. */
void lwip_utk_mqtt_poll(struct netif *netif);

/* Any task. */
void lwip_utk_mqtt_get_status(T_LWIP_UTK_MQTT_STATUS *status);

#endif
