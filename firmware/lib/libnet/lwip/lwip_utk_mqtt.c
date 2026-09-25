/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* MQTT client phase for the NO_SYS lwIP owner task, built on lwIP's own
 * apps/mqtt raw API client.  See lwip_utk_mqtt.h for the threading rule. */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

#include "lwip_utk_mqtt.h"
#include "smp_compat.h"

#define UTK_MQTT_QOS        0U
#define UTK_MQTT_NO_RETAIN  0U

typedef struct {
    char topic[LWIP_UTK_MQTT_TOPIC_LEN];
    uint8_t payload[LWIP_UTK_MQTT_TX_PAYLOAD];
    uint16_t length;
} tx_slot_t;

typedef struct {
    char topic[LWIP_UTK_MQTT_TOPIC_LEN];
    uint8_t payload[LWIP_UTK_MQTT_RX_PAYLOAD];
    uint16_t length;
} rx_slot_t;

static T_LWIP_UTK_MQTT_CONFIG mqtt_config;
static T_LWIP_UTK_MQTT_STATUS mqtt_status;
/* Static rather than mqtt_client_new(): the client is the size of its
   output ring, and a fixed object is easier to account for than a heap
   block that lives forever. */
static mqtt_client_t mqtt_client;
static ip_addr_t broker_address;
static T_SPLOCK ring_lock;

static tx_slot_t tx_ring[LWIP_UTK_MQTT_TX_SLOTS];
static uint32_t tx_head;   /* next slot to fill, application side */
static uint32_t tx_tail;   /* next slot to send, owner side */
static rx_slot_t rx_ring[LWIP_UTK_MQTT_RX_SLOTS];
static uint32_t rx_head;   /* next slot to fill, owner side */
static uint32_t rx_tail;   /* next slot to pop, application side */

static rx_slot_t incoming;
static uint32_t incoming_fits;
static uint32_t connecting;
static uint32_t last_attempt_ms;

static void subscribe_cb(void *arg, err_t err)
{
    (void)arg;
    mqtt_status.subscribed = (err == ERR_OK);
}

static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    (void)arg;
    strlcpy(incoming.topic, topic, sizeof(incoming.topic));
    incoming.length = 0;
    incoming_fits = (tot_len <= LWIP_UTK_MQTT_RX_PAYLOAD);
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags)
{
    (void)arg;
    if(incoming_fits) {
        if((uint32_t)incoming.length + len <= LWIP_UTK_MQTT_RX_PAYLOAD) {
            memcpy(&incoming.payload[incoming.length], data, len);
            incoming.length = (uint16_t)(incoming.length + len);
        } else {
            incoming_fits = 0;
        }
    }

    if((flags & MQTT_DATA_FLAG_LAST) != 0 && incoming_fits) {
        uint32_t saved;
        uint32_t next;

        (void)ISpinLock(&ring_lock, &saved);
        next = (rx_head + 1U) % LWIP_UTK_MQTT_RX_SLOTS;
        if(next != rx_tail) {
            rx_ring[rx_head] = incoming;
            rx_head = next;
            mqtt_status.received++;
        } else {
            mqtt_status.dropped++;
        }
        (void)ISpinUnlock(&ring_lock, saved);
    }
}

static void connection_cb(mqtt_client_t *client, void *arg,
                          mqtt_connection_status_t status)
{
    (void)arg;
    connecting = 0;
    mqtt_status.connect_result = (int32_t)status;
    if(status == MQTT_CONNECT_ACCEPTED) {
        mqtt_status.connected = 1;
        /* Set after connect: the client is wiped on every connect call. */
        mqtt_set_inpub_callback(client, incoming_publish_cb,
                                incoming_data_cb, NULL);
        if(mqtt_config.subscribe_topic[0] != '\0') {
            (void)mqtt_subscribe(client, mqtt_config.subscribe_topic,
                                 UTK_MQTT_QOS, subscribe_cb, NULL);
        }
    } else {
        mqtt_status.connected = 0;
        mqtt_status.subscribed = 0;
    }
}

void lwip_utk_mqtt_configure(const T_LWIP_UTK_MQTT_CONFIG *config)
{
    if(config == NULL) return;
    mqtt_config = *config;
    mqtt_config.host[LWIP_UTK_MQTT_HOST_LEN - 1U] = '\0';
    mqtt_config.client_id[LWIP_UTK_MQTT_ID_LEN - 1U] = '\0';
    mqtt_config.subscribe_topic[LWIP_UTK_MQTT_TOPIC_LEN - 1U] = '\0';
    mqtt_status.configured = ipaddr_aton(mqtt_config.host, &broker_address);
    mqtt_status.connected = 0;
    mqtt_status.subscribed = 0;
    connecting = 0;
    last_attempt_ms = 0;
}

int32_t lwip_utk_mqtt_publish(const char *topic, const void *payload,
                              uint16_t length)
{
    uint32_t saved;
    uint32_t next;
    int32_t result = LWIP_UTK_MQTT_QUEUED;

    if(topic == NULL || (payload == NULL && length != 0)) {
        return LWIP_UTK_MQTT_TOO_LONG;
    }
    if(length > LWIP_UTK_MQTT_TX_PAYLOAD) return LWIP_UTK_MQTT_TOO_LONG;
    if(!mqtt_status.connected) return LWIP_UTK_MQTT_NOT_CONNECTED;

    (void)ISpinLock(&ring_lock, &saved);
    next = (tx_head + 1U) % LWIP_UTK_MQTT_TX_SLOTS;
    if(next == tx_tail) {
        result = LWIP_UTK_MQTT_FULL;
        mqtt_status.dropped++;
    } else {
        tx_slot_t *slot = &tx_ring[tx_head];

        strlcpy(slot->topic, topic, sizeof(slot->topic));
        if(length != 0) memcpy(slot->payload, payload, length);
        slot->length = length;
        tx_head = next;
    }
    (void)ISpinUnlock(&ring_lock, saved);

    return result;
}

int32_t lwip_utk_mqtt_receive(char *topic, uint16_t topic_size, void *payload,
                              uint16_t payload_size, uint16_t *length)
{
    uint32_t saved;
    int32_t got = 0;

    if(topic == NULL || payload == NULL || length == NULL) return 0;

    (void)ISpinLock(&ring_lock, &saved);
    if(rx_tail != rx_head) {
        rx_slot_t *slot = &rx_ring[rx_tail];
        uint16_t n = slot->length;

        if(n > payload_size) n = payload_size;
        strlcpy(topic, slot->topic, topic_size);
        memcpy(payload, slot->payload, n);
        *length = n;
        rx_tail = (rx_tail + 1U) % LWIP_UTK_MQTT_RX_SLOTS;
        got = 1;
    }
    (void)ISpinUnlock(&ring_lock, saved);

    return got;
}

static void drain_publish_ring(void)
{
    uint32_t saved;
    int keep_going = 1;

    while(keep_going) {
        tx_slot_t *slot = NULL;
        err_t err;

        (void)ISpinLock(&ring_lock, &saved);
        if(tx_tail != tx_head) slot = &tx_ring[tx_tail];
        (void)ISpinUnlock(&ring_lock, saved);
        if(slot == NULL) break;

        err = mqtt_publish(&mqtt_client, slot->topic, slot->payload,
                           slot->length, UTK_MQTT_QOS, UTK_MQTT_NO_RETAIN,
                           NULL, NULL);
        if(err == ERR_MEM) {
            /* Output ring full: leave the slot for the next poll. */
            keep_going = 0;
        } else {
            if(err == ERR_OK) mqtt_status.published++;
            else mqtt_status.dropped++;
            (void)ISpinLock(&ring_lock, &saved);
            tx_tail = (tx_tail + 1U) % LWIP_UTK_MQTT_TX_SLOTS;
            (void)ISpinUnlock(&ring_lock, saved);
        }
    }
}

void lwip_utk_mqtt_poll(struct netif *netif)
{
    uint32_t now = sys_now();
    int link_ok;

    if(!mqtt_status.configured || netif == NULL) return;

    link_ok = netif_is_up(netif) && netif_is_link_up(netif)
           && !ip4_addr_isany_val(*netif_ip4_addr(netif));

    if(!link_ok) {
        if(mqtt_status.connected || connecting) {
            mqtt_disconnect(&mqtt_client);
        }
        mqtt_status.connected = 0;
        mqtt_status.subscribed = 0;
        connecting = 0;
        return;
    }

    if(mqtt_status.connected && !mqtt_client_is_connected(&mqtt_client)) {
        mqtt_status.connected = 0;
        mqtt_status.subscribed = 0;
    }

    if(!mqtt_status.connected && !connecting
       && (mqtt_status.connect_attempts == 0
           || now - last_attempt_ms >= mqtt_config.reconnect_ms)) {
        struct mqtt_connect_client_info_t info;
        err_t err;

        memset(&info, 0, sizeof(info));
        info.client_id = mqtt_config.client_id;
        info.keep_alive = mqtt_config.keepalive_seconds;

        last_attempt_ms = now;
        mqtt_status.connect_attempts++;
        err = mqtt_client_connect(&mqtt_client, &broker_address,
                                  mqtt_config.port, connection_cb, NULL,
                                  &info);
        mqtt_status.connect_result = err;
        connecting = (err == ERR_OK);
    }

    if(mqtt_status.connected) drain_publish_ring();
}

void lwip_utk_mqtt_get_status(T_LWIP_UTK_MQTT_STATUS *status)
{
    uint32_t saved;

    if(status == NULL) return;
    (void)ISpinLock(&ring_lock, &saved);
    *status = mqtt_status;
    (void)ISpinUnlock(&ring_lock, saved);
}
