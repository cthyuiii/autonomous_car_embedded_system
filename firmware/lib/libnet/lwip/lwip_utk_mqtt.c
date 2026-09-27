/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file lwip_utk_mqtt.c
 *
 * @brief MQTT client phase for the NO_SYS lwIP owner task, built on lwIP's
 *        own apps/mqtt raw API client.
 *
 * See lwip_utk_mqtt.h for the threading rule.
 */

#include <stdbool.h>
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

#define UTK_MQTT_QOS        0u
#define UTK_MQTT_NO_RETAIN  0u

/** One publish waiting for the owner task to send it. */
typedef struct
{
    char     topic[LWIP_UTK_MQTT_TOPIC_LEN];
    uint8_t  payload[LWIP_UTK_MQTT_TX_PAYLOAD];
    uint16_t length;
} tx_slot_t;

/** One received publish waiting for the application. */
typedef struct
{
    char     topic[LWIP_UTK_MQTT_TOPIC_LEN];
    uint8_t  payload[LWIP_UTK_MQTT_RX_PAYLOAD];
    uint16_t length;
} rx_slot_t;

static lwip_utk_mqtt_config_t g_config;
static lwip_utk_mqtt_status_t g_status;
/* Static rather than mqtt_client_new(): the client is the size of its
 * output ring, and a fixed object is easier to account for than a heap
 * block that lives forever. */
static mqtt_client_t          g_client;
static ip_addr_t              g_broker_address;
static T_SPLOCK               g_ring_lock       = 0u;

static tx_slot_t              g_tx_ring[LWIP_UTK_MQTT_TX_SLOTS];
static uint32_t               g_tx_head         = 0u;  // Fill, application
static uint32_t               g_tx_tail         = 0u;  // Send, owner
static rx_slot_t              g_rx_ring[LWIP_UTK_MQTT_RX_SLOTS];
static uint32_t               g_rx_head         = 0u;  // Fill, owner
static uint32_t               g_rx_tail         = 0u;  // Pop, application

/* The log stream: free running indices, masked into the buffer. The owner
 * task sends from the tail in chunks of whole lines. */
static char                   g_log[LWIP_UTK_MQTT_LOG_BYTES];
static uint32_t               g_log_head        = 0u;  // Fill, application
static uint32_t               g_log_tail        = 0u;  // Send, owner
static char                   g_log_chunk[LWIP_UTK_MQTT_TX_PAYLOAD];

static rx_slot_t              g_incoming;
static bool                   gb_incoming_fits  = false;
static bool                   gb_connecting     = false;
static uint32_t               g_last_attempt_ms = 0u;

static void    subscribe_cb (void * p_arg, err_t err);
static void    incoming_publish_cb (void * p_arg, char const * p_topic,
                                    u32_t tot_len);
static void    incoming_data_cb (void * p_arg, u8_t const * p_data,
                                 u16_t len, u8_t flags);
static void    queue_incoming (void);
static void    connection_cb (mqtt_client_t * p_client, void * p_arg,
                              mqtt_connection_status_t status);
static int32_t queue_publish (char const * p_topic, void const * p_payload,
                              uint16_t length);
static bool    link_is_up (struct netif * p_netif);
static void    drop_connection (void);
static void    keep_connected (uint32_t now);
static void    start_connect (uint32_t now);
static void    drain_publish_ring (void);
static void    finish_slot (err_t err);
static void    drain_log (void);

void lwip_utk_mqtt_configure (lwip_utk_mqtt_config_t const * p_config)
{
    if (NULL != p_config)
    {
        g_config = *p_config;
        g_config.host[LWIP_UTK_MQTT_HOST_LEN - 1u]              = '\0';
        g_config.client_id[LWIP_UTK_MQTT_ID_LEN - 1u]           = '\0';
        g_config.subscribe_topic[LWIP_UTK_MQTT_TOPIC_LEN - 1u]  = '\0';
        g_config.log_topic[LWIP_UTK_MQTT_TOPIC_LEN - 1u]        = '\0';
        g_status.b_configured = (0 != ipaddr_aton(g_config.host,
                                                  &g_broker_address));
        g_status.b_connected  = false;
        g_status.b_subscribed = false;
        gb_connecting         = false;
        g_last_attempt_ms     = 0u;
    }
}

int32_t lwip_utk_mqtt_publish (char const * p_topic, void const * p_payload,
                               uint16_t length)
{
    int32_t result = LWIP_UTK_MQTT_QUEUED;

    if ((NULL == p_topic) || ((NULL == p_payload) && (0u != length))
        || (length > LWIP_UTK_MQTT_TX_PAYLOAD))
    {
        result = LWIP_UTK_MQTT_TOO_LONG;
    }
    else if (!g_status.b_connected)
    {
        result = LWIP_UTK_MQTT_NOT_CONNECTED;
    }
    else
    {
        result = queue_publish(p_topic, p_payload, length);
    }

    return result;
}

void lwip_utk_mqtt_log (char const * p_text, uint16_t length)
{
    uint32_t saved = 0u;
    uint32_t index = 0u;

    if ((NULL != p_text) && (0u != length))
    {
        (void)ISpinLock(&g_ring_lock, &saved);

        /* Whole lines or nothing, so the stream never holds half a line. */
        if ((LWIP_UTK_MQTT_LOG_BYTES - (g_log_head - g_log_tail)) >= length)
        {
            for (index = 0u; index < length; index++)
            {
                g_log[(g_log_head + index) & (LWIP_UTK_MQTT_LOG_BYTES - 1u)]
                    = p_text[index];
            }

            g_log_head += length;
        }
        else
        {
            g_status.log_lines_lost++;
        }

        (void)ISpinUnlock(&g_ring_lock, saved);
    }
}

int32_t lwip_utk_mqtt_receive (char * p_topic, uint16_t topic_size,
                               void * p_payload, uint16_t payload_size,
                               uint16_t * p_length)
{
    int32_t  got   = 0;
    uint32_t saved = 0u;

    if ((NULL != p_topic) && (NULL != p_payload) && (NULL != p_length))
    {
        (void)ISpinLock(&g_ring_lock, &saved);

        if (g_rx_tail != g_rx_head)
        {
            rx_slot_t const * p_slot = &g_rx_ring[g_rx_tail];
            uint16_t          length = (p_slot->length > payload_size)
                                       ? payload_size : p_slot->length;

            (void)strlcpy(p_topic, p_slot->topic, topic_size);
            (void)memcpy(p_payload, p_slot->payload, length);
            *p_length = length;
            g_rx_tail = (g_rx_tail + 1u) % LWIP_UTK_MQTT_RX_SLOTS;
            got       = 1;
        }

        (void)ISpinUnlock(&g_ring_lock, saved);
    }

    return got;
}

void lwip_utk_mqtt_poll (struct netif * p_netif)
{
    if ((!g_status.b_configured) || (NULL == p_netif))
    {
        /* Nothing to connect to yet. */
    }
    else if (!link_is_up(p_netif))
    {
        drop_connection();
    }
    else
    {
        keep_connected(sys_now());
    }
}

void lwip_utk_mqtt_get_status (lwip_utk_mqtt_status_t * p_status)
{
    uint32_t saved = 0u;

    if (NULL != p_status)
    {
        (void)ISpinLock(&g_ring_lock, &saved);
        *p_status = g_status;
        (void)ISpinUnlock(&g_ring_lock, saved);
    }
}

/**
 * @brief lwIP callback: the broker answered the subscribe request.
 *
 * @param[in] p_arg Unused.
 * @param[in] err   ERR_OK if the subscription was accepted.
 */
static void subscribe_cb (void * p_arg, err_t err)
{
    (void)p_arg;
    g_status.b_subscribed = (ERR_OK == err);
}

/**
 * @brief lwIP callback: a publish has started arriving.
 *
 * @param[in] p_arg   Unused.
 * @param[in] p_topic Its topic.
 * @param[in] tot_len Its whole payload length.
 */
static void incoming_publish_cb (void * p_arg, char const * p_topic,
                                 u32_t tot_len)
{
    (void)p_arg;
    (void)strlcpy(g_incoming.topic, p_topic, sizeof(g_incoming.topic));
    g_incoming.length = 0u;
    gb_incoming_fits  = (tot_len <= LWIP_UTK_MQTT_RX_PAYLOAD);
}

/**
 * @brief lwIP callback: the next part of the publish's payload arrived.
 *
 * @param[in] p_arg  Unused.
 * @param[in] p_data This part.
 * @param[in] len    Its length.
 * @param[in] flags  MQTT_DATA_FLAG_LAST on the final part.
 */
static void incoming_data_cb (void * p_arg, u8_t const * p_data,
                              u16_t len, u8_t flags)
{
    (void)p_arg;

    /* Casts: widening the length before adding, then narrowing a sum the
     * test has just held to LWIP_UTK_MQTT_RX_PAYLOAD. */
    if (gb_incoming_fits
        && (((uint32_t)g_incoming.length + len) > LWIP_UTK_MQTT_RX_PAYLOAD))
    {
        gb_incoming_fits = false;
    }
    else if (gb_incoming_fits)
    {
        (void)memcpy(&g_incoming.payload[g_incoming.length], p_data, len);
        g_incoming.length = (uint16_t)(g_incoming.length + len);
    }
    else
    {
        /* Already too long for a slot: the rest of it is dropped. */
    }

    if ((0 != (flags & MQTT_DATA_FLAG_LAST)) && gb_incoming_fits)
    {
        queue_incoming();
    }
}

/**
 * @brief Hand a complete received publish to the application's ring.
 */
static void queue_incoming (void)
{
    uint32_t saved = 0u;
    uint32_t next  = 0u;

    (void)ISpinLock(&g_ring_lock, &saved);
    next = (g_rx_head + 1u) % LWIP_UTK_MQTT_RX_SLOTS;

    if (next != g_rx_tail)
    {
        g_rx_ring[g_rx_head] = g_incoming;
        g_rx_head            = next;
        g_status.received++;
    }
    else
    {
        g_status.dropped++;
    }

    (void)ISpinUnlock(&g_ring_lock, saved);
}

/**
 * @brief lwIP callback: the broker accepted or refused the connection.
 *
 * @param[in] p_client The client.
 * @param[in] p_arg    Unused.
 * @param[in] status   MQTT_CONNECT_ACCEPTED, or why not.
 */
static void connection_cb (mqtt_client_t * p_client, void * p_arg,
                           mqtt_connection_status_t status)
{
    (void)p_arg;
    gb_connecting           = false;
    g_status.connect_result = (int32_t)status;   /* A small enumerator */

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_status.b_connected = true;
        g_status.sessions++;
        /* Set after connect: the client is wiped on every connect call. */
        mqtt_set_inpub_callback(p_client, incoming_publish_cb,
                                incoming_data_cb, NULL);

        if ('\0' != g_config.subscribe_topic[0])
        {
            (void)mqtt_subscribe(p_client, g_config.subscribe_topic,
                                 UTK_MQTT_QOS, subscribe_cb, NULL);
        }
    }
    else
    {
        g_status.b_connected  = false;
        g_status.b_subscribed = false;
    }
}

/**
 * @brief Copy one publish into the next free ring slot.
 *
 * @param[in] p_topic   Topic.
 * @param[in] p_payload Payload.
 * @param[in] length    Payload length, already checked.
 *
 * @return LWIP_UTK_MQTT_QUEUED, or LWIP_UTK_MQTT_FULL.
 */
static int32_t queue_publish (char const * p_topic, void const * p_payload,
                              uint16_t length)
{
    int32_t  result = LWIP_UTK_MQTT_QUEUED;
    uint32_t saved  = 0u;
    uint32_t next   = 0u;

    (void)ISpinLock(&g_ring_lock, &saved);
    next = (g_tx_head + 1u) % LWIP_UTK_MQTT_TX_SLOTS;

    if (next == g_tx_tail)
    {
        result = LWIP_UTK_MQTT_FULL;
        g_status.dropped++;
    }
    else
    {
        tx_slot_t * p_slot = &g_tx_ring[g_tx_head];

        (void)strlcpy(p_slot->topic, p_topic, sizeof(p_slot->topic));

        if (0u != length)
        {
            (void)memcpy(p_slot->payload, p_payload, length);
        }

        p_slot->length = length;
        g_tx_head      = next;
    }

    (void)ISpinUnlock(&g_ring_lock, saved);

    return result;
}

/**
 * @brief Whether the station interface can carry a connection.
 *
 * @param[in] p_netif The station interface.
 *
 * @return true when it is up, linked and has an address.
 */
static bool link_is_up (struct netif * p_netif)
{
    return (0u != netif_is_up(p_netif))
           && (0u != netif_is_link_up(p_netif))
           && (!ip4_addr_isany_val(*netif_ip4_addr(p_netif)));
}

/**
 * @brief The link went: forget the session, it cannot have survived.
 */
static void drop_connection (void)
{
    if (g_status.b_connected || gb_connecting)
    {
        mqtt_disconnect(&g_client);
    }

    g_status.b_connected  = false;
    g_status.b_subscribed = false;
    gb_connecting         = false;
}

/**
 * @brief With the link up: notice a lost session, reconnect when due, and
 *        send what is queued.
 *
 * @param[in] now sys_now() for this poll.
 */
static void keep_connected (uint32_t now)
{
    if (g_status.b_connected && (0u == mqtt_client_is_connected(&g_client)))
    {
        g_status.b_connected  = false;
        g_status.b_subscribed = false;
    }

    if ((!g_status.b_connected) && (!gb_connecting)
        && ((0u == g_status.connect_attempts)
            || ((now - g_last_attempt_ms) >= g_config.reconnect_ms)))
    {
        start_connect(now);
    }

    if (g_status.b_connected)
    {
        drain_publish_ring();
        drain_log();
    }
}

/**
 * @brief Ask lwIP to open a session with the broker.
 *
 * @param[in] now sys_now() for this poll, the start of the backoff.
 */
static void start_connect (uint32_t now)
{
    struct mqtt_connect_client_info_t info = { 0 };
    err_t                             err  = ERR_OK;

    info.client_id  = g_config.client_id;
    info.keep_alive = g_config.keepalive_seconds;

    g_last_attempt_ms = now;
    g_status.connect_attempts++;
    err = mqtt_client_connect(&g_client, &g_broker_address, g_config.port,
                              connection_cb, NULL, &info);
    g_status.connect_result = err;
    gb_connecting           = (ERR_OK == err);
}

/**
 * @brief Send queued publishes until the ring is empty or lwIP is full.
 */
static void drain_publish_ring (void)
{
    bool b_more = true;

    while (b_more)
    {
        tx_slot_t * p_slot = NULL;
        uint32_t    saved  = 0u;
        err_t       err    = ERR_OK;

        (void)ISpinLock(&g_ring_lock, &saved);

        if (g_tx_tail != g_tx_head)
        {
            p_slot = &g_tx_ring[g_tx_tail];
        }

        (void)ISpinUnlock(&g_ring_lock, saved);

        if (NULL != p_slot)
        {
            err = mqtt_publish(&g_client, p_slot->topic, p_slot->payload,
                               p_slot->length, UTK_MQTT_QOS,
                               UTK_MQTT_NO_RETAIN, NULL, NULL);
        }

        if ((NULL == p_slot) || (ERR_MEM == err))
        {
            /* Nothing left, or lwIP's output ring is full, in which case
             * the slot waits for the next poll. */
            b_more = false;
        }
        else
        {
            finish_slot(err);
        }
    }
}

/**
 * @brief Count the oldest slot as sent or dropped, and free it.
 *
 * @param[in] err What lwIP said about it.
 */
static void finish_slot (err_t err)
{
    uint32_t saved = 0u;

    if (ERR_OK == err)
    {
        g_status.published++;
    }
    else
    {
        g_status.dropped++;
    }

    (void)ISpinLock(&g_ring_lock, &saved);
    g_tx_tail = (g_tx_tail + 1u) % LWIP_UTK_MQTT_TX_SLOTS;
    (void)ISpinUnlock(&g_ring_lock, saved);
}

/**
 * @brief Send the oldest whole lines of the log stream as one message.
 *
 * Takes as many whole lines as fit one message and leaves off the last
 * newline, which a subscriber prints anyway. A single line longer than a
 * message goes as it is.
 */
static void drain_log (void)
{
    uint32_t saved = 0u;
    uint32_t held  = 0u;
    uint32_t take  = 0u;
    uint32_t cut   = 0u;
    uint32_t index = 0u;

    (void)ISpinLock(&g_ring_lock, &saved);
    held = g_log_head - g_log_tail;
    (void)ISpinUnlock(&g_ring_lock, saved);

    take = (held < LWIP_UTK_MQTT_TX_PAYLOAD) ? held
                                             : LWIP_UTK_MQTT_TX_PAYLOAD;

    /* Only the owner task moves the tail, so the text read here is not
     * overwritten while it is copied out. */
    for (index = 0u; index < take; index++)
    {
        g_log_chunk[index] =
            g_log[(g_log_tail + index) & (LWIP_UTK_MQTT_LOG_BYTES - 1u)];

        if ('\n' == g_log_chunk[index])
        {
            cut = index + 1u;
        }
    }

    if (0u == cut)
    {
        cut = (LWIP_UTK_MQTT_TX_PAYLOAD == take) ? take : 0u;
    }

    if ((0u != cut) && ('\0' != g_config.log_topic[0]))
    {
        /* Cast: at most LWIP_UTK_MQTT_TX_PAYLOAD, well inside u16_t. */
        u16_t length = (u16_t)(('\n' == g_log_chunk[cut - 1u]) ? (cut - 1u)
                                                               : cut);
        err_t err    = mqtt_publish(&g_client, g_config.log_topic,
                                    g_log_chunk, length, UTK_MQTT_QOS,
                                    UTK_MQTT_NO_RETAIN, NULL, NULL);

        /* Anything but a send stays for the next poll: a full output
         * ring or a connection that dropped under us are both passing,
         * and a chunk always fits the ring once it is empty. */
        if (ERR_OK == err)
        {
            (void)ISpinLock(&g_ring_lock, &saved);
            g_log_tail += cut;
            (void)ISpinUnlock(&g_ring_lock, saved);
        }
    }
}

/*** end of file ***/
