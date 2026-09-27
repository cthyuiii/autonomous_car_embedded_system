/** @file car_log.c
 *
 * @brief Mirrors every console line to MQTT in the radio builds, so a car
 *        off its USB cable can still be followed.
 *
 * Build the car or any bench with --wifi, then record the run from a
 * laptop:
 *
 *     mosquitto_sub -h <broker ip> -t car/log | tee run.txt
 *
 * NOTE: Nothing waits for the radio. Lines are held while Wi-Fi joins or
 * the broker is away, and sent in order once it is back, up to
 * LWIP_UTK_MQTT_LOG_BYTES of them; a line that does not fit is lost and
 * counted. Start the subscriber first: a line only reaches subscribers
 * that are listening when it is sent.
 *
 * Owner: the team.
 */

#include "car_log.h"

#if !defined(CAR_HOST_TEST) && defined(TM_WIFI_MQTT) && TM_WIFI_MQTT

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "comms.h"
#include "lwip_utk_mqtt.h"

/* One buffer for every task, behind a mutex: the benches run on the
 * kernel's initial task, whose 1 KB stack has no room for a line. */
static UB   g_text[CAR_LOG_LINE_BYTES];
static ID   gh_lock    = 0;
static bool gb_started = false;

UB * car_log_begin (void)
{
    /* The first line comes from usermain() before any other task exists,
     * so creating the mutex here cannot race. */
    if (0 == gh_lock)
    {
        T_CMTX const cmtx =
        {
            .exinf   = NULL,
            .mtxatr  = TA_INHERIT,
            .ceilpri = 0,
        };

        gh_lock = tk_cre_mtx(&cmtx);
    }

    if (0 < gh_lock)
    {
        (void)tk_loc_mtx(gh_lock, TMO_FEVR);
    }

    return g_text;
}

void car_log_end (INT length)
{
    (void)tm_putstring(g_text);

    if (!gb_started)
    {
        /* Benches never call comms_init(), so the first line does. */
        gb_started = true;
        (void)comms_init();
    }

    /* Cast: a line is under CAR_LOG_LINE_BYTES, far below 65536. */
    if (0 < length)
    {
        lwip_utk_mqtt_log((char const *)g_text, (uint16_t)length);
    }

    if (0 < gh_lock)
    {
        (void)tk_unl_mtx(gh_lock);
    }
}

#endif

/*** end of file ***/
