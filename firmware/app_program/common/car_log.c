/** @file car_log.c
 *
 * @brief Mirrors every console line to MQTT in the radio builds, so a car
 *        off its USB cable can still be followed.
 *
 * Build the car or any bench with --wifi, then record the run from a
 * laptop. A bench holds at its first line until the broker is up, so a run
 * off the cable is watched from the start. The car waits for it later, in
 * car_main.c, once it has calibrated:
 *
 *     mosquitto_sub -h <broker ip> -t car/log | tee run.txt
 *
 * NOTE: Nothing waits for the radio. Lines are held while Wi-Fi joins or
 * the broker is away, and sent in order once it is back, up to
 * LWIP_UTK_MQTT_LOG_BYTES of them. A line that does not fit is lost and
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

#ifdef CAR_BENCH
#define CAR_LOG_BROKER_POLL_MSEC  100u

static void wait_for_broker (void);
#endif

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
#ifdef CAR_BENCH
        wait_for_broker();
#endif
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

#ifdef CAR_BENCH
/**
 * @brief Hold a bench at its first line until the broker is up, or for
 *        CAR_BROKER_WAIT_MSEC, as the car waits before driving.
 *
 * NOTE: The radio service joins and connects by itself once comms_init()
 * has told it where the broker is, so this only watches. Lines are held
 * for the broker either way. This is so a bench that moves the car does
 * not start before the dashboard can see it.
 */
static void wait_for_broker (void)
{
    lwip_utk_mqtt_status_t status = { 0 };
    uint32_t               waited = 0u;

    (void)tm_putstring((UB const *)"waiting for the broker before "
                                   "starting\n");   /* The kernel's type */
    lwip_utk_mqtt_get_status(&status);

    while ((!status.b_connected) && (waited < CAR_BROKER_WAIT_MSEC))
    {
        (void)tk_dly_tsk(CAR_LOG_BROKER_POLL_MSEC);
        waited += CAR_LOG_BROKER_POLL_MSEC;
        lwip_utk_mqtt_get_status(&status);
    }
}
#endif

#endif

/*** end of file ***/
