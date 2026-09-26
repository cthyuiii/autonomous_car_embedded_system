/** @file bench_scanning.c
 *
 * @brief Tests the servo and the sonar separately, then sweeps.
 *
 * Build with `make BENCH=scanning` and flash to a board with the servo on
 * S1 and the HC-SR04 on Grove 4. Three phases, in this order, because a
 * silent sweep tells you nothing about which of the two parts is at fault:
 *
 *   1. Servo only. The horn is parked at each end and the centre with a
 *      second between moves, and each move is announced before it happens.
 *      Watch the horn. No sonar reading is taken.
 *   2. Sonar only, with the horn parked straight ahead. Ten pings, each
 *      printed with its status. Put your hand 100 mm in front of it.
 *   3. The sweep both of the above feed, once they work.
 *
 * WARNING: The servo takes its power from the board's motor rail, so the
 * battery must be connected and the board switched on. On USB alone the
 * horn will not move at all, which looks exactly like a dead servo.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling. Extend it as you
 * need; nothing else depends on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "scan.h"

#define BENCH_STARTUP_MSEC   2000u
#define BENCH_SERVO_HOLD_MSEC 1000u
#define BENCH_SERVO_ROUNDS       2u
#define BENCH_PING_COUNT        10u
#define BENCH_SWEEP_GAP_MSEC  1000u
#define BENCH_ANGLES             3u

static char const * status_name (car_status_t status);
static void         servo_phase (void);
static void         sonar_phase (void);

INT usermain (void)
{
    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO,
            "scan bench: travel %u to %u deg, pulses %u to %u us\n",
            SCAN_MIN_ANGLE_DEG, SCAN_MAX_ANGLE_DEG,
            SERVO_PULSE_MIN_USEC, SERVO_PULSE_MAX_USEC);

    if (CAR_OK != scan_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "scan_init failed\n");
    }

    servo_phase();
    sonar_phase();

    CAR_LOG(CAR_LOG_INFO, "phase 3: sweeping\n");

    for (;;)
    {
        uint16_t angle_deg = 0u;

        /* Cast: at most one step past SCAN_MAX_ANGLE_DEG, under 200. */
        for (angle_deg = SCAN_MIN_ANGLE_DEG; angle_deg <= SCAN_MAX_ANGLE_DEG;
             angle_deg = (uint16_t)(angle_deg + SCAN_FINE_STEP_DEG))
        {
            uint16_t     range_mm = 0u;
            car_status_t status   = scan_measure(angle_deg, &range_mm);

            CAR_LOG(CAR_LOG_INFO, "angle %u range %u %s\n",
                    angle_deg, range_mm, status_name(status));
        }

        (void)tk_dly_tsk(BENCH_SWEEP_GAP_MSEC);
    }
}

/**
 * @brief Park the horn at each end and the centre, announcing every move.
 *
 * NOTE: This drives the servo through scan_measure(), which is the only
 * way the module moves it, and ignores the range it returns. A horn that
 * does not move here has no power, no signal, or is outside the pulse
 * range the servo accepts.
 */
static void servo_phase (void)
{
    /* Cast: the mean of two angles under 180. */
    uint16_t const angles[BENCH_ANGLES] =
    {
        SCAN_MIN_ANGLE_DEG,
        (uint16_t)((SCAN_MIN_ANGLE_DEG + SCAN_MAX_ANGLE_DEG) / 2u),
        SCAN_MAX_ANGLE_DEG
    };
    uint32_t round = 0u;
    uint32_t index = 0u;

    CAR_LOG(CAR_LOG_INFO, "phase 1: servo only, watch the horn\n");

    for (round = 0u; round < BENCH_SERVO_ROUNDS; round++)
    {
        for (index = 0u; index < BENCH_ANGLES; index++)
        {
            uint16_t range_mm = 0u;

            CAR_LOG(CAR_LOG_INFO, "  moving to %u deg\n", angles[index]);
            (void)scan_measure(angles[index], &range_mm);
            (void)tk_dly_tsk(BENCH_SERVO_HOLD_MSEC);
        }
    }

    CAR_LOG(CAR_LOG_INFO,
            "  if the horn never moved: battery on? servo on S1? pulses in "
            "range?\n");
}

/**
 * @brief Ping repeatedly straight ahead without moving the horn.
 *
 * NOTE: A timeout every time, with the horn known to be working, points at
 * the module's supply. A 5 V HC-SR04 often will not fire at 3.3 V at all.
 */
static void sonar_phase (void)
{
    /* Cast: the mean of two angles under 180. */
    uint16_t const centre = (uint16_t)((SCAN_MIN_ANGLE_DEG
                                        + SCAN_MAX_ANGLE_DEG) / 2u);
    uint32_t       index  = 0u;
    uint32_t       echoes = 0u;

    CAR_LOG(CAR_LOG_INFO,
            "phase 2: sonar only at %u deg, put a hand 100 mm ahead\n",
            centre);

    for (index = 0u; index < BENCH_PING_COUNT; index++)
    {
        uint16_t     range_mm = 0u;
        car_status_t status   = scan_measure(centre, &range_mm);

        if (CAR_OK == status)
        {
            echoes++;
        }

        CAR_LOG(CAR_LOG_INFO, "  ping %u range %u %s\n",
                index, range_mm, status_name(status));
    }

    CAR_LOG(CAR_LOG_INFO, "  %u of %u pings echoed\n",
            echoes, BENCH_PING_COUNT);

    if (0u == echoes)
    {
        CAR_LOG(CAR_LOG_INFO,
                "  nothing echoed: check trig on GP%u, echo on GP%u, and "
                "that a 5 V module is not being starved at 3.3 V\n",
                SONAR_TRIG_PIN, SONAR_ECHO_PIN);
    }
}

/**
 * @brief Name a status so the log reads without a lookup table.
 *
 * @param[in] status Value to name.
 *
 * @return A short static string.
 */
static char const * status_name (car_status_t status)
{
    char const * p_name = "unknown";

    switch (status)
    {
        case CAR_OK:                   p_name = "ok";        break;
        case CAR_ERR_TIMEOUT:          p_name = "no echo";   break;
        case CAR_ERR_RANGE:            p_name = "bad angle"; break;
        case CAR_ERR_HARDWARE:         p_name = "hardware";  break;
        case CAR_ERR_NO_DATA:          p_name = "no data";   break;
        case CAR_ERR_NOT_IMPLEMENTED:  p_name = "absent";    break;
        default:                       p_name = "unknown";   break;
    }

    return p_name;
}

/*** end of file ***/
