/** @file bench_servo.c
 *
 * @brief Drives the servo pin directly, with nothing else in the way.
 *
 * Build with `make BENCH=servo`. This is the bench to reach for when the
 * horn does not move and you need to know whether the fault is the servo,
 * the wiring, the power, or the firmware.
 *
 * It deliberately does not call scan.c at all. It talks straight to the
 * PWM block through car_hw, so the only code between this file and the pin
 * is the kernel's own PWM helper. If the horn moves here but not in the
 * scanning bench, the fault is in scan.c. If it moves in neither, the
 * fault is outside the firmware.
 *
 * WARNING: This sweeps only between SCAN_MIN_ANGLE_DEG and
 * SCAN_MAX_ANGLE_DEG, the same band scan.c clamps to, because the horn
 * on this car has very little room. Widen SCAN_HALF_SWEEP_DEG in
 * car_config.h rather than editing the pulse widths here, so the bench and
 * the car can never disagree about how far the horn may travel.
 *
 * WARNING: The very first pulse still throws the horn from wherever it was
 * left to SERVO_CENTRE_PULSE_USEC, because a servo has no way to report
 * where it is. That is the one large movement, and it shrinks to nothing
 * once SERVO_CENTRE_PULSE_USEC matches the mount. Set BENCH_FIND_CENTRE to
 * 1 below to measure it.
 *
 * NOTE: A ten degree sweep is a small movement and easy to miss. Stick a
 * strip of tape to the horn as a pointer before running this.
 *
 * Three phases, each announced before it runs:
 *
 *   1. Centre, held for four seconds. Almost every servo twitches as the
 *      first pulse arrives, so watch from the moment it starts: a twitch
 *      and nothing else still proves signal and power are getting through.
 *   2. Slow sweep across the band and back, one degree at a time, holding
 *      each step long enough to see. This is the phase to watch.
 *   3. Signal off for two seconds. A servo that was holding usually goes
 *      slack, which is one more sign it was listening.
 *
 * WARNING: The servo header takes its power from the board's motor rail,
 * not from USB. The battery must be connected and the board switched on or
 * the horn cannot move no matter what this prints.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_hw.h"
#include "car_log.h"

/* Set to 1 to hunt for SERVO_CENTRE_PULSE_USEC instead of sweeping. It
 * walks the pulse across a wide range so you can note the width at which
 * the horn points straight ahead. WARNING: that is a large movement by
 * definition, so run it with the horn removed or the car held clear. */
#define BENCH_FIND_CENTRE         0

#define BENCH_FIND_LOW_USEC    1200u
#define BENCH_FIND_HIGH_USEC   1900u
#define BENCH_FIND_STEP_USEC     10u

#define BENCH_STARTUP_MSEC     2000u
#define BENCH_CENTRE_HOLD_MSEC 4000u
#define BENCH_STEP_DEG            1u
#define BENCH_STEP_HOLD_MSEC    250u
#define BENCH_PHASE_GAP_MSEC   2000u
#define BENCH_OFF_HOLD_MSEC    2000u

/* One microsecond per count, so a compare value is a pulse width and the
 * wrap is the servo frame. Same numbers scan.c uses. */
#define BENCH_CLOCK_DIVIDER     125u
#define BENCH_FRAME_USEC        (1000000u / SERVO_PWM_FREQ_HZ)

#define BENCH_LOW_DEG             SCAN_MIN_ANGLE_DEG
#define BENCH_HIGH_DEG            SCAN_MAX_ANGLE_DEG

static uint16_t pulse_for (uint16_t angle_deg);
#if BENCH_FIND_CENTRE
static void     find_centre (void);
#else
static void     hold_at (uint16_t angle_deg, uint32_t msec);
static void     sweep (uint16_t from_deg, uint16_t to_deg,
                       char const * p_what);
#endif

INT usermain (void)
{
    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "servo bench: pin GP%u, sweep %u to %u deg\n",
            SERVO_PIN, BENCH_LOW_DEG, BENCH_HIGH_DEG);
    CAR_LOG(CAR_LOG_INFO,
            "centre pulse %u us, %u us per degree, so %u to %u us\n",
            SERVO_CENTRE_PULSE_USEC, SERVO_USEC_PER_DEG,
            pulse_for(BENCH_LOW_DEG), pulse_for(BENCH_HIGH_DEG));
    CAR_LOG(CAR_LOG_INFO,
            "battery must be connected: the servo header runs off the "
            "motor rail. Tape a pointer to the horn, the sweep is small\n");

    car_hw_enable_pwm();
    car_hw_enable_timer();
    car_hw_pwm_setup(SERVO_PIN, BENCH_CLOCK_DIVIDER, BENCH_FRAME_USEC - 1u);

    for (;;)
    {
#if BENCH_FIND_CENTRE
        find_centre();
#else
        CAR_LOG(CAR_LOG_INFO, "phase 1: centre %u deg for %u s, watch now\n",
                SCAN_CENTRE_ANGLE_DEG, BENCH_CENTRE_HOLD_MSEC / 1000u);
        hold_at(SCAN_CENTRE_ANGLE_DEG, BENCH_CENTRE_HOLD_MSEC);

        sweep(BENCH_LOW_DEG, BENCH_HIGH_DEG, "phase 2: sweep out");
        sweep(BENCH_HIGH_DEG, BENCH_LOW_DEG, "phase 2: sweep back");

        CAR_LOG(CAR_LOG_INFO, "phase 3: signal off for %u s\n",
                BENCH_OFF_HOLD_MSEC / 1000u);
        car_hw_pwm_level(SERVO_PIN, 0u);
        (void)tk_dly_tsk(BENCH_OFF_HOLD_MSEC);

        CAR_LOG(CAR_LOG_INFO,
                "nothing moved at all: check the battery, that the lead is "
                "on S1 the right way round, and that the horn is free\n");
        (void)tk_dly_tsk(BENCH_PHASE_GAP_MSEC);
#endif
    }
}

/**
 * @brief Pulse width for an angle, the same mapping scan.c uses.
 *
 * Measured from the mounted centre, so an angle is an offset from where
 * the horn already rests rather than a fraction of a 0 to 180 range.
 *
 * @param[in] angle_deg 0 to SERVO_TRAVEL_DEG, 90 being straight ahead.
 *
 * @return Pulse width in microseconds.
 */
static uint16_t pulse_for (uint16_t angle_deg)
{
    /* Casts: angles and pulse widths are under 3000, exact as int32_t. */
    int32_t offset = (int32_t)angle_deg - (int32_t)SCAN_CENTRE_ANGLE_DEG;
    int32_t pulse  = (int32_t)SERVO_CENTRE_PULSE_USEC
                     + (offset * (int32_t)SERVO_USEC_PER_DEG);

    /* Casts: the pulse limits are constants under 3000. */
    if (pulse < (int32_t)SERVO_PULSE_MIN_USEC)
    {
        pulse = (int32_t)SERVO_PULSE_MIN_USEC;
    }
    else if (pulse > (int32_t)SERVO_PULSE_MAX_USEC)
    {
        pulse = (int32_t)SERVO_PULSE_MAX_USEC;
    }
    else
    {
        /* Inside the servo's accepted range. */
    }

    return (uint16_t)pulse;   /* Clamped to the servo's range above */
}

#if BENCH_FIND_CENTRE
/**
 * @brief Walk the pulse width slowly so the mounted centre can be read off.
 *
 * Note the width printed when the horn points straight ahead and put it in
 * SERVO_CENTRE_PULSE_USEC. Every angle the car ever commands is measured
 * from there, so getting it right is what keeps the first movement small.
 */
static void find_centre (void)
{
    uint16_t pulse = BENCH_FIND_LOW_USEC;

    CAR_LOG(CAR_LOG_INFO,
            "centre finder: %u to %u us. Note the width where the horn "
            "points straight ahead\n",
            BENCH_FIND_LOW_USEC, BENCH_FIND_HIGH_USEC);

    while (pulse <= BENCH_FIND_HIGH_USEC)
    {
        car_hw_pwm_level(SERVO_PIN, pulse);
        CAR_LOG(CAR_LOG_INFO, "  %u us\n", pulse);
        (void)tk_dly_tsk(BENCH_STEP_HOLD_MSEC);
        pulse = (uint16_t)(pulse + BENCH_FIND_STEP_USEC);
    }

    (void)tk_dly_tsk(BENCH_PHASE_GAP_MSEC);
}
#endif

#if !BENCH_FIND_CENTRE
/**
 * @brief Command one angle and hold it, printing the angle and its pulse.
 *
 * @param[in] angle_deg Angle to command.
 * @param[in] msec      How long to hold it.
 */
static void hold_at (uint16_t angle_deg, uint32_t msec)
{
    uint16_t pulse = pulse_for(angle_deg);

    car_hw_pwm_level(SERVO_PIN, pulse);
    CAR_LOG(CAR_LOG_INFO, "  %u deg, %u us\n", angle_deg, pulse);
    (void)tk_dly_tsk(msec);
}

/**
 * @brief Walk one degree at a time between two angles.
 *
 * @param[in] from_deg Starting angle.
 * @param[in] to_deg   Ending angle, may be below from_deg.
 * @param[in] p_what   Name for the log line.
 */
static void sweep (uint16_t from_deg, uint16_t to_deg, char const * p_what)
{
    uint16_t angle_deg = from_deg;

    CAR_LOG(CAR_LOG_INFO, "%s, %u to %u deg\n", p_what, from_deg, to_deg);

    for (;;)
    {
        hold_at(angle_deg, BENCH_STEP_HOLD_MSEC);

        if (from_deg <= to_deg)
        {
            if (angle_deg >= to_deg)
            {
                break;
            }

            angle_deg = (uint16_t)(angle_deg + BENCH_STEP_DEG); /* <180 */
        }
        else
        {
            if (angle_deg <= to_deg)
            {
                break;
            }

            angle_deg = (uint16_t)(angle_deg - BENCH_STEP_DEG); /* >0 */
        }
    }

    (void)tk_dly_tsk(BENCH_PHASE_GAP_MSEC);
}
#endif /* !BENCH_FIND_CENTRE */

/*** end of file ***/
