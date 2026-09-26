/** @file scan.c
 *
 * @brief Servo sweep, HC-SR04 ranging, profiling, planning and recovery.
 *
 * NOTE: Range in mm is echo microseconds * 10 / SONAR_USEC_PER_CM, using
 * the conversion the HC-SR04 datasheet gives. No floating point needed.
 *
 * NOTE: The servo slice is clocked at one microsecond per count, so the
 * compare level is simply the pulse width in microseconds and the wrap is
 * the 20 ms frame of a 50 Hz servo.
 *
 * Owner: Buddy 5, ultrasonic scanning and obstacle profiling. This file is
 * yours.
 */

#include "scan.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#else
#include <tk/tkernel.h>
#include "car_hw.h"
#endif

#include "car_config.h"
#include "car_log.h"

#define SCAN_ANGLE_UNKNOWN            0xFFFFu
#define SCAN_FINE_MAX_READINGS ((SERVO_TRAVEL_DEG / SCAN_FINE_STEP_DEG) + 2u)
#define SCAN_MILLI_RAD_PER_DEG        17453u
#define SCAN_RECOVER_STEP_COUNT       (2u * SCAN_RECOVER_STEPS)

/* Servo clock: divide the system clock down to one count per microsecond,
 * so a 20 ms frame is 20000 counts. */
#define SERVO_CLOCK_DIVIDER           125u
#define SERVO_FRAME_USEC              (1000000u / SERVO_PWM_FREQ_HZ)

/* Belt and braces on the echo wait. The timeout is in microseconds and is
 * the real limit, but if the TIMER block were ever not counting the time
 * comparison would never advance and the loop would never end. A spin cap
 * turns that into a bounded delay and a timeout instead of a dead car. */
#define SONAR_SPIN_LIMIT              2000000u

/** One leg of the box detour. */
typedef struct
{
    bool     b_turn;        /* true turns, false drives forward */
    bool     b_away;        /* turn away from the obstacle, else toward */
    uint16_t amount;        /* degrees or mm */
} detour_leg_t;

static uint16_t const g_coarse_angles_deg[SCAN_COARSE_ANGLE_COUNT] =
    SCAN_COARSE_ANGLES_DEG;

/* Turn away, step sideways, turn back, pass the obstacle, turn in, step
 * back, turn straight. scan_detour_plan() sizes legs 1, 3 and 5. */
static detour_leg_t g_detour[SCAN_DETOUR_LEGS] =
{
    { true,  true,  SCAN_DETOUR_TURN_DEG },
    { false, false, SCAN_DETOUR_SIDE_MM },
    { true,  false, SCAN_DETOUR_TURN_DEG },
    { false, false, SCAN_DETOUR_DEPTH_MM },
    { true,  false, SCAN_DETOUR_TURN_DEG },
    { false, false, SCAN_DETOUR_SIDE_MM },
    { true,  true,  SCAN_DETOUR_TURN_DEG },
};

static uint16_t           g_last_angle_deg  = SCAN_ANGLE_UNKNOWN;
static uint32_t           g_last_ping_usec  = 0u;
static bool               gb_search_left     = true;
static bool               gb_line_seen       = false;
static uint8_t            g_recover_index   = 0u;
static car_avoid_action_t g_step_action     = CAR_AVOID_STOP;
static uint16_t           g_step_amount     = 0u;

static void         profile_run (uint16_t const * p_ranges, uint8_t count,
                                 uint8_t nearest, uint16_t start_deg,
                                 car_obstacle_profile_t * p_profile);
static uint16_t     servo_pulse_usec (uint16_t angle_deg);
static uint16_t     clamp_mm (uint32_t value, uint16_t low, uint16_t high);
static void         hw_init (void);
static void         hw_set_servo (uint16_t angle_deg);
static car_status_t hw_ping (uint16_t * p_range_mm);
static void         hw_delay_msec (uint32_t msec);
static uint32_t     hw_usec (void);

car_status_t scan_init (void)
{
    g_last_angle_deg = SCAN_ANGLE_UNKNOWN;
    g_last_ping_usec = 0u;
    g_recover_index  = 0u;
    gb_line_seen      = false;
    gb_search_left    = true;
    g_step_action    = CAR_AVOID_STOP;
    g_step_amount    = 0u;

    hw_init();
    hw_set_servo(SCAN_CENTRE_ANGLE_DEG);
    g_last_angle_deg = SCAN_CENTRE_ANGLE_DEG;
    hw_delay_msec(SERVO_SETTLE_MSEC);

    return CAR_OK;
}

car_status_t scan_measure (uint16_t angle_deg, uint16_t * p_range_mm)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_range_mm) && (angle_deg <= SERVO_TRAVEL_DEG))
    {
        uint32_t since_ping = hw_usec() - g_last_ping_usec;

        /* Keep the horn inside the travel the mount actually allows. */
        if (angle_deg < SCAN_MIN_ANGLE_DEG)
        {
            angle_deg = SCAN_MIN_ANGLE_DEG;
        }
        else if (angle_deg > SCAN_MAX_ANGLE_DEG)
        {
            angle_deg = SCAN_MAX_ANGLE_DEG;
        }
        else
        {
            /* Already inside the band. */
        }

        if (angle_deg != g_last_angle_deg)
        {
            hw_set_servo(angle_deg);
            g_last_angle_deg = angle_deg;
            hw_delay_msec(SERVO_SETTLE_MSEC);
        }

        /* The datasheet's minimum cycle keeps one echo from being heard
         * by the next ranging. */
        if (since_ping < (SONAR_MIN_CYCLE_MSEC * 1000u))
        {
            uint32_t wait_usec = (SONAR_MIN_CYCLE_MSEC * 1000u) - since_ping;

            hw_delay_msec((wait_usec + 999u) / 1000u);
        }

        status           = hw_ping(p_range_mm);
        g_last_ping_usec = hw_usec();
    }

    return status;
}

car_status_t scan_coarse (car_obstacle_profile_t * p_profile)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_profile)
    {
        uint16_t nearest_mm  = SONAR_MAX_RANGE_MM;
        uint16_t nearest_deg = SCAN_CENTRE_ANGLE_DEG;
        uint8_t  index       = 0u;
        int32_t  bearing     = 0;

        for (index = 0u; index < SCAN_COARSE_ANGLE_COUNT; index++)
        {
            uint16_t range_mm = SONAR_MAX_RANGE_MM;

            (void)scan_measure(g_coarse_angles_deg[index], &range_mm);

            if (range_mm < nearest_mm)
            {
                nearest_mm  = range_mm;
                nearest_deg = g_coarse_angles_deg[index];
            }
        }

        /* Casts: angles are 0 to 180, exact as int32_t. */
        bearing    = (int32_t)nearest_deg - (int32_t)SCAN_CENTRE_ANGLE_DEG;
        *p_profile = (car_obstacle_profile_t){ 0 };
        p_profile->bearing_deg      = (int16_t)bearing;
        p_profile->closest_range_mm = nearest_mm;
        p_profile->b_is_valid       = (nearest_mm < SCAN_OBSTACLE_RANGE_MM);
        status                      = CAR_OK;
    }

    return status;
}

car_status_t scan_fine (uint16_t start_deg, uint16_t end_deg,
                        car_obstacle_profile_t * p_profile)
{
    car_status_t status = CAR_ERR_RANGE;

    /* Pull the arc into the travel the mount allows before deciding
     * whether it is valid, so a caller asking for a wide sweep gets the
     * band it can actually have rather than a long run of readings the
     * servo never moved for. */
    if (start_deg < SCAN_MIN_ANGLE_DEG)
    {
        start_deg = SCAN_MIN_ANGLE_DEG;
    }

    if (end_deg > SCAN_MAX_ANGLE_DEG)
    {
        end_deg = SCAN_MAX_ANGLE_DEG;
    }

    if ((NULL != p_profile) && (start_deg < end_deg)
        && (end_deg <= SERVO_TRAVEL_DEG))
    {
        uint16_t ranges[SCAN_FINE_MAX_READINGS];
        uint8_t  count     = 0u;
        uint8_t  nearest   = 0u;
        uint16_t angle_deg = start_deg;

        while ((angle_deg <= end_deg) && (count < SCAN_FINE_MAX_READINGS))
        {
            ranges[count] = SONAR_MAX_RANGE_MM;
            (void)scan_measure(angle_deg, &ranges[count]);

            if (ranges[count] < ranges[nearest])
            {
                nearest = count;
            }

            count++;
            /* Cast: at most end_deg plus one step, under 200. */
            angle_deg = (uint16_t)(angle_deg + SCAN_FINE_STEP_DEG);
        }

        *p_profile = (car_obstacle_profile_t){ 0 };

        if (ranges[nearest] < SCAN_OBSTACLE_RANGE_MM)
        {
            profile_run(ranges, count, nearest, start_deg, p_profile);
        }

        status = CAR_OK;
    }

    return status;
}

/**
 * @brief Profile the obstacle from the run of close readings around the
 *        nearest one.
 *
 * @param[in]  p_ranges  Fine scan readings, lowest angle first.
 * @param[in]  count     How many readings there are.
 * @param[in]  nearest   Index of the closest, known to be an obstacle.
 * @param[in]  start_deg Angle of the first reading.
 * @param[out] p_profile Filled in. A side with no readings beyond the run
 *                       is unknown and its clearance stays 0.
 */
static void profile_run (uint16_t const * p_ranges, uint8_t count,
                         uint8_t nearest, uint16_t start_deg,
                         car_obstacle_profile_t * p_profile)
{
    uint8_t  run_start = nearest;
    uint8_t  run_end   = nearest;
    uint8_t  index     = 0u;
    uint16_t span_deg  = 0u;
    uint16_t mid_deg   = 0u;

    /* Grow the run outward from the nearest reading while the readings
     * still count as obstacle. */
    while ((run_start > 0u)
           && (p_ranges[run_start - 1u] < SCAN_OBSTACLE_RANGE_MM))
    {
        run_start--;
    }

    while ((run_end < (count - 1u))
           && (p_ranges[run_end + 1u] < SCAN_OBSTACLE_RANGE_MM))
    {
        run_end++;
    }

    /* Casts: indexes are under SCAN_FINE_MAX_READINGS and angles under
     * 180, so every angle fits a uint16_t and an int16_t, and a range
     * of at most SONAR_MAX_RANGE_MM times a span in milliradians fits a
     * uint32_t. */
    span_deg = (uint16_t)(((run_end - run_start) + 1u) * SCAN_FINE_STEP_DEG);
    mid_deg  = (uint16_t)(start_deg
                          + ((((uint16_t)run_start + (uint16_t)run_end)
                              * SCAN_FINE_STEP_DEG) / 2u));
    p_profile->bearing_deg      = (int16_t)((int32_t)mid_deg
                                            - (int32_t)SCAN_CENTRE_ANGLE_DEG);
    p_profile->closest_range_mm = p_ranges[nearest];
    p_profile->width_mm         = (uint16_t)(((uint32_t)p_ranges[nearest]
                                              * span_deg
                                              * SCAN_MILLI_RAD_PER_DEG)
                                             / 1000000u);

    /* Lower angles look to the right, higher to the left. */
    for (index = 0u; index < run_start; index++)
    {
        if ((0u == p_profile->clearance_right_mm)
            || (p_ranges[index] < p_profile->clearance_right_mm))
        {
            p_profile->clearance_right_mm = p_ranges[index];
        }
    }

    /* Cast: run_end is below count, itself a uint8_t. */
    for (index = (uint8_t)(run_end + 1u); index < count; index++)
    {
        if ((0u == p_profile->clearance_left_mm)
            || (p_ranges[index] < p_profile->clearance_left_mm))
        {
            p_profile->clearance_left_mm = p_ranges[index];
        }
    }

    p_profile->b_is_valid = true;
}

car_status_t scan_plan_avoidance (car_obstacle_profile_t const * p_profile,
                                  car_avoid_action_t * p_action)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_profile) && (NULL != p_action))
    {
        bool b_left_ok  = (p_profile->clearance_left_mm
                           >= SCAN_CLEARANCE_MIN_MM);
        bool b_right_ok = (p_profile->clearance_right_mm
                           >= SCAN_CLEARANCE_MIN_MM);

        if (!p_profile->b_is_valid)
        {
            *p_action = CAR_AVOID_CONTINUE;
        }
        else if (b_left_ok
                 && ((!b_right_ok)
                     || (p_profile->clearance_left_mm
                         >= p_profile->clearance_right_mm)))
        {
            *p_action = CAR_AVOID_LEFT;
        }
        else if (b_right_ok)
        {
            *p_action = CAR_AVOID_RIGHT;
        }
        else
        {
            *p_action = CAR_AVOID_REVERSE;
        }

        status = CAR_OK;
    }

    return status;
}

car_status_t scan_recover_start (bool b_search_left)
{
    gb_search_left   = b_search_left;
    gb_line_seen     = false;
    g_recover_index = 0u;
    g_step_action   = CAR_AVOID_STOP;
    g_step_amount   = 0u;

    return CAR_OK;
}

car_status_t scan_recover_report (bool b_line_seen)
{
    gb_line_seen = b_line_seen;

    return CAR_OK;
}

car_status_t scan_recover_line (void)
{
    car_status_t status = CAR_ERR_NO_DATA;

    if (gb_line_seen)
    {
        g_step_action = CAR_AVOID_STOP;
        g_step_amount = 0u;
        status        = CAR_OK;
    }
    else if (g_recover_index >= SCAN_RECOVER_STEP_COUNT)
    {
        g_step_action = CAR_AVOID_STOP;
        g_step_amount = 0u;
        status        = CAR_ERR_TIMEOUT;
    }
    else
    {
        /* Even steps turn toward the search side, odd steps drive on, so
         * the car arcs across where the line should be. */
        if (0u == (g_recover_index % 2u))
        {
            g_step_action = gb_search_left ? CAR_AVOID_LEFT : CAR_AVOID_RIGHT;
            g_step_amount = SCAN_RECOVER_TURN_DEG;
        }
        else
        {
            g_step_action = CAR_AVOID_CONTINUE;
            g_step_amount = SCAN_RECOVER_DRIVE_MM;
        }

        g_recover_index++;
    }

    return status;
}

car_status_t scan_recover_get_step (car_avoid_action_t * p_action,
                                    uint16_t * p_amount)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_action) && (NULL != p_amount))
    {
        *p_action = g_step_action;
        *p_amount = g_step_amount;
        status    = CAR_OK;
    }

    return status;
}

car_status_t scan_detour_plan (car_obstacle_profile_t const * p_profile)
{
    uint16_t side_mm  = SCAN_DETOUR_SIDE_MM;
    uint16_t depth_mm = SCAN_DETOUR_DEPTH_MM;

    if ((NULL != p_profile) && (0u != p_profile->width_mm))
    {
        /* Cast: widening a uint16_t. */
        uint32_t clear = ((uint32_t)p_profile->width_mm / 2u)
                         + SCAN_DETOUR_MARGIN_MM;

        side_mm  = clamp_mm((clear * SCAN_DETOUR_SIN45_RECIP) / 1000u,
                            SCAN_DETOUR_SIDE_MIN_MM,
                            SCAN_DETOUR_SIDE_MAX_MM);
        /* Width stands in for depth, because the sonar cannot see how far
         * back an obstacle goes. CAR_LENGTH_MM is on top of it: the back
         * of the car is still beside the obstacle when the bumper is past
         * it, and turning in there clips it. */
        depth_mm = clamp_mm((uint32_t)p_profile->width_mm + CAR_LENGTH_MM
                            + SCAN_DETOUR_MARGIN_MM,
                            SCAN_DETOUR_DEPTH_MIN_MM,
                            SCAN_DETOUR_DEPTH_MAX_MM);
    }

    g_detour[1].amount = side_mm;
    g_detour[3].amount = depth_mm;
    g_detour[5].amount = side_mm;

    CAR_LOG(CAR_LOG_INFO, "detour sized for %u mm wide: side %u depth %u\n",
            (NULL != p_profile) ? p_profile->width_mm : 0u, side_mm,
            depth_mm);

    return CAR_OK;
}

car_status_t scan_detour_get_leg (uint8_t index, car_avoid_action_t side,
                                  bool b_undo, car_avoid_action_t * p_action,
                                  uint16_t * p_amount)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((index < SCAN_DETOUR_LEGS) && (NULL != p_action)
        && (NULL != p_amount))
    {
        detour_leg_t const * p_leg = &g_detour[index];

        if (p_leg->b_turn)
        {
            /* Away from the obstacle turns toward the detour side. A turn
             * back toward it goes the other way, and undoing a turn
             * flips it again. */
            bool b_left = ((CAR_AVOID_LEFT == side) == p_leg->b_away);

            *p_action = (b_left != b_undo) ? CAR_AVOID_LEFT
                                           : CAR_AVOID_RIGHT;
        }
        else
        {
            *p_action = b_undo ? CAR_AVOID_REVERSE : CAR_AVOID_CONTINUE;
        }

        *p_amount = p_leg->amount;
        status    = CAR_OK;
    }

    return status;
}

/**
 * @brief Hold a value inside a range.
 *
 * @param[in] value What to clamp.
 * @param[in] low   Lowest allowed.
 * @param[in] high  Highest allowed.
 *
 * @return The clamped value.
 */
static uint16_t clamp_mm (uint32_t value, uint16_t low, uint16_t high)
{
    uint32_t held = (value < low) ? low : value;

    held = (held > high) ? high : held;

    return (uint16_t)held;   /* Cast: between low and high, both uint16_t */
}

/**
 * @brief Servo pulse width for an angle, measured from the mounted centre.
 *
 * NOTE: The pulse is SERVO_CENTRE_PULSE_USEC plus the offset from straight
 * ahead, not a fraction of a 0 to 180 range. That way the horn's rest
 * position is a number someone measured on this car, and every commanded
 * angle sits a known few degrees either side of it. The result is clamped
 * to the servo's accepted pulse range as a last check.
 *
 * @param[in] angle_deg 0 to SERVO_TRAVEL_DEG, 90 being straight ahead.
 *
 * @return Pulse width in microseconds.
 */
static uint16_t servo_pulse_usec (uint16_t angle_deg)
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

#ifdef CAR_HOST_TEST

/* Host fakes: no servo, no sonar, a clock that only moves on delays. */
static uint16_t g_host_range_mm   = SONAR_MAX_RANGE_MM;
static uint32_t g_host_usec       = 0u;
static uint16_t g_host_angle_deg  = 0u;
static uint16_t g_host_pulse_usec = 0u;

void scan_host_inject_range (uint16_t range_mm)
{
    g_host_range_mm = range_mm;
}

static void hw_init (void)
{
}

static void hw_set_servo (uint16_t angle_deg)
{
    g_host_angle_deg  = angle_deg;
    g_host_pulse_usec = servo_pulse_usec(angle_deg);
}

void scan_host_get_servo (uint16_t * p_angle_deg, uint16_t * p_pulse_usec)
{
    if ((NULL != p_angle_deg) && (NULL != p_pulse_usec))
    {
        *p_angle_deg  = g_host_angle_deg;
        *p_pulse_usec = g_host_pulse_usec;
    }
}

static car_status_t hw_ping (uint16_t * p_range_mm)
{
    *p_range_mm = g_host_range_mm;

    return CAR_OK;
}

static void hw_delay_msec (uint32_t msec)
{
    g_host_usec += msec * 1000u;
}

static uint32_t hw_usec (void)
{
    return g_host_usec;
}

#else /* CAR_HOST_TEST */

/**
 * @brief Bring up the servo slice and the two sonar pins.
 */
static void hw_init (void)
{
    car_hw_enable_pwm();
    car_hw_enable_timer();
    car_hw_pwm_setup(SERVO_PIN, SERVO_CLOCK_DIVIDER, SERVO_FRAME_USEC - 1u);
    car_hw_gpio_output(SONAR_TRIG_PIN);
    car_hw_gpio_put(SONAR_TRIG_PIN, false);
    car_hw_gpio_input(SONAR_ECHO_PIN);
}

static void hw_set_servo (uint16_t angle_deg)
{
    car_hw_pwm_level(SERVO_PIN, servo_pulse_usec(angle_deg));
}

/**
 * @brief One trigger pulse and one timed echo.
 *
 * @param[out] p_range_mm Distance, SONAR_MAX_RANGE_MM on timeout.
 *
 * @return CAR_OK, or CAR_ERR_TIMEOUT if the echo never came or never ended.
 */
static car_status_t hw_ping (uint16_t * p_range_mm)
{
    car_status_t status    = CAR_ERR_TIMEOUT;
    uint32_t     started   = 0u;
    uint32_t     rise      = 0u;
    uint32_t     fall      = 0u;
    uint32_t     spin      = 0u;
    bool         b_rose    = false;
    bool         b_fell    = false;

    *p_range_mm = SONAR_MAX_RANGE_MM;

    car_hw_gpio_put(SONAR_TRIG_PIN, true);
    WaitUsec(SONAR_TRIG_PULSE_USEC);
    car_hw_gpio_put(SONAR_TRIG_PIN, false);

    started = car_hw_usec();

    for (spin = 0u; spin < SONAR_SPIN_LIMIT; spin++)
    {
        if ((car_hw_usec() - started) >= SONAR_ECHO_TIMEOUT_USEC)
        {
            break;
        }

        if (car_hw_gpio_get(SONAR_ECHO_PIN))
        {
            b_rose = true;
            rise   = car_hw_usec();
            break;
        }
    }

    for (spin = 0u; b_rose && (spin < SONAR_SPIN_LIMIT); spin++)
    {
        if ((car_hw_usec() - rise) >= SONAR_ECHO_TIMEOUT_USEC)
        {
            break;
        }

        if (!car_hw_gpio_get(SONAR_ECHO_PIN))
        {
            b_fell = true;
            fall   = car_hw_usec();
            break;
        }
    }

    if (b_fell)
    {
        uint32_t range_mm = ((fall - rise) * 10u) / SONAR_USEC_PER_CM;

        if (range_mm < SONAR_MIN_RANGE_MM)
        {
            range_mm = SONAR_MIN_RANGE_MM;
        }

        if (range_mm > SONAR_MAX_RANGE_MM)
        {
            range_mm = SONAR_MAX_RANGE_MM;
        }

        *p_range_mm = (uint16_t)range_mm;   /* Clamped to the max above */
        status      = CAR_OK;
    }

    return status;
}

static void hw_delay_msec (uint32_t msec)
{
    (void)tk_dly_tsk((RELTIM)msec);   /* RELTIM is 32 bit, as msec is */
}

static uint32_t hw_usec (void)
{
    return car_hw_usec();
}

#endif /* CAR_HOST_TEST */

/*** end of file ***/
