/** @file motion.c
 *
 * @brief Motor drive, encoder counting and PID speed control.
 *
 * NOTE: The Robo Pico carries the H-bridge, two PWM pins per motor on the
 * MOTOR_* pins in car_config.h. Forward drives IN1 with the duty and holds
 * IN2 low, reverse swaps them, and stop leaves both low so the motor
 * coasts. That is the board maker's own convention for its driver.
 *
 * NOTE: Speed is measured from the time between encoder pulses, not from
 * pulses per tick. At 639 pulses per turn a count per 10 ms tick moves in
 * steps of 29 mm per second; a pulse interval resolves any speed the car
 * can reach.
 *
 * Owner: Buddy 2, motion control. This file is yours.
 */

#include "motion.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#else
#include <tk/tkernel.h>
#include "car_hw.h"
#endif

#include "car_config.h"

/* Circle constant in ten thousandths, enough for a wheel base in mm. */
#define MOTION_PI_10000              31416u

/* One PWM period in counter ticks: the slice counts the system clock. */
#define MOTOR_PWM_PERIOD (CAR_HW_SYS_CLOCK_HZ / MOTOR_PWM_FREQ_HZ)


/** What the current command is trying to do. */
typedef enum
{
    MOVE_NONE = 0,
    MOVE_DISTANCE,
    MOVE_TURN,
    MOVE_STEER
} move_kind_t;

/** Per wheel control state. */
typedef struct
{
    int8_t   direction;          /* +1 forward, -1 reverse, 0 coast */
    uint16_t target_mm_per_sec;
    uint16_t measured_mm_per_sec;
    int32_t  integral;
    int32_t  previous_error;
    uint32_t last_count;
    uint32_t stalled_msec;
    bool     b_ever_pulsed;
    bool     b_against;          /* Last pulse opposed the command */
} wheel_t;

/* Pulse counters and timestamps written by the encoder interrupt and read
 * by the tick. WARNING: Only read these through the DI() protected paths. */
static volatile uint32_t g_encoder_count_left     = 0u;
static volatile uint32_t g_encoder_count_right    = 0u;
static volatile uint32_t g_pulse_usec_left        = 0u;
static volatile uint32_t g_pulse_usec_right       = 0u;
static volatile uint32_t g_interval_usec_left     = 0u;
static volatile uint32_t g_interval_usec_right    = 0u;
static volatile bool     gb_backward_left          = false;  /* Phase B */
static volatile bool     gb_backward_right         = false;
static volatile uint8_t  g_disagree_left          = 0u;
static volatile uint8_t  g_disagree_right         = 0u;
static volatile uint32_t g_glitches_left          = 0u;
static volatile uint32_t g_glitches_right         = 0u;
static volatile uint32_t g_reversals_left         = 0u;
static volatile uint32_t g_reversals_right        = 0u;

static motion_state_t g_state             = { 0 };
static wheel_t        g_left              = { 0 };
static wheel_t        g_right             = { 0 };
static move_kind_t    g_move_kind         = MOVE_NONE;
static uint16_t       g_speed_setpoint    = MOTION_DEFAULT_SPEED_MM_PER_SEC;
static uint32_t       g_target_mm         = 0u;
static uint32_t       g_progress_milli_mm = 0u;
static uint32_t       g_distance_milli_mm = 0u;
static uint16_t       g_move_speed        = 0u;  /* Distance move's speed */
static int32_t        g_straight_pulses   = 0;   /* Left ahead of right */

static void         start_move (move_kind_t kind, int8_t left_direction,
                                int8_t right_direction, uint32_t travel_mm);
static void         set_wheel (wheel_t * p_wheel, int32_t signed_speed);
static uint16_t     wheel_duty (wheel_t * p_wheel);
static void         update_measurements (void);
static void         apply_motors (void);
static uint32_t     turn_arc_mm (uint16_t angle_deg);
static void         read_counters (uint32_t * p_left, uint32_t * p_right,
                                   uint32_t * p_interval_left,
                                   uint32_t * p_interval_right,
                                   uint32_t * p_pulse_left,
                                   uint32_t * p_pulse_right);
static uint16_t     interval_to_speed (uint32_t interval_usec,
                                       uint32_t last_pulse_usec,
                                       uint32_t now_usec);
static uint32_t     now_usec (void);
static void         note_direction (volatile bool * p_backward,
                                    volatile uint8_t * p_disagree,
                                    volatile uint32_t * p_reversals,
                                    bool b_backward);
static void         straighten (void);
static int32_t      measure_encoders (void);
static void         note_stall (wheel_t * p_wheel);
static void         hw_init (void);
static void         hw_set_motor (uint32_t in1_pin, uint32_t in2_pin,
                                  int8_t direction, uint16_t duty);

car_status_t motion_init (void)
{
    g_encoder_count_left  = 0u;
    g_encoder_count_right = 0u;
    g_move_kind           = MOVE_NONE;
    g_target_mm           = 0u;
    g_progress_milli_mm   = 0u;
    g_distance_milli_mm   = 0u;
    g_left                = (wheel_t){ 0 };
    g_right               = (wheel_t){ 0 };
    g_state               = (motion_state_t){ 0 };

    hw_init();
    apply_motors();

    return CAR_OK;
}

car_status_t motion_tick (void)
{
    car_status_t status = CAR_OK;

    update_measurements();

    if (((MOVE_DISTANCE == g_move_kind) || (MOVE_TURN == g_move_kind))
        && ((g_progress_milli_mm / 1000u) >= g_target_mm))
    {
        (void)motion_stop();
    }

    if (MOVE_DISTANCE == g_move_kind)
    {
        straighten();
    }

    /* A wheel that pulsed once and then went quiet while still commanded
     * has lost its motor, its wiring or its battery: stop. A wheel that has
     * never pulsed has no working encoder; its speed loop reads zero and
     * drives it hard, and motion_get_state() reports the encoder missing.
     * The wait is long enough for the PID to push a stalled wheel to full
     * duty first, so a hump that stops the car gets climbed rather than
     * halting it. */
    if ((g_left.b_ever_pulsed
         && (g_left.stalled_msec > MOTION_STALL_FAULT_MSEC))
        || (g_right.b_ever_pulsed
            && (g_right.stalled_msec > MOTION_STALL_FAULT_MSEC)))
    {
        (void)motion_stop();
        status = CAR_ERR_HARDWARE;
    }

    apply_motors();

    return status;
}

car_status_t motion_move_forward (uint32_t distance_mm)
{
    start_move(MOVE_DISTANCE, 1, 1, distance_mm);

    return CAR_OK;
}

car_status_t motion_move_backward (uint32_t distance_mm)
{
    start_move(MOVE_DISTANCE, -1, -1, distance_mm);

    return CAR_OK;
}

car_status_t motion_turn_left (uint16_t angle_deg)
{
    car_status_t status = CAR_ERR_RANGE;

    if (angle_deg <= 360u)
    {
        start_move(MOVE_TURN, -1, 1, turn_arc_mm(angle_deg));
        status = CAR_OK;
    }

    return status;
}

car_status_t motion_turn_right (uint16_t angle_deg)
{
    car_status_t status = CAR_ERR_RANGE;

    if (angle_deg <= 360u)
    {
        start_move(MOVE_TURN, 1, -1, turn_arc_mm(angle_deg));
        status = CAR_OK;
    }

    return status;
}

car_status_t motion_drive_steer (int16_t steer_permille)
{
    int32_t steer = steer_permille;
    int32_t speed = g_speed_setpoint;

    if (steer > MOTION_MAX_STEER_PERMILLE)
    {
        steer = MOTION_MAX_STEER_PERMILLE;
    }
    else if (steer < -MOTION_MAX_STEER_PERMILLE)
    {
        steer = -MOTION_MAX_STEER_PERMILLE;
    }
    else
    {
        /* In range, nothing to clamp. */
    }

    set_wheel(&g_left, (speed * (1000 + steer)) / 1000);
    set_wheel(&g_right, (speed * (1000 - steer)) / 1000);
    g_move_kind      = MOVE_STEER;
    g_target_mm      = 0u;
    g_state.b_is_busy = false;

    return CAR_OK;
}

car_status_t motion_set_speed (uint16_t speed_mm_per_sec)
{
    car_status_t status = CAR_ERR_RANGE;

    if (speed_mm_per_sec <= MOTION_MAX_SPEED_MM_PER_SEC)
    {
        g_speed_setpoint = speed_mm_per_sec;
        status           = CAR_OK;
    }

    return status;
}

car_status_t motion_stop (void)
{
    set_wheel(&g_left, 0);
    set_wheel(&g_right, 0);
    g_move_kind         = MOVE_NONE;
    g_target_mm         = 0u;
    g_state.b_is_busy   = false;
    apply_motors();

    return CAR_OK;
}

bool motion_is_busy (void)
{
    return g_state.b_is_busy;
}

car_status_t motion_get_state (motion_state_t * p_state)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_state)
    {
        uint32_t unused = 0u;

        read_counters(&g_state.encoder_count_left,
                      &g_state.encoder_count_right, &unused, &unused,
                      &unused, &unused);
        g_state.b_left_backward  = gb_backward_left;
        g_state.b_right_backward = gb_backward_right;
        g_state.glitches_left    = g_glitches_left;
        g_state.glitches_right   = g_glitches_right;
        g_state.reversals_left   = g_reversals_left;
        g_state.reversals_right  = g_reversals_right;
        *p_state = g_state;
        status   = CAR_OK;
    }

    return status;
}

/**
 * @brief Begin a distance or turn move and mark the state busy.
 *
 * @param[in] kind            MOVE_DISTANCE or MOVE_TURN.
 * @param[in] left_direction  +1 or -1 for the left wheel.
 * @param[in] right_direction +1 or -1 for the right wheel.
 * @param[in] travel_mm       Distance each wheel must cover.
 */
static void start_move (move_kind_t kind, int8_t left_direction,
                        int8_t right_direction, uint32_t travel_mm)
{
    /* Casts: both speeds are 0 to MOTION_MAX_SPEED_MM_PER_SEC, which an
     * int32_t holds, and so does the uint16_t it is stored back into. */
    int32_t speed = (MOVE_TURN == kind)
                    ? (int32_t)MOTION_TURN_SPEED_MM_PER_SEC
                    : (int32_t)g_speed_setpoint;

    set_wheel(&g_left, speed * left_direction);
    set_wheel(&g_right, speed * right_direction);
    g_move_kind         = kind;
    g_target_mm         = travel_mm;
    g_progress_milli_mm = 0u;
    g_move_speed        = (uint16_t)speed;   /* 0 to 65535, see above */
    g_straight_pulses   = 0;
    g_state.b_is_busy   = (0u != travel_mm);

    if (!g_state.b_is_busy)
    {
        (void)motion_stop();
    }
}

/**
 * @brief Set one wheel's direction and speed target from a signed speed.
 *
 * @param[in,out] p_wheel      Wheel to command.
 * @param[in]     signed_speed Negative reverses the wheel.
 */
static void set_wheel (wheel_t * p_wheel, int32_t signed_speed)
{
    int32_t magnitude          = (signed_speed < 0) ? -signed_speed
                                                    : signed_speed;
    int8_t  previous_direction = p_wheel->direction;

    /* Cast: the limit is a small positive constant, exact as int32_t. */
    if (magnitude > (int32_t)MOTION_MAX_SPEED_MM_PER_SEC)
    {
        magnitude = (int32_t)MOTION_MAX_SPEED_MM_PER_SEC;
    }

    if (0 == signed_speed)
    {
        p_wheel->direction = 0;
    }
    else
    {
        p_wheel->direction = (signed_speed < 0) ? -1 : 1;
    }

    /* A steering update every tick must not wipe the integral, or the I
     * term never accumulates. Only a change of direction resets it.
     * b_ever_pulsed deliberately survives: whether an encoder exists is a
     * property of the wiring, not of the current command. */
    if (p_wheel->direction != previous_direction)
    {
        p_wheel->integral       = 0;
        p_wheel->previous_error = 0;
        p_wheel->stalled_msec   = 0u;
    }

    /* Cast: magnitude is 0 to MOTION_MAX_SPEED_MM_PER_SEC by now. */
    p_wheel->target_mm_per_sec = (uint16_t)magnitude;
}

/**
 * @brief Compute the duty for one wheel, per mille.
 *
 * A feedforward term from the target, plus a PID correction on the
 * measured speed. A nonzero target never
 * drops below MOTOR_MIN_DUTY, because the motor would only hum there.
 *
 * @param[in,out] p_wheel Wheel whose PID state is advanced.
 *
 * @return Duty, 0 to MOTOR_PWM_MAX_DUTY.
 */
static uint16_t wheel_duty (wheel_t * p_wheel)
{
    /* Casts: the duty limits are per mille, so int32_t holds them. */
    int32_t const max_duty = (int32_t)MOTOR_PWM_MAX_DUTY;
    int32_t const min_duty = (int32_t)MOTOR_MIN_DUTY;
    int32_t       duty     = 0;

    if (0u != p_wheel->target_mm_per_sec)
    {
        /* Casts: a uint16_t speed times 1000 fits an int32_t, and the
         * divisor is a small positive constant. */
        duty = ((int32_t)p_wheel->target_mm_per_sec
                * (int32_t)MOTOR_PWM_MAX_DUTY)
               / (int32_t)MOTION_MAX_SPEED_MM_PER_SEC;

        /* A wheel turning against its command, rolling back down a
         * hump, is a negative speed, so the error grows and the PID
         * pushes harder instead of easing off. */
        /* Casts: speeds are uint16_t, so int32_t holds them and
         * their difference exactly. */
        int32_t measured = p_wheel->b_against
                           ? -(int32_t)p_wheel->measured_mm_per_sec
                           : (int32_t)p_wheel->measured_mm_per_sec;
        int32_t error = (int32_t)p_wheel->target_mm_per_sec - measured;
        int32_t derivative = error - p_wheel->previous_error;

        p_wheel->integral += error;

        if (p_wheel->integral > MOTION_PID_INTEGRAL_LIMIT)
        {
            p_wheel->integral = MOTION_PID_INTEGRAL_LIMIT;
        }
        else if (p_wheel->integral < -MOTION_PID_INTEGRAL_LIMIT)
        {
            p_wheel->integral = -MOTION_PID_INTEGRAL_LIMIT;
        }
        else
        {
            /* Inside the anti windup band. */
        }

        /* Casts: the gains are small positive constants. The error
         * is under ±2 * MOTION_MAX_SPEED_MM_PER_SEC and the integral
         * is clamped, so each product stays far inside int32_t. */
        duty += (((int32_t)MOTION_PID_KP_MILLI * error)
                 + ((int32_t)MOTION_PID_KI_MILLI * p_wheel->integral)
                 + ((int32_t)MOTION_PID_KD_MILLI * derivative)) / 1000;
        p_wheel->previous_error = error;

        if (duty > max_duty)
        {
            duty = max_duty;
        }

        if (duty < min_duty)
        {
            duty = min_duty;
        }
    }

    return (uint16_t)duty;   /* Clamped to 0 to MOTOR_PWM_MAX_DUTY above */
}


/**
 * @brief Refresh measured speeds, move progress and the telemetry snapshot.
 *
 * Reads the counters and pulse intervals the encoder interrupt maintains.
 */
static void update_measurements (void)
{
    int32_t step_milli_mm = measure_encoders();

    /* Casts: each branch converts only a value its test has proved is
     * not negative, a single tick's step of a few thousand. */
    if (step_milli_mm >= 0)
    {
        g_progress_milli_mm += (uint32_t)step_milli_mm;
    }
    else if ((uint32_t)-step_milli_mm < g_progress_milli_mm)
    {
        g_progress_milli_mm -= (uint32_t)-step_milli_mm;
    }
    else
    {
        g_progress_milli_mm = 0u;   /* Rolled back past where it began. */
    }

    /* A turn spins in place, so it adds nothing to the ground distance.
     * Rolling back adds nothing either: the distance only ever grows,
     * because car_main measures legs as unsigned differences of it. */
    if ((MOVE_TURN != g_move_kind) && (step_milli_mm > 0))
    {
        /* Accumulate in milli mm: dividing each tick's step by 1000 threw
         * away up to a millimetre a tick, 17 percent at 240 mm/s. */
        g_distance_milli_mm += (uint32_t)step_milli_mm;   /* Positive */
        g_state.distance_mm  = g_distance_milli_mm / 1000u;
    }

    /* Casts: the mean of two uint16_t speeds fits a uint16_t. A signed
     * speed is at most 2 * MOTION_MAX_SPEED_MM_PER_SEC, which int16_t
     * holds. */
    g_state.speed_mm_per_sec = (uint16_t)(((uint32_t)g_left.measured_mm_per_sec
                                + (uint32_t)g_right.measured_mm_per_sec)
                               / 2u);
    g_state.left_mm_per_sec  = (int16_t)((int32_t)g_left.measured_mm_per_sec
                                         * g_left.direction
                                         * (g_left.b_against ? -1 : 1));
    g_state.right_mm_per_sec = (int16_t)((int32_t)g_right.measured_mm_per_sec
                                         * g_right.direction
                                         * (g_right.b_against ? -1 : 1));
    g_state.b_left_encoder   = g_left.b_ever_pulsed;
    g_state.b_right_encoder  = g_right.b_ever_pulsed;
}

/**
 * @brief Push the current duty and direction of both wheels to the driver.
 */
static void apply_motors (void)
{
    hw_set_motor(MOTOR_LEFT_IN1_PIN, MOTOR_LEFT_IN2_PIN, g_left.direction,
                 wheel_duty(&g_left));
    hw_set_motor(MOTOR_RIGHT_IN1_PIN, MOTOR_RIGHT_IN2_PIN, g_right.direction,
                 wheel_duty(&g_right));
}

/**
 * @brief Arc each wheel covers in an in place turn through angle_deg.
 *
 * @param[in] angle_deg Turn angle.
 *
 * @return WHEEL_BASE_MM * pi * angle / 360, rounded down.
 */
static uint32_t turn_arc_mm (uint16_t angle_deg)
{
    /* Cast: widening, and 360 degrees of the arc product fits uint32_t. */
    return (WHEEL_BASE_MM * MOTION_PI_10000 * (uint32_t)angle_deg)
           / (360u * 10000u);
}

/**
 * @brief Convert one pulse interval into a wheel speed.
 *
 * @param[in] interval_usec  Time between the last two pulses.
 * @param[in] last_pulse_usec Time of the last pulse.
 * @param[in] now_usec       Current time.
 *
 * @return Speed in mm per second, 0 if the wheel has stopped pulsing.
 */
static uint16_t interval_to_speed (uint32_t interval_usec,
                                   uint32_t last_pulse_usec,
                                   uint32_t now_usec)
{
    uint32_t speed  = 0u;
    uint32_t silent = now_usec - last_pulse_usec;

    if ((0u != interval_usec)
        && (silent < (ENCODER_STALL_TIMEOUT_MSEC * 1000u)))
    {
        speed = (WHEEL_CIRCUMFERENCE_MM * 1000000u)
                / (ENCODER_SLOTS_PER_REV * interval_usec);

        if (speed > (2u * MOTION_MAX_SPEED_MM_PER_SEC))
        {
            speed = 2u * MOTION_MAX_SPEED_MM_PER_SEC;
        }
    }

    return (uint16_t)speed;   /* Capped at 2 * MOTION_MAX_SPEED_MM_PER_SEC */
}

/**
 * @brief Read the encoders into both wheels' speeds, directions and stall
 *        timers.
 *
 * @return How far the car moved since the last tick, in thousandths of a
 *         mm, negative if it rolled against the command.
 */
static int32_t measure_encoders (void)
{
    uint32_t left           = 0u;
    uint32_t right          = 0u;
    uint32_t interval_left  = 0u;
    uint32_t interval_right = 0u;
    uint32_t pulse_left     = 0u;
    uint32_t pulse_right    = 0u;
    uint32_t now            = now_usec();
    int32_t  delta_left     = 0;
    int32_t  delta_right    = 0;

    read_counters(&left, &right, &interval_left, &interval_right,
                  &pulse_left, &pulse_right);
    g_left.measured_mm_per_sec  = interval_to_speed(interval_left,
                                                    pulse_left, now);
    g_right.measured_mm_per_sec = interval_to_speed(interval_right,
                                                    pulse_right, now);
    g_left.b_ever_pulsed  = g_left.b_ever_pulsed
                            || (left != g_left.last_count);
    g_right.b_ever_pulsed = g_right.b_ever_pulsed
                            || (right != g_right.last_count);

    /* Phase B says which way each wheel really turned. Pulses against the
     * commanded direction take progress away instead of adding. */
    g_left.b_against  = (0 != g_left.direction)
                        && (gb_backward_left == (g_left.direction > 0));
    g_right.b_against = (0 != g_right.direction)
                        && (gb_backward_right == (g_right.direction > 0));
    /* Casts: the counters wrap, and one tick's difference is a few
     * pulses, so the unsigned difference is exact as an int32_t. */
    delta_left  = (int32_t)(left - g_left.last_count);
    delta_right = (int32_t)(right - g_right.last_count);
    delta_left  = g_left.b_against ? -delta_left : delta_left;
    delta_right = g_right.b_against ? -delta_right : delta_right;
    g_left.last_count  = left;
    g_right.last_count = right;
    g_straight_pulses += delta_left - delta_right;
    note_stall(&g_left);
    note_stall(&g_right);

    /* Casts: small positive constants. A tick's pulses times the
     * circumference times 1000 stays far inside int32_t. */
    return ((delta_left + delta_right) * (int32_t)WHEEL_CIRCUMFERENCE_MM
            * 1000) / (2 * (int32_t)ENCODER_SLOTS_PER_REV);
}

/**
 * @brief Count how long a driven wheel has measured no speed.
 *
 * @param[in,out] p_wheel Wheel to update.
 */
static void note_stall (wheel_t * p_wheel)
{
    if ((0u != p_wheel->target_mm_per_sec)
        && (0u == p_wheel->measured_mm_per_sec))
    {
        p_wheel->stalled_msec += MOTION_TICK_PERIOD_MSEC;
    }
    else
    {
        p_wheel->stalled_msec = 0u;
    }
}

/**
 * @brief Take one pulse's phase B reading into a wheel's direction.
 *
 * The direction only changes once ENCODER_DIRECTION_PULSES pulses in a row
 * disagree with it, so a single noisy reading is ignored.
 *
 * @param[in,out] p_backward Wheel direction, true while going backward.
 * @param[in,out] p_disagree Pulses in a row that disagreed so far.
 * @param[in,out] p_reversals Direction changes so far, one added per change.
 * @param[in]     b_backward What this pulse's phase B says.
 */
static void note_direction (volatile bool * p_backward,
                            volatile uint8_t * p_disagree,
                            volatile uint32_t * p_reversals,
                            bool b_backward)
{
    if (b_backward == *p_backward)
    {
        *p_disagree = 0u;
    }
    else if ((*p_disagree + 1u) >= ENCODER_DIRECTION_PULSES)
    {
        *p_backward = b_backward;
        *p_disagree = 0u;
        (*p_reversals)++;
    }
    else
    {
        /* Cast: below ENCODER_DIRECTION_PULSES, a small constant. */
        *p_disagree = (uint8_t)(*p_disagree + 1u);
    }
}

/**
 * @brief Hold a distance move's heading by trimming the two wheel speeds.
 *
 * On the spot the heading is the difference between the distances the two
 * wheels have covered, so a proportional term on that difference steers
 * the car back onto the line it started along, not just parallel to it.
 * Skipped until both encoders have pulsed: with one missing the
 * difference only grows and would drive the car in a circle.
 */
static void straighten (void)
{
    /* Casts: the speed and constants are small and positive. The pulse
     * difference is bounded by the move, so the products fit int32_t. */
    int32_t limit      = (int32_t)g_move_speed / 2;
    int32_t correction = ((g_straight_pulses * (int32_t)WHEEL_CIRCUMFERENCE_MM)
                          * (int32_t)MOTION_STRAIGHT_KP)
                         / (int32_t)ENCODER_SLOTS_PER_REV;

    if (g_left.b_ever_pulsed && g_right.b_ever_pulsed)
    {
        if (correction > limit)
        {
            correction = limit;
        }
        else if (correction < -limit)
        {
            correction = -limit;
        }
        else
        {
            /* Inside the limit. */
        }

        /* Casts: g_move_speed is a uint16_t, exact as an int32_t. */
        set_wheel(&g_left, g_left.direction
                           * ((int32_t)g_move_speed - correction));
        set_wheel(&g_right, g_right.direction
                            * ((int32_t)g_move_speed + correction));
    }
}

#ifdef CAR_HOST_TEST

/* Host fakes: no hardware and no interrupts. The test plays the encoder
 * interrupt through motion_host_inject_pulses(). */

static uint32_t g_host_usec = 0u;

static uint32_t now_usec (void)
{
    g_host_usec += MOTION_TICK_PERIOD_MSEC * 1000u;

    return g_host_usec;
}

void motion_host_inject_pulses (uint32_t left, uint32_t right,
                                bool b_left_backward, bool b_right_backward)
{
    uint32_t pulse = 0u;

    for (pulse = 0u; pulse < left; pulse++)
    {
        note_direction(&gb_backward_left, &g_disagree_left,
                       &g_reversals_left, b_left_backward);
    }

    for (pulse = 0u; pulse < right; pulse++)
    {
        note_direction(&gb_backward_right, &g_disagree_right,
                       &g_reversals_right, b_right_backward);
    }

    g_encoder_count_left  += left;
    g_encoder_count_right += right;
    g_interval_usec_left   = 20000u;
    g_interval_usec_right  = 20000u;
    g_pulse_usec_left      = g_host_usec;
    g_pulse_usec_right     = g_host_usec;
}

static void read_counters (uint32_t * p_left, uint32_t * p_right,
                           uint32_t * p_interval_left,
                           uint32_t * p_interval_right,
                           uint32_t * p_pulse_left, uint32_t * p_pulse_right)
{
    *p_left           = g_encoder_count_left;
    *p_right          = g_encoder_count_right;
    *p_interval_left  = g_interval_usec_left;
    *p_interval_right = g_interval_usec_right;
    *p_pulse_left     = g_pulse_usec_left;
    *p_pulse_right    = g_pulse_usec_right;
}

static uint16_t g_host_duty_left  = 0u;
static uint16_t g_host_duty_right = 0u;

void motion_host_get_duty (uint16_t * p_left, uint16_t * p_right)
{
    *p_left  = g_host_duty_left;
    *p_right = g_host_duty_right;
}

static void hw_init (void)
{
}

static void hw_set_motor (uint32_t in1_pin, uint32_t in2_pin,
                          int8_t direction, uint16_t duty)
{
    (void)in2_pin;
    (void)direction;

    if (MOTOR_LEFT_IN1_PIN == in1_pin)
    {
        g_host_duty_left = duty;
    }
    else
    {
        g_host_duty_right = duty;
    }
}

#else /* CAR_HOST_TEST */

static void encoder_isr (UINT intno);

static uint32_t now_usec (void)
{
    return car_hw_usec();
}

static void read_counters (uint32_t * p_left, uint32_t * p_right,
                           uint32_t * p_interval_left,
                           uint32_t * p_interval_right,
                           uint32_t * p_pulse_left, uint32_t * p_pulse_right)
{
    UINT imask = 0u;

    DI(imask);
    *p_left           = g_encoder_count_left;
    *p_right          = g_encoder_count_right;
    *p_interval_left  = g_interval_usec_left;
    *p_interval_right = g_interval_usec_right;
    *p_pulse_left     = g_pulse_usec_left;
    *p_pulse_right    = g_pulse_usec_right;
    EI(imask);
}

/**
 * @brief Bring up the motor PWM slices and the encoder edge interrupt.
 */
static void hw_init (void)
{
    car_hw_enable_pwm();
    car_hw_enable_timer();
    car_hw_pwm_setup(MOTOR_LEFT_IN1_PIN, 1u, MOTOR_PWM_PERIOD - 1u);
    car_hw_pwm_setup(MOTOR_LEFT_IN2_PIN, 1u, MOTOR_PWM_PERIOD - 1u);
    car_hw_pwm_setup(MOTOR_RIGHT_IN1_PIN, 1u, MOTOR_PWM_PERIOD - 1u);
    car_hw_pwm_setup(MOTOR_RIGHT_IN2_PIN, 1u, MOTOR_PWM_PERIOD - 1u);

    car_hw_gpio_input_pullup(ENCODER_LEFT_PIN);
    car_hw_gpio_input_pullup(ENCODER_RIGHT_PIN);
    car_hw_gpio_input_pullup(ENCODER_LEFT_B_PIN);
    car_hw_gpio_input_pullup(ENCODER_RIGHT_B_PIN);

    /* Enable rising edges for processor 0 and hand the bank IRQ to the
     * kernel. Both pins share one NVIC line, so one handler serves both. */
    car_hw_gpio_rise_irq_enable(ENCODER_LEFT_PIN);
    car_hw_gpio_rise_irq_enable(ENCODER_RIGHT_PIN);

    {
        T_DINT dint =
        {
            .intatr = TA_HLNG,
            .inthdr = encoder_isr,
        };

        if (E_OK == tk_def_int(ENCODER_IRQ_NUM, &dint))
        {
            ClearInt(ENCODER_IRQ_NUM);
            EnableInt(ENCODER_IRQ_NUM, ENCODER_IRQ_LEVEL);
        }
    }
}

/**
 * @brief Drive one motor through its two H-bridge inputs.
 *
 * @param[in] in1_pin   Board input that drives forward.
 * @param[in] in2_pin   Board input that drives reverse.
 * @param[in] direction +1, -1 or 0.
 * @param[in] duty      Per mille of full on.
 */
static void hw_set_motor (uint32_t in1_pin, uint32_t in2_pin,
                          int8_t direction, uint16_t duty)
{
    /* Cast: widening. A per mille duty times the period fits uint32_t. */
    uint32_t level = ((uint32_t)duty * MOTOR_PWM_PERIOD) / MOTOR_PWM_MAX_DUTY;

    if (direction > 0)
    {
        car_hw_pwm_level(in1_pin, level);
        car_hw_pwm_level(in2_pin, 0u);
    }
    else if (direction < 0)
    {
        car_hw_pwm_level(in1_pin, 0u);
        car_hw_pwm_level(in2_pin, level);
    }
    else
    {
        car_hw_pwm_level(in1_pin, 0u);
        car_hw_pwm_level(in2_pin, 0u);
    }
}

/**
 * @brief Count one encoder pulse. Kernel interrupt handler for IO_BANK0.
 *
 * NOTE: The RP2040 raises one interrupt for every GPIO in the bank, so
 * this reads the processor 0 status register to learn which encoder pin
 * fired and clears that bit before returning. The pulse interval is what
 * the speed measurement uses; the count is what distance uses. Phase B is
 * sampled at the A edge: its level says which way the wheel turned.
 *
 * @param[in] intno Interrupt number the kernel dispatched, ENCODER_IRQ_NUM.
 */
static void encoder_isr (UINT intno)
{
    uint32_t now = car_hw_usec();

    /* A rising edge is noise if the pin has already dropped back, since a
     * real pulse stays high for at least ENCODER_MIN_PULSE_USEC, or if it
     * came too soon after the last pulse. Noise is counted as a glitch and
     * otherwise ignored, timestamp included. */
    if (car_hw_gpio_rise_irq_take(ENCODER_LEFT_PIN))
    {
        if ((!car_hw_gpio_get(ENCODER_LEFT_PIN))
            || ((now - g_pulse_usec_left) < ENCODER_MIN_PULSE_USEC))
        {
            g_glitches_left++;
        }
        else
        {
            g_encoder_count_left++;
            g_interval_usec_left = now - g_pulse_usec_left;
            g_pulse_usec_left    = now;
            note_direction(&gb_backward_left, &g_disagree_left,
                           &g_reversals_left,
                           ((0u != ENCODER_LEFT_B_FORWARD)
                            != car_hw_gpio_get(ENCODER_LEFT_B_PIN)));
        }
    }

    if (car_hw_gpio_rise_irq_take(ENCODER_RIGHT_PIN))
    {
        if ((!car_hw_gpio_get(ENCODER_RIGHT_PIN))
            || ((now - g_pulse_usec_right) < ENCODER_MIN_PULSE_USEC))
        {
            g_glitches_right++;
        }
        else
        {
            g_encoder_count_right++;
            g_interval_usec_right = now - g_pulse_usec_right;
            g_pulse_usec_right    = now;
            note_direction(&gb_backward_right, &g_disagree_right,
                           &g_reversals_right,
                           ((0u != ENCODER_RIGHT_B_FORWARD)
                            != car_hw_gpio_get(ENCODER_RIGHT_B_PIN)));
        }
    }

    ClearInt(intno);
}

#endif /* CAR_HOST_TEST */

/*** end of file ***/
