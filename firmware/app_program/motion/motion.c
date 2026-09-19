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
 * pulses per tick. At 20 pulses per turn and a 10 ms tick a pulse count
 * would resolve nothing below about 1000 mm per second; a pulse interval
 * resolves any speed the car can reach.
 *
 * Owner: Buddy 2, motion control. Implement the TODOs in this file. It is
 * yours.
 */

#include "motion.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#else
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "car_hw.h"
#endif

#include "car_config.h"

/* Circle constant in ten thousandths, enough for a wheel base in mm. */
#define MOTION_PI_10000              31416u

/* One PWM period in counter ticks: the slice counts the system clock. */
#define MOTOR_PWM_PERIOD (CAR_HW_SYS_CLOCK_HZ / MOTOR_PWM_FREQ_HZ)

/* IO_BANK0 interrupt registers, RP2040 datasheet section 2.19.6.1. Each
 * GPIO owns four bits per register, eight GPIO per register, and the raw
 * INTR bits are cleared by writing one to them. libbsp names none of these. */
#define IO_BANK0_INTR(reg)           (IO_BANK0_BASE + 0x0F0u + ((reg) * 4u))
#define IO_BANK0_PROC0_INTE(reg)     (IO_BANK0_BASE + 0x100u + ((reg) * 4u))
#define IO_BANK0_PROC0_INTS(reg)     (IO_BANK0_BASE + 0x120u + ((reg) * 4u))
#define GPIO_INT_REG(pin)            ((pin) / 8u)
#define GPIO_EDGE_HIGH_BIT(pin)      (1u << ((((pin) % 8u) * 4u) + 3u))

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
} wheel_t;

/* Pulse counters and timestamps written by the encoder interrupt and read
 * by the tick. WARNING: Only read these through the DI() protected paths. */
static volatile uint32_t g_encoder_count_left     = 0u;
static volatile uint32_t g_encoder_count_right    = 0u;
static volatile uint32_t g_pulse_usec_left        = 0u;
static volatile uint32_t g_pulse_usec_right       = 0u;
static volatile uint32_t g_interval_usec_left     = 0u;
static volatile uint32_t g_interval_usec_right    = 0u;

static motion_state_t g_state             = { 0 };
static wheel_t        g_left              = { 0 };
static wheel_t        g_right             = { 0 };
static move_kind_t    g_move_kind         = MOVE_NONE;
static uint16_t       g_speed_setpoint    = MOTION_DEFAULT_SPEED_MM_PER_SEC;
static uint32_t       g_target_mm         = 0u;
static uint32_t       g_progress_milli_mm = 0u;

static void         start_move (move_kind_t kind, int8_t left_direction,
                                int8_t right_direction, uint32_t travel_mm);
static void         set_wheel (wheel_t * p_wheel, int32_t signed_speed);
static uint16_t     wheel_duty (wheel_t * p_wheel);
#if MOTION_OPEN_LOOP
static uint16_t     duty_to_speed (uint16_t duty);
#endif
static void         update_measurements (void);
static void         apply_motors (void);
static uint32_t     turn_arc_mm (uint16_t angle_deg);
static void         read_counters (uint32_t * p_left, uint32_t * p_right,
                                   uint32_t * p_interval_left,
                                   uint32_t * p_interval_right,
                                   uint32_t * p_pulse_left,
                                   uint32_t * p_pulse_right);
#if !MOTION_OPEN_LOOP
static uint16_t     interval_to_speed (uint32_t interval_usec,
                                       uint32_t last_pulse_usec,
                                       uint32_t now_usec);
static uint32_t     now_usec (void);
#endif
static void         hw_init (void);
static void         hw_set_motor (uint32_t in1_pin, uint32_t in2_pin,
                                  int8_t direction, uint16_t duty);

car_status_t motion_init (void)
{
    // TODO: pwm_set_pin() each MOTOR_*_PIN, pwm_set_wrap() for
    //       MOTOR_PWM_FREQ_HZ, duty zero, pwm_set_enabled(). Then
    //       gpio_set_pin(GPIO_MODE_IN) both encoder pins, enable their
    //       rising edge in IO_BANK0, and register encoder_isr with
    //       tk_def_int(ENCODER_IRQ_NUM, ...) and EnableInt().
    g_encoder_count_left  = 0u;
    g_encoder_count_right = 0u;
    g_move_kind           = MOVE_NONE;
    g_target_mm           = 0u;
    g_progress_milli_mm   = 0u;
    g_left                = (wheel_t){ 0 };
    g_right               = (wheel_t){ 0 };
    g_state               = (motion_state_t){ 0 };

    hw_init();
    apply_motors();

    return CAR_OK;
}

car_status_t motion_tick (void)
{
    // TODO: Convert the encoder delta since last tick to mm per second,
    //       run the PID with MOTION_PID_*_MILLI, apply duty to each motor,
    //       and clear b_is_busy when the target distance is reached.
    car_status_t status = CAR_OK;

    update_measurements();

    if (((MOVE_DISTANCE == g_move_kind) || (MOVE_TURN == g_move_kind))
        && ((g_progress_milli_mm / 1000u) >= g_target_mm))
    {
        (void)motion_stop();
    }

#if !MOTION_OPEN_LOOP
    /* A wheel that pulsed once and then went quiet while still commanded
     * has lost its motor, its wiring or its battery: stop. A wheel that
     * has never pulsed at all simply has no encoder fitted, so let it run
     * open loop rather than refusing to move a half wired car. */
    if ((g_left.b_ever_pulsed
         && (g_left.stalled_msec > ENCODER_STALL_TIMEOUT_MSEC))
        || (g_right.b_ever_pulsed
            && (g_right.stalled_msec > ENCODER_STALL_TIMEOUT_MSEC)))
    {
        (void)motion_stop();
        status = CAR_ERR_HARDWARE;
    }
#endif

    apply_motors();

    return status;
}

car_status_t motion_move_forward (uint32_t distance_mm)
{
    // TODO: Convert distance_mm to a pulse target using
    //       ENCODER_SLOTS_PER_REV and WHEEL_CIRCUMFERENCE_MM, then set
    //       both motors forward and mark the state busy.
    start_move(MOVE_DISTANCE, 1, 1, distance_mm);

    return CAR_OK;
}

car_status_t motion_move_backward (uint32_t distance_mm)
{
    // TODO: Same as forward with both motor directions reversed.
    start_move(MOVE_DISTANCE, -1, -1, distance_mm);

    return CAR_OK;
}

car_status_t motion_turn_left (uint16_t angle_deg)
{
    // TODO: Arc length per wheel is WHEEL_BASE_MM * pi * angle / 360.
    //       Drive wheels in opposite directions for that many pulses.
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
    // TODO: Mirror of motion_turn_left().
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
    // TODO: Reject above MOTION_MAX_SPEED_MM_PER_SEC, else store as the
    //       PID setpoint.
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
    // TODO: Zero both duties, clear the pulse target and b_is_busy.
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
    // TODO: Refresh encoder counts and distance before copying, then
    //       return CAR_OK.
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_state)
    {
        uint32_t unused = 0u;

        read_counters(&g_state.encoder_count_left,
                      &g_state.encoder_count_right, &unused, &unused,
                      &unused, &unused);
        *p_state = g_state;
        status   = CAR_OK;
    }

    return status;
}

car_status_t motion_get_encoder_counts (uint32_t * p_left, uint32_t * p_right)
{
    // TODO: Wrap the two reads in the kernel's DI() and EI(), then
    //       return CAR_OK.
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_left) && (NULL != p_right))
    {
        uint32_t unused = 0u;

        read_counters(p_left, p_right, &unused, &unused, &unused, &unused);
        status = CAR_OK;
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
    int32_t speed = g_speed_setpoint;

    set_wheel(&g_left, speed * left_direction);
    set_wheel(&g_right, speed * right_direction);
    g_move_kind         = kind;
    g_target_mm         = travel_mm;
    g_progress_milli_mm = 0u;
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

    p_wheel->target_mm_per_sec = (uint16_t)magnitude;
}

/**
 * @brief Compute the duty for one wheel, per mille.
 *
 * Open loop uses the feedforward term alone. Closed loop adds a PID
 * correction on the measured speed. Either way a nonzero target never
 * drops below MOTOR_MIN_DUTY, because the motor would only hum there.
 *
 * @param[in,out] p_wheel Wheel whose PID state is advanced.
 *
 * @return Duty, 0 to MOTOR_PWM_MAX_DUTY.
 */
static uint16_t wheel_duty (wheel_t * p_wheel)
{
    int32_t duty = 0;

    if (0u != p_wheel->target_mm_per_sec)
    {
        duty = ((int32_t)p_wheel->target_mm_per_sec
                * (int32_t)MOTOR_PWM_MAX_DUTY)
               / (int32_t)MOTION_MAX_SPEED_MM_PER_SEC;

#if !MOTION_OPEN_LOOP
        {
            int32_t error = (int32_t)p_wheel->target_mm_per_sec
                            - (int32_t)p_wheel->measured_mm_per_sec;
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

            duty += (((int32_t)MOTION_PID_KP_MILLI * error)
                     + ((int32_t)MOTION_PID_KI_MILLI * p_wheel->integral)
                     + ((int32_t)MOTION_PID_KD_MILLI * derivative)) / 1000;
            p_wheel->previous_error = error;
        }
#endif

        if (duty > (int32_t)MOTOR_PWM_MAX_DUTY)
        {
            duty = (int32_t)MOTOR_PWM_MAX_DUTY;
        }

        if (duty < (int32_t)MOTOR_MIN_DUTY)
        {
            duty = (int32_t)MOTOR_MIN_DUTY;
        }
    }

    return (uint16_t)duty;
}

#if MOTION_OPEN_LOOP
/**
 * @brief The speed a duty actually produces, the inverse of wheel_duty().
 *
 * @param[in] duty Per mille, 0 to MOTOR_PWM_MAX_DUTY.
 *
 * @return Millimetres per second.
 */
static uint16_t duty_to_speed (uint16_t duty)
{
    return (uint16_t)(((uint32_t)duty * MOTION_MAX_SPEED_MM_PER_SEC)
                      / MOTOR_PWM_MAX_DUTY);
}
#endif

/**
 * @brief Refresh measured speeds, move progress and the telemetry snapshot.
 *
 * Open loop dead reckons from the targets: speed times tick period. Closed
 * loop reads the counters and pulse intervals the interrupt maintains.
 */
static void update_measurements (void)
{
    uint32_t step_milli_mm = 0u;

#if MOTION_OPEN_LOOP
    /* Dead reckon from the duty that is about to be applied, not from the
     * speed that was asked for. MOTOR_MIN_DUTY floors any request below
     * it, so a slow command makes the car run faster than it was told,
     * and every timed distance and turn would overshoot by that ratio
     * with no way to see it. wheel_duty() has no side effects in this
     * build, so asking it here is free. */
    g_left.measured_mm_per_sec  = duty_to_speed(wheel_duty(&g_left));
    g_right.measured_mm_per_sec = duty_to_speed(wheel_duty(&g_right));
    step_milli_mm = ((uint32_t)g_left.measured_mm_per_sec
                     + (uint32_t)g_right.measured_mm_per_sec)
                    * MOTION_TICK_PERIOD_MSEC / 2u;
#else
    {
        uint32_t left           = 0u;
        uint32_t right          = 0u;
        uint32_t interval_left  = 0u;
        uint32_t interval_right = 0u;
        uint32_t pulse_left     = 0u;
        uint32_t pulse_right    = 0u;
        uint32_t now            = now_usec();
        uint32_t delta          = 0u;

        read_counters(&left, &right, &interval_left, &interval_right,
                      &pulse_left, &pulse_right);
        g_left.measured_mm_per_sec  = interval_to_speed(interval_left,
                                                        pulse_left, now);
        g_right.measured_mm_per_sec = interval_to_speed(interval_right,
                                                        pulse_right, now);

        if (left != g_left.last_count)
        {
            g_left.b_ever_pulsed = true;
        }

        if (right != g_right.last_count)
        {
            g_right.b_ever_pulsed = true;
        }

        delta = (left - g_left.last_count) + (right - g_right.last_count);
        g_left.last_count  = left;
        g_right.last_count = right;
        step_milli_mm = (delta * WHEEL_CIRCUMFERENCE_MM * 1000u)
                        / (2u * ENCODER_SLOTS_PER_REV);

        if ((0u != g_left.target_mm_per_sec)
            && (0u == g_left.measured_mm_per_sec))
        {
            g_left.stalled_msec += MOTION_TICK_PERIOD_MSEC;
        }
        else
        {
            g_left.stalled_msec = 0u;
        }

        if ((0u != g_right.target_mm_per_sec)
            && (0u == g_right.measured_mm_per_sec))
        {
            g_right.stalled_msec += MOTION_TICK_PERIOD_MSEC;
        }
        else
        {
            g_right.stalled_msec = 0u;
        }
    }
#endif

    g_progress_milli_mm += step_milli_mm;

    /* A turn spins in place, so it adds nothing to the ground distance. */
    if (MOVE_TURN != g_move_kind)
    {
        g_state.distance_mm += step_milli_mm / 1000u;
    }

    g_state.speed_mm_per_sec = (uint16_t)(((uint32_t)g_left.measured_mm_per_sec
                                + (uint32_t)g_right.measured_mm_per_sec)
                               / 2u);
    g_state.left_mm_per_sec  = (int16_t)((int32_t)g_left.measured_mm_per_sec
                                         * g_left.direction);
    g_state.right_mm_per_sec = (int16_t)((int32_t)g_right.measured_mm_per_sec
                                         * g_right.direction);
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
#if !MOTION_OPEN_LOOP
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

    return (uint16_t)speed;
}
#endif /* !MOTION_OPEN_LOOP */

#ifdef CAR_HOST_TEST

/* Host fakes: no hardware, no interrupts, counters stay at zero. */

#if !MOTION_OPEN_LOOP
static uint32_t now_usec (void)
{
    static uint32_t fake_usec = 0u;

    fake_usec += MOTION_TICK_PERIOD_MSEC * 1000u;

    return fake_usec;
}
#endif

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

static void hw_init (void)
{
}

static void hw_set_motor (uint32_t in1_pin, uint32_t in2_pin,
                          int8_t direction, uint16_t duty)
{
    (void)in1_pin;
    (void)in2_pin;
    (void)direction;
    (void)duty;
}

#else /* CAR_HOST_TEST */

#if !MOTION_OPEN_LOOP
static void encoder_isr (UINT intno);

static uint32_t now_usec (void)
{
    return car_hw_usec();
}
#endif

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

#if !MOTION_OPEN_LOOP
    car_hw_gpio_input_pullup(ENCODER_LEFT_PIN);
    car_hw_gpio_input_pullup(ENCODER_RIGHT_PIN);

    /* Clear any edge latched while the pins were being configured, then
     * enable rising edges for processor 0 and hand the bank IRQ to the
     * kernel. Both pins share one NVIC line, so one handler serves both. */
    out_w(IO_BANK0_INTR(GPIO_INT_REG(ENCODER_LEFT_PIN)),
          GPIO_EDGE_HIGH_BIT(ENCODER_LEFT_PIN));
    out_w(IO_BANK0_INTR(GPIO_INT_REG(ENCODER_RIGHT_PIN)),
          GPIO_EDGE_HIGH_BIT(ENCODER_RIGHT_PIN));
    set_w(IO_BANK0_PROC0_INTE(GPIO_INT_REG(ENCODER_LEFT_PIN)),
          GPIO_EDGE_HIGH_BIT(ENCODER_LEFT_PIN));
    set_w(IO_BANK0_PROC0_INTE(GPIO_INT_REG(ENCODER_RIGHT_PIN)),
          GPIO_EDGE_HIGH_BIT(ENCODER_RIGHT_PIN));

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
#endif
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
 * the speed measurement uses; the count is what distance uses.
 *
 * @param[in] intno Interrupt number the kernel dispatched, ENCODER_IRQ_NUM.
 */
#if !MOTION_OPEN_LOOP
static void encoder_isr (UINT intno)
{
    // TODO: Read PROC0_INTS for the bank, increment g_encoder_count_left or
    //       g_encoder_count_right for whichever pin is set, and write the
    //       INTR register to clear it. Nothing else belongs in here.
    uint32_t now       = car_hw_usec();
    UW       left_bit  = GPIO_EDGE_HIGH_BIT(ENCODER_LEFT_PIN);
    UW       right_bit = GPIO_EDGE_HIGH_BIT(ENCODER_RIGHT_PIN);
    UW       left_reg  = GPIO_INT_REG(ENCODER_LEFT_PIN);
    UW       right_reg = GPIO_INT_REG(ENCODER_RIGHT_PIN);

    if (0u != (in_w(IO_BANK0_PROC0_INTS(left_reg)) & left_bit))
    {
        g_encoder_count_left++;
        g_interval_usec_left = now - g_pulse_usec_left;
        g_pulse_usec_left    = now;
        out_w(IO_BANK0_INTR(left_reg), left_bit);
    }

    if (0u != (in_w(IO_BANK0_PROC0_INTS(right_reg)) & right_bit))
    {
        g_encoder_count_right++;
        g_interval_usec_right = now - g_pulse_usec_right;
        g_pulse_usec_right    = now;
        out_w(IO_BANK0_INTR(right_reg), right_bit);
    }

    ClearInt(intno);
}
#endif /* !MOTION_OPEN_LOOP */

#endif /* CAR_HOST_TEST */

/*** end of file ***/
