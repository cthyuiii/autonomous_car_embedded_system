/** @file bench_duty.c
 *
 * @brief Finds MOTOR_MIN_DUTY: the lowest duty at which each wheel keeps
 *        turning, and the one at which it starts.
 *
 * Build and flash with `./flash.sh duty`, battery connected, car ON the
 * floor with about a metre clear ahead. It drives both wheels forward at a
 * raw duty, bypassing the speed loop and the floor, and prints one row per
 * step:
 *
 *   1. Up from 0 in BENCH_DUTY_STEP steps until both wheels have been
 *      turning for BENCH_EXTRA_STEPS, which finds where each starts.
 *   2. Back down to 0, which finds where each stops. A turning motor keeps
 *      going below the duty it needed to start.
 *
 * Set MOTOR_MIN_DUTY to the higher of the two "stops below" duties. The
 * speed loop pushes past it to start a wheel from rest, so the floor only
 * has to stop it asking for a duty that just hums. The speed in that row
 * is the slowest the car can hold; MOTION_TURN_SPEED_MM_PER_SEC and
 * CAR_BARCODE_SPEED_MM_PER_SEC can come down to just above it.
 *
 * Owner: Buddy 2, motion control. Extend it as you need; nothing else
 * depends on it.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_hw.h"
#include "car_log.h"
#include "car_time.h"
#include "motion.h"

#define BENCH_STARTUP_MSEC     3000u
#define BENCH_HOLD_MSEC         500u   /* Per duty step */
#define BENCH_DUTY_STEP          10u   /* Per mille */
#define BENCH_DUTY_LIMIT        500u   /* Never ramps past this */
#define BENCH_EXTRA_STEPS         3u   /* Both turning this many steps */
/* 1/20 of a turn per step, about 9 mm: well above the stray pulses a
 * noisy encoder shows standing still. */
#define BENCH_TURNING_PULSES   (ENCODER_SLOTS_PER_REV / 20u)
/* The same period motion.c gives the motor slices. */
#define BENCH_PWM_PERIOD       (CAR_HW_SYS_CLOCK_HZ / MOTOR_PWM_FREQ_HZ)

static void step_duty (uint16_t duty, bool * p_left, bool * p_right);
static void set_duty (uint16_t duty);

INT usermain (void)
{
    uint16_t duty          = 0u;
    uint16_t left_start    = 0u;
    uint16_t right_start   = 0u;
    uint16_t left_stop     = 0u;
    uint16_t right_stop    = 0u;
    uint8_t  extra         = 0u;
    bool     b_left        = false;
    bool     b_right       = false;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    (void)car_time_start_ticks();

    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed\n");
    }

    CAR_LOG(CAR_LOG_INFO, "duty bench: car ON the floor, a metre clear "
            "ahead. MOTOR_MIN_DUTY now %u\n", MOTOR_MIN_DUTY);
    CAR_LOG(CAR_LOG_INFO, "| Duty per mille | Left mm/s | Right mm/s |\n");
    CAR_LOG(CAR_LOG_INFO, "|---|---|---|\n");

    /* Up until both have turned for a few steps. */
    while ((extra < BENCH_EXTRA_STEPS) && (duty < BENCH_DUTY_LIMIT))
    {
        duty = (uint16_t)(duty + BENCH_DUTY_STEP);   /* Cast: under 1000 */
        step_duty(duty, &b_left, &b_right);

        if (b_left && (0u == left_start))
        {
            left_start = duty;
        }

        if (b_right && (0u == right_start))
        {
            right_start = duty;
        }

        if ((0u != left_start) && (0u != right_start))
        {
            extra++;
        }
    }

    /* Back down; the last duty each still turned at is where it stops. */
    while (duty > 0u)
    {
        step_duty(duty, &b_left, &b_right);

        if (b_left)
        {
            left_stop = duty;
        }

        if (b_right)
        {
            right_stop = duty;
        }

        duty = (uint16_t)(duty - BENCH_DUTY_STEP);   /* Cast: stays >= 0 */
    }

    set_duty(0u);
    CAR_LOG(CAR_LOG_INFO, "left starts at %u, keeps turning down to %u\n",
            left_start, left_stop);
    CAR_LOG(CAR_LOG_INFO, "right starts at %u, keeps turning down to %u\n",
            right_start, right_stop);
    CAR_LOG(CAR_LOG_INFO, "set MOTOR_MIN_DUTY to %u. 0 means that wheel "
            "never turned below %u: check it with ./flash.sh motion\n",
            (left_stop > right_stop) ? left_stop : right_stop,
            BENCH_DUTY_LIMIT);
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}

/**
 * @brief Hold one duty for BENCH_HOLD_MSEC and print the speed it gave.
 *
 * @param[in]  duty    Per mille, both wheels forward.
 * @param[out] p_left  true if the left wheel was turning.
 * @param[out] p_right true if the right wheel was turning.
 */
static void step_duty (uint16_t duty, bool * p_left, bool * p_right)
{
    motion_state_t before = { 0 };
    motion_state_t after  = { 0 };
    uint32_t       start  = 0u;
    uint32_t       msec   = 0u;
    int32_t        left   = 0;
    int32_t        right  = 0;

    set_duty(duty);
    (void)tk_dly_tsk(BENCH_HOLD_MSEC / 2u);     /* Let the speed settle */
    (void)motion_get_state(&before);
    start = car_time_msec();
    (void)tk_dly_tsk(BENCH_HOLD_MSEC / 2u);
    (void)motion_get_state(&after);
    msec = car_time_msec() - start;

    /* Casts: a half step's pulses fit int32_t either way round. */
    left  = (int32_t)(after.encoder_count_left - before.encoder_count_left);
    right = (int32_t)(after.encoder_count_right - before.encoder_count_right);
    left  = (left < 0) ? -left : left;
    right = (right < 0) ? -right : right;

    /* Doubled because the pulses were counted over half the step. */
    *p_left  = ((2 * left) >= (int32_t)BENCH_TURNING_PULSES);
    *p_right = ((2 * right) >= (int32_t)BENCH_TURNING_PULSES);

    /* Casts: small positive counts and constants, exact in int32_t. */
    CAR_LOG(CAR_LOG_INFO, "| %u | %d | %d |\n", duty,
            (left * (int32_t)WHEEL_CIRCUMFERENCE_MM * 1000)
            / ((int32_t)ENCODER_SLOTS_PER_REV * (int32_t)msec),
            (right * (int32_t)WHEEL_CIRCUMFERENCE_MM * 1000)
            / ((int32_t)ENCODER_SLOTS_PER_REV * (int32_t)msec));
}

/**
 * @brief Drive both wheels forward at a raw duty, as motion.c's forward
 *        direction does: IN1 pulsed, IN2 held low.
 *
 * @param[in] duty Per mille.
 */
static void set_duty (uint16_t duty)
{
    /* Cast: widening. A per mille duty times the period fits uint32_t. */
    uint32_t level = ((uint32_t)duty * BENCH_PWM_PERIOD) / MOTOR_PWM_MAX_DUTY;

    car_hw_pwm_level(MOTOR_LEFT_IN1_PIN, level);
    car_hw_pwm_level(MOTOR_LEFT_IN2_PIN, 0u);
    car_hw_pwm_level(MOTOR_RIGHT_IN1_PIN, level);
    car_hw_pwm_level(MOTOR_RIGHT_IN2_PIN, 0u);
}

/*** end of file ***/
