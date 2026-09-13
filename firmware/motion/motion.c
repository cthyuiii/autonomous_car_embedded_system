/** @file motion.c
 *
 * @brief Motor drive, encoder counting and PID speed control.
 *
 * NOTE: Assumes a DRV8833 style H-bridge with two PWM pins per motor. See
 * the MOTOR_* constants in car_config.h for the wiring this expects.
 */

#include "motion.h"

#include <stddef.h>

#include "car_config.h"

/* Pulse counters written by the encoder interrupt and read by the tick.
 * WARNING: Only read these through motion_get_encoder_counts(). */
static volatile uint32_t g_encoder_count_left  = 0u;
static volatile uint32_t g_encoder_count_right = 0u;

static motion_state_t g_state = { 0 };

static void encoder_isr (unsigned int gpio_num, uint32_t event_mask);

car_status_t motion_init (void)
{
    // TODO: Set MOTOR_*_PIN to PWM at MOTOR_PWM_FREQ_HZ with zero duty.
    // TODO: Register encoder_isr on ENCODER_LEFT_PIN and ENCODER_RIGHT_PIN
    //       for rising edges with gpio_set_irq_enabled_with_callback().
    (void)encoder_isr;

    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_tick (void)
{
    // TODO: Convert the encoder delta since last tick to mm per second,
    //       run the PID with MOTION_PID_*_MILLI, apply duty to each motor,
    //       and clear b_is_busy when the target distance is reached.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_move_forward (uint32_t distance_mm)
{
    (void)distance_mm;

    // TODO: Convert distance_mm to a pulse target using
    //       ENCODER_SLOTS_PER_REV and WHEEL_CIRCUMFERENCE_MM, then set
    //       both motors forward and mark the state busy.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_move_backward (uint32_t distance_mm)
{
    (void)distance_mm;

    // TODO: Same as forward with both motor directions reversed.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_turn_left (uint16_t angle_deg)
{
    (void)angle_deg;

    // TODO: Arc length per wheel is WHEEL_BASE_MM * pi * angle / 360.
    //       Drive wheels in opposite directions for that many pulses.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_turn_right (uint16_t angle_deg)
{
    (void)angle_deg;

    // TODO: Mirror of motion_turn_left().
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_set_speed (uint16_t speed_mm_per_sec)
{
    (void)speed_mm_per_sec;

    // TODO: Reject above MOTION_MAX_SPEED_MM_PER_SEC, else store as the
    //       PID setpoint.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t motion_stop (void)
{
    // TODO: Zero both duties, clear the pulse target and b_is_busy.
    return CAR_ERR_NOT_IMPLEMENTED;
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
        *p_state = g_state;
        status   = CAR_ERR_NOT_IMPLEMENTED;
    }

    // TODO: Refresh encoder counts and distance before copying, then
    //       return CAR_OK.
    return status;
}

car_status_t motion_get_encoder_counts (uint32_t * p_left, uint32_t * p_right)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_left) && (NULL != p_right))
    {
        // TODO: Wrap the two reads in save_and_disable_interrupts() and
        //       restore_interrupts(), then return CAR_OK.
        *p_left  = g_encoder_count_left;
        *p_right = g_encoder_count_right;
        status   = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

/**
 * @brief Count one encoder pulse. Shared callback for both wheels.
 *
 * NOTE: The Pico SDK routes every GPIO interrupt on a core through one
 * callback, so a single ISR dispatches on gpio_num. The signature must
 * match gpio_irq_callback_t exactly, which is why gpio_num is unsigned int.
 *
 * @param[in] gpio_num   Pin that fired, ENCODER_LEFT_PIN or ENCODER_RIGHT_PIN.
 * @param[in] event_mask Edge flags from the SDK, unused for a single edge.
 */
static void encoder_isr (unsigned int gpio_num, uint32_t event_mask)
{
    (void)gpio_num;
    (void)event_mask;

    // TODO: Increment g_encoder_count_left or g_encoder_count_right
    //       depending on gpio_num. Nothing else belongs in here.
}

/*** end of file ***/

