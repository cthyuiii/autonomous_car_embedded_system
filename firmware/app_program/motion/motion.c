/** @file motion.c
 *
 * @brief Motor drive, encoder counting and PID speed control.
 *
 * NOTE: The Robo Pico carries the H-bridge, two PWM pins per motor on the
 * MOTOR_* pins in car_config.h. Drive them with the kernel BSP helpers
 * pwm_set_pin(), pwm_set_wrap(), pwm_set_cc() and pwm_set_enabled().
 */

#include "motion.h"

#include <stddef.h>

#include "car_config.h"

/* Pulse counters written by the encoder interrupt and read by the tick.
 * WARNING: Only read these through motion_get_encoder_counts(). */
static volatile uint32_t g_encoder_count_left  = 0u;
static volatile uint32_t g_encoder_count_right = 0u;

static motion_state_t g_state = { 0 };

static void encoder_isr (uint32_t intno);

car_status_t motion_init (void)
{
    // TODO: pwm_set_pin() each MOTOR_*_PIN, pwm_set_wrap() for
    //       MOTOR_PWM_FREQ_HZ, duty zero, pwm_set_enabled(). Then
    //       gpio_set_pin(GPIO_MODE_IN) both encoder pins, enable their
    //       rising edge in IO_BANK0, and register encoder_isr with
    //       tk_def_int(ENCODER_IRQ_NUM, ...) and EnableInt().
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
        // TODO: Wrap the two reads in the kernel's DI() and EI(), then
        //       return CAR_OK.
        *p_left  = g_encoder_count_left;
        *p_right = g_encoder_count_right;
        status   = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

/**
 * @brief Count one encoder pulse. Kernel interrupt handler for IO_BANK0.
 *
 * NOTE: The RP2040 raises one interrupt for every GPIO in the bank, so
 * this reads the IO_BANK0 status register to learn which encoder pin fired
 * and clears that bit before returning. The kernel calls it with a UINT,
 * which is 32 bits on this core, so uint32_t keeps the host test building.
 *
 * @param[in] intno Interrupt number the kernel dispatched, ENCODER_IRQ_NUM.
 */
static void encoder_isr (uint32_t intno)
{
    (void)intno;

    // TODO: Read PROC0_INTS for the bank, increment g_encoder_count_left or
    //       g_encoder_count_right for whichever pin is set, and write the
    //       INTR register to clear it. Nothing else belongs in here.
}

/*** end of file ***/

