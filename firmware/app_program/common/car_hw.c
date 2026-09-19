/** @file car_hw.c
 *
 * @brief Board level helpers shared by the modules that touch RP2040 blocks.
 *
 * Register addresses that libbsp does not expose are named here with the
 * RP2040 datasheet section they come from.
 *
 * Owner: the team.
 */

#include "car_hw.h"

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

/* Watchdog tick generator, RP2040 datasheet section 4.7.6. The TIMER block
 * counts these ticks. Reset value is enabled with a zero cycle count. */
#define CAR_HW_WATCHDOG_TICK      0x4005802Cu
#define CAR_HW_TICK_ENABLE        (1u << 9)
#define CAR_HW_TICK_CYCLES_MASK   0x1FFu
#define CAR_HW_TICK_CYCLES        12u   /* clk_ref is the 12 MHz crystal */

/* PWM slice divider, RP2040 datasheet section 4.5.3. Integer part in bits
 * 11:4, fraction in bits 3:0. libbsp exposes no setter for it. */
#define CAR_HW_PWM_DIV(slice)     (PWM_BASE + PWM_CHx_DIV + ((slice) * 0x14u))
#define CAR_HW_PWM_DIV_INT_LSB    4u
#define CAR_HW_PWM_SLICE(pin)     (((pin) >> 1u) & 0x07u)

static void release_reset (UW block);

void car_hw_enable_pwm (void)
{
    release_reset(RESETS_RESET_PWM);
}

void car_hw_enable_timer (void)
{
    UW tick = 0u;

    release_reset(RESETS_RESET_TIMER);
    tick = in_w(CAR_HW_WATCHDOG_TICK);

    if (0u == (tick & CAR_HW_TICK_CYCLES_MASK))
    {
        out_w(CAR_HW_WATCHDOG_TICK, CAR_HW_TICK_ENABLE | CAR_HW_TICK_CYCLES);
    }
}

uint32_t car_hw_usec (void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

uint32_t car_hw_msec (void)
{
    SYSTIM now = { 0, 0u };

    (void)tk_get_otm(&now);

    return (uint32_t)now.lo;
}

void car_hw_gpio_input_pullup (uint32_t pin)
{
    (void)gpio_set_pin((UINT)pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

void car_hw_gpio_input_pulldown (uint32_t pin)
{
    (void)gpio_set_pin((UINT)pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PUE);
    set_w(GPIO(pin), GPIO_PDE);
}

void car_hw_pwm_setup (uint32_t pin, uint32_t div_int, uint32_t wrap)
{
    UW slice = CAR_HW_PWM_SLICE(pin);

    (void)pwm_set_pin((UINT)pin);
    out_w(CAR_HW_PWM_DIV(slice), (UW)div_int << CAR_HW_PWM_DIV_INT_LSB);
    (void)pwm_set_wrap((UINT)pin, (UW)wrap);
    (void)pwm_set_cc((UINT)pin, 0u);
    (void)pwm_set_enabled((UINT)pin, TRUE);
}

void car_hw_pwm_level (uint32_t pin, uint32_t level)
{
    (void)pwm_set_cc((UINT)pin, (UW)level);
}

/**
 * @brief Deassert one block's reset and wait for the block to report ready.
 *
 * @param[in] block One RESETS_RESET_* bit.
 */
static void release_reset (UW block)
{
    if (0u == (in_w(RESETS_RESET_DONE) & block))
    {
        clr_w(RESETS_RESET, block);

        while (0u == (in_w(RESETS_RESET_DONE) & block))
        {
        }
    }
}

/*** end of file ***/
