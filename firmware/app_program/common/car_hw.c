/** @file car_hw.c
 *
 * @brief The RP2040 hardware our code drives, through the Pico C SDK.
 *
 * Header-only SDK parts only. The SDK's gpio.c and irq.c would bring the
 * SDK's own interrupt layer and vector table, which cannot share the
 * hardware with the kernel's, so pin functions and pulls are set through
 * the SDK's register structs rather than gpio_set_function() and
 * gpio_set_pulls().
 *
 * NOTE: No kernel header may be included here, see car_hw.h. The kernel's
 * BSP also has a pwm_set_wrap() and a pwm_set_enabled() of its own.
 *
 * Owner: the team.
 */

#include "car_hw.h"

#include "hardware/address_mapped.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/watchdog.h"

/* One watchdog tick per microsecond from clk_ref, the 12 MHz crystal. */
#define CAR_HW_TICK_CYCLES       12u
/* IO_BANK0 interrupt registers hold four event bits for each of 8 GPIO. */
#define CAR_HW_GPIO_PER_IRQ_REG   8u
#define CAR_HW_IRQ_BITS_PER_GPIO  4u
/* A pad the way the kernel's I2C driver sets one up. */
#define CAR_HW_I2C_PAD  (PADS_BANK0_GPIO0_IE_BITS                            \
                         | (PADS_BANK0_GPIO0_DRIVE_VALUE_4MA                 \
                            << PADS_BANK0_GPIO0_DRIVE_LSB)                   \
                         | PADS_BANK0_GPIO0_PUE_BITS                         \
                         | PADS_BANK0_GPIO0_SCHMITT_BITS)

static void     set_function (uint32_t pin, gpio_function_t function);
static uint32_t rise_bit (uint32_t pin);

void car_hw_enable_pwm (void)
{
    unreset_block_wait(RESETS_RESET_PWM_BITS);
}

void car_hw_enable_timer (void)
{
    unreset_block_wait(RESETS_RESET_TIMER_BITS);

    if (0u == (watchdog_hw->tick & WATCHDOG_TICK_CYCLES_BITS))
    {
        watchdog_hw->tick = WATCHDOG_TICK_ENABLE_BITS | CAR_HW_TICK_CYCLES;
    }
}

uint32_t car_hw_usec (void)
{
    /* What the SDK's time_us_32() reads; its header is not warning clean
     * under -Wextra. */
    return timer_hw->timerawl;
}

void car_hw_gpio_input (uint32_t pin)
{
    gpio_set_dir(pin, GPIO_IN);
    hw_set_bits(&pads_bank0_hw->io[pin],
                PADS_BANK0_GPIO0_OD_BITS | PADS_BANK0_GPIO0_IE_BITS);
    set_function(pin, GPIO_FUNC_SIO);
}

void car_hw_gpio_input_pullup (uint32_t pin)
{
    car_hw_gpio_input(pin);
    hw_write_masked(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_PUE_BITS,
                    PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

void car_hw_gpio_input_pulldown (uint32_t pin)
{
    car_hw_gpio_input(pin);
    hw_write_masked(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_PDE_BITS,
                    PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

void car_hw_gpio_output (uint32_t pin)
{
    gpio_set_dir(pin, GPIO_OUT);
    hw_clear_bits(&pads_bank0_hw->io[pin],
                  PADS_BANK0_GPIO0_OD_BITS | PADS_BANK0_GPIO0_IE_BITS);
    set_function(pin, GPIO_FUNC_SIO);
}

bool car_hw_gpio_get (uint32_t pin)
{
    return gpio_get(pin);
}

void car_hw_gpio_put (uint32_t pin, bool b_high)
{
    gpio_put(pin, b_high);
}

void car_hw_gpio_i2c (uint32_t pin)
{
    set_function(pin, GPIO_FUNC_I2C);
    pads_bank0_hw->io[pin] = CAR_HW_I2C_PAD;
}

bool car_hw_gpio_is_i2c (uint32_t pin)
{
    /* Cast: GPIO_FUNC_I2C is a small positive enumerator. */
    return ((uint32_t)GPIO_FUNC_I2C
            == ((io_bank0_hw->io[pin].ctrl
                 & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS)
                >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB));
}

void car_hw_gpio_rise_irq_enable (uint32_t pin)
{
    uint32_t reg = pin / CAR_HW_GPIO_PER_IRQ_REG;

    io_bank0_hw->intr[reg] = rise_bit(pin);
    hw_set_bits(&io_bank0_hw->proc0_irq_ctrl.inte[reg], rise_bit(pin));
}

bool car_hw_gpio_rise_irq_take (uint32_t pin)
{
    uint32_t reg      = pin / CAR_HW_GPIO_PER_IRQ_REG;
    bool     b_edge   = (0u != (io_bank0_hw->proc0_irq_ctrl.ints[reg]
                                & rise_bit(pin)));

    if (b_edge)
    {
        io_bank0_hw->intr[reg] = rise_bit(pin);
    }

    return b_edge;
}

void car_hw_alarm_start (uint32_t alarm, uint32_t delay_usec)
{
    hw_set_bits(&timer_hw->inte, 1u << alarm);
    timer_hw->alarm[alarm] = timer_hw->timerawl + delay_usec;
}

void car_hw_alarm_rearm (uint32_t alarm, uint32_t delay_usec)
{
    /* An alarm matches the timer's low word exactly, so it is armed from a
     * fresh read of the timer, a few instructions before the write. */
    timer_hw->intr         = 1u << alarm;
    timer_hw->alarm[alarm] = timer_hw->timerawl + delay_usec;
}

void car_hw_pwm_setup (uint32_t pin, uint32_t div_int, uint32_t wrap)
{
    uint32_t slice = pwm_gpio_to_slice_num(pin);

    hw_write_masked(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS,
                    PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    set_function(pin, GPIO_FUNC_PWM);
    /* Casts: car_hw.h limits div_int to 1..255 and wrap to 65535. */
    pwm_set_clkdiv_int_frac4(slice, (uint8_t)div_int, 0u);
    pwm_set_wrap(slice, (uint16_t)wrap);
    pwm_set_gpio_level(pin, 0u);
    pwm_set_enabled(slice, true);
}

void car_hw_pwm_level (uint32_t pin, uint32_t level)
{
    pwm_set_gpio_level(pin, (uint16_t)level);   /* At most wrap + 1 */
}

/**
 * @brief Select what drives a pin, as the SDK's gpio_set_function() does
 *        on the RP2040 without the rest of gpio.c.
 *
 * @param[in] pin      GPIO number.
 * @param[in] function GPIO_FUNC_SIO, GPIO_FUNC_PWM or GPIO_FUNC_I2C.
 */
static void set_function (uint32_t pin, gpio_function_t function)
{
    /* Cast: a function number is 0 to 31, the FUNCSEL field. */
    io_bank0_hw->io[pin].ctrl = (uint32_t)function
                                << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

/**
 * @brief A pin's rising edge bit within its IO_BANK0 interrupt register.
 *
 * @param[in] pin GPIO number.
 *
 * @return The bit mask.
 */
static uint32_t rise_bit (uint32_t pin)
{
    /* Cast: GPIO_IRQ_EDGE_RISE is 8, shifted at most 31 places. */
    return (uint32_t)GPIO_IRQ_EDGE_RISE
           << ((pin % CAR_HW_GPIO_PER_IRQ_REG) * CAR_HW_IRQ_BITS_PER_GPIO);
}

/*** end of file ***/
