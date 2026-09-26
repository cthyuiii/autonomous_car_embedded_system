/** @file car_hw.h
 *
 * @brief The RP2040 hardware our code drives, through the Pico C SDK.
 *
 * Every register access in our code goes through here, and car_hw.c does
 * it with the Pico C SDK's hardware API. Only the SDK's header-only parts
 * are used: the register structs and the inline gpio, pwm and reset
 * helpers. The kernel keeps what it owns: interrupt registration, task
 * timing and its device drivers, I2C among them.
 *
 * NOTE: car_hw.c cannot include a kernel header: the kernel and the SDK
 * each define size_t and the block base addresses. So this header uses
 * only standard types, and the one helper that needs the kernel, the
 * millisecond clock, lives in car_time.h.
 *
 * NOTE: The kernel releases only the blocks it uses from reset. The PWM and
 * TIMER blocks are still held in reset when usermain() runs, so every
 * module that needs one calls the matching enable here. Each call is
 * idempotent: a block that is already running is left alone, which matters
 * because motion and scanning both use PWM and neither may reset the other.
 *
 * Target only. Host tests never compile this file; a module includes it
 * inside #ifndef CAR_HOST_TEST and keeps its own fake for the host.
 *
 * Owner: the team.
 */

#ifndef CAR_HW_H
#define CAR_HW_H

#include <stdbool.h>
#include <stdint.h>

/** System clock the PWM slices count, set by the kernel's clock setup. */
#define CAR_HW_SYS_CLOCK_HZ    125000000u

/**
 * @brief Release the PWM block from reset if it still is.
 */
void car_hw_enable_pwm (void);

/**
 * @brief Release the TIMER block from reset and make it count microseconds.
 *
 * NOTE: The timer counts watchdog ticks. The tick generator resets to a
 * zero cycle count, which would make the timer run at the crystal rate, so
 * this sets it to one tick per microsecond unless someone already has.
 */
void car_hw_enable_timer (void);

/**
 * @brief Read the free running microsecond timer, wraps every 71 minutes.
 *
 * @return Microseconds since the timer was enabled.
 */
uint32_t car_hw_usec (void);

/**
 * @brief Make a pin a digital input, leaving its pulls as they are.
 *
 * @param[in] pin GPIO number.
 */
void car_hw_gpio_input (uint32_t pin);

/**
 * @brief Make a pin a digital input with the pull up on and pull down off.
 *
 * NOTE: Open collector sensor outputs need the pull up. A push pull output
 * does not mind it.
 *
 * @param[in] pin GPIO number.
 */
void car_hw_gpio_input_pullup (uint32_t pin);

/**
 * @brief Make a pin a digital input with the pull down on and pull up off.
 *
 * NOTE: Use this where a disconnected wire must read as 0 rather than 1.
 * A module that drives its output both ways overrides the pull; one that
 * only pulls low needs car_hw_gpio_input_pullup() instead.
 *
 * @param[in] pin GPIO number.
 */
void car_hw_gpio_input_pulldown (uint32_t pin);

/**
 * @brief Make a pin a digital output.
 *
 * @param[in] pin GPIO number.
 */
void car_hw_gpio_output (uint32_t pin);

/**
 * @brief Read a pin.
 *
 * @param[in] pin GPIO number.
 *
 * @return true while the pin is high.
 */
bool car_hw_gpio_get (uint32_t pin);

/**
 * @brief Drive an output pin.
 *
 * @param[in] pin    GPIO number set up with car_hw_gpio_output().
 * @param[in] b_high true drives it high.
 */
void car_hw_gpio_put (uint32_t pin, bool b_high);

/**
 * @brief Route a pin to its I2C block, with the pad set up the way the
 *        kernel's I2C driver sets up its own pins.
 *
 * @param[in] pin GPIO number.
 */
void car_hw_gpio_i2c (uint32_t pin);

/**
 * @brief Whether a pin is routed to an I2C block.
 *
 * @param[in] pin GPIO number.
 *
 * @return true if its function is I2C.
 */
bool car_hw_gpio_is_i2c (uint32_t pin);

/**
 * @brief Latch rising edges on a pin for processor 0's IO_BANK0 interrupt.
 *
 * Clears an edge latched while the pin was being set up first. The
 * interrupt itself is registered with the kernel by the caller.
 *
 * @param[in] pin GPIO number.
 */
void car_hw_gpio_rise_irq_enable (uint32_t pin);

/**
 * @brief Take a latched rising edge on a pin, if there is one.
 *
 * @param[in] pin GPIO number set up with car_hw_gpio_rise_irq_enable().
 *
 * @return true if an edge was pending; it is cleared.
 */
bool car_hw_gpio_rise_irq_take (uint32_t pin);

/**
 * @brief Enable a TIMER alarm's interrupt and arm it.
 *
 * The interrupt itself is registered with the kernel by the caller.
 *
 * @param[in] alarm      Alarm number, 0 to 3.
 * @param[in] delay_usec Time from now until it fires.
 */
void car_hw_alarm_start (uint32_t alarm, uint32_t delay_usec);

/**
 * @brief From an alarm's interrupt: acknowledge it and arm it again.
 *
 * @param[in] alarm      Alarm number, 0 to 3.
 * @param[in] delay_usec Time from now until it next fires.
 */
void car_hw_alarm_rearm (uint32_t alarm, uint32_t delay_usec);

/**
 * @brief Route a pin to its PWM slice and start the slice.
 *
 * The slice counts the system clock divided by div_int from 0 to wrap, so
 * the output period is (wrap + 1) * div_int / CAR_HW_SYS_CLOCK_HZ. Both pins
 * of a slice share div_int and wrap; the level is set per pin.
 *
 * @param[in] pin     GPIO number.
 * @param[in] div_int Integer clock divider, 1 to 255.
 * @param[in] wrap    Counter top value, up to 65535.
 */
void car_hw_pwm_setup (uint32_t pin, uint32_t div_int, uint32_t wrap);

/**
 * @brief Set the compare level of one PWM pin. The pin is high while the
 *        counter is below level, so level 0 is off and wrap + 1 is fully on.
 *
 * @param[in] pin   GPIO number already set up with car_hw_pwm_setup().
 * @param[in] level Compare value, 0 to wrap + 1.
 */
void car_hw_pwm_level (uint32_t pin, uint32_t level);

#endif /* CAR_HW_H */

/*** end of file ***/
