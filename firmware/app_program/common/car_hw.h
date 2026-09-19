/** @file car_hw.h
 *
 * @brief Board level helpers shared by the modules that touch RP2040 blocks.
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
 * Owner: the team. Extend only with helpers that two modules share.
 */

#ifndef CAR_HW_H
#define CAR_HW_H

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
 * @brief Read the kernel's operating time.
 *
 * @return Milliseconds since the kernel started.
 */
uint32_t car_hw_msec (void);

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
