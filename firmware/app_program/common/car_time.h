/** @file car_time.h
 *
 * @brief The kernel's clock, and the tick every control loop runs on.
 *
 * Kept apart from car_hw.h because it needs a kernel header and car_hw.c
 * may not include one.
 *
 * NOTE: tk_dly_tsk(10) cannot run a loop every 10 ms. A kernel delay waits
 * the ticks asked for plus one, so a loop paced by it runs every 20 ms. A
 * cyclic handler wakes the loops instead, once per kernel tick.
 *
 * Target only. Host tests never compile this file.
 *
 * Owner: the team.
 */

#ifndef CAR_TIME_H
#define CAR_TIME_H

#include <stdint.h>

#include "car.h"

/** The kernel tick, CNF_TIMER_PERIOD in config/config.h. Every control
 *  loop runs once per tick. */
#define CAR_TIME_TICK_MSEC  10u

/**
 * @brief Read the kernel's operating time.
 *
 * @return Milliseconds since the kernel started.
 */
uint32_t car_time_msec (void);

/**
 * @brief Wake the calling task at every kernel tick from now on.
 *
 * Call once, before the task's loop, then car_time_wait_tick() at the end
 * of every pass.
 *
 * @return CAR_OK, CAR_ERR_RANGE if too many tasks asked, or
 *         CAR_ERR_HARDWARE if the kernel refused the cyclic handler.
 */
car_status_t car_time_start_ticks (void);

/**
 * @brief Sleep until the next kernel tick.
 *
 * A pass that overran a tick runs again at once, but only once: missed
 * ticks are dropped, never made up in a burst. If the ticks never arrive,
 * the wait gives up after two ticks, so a loop slows down but never stops.
 */
void car_time_wait_tick (void);

#endif /* CAR_TIME_H */

/*** end of file ***/
