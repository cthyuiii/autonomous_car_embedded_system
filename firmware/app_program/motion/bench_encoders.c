/** @file bench_encoders.c
 *
 * @brief Counts encoder pulses with the motors held off, for calibrating
 *        the encoders by hand.
 *
 * Build and flash with `./flash.sh encoders`. The motors are initialised
 * at zero duty and never driven, so the battery can stay connected, which
 * the encoders may need for their supply.
 *
 * Mark a wheel, turn it exactly one revolution by hand in the direction it
 * rolls when the car drives forward, and read two things off its column:
 *
 * - the change in the count is ENCODER_SLOTS_PER_REV;
 * - the direction must read fwd. If it reads back, flip that wheel's
 *   ENCODER_*_B_FORWARD in car_config.h.
 *
 * A count that moves while nothing turns is electrical noise on that pin.
 *
 * Owner: Buddy 2, motion control. Extend it as you need; nothing else
 * depends on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "motion.h"

#define BENCH_STARTUP_MSEC 2000u
#define BENCH_PRINT_MSEC    500u

INT usermain (void)
{
    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO,
            "encoder bench: left A GP%u B GP%u, right A GP%u B GP%u, "
            "motors held off\n", ENCODER_LEFT_PIN, ENCODER_LEFT_B_PIN,
            ENCODER_RIGHT_PIN, ENCODER_RIGHT_B_PIN);


    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed\n");
    }

    CAR_LOG(CAR_LOG_INFO,
            "turn each wheel one full turn forward by hand\n");

    for (;;)
    {
        motion_state_t state = { 0 };

        (void)motion_get_state(&state);
        CAR_LOG(CAR_LOG_INFO, "left %u %s | right %u %s\n",
                state.encoder_count_left,
                state.b_left_backward ? "back" : "fwd",
                state.encoder_count_right,
                state.b_right_backward ? "back" : "fwd");
        (void)tk_dly_tsk(BENCH_PRINT_MSEC);
    }
}

/*** end of file ***/
