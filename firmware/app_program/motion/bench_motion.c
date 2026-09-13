/** @file bench_motion.c
 *
 * @brief Drives a fixed distance and prints what the encoders counted.
 *
 * Build with `make BENCH=motion` and flash to a Pico with only the motion
 * hardware attached. Compare the commanded distance to the pulse counts to
 * set WHEEL_CIRCUMFERENCE_MM and ENCODER_SLOTS_PER_REV, and compare left to
 * right for a duty mismatch.
 *
 * Owner: Buddy 2, motion control. Extend it as you need; nothing else depends
 * on it.
 */

#include <stdint.h>

#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "motion.h"

#define BENCH_DISTANCE_MM    500u
#define BENCH_STARTUP_MSEC  2000u

INT usermain (void)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;

    (void)tk_dly_tsk(BENCH_STARTUP_MSEC);
    CAR_LOG(CAR_LOG_INFO, "motion bench: %u mm forward\n", BENCH_DISTANCE_MM);

    if (CAR_OK != motion_init())
    {
        CAR_LOG(CAR_LOG_ERROR, "motion_init failed\n");
    }
    else
    {
        (void)motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC);
        (void)motion_move_forward(BENCH_DISTANCE_MM);

        while (motion_is_busy())
        {
            (void)motion_tick();
            (void)tk_dly_tsk(MOTION_TICK_PERIOD_MSEC);
        }

        (void)motion_get_encoder_counts(&left, &right);
        CAR_LOG(CAR_LOG_INFO, "left %u right %u pulses\n", left, right);
    }

    /* The initial task must never return: the kernel shuts down if it does. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}

/*** end of file ***/

