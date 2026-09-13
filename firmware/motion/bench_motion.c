/** @file bench_motion.c
 *
 * @brief Drives a fixed distance and prints what the encoders counted.
 *
 * Flash this to a Pico with only the motion hardware attached. Compare the
 * commanded distance to the pulse counts to set WHEEL_CIRCUMFERENCE_MM and
 * ENCODER_SLOTS_PER_REV, and compare left to right for a duty mismatch.
 */

#include <inttypes.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "car_config.h"
#include "motion.h"

#define BENCH_DISTANCE_MM    500u
#define BENCH_STARTUP_MSEC  2000u
#define BENCH_IDLE_MSEC      500u

int main (void)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;

    stdio_init_all();
    sleep_ms(BENCH_STARTUP_MSEC);
    printf("motion bench: %u mm forward\n", BENCH_DISTANCE_MM);

    if (CAR_OK != motion_init())
    {
        printf("motion_init failed\n");
    }
    else
    {
        (void)motion_set_speed(MOTION_DEFAULT_SPEED_MM_PER_SEC);
        (void)motion_move_forward(BENCH_DISTANCE_MM);

        while (motion_is_busy())
        {
            (void)motion_tick();
            sleep_ms(MOTION_TICK_PERIOD_MSEC);
        }

        (void)motion_get_encoder_counts(&left, &right);
        printf("left %" PRIu32 " right %" PRIu32 " pulses\n", left, right);
    }

    for (;;)
    {
        sleep_ms(BENCH_IDLE_MSEC);
    }
}

/*** end of file ***/

