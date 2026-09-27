/** @file car_log.h
 *
 * @brief One logging entry point so the console path lives in one place.
 *
 * NOTE: Barr C deviation 6.3.a: this is a parameterized macro, because the
 * kernel console tm_printf() has no va_list entry point to forward to. The
 * level test is a compile time constant, so a disabled level costs nothing.
 *
 * NOTE on the radio builds. There every line is also published on
 * COMMS_TOPIC_LOG, so a car off its USB cable can still be followed from a
 * laptop; see car_log.c. That needs the line formatted into a buffer
 * first, CAR_LOG_LINE_BYTES long, and tm_sprintf() has no length limit.
 * ponytail: the longest line today is the IMU bench's status line, 279
 * bytes with every field at its widest; a longer format overruns the
 * buffer, so grow it with the format.
 *
 * Owner: the team.
 */

#ifndef CAR_LOG_H
#define CAR_LOG_H

#include "car_config.h"

/** Severity, ordered so that a threshold comparison works. */
typedef enum
{
    CAR_LOG_ERROR = 0,
    CAR_LOG_INFO,
    CAR_LOG_DEBUG
} car_log_level_t;

/* Barr C deviation 6.3.a for CAR_LOG_PRINT and CAR_LOG, see the NOTE. */
#ifdef CAR_HOST_TEST
#include <stdio.h>
#define CAR_LOG_PRINT(...) printf(__VA_ARGS__)
#elif defined(TM_WIFI_MQTT) && TM_WIFI_MQTT
#include <tm/tmonitor.h>
#define CAR_LOG_LINE_BYTES  384

/**
 * @brief Lock the shared line buffer and return it. car_log.c.
 *
 * @return The buffer, CAR_LOG_LINE_BYTES long.
 */
UB * car_log_begin (void);

/**
 * @brief Print the line, publish it on COMMS_TOPIC_LOG, and unlock.
 *
 * @param[in] length What tm_sprintf() wrote, in bytes.
 */
void car_log_end (INT length);

/* Formatted once, into car_log.c's buffer. The kernel's format string type
 * is UB const *, hence the cast. Barr C deviation 6.3.a, see the NOTE. */
#define CAR_LOG_PRINT(...)                                                   \
    do                                                                       \
    {                                                                        \
        car_log_end(tm_sprintf(car_log_begin(), (UB const *)__VA_ARGS__));   \
    } while (0)
#else
#include <tm/tmonitor.h>
/* Barr C deviation 6.3.a, see the NOTE. The kernel's format string type
 * is UB const *, hence the cast. */
#define CAR_LOG_PRINT(...) tm_printf((UB const *)__VA_ARGS__)
#endif

/**
 * @brief Print a formatted line if level is at or below CAR_LOG_LEVEL.
 *        Barr C deviation 6.3.a, see the NOTE at the top.
 *
 * @param[in] level Severity, a car_log_level_t.
 * @param[in] ...   printf style format string and its arguments.
 */
#define CAR_LOG(level, ...)                                                  \
    do                                                                       \
    {                                                                        \
        if ((level) <= (CAR_LOG_LEVEL))                                      \
        {                                                                    \
            CAR_LOG_PRINT(__VA_ARGS__);                                      \
        }                                                                    \
    } while (0)

#endif /* CAR_LOG_H */

/*** end of file ***/

