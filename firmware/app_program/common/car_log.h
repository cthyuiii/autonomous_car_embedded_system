/** @file car_log.h
 *
 * @brief One logging entry point so the console path lives in one place.
 *
 * NOTE: This is a macro, which the coding standard normally forbids, because
 * the kernel console tm_printf() has no va_list entry point to forward to. A
 * function would have to format into a fixed buffer first, and a fixed
 * buffer is a truncation bug waiting to happen. The level test is a compile
 * time constant, so a disabled level costs nothing.
 *
 * Owner: the team. Finished; nothing to edit here.
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

#ifdef CAR_HOST_TEST
#include <stdio.h>
#define CAR_LOG_PRINT(...) printf(__VA_ARGS__)
#else
#include <tm/tmonitor.h>
/* The kernel's format string type is UB const *, hence the cast. */
#define CAR_LOG_PRINT(...) tm_printf((UB const *)__VA_ARGS__)
#endif

/**
 * @brief Print a formatted line if level is at or below CAR_LOG_LEVEL.
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

