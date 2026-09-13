/** @file car_log.c
 *
 * @brief Thin wrapper over vprintf with a compile time level threshold.
 */

#include "car_log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "car_config.h"

static car_log_level_t const g_log_threshold = CAR_LOG_LEVEL;

void car_log_write (car_log_level_t level, char const * p_format, ...)
{
    bool b_enabled = (level <= g_log_threshold);

#ifdef NDEBUG
    if (CAR_LOG_DEBUG == level)
    {
        b_enabled = false;
    }
#endif

    if (b_enabled)
    {
        va_list args;

        va_start(args, p_format);
        vprintf(p_format, args);
        va_end(args);
    }
}

/*** end of file ***/

