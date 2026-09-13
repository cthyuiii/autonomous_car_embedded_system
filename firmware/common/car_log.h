/** @file car_log.h
 *
 * @brief One logging entry point so output format lives in a single place.
 */

#ifndef CAR_LOG_H
#define CAR_LOG_H

/** Severity, ordered so that a threshold comparison works. */
typedef enum
{
    CAR_LOG_ERROR = 0,
    CAR_LOG_INFO,
    CAR_LOG_DEBUG
} car_log_level_t;

/**
 * @brief Print a formatted line if level is at or below CAR_LOG_LEVEL.
 *
 * NOTE: Debug messages are compiled out entirely when NDEBUG is defined.
 *
 * @param[in] level    Severity of this message.
 * @param[in] p_format printf style format string, newline not implied.
 */
void car_log_write (car_log_level_t level, char const * p_format, ...);

#endif /* CAR_LOG_H */

/*** end of file ***/

