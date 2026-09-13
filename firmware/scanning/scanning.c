/** @file scanning.c
 *
 * @brief Servo sweep, HC-SR04 ranging, profiling, planning and recovery.
 *
 * NOTE: Range in mm is echo microseconds * 10 / SONAR_USEC_PER_CM, using
 * the conversion the HC-SR04 datasheet gives. No floating point needed.
 */

#include "scanning.h"

#include <stddef.h>

#include "car_config.h"

static uint16_t const g_coarse_angles_deg[SCAN_COARSE_ANGLE_COUNT] =
    SCAN_COARSE_ANGLES_DEG;

car_status_t scan_init (void)
{
    // TODO: PWM on SERVO_PIN at SERVO_PWM_FREQ_HZ, gpio_init() the trigger
    //       as output and echo as input, then centre the servo.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t scan_measure (uint16_t angle_deg, uint16_t * p_range_mm)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_range_mm) && (angle_deg <= SERVO_TRAVEL_DEG))
    {
        // TODO: Map angle to a pulse between SERVO_PULSE_MIN_USEC and
        //       SERVO_PULSE_MAX_USEC, wait SERVO_SETTLE_MSEC, pulse the
        //       trigger for SONAR_TRIG_PULSE_USEC, time the echo high with
        //       a SONAR_ECHO_TIMEOUT_USEC cap, and convert to mm.
        *p_range_mm = SONAR_MAX_RANGE_MM;
        status      = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

car_status_t scan_coarse (car_obstacle_profile_t * p_profile)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_profile)
    {
        // TODO: scan_measure() each entry of g_coarse_angles_deg, keep the
        //       nearest, and mark valid if within SCAN_OBSTACLE_RANGE_MM.
        *p_profile = (car_obstacle_profile_t){ 0 };
        status     = CAR_ERR_NOT_IMPLEMENTED;
    }

    (void)g_coarse_angles_deg;

    return status;
}

car_status_t scan_fine (uint16_t start_deg, uint16_t end_deg,
                        car_obstacle_profile_t * p_profile)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_profile) && (start_deg < end_deg)
        && (end_deg <= SERVO_TRAVEL_DEG))
    {
        // TODO: Step from start_deg to end_deg by SCAN_FINE_STEP_DEG. The
        //       obstacle is the run of readings below the coarse range;
        //       width is that run's angular extent times its range, and
        //       clearance is the free distance either side of the run.
        *p_profile = (car_obstacle_profile_t){ 0 };
        status     = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

car_status_t scan_plan_avoidance (car_obstacle_profile_t const * p_profile,
                                  car_avoid_action_t * p_action)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_profile) && (NULL != p_action))
    {
        // TODO: If not valid, CONTINUE. Else pick the side whose clearance
        //       exceeds SCAN_CLEARANCE_MIN_MM by the most, else REVERSE.
        *p_action = CAR_AVOID_STOP;
        status    = CAR_ERR_NOT_IMPLEMENTED;
    }

    return status;
}

car_status_t scan_recover_line (void)
{
    // TODO: Arc back toward the original heading in fixed steps and ask the
    //       line module for a reading after each. Return CAR_OK on a hit,
    //       CAR_ERR_TIMEOUT when the pattern is exhausted.
    return CAR_ERR_NOT_IMPLEMENTED;
}

/*** end of file ***/

