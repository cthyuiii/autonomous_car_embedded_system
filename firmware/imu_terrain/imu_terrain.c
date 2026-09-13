/** @file imu_terrain.c
 *
 * @brief LSM303DLHC driver plus tilt, hump and event estimation.
 *
 * NOTE: Register addresses and scale factors come from the LSM303DLHC
 * datasheet, sections on CTRL_REG1_A, CRA_REG_M and the output registers.
 * Cite the section beside each constant when adding it.
 */

#include "imu_terrain.h"

#include <stddef.h>

#include "car_config.h"

static car_hump_t g_peak_hump = { 0 };

car_status_t imu_init (void)
{
    // TODO: i2c_init() on IMU_I2C_INDEX at IMU_I2C_BAUD_HZ, set the pins,
    //       then write the control registers of both devices and read back
    //       a known register to confirm each one answers.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t imu_calibrate (void)
{
    // TODO: Average N samples for the gravity reference. Rotate the car
    //       through a full turn with motors on and record magnetometer min
    //       and max per axis for the hard iron offset.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t imu_update (void)
{
    // TODO: Read six accelerometer and six magnetometer bytes, convert to
    //       milli g and milli gauss, low pass filter, derive pitch and
    //       heading, run the event classifier, and track the hump peak.
    return CAR_ERR_NOT_IMPLEMENTED;
}

car_status_t imu_get_orientation (int16_t * p_pitch_deg,
                                  int16_t * p_heading_deg)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_pitch_deg) && (NULL != p_heading_deg))
    {
        *p_pitch_deg   = 0;
        *p_heading_deg = 0;
        status         = CAR_ERR_NOT_IMPLEMENTED;
    }

    // TODO: Copy the filtered values from imu_update().
    return status;
}

car_status_t imu_get_event (car_motion_event_t * p_event)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_event)
    {
        *p_event = CAR_MOTION_STATIONARY;
        status   = CAR_ERR_NOT_IMPLEMENTED;
    }

    // TODO: Threshold the filtered magnitude, pitch and turn rate into
    //       one of the car_motion_event_t classes with hysteresis.
    return status;
}

car_status_t imu_get_turn_rate_dps (int16_t * p_rate_dps)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_rate_dps)
    {
        *p_rate_dps = 0;
        status      = CAR_ERR_NOT_IMPLEMENTED;
    }

    // TODO: If IMU_TURN_RATE_FROM_ENCODERS, take the wheel speed difference
    //       from motion_get_state() through the controller, else
    //       differentiate the heading. See the header for the tradeoff.
    return status;
}

bool imu_is_hump_detected (void)
{
    // TODO: True while pitch magnitude exceeds IMU_HUMP_PITCH_THRESHOLD_DEG.
    return false;
}

car_status_t imu_get_peak_hump (car_hump_t * p_hump)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_hump)
    {
        *p_hump = g_peak_hump;
        status  = CAR_ERR_NOT_IMPLEMENTED;
    }

    // TODO: Return CAR_OK once imu_update() maintains g_peak_hump.
    return status;
}

bool imu_is_collision_detected (void)
{
    // TODO: Latch true when the acceleration magnitude minus 1 g exceeds
    //       IMU_COLLISION_THRESHOLD_MILLI_G, clear on read.
    return false;
}

/*** end of file ***/

