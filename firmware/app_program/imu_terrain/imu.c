/** @file imu.c
 *
 * @brief LSM303DLHC driver plus tilt, hump and event estimation.
 *
 * NOTE: Register addresses and scale factors come from the LSM303DLHC
 * datasheet (STMicroelectronics DocID018771), section 7 register
 * description. Each constant names its register.
 *
 * NOTE: All maths is integer. Angles use an octant reduced polynomial for
 * atan, good to a tenth of a degree, and the hump integral uses the small
 * angle sine, which is 5 percent low at 30 degrees and exact at zero.
 *
 * Owner: Buddy 4, IMU based motion and terrain monitoring. This file is
 * yours.
 */

#include "imu.h"

#ifdef CAR_HOST_TEST
#include <stddef.h>
#else
#include <tk/tkernel.h>
#include <tk/device.h>
#include "car_hw.h"
#endif

#include "car_config.h"

/* Accelerometer registers, datasheet section 7.1. */
#define LSM_CTRL_REG1_A          0x20u
#define LSM_CTRL_REG4_A          0x23u
#define LSM_OUT_X_L_A            0x28u
#define LSM_AUTO_INCREMENT       0x80u
#define LSM_CTRL_REG1_A_100HZ    0x57u   /* ODR 100 Hz, X Y Z enabled */
/* Block update, high resolution, and the full scale from car_config.h.
 * WARNING: at the datasheet default of plus or minus 2 g the sensor
 * cannot report more than 2000 milli g on an axis, so any collision
 * threshold above about 1 g over gravity can never be reached and
 * impacts are simply invisible. The full scale has to be wide enough for
 * the knock you want to catch. */
#define LSM_CTRL_REG4_A_BASE     0x88u   /* Block update, high resolution */
#define LSM_CTRL_REG4_A_FS_SHIFT    4u
#define LSM_CTRL_REG4_A_HR_2G \
    ((uint8_t)(LSM_CTRL_REG4_A_BASE \
               | (IMU_ACCEL_FS_SELECT << LSM_CTRL_REG4_A_FS_SHIFT)))
#define LSM_ACCEL_SHIFT          4u      /* 12 bit left justified */

/* Magnetometer registers, datasheet section 7.2. Output order is X, Z, Y. */
#define LSM_CRA_REG_M            0x00u
#define LSM_CRB_REG_M            0x01u
#define LSM_MR_REG_M             0x02u
#define LSM_OUT_X_H_M            0x03u
#define LSM_IRA_REG_M            0x0Au
#define LSM_CRA_REG_M_75HZ       0x18u
#define LSM_CRB_REG_M_1_3_GAUSS  0x20u
#define LSM_MR_REG_M_CONTINUOUS  0x00u
#define LSM_IRA_REG_M_VALUE      0x48u   /* Reads as ASCII 'H' */

#define IMU_AXES                 3u
#define IMU_SIDE_AXIS            (1u - IMU_FORWARD_AXIS)
#define IMU_UP_AXIS              2u
#define IMU_GRAVITY_MILLI_G      1000
#define IMU_DEGREES_PER_RADIAN_MILLI 57296
#define IMU_MILLI_RAD_PER_DEG    17453   /* Small angle sine, times 1000 */
#define IMU_MAG_MIN_SPREAD       200     /* Counts before the offset counts */

typedef struct
{
    int16_t accel_mg[IMU_AXES];
    int16_t mag[IMU_AXES];
} imu_raw_t;

static int32_t  g_accel_mg[IMU_AXES]     = { 0 };
static int32_t  g_mag[IMU_AXES]          = { 0 };
static int32_t  g_mag_min[IMU_AXES]      = { 0 };
static int32_t  g_mag_max[IMU_AXES]      = { 0 };
static int16_t  g_pitch_ref_deg          = 0;
static int32_t  g_accel_ref_fwd_mg       = 0;
static int16_t  g_pitch_deg              = 0;
static uint16_t g_accel_milli_g          = IMU_GRAVITY_MILLI_G;
static bool     gb_pitch_trusted          = false;
static bool     gb_calibrated            = false;
static uint16_t g_roughness_milli_g      = 0u;
static int16_t  g_heading_deg            = 0;
static int16_t  g_turn_rate_dps          = 0;
static int16_t  g_wheel_left             = 0;
static int16_t  g_wheel_right            = 0;
static uint32_t g_distance_mm            = 0u;
static uint32_t g_prev_distance_mm       = 0u;
static car_motion_event_t g_event        = CAR_MOTION_STATIONARY;
static uint8_t  g_event_hold             = 0u;
static bool     gb_collision_latched      = false;
static uint16_t g_accel_peak_mg          = IMU_GRAVITY_MILLI_G;
/* What this sensor reads for one g, taken level and still by
 * imu_calibrate(). Not every module reads 1000: one on this car reads
 * about 794, which against a fixed 1000 looked like constant shaking,
 * reported ROUGH at rest and left the pitch trust band almost no room. */
static uint16_t g_gravity_mg             = IMU_GRAVITY_MILLI_G;
static bool     gb_on_hump                = false;
static int32_t  g_hump_milli_mm          = 0;
static int16_t  g_hump_pitch_deg         = 0;   /* At the last trusted sample */
static uint16_t g_hump_count             = 0u;
static uint16_t g_last_hump_mm           = 0u;
static uint16_t g_peak_hump_mm           = 0u;

static int16_t  atan2_deg (int32_t y_axis, int32_t x_axis);
static int32_t  atan_ratio_100 (int32_t num, int32_t den);
static uint32_t isqrt (uint32_t value);
static int16_t  pitch_from (int32_t fwd_mg, int32_t side_mg, int32_t up_mg);
static int32_t  difference_from_gravity (uint16_t milli_g);
static void     filter_sample (imu_raw_t const * p_raw);
static void     update_heading (void);
static void     update_hump (void);
static void     end_hump (void);
static void     update_event (void);
static bool     hw_init (void);
static bool     hw_read (imu_raw_t * p_raw);
static void     hw_delay_msec (uint32_t msec);

car_status_t imu_init (void)
{
    car_status_t status = CAR_ERR_HARDWARE;

    g_pitch_deg         = 0;
    g_accel_milli_g     = IMU_GRAVITY_MILLI_G;
    gb_pitch_trusted     = false;
    gb_calibrated       = false;
    g_roughness_milli_g = 0u;
    g_heading_deg       = 0;
    g_turn_rate_dps     = 0;
    g_event             = CAR_MOTION_STATIONARY;
    g_event_hold        = 0u;
    gb_collision_latched = false;
    g_accel_peak_mg     = IMU_GRAVITY_MILLI_G;
    g_gravity_mg        = IMU_GRAVITY_MILLI_G;
    gb_on_hump           = false;
    g_hump_milli_mm     = 0;
    g_hump_pitch_deg    = 0;
    g_hump_count        = 0u;
    g_distance_mm       = 0u;
    g_prev_distance_mm  = 0u;
    g_last_hump_mm      = 0u;
    g_peak_hump_mm      = 0u;

    if (hw_init())
    {
        status = CAR_OK;
    }

    return status;
}

car_status_t imu_calibrate (void)
{
    car_status_t status = CAR_OK;
    int32_t      sum[IMU_AXES] = { 0 };
    imu_raw_t    raw = { { 0 }, { 0 } };
    uint32_t     sample = 0u;
    uint32_t     axis   = 0u;
    uint32_t     magnitude = 0u;

    for (sample = 0u; (sample < IMU_CALIBRATION_SAMPLES)
                      && (CAR_OK == status); sample++)
    {
        if (hw_read(&raw))
        {
            for (axis = 0u; axis < IMU_AXES; axis++)
            {
                sum[axis] += raw.accel_mg[axis];
            }

            hw_delay_msec(IMU_SAMPLE_PERIOD_MSEC);
        }
        else
        {
            status = CAR_ERR_HARDWARE;
        }
    }

    if (CAR_OK == status)
    {
        for (axis = 0u; axis < IMU_AXES; axis++)
        {
            /* Cast: a small positive constant, exact as int32_t. */
            g_accel_mg[axis] = sum[axis] / (int32_t)IMU_CALIBRATION_SAMPLES;
            g_mag[axis]      = raw.mag[axis];
            g_mag_min[axis]  = raw.mag[axis];
            g_mag_max[axis]  = raw.mag[axis];
        }

        /* Level and still means the vector is close to one g. Anything far
         * from that is a wrong scale, a moving car, or a dead sensor. */
        magnitude = isqrt((uint32_t)(g_accel_mg[0] * g_accel_mg[0])
                          + (uint32_t)(g_accel_mg[1] * g_accel_mg[1])
                          + (uint32_t)(g_accel_mg[2] * g_accel_mg[2]));

        if ((magnitude < 700u) || (magnitude > 1300u))
        {
            status = CAR_ERR_RANGE;
        }
        else
        {
            g_pitch_ref_deg  = pitch_from(g_accel_mg[IMU_FORWARD_AXIS],
                                          g_accel_mg[IMU_SIDE_AXIS],
                                          g_accel_mg[IMU_UP_AXIS]);
            g_accel_ref_fwd_mg = g_accel_mg[IMU_FORWARD_AXIS];
            g_gravity_mg     = (uint16_t)magnitude;   /* 700 to 1300 here */
            g_accel_peak_mg  = g_gravity_mg;
            gb_calibrated    = true;
        }
    }

    return status;
}

car_status_t imu_update (void)
{
    car_status_t status = CAR_ERR_HARDWARE;
    imu_raw_t    raw    = { { 0 }, { 0 } };

    if (hw_read(&raw))
    {
        int32_t magnitude = 0;

        /* Impacts are sharp, so the collision test uses the raw sample. */
        magnitude = (int32_t)isqrt(
            (uint32_t)((int32_t)raw.accel_mg[0] * raw.accel_mg[0])
            + (uint32_t)((int32_t)raw.accel_mg[1] * raw.accel_mg[1])
            + (uint32_t)((int32_t)raw.accel_mg[2] * raw.accel_mg[2]));

        /* Casts: the magnitude of three 8 g axes is under 14000 mg, so
         * it fits a uint16_t, and a uint16_t fits an int32_t. */
        if (magnitude > (int32_t)g_accel_peak_mg)
        {
            g_accel_peak_mg = (uint16_t)magnitude;
        }

        /* Casts: uint16_t and a small constant, exact as int32_t. */
        if ((magnitude - (int32_t)g_gravity_mg)
            > (int32_t)IMU_COLLISION_THRESHOLD_MILLI_G)
        {
            gb_collision_latched = true;
        }

        filter_sample(&raw);

        /* Only believe the tilt while the filtered vector really is one g.
         * Anything else is the car being pushed or shaken, and an
         * accelerometer cannot tell that from a slope. */
        g_accel_milli_g = (uint16_t)isqrt(
            (uint32_t)(g_accel_mg[0] * g_accel_mg[0])
            + (uint32_t)(g_accel_mg[1] * g_accel_mg[1])
            + (uint32_t)(g_accel_mg[2] * g_accel_mg[2]));
        {
            /* How far the gravity vector is from one g, smoothed. On
             * smooth ground it sits near zero because the only force is
             * gravity; every bump, slip and rattle adds to it. That makes
             * it a terrain roughness measure as well as the trust gate,
             * which is the same number read two ways. */
            int32_t  offset = difference_from_gravity(g_accel_milli_g);
            uint16_t sample = (uint16_t)((offset < 0) ? -offset : offset);

            /* Casts: the offset is under 14000 mg, and the filtered
             * roughness never exceeds the largest sample. The trust band
             * is a small constant. */
            g_roughness_milli_g = (uint16_t)
                (g_roughness_milli_g
                 - (g_roughness_milli_g >> IMU_ROUGHNESS_SHIFT)
                 + (sample >> IMU_ROUGHNESS_SHIFT));
            gb_pitch_trusted = (offset
                               <= (gb_on_hump
                                   ? (int32_t)IMU_HUMP_TRUST_BAND_MILLI_G
                                   : (int32_t)IMU_PITCH_TRUST_BAND_MILLI_G));
        }

        /* Cast: both pitches are -180 to 180 degrees, so the difference
         * fits an int16_t. */
        if (gb_pitch_trusted)
        {
            g_pitch_deg = (int16_t)(pitch_from(g_accel_mg[IMU_FORWARD_AXIS],
                                               g_accel_mg[IMU_SIDE_AXIS],
                                               g_accel_mg[IMU_UP_AXIS])
                                    - g_pitch_ref_deg);
        }

        update_heading();
        update_hump();
        update_event();
        status = CAR_OK;
    }

    return status;
}

car_status_t imu_feed_odometry (uint32_t distance_mm, int16_t left_mm_per_sec,
                                int16_t right_mm_per_sec)
{
    g_distance_mm = distance_mm;
    g_wheel_left  = left_mm_per_sec;
    g_wheel_right = right_mm_per_sec;

    return CAR_OK;
}

car_status_t imu_get_orientation (int16_t * p_pitch_deg,
                                  int16_t * p_heading_deg)
{
    car_status_t status = CAR_ERR_RANGE;

    if ((NULL != p_pitch_deg) && (NULL != p_heading_deg))
    {
        *p_pitch_deg   = g_pitch_deg;
        *p_heading_deg = g_heading_deg;
        status         = CAR_OK;
    }

    return status;
}

car_status_t imu_get_event (car_motion_event_t * p_event)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_event)
    {
        *p_event = g_event;
        status   = CAR_OK;
    }

    return status;
}

car_status_t imu_get_turn_rate_dps (int16_t * p_rate_dps)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_rate_dps)
    {
        *p_rate_dps = g_turn_rate_dps;
        status      = CAR_OK;
    }

    return status;
}

car_status_t imu_get_peak_accel_magnitude (uint16_t * p_milli_g)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_milli_g)
    {
        *p_milli_g      = g_accel_peak_mg;
        g_accel_peak_mg = g_gravity_mg;
        status          = CAR_OK;
    }

    return status;
}

bool imu_is_pitch_trusted (void)
{
    return gb_pitch_trusted;
}

car_status_t imu_get_accel_magnitude (uint16_t * p_milli_g)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_milli_g)
    {
        *p_milli_g = g_accel_milli_g;
        status     = CAR_OK;
    }

    return status;
}

/**
 * @brief How far a magnitude sits from one g, always positive.
 *
 * @param[in] milli_g Magnitude to test.
 *
 * @return Absolute difference from the calibrated one g.
 */
bool imu_is_calibrated (void)
{
    return gb_calibrated;
}

car_status_t imu_get_terrain_roughness (uint16_t * p_milli_g)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_milli_g)
    {
        *p_milli_g = g_roughness_milli_g;
        status     = CAR_OK;
    }

    return status;
}

bool imu_is_terrain_stable (void)
{
    return (g_roughness_milli_g < IMU_TERRAIN_ROUGH_MILLI_G);
}

static int32_t difference_from_gravity (uint16_t milli_g)
{
    /* Casts: two uint16_t values, exact as int32_t. */
    int32_t difference = (int32_t)milli_g - (int32_t)g_gravity_mg;

    return (difference < 0) ? -difference : difference;
}

bool imu_is_hump_detected (void)
{
    return gb_on_hump;
}

car_status_t imu_get_peak_hump (uint16_t * p_height_mm)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_height_mm)
    {
        *p_height_mm = g_peak_hump_mm;
        status       = CAR_OK;
    }

    return status;
}

car_status_t imu_get_last_hump (uint16_t * p_height_mm)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_height_mm)
    {
        *p_height_mm = g_last_hump_mm;
        status       = CAR_OK;
    }

    return status;
}

car_status_t imu_get_hump_count (uint16_t * p_count)
{
    car_status_t status = CAR_ERR_RANGE;

    if (NULL != p_count)
    {
        *p_count = g_hump_count;
        status   = CAR_OK;
    }

    return status;
}

bool imu_is_collision_detected (void)
{
    bool b_latched = gb_collision_latched;

    gb_collision_latched = false;

    return b_latched;
}

/**
 * @brief Low pass both sensors and keep the magnetometer extremes.
 *
 * @param[in] p_raw This period's readings.
 */
static void filter_sample (imu_raw_t const * p_raw)
{
    uint32_t axis = 0u;

    for (axis = 0u; axis < IMU_AXES; axis++)
    {
        g_accel_mg[axis] += (p_raw->accel_mg[axis] - g_accel_mg[axis])
                            >> IMU_FILTER_SHIFT;
        g_mag[axis]      += (p_raw->mag[axis] - g_mag[axis])
                            >> IMU_FILTER_SHIFT;

        if (g_mag[axis] < g_mag_min[axis])
        {
            g_mag_min[axis] = g_mag[axis];
        }

        if (g_mag[axis] > g_mag_max[axis])
        {
            g_mag_max[axis] = g_mag[axis];
        }
    }
}

/**
 * @brief Heading from the horizontal magnetometer axes, hard iron removed.
 *
 * NOTE: The offset is the midpoint of the running extremes, so it only
 * becomes meaningful once the car has turned through most of a circle.
 * Until the spread is wide enough the raw axes are used as they are.
 * TODO: confirm the sign convention on the bench, it depends on which way
 * the module's X axis points on the mount.
 */
static void update_heading (void)
{
    int32_t mag_x = g_mag[0];
    int32_t mag_y = g_mag[1];
    int16_t heading = 0;

    if ((g_mag_max[0] - g_mag_min[0]) > IMU_MAG_MIN_SPREAD)
    {
        mag_x -= (g_mag_max[0] + g_mag_min[0]) / 2;
    }

    if ((g_mag_max[1] - g_mag_min[1]) > IMU_MAG_MIN_SPREAD)
    {
        mag_y -= (g_mag_max[1] + g_mag_min[1]) / 2;
    }

    heading = atan2_deg(-mag_y, mag_x);

    if (heading < 0)
    {
        heading = (int16_t)(heading + 360);   /* -180..-1 becomes 180..359 */
    }

    g_heading_deg = heading;
}

/**
 * @brief Integrate sin(pitch) over distance while climbing, and close the
 *        hump when the pitch drops back below the threshold.
 *
 * Only trusted samples count. Each adds the distance travelled since the
 * previous trusted sample at the mean of the two pitches, so a stretch
 * where pitch could not be believed, often the jolt at the foot of a
 * hump, is bridged by the straight line between the pitch before it and
 * the pitch after it, instead of being counted at the frozen value.
 */
static void update_hump (void)
{
    /* Casts: odometry only grows, and between two trusted samples by far
     * less than 2^31 mm. The pitches are int16_t, exact as int32_t. */
    int32_t step_mm  = (int32_t)(g_distance_mm - g_prev_distance_mm);
    int32_t mean_deg = ((int32_t)g_hump_pitch_deg + (int32_t)g_pitch_deg)
                       / 2;

    if (!gb_pitch_trusted)
    {
        /* Bridged by the next trusted sample. */
    }
    else if (g_pitch_deg > (int16_t)IMU_HUMP_PITCH_THRESHOLD_DEG)
    {
        gb_on_hump       = true;
        g_hump_milli_mm += (mean_deg * IMU_MILLI_RAD_PER_DEG * step_mm)
                           / 1000;
    }
    else if (gb_on_hump)
    {
        end_hump();
    }
    else
    {
        /* Flat ground. */
    }

    if (gb_pitch_trusted)
    {
        g_prev_distance_mm = g_distance_mm;
        g_hump_pitch_deg   = g_pitch_deg;
    }
}

/**
 * @brief The climb is over: record the hump, and the run's peak if it is.
 *
 * A hump under 1 mm is a tilt the car barely moved on, not a hump.
 */
static void end_hump (void)
{
    if (g_hump_milli_mm >= 1000)
    {
        g_hump_count++;
        /* Cast: a hump on a toy car is millimetres high, not 65 metres. */
        g_last_hump_mm = (uint16_t)(g_hump_milli_mm / 1000);

        if (g_last_hump_mm > g_peak_hump_mm)
        {
            g_peak_hump_mm = g_last_hump_mm;
        }
    }

    /* Still on the hump while coming down its far side, so the descent
     * never opens a second one. */
    gb_on_hump      = (g_pitch_deg < -(int16_t)IMU_HUMP_PITCH_THRESHOLD_DEG);
    g_hump_milli_mm = 0;
}

/**
 * @brief Turn rate from the wheel speeds, then the event classifier.
 */
static void update_event (void)
{
    car_motion_event_t candidate = CAR_MOTION_STATIONARY;
    int32_t rate    = 0;
    int32_t forward = g_accel_mg[IMU_FORWARD_AXIS] - g_accel_ref_fwd_mg;

    /* Casts: two int16_t speeds and a small constant, exact as int32_t;
     * the product stays well inside it. */
    rate = (((int32_t)g_wheel_left - (int32_t)g_wheel_right)
            * IMU_DEGREES_PER_RADIAN_MILLI) / ((int32_t)WHEEL_BASE_MM * 1000);

    /* Cast: a turn rate from wheel speeds of at most 1600 mm/s over the
     * wheel base is under 32767 degrees per second. */
    g_turn_rate_dps = (int16_t)rate;

    if (forward < 0)
    {
        forward = -forward;
    }

    if (rate < 0)
    {
        rate = -rate;
    }

    /* Casts: the thresholds are small positive constants. */
    if (gb_collision_latched)
    {
        candidate = CAR_MOTION_IMPACT;
    }
    else if (g_pitch_deg > (int16_t)IMU_HUMP_PITCH_THRESHOLD_DEG)
    {
        candidate = CAR_MOTION_CLIMBING;
    }
    else if (g_pitch_deg < -(int16_t)IMU_HUMP_PITCH_THRESHOLD_DEG)
    {
        candidate = CAR_MOTION_DESCENDING;
    }
    else if (rate > (int32_t)IMU_TURN_EVENT_DPS)
    {
        candidate = CAR_MOTION_TURNING;
    }
    else if (forward > (int32_t)IMU_ACCEL_EVENT_MILLI_G)
    {
        candidate = CAR_MOTION_ACCELERATING;
    }
    else
    {
        candidate = CAR_MOTION_STATIONARY;
    }

    /* Hysteresis: an impact always wins, anything else has to outlast the
     * hold of whatever is current before it replaces it. */
    if ((CAR_MOTION_IMPACT == candidate) || (0u == g_event_hold))
    {
        if (candidate != g_event)
        {
            g_event      = candidate;
            g_event_hold = IMU_EVENT_HOLD_SAMPLES;
        }
    }
    else
    {
        g_event_hold--;
    }
}

/**
 * @brief Pitch of the forward axis from the gravity vector.
 *
 * @param[in] fwd_mg  Forward axis acceleration, mg.
 * @param[in] side_mg Sideways axis acceleration, mg.
 * @param[in] up_mg   Vertical axis acceleration, mg.
 *
 * @return Degrees, nose up positive with IMU_PITCH_SIGN at +1.
 */
static int16_t pitch_from (int32_t fwd_mg, int32_t side_mg, int32_t up_mg)
{
    /* Casts: squares of axes under 8000 mg are positive and fit
     * uint32_t, and the root of their sum fits int32_t. */
    int32_t horizontal = (int32_t)isqrt((uint32_t)(side_mg * side_mg)
                                        + (uint32_t)(up_mg * up_mg));

    return atan2_deg(-fwd_mg * IMU_PITCH_SIGN, horizontal);
}

/**
 * @brief Integer atan2 in whole degrees, -180 to 180.
 *
 * @param[in] y_axis Numerator axis.
 * @param[in] x_axis Denominator axis.
 *
 * @return Angle of the point (x_axis, y_axis) from the positive x axis,
 *         counter clockwise.
 */
static int16_t atan2_deg (int32_t y_axis, int32_t x_axis)
{
    int32_t abs_x       = (x_axis < 0) ? -x_axis : x_axis;
    int32_t abs_y       = (y_axis < 0) ? -y_axis : y_axis;
    int32_t angle100 = 0;

    if ((0 != abs_x) || (0 != abs_y))
    {
        if (abs_y <= abs_x)
        {
            angle100 = atan_ratio_100(abs_y, abs_x);
        }
        else
        {
            angle100 = 9000 - atan_ratio_100(abs_x, abs_y);
        }

        if (x_axis < 0)
        {
            angle100 = 18000 - angle100;
        }

        if (y_axis < 0)
        {
            angle100 = -angle100;
        }
    }

    angle100 += (angle100 >= 0) ? 50 : -50;

    return (int16_t)(angle100 / 100);   /* -180 to 180 degrees */
}

/**
 * @brief atan(num / den) in hundredths of a degree for 0 <= num <= den.
 *
 * Polynomial 45z - z(|z| - 1)(14.02 + 3.80|z|) on z in [0, 1], within a
 * tenth of a degree of the true value.
 *
 * @param[in] num Numerator, not above den.
 * @param[in] den Denominator, positive.
 *
 * @return Hundredths of a degree, 0 to 4500.
 */
static int32_t atan_ratio_100 (int32_t num, int32_t den)
{
    int32_t ratio    = (num * 1000) / den;
    int32_t ratio_term   = ratio * (ratio - 1000);
    int32_t coef = 1402 + ((380 * ratio) / 1000);

    return ((45 * ratio) / 10) - ((ratio_term * coef) / 1000000);
}

/**
 * @brief Integer square root, rounded down.
 *
 * @param[in] value Radicand.
 *
 * @return Largest root whose square does not exceed value.
 */
static uint32_t isqrt (uint32_t value)
{
    uint32_t root = 0u;
    uint32_t bit  = 1u << 30;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (0u != bit)
    {
        if (value >= (root + bit))
        {
            value -= root + bit;
            root   = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }

        bit >>= 2;
    }

    return root;
}

#ifdef CAR_HOST_TEST

/* Host fakes: the test injects readings; the default is level and still. */
static imu_raw_t g_host_raw =
{
    { 0, 0, IMU_GRAVITY_MILLI_G },
    { 300, 0, 0 }
};

void imu_host_inject (int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
                      int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
    g_host_raw.accel_mg[0] = ax_mg;
    g_host_raw.accel_mg[1] = ay_mg;
    g_host_raw.accel_mg[2] = az_mg;
    g_host_raw.mag[0]      = mag_x;
    g_host_raw.mag[1]      = mag_y;
    g_host_raw.mag[2]      = mag_z;
}

static bool hw_init (void)
{
    return true;
}

static bool hw_read (imu_raw_t * p_raw)
{
    *p_raw = g_host_raw;

    return true;
}

static void hw_delay_msec (uint32_t msec)
{
    (void)msec;
}

#else /* CAR_HOST_TEST */

/* Where the kernel's hw_setting.c puts I2C0 at boot. */
#define KERNEL_I2C_SDA_PIN       8u
#define KERNEL_I2C_SCL_PIN       9u

static ID gh_i2c = 0;

static bool read_block (UW address, uint8_t reg, uint8_t * p_buf, SZ len);
static bool write_reg (UW address, uint8_t reg, uint8_t value);
static bool read_reg (UW address, uint8_t reg, uint8_t * p_value);

/**
 * @brief Open the I2C unit, move it to the Grove pins, configure and verify
 *        both sensors.
 *
 * @return true if both sensors answered as expected.
 */
static bool hw_init (void)
{
    bool b_ok  = false;
    UB   value = 0u;

    /* Cast: the kernel spells a string as UB const *, same bytes. */
    gh_i2c = tk_opn_dev((UB const *)IMU_I2C_DEVICE_NAME, TD_UPDATE);

    if (gh_i2c > 0)
    {
        /* The kernel brings I2C0 up on GP8 and GP9, which on this board
         * are the left motor's inputs, and a pin keeps its function until
         * told otherwise, so the bus traffic would drive that motor too.
         * Unless motion has already claimed them, park them as inputs
         * pulled low, which leaves the motor off. */
        if (car_hw_gpio_is_i2c(KERNEL_I2C_SDA_PIN))
        {
            car_hw_gpio_input_pulldown(KERNEL_I2C_SDA_PIN);
        }

        if (car_hw_gpio_is_i2c(KERNEL_I2C_SCL_PIN))
        {
            car_hw_gpio_input_pulldown(KERNEL_I2C_SCL_PIN);
        }

        /* Same pad setup the kernel driver applies to its default pins. */
        car_hw_gpio_i2c(IMU_I2C_SDA_PIN);
        car_hw_gpio_i2c(IMU_I2C_SCL_PIN);

        b_ok = (write_reg(IMU_ACCEL_I2C_ADDR, LSM_CTRL_REG1_A,
                          LSM_CTRL_REG1_A_100HZ))
               && (write_reg(IMU_ACCEL_I2C_ADDR, LSM_CTRL_REG4_A,
                             LSM_CTRL_REG4_A_HR_2G))
               && (read_reg(IMU_ACCEL_I2C_ADDR, LSM_CTRL_REG1_A, &value))
               && (LSM_CTRL_REG1_A_100HZ == value);

        b_ok = b_ok
               && (write_reg(IMU_MAG_I2C_ADDR, LSM_CRA_REG_M,
                             LSM_CRA_REG_M_75HZ))
               && (write_reg(IMU_MAG_I2C_ADDR, LSM_CRB_REG_M,
                             LSM_CRB_REG_M_1_3_GAUSS))
               && (write_reg(IMU_MAG_I2C_ADDR, LSM_MR_REG_M,
                             LSM_MR_REG_M_CONTINUOUS))
               && (read_reg(IMU_MAG_I2C_ADDR, LSM_IRA_REG_M, &value))
               && (LSM_IRA_REG_M_VALUE == value);
    }

    return b_ok;
}

/**
 * @brief Read both sensors' output registers in two bursts.
 *
 * @param[out] p_raw Converted readings.
 *
 * @return true if both bursts succeeded.
 */
static bool hw_read (imu_raw_t * p_raw)
{
    uint8_t accel[6] = { 0 };
    uint8_t mag[6]   = { 0 };
    bool    b_ok     = false;

    /* Casts: six byte buffers, and SZ holds any object size. */
    if ((read_block(IMU_ACCEL_I2C_ADDR, LSM_OUT_X_L_A | LSM_AUTO_INCREMENT,
                    accel, (SZ)sizeof(accel)))
        && (read_block(IMU_MAG_I2C_ADDR, LSM_OUT_X_H_M, mag, (SZ)sizeof(mag))))
    {
        uint32_t axis = 0u;

        /* Accelerometer: little endian, 12 bits left justified. */
        for (axis = 0u; axis < IMU_AXES; axis++)
        {
            int16_t value = (int16_t)(((uint16_t)accel[(2u * axis) + 1u] << 8)
                                      | (uint16_t)accel[2u * axis]);

            /* High resolution mode is 1 mg per count at plus or minus
             * 2 g and doubles with each step up the full scale. */
            p_raw->accel_mg[axis] = (int16_t)((value >> LSM_ACCEL_SHIFT)
                                              * (int16_t)IMU_ACCEL_MG_PER_LSB);
        }

        /* Magnetometer: big endian, registers ordered X, Z, Y. */
        p_raw->mag[0] = (int16_t)(((uint16_t)mag[0] << 8) | (uint16_t)mag[1]);
        p_raw->mag[2] = (int16_t)(((uint16_t)mag[2] << 8) | (uint16_t)mag[3]);
        p_raw->mag[1] = (int16_t)(((uint16_t)mag[4] << 8) | (uint16_t)mag[5]);
        b_ok = true;
    }

    return b_ok;
}

/**
 * @brief One register address write followed by a repeated start read.
 *
 * @param[in]  address Seven bit slave address.
 * @param[in]  reg     First register, with the auto increment bit if needed.
 * @param[out] p_buf   Destination.
 * @param[in]  len     Bytes to read.
 *
 * @return true on success.
 */
static bool read_block (UW address, uint8_t reg, uint8_t * p_buf, SZ len)
{
    T_I2C_EXEC exec =
    {
        .sadr     = address,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = len,
        .rcv_data = p_buf,
    };
    SZ actual = 0;

    /* Cast: SZ holds any object size. */
    return (E_OK <= tk_swri_dev(gh_i2c, TDN_I2C_EXEC, &exec,
                                (SZ)sizeof(exec), &actual));
}

/**
 * @brief Write one register on one device.
 *
 * @param[in] address Seven bit slave address.
 * @param[in] reg     Register.
 * @param[in] value   Byte to write.
 *
 * @return true on success.
 */
static bool write_reg (UW address, uint8_t reg, uint8_t value)
{
    return (E_OK <= i2c_write_reg(gh_i2c, address, reg, value));
}

/**
 * @brief Read one register from one device.
 *
 * @param[in]  address Seven bit slave address.
 * @param[in]  reg     Register.
 * @param[out] p_value Byte read.
 *
 * @return true on success.
 */
static bool read_reg (UW address, uint8_t reg, uint8_t * p_value)
{
    return (E_OK <= i2c_read_reg(gh_i2c, address, reg, p_value));
}

static void hw_delay_msec (uint32_t msec)
{
    (void)tk_dly_tsk((RELTIM)msec);   /* RELTIM is 32 bit, as msec is */
}

#endif /* CAR_HOST_TEST */

/*** end of file ***/
