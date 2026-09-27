/** @file motion.h
 *
 * @brief Motor drive, encoder counting and closed loop motion control.
 *
 * Units throughout: distance mm, speed mm per second, angle degrees, time
 * milliseconds. Every function is non blocking. Call motion_tick() at a fixed
 * rate and poll motion_is_busy() to learn when a move has finished.
 *
 * Owner: Buddy 2, motion control. A changed signature here also changes the
 * test, the bench and car_main.c, so agree it with the team first.
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "car.h"

/** Snapshot of the motion subsystem for telemetry and the controller. */
typedef struct
{
    uint16_t speed_mm_per_sec;
    int16_t  left_mm_per_sec;    /* Signed, negative in reverse */
    int16_t  right_mm_per_sec;   /* Signed, negative in reverse */
    uint32_t distance_mm;
    uint32_t encoder_count_left;
    uint32_t encoder_count_right;
    bool     b_left_encoder;     /* Has ever produced a pulse */
    bool     b_right_encoder;    /* Has ever produced a pulse */
    bool     b_left_backward;    /* Phase B: last pulse turned backward */
    bool     b_right_backward;   /* Phase B: last pulse turned backward */
    /* Encoder noise since boot: edges too soon after the last pulse to be
     * real, thrown away, and changes of direction phase B reported. */
    uint32_t glitches_left;
    uint32_t glitches_right;
    uint32_t reversals_left;
    uint32_t reversals_right;
    bool     b_is_busy;
} motion_state_t;

/**
 * @brief Configure motor PWM outputs and encoder edge interrupts.
 *
 * @return CAR_OK once both motors are stopped and counters are zero.
 */
car_status_t motion_init (void);

/**
 * @brief Run one PID update. Call every MOTION_TICK_PERIOD_MSEC.
 *
 * NOTE: Timing jitter here shows up directly as speed ripple, so this must
 * be called from a timer or a tightly paced main loop, not ad hoc.
 *
 * NOTE: A wheel that has never produced a single pulse has no working
 * encoder. Its speed loop reads zero and drives it hard, so the car veers,
 * but it is not reported as a fault; motion_get_state() says which
 * encoders were found. Only a wheel that pulsed and then stopped while
 * still being commanded counts as a fault.
 *
 * @return CAR_OK, or CAR_ERR_HARDWARE if a working encoder stopped.
 */
car_status_t motion_tick (void);

/**
 * @brief Start driving forward for the given distance at the set speed.
 *
 * With encoders fitted the move also holds its heading: the wheel that
 * gets ahead of the other is slowed, see MOTION_STRAIGHT_KP. The same
 * applies to motion_move_backward().
 *
 * @param[in] distance_mm Distance to travel along the ground.
 *
 * @return CAR_OK if the move was accepted.
 */
car_status_t motion_move_forward (uint32_t distance_mm);

/**
 * @brief Start driving backward for the given distance at the set speed.
 *
 * @param[in] distance_mm Distance to travel along the ground.
 *
 * @return CAR_OK if the move was accepted.
 */
car_status_t motion_move_backward (uint32_t distance_mm);

/**
 * @brief Start an in place turn to the left using encoder counts.
 *
 * @param[in] angle_deg Angle to turn through, 0 to 360.
 *
 * @return CAR_OK if accepted, CAR_ERR_RANGE if the angle is above 360.
 */
car_status_t motion_turn_left (uint16_t angle_deg);

/**
 * @brief Start an in place turn to the right using encoder counts.
 *
 * @param[in] angle_deg Angle to turn through, 0 to 360.
 *
 * @return CAR_OK if accepted, CAR_ERR_RANGE if the angle is above 360.
 */
car_status_t motion_turn_right (uint16_t angle_deg);

/**
 * @brief Drive forward continuously at the set speed with a steering bias.
 *
 * This is the line following primitive. There is no distance target, so
 * motion_is_busy() stays false; the drive continues until the next command
 * or motion_stop(). Positive steers right by speeding the left wheel up and
 * slowing the right wheel down by the same fraction of the set speed.
 * Beyond 1000 per mille the inner wheel reverses, which pivots the car.
 *
 * @param[in] steer_permille Bias, clamped to MOTION_MAX_STEER_PERMILLE.
 *
 * @return CAR_OK.
 */
car_status_t motion_drive_steer (int16_t steer_permille);

/**
 * @brief Set the target speed used by subsequent moves.
 *
 * @param[in] speed_mm_per_sec Target ground speed.
 *
 * @return CAR_OK, or CAR_ERR_RANGE above MOTION_MAX_SPEED_MM_PER_SEC.
 */
car_status_t motion_set_speed (uint16_t speed_mm_per_sec);

/**
 * @brief Cut both motors immediately and cancel any move in progress.
 *
 * @return CAR_OK.
 */
car_status_t motion_stop (void);

/**
 * @brief Report whether a move or turn is still in progress.
 *
 * @return true while a commanded move has not yet reached its target.
 */
bool motion_is_busy (void);

/**
 * @brief Copy the current motion snapshot, encoder counts included.
 *
 * NOTE: The encoder interrupt writes the counters, so they are copied
 * with interrupts briefly masked. This is the way to read them.
 *
 * @param[out] p_state Destination, must not be NULL.
 *
 * @return CAR_OK, or CAR_ERR_RANGE if p_state is NULL.
 */
car_status_t motion_get_state (motion_state_t * p_state);

#ifdef CAR_HOST_TEST
/**
 * @brief Host test hook: what the encoder interrupt would have recorded.
 *
 * @param[in] left             Pulses to add on the left wheel.
 * @param[in] right            Pulses to add on the right wheel.
 * @param[in] b_left_backward  Phase B said the left wheel turned backward.
 * @param[in] b_right_backward Phase B said the right wheel turned backward.
 */
void motion_host_inject_pulses (uint32_t left, uint32_t right,
                                bool b_left_backward, bool b_right_backward);

/**
 * @brief Host test hook: the duty last sent to each motor, per mille.
 *
 * @param[out] p_left  Left motor duty.
 * @param[out] p_right Right motor duty.
 */
void motion_host_get_duty (uint16_t * p_left, uint16_t * p_right);
#endif

#endif /* MOTION_H */

/*** end of file ***/
