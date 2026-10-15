/** @file car_config.h
 *
 * @brief Every pin, gain, threshold and timing the car depends on.
 *
 * NOTE: Change values here, never in the modules. Pins marked verified come
 * from the board maker's example code.
 *
 * Owner: each block below names the buddy who owns its values.
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * Board: Cytron Robo Pico carrying a Raspberry Pi Pico W. The board owns
 * GP8 to GP11 (motor driver), GP12 to GP15 (servo headers S1 to S4), GP18
 * (NeoPixels), GP20 and GP21 (buttons) and GP22 (buzzer). The Pico W radio
 * owns GP23, GP24, GP25 and GP29.
 *
 * Grove ports. Each brings out two GPIO plus 3.3 V and ground.
 * WARNING: On every port the WHITE wire carries the lower GPIO and the
 * YELLOW the higher.
 *
 *   Port     White  Yellow  Use
 *   Grove 1  GP0    GP1     nothing, the kernel's UART0 console
 *   Grove 2  GP2    GP3     right line sensor on yellow
 *   Grove 3  GP4    GP5     LSM303DLHC compass, I2C0 SDA and SCL
 *   Grove 4  GP16   GP17    HC-SR04 trigger and echo
 *   Grove 5  GP6    GP26    left line sensor on yellow
 *   Grove 6  GP26   GP27    barcode sensor on yellow
 *   Grove 7  GP7    GP28    right wheel encoder, A on white, B on yellow
 *   Header   GP19, GP6      left wheel encoder, A on GP19, B on GP6
 *
 * NOTE: Grove 6 white (GP26, the left line sensor) and Grove 5 white (GP6,
 * the left encoder's B phase) must stay unconnected at their sensors.
 */

/* Owner: the team. The car's size, measured. */
#define CAR_WIDTH_MM                 150u   // Across the front
#define CAR_LENGTH_MM                200u   // Bumper to back

/* Owner: Buddy 2, motion control. */
/* Motor driver, on the board. Two PWM pins per motor. */
#define MOTOR_LEFT_IN1_PIN             8u   // M1A, verified
#define MOTOR_LEFT_IN2_PIN             9u   // M1B, verified
#define MOTOR_RIGHT_IN1_PIN           10u   // M2A, verified
#define MOTOR_RIGHT_IN2_PIN           11u   // M2B, verified
#define MOTOR_PWM_FREQ_HZ          10000u   // Board maker's, motors verified
#define MOTOR_PWM_MAX_DUTY          1000u   // Duty is per mille
#define MOTOR_MIN_DUTY                70u   // From ./flash.sh duty

/* Wheel encoders. Phase A raises the interrupt and is counted. Phase B, read
 * at each A edge, gives the direction, so rolling back down a hump counts
 * against progress. One interrupt serves the whole GPIO bank. */
#define ENCODER_LEFT_PIN              19u   // Header, phase A
#define ENCODER_LEFT_B_PIN             6u   // Header, phase B
#define ENCODER_RIGHT_PIN              7u   // Grove 7 white, phase A
#define ENCODER_RIGHT_B_PIN           28u   // Grove 7 yellow, phase B
/* Level phase B reads at a rising A edge while that wheel drives forward.
 * The motors are mirrored, so the two differ. Check with ./flash.sh
 * encoders: a wheel turned forward by hand must read fwd. */
#define ENCODER_LEFT_B_FORWARD         0u   // Read back turning forward
#define ENCODER_RIGHT_B_FORWARD        1u
#define ENCODER_IRQ_NUM               13u   // IO_IRQ_BANK0, RP2040 datasheet
#define ENCODER_IRQ_LEVEL              2    // Same level the I2C driver uses
#define ENCODER_SLOTS_PER_REV        639u   // Hand turned: 638, 639, 640
/* Pulses in a row phase B must agree on before a wheel counts as reversed,
 * so one noisy reading cannot flip it. 4 is about 1.3 mm. */
#define ENCODER_DIRECTION_PULSES       4u
/* Shortest real gap between pulses: 0.32 mm at twice
 * MOTION_MAX_SPEED_MM_PER_SEC is about 200 us. A sooner edge, or one whose
 * pin is already low, is motor noise and is not counted. motion_get_state()
 * reports how many were dropped. */
#define ENCODER_MIN_PULSE_USEC       150u
#define ENCODER_STALL_TIMEOUT_MSEC   300u   // No pulse this long reads 0 mm/s
/* A driven wheel silent this long is a fault and halts the car. It must
 * outlast the PID's climb to full duty, about a second, so a hump that slows
 * the car is pushed over rather than halting it. */
#define MOTION_STALL_FAULT_MSEC     2000u
/* Ground per wheel turn, both wheels' average, from 500 mm tape runs. */
#define WHEEL_CIRCUMFERENCE_MM       205u
/* Each wheel's share of that, per mille. The left tyre covers less ground
 * per turn. The speed loops and the straight-line correction use these, so
 * both wheels cover the same ground. Keep the two summing to 2000. */
#define WHEEL_LEFT_PERMILLE          993u
#define WHEEL_RIGHT_PERMILLE        1007u
/* Turning base, tyre centre to tyre centre, checked with a protractor. */
#define WHEEL_BASE_MM                110u

/* Speed loop. Gains are in thousandths, so no floating point, and act once
 * per tick. Tune with ./flash.sh tuning, see motion/tuning_report.md. */
#define MOTION_TICK_PERIOD_MSEC       10u
#define MOTION_PID_KP_MILLI          500u   // Per mm/s of error
#define MOTION_PID_KI_MILLI           25u   // Per mm/s of error per tick
#define MOTION_PID_KD_MILLI           20u   // Per mm/s change per tick
#define MOTION_PID_INTEGRAL_LIMIT  40000    // Anti windup, in mm/s * ticks
#define MOTION_DEFAULT_SPEED_MM_PER_SEC 200u
/* Wheel speed for every turn on the spot. Slower turns overshoot less and
 * give the line sensors longer over the line. */
#define MOTION_TURN_SPEED_MM_PER_SEC  60u   // 100 overshot 2.4 to 3.7 deg
/* Every speed-up climbs at this rate: the faster wheel ramps and the other
 * keeps its share. Slowing and stopping are immediate. 200 is 0 to
 * 150 mm/s in 0.75 s. */
#define MOTION_ACCEL_MM_PER_SEC2     200u
/* Straight-line correction on distance moves. The wheel that has gone
 * further is slowed and the other sped up by this many mm/s per mm between
 * them, at most half the move speed. Raise it if a 500 mm drive curves,
 * lower it if the car wags. */
#define MOTION_STRAIGHT_KP             4u   // mm/s per mm
/* Speed at full duty, for the feedforward duty. A wrong value only leaves
 * more for the PID to correct. */
#define MOTION_MAX_SPEED_MM_PER_SEC  800u
#define MOTION_MAX_STEER_PERMILLE   2000    // Over 1000 reverses inner wheel

/* Owner: Buddy 3, line and barcode. */
/* Three MH-Sensor-Series IR modules with digital outputs. Left and right
 * straddle the 18 mm line with their eyes 35 mm apart, so both see floor on
 * a straight with 8.5 mm to spare each side. The third sits to the left
 * over the barcodes and never steers. LINE_SENSOR_IS_ANALOG stays 0. */
#define LINE_SENSOR_LEFT_PIN          26u   // Grove 5 yellow
#define LINE_SENSOR_BARCODE_PIN       27u   // Grove 6 yellow
#define LINE_SENSOR_RIGHT_PIN          3u   // Grove 2 yellow
#define LINE_SENSOR_IS_ANALOG          0u   // Must stay 0 on this layout
#define LINE_SENSOR_DARK_LEVEL         1u   // Level over black, verified

/* Pin pull. WARNING: This and LINE_SENSOR_DARK_LEVEL must differ, so a
 * loose wire reads as a lost line, not as a junction. */
#define LINE_SENSOR_PULL_UP            0u   // 0 pull down, 1 pull up
/* A junction is both line sensors dark within this window, not
 * necessarily together: a skewed car meets the cross with one sensor
 * first. 100 ms is 15 mm at 150 mm/s, enough for a skew of about 43
 * degrees with the eyes 35 mm apart. Raise it if
 * crosses slip past, lower it if the car stops where there is none. */
#define LINE_JUNCTION_WINDOW_MSEC    100u
/* Start-up check: every sensor must read floor for this many samples. */
#define LINE_CALIBRATE_SAMPLES        20u

/* With neither sensor dark the line is centred or gone. It counts as
 * centred for this long after a sensor last saw it, then as lost. */
#define LINE_CENTRED_HOLD_MSEC       500u

/* Line sampling, one kernel tick. The barcode sensor has its own sampler,
 * timer alarm 0, every BARCODE_SAMPLE_USEC: a 3 mm bar stays readable up to
 * about 3 m/s. */
#define LINE_SAMPLE_PERIOD_MSEC       10u
#define BARCODE_SAMPLE_USEC          500u

/* Course geometry. Junctions are crosses. Barcodes sit beside the line on
 * the left, TRACK_BARCODE_GAP_MM from its edge, and act at the next cross.
 * A 141 mm barcode is one Code 39 character between start and stop symbols
 * at a 3 mm narrow element. */
#define TRACK_LINE_WIDTH_MM           18u
#define TRACK_BARCODE_GAP_MM          20u   // Line edge to barcode
#define TRACK_BARCODE_LENGTH_MM      141u   // Along the line
#define BARCODE_NARROW_MM              3u
#define BARCODE_MAX_ELEMENT_MSEC     500u   // Longer runs reset the decoder

/* Code 39 characters for each command. */
#define BARCODE_CHAR_LEFT             'A'
#define BARCODE_CHAR_RIGHT            'B'
#define BARCODE_CHAR_STRAIGHT         'C'
#define BARCODE_CHAR_UTURN            'D'

/* Owner: Buddy 4, IMU and terrain. */
/* LSM303DLHC on I2C0 through Grove 3. The kernel defaults I2C0 to the motor
 * pins, so imu_init() moves it to the pins below. */
#define IMU_I2C_DEVICE_NAME       "iica"    // Kernel I2C unit 0
#define IMU_I2C_SDA_PIN                4u   // Grove 3 white
#define IMU_I2C_SCL_PIN                5u   // Grove 3 yellow
#define IMU_ACCEL_I2C_ADDR          0x19u   // LSM303DLHC datasheet, SAD accel
#define IMU_MAG_I2C_ADDR            0x1Eu   // LSM303DLHC datasheet, SAD mag
#define IMU_SAMPLE_PERIOD_MSEC        10u   // 100 Hz
#define IMU_FILTER_SHIFT               5u   // Alpha 1/2^n, ~320 ms at 100 Hz
#define IMU_CALIBRATION_SAMPLES       32u   // Level reference average
/* Pitch is the direction of apparent gravity, so it is frozen while the
 * filtered acceleration strays this far from 1 g: a running motor shakes the
 * chassis and reads as tilt. Widen it if pitch stops following a real
 * slope, narrow it if a shaking car reports tilt. */
#define IMU_PITCH_TRUST_BAND_MILLI_G 300u   // Mast mount shakes
/* The same band while a hump is open, wider so the changing slope is kept.
 * Keep it under 500: the host test shakes the car at 1.5 g on the flat and
 * expects a freeze. */
#define IMU_HUMP_TRUST_BAND_MILLI_G  450u
#define IMU_HUMP_PITCH_THRESHOLD_DEG   5u   // Pitch that opens a hump
/* Accelerometer full scale: 0 is 2 g, 1 is 4 g, 2 is 8 g, 3 is 16 g. The
 * part saturates there, so 2 g would clip every knock to 2000. */
#define IMU_ACCEL_FS_SELECT            2u   // 0 is 2 g, 2 is 8 g
#define IMU_ACCEL_MG_PER_LSB           4u   // 1 << IMU_ACCEL_FS_SELECT
#define IMU_COLLISION_THRESHOLD_MILLI_G  600u  // Jolt from any side
#define IMU_ACCEL_EVENT_MILLI_G      250u   // Forward acceleration event
/* Accelerating also counts from the wheels: the mean wheel speed up by this
 * rate over IMU_ACCEL_WINDOW_SAMPLES. The car's speed-ups, at
 * MOTION_ACCEL_MM_PER_SEC2, are about 20 mg, far under what the
 * accelerometer can tell from vibration. */
#define IMU_ACCEL_EVENT_MM_PER_SEC2  100u   // Half the speed-up ramp
#define IMU_ACCEL_WINDOW_SAMPLES      20u   // 200 ms
#define IMU_TURN_EVENT_DPS            30u   // Turning event
#define IMU_EVENT_HOLD_SAMPLES        20u   // Samples an event holds, 200 ms
/* Accelerometer axis that points along the car: 0 is the module's X, 1 its
 * Y. This module is turned 90 degrees. IMU_PITCH_SIGN picks the nose end. */
#define IMU_FORWARD_AXIS               1u
#define IMU_PITCH_SIGN                 1    // Nose up reads positive, verified
/* Roughness is the smoothed distance of the gravity vector from 1 g, near
 * zero on a smooth floor. Above the threshold the terrain is rough. */
#define IMU_ROUGHNESS_SHIFT            5u   // Alpha 1/2^n, ~320 ms at 100 Hz
#define IMU_TERRAIN_ROUGH_MILLI_G    120u

/* Owner: Buddy 5, scanning. */
/* HC-SR04 on Grove 4, powered at 3.3 V so its echo needs no divider. */
#define SONAR_TRIG_PIN                16u   // Grove 4 yellow
#define SONAR_ECHO_PIN                17u   // Grove 4 white
#define SONAR_TRIG_PULSE_USEC         10u   // Datasheet minimum.
#define SONAR_USEC_PER_CM             58u   // Datasheet conversion.
#define SONAR_MIN_CYCLE_MSEC          60u   // Datasheet minimum cycle.
#define SONAR_MAX_RANGE_MM          4000u   // Datasheet.
#define SONAR_MIN_RANGE_MM            20u   // Datasheet.
#define SONAR_ECHO_TIMEOUT_USEC    30000u   // 400 cm * 58 = 23200, plus margin

/* Scan servo on header S1, powered from the motor rail, so it needs the
 * battery. Angle 90 is straight ahead, above 90 looks left. */
#define SERVO_PIN                     12u   // S1, verified
#define SERVO_PWM_FREQ_HZ             50u   // Verified example value
/* Pulse width that points the horn straight ahead on this car's mount.
 * Angles are measured from it, so no command can swing the horn far.
 *
 * WARNING: A servo has no position feedback, so the first pulse jumps the
 * horn to it. Find the width with BENCH=servo's centre finder and the jump
 * disappears. */
#define SERVO_CENTRE_PULSE_USEC     1744u   // Measured on this car's mount
#define SERVO_USEC_PER_DEG            11u   // 1000 us per 90 deg

/* Hard limits on the pulse. Above 2000 because the high centre needs 2019
 * at 115 degrees. Put it back to 2000 if the horn binds. */
#define SERVO_PULSE_MIN_USEC        1000u
#define SERVO_PULSE_MAX_USEC        2100u
#define SERVO_TRAVEL_DEG             180u
#define SERVO_SETTLE_MSEC            200u   // Wait after each move

/* Travel used. Change SCAN_HALF_SWEEP_DEG and every angle follows.
 * scan_measure() clamps every angle into the band.
 *
 * NOTE: 25 degrees either side clears the sonar's 15 degree beam, so the
 * side readings are independent and avoidance can pick a side. */
#define SCAN_CENTRE_ANGLE_DEG         90u   // Straight ahead
#define SCAN_HALF_SWEEP_DEG           25u   // 65, 90 and 115 degrees
#define SCAN_MIN_ANGLE_DEG   (SCAN_CENTRE_ANGLE_DEG - SCAN_HALF_SWEEP_DEG)
#define SCAN_MAX_ANGLE_DEG   (SCAN_CENTRE_ANGLE_DEG + SCAN_HALF_SWEEP_DEG)

/* The coarse scan spans the band in three steps. */
#define SCAN_COARSE_ANGLES_DEG   { SCAN_MIN_ANGLE_DEG, SCAN_CENTRE_ANGLE_DEG, \
                                   SCAN_MAX_ANGLE_DEG }
#define SCAN_COARSE_ANGLE_COUNT        3u
/* Fine step, half the sweep: 65, 77, 89, 101, 113. Never below one degree,
 * or scan_fine() could not advance. */
#define SCAN_FINE_STEP_DEG \
    (((SCAN_HALF_SWEEP_DEG / 2u) > 0u) ? (SCAN_HALF_SWEEP_DEG / 2u) : 1u)
#define SCAN_FINE_HALF_SPAN_DEG SCAN_HALF_SWEEP_DEG
#define SCAN_OBSTACLE_RANGE_MM       200u   // Stop this far from an obstacle
/* A lane fits the car plus 50 mm each side. */
#define SCAN_CLEARANCE_MIN_MM  (CAR_WIDTH_MM + 100u)
#define SCAN_DETOUR_TURN_DEG          45u   // Each leg of the box detour
#define SCAN_DETOUR_SIN45_RECIP     1414u   // 1 / sin 45, times 1000
/* How far the nose swings sideways turning on the spot: the front corner,
 * sqrt(150^2 + 75^2) = 168. The sideways leg leaves this plus 30 mm, so a
 * look turn cannot hit the obstacle. */
#define SCAN_NOSE_SWING_MM           170u
#define SCAN_DETOUR_MARGIN_MM  (SCAN_NOSE_SWING_MM + 30u)
/* Walking the obstacle's length. Beside it, the car steps
 * SCAN_DETOUR_STEP_MM and turns SCAN_DETOUR_LOOK_DEG toward it to look, so
 * the sonar meets the side square on. Nearer than the line means it is
 * still there. Once clear, the car drives SCAN_DETOUR_PAST_MM on and turns
 * in. Raise that if it clips the corner. The walk stops at
 * SCAN_DETOUR_DEPTH_MAX_MM. */
#define SCAN_DETOUR_STEP_MM           50u
#define SCAN_DETOUR_LOOK_DEG  (90u - SCAN_HALF_SWEEP_DEG)   // 65
#define SCAN_DETOUR_PAST_MM          100u
/* Fallback legs when no width was measured: the obstacle as a point. The
 * sideways leg is divided by sin 45 for the diagonal. */
#define SCAN_DETOUR_SIDE_MM \
    ((SCAN_DETOUR_MARGIN_MM * SCAN_DETOUR_SIN45_RECIP) / 1000u)
#define SCAN_DETOUR_DEPTH_MM   (CAR_LENGTH_MM + SCAN_DETOUR_MARGIN_MM)
/* With a width, the legs are sized from it: half the obstacle, the car's
 * half width and the margin, over sin 45. Depth is the width plus
 * CAR_LENGTH_MM. Both are clamped, and the minima are the fallback legs. */
#define SCAN_DETOUR_SIDE_MIN_MM      SCAN_DETOUR_SIDE_MM
/* Narrowest width a detour plans for. A flat face met at an angle sends the
 * echo away, so obstacles read narrow. Set it to the widest obstacle on the
 * course plus a little. */
#define SCAN_DETOUR_WIDTH_MIN_MM     220u
#define SCAN_DETOUR_SIDE_MAX_MM      500u
#define SCAN_DETOUR_DEPTH_MIN_MM     SCAN_DETOUR_DEPTH_MM
#define SCAN_DETOUR_DEPTH_MAX_MM     600u
#define SCAN_REVERSE_MM              100u   // Back off before probing a side
/* Sweep the coarse angles while driving, so the car knows which side is open
 * before it must choose. Forward is then ranged every other reading. 0
 * stares straight ahead. */
#define SCAN_SWEEP_WHILE_MOVING        1u
#define SCAN_RECOVER_TURN_DEG         15u   // Search arc step
#define SCAN_RECOVER_DRIVE_MM         60u   // Search drive step
#define SCAN_RECOVER_STEPS            12u   // Turn and drive pairs, then halt

/* Owner: Buddy 1, comms. */
/* WiFi and MQTT. The SSID and password live in the kernel's
 * config/wifi_credentials.h, which git ignores. */
#define COMMS_MQTT_BROKER_HOST   "192.168.50.131"   // Verified
#define COMMS_MQTT_BROKER_PORT      1883u
#define COMMS_MQTT_CLIENT_ID     "car"
#define COMMS_TOPIC_TELEMETRY    "car/telemetry"
#define COMMS_TOPIC_HEARTBEAT    "car/heartbeat"
#define COMMS_TOPIC_COMMAND      "car/command"
#define COMMS_TOPIC_TERRAIN      "car/terrain"
#define COMMS_TOPIC_EVENT        "car/event"        // Sent as it happens
#define COMMS_TOPIC_LOG          "car/log"          // Console mirror
#define COMMS_WIFI_TIMEOUT_MSEC    30000u
#define COMMS_MQTT_KEEPALIVE_SECONDS  60u
#define COMMS_POLL_PERIOD_MSEC        10u
#define COMMS_HEARTBEAT_PERIOD_MSEC 5000u
#define COMMS_RECONNECT_BACKOFF_MSEC 5000u
/* Telemetry is the longest payload: 297 bytes typically, 365 at most. The
 * MQTT ring slot, LWIP_UTK_MQTT_TX_PAYLOAD, must be at least this, and
 * comms.c refuses to build if not. A message that does not fit is dropped
 * whole. */
#define COMMS_PAYLOAD_MAX_BYTES      480u

/* Owner: the team. */
/* Mission timing and line following. Steering is a proportional correction
 * on the sensor error, in per mille of the speed given to
 * motion_drive_steer(). */
#define CAR_MISSION_PERIOD_MSEC       10u
/* Anything that happens in between goes out at once on COMMS_TOPIC_EVENT. */
#define CAR_TELEMETRY_PERIOD_MSEC   5000u
#define CAR_TERRAIN_PERIOD_MSEC     5000u   // The terrain status
#define CAR_FOLLOW_SPEED_MM_PER_SEC  150u
#define CAR_BARCODE_SPEED_MM_PER_SEC 100u   // Slower so every bar is sampled
/* Climbing keeps the set speed and the speed loop adds power. Descending or
 * rough ground drops to this, so the car does not run away. */
#define CAR_DESCEND_SPEED_MM_PER_SEC 120u
/* Proportional only: the error jumps between 0 and 2, so a derivative term
 * would kick the car from one sensor to the other. 150 steers 300 per mille
 * at an edge. Raise it in steps of 25 if the car drifts off, lower it if it
 * swings. */
#define CAR_STEER_KP_PERMILLE        150    // Per unit of error, 2 at an edge
/* Re-entry after a detour, a search or a lost line: cross, creep, then turn
 * on the spot until the second sensor touches the line (reenter_line()).
 * This only bounds a spin that never sees it. */
#define CAR_REENTRY_SPIN_MAX_DEG     120u
/* 1 to test re-entry by hand: the car starts off the line, pointed at it,
 * its sensors within 150 mm. 0 for runs. */
#define CAR_REENTRY_TEST               0u
/* 1 to test barcodes and junctions alone: the sonar is not used. 0 for
 * runs. */
#define CAR_BARCODE_TEST               0u
/* Wait at the start for the broker, so a run is reported from its first
 * metre. With no broker after this long the car drives anyway. 0 drives at
 * once. Builds without the radio never wait. Benches built with --wifi wait
 * at their first console line, see car_log.c. */
#define CAR_BROKER_WAIT_MSEC       60000u
#define CAR_STEER_LOST_PERMILLE      900    // Applied toward the last seen side
#define CAR_LINE_SEEN_SAMPLES         40u   // Ticks seen before speeding up
#define CAR_LINE_LOST_STRAIGHT_MM    200u   // Gap plus barcode plus margin
#define CAR_LINE_LOST_LIMIT_MM       600u   // Beyond this, search for the line

/* An obstacle still there after CAR_MAX_DETOUR_ATTEMPTS tries cannot be
 * driven round, so the car reverses. Attempts reset after
 * CAR_DETOUR_CLEAR_MM of clear following. */
#define CAR_MAX_DETOUR_ATTEMPTS        4u   // Two probes each side
/* A collision outranks everything: stop, back off, then scan before moving.
 * Repeated hits halt the car. */
#define CAR_COLLISION_BACKOFF_MM     150u   // Further than SCAN_REVERSE_MM
#define CAR_COLLISION_SETTLE_MSEC    400u   // Let the chassis stop ringing
#define CAR_MAX_COLLISIONS             3u   // Then stop for good
#define CAR_DETOUR_CLEAR_MM          400u   // Clear run that ends a detour
/* Leg index past which the obstacle is behind the car, so the line seen
 * again is the one to rejoin. Legs 0 to 3 are out, sideways, back and
 * past. */
#define CAR_DETOUR_PAST_INDEX          4u

/* Junctions. The car stops and waits CAR_JUNCTION_WAIT_MSEC for a late
 * barcode or a remote command, then turns as the last barcode said, or goes
 * straight on. */
#define CAR_JUNCTION_WAIT_MSEC       500u   // Stopped, waiting for a command
/* Sensor eyes to the rear axle, the line through the two drive wheels'
 * centres, measured. The car spins about the middle of that axle. */
#define CAR_SENSOR_TO_WHEEL_MM       150u
/* Forward from the junction stop to put the wheels over the crossing, so a
 * turn lands on the new branch: CAR_SENSOR_TO_WHEEL_MM less the coast from
 * stopping. U-turns do not creep. */
#define CAR_JUNCTION_CREEP_MM        100u
/* A junction with no barcode read since the last one: the side sensor
 * missed it. The car backs up at CAR_REREAD_SPEED_MM_PER_SEC, reading, until
 * a barcode is read, or bars then CAR_REREAD_CLEAR_MM of plain floor pass,
 * or it has gone CAR_REREAD_MAX_MM. It never backs up further than it has
 * driven since the start or the last junction, and skips the first
 * CAR_REREAD_CLEAR_MM, where the sensor is over the crossing. It then
 * follows the line back, reading again, once per junction. 0 turns the
 * reread off. */
#define CAR_REREAD_MAX_MM            600u
#define CAR_REREAD_CLEAR_MM           50u   // Plain floor that ends a barcode
#define CAR_REREAD_SPEED_MM_PER_SEC   60u
#define CAR_SONAR_CHECK_PERIOD_MSEC   60u   // Forward ping while following
/* Junction turns end on the line, not on the encoders. The car spins up to
 * CAR_TURN_MAX_DEG and stops once the line sensor on the side it turns
 * toward has crossed the new branch, looking only from
 * CAR_TURN_SENSE_FROM_DEG so the branch it leaves does not count. U-turns
 * use the UTURN pair. With nothing found, line following looks for it. */
#define CAR_TURN_MAX_DEG             120u
#define CAR_TURN_SENSE_FROM_DEG       60u
#define CAR_UTURN_MAX_DEG            210u
#define CAR_UTURN_SENSE_FROM_DEG     150u

/* Logging threshold, matches car_log_level_t: 0 error, 1 info, 2 debug. */
#define CAR_LOG_LEVEL                  1

#endif /* CAR_CONFIG_H */

/*** end of file ***/
