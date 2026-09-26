/** @file car_config.h
 *
 * @brief Every pin, gain, threshold and timing the car depends on.
 *
 * NOTE: Pins marked verified come from the board maker's own example code.
 * Everything else is a starting guess that the bench programs exist to
 * replace with a measurement. Change values here, never in the modules.
 *
 * Owner: each block group below names the buddy who owns its values.
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * Board: Cytron Robo Pico carrying a Raspberry Pi Pico W. The board owns
 * GP8 to GP11 (motor driver), GP12 to GP15 (servo headers S1 to S4), GP18
 * (two NeoPixels), GP20 and GP21 (buttons) and GP22 (buzzer). The Pico W
 * radio owns GP23, GP24, GP25 and GP29.
 *
 * Grove ports as wired on this car. Each port brings out two GPIO plus
 * 3.3 V and ground. WARNING: On every port the WHITE wire carries the
 * lower GPIO and the YELLOW the higher, read off the board 2026-09-25.
 * Earlier notes had it the other way round, which put the first right line
 * sensor on GP0, the console's transmit pin.
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
 * NOTE: Two white wires are someone else's pin and must stay unconnected
 * at their sensor: Grove 6 white is GP26, the left line sensor, and Grove
 * 5 white is GP6, the left encoder's B phase.
 */

/* Owner: the team. The car's own size, measured with a ruler. Anything
 * that has to fit past, around or between something derives from these
 * two rather than carrying its own guess. */
#define CAR_WIDTH_MM                 150u   // Across the front
#define CAR_LENGTH_MM                200u   // Front bumper to the back

/* Owner: Buddy 2, motion control. The values down to the next owner line. */
/* Motor driver, on the board. Two PWM pins per motor, verified. PWM
 * frequency is the board maker's example value. */
#define MOTOR_LEFT_IN1_PIN             8u   // M1A, verified
#define MOTOR_LEFT_IN2_PIN             9u   // M1B, verified
#define MOTOR_RIGHT_IN1_PIN           10u   // M2A, verified
#define MOTOR_RIGHT_IN2_PIN           11u   // M2B, verified
#define MOTOR_PWM_FREQ_HZ          10000u   // Board maker's, motors verified
#define MOTOR_PWM_MAX_DUTY          1000u   // Duty is expressed per mille.
#define MOTOR_MIN_DUTY               150u   // TODO: measure, ./flash.sh duty

/* Wheel encoders, the Hall sensor built into each geared motor. Phase A
 * raises the interrupt and is counted; phase B is read at every A edge to
 * tell which way the wheel really turned, so a wheel rolling back down a
 * hump counts against progress instead of for it. The RP2040 raises one
 * interrupt for the whole GPIO bank; the handler sorts out which pin. */
#define ENCODER_LEFT_PIN              19u   // Header, phase A
#define ENCODER_LEFT_B_PIN             6u   // Header, phase B
#define ENCODER_RIGHT_PIN              7u   // Grove 7 white, phase A
#define ENCODER_RIGHT_B_PIN           28u   // Grove 7 yellow, phase B
/* Level phase B reads at a rising A edge while that wheel drives forward.
 * The motors are mirrored, so expect the two to differ. 1 matches the pull
 * up, so an unconnected B reads forward, which is the old A only
 * behaviour. Calibrated with ./flash.sh encoders: a wheel turned forward
 * by hand must read fwd. */
#define ENCODER_LEFT_B_FORWARD         0u   // Read back turning forward
#define ENCODER_RIGHT_B_FORWARD        1u
#define ENCODER_IRQ_NUM               13u   // IO_IRQ_BANK0, RP2040 datasheet
#define ENCODER_IRQ_LEVEL              2    // Same level the I2C driver uses
#define ENCODER_SLOTS_PER_REV        639u   // Right, hand turned 638/639/640
/* Phase B must say the same direction for this many pulses in a row before
 * a wheel counts as reversed, so one noisy reading cannot flip it. 4 is
 * about 1.3 mm of travel. The right encoder showed single stray reversals
 * on 2026-09-25, each one a power surge from the speed loop. */
#define ENCODER_DIRECTION_PULSES       4u
#define ENCODER_STALL_TIMEOUT_MSEC   300u   // No pulse this long reads 0 mm/s
/* A driven wheel silent this long is a fault and halts the car. It must
 * outlast the PID's climb to full duty on a stopped wheel, about a second
 * at the gains below, or a hump that stalls the car halts it instead of
 * being pushed over. TODO: tune with the gains. */
#define MOTION_STALL_FAULT_MSEC     2000u
#define WHEEL_CIRCUMFERENCE_MM       188u   // pi x 60 mm, 500 mm drive checked
#define WHEEL_BASE_MM                110u   // Tyre centres, ~11 cm, 2026-09-25

/* Motion control loop. Gains are in thousandths to avoid floating point,
 * and act once per tick. The car was tuned with KP 500, KI 50, KD 10 and
 * a limit of 20000 while the loop ran every 20 ms. At 10 ms the same
 * controller in real time is KP unchanged, KI halved, KD doubled and the
 * limit doubled, which keeps the integral's full authority at 1000 per
 * mille. Refine with ./flash.sh tuning, motion/tuning_report.md. */
#define MOTION_TICK_PERIOD_MSEC       10u
#define MOTION_PID_KP_MILLI          500u   // Per mm/s of error
#define MOTION_PID_KI_MILLI           25u   // Per mm/s of error per tick
#define MOTION_PID_KD_MILLI           20u   // Per mm/s change per tick
#define MOTION_PID_INTEGRAL_LIMIT  40000    // Anti windup, in mm/s * ticks
#define MOTION_DEFAULT_SPEED_MM_PER_SEC 200u
/* Wheel speed for every turn on the spot, whatever the driving speed is.
 * Slower turns overshoot less once the motors are cut and give the line
 * sensors longer over the line. MOTOR_MIN_DUTY is the floor under it, so
 * if turns stay too quick at a low value, that floor is what to lower. */
#define MOTION_TURN_SPEED_MM_PER_SEC  100u   // TODO: tune on the floor
/* Straight line correction on distance moves, encoders only. The wheel
 * that has travelled further since the move began is slowed and the other
 * sped up by this many mm/s per mm between them, at most half the move
 * speed. A 1 mm difference is half a degree of heading at WHEEL_BASE_MM.
 * Raise it if a 500 mm drive still curves; lower it if the car wags. */
#define MOTION_STRAIGHT_KP             4u   // TODO: tune, mm/s per mm
/* Speed at full duty. This converts a speed request into the feedforward
 * duty, so a wrong value leaves more for the PID to correct. 800 is the
 * 1600 estimated from a timed turn while the loops ran every 20 ms but
 * counted 10, halved now that they run every 10. TODO: measure it,
 * tuning_report.md section 2.
 *
 * NOTE: MOTOR_MIN_DUTY floors every nonzero duty, so speeds under about
 * 150/1000 * 800 = 120 mm/s cannot be held. */
#define MOTION_MAX_SPEED_MM_PER_SEC  800u   // TODO: measure at full duty
#define MOTION_MAX_STEER_PERMILLE   2000    // Over 1000 reverses inner wheel

/* Owner: Buddy 3, line and barcode. The values down to the next owner line. */
/* Line sensors, three MH-Sensor-Series IR reflective modules with an LM393
 * comparator and a digital output, one per Grove port. Left and right sit
 * at the front and straddle the line: on a straight both see floor, with
 * the 18 mm line between them, so their eyes want to be 24 to 28 mm apart,
 * each 12 to 14 mm from the line's centre. The third sits off to the left
 * and only reads barcodes; it never steers. Its eye must be over the
 * barcode, which starts 29 mm left of the line's centre (TRACK_* below).
 * The analog path is unused; LINE_SENSOR_IS_ANALOG stays 0. */
#define LINE_SENSOR_LEFT_PIN          26u   // Grove 5 yellow
#define LINE_SENSOR_BARCODE_PIN       27u   // Grove 6 yellow
#define LINE_SENSOR_RIGHT_PIN          3u   // Grove 2 yellow
#define LINE_SENSOR_IS_ANALOG          0u   // Must stay 0 on this layout
#define LINE_SENSOR_DARK_LEVEL         1u   // Level over black, verified

/* Pin pull. WARNING: This and LINE_SENSOR_DARK_LEVEL must disagree, or a
 * disconnected sensor reads as if it were over the line and mask 111 looks
 * exactly like a junction. Pull down with dark level 1 means a loose wire
 * reads 0, which shows up as a lost line instead of a false junction. Set
 * this to 1 only if a module pulls its output low but never drives it
 * high, which line_get_health() will show as a sensor that never reads
 * dark. */
#define LINE_SENSOR_PULL_UP            0u   // 0 pull down, 1 pull up
#define LINE_JUNCTION_SAMPLES          4u   // Both dark this many, 40 ms
/* Start-up check in line_calibrate(): the car starts straddling the line,
 * so every sensor must read floor, steadily, for this many samples, one
 * LINE_SAMPLE_PERIOD_MSEC apart. */
#define LINE_CALIBRATE_SAMPLES        20u

/* With neither line sensor dark the line is either centred between them or
 * gone, and two sensors cannot tell which. It counts as centred for this
 * long after either sensor last saw it, then as lost. Longer rides out a
 * well aligned straight; shorter notices an overshot curve sooner. */
#define LINE_CENTRED_HOLD_MSEC       500u   // TODO: tune on the track

/* Line sampling period, one kernel tick. That is fine for steering but
 * too coarse for a 3 mm bar, so the barcode sensor has its own sampler:
 * the RP2040 timer's alarm 0 interrupt reads it every BARCODE_SAMPLE_USEC
 * and timestamps each change. 500 us keeps a 3 mm bar readable up to
 * about 3 m/s. */
#define LINE_SAMPLE_PERIOD_MSEC       10u
#define BARCODE_SAMPLE_USEC          500u

/* Track geometry from the course. Junctions are crosses, so a junction
 * puts the line under both line sensors at once. Barcodes sit along the
 * track, beside the line on the left, TRACK_BARCODE_GAP_MM from its
 * edge; the car acts on the last one read at the next cross. A 141 mm
 * barcode is one Code 39 character between its start and stop symbols at
 * a 3 mm narrow element: 47 narrow widths in total. */
#define TRACK_LINE_WIDTH_MM           18u
#define TRACK_BARCODE_GAP_MM          20u   // Line edge to barcode
#define TRACK_BARCODE_LENGTH_MM      141u   // Along the line
#define BARCODE_NARROW_MM              3u
#define BARCODE_MAX_ELEMENT_MSEC     500u   // Longer runs reset the decoder

/* Code 39 characters that carry each navigation command, from the
 * barcode table in the course write-up. */
#define BARCODE_CHAR_LEFT             'A'
#define BARCODE_CHAR_RIGHT            'B'
#define BARCODE_CHAR_STRAIGHT         'C'
#define BARCODE_CHAR_UTURN            'D'

/* Owner: Buddy 4, IMU and terrain. The values down to the next owner line. */
/* IMU, an LSM303DLHC on I2C0 through Grove 3. Accelerometer and
 * magnetometer answer at two different addresses on the same bus. The
 * kernel driver defaults I2C0 to GP8 and GP9, which are the motor pins, so
 * imu_init() moves the peripheral to the pins below itself. */
#define IMU_I2C_DEVICE_NAME       "iica"    // Kernel I2C unit 0
#define IMU_I2C_SDA_PIN                4u   // Grove 3 white
#define IMU_I2C_SCL_PIN                5u   // Grove 3 yellow
#define IMU_ACCEL_I2C_ADDR          0x19u   // LSM303DLHC datasheet, SAD accel
#define IMU_MAG_I2C_ADDR            0x1Eu   // LSM303DLHC datasheet, SAD mag
#define IMU_SAMPLE_PERIOD_MSEC        10u   // 100 Hz
#define IMU_FILTER_SHIFT               5u   // Alpha 1/2^n, ~320 ms at 100 Hz
#define IMU_CALIBRATION_SAMPLES       32u   // Level reference average
/* Pitch is the direction of apparent gravity, so it is only meaningful
 * while the car is neither accelerating nor shaking. A running motor makes
 * the chassis vibrate and the vector wander, which reads as tilt that is
 * not there. Pitch is therefore frozen whenever the filtered acceleration
 * magnitude strays this far from one g. Widen it if pitch stops responding
 * on a real slope, narrow it if a vibrating car still reports tilt. */
#define IMU_PITCH_TRUST_BAND_MILLI_G 300u   // Was 150u, mast mount shakes
/* The same band while a hump is open. On a hump the slope keeps changing,
 * and a frozen pitch loses that change, where the vibration it would let
 * through averages out over the height integral. Keep it under 500: the
 * host test shakes the car at 1.5 g on the flat and expects a freeze. */
#define IMU_HUMP_TRUST_BAND_MILLI_G  450u   // TODO: tune on the real hump
#define IMU_HUMP_PITCH_THRESHOLD_DEG   5u   // TODO: tune on the real hump
/* Accelerometer full scale. 0 is plus or minus 2 g, 1 is 4 g, 2 is 8 g,
 * 3 is 16 g, and the milli g per count doubles with each step. The part
 * saturates at its full scale, so this is the hard ceiling on what a
 * collision can ever read: at 2 g a bumper knock is clipped to 2000 and
 * IMU_COLLISION_THRESHOLD_MILLI_G above 1000 can never be met. 8 g gives
 * room for a real impact and costs a little pitch resolution. */
#define IMU_ACCEL_FS_SELECT            2u   // 0 is 2 g, 2 is 8 g
#define IMU_ACCEL_MG_PER_LSB           4u   // 1 << IMU_ACCEL_FS_SELECT
#define IMU_COLLISION_THRESHOLD_MILLI_G  600u  // Over 1 g. Tune on peak
#define IMU_ACCEL_EVENT_MILLI_G      250u   // TODO: tune, forward accel event
#define IMU_TURN_EVENT_DPS            30u   // TODO: tune, turning event
#define IMU_EVENT_HOLD_SAMPLES        20u   // Samples an event holds, 200 ms
/* Which accelerometer axis points along the car: 0 is the module's X, 1
 * its Y, and the other one is then sideways. This car's module is turned
 * 90 degrees, found 2026-09-25 when tilting it sideways read as a climb.
 * IMU_PITCH_SIGN then picks which end of that axis is the nose. */
#define IMU_FORWARD_AXIS               1u
#define IMU_PITCH_SIGN                 1    // TODO: lift the nose, flip if < 0
/* Terrain roughness is the smoothed distance of the gravity vector from
 * one g, so it is near zero on a smooth floor and climbs with every bump.
 * The shift is the filter weight, the threshold is where stable ends. */
#define IMU_ROUGHNESS_SHIFT            5u   // Alpha 1/2^n, ~320 ms at 100 Hz
#define IMU_TERRAIN_ROUGH_MILLI_G    120u   // TODO: tune on the real course

/* Owner: Buddy 5, scanning. The values down to the next owner line. */
/* Ultrasonic ranging, an HC-SR04 on Grove 4. Values marked datasheet are
 * from it. NOTE: The module is powered from the Grove port's 3.3 V, on
 * purpose, so its echo output is a 3.3 V signal and needs no divider. The
 * price is maximum range, which is measured on the bench. */
#define SONAR_TRIG_PIN                16u   // Grove 4 yellow
#define SONAR_ECHO_PIN                17u   // Grove 4 white
#define SONAR_TRIG_PULSE_USEC         10u   // Datasheet minimum.
#define SONAR_USEC_PER_CM             58u   // Datasheet conversion.
#define SONAR_MIN_CYCLE_MSEC          60u   // Datasheet minimum cycle.
#define SONAR_MAX_RANGE_MM          4000u   // Datasheet.
#define SONAR_MIN_RANGE_MM            20u   // Datasheet.
#define SONAR_ECHO_TIMEOUT_USEC    30000u   // 400 cm * 58 = 23200, plus margin

/* Scan servo on header S1. The header is powered from the board's motor
 * rail, so the servo needs the battery, not USB power alone. Pulse range
 * is the board maker's calibrated example; the end stops vary per servo.
 * Angle 90 is straight ahead; angles above 90 look to the car's left.
 * TODO: confirm the direction on the bench and swap the pulse ends if not. */
#define SERVO_PIN                     12u   // S1, verified
#define SERVO_PWM_FREQ_HZ             50u   // Verified example value
/* Servo position is a pulse width, and the width below is the horn's
 * straight ahead as it is actually mounted on this car. Angles are measured
 * from it, rather than mapped across a 0 to 180 range, so that no commanded
 * angle can put the horn anywhere but a few degrees either side of where it
 * already rests.
 *
 * WARNING: A servo has no position feedback. The firmware cannot know where
 * the horn is until it sends a pulse, and the horn then jumps to whatever
 * that pulse says. That first jump is the one large movement the car ever
 * makes, and its size is the gap between where the horn was left and
 * SERVO_CENTRE_PULSE_USEC. Measure the width that points the horn straight
 * ahead with BENCH=servo and its centre finder, set it here, and the jump
 * disappears. */
#define SERVO_CENTRE_PULSE_USEC     1744u   // Measured on this car's mount
#define SERVO_USEC_PER_DEG            11u   // TODO: confirm, 1000 us per 90 deg

/* Hard limits on the computed pulse, the last line of defence. 1000 to
 * 2000 is the range every hobby servo accepts; most accept rather more.
 * The ceiling is above 2000 because this car's centre sits high at 1744,
 * so 115 degrees needs 2019 and would otherwise clamp silently to about
 * 113. If the horn binds at the top of the sweep, put this back to 2000
 * and the clamp will protect it again. */
#define SERVO_PULSE_MIN_USEC        1000u
#define SERVO_PULSE_MAX_USEC        2100u
#define SERVO_TRAVEL_DEG             180u   // TODO: confirm for this servo
#define SERVO_SETTLE_MSEC            200u   // TODO: find on the bench

/* Travel actually used. The servo reaches 0 to SERVO_TRAVEL_DEG, but the
 * mount, the wiring loom and the car's own body allow far less. Change
 * SCAN_HALF_SWEEP_DEG alone and every angle below follows it, including
 * the servo bench, so there is one number to edit and nothing to keep in
 * step by hand. scan_measure() clamps every commanded angle into the band,
 * so no code path can drive the horn into something.
 *
 * NOTE: At 25 degrees either side the three angles are 25 degrees apart,
 * which finally clears the sonar's own 15 degree beam. The left and right
 * readings are therefore independent of the centre one, and the clearance
 * either side of an obstacle means something for the first time, so
 * scan_plan_avoidance() can choose a side instead of always reversing.
 * Narrower than about 15 either side and all three readings fall inside
 * one beam again; see the scanning README. */
#define SCAN_CENTRE_ANGLE_DEG         90u   // Straight ahead
#define SCAN_HALF_SWEEP_DEG           25u   // 65, 90 and 115 degrees
#define SCAN_MIN_ANGLE_DEG   (SCAN_CENTRE_ANGLE_DEG - SCAN_HALF_SWEEP_DEG)
#define SCAN_MAX_ANGLE_DEG   (SCAN_CENTRE_ANGLE_DEG + SCAN_HALF_SWEEP_DEG)

/* Scan pattern and avoidance thresholds. The coarse set spans the band in
 * three steps; more would only re-read the same beam. */
#define SCAN_COARSE_ANGLES_DEG   { SCAN_MIN_ANGLE_DEG, SCAN_CENTRE_ANGLE_DEG, \
                                   SCAN_MAX_ANGLE_DEG }
#define SCAN_COARSE_ANGLE_COUNT        3u
/* Half the sweep, so a fine scan puts a reading between each pair of
 * coarse angles: 65, 77, 89, 101, 113. The coarse set stays at three, so
 * the scan that decides whether anything is there is still quick, and only
 * the profiling scan pays for the extra two readings. The floor keeps the
 * step at one degree or more whatever the sweep shrinks to, because a zero
 * step would leave scan_fine() unable to advance. */
#define SCAN_FINE_STEP_DEG \
    (((SCAN_HALF_SWEEP_DEG / 2u) > 0u) ? (SCAN_HALF_SWEEP_DEG / 2u) : 1u)
#define SCAN_FINE_HALF_SPAN_DEG SCAN_HALF_SWEEP_DEG
#define SCAN_OBSTACLE_RANGE_MM       200u   // TODO: tune, trigger distance
/* A lane is only a lane if the whole car fits down it with room to be
 * steered, so this is the car plus 50 mm each side. */
#define SCAN_CLEARANCE_MIN_MM  (CAR_WIDTH_MM + 100u)
#define SCAN_DETOUR_TURN_DEG          45u   // Each leg of the box detour
/* Every detour leg is sized from the car, not guessed. Sideways has to
 * move the car's own half width plus the obstacle's, and forward has to
 * carry the whole car past the obstacle before it turns back in. */
#define SCAN_DETOUR_SIN45_RECIP     1414u   // 1 / sin 45, times 1000
#define SCAN_DETOUR_MARGIN_MM  ((CAR_WIDTH_MM / 2u) + 50u)  // Half plus slack
/* Fallback legs, used when the fine scan could not measure a width: the
 * same sums with the obstacle taken as a point. The sideways leg is the
 * clearance divided by sin 45, because the car travels it at
 * SCAN_DETOUR_TURN_DEG rather than straight sideways. */
#define SCAN_DETOUR_SIDE_MM \
    ((SCAN_DETOUR_MARGIN_MM * SCAN_DETOUR_SIN45_RECIP) / 1000u)
#define SCAN_DETOUR_DEPTH_MM   (CAR_LENGTH_MM + SCAN_DETOUR_MARGIN_MM)
/* When a width was measured, the legs are sized from it instead: half
 * the obstacle to get clear of its edge, plus the car's own half width
 * and a margin, all divided by sin 45 for the diagonal. Depth is the
 * width plus CAR_LENGTH_MM, because nothing measures how deep an obstacle
 * is from the front, and the back of the car is still beside it when the
 * bumper has cleared. Both are clamped so a bad reading cannot send the
 * car across the room or clip the obstacle. The minima are the fallback
 * legs: nothing measured can justify less room than a point obstacle. */
#define SCAN_DETOUR_SIDE_MIN_MM      SCAN_DETOUR_SIDE_MM
#define SCAN_DETOUR_SIDE_MAX_MM      500u
#define SCAN_DETOUR_DEPTH_MIN_MM     SCAN_DETOUR_DEPTH_MM
#define SCAN_DETOUR_DEPTH_MAX_MM     600u
#define SCAN_REVERSE_MM              100u   // Back off before probing a side
/* Sweep the coarse angles while driving instead of staring straight
 * ahead, so the car knows which side is open before it has to choose.
 * The cost is real: the pattern is centre, side, centre, other side, so
 * forward is only ranged on every other reading and an obstacle can be
 * one reading closer before it is seen. Raise SCAN_OBSTACLE_RANGE_MM or
 * drop the speed to pay for it, or set this to 0 to stare ahead. */
#define SCAN_SWEEP_WHILE_MOVING        1u
#define SCAN_RECOVER_TURN_DEG         15u   // Search arc step
#define SCAN_RECOVER_DRIVE_MM         60u   // Search drive step
#define SCAN_RECOVER_STEPS            12u   // Turn and drive pairs, then halt

/* Owner: Buddy 1, comms. The values down to the next owner line. */
/* WiFi and MQTT. The kernel reads the SSID and password from its own
 * config/wifi_credentials.h, which git ignores. Nothing secret lives here. */
#define COMMS_MQTT_BROKER_HOST   "192.168.50.131"   // Verified
#define COMMS_MQTT_BROKER_PORT      1883u
#define COMMS_MQTT_CLIENT_ID     "car"
#define COMMS_TOPIC_TELEMETRY    "car/telemetry"
#define COMMS_TOPIC_HEARTBEAT    "car/heartbeat"
#define COMMS_TOPIC_COMMAND      "car/command"
#define COMMS_TOPIC_TERRAIN      "car/terrain"
#define COMMS_WIFI_TIMEOUT_MSEC    30000u
#define COMMS_MQTT_KEEPALIVE_SECONDS  60u
#define COMMS_POLL_PERIOD_MSEC        10u
#define COMMS_HEARTBEAT_PERIOD_MSEC 1000u
#define COMMS_RECONNECT_BACKOFF_MSEC 5000u
/* Telemetry is the longest payload: 268 bytes on a typical run and 336
 * with every field at its widest, measured on the host. The MQTT ring
 * slot, LWIP_UTK_MQTT_TX_PAYLOAD, must be at least this size, and comms.c
 * refuses to build if it is not: a slot smaller than the message drops
 * every telemetry message on the car without a word. The formatter drops
 * a message that does not fit rather than sending half an object. */
#define COMMS_PAYLOAD_MAX_BYTES      480u

/* Owner: the team. The values down to the next owner line. */
/* Vehicle controller timing and the line following law. Steering is a
 * proportional correction on the sensor error, expressed in
 * per mille of the set speed handed to motion_drive_steer(). */
#define CAR_MISSION_PERIOD_MSEC       10u
#define CAR_TELEMETRY_PERIOD_MSEC    200u
#define CAR_TERRAIN_PERIOD_MSEC     2000u   // The terrain report's status
#define CAR_FOLLOW_SPEED_MM_PER_SEC  150u   // TODO: raise once following works
#define CAR_BARCODE_SPEED_MM_PER_SEC 100u   // Slower so every bar is sampled
/* Terrain speed. Climbing keeps the asked speed: the speed loop adds
 * power gradually as the slope slows the wheels, where jumping to a climb
 * speed made the car lunge up the hump. Descending, or rough ground, drops
 * to this, or the car runs away and lands hard. */
#define CAR_DESCEND_SPEED_MM_PER_SEC 120u   // TODO: tune, floored for now
/* Steering with two straddling sensors is proportional only. The error
 * only ever jumps between 0 and 2, so a derivative term would be a pure
 * kick: full lock when a sensor hits the line, then a hard swing the other
 * way the instant it clears, which threw the car from one sensor to the
 * other until it broke free. KP 150 steers 300 per mille at the edge:
 * inner wheel 70 percent, outer 130. Raise KP in steps of 25 if the car
 * drifts off before correcting; lower it if it still swings side to side. */
#define CAR_STEER_KP_PERMILLE        150    // Per unit of error, 2 at an edge
/* Coming back onto the line after a detour, a search, or losing it: the car
 * crosses the line, creeps its wheels over it, and turns on the spot until
 * the second sensor touches it (reenter_line() in car_main.c). The spin
 * stops on the sensor; this only bounds it if it never sees the line. */
#define CAR_REENTRY_SPIN_MAX_DEG     120u
#define CAR_STEER_LOST_PERMILLE      900    // Applied toward the last seen side
#define CAR_LINE_SEEN_SAMPLES         40u   // Ticks seen before speeding up
#define CAR_LINE_LOST_STRAIGHT_MM    200u   // Gap plus barcode plus margin
#define CAR_LINE_LOST_LIMIT_MM       600u   // Beyond this, search for the line

/* An obstacle that is still there after CAR_MAX_DETOUR_ATTEMPTS trips round
 * it is not one this car can drive round, so it reverses instead. Attempts
 * reset after CAR_DETOUR_CLEAR_MM of following with nothing in the way,
 * which is the only evidence the obstacle is really behind us. */
#define CAR_MAX_DETOUR_ATTEMPTS        4u   // Two probes each side
/* A collision outranks everything. The car cannot tell where it was hit,
 * so it stops dead, backs straight off far enough to give the scan room
 * to work, and then looks before moving again. Repeated hits mean
 * something is wrong that backing off will not fix. */
#define CAR_COLLISION_BACKOFF_MM     150u   // Further than SCAN_REVERSE_MM
#define CAR_COLLISION_SETTLE_MSEC    400u   // Let the chassis stop ringing
#define CAR_MAX_COLLISIONS             3u   // Then stop for good
#define CAR_DETOUR_CLEAR_MM          400u   // Clear run that ends a detour
/* Leg index past which the obstacle is behind the car, so seeing the line
 * again means the detour worked. Legs 0 to 3 are the turn out, the step
 * sideways, the turn back and the drive past, and the line crossed during
 * those is the one the car just left. */
#define CAR_DETOUR_PAST_INDEX          4u

/* Junctions. Left and right line sensors dark together is a junction.
 * The car stops there and waits CAR_JUNCTION_WAIT_MSEC so that a barcode
 * the decoder is still finishing, or a remote command, can arrive. Then it
 * turns as the last barcode read since the previous junction said, or
 * goes straight on if none was read. */
#define CAR_JUNCTION_WAIT_MSEC       500u   // Stopped, waiting for a command
/* Forward from where the car stops at a junction to where its wheels, the
 * point it spins about, sit over the crossing. The sensors are at the
 * front, so turning where they first see the junction would leave the car
 * beside the new branch instead of on it. U-turns do not creep. Coming
 * back onto the line uses it as the sensor to wheel distance too.
 * Measured sensor eye to tyre centres, about 150 mm. At a junction the car
 * also coasts a little past where it saw the crossing: if left and right
 * turns land past the new branch, take that coast off this. */
#define CAR_JUNCTION_CREEP_MM        150u
#define CAR_SONAR_CHECK_PERIOD_MSEC   60u   // Forward ping while following
#define CAR_UTURN_DEG                180u

/* Logging threshold, matches car_log_level_t: 0 error, 1 info, 2 debug. */
#define CAR_LOG_LEVEL                  1

#endif /* CAR_CONFIG_H */

/*** end of file ***/
