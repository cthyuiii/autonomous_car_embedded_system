/** @file car_config.h
 *
 * @brief Every pin, gain, threshold and timing the car depends on.
 *
 * NOTE: Nothing here is verified. Each value is a starting guess that the
 * bench programs exist to replace with a measurement. Change values here,
 * never in the modules.
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * Pin budget. The board is a Cytron Maker Pi Pico carrying a Pico W. The
 * board commits GP10 to GP22 and GP28 to onboard peripherals, leaving GP0 to
 * GP9, GP26 and GP27 free, which is 12 pins against the 14 needed. GP16 and
 * GP17 are reclaimed from the ESP-01 socket, which is unused because the
 * Pico W does its own WiFi. See README.md for the full table.
 */

/* Motor driver.
 * NOTE: Assumes a DRV8833 style bridge with two PWM pins per motor. An L298N
 * needs one PWM plus two direction pins per motor, two more pins in total,
 * which forces the micro SD pins GP10 to GP15 to be reclaimed as well. */
#define MOTOR_LEFT_IN1_PIN             8u   // TODO: confirm against wiring
#define MOTOR_LEFT_IN2_PIN             9u   // TODO: confirm against wiring
#define MOTOR_RIGHT_IN1_PIN            6u   // TODO: confirm against wiring
#define MOTOR_RIGHT_IN2_PIN            7u   // TODO: confirm against wiring
#define MOTOR_PWM_FREQ_HZ          20000u   // TODO: tune, above audible range
#define MOTOR_PWM_MAX_DUTY          1000u   // Duty is expressed per mille.

/* Wheel encoders, one TCRT5000 plus LM393 per wheel, one channel each.
 * NOTE: A single channel cannot sense direction. Direction is taken from
 * the commanded motor sign instead. */
#define ENCODER_LEFT_PIN               2u   // TODO: confirm against wiring
#define ENCODER_RIGHT_PIN              3u   // TODO: confirm against wiring
#define ENCODER_SLOTS_PER_REV         20u   // TODO: count the disc slots
#define WHEEL_CIRCUMFERENCE_MM       204u   // TODO: measure, roll one turn
#define WHEEL_BASE_MM                150u   // TODO: measure, centre to centre

/* Motion control loop. Gains are in thousandths to avoid floating point. */
#define MOTION_TICK_PERIOD_MSEC       10u
#define MOTION_PID_KP_MILLI          500u   // TODO: tune on hardware
#define MOTION_PID_KI_MILLI           50u   // TODO: tune on hardware
#define MOTION_PID_KD_MILLI           10u   // TODO: tune on hardware
#define MOTION_DEFAULT_SPEED_MM_PER_SEC 200u
#define MOTION_MAX_SPEED_MM_PER_SEC  400u   // TODO: measure at full duty

/* Line sensors, three IR reflective sensors.
 * NOTE: GP26 and GP27 are ADC capable so the analog TCRT5000 path needs only
 * the right sensor moved. GP28, the third ADC, drives the onboard NeoPixel.
 * See line_barcode.h for the digital versus analog tradeoff. */
#define LINE_SENSOR_LEFT_PIN          26u   // TODO: confirm against wiring
#define LINE_SENSOR_CENTRE_PIN        27u   // TODO: confirm against wiring
#define LINE_SENSOR_RIGHT_PIN          0u   // TODO: confirm against wiring
#define LINE_SENSOR_IS_ANALOG          0u   // 0 LM393 digital, 1 TCRT5000 ADC
#define LINE_ANALOG_THRESHOLD       2048u   // TODO: calibrate, 12 bit ADC
#define LINE_SAMPLE_PERIOD_MSEC        5u   // TODO: derive from speed, below

/* Track geometry from the course specification. At speed v mm per second a
 * bar of width w mm passes the sensor in w / v seconds, which bounds the
 * sample period the barcode decoder can tolerate. */
#define TRACK_LINE_WIDTH_MM           20u
#define TRACK_BARCODE_GAP_MM          18u
#define TRACK_BARCODE_LENGTH_MM      141u

/* IMU, an LSM303DLHC on I2C. Accelerometer and magnetometer answer at two
 * different addresses on the same bus. */
#define IMU_I2C_INDEX                  0u   // i2c0
#define IMU_I2C_SDA_PIN                4u   // TODO: confirm Grove port pins
#define IMU_I2C_SCL_PIN                5u   // TODO: confirm Grove port pins
#define IMU_I2C_BAUD_HZ           400000u
#define IMU_ACCEL_I2C_ADDR          0x19u   // TODO: confirm, LSM303DLHC sheet
#define IMU_MAG_I2C_ADDR            0x1Eu   // TODO: confirm, LSM303DLHC sheet
#define IMU_SAMPLE_PERIOD_MSEC        10u   // 100 Hz
#define IMU_HUMP_PITCH_THRESHOLD_DEG   5u   // TODO: tune on the real hump
#define IMU_COLLISION_THRESHOLD_MILLI_G 2000u  // TODO: tune, 2 g is a guess
#define IMU_TURN_RATE_FROM_ENCODERS    1u   // 1 encoders, 0 magnetometer

/* Ultrasonic ranging, an HC-SR04. Values marked datasheet are from it.
 * WARNING: Echo is a 5 V output. Divide it down before GP17. */
#define SONAR_TRIG_PIN                16u   // NOTE: reclaimed from ESP-01
#define SONAR_ECHO_PIN                17u   // NOTE: reclaimed from ESP-01
#define SONAR_TRIG_PULSE_USEC         10u   // Datasheet minimum.
#define SONAR_USEC_PER_CM             58u   // Datasheet conversion.
#define SONAR_MIN_CYCLE_MSEC          60u   // Datasheet minimum cycle.
#define SONAR_MAX_RANGE_MM          4000u   // Datasheet.
#define SONAR_MIN_RANGE_MM            20u   // Datasheet.
#define SONAR_BEAM_ANGLE_DEG          15u   // Datasheet.
#define SONAR_ECHO_TIMEOUT_USEC    30000u   // 400 cm * 58 = 23200, plus margin

/* Scan servo, a generic hobby servo on a PWM pin with its own 5 V supply.
 * WARNING: Never power the servo from the Pico 3V3 pin. */
#define SERVO_PIN                      1u   // TODO: confirm against wiring
#define SERVO_PWM_FREQ_HZ             50u   // TODO: confirm for this servo
#define SERVO_PULSE_MIN_USEC         500u   // TODO: confirm for this servo
#define SERVO_PULSE_MAX_USEC        2500u   // TODO: confirm for this servo
#define SERVO_TRAVEL_DEG             180u   // TODO: confirm for this servo
#define SERVO_SETTLE_MSEC            200u   // TODO: find on the bench

/* Scan pattern and avoidance thresholds. */
#define SCAN_COARSE_ANGLES_DEG      { 30u, 60u, 90u, 120u, 150u }
#define SCAN_COARSE_ANGLE_COUNT        5u
#define SCAN_FINE_STEP_DEG            15u   // WARNING: not below beam angle
#define SCAN_OBSTACLE_RANGE_MM       200u   // TODO: tune, trigger distance
#define SCAN_CLEARANCE_MIN_MM        150u   // TODO: car width plus margin

/* WiFi and MQTT.
 * WARNING: These are placeholders. Real credentials must never be committed.
 * Load them from an untracked header or a build flag once they exist. */
#define COMMS_WIFI_SSID          "CHANGE_ME"
#define COMMS_WIFI_PASSWORD      "CHANGE_ME"
#define COMMS_MQTT_BROKER_HOST   "192.168.1.10"   // TODO: confirm broker IP
#define COMMS_MQTT_BROKER_PORT      1883u
#define COMMS_MQTT_CLIENT_ID     "car"
#define COMMS_TOPIC_TELEMETRY    "car/telemetry"
#define COMMS_TOPIC_HEARTBEAT    "car/heartbeat"
#define COMMS_TOPIC_COMMAND      "car/command"
#define COMMS_WIFI_TIMEOUT_MSEC    30000u
#define COMMS_MQTT_KEEPALIVE_SECONDS  60u
#define COMMS_HEARTBEAT_PERIOD_MSEC 1000u
#define COMMS_RECONNECT_BACKOFF_MSEC 5000u

/* Vehicle controller timing. */
#define CAR_MAIN_LOOP_PERIOD_MSEC     10u
#define CAR_TELEMETRY_PERIOD_MSEC    200u

/* Logging threshold, matches car_log_level_t: 0 error, 1 info, 2 debug. */
#define CAR_LOG_LEVEL                  1

#endif /* CAR_CONFIG_H */

/*** end of file ***/

