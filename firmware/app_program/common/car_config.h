/** @file car_config.h
 *
 * @brief Every pin, gain, threshold and timing the car depends on.
 *
 * NOTE: Pins marked verified come from the board maker's own example code.
 * Everything else is a starting guess that the bench programs exist to
 * replace with a measurement. Change values here, never in the modules.
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * Board: Cytron Robo Pico carrying a Raspberry Pi Pico W. The board owns
 * GP8 to GP11 (motor driver), GP12 to GP15 (servo headers S1 to S4), GP18
 * (two NeoPixels), GP20 and GP21 (buttons) and GP22 (buzzer). The Pico W
 * radio owns GP23, GP24, GP25 and GP29. The kernel console is UART0 on GP0
 * and GP1, which is Grove port 1, even in the USB console profile. That
 * leaves GP2 to GP7, GP16, GP17, GP19 and GP26 to GP28 on the Grove and
 * Maker ports for sensors. The car needs nine of them.
 */

/* Motor driver, on the board. Two PWM pins per motor, verified. PWM
 * frequency is the board maker's example value. */
#define MOTOR_LEFT_IN1_PIN             8u   // M1A, verified
#define MOTOR_LEFT_IN2_PIN             9u   // M1B, verified
#define MOTOR_RIGHT_IN1_PIN           10u   // M2A, verified
#define MOTOR_RIGHT_IN2_PIN           11u   // M2B, verified
#define MOTOR_PWM_FREQ_HZ          10000u   // TODO: tune on hardware
#define MOTOR_PWM_MAX_DUTY          1000u   // Duty is expressed per mille.

/* Wheel encoders, one TCRT5000 plus LM393 per wheel, one channel each, on
 * a Grove port. NOTE: A single channel cannot sense direction. Direction
 * is taken from the commanded motor sign instead. The RP2040 raises one
 * interrupt for the whole GPIO bank; the handler sorts out which pin. */
#define ENCODER_LEFT_PIN               6u   // TODO: confirm Grove port
#define ENCODER_RIGHT_PIN              7u   // TODO: confirm Grove port
#define ENCODER_IRQ_NUM               13u   // IO_IRQ_BANK0, TODO: confirm
#define ENCODER_IRQ_LEVEL              2    // TODO: confirm kernel levels
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

/* Line sensors, three IR reflective sensors on the three ADC capable pins,
 * so either the digital LM393 modules or the analog TCRT5000 path fits
 * without moving a wire. See line_barcode.h for the tradeoff. */
#define LINE_SENSOR_LEFT_PIN          26u   // TODO: confirm Grove port
#define LINE_SENSOR_CENTRE_PIN        27u   // TODO: confirm Grove port
#define LINE_SENSOR_RIGHT_PIN         28u   // TODO: confirm Grove port
#define LINE_SENSOR_IS_ANALOG          0u   // 0 LM393 digital, 1 TCRT5000 ADC
#define LINE_ANALOG_THRESHOLD       2048u   // TODO: calibrate, 12 bit ADC

/* Line sampling period. The kernel tick is CNF_TIMER_PERIOD in
 * config/config.h, 10 ms as shipped, and a task delay rounds up to it. At
 * 200 mm per second a 10 ms sample is 2 mm of travel; at 400 it is 4 mm.
 * TODO: If the narrowest bar gets fewer than three samples at run speed,
 * lower CNF_TIMER_PERIOD and this value together. */
#define LINE_SAMPLE_PERIOD_MSEC       10u

/* Track geometry from the course specification. */
#define TRACK_LINE_WIDTH_MM           20u
#define TRACK_BARCODE_GAP_MM          18u
#define TRACK_BARCODE_LENGTH_MM      141u

/* IMU, an LSM303DLHC on I2C0. Accelerometer and magnetometer answer at two
 * different addresses on the same bus. */
#define IMU_I2C_INDEX                  0u   // Kernel I2C device unit
#define IMU_I2C_SDA_PIN                4u   // TODO: confirm Grove port
#define IMU_I2C_SCL_PIN                5u   // TODO: confirm Grove port
#define IMU_I2C_BAUD_HZ           400000u
#define IMU_ACCEL_I2C_ADDR          0x19u   // TODO: confirm, LSM303DLHC sheet
#define IMU_MAG_I2C_ADDR            0x1Eu   // TODO: confirm, LSM303DLHC sheet
#define IMU_SAMPLE_PERIOD_MSEC        10u   // 100 Hz
#define IMU_HUMP_PITCH_THRESHOLD_DEG   5u   // TODO: tune on the real hump
#define IMU_COLLISION_THRESHOLD_MILLI_G 2000u  // TODO: tune, 2 g is a guess
#define IMU_TURN_RATE_FROM_ENCODERS    1u   // 1 encoders, 0 magnetometer

/* Ultrasonic ranging, an HC-SR04 on a Grove port. Values marked datasheet
 * are from it. WARNING: Echo is a 5 V output. Divide it down before the
 * Pico pin; the board adds no level shifting. */
#define SONAR_TRIG_PIN                16u   // TODO: confirm Grove port
#define SONAR_ECHO_PIN                17u   // TODO: confirm Grove port
#define SONAR_TRIG_PULSE_USEC         10u   // Datasheet minimum.
#define SONAR_USEC_PER_CM             58u   // Datasheet conversion.
#define SONAR_MIN_CYCLE_MSEC          60u   // Datasheet minimum cycle.
#define SONAR_MAX_RANGE_MM          4000u   // Datasheet.
#define SONAR_MIN_RANGE_MM            20u   // Datasheet.
#define SONAR_BEAM_ANGLE_DEG          15u   // Datasheet.
#define SONAR_ECHO_TIMEOUT_USEC    30000u   // 400 cm * 58 = 23200, plus margin

/* Scan servo on header S1. The header is powered from the board's motor
 * rail, so the servo needs the battery, not USB power alone. Pulse range
 * is the board maker's calibrated example; the end stops vary per servo. */
#define SERVO_PIN                     12u   // S1, verified
#define SERVO_PWM_FREQ_HZ             50u   // Verified example value
#define SERVO_PULSE_MIN_USEC         580u   // TODO: find the end stop
#define SERVO_PULSE_MAX_USEC        2700u   // TODO: find the end stop
#define SERVO_TRAVEL_DEG             180u   // TODO: confirm for this servo
#define SERVO_SETTLE_MSEC            200u   // TODO: find on the bench

/* Scan pattern and avoidance thresholds. */
#define SCAN_COARSE_ANGLES_DEG      { 30u, 60u, 90u, 120u, 150u }
#define SCAN_COARSE_ANGLE_COUNT        5u
#define SCAN_FINE_STEP_DEG            15u   // WARNING: not below beam angle
#define SCAN_OBSTACLE_RANGE_MM       200u   // TODO: tune, trigger distance
#define SCAN_CLEARANCE_MIN_MM        150u   // TODO: car width plus margin

/* WiFi and MQTT. The kernel reads the SSID and password from its own
 * config/wifi_credentials.h, which git ignores. Nothing secret lives here. */
#define COMMS_MQTT_BROKER_HOST   "192.168.1.10"   // TODO: confirm broker IP
#define COMMS_MQTT_BROKER_PORT      1883u
#define COMMS_MQTT_CLIENT_ID     "car"
#define COMMS_TOPIC_TELEMETRY    "car/telemetry"
#define COMMS_TOPIC_HEARTBEAT    "car/heartbeat"
#define COMMS_TOPIC_COMMAND      "car/command"
#define COMMS_WIFI_TIMEOUT_MSEC    30000u
#define COMMS_MQTT_KEEPALIVE_SECONDS  60u
#define COMMS_POLL_PERIOD_MSEC        10u
#define COMMS_HEARTBEAT_PERIOD_MSEC 1000u
#define COMMS_RECONNECT_BACKOFF_MSEC 5000u

/* Vehicle controller timing. */
#define CAR_MISSION_PERIOD_MSEC       10u
#define CAR_TELEMETRY_PERIOD_MSEC    200u

/* Logging threshold, matches car_log_level_t: 0 error, 1 info, 2 debug. */
#define CAR_LOG_LEVEL                  1

#endif /* CAR_CONFIG_H */

/*** end of file ***/

