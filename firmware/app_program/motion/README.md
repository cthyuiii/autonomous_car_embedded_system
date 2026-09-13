# motion

Owns motor PWM, encoder counting, PID speed control, distance and turns.
API is `include/motion.h`. Knobs are `MOTOR_*`, `ENCODER_*` and `MOTION_*`
in `common/car_config.h`. Direction comes from the commanded sign, since a
single channel encoder cannot sense it.

Done means `test_motion.c` passes on the host, and `bench_motion` drives
500 mm within 5 percent three runs in a row with the wheels on the floor.

| Calibration            | Measured | How                              |
|------------------------|----------|----------------------------------|
| WHEEL_CIRCUMFERENCE_MM |          | roll one revolution, measure     |
| ENCODER_SLOTS_PER_REV  |          | count the disc                   |
| WHEEL_BASE_MM          |          | wheel centre to wheel centre     |
| KP / KI / KD in milli  |          | step response, record overshoot  |
| Left vs right duty     |          | same setpoint, compare counts    |
