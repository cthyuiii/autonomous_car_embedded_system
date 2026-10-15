# motion

Owner: Buddy 2, motion control.

Owns motor PWM, encoder counting, speed control, distance, turns and the
steering the line follower drives through. The API is in
`include/motion.h`, the values are `MOTOR_*`, `ENCODER_*`, `MOTION_*` and
`WHEEL_*` in `common/car_config.h`.

Each wheel has a speed loop: a feedforward duty from the target speed plus
a PID correction on the measured speed. Speed comes from the time between
encoder pulses. Phase B, read at every phase A edge, gives the direction,
and a reversal counts only once `ENCODER_DIRECTION_PULSES` pulses in a row
agree. Distance moves hold their heading (`MOTION_STRAIGHT_KP`). Turns on
the spot run at `MOTION_TURN_SPEED_MM_PER_SEC`. Every speed-up ramps at
`MOTION_ACCEL_MM_PER_SEC2` with the slower wheel keeping its share, and
slowing is immediate.

The car needs both encoders. A wheel that never pulsed reads zero speed, so
its loop drives it hard and the car veers. `motion_get_state()` reports
which encoders were found. A wheel that pulsed and then stopped while
driven is a fault: `motion_tick()` returns `CAR_ERR_HARDWARE` after
`MOTION_STALL_FAULT_MSEC`, and the car handles it like a collision.

The motion bench drives M1 alone, then M2, then both, so a wheel that turns
one way but not the other points at that motor, its terminal or that board
channel.

Tests: `test_motion.c` on the host, `./flash.sh tuning` on the floor.
`tuning_report.md` is the PID tuning and motion accuracy report.

| Calibration                  | Value           | How                                  |
|------------------------------|-----------------|--------------------------------------|
| ENCODER_SLOTS_PER_REV        | 639             | `./flash.sh encoders`, one hand turn |
| ENCODER_*_B_FORWARD          | left 0, right 1 | `./flash.sh encoders`, flip a wheel that reads `back` |
| WHEEL_CIRCUMFERENCE_MM       | 205             | 500 mm drives against a tape         |
| WHEEL_LEFT_PERMILLE, WHEEL_RIGHT_PERMILLE | 993, 1007 | sideways drift at the end of 500 mm drives |
| WHEEL_BASE_MM                | 110             | 90 degree turns against a protractor |
| MOTOR_MIN_DUTY               | 70              | `./flash.sh duty`, the higher wheel's stop value |
| KP, KI, KD in milli          | 500, 25, 20     | step response, `tuning_report.md` section 2 |
| MOTION_STRAIGHT_KP           | 4               | 500 mm drives                        |
| MOTION_TURN_SPEED_MM_PER_SEC | 60              | 100 overshot by up to 3.7 degrees    |

`MOTOR_MIN_DUTY` floors every nonzero duty, so it sets the slowest speed
the car can hold, about 20 mm/s. `./flash.sh duty` ramps the raw duty up
and down on the floor and prints where each wheel starts and stops. Set the
floor to the higher stop value.
