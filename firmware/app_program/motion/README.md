# motion

Owner: Buddy 2, motion control.

Owns motor PWM, encoder counting, speed control, distance, turns and the
steering primitive the line follower drives through. API is
`include/motion.h`. Knobs are `MOTOR_*`, `ENCODER_*` and `MOTION_*` in
`common/car_config.h`.

Each wheel has a speed loop: a feedforward duty from the target speed plus
a PID correction on the measured speed. Speed is measured from the time
between encoder pulses, and phase B, read at every phase A edge, gives the
direction; a reversal only counts once `ENCODER_DIRECTION_PULSES` pulses
in a row agree. Distance moves also hold their heading
(`MOTION_STRAIGHT_KP`), and turns on the spot run at
`MOTION_TURN_SPEED_MM_PER_SEC`.

The car needs both encoders. A wheel that has never pulsed reads zero
speed, so its speed loop drives it hard and the car veers;
`motion_get_state()` reports which encoders were found. A wheel that
pulsed and then stopped while still driven is a fault: `motion_tick()`
returns `CAR_ERR_HARDWARE` after `MOTION_STALL_FAULT_MSEC`, and the car
handles it like a collision.

The motion bench drives M1 alone, then M2 alone, then both, before it
measures anything. One wheel turning in its own phase and not the other
isolates the fault to that motor, its terminal, or that channel of the
board.

Done means `test_motion.c` passes on the host, and `./flash.sh tuning`
drives 500 mm within 5 percent three runs in a row on the floor.

| Calibration            | Value | How                              |
|------------------------|-------|----------------------------------|
| ENCODER_SLOTS_PER_REV  | 639   | `./flash.sh encoders`, count change over one hand turn |
| ENCODER_*_B_FORWARD    | left 0, right 1 | `./flash.sh encoders`: a wheel reading `back` while turned forward gets flipped |
| WHEEL_CIRCUMFERENCE_MM | 188   | 500 mm floor drive matched the tape |
| WHEEL_BASE_MM          | 110   | tyre centres; confirm with the turn test in `tuning_report.md` |
| MOTOR_MIN_DUTY         |       | `./flash.sh duty` on the floor: the higher wheel's "keeps turning down to" |
| MOTION_MAX_SPEED       |       | steady error in the step test, `tuning_report.md` section 2 |
| KP / KI / KD in milli  |       | step response, `tuning_report.md` section 2 |
| MOTION_STRAIGHT_KP     |       | 500 mm drive, sideways drift at the end |

`./flash.sh tuning` measures the step response and the accuracy of drives
and turns, and prints table rows for `tuning_report.md`, the PID tuning
report and motion accuracy evaluation the write-up asks for.

`MOTOR_MIN_DUTY` floors every nonzero duty, so it sets the slowest speed
the car can hold: at 150 per mille and 800 mm/s at full duty that is about
120 mm/s. `./flash.sh duty` ramps the raw duty up and back down on the
floor and prints where each wheel starts and where it stops. Set the floor
to the higher stop value: the speed loop pushes past it to start a wheel
from rest.
