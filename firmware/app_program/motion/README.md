# motion

Owner: Buddy 2, motion control.

Owns motor PWM, encoder counting, speed control, distance, turns and the
steering primitive the line follower drives through. API is
`include/motion.h`. Knobs are `MOTOR_*`, `ENCODER_*` and `MOTION_*` in
`common/car_config.h`. Direction comes from the commanded sign, since a
single channel encoder cannot sense it.

Implemented. `MOTION_OPEN_LOOP` is 1 until both encoders are wired: moves
and turns are then timed from the speed setpoint, the PID is compiled but
idle, and `motion_tick()` never reports a stall. Even with it at 0, a wheel
that has never produced a single pulse is treated as having no encoder
fitted and is driven open loop; only a wheel that pulsed and then went
quiet while still commanded is a fault. `motion_get_state()` reports which
encoders were actually found.

Set `MOTION_OPEN_LOOP` to 0 once the encoder leads are on Grove 2 and 7,
then tune the gains below. Speed is measured from the interval between
pulses, so it resolves properly even at 20 pulses per turn.

The bench drives M1 alone, then M2 alone, then both, before it measures
anything. One wheel turning in its own phase and not the other isolates
the fault to that motor, its terminal, or that channel of the board.

Done means `test_motion.c` passes on the host, and `bench_motion` drives
500 mm within 5 percent three runs in a row with the wheels on the floor.

| Calibration            | Measured | How                              |
|------------------------|----------|----------------------------------|
| WHEEL_CIRCUMFERENCE_MM |          | roll one revolution, measure     |
| ENCODER_SLOTS_PER_REV  |          | pulses per wheel turn, by hand   |
| WHEEL_BASE_MM          |          | wheel centre to wheel centre     |
| MOTOR_MIN_DUTY         |          | lowest per mille that moves      |
| MOTION_MAX_SPEED       |          | mm per second at full duty       |
| KP / KI / KD in milli  |          | step response, record overshoot  |
| Left vs right duty     |          | same setpoint, compare counts    |

## Calibrating open loop speed

Nothing is counted while `MOTION_OPEN_LOOP` is 1. A move runs for
`distance / speed` seconds, so if the car's real speed does not match the
commanded one, every distance and every turn is wrong by that same ratio.
`MOTION_MAX_SPEED_MM_PER_SEC` is the only knob that sets the ratio.

Measure it rather than guessing, because a turn is the easiest place to see
the error but the hardest place to measure it:

1. `./flash.sh motion` and let the car drive a straight leg of a known
   commanded distance on the floor.
2. Measure how far it actually went with a tape.
3. Multiply `MOTION_MAX_SPEED_MM_PER_SEC` by actual / commanded.

A 45 degree turn coming out at 180 means the same four times over, which
is how the current 1600 was arrived at. Redo it with a tape when there is
time; the turn is a coarse instrument.

`MOTOR_MIN_DUTY` deserves the same treatment. It floors every duty, so it
sets the slowest the car can physically go: at 150 per mille and a
1600 mm/s top speed that floor is 240 mm/s, above both
`CAR_FOLLOW_SPEED_MM_PER_SEC` and `CAR_BARCODE_SPEED_MM_PER_SEC`. Find the
lowest duty that still starts the car from rest on carpet and set it to
that, or the barcode will always be read at 240 mm/s whatever the config
asks for.

Once encoders are wired and `MOTION_OPEN_LOOP` goes to 0, none of this
matters: distance comes from counted pulses and the PID corrects speed.
