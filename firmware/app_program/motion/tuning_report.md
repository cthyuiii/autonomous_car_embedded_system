# Motion control: PID tuning report and motion accuracy evaluation

Owner: Buddy 2, motion control. The write-up asks for both deliverables.

**Status:** sections 1 to 3 and 6 are complete. Sections 4 and 5 fill in
from one run of `./flash.sh tuning`, whose console output prints the table
rows ready to paste, plus a tape measure and a protractor.

## 1. What is controlled

Each wheel has its own speed loop, run every motion tick:

```
duty = target * 1000 / MOTION_MAX_SPEED_MM_PER_SEC      feedforward
     + (KP * e + KI * sum(e) + KD * (e - e_prev)) / 1000  PID, e in mm/s
```

- **Speed** is measured from the time between encoder pulses, not from
  pulses per tick. At 639 pulses per wheel turn and 188 mm per turn, one
  pulse is 0.29 mm.
- **Direction** comes from phase B, read at each phase A edge. A reversal
  only counts after 4 pulses in a row agree. A wheel turning against its
  command is a negative speed, so the PID pushes harder.
- **The integral** is clamped at ±20000 (anti-windup). It is reset only
  when a wheel changes direction, so steering updates every tick don't
  wipe it.
- **Duty floor:** any non-zero duty is floored at `MOTOR_MIN_DUTY`, below
  which the motor only hums.
- **Straight-line correction:** distance moves also trim the two targets.
  The wheel that has travelled further since the move began is slowed,
  and the other sped up, by `MOTION_STRAIGHT_KP` mm/s per mm between them.
  This is a proportional loop on heading, because on the spot the heading
  is that difference divided by the wheel base.
- **Turns** spin on the spot at `MOTION_TURN_SPEED_MM_PER_SEC`. Each wheel
  covers the arc `WHEEL_BASE_MM * pi * angle / 360`.
- **Stall:** a driven wheel silent for `MOTION_STALL_FAULT_MSEC` stops
  the car. The mission treats that as a collision.

| Setting | Value | Source |
|---|---|---|
| `MOTION_PID_KP_MILLI` / `KI` / `KD` | 500 / 25 / 20 | The 20 ms tuning carried to 10 ms, see section 3 |
| `MOTION_PID_INTEGRAL_LIMIT` | 40000 | mm/s × ticks, full authority 1000 ‰ |
| `MOTION_MAX_SPEED_MM_PER_SEC` | 800 | Half the 1600 eyeballed from a timed turn at the old 20 ms loop |
| `MOTOR_MIN_DUTY` | 150 ‰ | Not yet measured |
| `ENCODER_SLOTS_PER_REV` | 639 | Hand turned: 638, 639, 640 |
| `WHEEL_CIRCUMFERENCE_MM` | 188 | π × 60 mm, checked by a 500 mm drive |
| `WHEEL_BASE_MM` | 110 | Tyre centres |
| `ENCODER_DIRECTION_PULSES` | 4 | About 1.2 mm of travel |
| `MOTION_TURN_SPEED_MM_PER_SEC` | 100 | Section 3 |
| `MOTION_STRAIGHT_KP` | 4 mm/s per mm | Not yet tuned |

## 2. Tuning method

**Step test.** The bench's part 1 lifts the car and steps both wheels
from rest to 100, 200 and 300 mm/s. Each step runs 3 s. Per wheel it
reports:

- **Rise:** time from 10 % to 90 % of the target;
- **Overshoot:** peak above the target, as a percentage;
- **Steady error:** mean speed minus the target over the last second;
- **Ripple:** highest minus lowest speed over the same second.

**Targets.** Rise under 300 ms, overshoot under 15 %, steady error within
±5 % of the target, ripple under 20 % of the target.

**Order.** Change one gain at a time and re-run the bench after each.

1. **Feedforward first.** With KI and KD at 0, the steady error shows how
   wrong `MOTION_MAX_SPEED_MM_PER_SEC` is:
   - both wheels settling below target means the value is too high;
   - both settling above means it is too low.

   Fix it there, because every other gain works around it.
2. **KP.** Raise it until the rise meets the target, then back off if the
   overshoot passes 15 %.
3. **KI.** Raise it until the steady error is inside ±5 %. It also sets
   how fast the car pushes over a hump (section 3).
4. **KD.** Only if overshoot stays high. Encoder noise is amplified by KD,
   so it stays small.

Record every change in section 3, with the bench rows before and after it.

## 3. Tuning history

| Date | Seen on the car | Change |
|---|---|---|
| 2026-09-25 | Closed loop counts 3079 and 5260 for a short drive; hand turn gives 639 per revolution | `ENCODER_SLOTS_PER_REV` 20 → 639 |
| 2026-09-25 | Right wheel pulsed and surged; its speed read 304–525 at a 400 target, with single samples at −468 | Phase B debounce, `ENCODER_DIRECTION_PULSES` 4. Surges gone |
| 2026-09-25 | Motion bench tracks left 400 ±10, both wheels about 200 at a 200 target | Gains kept |
| 2026-09-25 | 500 mm floor drive matched the tape | `WHEEL_CIRCUMFERENCE_MM` 204 → 188 |
| 2026-09-25 | Turns overshot and swept past the line too fast for the sensors | Turns use their own 100 mm/s |
| 2026-09-25 | Jumping to 600 mm/s on a climb made the car lunge | Closed loop keeps the level speed on a climb; KI adds power gradually |
| 2026-09-25 | Found in review: a stopped wheel faulted after 300 ms, before the PID could reach full duty on a hump | Stall fault moved to its own `MOTION_STALL_FAULT_MSEC` 2000 |
| 2026-09-25 | Nothing held heading on distance moves | Straight-line correction added |
| 2026-09-25 | Found in review: `tk_dly_tsk(10)` ran every loop every 20 ms while the PID and stall timers counted 10 | Loops woken every 10 ms kernel tick; `MOTION_MAX_SPEED_MM_PER_SEC` 1600 → 800, which keeps open-loop distances right |
| 2026-09-25 | The gains tuned on the car assumed 20 ms ticks | Carried to 10 ms unchanged in real time: KI 50 → 25, KD 10 → 20, integral limit 20000 → 40000, KP stays 500 |

## 4. Step response results

Paste part 1 of the bench output below. Firmware under test: gains as in
section 1.

| Target mm/s | Wheel | Rise 10-90% ms | Overshoot % | Steady error mm/s | Ripple mm/s |
|---|---|---|---|---|---|
| 100 | left | | | | |
| 100 | right | | | | |
| 200 | left | | | | |
| 200 | right | | | | |
| 300 | left | | | | |
| 300 | right | | | | |

**Loop period** (printed after the table): ___ ms. It must read 10; 20
means the tick wake-up in `common/car_time.c` is not working.

**Reading:** ___ (does each row meet the section 2 targets? Which gain
changes, and what did the next run show?)

## 5. Motion accuracy evaluation

Part 2 of the bench, on the floor. Put a strip of tape on the floor as
the start line and heading reference. After each run, measure before the
car is moved:

- **Drives:** distance along the tape from the start mark to the same
  point on the car.
- **Turns:** the angle between the tape and the car's centre line.

The encoder columns are what the car believes. The tape column is the
truth. The gap between them is the accuracy being evaluated.

**Distance, asked 500 mm at 200 mm/s**

| Run | Asked mm | Encoders mm | Heading deg, + right | Time ms | Tape mm |
|---|---|---|---|---|---|
| drive 1 | 500 | | | | |
| drive 2 | 500 | | | | |
| drive 3 | 500 | | | | |

**Turns, asked 90° at 100 mm/s**

| Run | Asked deg | Encoders deg | Time ms | Protractor deg |
|---|---|---|---|---|
| left 1 | 90 | | | |
| left 2 | 90 | | | |
| left 3 | 90 | | | |
| right 1 | 90 | | | |
| right 2 | 90 | | | |
| right 3 | 90 | | | |

**Summary** (from the tape columns):

| Measure | Result | How |
|---|---|---|
| Distance error | ___ % | mean of (tape − 500) / 500 |
| Distance repeatability | ± ___ mm | half the spread of the tape column |
| Sideways drift over 500 mm | ___ mm | measured off the tape line |
| Turn error, left / right | ___ / ___ ° | mean of (protractor − 90) |
| Turn repeatability | ± ___ ° | half the spread |

**Corrections these numbers drive:**

- **Distance error** means `WHEEL_CIRCUMFERENCE_MM` is off. Scale it by
  tape ÷ encoders.
- **Turn error** means `WHEEL_BASE_MM` is off. Scale it by encoder
  degrees ÷ protractor degrees.
- **Encoder degrees above 90** mean the turn coasted past its target
  after the motors were cut. Lower `MOTION_TURN_SPEED_MM_PER_SEC`.
- **Drift that grows with distance** means `MOTION_STRAIGHT_KP` is too
  low. If the car wags instead, it is too high.

## 6. Known limitations

- **The PID gains are converted, not measured.** They reproduce the
  tuning done on the car at 20 ms, but that tuning was itself by eye, with
  no step response recorded. Section 4 is the first measurement.
- **The feedforward doubled with `MOTION_MAX_SPEED_MM_PER_SEC` at 800.**
  If 800 is right the PID has less to do; if not, the steady error in
  section 4 shows which way it is off.
- **`MOTION_MAX_SPEED_MM_PER_SEC` and `MOTOR_MIN_DUTY` are unmeasured.**
  The feedforward is wrong by an unknown factor and the PID covers for
  it. At 800 and 150 ‰ the duty floor holds the car at no less than
  about 120 mm/s, so targets below that are not reached.
- **The right encoder is noisy.** The debounce hides the direction flips,
  but the speed ripple remains and shows up in the ripple column.
- **Straight-line correction only acts on distance moves.** Line
  following steers from the line sensors instead.
