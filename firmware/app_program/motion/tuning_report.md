# Motion control: PID tuning report and motion accuracy evaluation

Owner: Buddy 2, motion control.

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
  pulses per tick. At 639 pulses per wheel turn and 205 mm per turn, one
  pulse is 0.32 mm.
- **Direction** comes from phase B, read at each phase A edge. A reversal
  only counts after 4 pulses in a row agree. A wheel turning against its
  command is a negative speed, so the PID pushes harder.
- **The integral** is clamped at ±40000 (anti-windup). It is reset only
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
| `MOTION_MAX_SPEED_MM_PER_SEC` | 800 | Half the 1600 from a timed turn at the old 20 ms loop |
| `MOTOR_MIN_DUTY` | 70 ‰ | `./flash.sh duty` on the floor: both wheels start at 90 and keep turning down to 70. 60 still creeps at 10 to 14 mm/s |
| `ENCODER_SLOTS_PER_REV` | 639 | Hand turned: 638, 639, 640 |
| `WHEEL_CIRCUMFERENCE_MM` | 205 | 500 mm drives against a tape, section 5 |
| `WHEEL_LEFT_PERMILLE`, `WHEEL_RIGHT_PERMILLE` | 993, 1007 | Sideways drift at the end of 500 mm drives, section 3 |
| `WHEEL_BASE_MM` | 110 | Tyre centres, and 89° turns against a protractor |
| `ENCODER_DIRECTION_PULSES` | 4 | About 1.3 mm of travel |
| `MOTION_TURN_SPEED_MM_PER_SEC` | 60 | Section 3: overshoot 0.8 to 2.4° against 2.4 to 3.7° at 100 |
| `MOTION_STRAIGHT_KP` | 4 mm/s per mm | 500 mm drives end straight, section 5 |

## 2. Tuning method

**Step test.** The bench's part 1 lifts the car and steps both wheels
from rest to 100, 200 and 300 mm/s. Each step runs 3 s. Per wheel it
reports:

- **Rise:** time from 10 % to 90 % of the target.
- **Overshoot:** peak above the target, as a percentage.
- **Steady error:** mean speed minus the target over the last second.
- **Ripple:** highest minus lowest speed over the same second.

**Targets.** Rise under 300 ms, overshoot under 15 %, steady error within
±5 % of the target, ripple under 20 % of the target.

**Order.** Change one gain at a time and re-run the bench after each.

1. **Feedforward first.** With KI and KD at 0, the steady error shows how
   wrong `MOTION_MAX_SPEED_MM_PER_SEC` is:
   - both wheels settling below target means the value is too high.
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
| 2026-09-25 | Closed loop counts 3079 and 5260 for a short drive. Hand turn gives 639 per revolution | `ENCODER_SLOTS_PER_REV` 20 → 639 |
| 2026-09-25 | Right wheel pulsed and surged. Its speed read 304–525 at a 400 target, with single samples at −468 | Phase B debounce, `ENCODER_DIRECTION_PULSES` 4. Surges gone |
| 2026-09-25 | Motion bench tracks left 400 ±10, both wheels about 200 at a 200 target | Gains kept |
| 2026-09-25 | 500 mm floor drive matched the tape | `WHEEL_CIRCUMFERENCE_MM` 204 → 188 |
| 2026-09-25 | Turns overshot and swept past the line too fast for the sensors | Turns use their own 100 mm/s |
| 2026-09-25 | Jumping to 600 mm/s on a climb made the car lunge | Closed loop keeps the level speed on a climb. KI adds power gradually |
| 2026-09-25 | Found in review: a stopped wheel faulted after 300 ms, before the PID could reach full duty on a hump | Stall fault moved to its own `MOTION_STALL_FAULT_MSEC` 2000 |
| 2026-09-25 | Nothing held heading on distance moves | Straight-line correction added |
| 2026-09-25 | Found in review: `tk_dly_tsk(10)` ran every loop every 20 ms while the PID and stall timers counted 10 | Loops woken every 10 ms kernel tick. `MOTION_MAX_SPEED_MM_PER_SEC` 1600 → 800, which keeps open-loop distances right |
| 2026-09-25 | The gains tuned on the car assumed 20 ms ticks | Carried to 10 ms unchanged in real time: KI 50 → 25, KD 10 → 20, integral limit 20000 → 40000, KP stays 500 |
| 2026-09-26 | First tuning run. Left wheel at 300 mm/s: overshoot 433 %, ripple 1409 mm/s, right normal. Drives drift left on the floor while the encoder heading reads under 1° | Encoder edges closer than `ENCODER_MIN_PULSE_USEC` (150 µs) to the last pulse are thrown away and counted. The bench prints the counts |
| 2026-09-26 | Second run: left 54 glitches thrown away, right 1, yet the left still spiked (1196 % at 100 mm/s, ripple 1021 at 300). Right wheel clean: rise 110 to 200 ms, overshoot 3 to 4 %, steady error ±1 | Also thrown away: an edge whose pin is already low when the interrupt reads it |
| 2026-09-26 | Duty bench: start 90 ‰, keep turning to 70 ‰, 35 mm/s at 100 ‰ | `MOTOR_MIN_DUTY` 150 → 70 |
| 2026-09-26 | Third run, with the pin level check and the floor at 70: left matches right at 100 and 200 mm/s (overshoot 4 to 5 %, ripple 8 to 14). Left still noisy at 300 only (351 %, ripple 1097). 180 glitches in the step test, none during the 200 mm/s drives. Turns 92.4 to 93.7° | Kept. The car never asks for 300 |
| 2026-09-26 | Fourth run, turn speed 60: turns 90.8 to 92.4° in about 1.75 s | `MOTION_TURN_SPEED_MM_PER_SEC` 100 → 60 |
| 2026-10-14 | Fifth run, the first measured by hand. Drives read 508 to 510 mm on the encoders and about 550 mm on the tape, ending about 10 mm left. The 2026-09-25 cut to 188 had matched the tape while the left encoder still added spurious pulses, which the 2026-09-26 filter now throws away | `WHEEL_CIRCUMFERENCE_MM` 188 → 203 (188 × 550 / 509) |
| 2026-10-14 | Same run: turns read 90.7 to 92.4° on the encoders and 105 to 110° on a protractor, and the car pivoted off centre | `WHEEL_BASE_MM` 110 → 101 (91.4 × 203 / 188 = 98.7, × 110 / 107.5). Junction turns now stop when the leading line sensor reaches the new branch, so their angle no longer rests on the encoders |
| 2026-10-14 | Sixth run, with 203 and 101: drives 520 mm on the tape against 508 to 509 on the encoders, ending 10 to 25 mm left and pointing about 5° left. Turns about 82°. Worked in pulses, so each run's calibration cancels out: the circumference comes to 203 and 207.5, the turning base to 102 and 114 | `WHEEL_CIRCUMFERENCE_MM` 203 → 205 and `WHEEL_BASE_MM` 101 → 108, between the two runs |
| 2026-10-14 | Same run: the encoder heading read 0.0 to +0.3° while the car turned about 5° left over 500 mm, so equal pulse counts are not equal ground: the left tyre covers about 1.4 % less per turn | Per-wheel shares `WHEEL_LEFT_PERMILLE` 993 and `WHEEL_RIGHT_PERMILLE` 1007, used by both speed loops and the straight-line correction |
| 2026-10-14 | Seventh run, with 205, 108 and the shares: drives 500 to 515 mm on the tape against 509 to 510 on the encoders, small sideways deviation, car facing along the tape. Turns about 89° | Distance and straightness kept. `WHEEL_BASE_MM` 108 → 110 (90.9 / 89), which is the tyre centres measured with a ruler |

## 4. Step response results

Firmware under test: gains as in section 1.

Fifth run, 2026-10-14: `MOTOR_MIN_DUTY` 70, turn speed 60, circumference
still 188 (the run that set it to 203).

| Target mm/s | Wheel | Rise 10-90% ms | Overshoot % | Steady error mm/s | Ripple mm/s |
|---|---|---|---|---|---|
| 100 | left | 70 | 5 | 0 | 8 |
| 100 | right | 70 | 4 | 0 | 7 |
| 200 | left | 70 | 4 | -1 | 13 |
| 200 | right | 60 | 4 | 0 | 14 |
| 300 | left | 80 | 433 | 9 | 1348 |
| 300 | right | 70 | 4 | 0 | 21 |

**Loop period** (printed after the table): 9 ms, the tick wake-up
working (20 would mean it is not).

**Encoder noise:** left 203 glitches thrown away in the step test, right
3. None reversed. The fourth run (2026-09-26) gave the same picture: left
166, right 0, left overshoot 178 % at 300.

**Reading:** every row meets the section 2 targets (rise under 300 ms,
overshoot under 15 %, steady error within 5 %, ripple under 20 %) except
left at 300 mm/s, where the overshoot and ripple are the left encoder's
noise rather than the loop: the right wheel on the same gains is clean,
and the left is clean at 100 and 200. No gain change was needed.

## 5. Motion accuracy evaluation

Part 2 of the bench, on the floor. Put a strip of tape on the floor as
the start line and heading reference. After each run, measure before the
car is moved:

- **Drives:** distance along the tape from the start mark to the same
  point on the car, and how far that point ended up to the side of the
  tape line, + right.
- **Turns:** the angle between the tape and the car's centre line.

The bench gives 15 s between runs for this, and prints how much encoder
noise it has thrown away after the step test and after the drives.

The encoder columns are what the car believes. The tape column is the
truth. The gap between them is the accuracy being evaluated.

**Distance, asked 500 mm at 200 mm/s** (fifth run, 2026-10-14, with the
circumference at 188). The three drives were measured as one figure, so
each tape cell holds that average. Single runs were not recorded.

| Run | Asked mm | Encoders mm | Heading deg, + right | Time ms | Tape mm | Sideways mm, + right |
|---|---|---|---|---|---|---|
| drive 1 | 500 | 509 | +0.3 | 2620 | ≈ 550 | ≈ −10 |
| drive 2 | 500 | 509 | +0.1 | 2620 | ≈ 550 | ≈ −10 |
| drive 3 | 500 | 510 | +0.6 | 2620 | ≈ 550 | ≈ −10 |

**Turns, asked 90° at 60 mm/s** (same run). Measured as a range for all
six.

| Run | Asked deg | Encoders deg | Time ms | Protractor deg |
|---|---|---|---|---|
| left 1 | 90 | 92.4 | 1980 | 105 to 110 |
| left 2 | 90 | 91.0 | 2030 | 105 to 110 |
| left 3 | 90 | 91.4 | 1980 | 105 to 110 |
| right 1 | 90 | 91.9 | 1950 | 105 to 110 |
| right 2 | 90 | 90.7 | 1960 | 105 to 110 |
| right 3 | 90 | 91.0 | 1980 | 105 to 110 |

**Summary** (fifth run, before the corrections it led to):

| Measure | Result | How |
|---|---|---|
| Distance error | about +10 % | (550 − 500) / 500 |
| Distance repeatability | not recorded | runs measured as one figure |
| Sideways drift over 500 mm | about 10 mm left | measured off the tape line |
| Turn error, left / right | about +15 to +20 ° both ways | protractor − 90 |
| Turn repeatability | not recorded | runs measured as one range |

**Sixth run, 2026-10-14**, with `WHEEL_CIRCUMFERENCE_MM` 203 and
`WHEEL_BASE_MM` 101. Again measured as one figure for the drives and one
for the turns.

| Run | Asked mm | Encoders mm | Heading deg, + right | Time ms | Tape mm | Sideways mm, + right |
|---|---|---|---|---|---|---|
| drive 1 | 500 | 509 | +0.3 | 2590 | ≈ 520 | −10 to −25 |
| drive 2 | 500 | 508 | +0.0 | 2590 | ≈ 520 | −10 to −25 |
| drive 3 | 500 | 509 | +0.1 | 2590 | ≈ 520 | −10 to −25 |

The car also ended pointing about 5° left of the tape.

| Run | Asked deg | Encoders deg | Time ms | Protractor deg |
|---|---|---|---|---|
| left 1 | 90 | 91.5 | 1830 | ≈ 82 |
| left 2 | 90 | 91.5 | 1870 | ≈ 82 |
| left 3 | 90 | 90.8 | 1800 | ≈ 82 |
| right 1 | 90 | 91.9 | 1810 | ≈ 82 |
| right 2 | 90 | 91.1 | 1830 | ≈ 82 |
| right 3 | 90 | not in the export | | |

The two runs bracket the true values from either side (550 then 520 mm,
107.5 then 82°), which is what set 205 and 108.

**Seventh run, 2026-10-14**, with `WHEEL_CIRCUMFERENCE_MM` 205,
`WHEEL_BASE_MM` 108 and the per-wheel shares 993 / 1007.

| Run | Asked mm | Encoders mm | Heading deg, + right | Time ms | Tape mm | Sideways mm, + right |
|---|---|---|---|---|---|---|
| drive 1 | 500 | 509 | +0.1 | 2580 | 500 to 515 | small |
| drive 2 | 500 | 510 | +0.0 | 2580 | 500 to 515 | small |
| drive 3 | 500 | 509 | +0.0 | 2580 | 500 to 515 | small |

| Run | Asked deg | Encoders deg | Time ms | Protractor deg |
|---|---|---|---|---|
| left 1 | 90 | 91.4 | 1910 | ≈ 89 |
| left 2 | 90 | 90.4 | 1900 | ≈ 89 |
| left 3 | 90 | 90.7 | 1910 | ≈ 89 |
| right 1 | 90 | 91.7 | 1860 | ≈ 89 |
| right 2 | 90 | 90.4 | 1870 | ≈ 89 |
| right 3 | 90 | not in the export | | |

**Summary, seventh run:**

| Measure | Result | How |
|---|---|---|
| Distance error | about +1.5 % (500 to 515 mm) | tape against 500 |
| Distance repeatability | ± 7.5 mm | half the spread of the tape range |
| Sideways drift over 500 mm | small, car facing along the tape | by eye, down from 10 to 25 mm and 5° in the sixth run |
| Turn error, left / right | about −1 ° both ways | protractor − 90 |
| Turn repeatability | not recorded | turns measured as one figure |

The turn shortfall set `WHEEL_BASE_MM` to 110. Junction turns end on the
line sensor, so this only shapes the detour's 45° legs and how far a spin
goes.

**Corrections these numbers drive:**

- **Distance error** means `WHEEL_CIRCUMFERENCE_MM` is off. Scale it by
  tape ÷ encoders.
- **Turn error** means `WHEEL_BASE_MM` is off. Scale it by encoder
  degrees ÷ protractor degrees.
- **Encoder degrees above 90** mean the turn coasted past its target
  after the motors were cut. Lower `MOTION_TURN_SPEED_MM_PER_SEC`.
- **Sideways drift while the encoder heading reads near 0** means one
  wheel's count is not what it rolled. First check the noise line: a
  wheel with glitches or reversals counts ahead, the straight line
  correction slows it, and the car curves toward that wheel. With no
  noise, that wheel's tyre covers less ground per turn than the other.
- **Drift the encoder heading also shows** means `MOTION_STRAIGHT_KP` is
  too low. If the car wags instead, it is too high.

## 6. Known limitations

- **The gains were tuned by eye at 20 ms** and carried to 10 ms. The step
  response in section 4 meets every target except the left wheel at
  300 mm/s.
- **The feedforward is high.** The duty bench saw about 35 mm/s at 100 ‰,
  where 800 at full duty predicts 80. The PID covers the difference, and
  the steady error in section 4 is within 1 mm/s apart from the left
  wheel at 300 mm/s.
- **The left encoder is noisy above 200 mm/s.** Impossible edges are
  thrown away and counted. The car never drives that fast.
- **Straight-line correction only acts on distance moves.** Line
  following steers from the line sensors instead.
