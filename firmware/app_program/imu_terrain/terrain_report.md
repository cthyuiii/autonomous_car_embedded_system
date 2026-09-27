# Terrain analysis report

Owner: Buddy 4, IMU based motion and terrain monitoring. This is the
deliverable the write-up asks for.

**Status:** sections 1, 2, 3 and 6 are complete. Section 4 fills in from
`./flash.sh imu_terrain` runs, the car's `car/terrain` messages, a
protractor and a ruler; section 5 is written from those results.

**Live status.** The car publishes its terrain status on `car/terrain`
every 2 seconds: pitch, whether it is trusted, the vector length,
roughness, STABLE or ROUGH, the motion event, the hump count, and the last
and highest hump heights. The keys are listed in `comms/README.md`. Record
a run with:

    mosquitto_sub -h <broker ip> -t car/terrain -v | tee terrain_log.txt

## 1. What the sensor can and cannot measure

The LSM303DLHC is an accelerometer and a magnetometer. It has **no
gyroscope**, and that shapes everything below.

| Setting | Value | Why |
|---|---|---|
| Accelerometer range | ±8 g, 4 mg per count | At ±2 g a bumper knock clips at 2000 mg and a collision can never cross the threshold |
| Sample rate | 100 Hz, one kernel tick (`IMU_SAMPLE_PERIOD_MSEC` 10) | Woken by the kernel tick, see `common/car_time.h` |
| Low pass filter | 1/32 per sample (`IMU_FILTER_SHIFT` 5) | Time constant about 320 ms |
| Forward axis | Module Y (`IMU_FORWARD_AXIS` 1) | The module is mounted turned 90° |
| One g | Measured at start-up | This module reads 1 g as about 794 mg |

An accelerometer measures the direction of apparent gravity. It cannot tell
a slope from a push: accelerating, braking and vibration move the vector
exactly the way a tilt does. So pitch can only be believed while the car is
not being pushed.

## 2. How each quantity is worked out

**Calibration.** `imu_calibrate()` averages 32 samples with the car still
and level. That sample gives the level pitch reference, the forward axis
reference for the acceleration event, and the module's own one g, which
every later test is measured against.

**Pitch.** It is computed as `atan2(forward, √(side² + up²))`, minus the
level reference.
- It is **frozen** whenever the filtered vector is more than
  `IMU_PITCH_TRUST_BAND_MILLI_G` (300 mg) away from one g.
- While a hump is open the band widens to `IMU_HUMP_TRUST_BAND_MILLI_G`
  (450 mg). On a hump the slope keeps changing and a frozen pitch loses
  that change, while the extra vibration let through averages out over
  the height integral.
- `imu_is_pitch_trusted()` and the `ok` bench column show whether it is
  frozen.

**Hump detection and height.**
- Pitch above `IMU_HUMP_PITCH_THRESHOLD_DEG` (5°) opens a hump.
- While it is open, height grows by `sin(pitch) × distance travelled`. The
  distance comes from the encoders, fed in every update by
  `imu_feed_odometry()`. For small angles sin(θ) ≈ θ in radians, which is
  within 2 % up to 20°.
- Only trusted samples count. Each adds the distance travelled since the
  previous trusted sample, at the mean of the two samples' pitches. A
  stretch where pitch was frozen, often the jolt at the foot of a hump, is
  therefore bridged by a straight line from the pitch before it to the
  pitch after it, instead of being counted at the frozen value.
- The hump closes when pitch falls back below the threshold. It is counted
  and kept as the last hump, and as the run's peak if it is the highest.
  A hump under 1 mm is not counted.
- The descent stays part of the same hump until pitch rises back above
  −5°, so the far side is never counted as a second hump.
- The run's highest hump goes out in telemetry as `hump`, and the count,
  last and highest in `car/terrain`.

The height is not measured directly. Integrating vertical acceleration
twice drifts far more than a hump is tall in the second or two it takes to
cross one; integrating slope over distance does not.

**Terrain roughness.** This is the vector's distance from one g, smoothed
at 1/32 per sample (`IMU_ROUGHNESS_SHIFT` 5). On smooth ground the only force is gravity, so it sits
near zero. Every bump, rattle and wheel slip adds to it.
- Above `IMU_TERRAIN_ROUGH_MILLI_G` (120 mg) the terrain counts as rough.
- The same number drives the pitch trust gate, read the other way round.

**Motion events.** Candidates are checked in this order, and the first
match wins:

| Event | Condition |
|---|---|
| Sudden impact | Collision latched (below) |
| Climbing hump | Pitch above +5° |
| Descending hump | Pitch below −5° |
| Turning | Turn rate above `IMU_TURN_EVENT_DPS` (30) |
| Accelerating | Forward axis more than `IMU_ACCEL_EVENT_MILLI_G` (250) from its level value |
| Stationary | None of the above |

A new event must outlast `IMU_EVENT_HOLD_SAMPLES` (20) samples, 200 ms,
of the current one before it replaces it. An impact replaces anything at once.

**Collision.** The jolt is how far one raw sample sits from the filtered
acceleration vector, in any direction. A single sample with a jolt over
`IMU_COLLISION_THRESHOLD_MILLI_G` (600 mg) latches a collision. It uses
the raw sample, not the filtered one, because an impact lasts a few
milliseconds. It measures the difference as a vector, not the change in
the vector's length, because a bumper knock is sideways to gravity: an
800 mg knock from the side lengthens the vector by only about 280 mg, so
a length test missed firm hits.

**Turn rate.** This is the wheel speed difference divided by
`WHEEL_BASE_MM`. The magnetometer is not used for it: the motors'
magnets pull the heading.

**What the car does with it.**
- **Climbing:** the car keeps its speed and the speed loop adds power.
- **Descending or rough:** the car slows to `CAR_DESCEND_SPEED_MM_PER_SEC`.
- **Collision:** the car stops, backs off 150 mm, and scans.

## 3. Calibration and tuning history

| Date | Seen on the car | Change |
|---|---|---|
| 2026-09-18 | Pitch wandered several degrees on a level car with a motor running | Pitch frozen outside the trust band; `IMU_FILTER_SHIFT` 2 → 4 |
| Before 2026-09-25 | Mast mount shakes | Trust band 150 → 300 mg |
| 2026-09-25 | The IMU bench spun the left motor | The kernel brings I2C up on GP8/GP9, the left motor's pins; init now parks them |
| 2026-09-25 | One g read as about 794 mg, so the car read ROUGH at rest | One g measured at calibration; roughness, trust and collision measured against it |
| 2026-09-25 | Tilting sideways read as a climb | The module sits turned 90°; `IMU_FORWARD_AXIS` added, set to Y |
| 2026-09-25 | Found in review: the IMU task ran every 20 ms, not 10, because a kernel delay adds a tick | Loops now woken every tick; `IMU_FILTER_SHIFT`, `IMU_ROUGHNESS_SHIFT` and `IMU_EVENT_HOLD_SAMPLES` doubled in samples so their time in ms is unchanged |
| 2026-09-25 | Found in review: pitch frozen at the foot of a hump lost that part of the climb | Frozen stretches bridged between trusted samples; hump count and last hump kept |
| 2026-09-25 | Found in review: a slope that changed while pitch was frozen was still miscounted | Trust band 450 mg while a hump is open, 300 on the flat |
| 2026-09-27 | Firm hits on the bumper sometimes did not register | Collision judged on the jolt, the vector difference from the filtered vector, instead of the vector's length over one g, which barely moves for a knock from the side. The bench column `raw` became `jolt` |

## 4. Measurements

Build and flash `./flash.sh imu_terrain` and watch the console. Parts a, b
and f need the car still, so leave `BENCH_DRIVE` at 0. Parts c, d and e
need it moving: set `BENCH_DRIVE` to 1 in `bench_imu_terrain.c`, and put
it back to 0 afterwards. Reset the board before every run.

**a. At rest, level, five boots**

| Boot | `mag` (the module's one g) | `ok` | `rough` | Pitch |
|---|---|---|---|---|
| 1 | 793 | 1 | 31 | 1 |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
| 5 | | | | |

Pass: `ok` 1, `rough` well below 120, and pitch 0 ± 1°. Boot 1
(2026-09-26) passes: the old `raw` column, the vector's length, peaked at
817 to 833 against 793, about 40 mg of noise at rest.

**b. Static tilt.** Put the car on a ramp or a stack of books and measure
the angle with a protractor or phone level.

| Tilt | Protractor ° | Bench pitch ° | Error ° |
|---|---|---|---|
| Nose up | 5 | | |
| Nose up | 10 | | |
| Nose up | 15 | | |
| Nose up | 20 | | |
| Nose down | 10 | | |
| Sideways, left side up | 10 | | |
| Sideways, right side up | 10 | | |

Pass: within 2°. Nose up must read positive; if it reads negative, flip
`IMU_PITCH_SIGN`. The sideways rows must read about 0; if they read the
tilt, `IMU_FORWARD_AXIS` is wrong.

**c. Driving on the flat.** Run each surface for about 20 s.

| Surface | `mag` range | Share of lines with `ok` 1 | `rough`, typical | STABLE or ROUGH |
|---|---|---|---|---|
| Course floor | | | | |
| Tape joins or seams | | | | |
| Other: | | | | |

This shows whether the trust band and the roughness threshold separate the
course floor from real rough ground. If the course floor reads ROUGH,
raise `IMU_TERRAIN_ROUGH_MILLI_G` above its typical value.

**d. Hump height.** Measure each hump's height at its peak with a ruler,
then drive over it three times. The number to record is the bench's
`hump over: N mm` line, or `last_mm` from `car/terrain` on a full run.

| Hump | Ruler mm | Run 1 mm | Run 2 mm | Run 3 mm | Mean error mm | `ok` 1 while climbing? |
|---|---|---|---|---|---|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |

Target: within 5 mm. The last column matters most.
- **`ok` 0 on the climb:** pitch froze even with the wider hump band. The
  frozen stretch is bridged by a straight line, which is close on a
  steady ramp but not on a curving one. If the error is large, widen
  `IMU_HUMP_TRUST_BAND_MILLI_G` (for pitch before the hump opens,
  `IMU_PITCH_TRUST_BAND_MILLI_G`), or climb slower.
- **Heights scattered run to run with `ok` 1 throughout:** vibration is
  reaching pitch through the wider band. Narrow
  `IMU_HUMP_TRUST_BAND_MILLI_G` towards 300.
- **Nothing counted on a low hump:** its slope stays under 5°. Lower
  `IMU_HUMP_PITCH_THRESHOLD_DEG` towards 3.
- **Every run off by the same factor:** the distance is wrong. Check the
  motion report's distance error first.

**e. Motion events**

| Action | Expected event | Seen | Delay, bench lines |
|---|---|---|---|
| Standing still | STILL | | |
| Pull away hard | ACCEL | | |
| Turn on the spot | TURN | | |
| Up the hump | CLIMB | | |
| Down the hump | DESCEND | | |
| Tap the bumper | IMPACT | | |

**f. Collision threshold.** Tap the bumper at a few strengths and read the
`jolt` column, which is the peak since the last line.

| Tap | `jolt` peak mg | `hit` 1? |
|---|---|---|
| Light touch | | |
| Firm knock | | |
| Push into a wall at driving speed | | |
| Driving over the hump, no contact | | |

The threshold (600 mg of jolt) must sit above the hump row and below the
knock rows. If the hump row triggers a hit, the car will stop on every
hump.

## 5. Results and conclusions

Written from section 4 once it is filled in:

- How accurate the peak hump height is, and whether it meets the 5 mm target.
- Whether pitch stays trusted on the climb at the car's normal speed.
- Which surfaces read STABLE, and the roughness threshold chosen.
- The collision threshold chosen, with the margin either side.
- Every knob changed, with the section 4 row that justified it.

## 6. Known limitations

- **No gyroscope.** Pitch is frozen whenever the car accelerates or
  shakes harder than the trust band allows. On a hump the band is wider,
  and any stretch still frozen is bridged by a straight line, so only
  shaking past 450 mg on a curving slope still costs accuracy. The wider
  band lets more vibration into pitch in exchange. Section 4d shows
  where the balance lies on this car.
- **Height depends on the encoders.** A distance error in the motion
  calibration scales every hump height by the same factor.
- **Humps under 5° of slope** are never opened, and a hump under 1 mm is
  not counted.
- **Heading is unreliable** near the motors, which is why turn rate comes
  from the encoders.
