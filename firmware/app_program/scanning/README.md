# scanning

Owner: Buddy 5, ultrasonic scanning and obstacle profiling.

Owns the scan servo, HC-SR04 ranging, coarse and fine scans, obstacle
profiling, detour planning and the line search pattern. The API is in
`include/scan.h`, the values are `SONAR_*`, `SERVO_*` and `SCAN_*` in
`common/car_config.h`.

The HC-SR04 runs at 3.3 V from the Grove port, so its echo needs no
divider.

**No echoes:** suspect the supply first. Many HC-SR04s fire at 3.3 V with
less range and some not at all, and the bench says so when it counts zero
echoes. Either power it from 5 V on a spare servo header and divide the
echo with about 1 k and 2 k, or swap in an HC-SR04P or RCWL-1601, which run
at 3.3 V.

**The horn never moves:** run `./flash.sh servo`. It drives the pin through
car_hw without this module, so it separates a firmware fault from a
hardware one. If the horn does not move there either, check the battery:
the servo header runs from the motor rail.

## Servo centre

A servo reports nothing about where it is, so the first pulse after power
on throws the horn from wherever it was left to `SERVO_CENTRE_PULSE_USEC`.
That one swing at start-up is normal.

On this car the horn points straight ahead at 1744 µs. If the horn is
refitted, set `BENCH_FIND_CENTRE` to 1 in `bench_servo.c` and run
`./flash.sh servo`. It walks the pulse from 1200 to 1800 µs in 10 µs
steps. Put the width where the horn points straight ahead in
`SERVO_CENTRE_PULSE_USEC` and set the flag back to 0.

Angles are measured from that centre at `SERVO_USEC_PER_DEG`, 11 µs per
degree. `SERVO_PULSE_MAX_USEC` is 2100 because 115 degrees needs 2019 µs.
Put it back to 2000 if the horn binds, and 115 clamps to about 113.

## Sweep

`SCAN_HALF_SWEEP_DEG` sets the travel, and everything else follows it: the
clamp in `scan_measure()`, the coarse angles, the fine step and the servo
bench.

| | Angle | Pulse |
|---|---|---|
| Right of centre | 65 | 1469 µs |
| Straight ahead | 90 | 1744 µs |
| Left of centre | 115 | 2019 µs |

The sonar's beam is about 15 degrees wide. At 25 degrees apart the side
readings look where the centre one does not, so the clearance either side
means something and `scan_plan_avoidance()` can pick a side. A fine scan
steps 12 degrees, five readings from 65 to 115, about 300 ms standing
still plus `SERVO_SETTLE_MSEC` per move. Its steps overlap, so bearing and
width are approximate.

## Going round

Each side's clearance is the shortest reading on that side.
`scan_plan_avoidance()` goes round on a side with at least
`SCAN_CLEARANCE_MIN_MM` (250 mm, the car plus 50 mm each side), the wider
if both. If neither, the car backs off `SCAN_REVERSE_MM` and probes a
side, pinging before every driving leg. A blocked leg is undone and the
other side tried, up to `CAR_MAX_DETOUR_ATTEMPTS`.

The detour plans for an obstacle at least `SCAN_DETOUR_WIDTH_MIN_MM`
(220 mm) wide, since flat faces read narrow. Beside it the car walks its
length: it steps `SCAN_DETOUR_STEP_MM` (50 mm), turns 65 degrees toward it
and pings, and steps on while the obstacle is nearer than the line. Once
clear it drives `SCAN_DETOUR_PAST_MM` (100 mm) on and turns in.

A ranging at the angle the servo already holds skips the settle, which
keeps the forward ping every 60 ms while following cheap. The line search
is a sequencer: the controller drives each turn and step through motion
and reports what the line sensors saw.

Tests: `test_scanning.c` on the host. `./flash.sh scanning` for the servo
and sonar, `./flash.sh detour` to drive round a box with no line sensors.

| Calibration             | Value               | How                                |
|-------------------------|---------------------|------------------------------------|
| SERVO_CENTRE_PULSE_USEC | 1744                | centre finder, horn straight ahead |
| SCAN_HALF_SWEEP_DEG     | 25                  | clears the sonar's 15 degree beam  |
| Servo direction         | above 90 looks left | `./flash.sh servo`                 |
| SCAN_DETOUR_STEP_MM     | 50                  | detour bench, a 220 by 195 mm box cleared |
