# scanning

Owner: Buddy 5, ultrasonic scanning and obstacle profiling.

Owns the scan servo, HC-SR04 ranging, coarse and fine scans, obstacle
profiling, avoidance planning and the line recovery search pattern. API is
`include/scan.h`. Knobs are `SONAR_*`, `SERVO_*` and `SCAN_*` in
`common/car_config.h`. Read the notes in the header before wiring.

Implemented. The module runs at 3.3 V from the Grove port, so the echo
needs no divider and the maximum range is whatever the bench measures.

**If nothing ever echoes, suspect the supply before the code.** A classic
HC-SR04 is a 5 V part. Many fire at 3.3 V with reduced range, but a fair
number do not fire at all, and no firmware change fixes that. The bench
says so when it counts zero echoes. Two ways out: power the module from
5 V off a spare servo header, which sits on the motor rail, and divide its
echo down with roughly 1 k and 2 k before the pin; or swap it for an
HC-SR04P or RCWL-1601, which are pin compatible and run natively at 3.3 V.
Check the marking on the back of the board before buying anything.

**If the horn never moves, reach for `make BENCH=servo` first.** That
bench lives in this folder as `bench_servo.c` and drives the pin straight
through car_hw, calling nothing in this module, so it separates a firmware
fault from a hardware one in one flash. It holds a centre pulse, then
sweeps the safe 1000 to 2000 microsecond range, then the wider 500 to 2500.

**If the horn never moves there either, suspect power.** The servo header
runs from the motor rail, so the battery has to be connected and the board
switched on; USB alone moves nothing. Pulse width defaults to the 1000 to
2000 microseconds every servo accepts; widen towards 500 and 2500 only
while the horn still moves freely and does not buzz against a stop.

## The first movement, and why it is the big one

A servo reports nothing about where it is. The firmware cannot know the
horn's position until it sends a pulse, and the horn then jumps to whatever
that pulse says. So the first pulse after power on is the one large
movement the car ever makes, and how large is the gap between where
the horn was left and `SERVO_CENTRE_PULSE_USEC`.

That is the whole explanation for a horn that swings a long way at startup
and then only creeps afterwards. Nothing is sweeping 90 degrees; it is
travelling once to the commanded centre.

The fix is to measure the pulse width that points the horn straight ahead
**on this car, as mounted**, and put it in `SERVO_CENTRE_PULSE_USEC`. Set
`BENCH_FIND_CENTRE` to 1 at the top of `bench_servo.c`, run `BENCH=servo`
with the horn removed or the car held clear, and it walks the pulse from
1200 to 1800 microseconds in 10 microsecond steps, printing each. Note the
width where the horn points straight ahead, put it in the config, set
`BENCH_FIND_CENTRE` back to 0, and the startup jump disappears.

Angles are measured from that centre rather than mapped across a 0 to 180
range, so once it is right, no commanded angle can move the horn further
than the sweep allows. `SERVO_USEC_PER_DEG` sets the scale, 11 microseconds
per degree for a servo that covers 90 degrees in 1000 microseconds.

Measured on this car: the horn points straight ahead at **1744
microseconds**. Re-measure if the horn is ever taken off its splines and
refitted, because the splines index only every few degrees and it will not
go back in the same place.

One thing to know before widening further. A 1744 microsecond centre sits
off to one side of the pulse band, so the upper end clamps first.
`SERVO_PULSE_MAX_USEC` is raised to 2100, which leaves 356 microseconds
above the centre: 32 degrees at 11 microseconds per degree. Wider than that
means raising it again, which most servos accept, or refitting the horn a
spline or two over so the centre lands nearer 1500.

## Sweep width, and what it costs

`SCAN_HALF_SWEEP_DEG` in `car_config.h` is the one number that sets how far
the horn travels. Everything else follows it: the band `scan_measure()`
clamps to, the coarse angle set, the fine scan step and span, and the servo
bench. Change it alone and nothing needs keeping in step by hand.

It is set to 25, so the horn swings 25 degrees either side of straight
ahead, a 50 degree sweep.

| | Angle | Pulse |
|---|---|---|
| Right of centre | 65 | 1469 us |
| Straight ahead | 90 | 1744 us |
| Left of centre | 115 | 2019 us |

A fine scan steps 12 degrees, half the sweep, across 25 degrees either
side of the coarse bearing, so it takes five readings within 65 to 115.
That is five rangings at the datasheet's 60 millisecond minimum, so 300
milliseconds standing still, plus `SERVO_SETTLE_MSEC` per move.

**Neighbouring readings.** The sonar's beam is about 15 degrees across.
The three coarse angles, 25 degrees apart, clear one beam width, so the
side readings are independent of the centre one. The 12 degree fine steps
overlap a little, so bearing and width are indicative rather than
measured.

**Choosing a side.** Each side's clearance is the shortest reading on
that side of the obstacle, measured along the ray. `scan_plan_avoidance()`
goes round on the side whose clearance reaches `SCAN_CLEARANCE_MIN_MM`
(250 mm, the car plus 50 mm each side), the larger one if both do. If
neither does, it picks `CAR_AVOID_REVERSE`: the car backs off
`SCAN_REVERSE_MM` and drives the box detour on one side, pinging ahead
before every driving leg. A blocked leg is undone and the other side tried,
up to `CAR_MAX_DETOUR_ATTEMPTS`, after which the car halts.

A ranging at the angle the servo already holds skips the settle, which is
what makes the forward ping every 60 ms while following affordable. The
recovery search is a sequencer: the controller executes each turn and
drive step through the motion API and reports what the line sensors saw.

Scan budget: 60 ms minimum per ranging, from the datasheet, plus settle per
move. Five coarse angles is 5 x 60 = 300 ms standing still before travel.
Fine scan over 60 degrees at 15 degree steps is 5 more. Plan for it.

Done means `test_scanning.c` passes on the host, and `bench_scanning` puts
a box at 300 mm within 10 mm and 15 degrees of its true bearing.

| Calibration              | Measured | How                                 |
|--------------------------|----------|-------------------------------------|
| Echoes at 3.3 V          |          | phase 2 of the bench, out of ten    |
| Max range at 3.3 V       |          | walk a target away until it fails   |
| SERVO_CENTRE_PULSE_USEC  | 1744     | centre finder, horn straight ahead  |
| Usable travel, degrees   |          | widen SCAN_HALF_SWEEP_DEG while free |
| SERVO_SETTLE_MSEC        |          | reduce until readings smear         |
| Range error at 100, 300 mm|         | tape measure vs printed             |
| SERVO_PULSE_MIN and MAX  |          | find the mechanical end stops       |
| Servo direction          |          | angle above 90 must look left       |

## Scan angles

The band is `SCAN_HALF_SWEEP_DEG` either side of `SCAN_CENTRE_ANGLE_DEG`,
25 and 90 today, so the three angles are 65, 90 and 115 degrees. At this
car's measured centre of 1744 us and 11 us per degree those are 1469, 1744
and 2019 us. Note 2019 is above the usual 2000 ceiling, which is why
`SERVO_PULSE_MAX_USEC` is 2100; put it back to 2000 if the horn binds and
115 will clamp to about 113 instead of forcing the mount.

25 degrees apart matters: the HC-SR04's beam is about 15 degrees wide, so
this is the first setting at which the left and right readings look
somewhere the centre reading does not. That is what makes
`clearance_left_mm` and `clearance_right_mm` mean anything, and therefore
what lets `scan_plan_avoidance()` pick a side instead of always reversing.

`SCAN_FINE_STEP_DEG` is half the half sweep, 12 degrees, so a fine scan
puts one reading between each pair of coarse angles. Finer steps cost
60 ms each to re-measure the same beam, and that time is spent standing
still in front of an obstacle.

Test it with `./flash.sh detour`, which drives straight and detours with no
line sensors involved.
