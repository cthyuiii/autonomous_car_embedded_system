# Autonomous Car Firmware

Autonomous line following car on a Raspberry Pi Pico W carried by a Cytron
Robo Pico, running micro T-Kernel 3.0. Five subsystems, one vehicle
controller, one Pico on the car. The other Picos are development benches,
one per person.

## Layout

`firmware/` is the kernel port, vendored from
[sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040)
under its own licences (see `firmware/LICENSE` and `firmware/LICENSES/`).
Everything of ours lives in `firmware/app_program/`. Do not edit the kernel
tree except the places listed under Kernel notes.

| Path                                | Owns                                   |
|-------------------------------------|----------------------------------------|
| `app_program/car_main.c`            | `usermain()`, the tasks, mission state |
| `app_program/common/`               | shared types, every config knob, logging, board helpers |
| `app_program/comms/`                | MQTT telemetry, heartbeat, commands    |
| `app_program/motion/`               | motors, encoders, PID, distance, turns, steering |
| `app_program/line_barcode/`         | three IR sensors, position, Code 39 barcodes |
| `app_program/imu_terrain/`          | LSM303DLHC, tilt, humps, events, collision |
| `app_program/scanning/`             | servo, HC-SR04, profiling, avoidance, recovery |
| `build_make/mtkernel_3/app_program/`| `subdir.mk`, how our code is built     |
| `flash.sh`                          | build one image and flash it, in one step |
| `lib/libnet/lwip/lwip_utk_mqtt.*`   | the MQTT phase on the kernel's radio task |

Each subsystem folder holds its public header under `include/`, the
implementation, a host test, a hardware bench and a README with its
calibration table.

## Who edits what

| Owner   | Files                                                     |
|---------|-----------------------------------------------------------|
| Buddy 1 | `app_program/comms/`, `lib/libnet/lwip/lwip_utk_mqtt.*`, plus the `COMMS_*` config block |
| Buddy 2 | `app_program/motion/`, plus `MOTOR_*`, `ENCODER_*`, `MOTION_*` |
| Buddy 3 | `app_program/line_barcode/`, plus `LINE_*`, `TRACK_*`, `BARCODE_*` |
| Buddy 4 | `app_program/imu_terrain/`, plus `IMU_*`                  |
| Buddy 5 | `app_program/scanning/`, plus `SONAR_*`, `SERVO_*`, `SCAN_*` |
| Team    | `car_main.c`, `common/`, `CAR_*`, `subdir.mk`             |
| Nobody  | Everything else under `firmware/`. That is the kernel     |

Every file you own says so in its header comment. The header changes only
by agreement; the test grows and never shrinks; the bench is yours to
extend; the README table is where your measurements go.

## Where the code stands

Every subsystem is implemented and every host test passes, including the
decode of a synthetic Code 39 symbol in both directions, a 20 degree hump
climb with its height integral, the obstacle planner on injected ranges,
and the command parser. None of it has run on the car yet. The `TODO:`
comments from the scaffold were left in place above each implementation
so the original intent can be read against what was written; strip them
before submission if the coding standard reviewer would count them.

What each module does today, and the one thing to confirm first on it:

- **motion** drives the board's H-bridge over PWM and steers with
  `motion_drive_steer()`. `MOTION_OPEN_LOOP` is 1 because the encoders are
  not wired, so moves and turns are timed from the speed setpoint. Set it
  to 0 when the encoder leads are on Grove 7 and 1. First check: the motor
  test buttons, then wheel direction.
- **line_barcode** reads three MH-Sensor-Series modules as a 3 bit mask.
  Left (Grove 5) and right (Grove 2) straddle the line and steer; the
  third (Grove 6) sits off to the right and decodes Code 39 with the
  microsecond timer. First check: `LINE_SENSOR_DARK_LEVEL`.
- **imu_terrain** runs the LSM303DLHC on Grove 3, moving the kernel's I2C
  unit off the motor pins at init. Pitch is relative to a level reference
  taken at boot. First check: `IMU_PITCH_SIGN`, nose up must read positive.
- **scanning** runs the HC-SR04 at 3.3 V from Grove 4, with no echo
  divider, and the servo on S1. First check: which way angles above 90
  look; the header assumes left.
- **comms** publishes JSON over MQTT through a new phase on the kernel's
  radio task, only in the `WIFI_MQTT=1` build. Without it the controller
  logs the radio as absent and drives anyway.
- **car_main** calibrates before starting tasks, then follows the line with
  a proportional plus derivative law, slows through the barcode gap, turns
  at the next junction after a command, drives a box detour around an
  obstacle, and searches back to the line afterwards.

The obstacle ping is shared by every state, not just line following. One
ranging per `CAR_SONAR_CHECK_PERIOD_MSEC` is taken above the state switch
and every state reads the same answer, so a turn, a detour leg and the
recovery search all give way to something in the path. The two exceptions
are the scan phase of avoidance, which is about to sweep the whole arc
anyway, and `HALTED`, which is not going anywhere.

The detour deliberately leaves the line, which the brief allows, and every
way out of it ends in `RECOVER_LINE`, which the brief requires. It ends
early if the line reappears after `CAR_DETOUR_PAST_INDEX`, and a leg that
pings blocked is abandoned mid-drive and replanned. `CAR_MAX_DETOUR_ATTEMPTS`
bounds how many times the car will try to go round the same obstacle before
reversing instead; the count clears only after `CAR_DETOUR_CLEAR_MM` of
following with nothing ahead, so going round, finding the line and meeting
the same obstacle again does not buy a fresh budget.

Two things the code assumes that the course write-up should settle: the
Code 39 characters that carry each command (`BARCODE_CHAR_*`), and whether
a turn command means "at the next junction" (as implemented) or "now".

## Build

Needs `arm-none-eabi-gcc`, GNU make, a host C++ compiler for the UF2 tool
on first run, and `PICO_SDK_PATH` for the USB console and WiFi profiles.
The VS Code Pico extension installs the SDK under `~/.pico-sdk/sdk/`.

    export PICO_SDK_PATH=~/.pico-sdk/sdk/2.3.0
    cd firmware/build_make
    make CONSOLE=usb_cdc -j8                 # the car, no radio
    make CONSOLE=usb_cdc BENCH=motion -j8    # one bench

`firmware/flash.sh` wraps both of these and the flashing step; see Flash
below. It is the shorter path for everyday work.

With the radio and MQTT, copy `firmware/config/wifi_credentials.example.h`
to `wifi_credentials.h`, fill it in (git ignores it), set
`COMMS_MQTT_BROKER_HOST` in `car_config.h`, and build:

    make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
         WIFI_MQTT=1 -j8

`WIFI_MQTT` sits on DHCP alone; it does not pull in the port's DNS, UDP and
TCP echo qualification phases, so no echo server config files are needed.
The image name gains a `_mqtt` suffix. Changing any profile flag triggers a
full rebuild, by the port's design.

Switching between the car and a bench relinks by itself. Make only
compares timestamps and a switch touches no source, so `subdir.mk` keeps a
stamp under `build_make/mtkernel_3/app_program/` recording which selection
the image was linked from and removes the image when it changes. Without
that, `make` after `make BENCH=motion` would report nothing to do and the
bench would get flashed as the car.

## Flash

Every profile links to the same file name, so a bench built after the car
replaces it on disk. Build and flash in one step and that cannot bite you:

    cd firmware
    ./flash.sh                  # the whole car
    ./flash.sh motion           # the motion bench
    ./flash.sh line_barcode     # and so on for each subsystem
    ./flash.sh --wifi           # the car with the radio and MQTT
    ./flash.sh --no-recover     # the car, but it never searches for the line
    ./flash.sh --no-encoders    # the car timed open loop, encoders ignored
    ./flash.sh --wifi comms     # the comms bench, which needs the radio
    ./flash.sh servo            # servo only, straight at the PWM block
    ./flash.sh --build-only motion

It builds first and refuses to flash if the build failed, so the image on
the board is always the one you just asked for. Needs `picotool`, which
installs with `brew install picotool`.

`--no-recover` defines `CAR_SKIP_LINE_RECOVERY`, which redirects every
route into `RECOVER_LINE` back to `FOLLOW_LINE` and makes a lost line hold
course instead of giving up. With the IR modules off the car the search
finds nothing and ends in `HALTED` within a metre, which is why the hump
and obstacle behaviour cannot otherwise be driven. It is a car option and
refuses to combine with a bench. Switching it on or off rebuilds every
`app_program` object, because it changes a define and no source file.

`--no-encoders` forces `MOTION_OPEN_LOOP` to 1, so moves and turns are
timed and the encoder pins are never touched, whatever `car_config.h`
says. Without it, a closed loop image on a car with no encoders never
sees the wheels move, so every move and turn waits forever. It combines
with everything, benches included, and rebuilds the same way.

**The first flash of a Pico needs the BOOTSEL button.** Hold it down while
plugging in the USB cable, then run the script. From then on `picotool`
reboots the board into the loader itself and the button is never needed
again, because every profile here carries the USB console.

To flash by hand instead, hold BOOTSEL while plugging in, wait for the
`RPI-RP2` volume, and copy the image onto it:

    cp firmware/build_make/mtk3pico_smp0_usb_cdc.uf2 /Volumes/RPI-RP2/

Or, on a board already running any build from this tree:

    picotool load -f -x firmware/build_make/mtk3pico_smp0_usb_cdc.uf2

`-f` reboots the running board into the loader and `-x` starts the program
after loading.

WARNING: iCloud sometimes leaves conflict copies beside the build output,
named `mtk3pico_smp0_usb_cdc 2.uf2` and so on. They are stale images from
whenever the conflict happened. Never drag one of those onto `RPI-RP2`;
deleting them is safe, since the build recreates what it needs.

## Serial output

    ls /dev/cu.usbmodem*
    screen /dev/cu.usbmodemXXXX 115200       # quit: Ctrl-A, then K, then Y

Every bench waits 2 seconds before its first line, long enough to attach.
The car prints at boot, and USB serial drops anything sent before a
terminal connects, so open `screen` first and press the board's RESET
button to see its startup lines. The car logs every state change as
`state A -> B`, using these numbers:

| N | State | What it does | How it leaves |
|---|---|---|---|
| 0 | INIT | Nothing; calibration already ran before any task existed | Straight to 1 |
| 1 | FOLLOW_LINE | The PD steering law, speed adapted for terrain | 4 obstacle, 2 barcode, 5 line lost, 7 hit |
| 2 | DECODE_BARCODE | One tick: take the command, record it | 3 for a turn, 1 for straight |
| 3 | EXECUTE_TURN | Follows to the next junction, then turns once | 1 when the turn finishes, 4 if blocked first |
| 4 | AVOID_OBSTACLE | Scans, decides, drives the box, probes both sides | 1 if clear, 5 after a bypass, 6 if no lane |
| 5 | RECOVER_LINE | Search pattern, one step per tick | 1 when reacquired, 6 when exhausted |
| 6 | HALTED | `motion_stop()` every tick | Never; power cycle |
| 7 | COLLISION | Stops dead, settles, backs off, hands to 4 | 4 after backing off, 6 after three hits |

Two halts arrive with no `state ->` line, because they set the state
directly from outside the machine: `subsystem init failed, halting` at
bring up, and `encoder stalled, halting` from the motion task.

## Benches

Two layers, easy to confuse. `firmware/run_host_tests.sh` needs no board:
it compiles each module for the Mac with fake hardware underneath and
checks the maths and the contracts. Run it after any edit. A bench needs
exactly one subsystem's hardware, and ignores anything else that happens
to be plugged in.

Every bench is built and flashed the same way, and the build always
happens first because every profile links to the same `.uf2` name:

```sh
cd firmware
./flash.sh <bench>          # build it and flash it
./flash.sh <bench> --build-only
screen $(ls /dev/cu.usbmodem* | head -1) 115200
```

| Bench | Hardware | Power | Status |
|---|---|---|---|
| `run_host_tests.sh` | none | none | passing |
| car, nothing wired | Pico alone | USB | passing |
| `motion` | 2 motors on M1, M2 | battery | drives, turns accurate |
| `servo` | servo on S1 | battery | proven |
| `scanning` | servo on S1, HC-SR04 on Grove 4 | battery | proven |
| `detour` | 2 motors, servo, HC-SR04 | battery | proven; probe path untested |
| `imu_terrain` | GY-511 on Grove 3 | USB | pitch proven |
| `comms` | Pico W radio, a broker | USB | proven end to end |
| `line_barcode` | 3 IR modules | USB | **on hold** |

The motion, servo, scanning and detour benches need the battery. Both the
motor rail and the servo header run from it, and USB alone sags and
resets the board however healthy the console output looks.

### Per buddy, every command

Run all of these from `firmware/`. `./flash.sh` builds first and refuses
to flash a failed build. Watch any of them with
`screen $(ls /dev/cu.usbmodem* | head -1) 115200`, quit with Ctrl-A K Y.

| Buddy | Commands |
|---|---|
| all | `./run_host_tests.sh` before every push |
| 1 comms | `./flash.sh --wifi comms`, then `mosquitto_sub -h <broker> -t 'car/#' -v` and `mosquitto_pub -h <broker> -t car/command -m L` |
| 2 motion | `./flash.sh motion` |
| 3 line_barcode | `./flash.sh line_barcode` |
| 4 imu_terrain | `./flash.sh imu_terrain`, and again with `BENCH_DRIVE` 1 in `bench_imu_terrain.c` for hump height |
| 5 scanning | `./flash.sh servo`, `./flash.sh scanning`, `./flash.sh detour` |
| team | `./flash.sh`, `./flash.sh --wifi` for the radio build, `./flash.sh --no-recover` to drive with no line sensors fitted, `./flash.sh --no-encoders` to drive with no encoders fitted |

Add `--build-only` to any of them to build without flashing, for example
`./flash.sh --build-only motion`. Two benches have a flag to flip in the
source first: `BENCH_DRIVE` in `bench_imu_terrain.c` and
`BENCH_FIND_CENTRE` in `bench_servo.c`.

### `motion`

Wheels off the ground for the first run. It drives each motor in its own
phase, so a wheel that turns in one phase and not the other isolates the
fault to that motor, its screw terminal or that channel of the board; the
board's own M1A and M2A buttons settle which. It then prints
`encoders seen: left N right N`.

The encoder half only runs when `MOTION_OPEN_LOOP` is 0. Until then it
says so and skips to the drive. An encoder that never pulses is treated as
absent rather than broken, so the car drives open loop on that wheel
instead of refusing to move; only a wheel that pulsed and then went quiet
while still commanded is a fault.

Use this bench to calibrate distance. Command a straight leg, measure what
it actually travelled with a tape, and scale `MOTION_MAX_SPEED_MM_PER_SEC`
by actual over commanded. Nothing is counted in open loop, so that one
constant scales every distance and every turn in the whole car. The motion
README has the procedure.

### `servo`

Reach for this when the horn does not move. It talks straight to the PWM
block through `car_hw` and never calls `scanning.c`, so it takes all of
our scanning code out of the question. Three phases: centre held four
seconds, a one degree sweep across the band and back, then signal off for
two seconds, which usually makes a servo that was holding go slack.

Tape a pointer to the horn first. On this car the horn points straight
ahead at 1744 us, so the band runs 1469 to 2019 us for 65 to 115 degrees.

A horn that swings a long way once at startup and then barely moves is not
a bug. A servo reports nothing about where it is, so the first pulse
throws it from wherever it was left to the commanded centre, and how far
it travels is just the gap between the two. Set `BENCH_FIND_CENTRE` to 1
at the top of `bench_servo.c`, run it with the horn clear, and it walks
the pulse across a wide range printing each step so the right value can be
read off into `SERVO_CENTRE_PULSE_USEC`.

If the horn moves here but not in `scanning`, the fault is in
`scanning.c`. If it moves in neither, work down this list: the battery is
not connected or the board is off; the three pin lead is on the wrong
header or reversed; the horn is jammed against its mount; or the servo is
dead, which a spare settles in a minute.

### `scanning`

Servo first, sonar second, so the two faults cannot be confused. It parks
the horn at each end and the centre with a second between moves and takes
one reading at each, then pings ten times without moving and counts the
echoes. A horn that moves with zero echoes is a sonar problem. No movement
and no echoes usually means the battery.

The HC-SR04 runs from the Grove port's 3.3 V with no divider, which costs
maximum range but keeps the echo pin safe. This is the bench that measures
what that range actually is.

### `detour`

Drives straight and goes round whatever it meets, with no line sensors
involved at all. This is the bench for "does the box detour actually
detour". It stops when something comes inside `SCAN_OBSTACLE_RANGE_MM`,
prints what each of the three scan angles sees, prints the decision, then
drives the box announcing each leg.

```
obstacle at 180 mm, stopping to look
   65 deg    812 mm  clear
   90 deg    180 mm  blocked
  115 deg    790 mm  clear
profile: valid 1 bearing 0 closest 180 width 61, clear left 790 right 812
decision: go RIGHT (action 3)
  leg 1/7  turn away 45 deg right
```

Leg lengths come from `CAR_WIDTH_MM` and `CAR_LENGTH_MM`, which are the
car measured with a ruler, currently 150 and 200. The sideways leg moves
half the obstacle plus half the car plus 50 mm of slack, divided by sin 45
for the diagonal; the forward leg is the obstacle's width plus the whole
car's length, because the back wheels are still beside the obstacle when
the bumper is past it. A 100 mm obstacle gives side 247 and depth 425.
`SCAN_CLEARANCE_MIN_MM` is `CAR_WIDTH_MM` plus 100, so a lane under 250 mm
does not count as one; if the car now reverses where it used to squeeze
through, that is this number and not a fault.

Those three range lines are the whole story when a decision looks wrong:
the planner never sees anything else. If it always reverses, both side
readings are coming back under `SCAN_CLEARANCE_MIN_MM`, and the printed
numbers say whether that is a wide obstacle or the sonar catching the
floor.

If a driving leg pings blocked part way round, the lane is a dead end
too. The bench undoes the legs already driven, newest first, which puts
the car back where the detour started, then tries the other side:

```
  leg 4/7 blocked at 140 mm, backing out
  backing out 3 leg(s)
  undo 3/3  turn back parallel
  undo 2/3  step sideways
  undo 1/3  turn away
back at the start, trying right
```

Undoing means turning the same amount the other way and driving the same
distance in reverse, so it is only as accurate as the open loop odometry.
Expect drift over several probes, and note that `CAR_MAX_DETOUR_ATTEMPTS`
caps the shuffling at four tries before it gives up.

Nothing brings the car back to anything here, so after a detour it drives
off in whatever direction the box left it pointing. Give it floor space.

### `imu_terrain`

The `ok` and `mag` columns are the ones to read. An accelerometer measures
apparent gravity and cannot tell a tilt from a push, so one running motor
shakes an unbalanced chassis enough to swing pitch by several degrees on a
perfectly level car.

Pitch is therefore frozen whenever the reading cannot be believed. `mag`
is the filtered vector length in milli g, near 1000 when still. `ok` is 1
while `mag` is within `IMU_PITCH_TRUST_BAND_MILLI_G` of one g, and pitch
holds its last value whenever `ok` is 0. A motor running, `mag` swinging
and `ok 0` is correct behaviour. `ok 0` with the car standing still means
the mount transmits too much vibration, or the band is too tight.

The bench prints one line carrying every item Buddy 4 owns, in order:

```
cal 1 | pitch 3 ok 1 mag 1004 raw 1012 | hump 0 peak 0 mm | event STILL |
hit 0 | rate 0 dps | heading 187 | terrain STABLE rough 11
```

`cal` is sensor calibration, `pitch`/`ok`/`mag` are tilt and whether it
can be believed, `raw` is the largest unfiltered magnitude since the last
line and the only column a knock shows up in, `hump`/`peak` are detection
and peak measurement,
`event` is the motion class, `hit` is collision, `rate` is turn rate, and
`terrain`/`rough` is the terrain summary. Roughness is the smoothed
distance of the gravity vector from one g: near zero on a smooth floor
because gravity is then the only force, climbing with every bump.

**Collision:** the `hit` column, and `raw` is how you tune it. Hold the
car still and tap the bumper harder and harder until `hit` goes to 1, then
back off until it stops triggering. `IMU_COLLISION_THRESHOLD_MILLI_G` is
600 over one g, so a knock has to reach `raw` 1600 to count; set it just
above the hardest knock that should not.

Watch `raw`, not `mag`. `mag` is the filtered vector, and the filter takes
16 samples to respond, so a tap that peaks at 2500 for one sample barely
moves it: that column simply cannot show you an impact. The collision test
has always run on the raw sample, which is what `raw` reports. `hit` is
latched and cleared when read, so it appears on exactly one printed line
and is easy to scroll past.

A sensor mounted high on the chassis feels a bumper tap through whatever
it is bolted to. If `raw` barely lifts when you knock the bumper hard,
the mount is absorbing the impact and the threshold is not the problem.

**Hump height:** the `peak` column, and it needs the car driving. Height
is the integral of sin(pitch) over ground distance, so with the car
stationary it stays at zero however clearly the pitch moves. Set
`BENCH_DRIVE` to 1 at the top of `bench_imu_terrain.c`, which makes the
bench drive straight and feed the odometry in. Then:

1. `BENCH_DRIVE` 0 first, on the desk. Tilt the car nose up by hand and
   confirm pitch reads positive; that settles `IMU_PITCH_SIGN`.
2. Still static, tilt to roughly the angle of your real hump and check
   pitch passes `IMU_HUMP_PITCH_THRESHOLD_DEG`, which is 5 degrees.
3. `BENCH_DRIVE` 1, battery on, drive over the hump on the floor.
4. Compare `peak` against a ruler on the hump. Scale
   `IMU_PITCH_SIGN` or the threshold if the sign or the trigger is wrong.

Watch `ok` during step 3. A car driving over a bump is accelerating, so
the trust gate may freeze pitch exactly when the hump is under the wheels
and leave `peak` at zero. If that happens, widen
`IMU_PITCH_TRUST_BAND_MILLI_G` until pitch survives the climb, at the cost
of letting some vibration through. That trade is the whole difficulty of
doing this without a gyroscope. It is now 300, raised from 150 because a
sensor mounted high on the chassis swings through a tight band on any
floor: the higher the mount, the longer the lever arm on every vibration.

The same gate is why the car may never switch to the climb speed. The
event that `terrain_speed()` keys off is derived from pitch, and frozen
pitch means a frozen event. The car logs `terrain speed N, event E,
pitch ok K` whenever the commanded speed changes, so a run that never
prints that line at all on a hump is the gate holding, not the hump being
missed. `ok 0` for most of a drive says the band is still too tight.

### `comms`

Needs the radio profile: `./flash.sh --wifi comms`. It refuses without it.
Proves the join, the MQTT connect and the command echo. Watch it from a
second terminal:

```sh
mosquitto_sub -h <broker ip> -t 'car/#' -v
mosquitto_pub -h <broker ip> -t car/command -m L
```

The broker's own `-v` log prints protocol lines and byte counts but never
payloads, so use `mosquitto_sub` to see contents.

### `line_barcode` (on hold)

Held while the IR modules are off the car. When they go back on, `raw` is
the electrical level on each pin before interpretation, `changes` counts
every flip per sensor since boot, and `health` goes CAPITAL per sensor
once it has been seen both dark and light.

Wave a hand across one sensor at a time and watch `changes` climb: that
proves the sensor is wired, powered and detecting. A counter stuck at zero
means nothing is reaching the pin. Between them they tell a junction from
a dead loom: `mask 111` with three CAPITALS is a real junction, `mask 111`
with three lower case letters is three sensors that have never changed.
`mask 000 health rcl` with nothing under the car is the normal resting
state, not a fault.

## Mission status

The brief's requirements, against what has actually run on hardware. The
course write-up PDF is not in the repo, so this tracks capability rather
than quoting requirement numbers.

**Collision failsafe**

A hit outranks every state. `imu_is_collision_detected()` is checked
above the state switch, before the sonar and before any state runs, so it
can interrupt a detour leg, a turn or a search part way through a move.
State 7 then stops dead, holds still for `CAR_COLLISION_SETTLE_MSEC` while
the impact rings out of the accelerometer, backs straight off
`CAR_COLLISION_BACKOFF_MM`, and hands over to avoidance to scan and
decide. Three hits in one run and it halts for good.

It backs off blind on purpose. The accelerometer says a hit happened and
roughly how hard, but nothing says where: the sonar was pointing wherever
it was pointing and the line sensors look at the floor. Straight back
along the path just driven is the one direction known to have been clear
a moment ago. Looking happens afterwards, from far enough away that the
sonar's minimum range is not in the way.

**Scanning while moving**

`SCAN_SWEEP_WHILE_MOVING` is 1, so the coarse angles are swept while
driving in the pattern centre, right, centre, left. Only a centre reading
can trigger avoidance; the side readings are early warning that goes in
the log, so the car knows which side is open before it has to choose.

The cost is real and worth stating: forward is ranged on every other
reading rather than every one, so an obstacle can be one reading closer
before it is seen. Raise `SCAN_OBSTACLE_RANGE_MM`, drop the speed, or set
the flag to 0 to stare straight ahead instead.

**Terrain speed**

`terrain_speed()` overrides the commanded speed from the motion event:
`CAR_CLIMB_SPEED_MM_PER_SEC` while climbing, because the car stalls on
the face of a hump at following speed, and `CAR_DESCEND_SPEED_MM_PER_SEC`
while descending or while `imu_is_terrain_stable()` is false. The climb
figure is 600 mm/s, raised from 300 because the car stopped on the face of
the hump: speed maps straight to duty, so 300 asked for 187 per mille
against a floor of 150 and there was no torque left to climb with. It keys off
the motion event rather than the pitch angle, because pitch is frozen
exactly when the accelerometer cannot be believed, which is exactly when
a car is climbing a bump.

**Working, seen on hardware**

| Capability | Evidence |
|---|---|
| Both motors driven over PWM, forward, reverse, turn | motion and detour benches |
| Distances and turn angles accurate | detour legs measured correct after calibration |
| Servo pointing the sonar across the band | servo and scanning benches |
| Obstacle side decision, left against right | detour bench picks the side with room |
| Box detour round an obstacle | detour bench, full seven legs |
| Ultrasonic ranging at 3.3 V | telemetry reported 519 mm, correctly not an obstacle |
| Tilt from the IMU, with a vibration trust gate | imu_terrain bench |
| WiFi join and MQTT connect | broker log, car connected as client `car` |
| Telemetry and heartbeat published as JSON | both topics live on the broker |
| Remote commands received and parsed | `car/command` subscribed, L/R/S/U parsed |
| Mission state machine runs and reports state | telemetry `state` field tracked to 6 |

**Needs further testing**

| Capability | What is blocking it |
|---|---|
| Alternating left and right probe | new; the retrace leans entirely on open loop odometry |
| Five angle fine scan | was three until now, so the extra two are untested |
| Hump detection and peak height | set `BENCH_DRIVE` to 1 and drive over one |
| Collision detection | threshold never triggered on hardware |
| Barcode decode | on hold with the IR modules |
| Line following and junction detection | on hold with the IR modules |
| Line reacquisition after a bypass | code is in, needs the IR modules to prove |
| Encoders, closed loop speed and counted distance | not fitted; `MOTION_OPEN_LOOP` stays 1 |

**Known limits right now**

`MOTOR_MIN_DUTY` floors every duty, which at the current top speed puts
the slowest achievable speed at about 240 mm/s. Both
`CAR_FOLLOW_SPEED_MM_PER_SEC` and `CAR_BARCODE_SPEED_MM_PER_SEC` are below
that, so the car cannot yet go as slowly as the barcode decoder wants.
Measure the lowest duty that starts the car from rest and lower it.


## Bring up order

Prove each thing before the next depends on it. Wheels off the ground
until line following behaves.

1. Board alone, nothing plugged in: `./flash.sh`, then watch the console
   for the startup lines and `motion ready`, `line ready`. Missing sensors
   log as absent rather than failing.
2. Motors on the battery: press the board's M1A and M2A test buttons before
   any code. Then `./flash.sh motion`, which drives each motor alone before
   it drives both. Swap a terminal pair if a wheel runs backwards, and
   confirm M1 is physically the left motor.
3. `./flash.sh imu_terrain`: tilt nose up, pitch positive. Run the motors
   held still and note the heading shift.
4. `./flash.sh servo`: the horn must move. Everything below this step
   depends on it, and nothing else can tell you it is broken.
5. `./flash.sh scanning`: watch the horn move in phase 1, count echoes in
   phase 2, then place a box at 300 mm for the sweep.
6. `./flash.sh detour`: a box on the floor, off to one side. Check the
   three printed ranges match where the box really is, that the decision
   picks the side with more room, and that leg 1 is a recognisable 45
   degrees rather than a spin.
7. `./flash.sh --wifi comms`: `connected 1`, then publish to `car/command`
   from a laptop and watch it echo.
8. **On hold** until the IR modules go back on: `./flash.sh line_barcode`.
   Wave each sensor over tape until all three health letters are CAPITAL,
   then slide the car sideways across the line so the mask walks 001, 000,
   100 (right, barcode, left) and the error -2, 0, 2.
   Drive it over a printed barcode and watch for `barcode command`.
9. The whole car on the floor. This needs step 8 first: without the line
   sensors the car has nothing to follow and halts once the search gives
   up.

## Pin map

Verified pins come from the board maker's example code. Grove port
numbers and pairs are from the board documentation.

| Function                  | Pins            | Where                       |
|---------------------------|-----------------|-----------------------------|
| Motor left M1A, M1B       | GP8, GP9        | M1 terminal, verified       |
| Motor right M2A, M2B      | GP10, GP11      | M2 terminal, verified       |
| Scan servo                | GP12            | header S1, verified         |
| Console mirror, UART0 TX  | GP0             | Grove 1 yellow, leave loose |
| Encoder right             | GP1             | Grove 1, white              |
| Line sensor right         | GP2             | Grove 2, yellow             |
| I2C0 SDA, SCL to IMU      | GP4, GP5        | Grove 3                     |
| Sonar trigger, echo       | GP16, GP17      | Grove 4                     |
| Line sensor left          | GP6             | Grove 5, yellow             |
| Barcode sensor            | GP26            | Grove 6, yellow             |
| Encoder left              | GP7             | Grove 7, yellow             |
| Board: NeoPixels          | GP18            | verified, leave alone       |
| Board: buttons            | GP20, GP21      | verified, spare inputs      |
| Board: buzzer             | GP22            | verified, leave alone       |
| Pico W radio              | GP23 to 25, 29  | not available               |

GP26 is shared between Grove 5 and Grove 6; only Grove 6 uses it. Which
physical sensor is left, barcode and right is set by where each module is
mounted: swap the three `LINE_SENSOR_*_PIN` numbers to match.

## Line sensors and encoders

Grove 1 carries UART0, the kernel's serial console. The console you
actually read is USB and UART0 only mirrors it, so its pins can be taken
back. GP1 is UART receive, an input on the Pico's side, so the right
encoder can sit on it from power on with nothing driving against it;
`motion_init()` then switches it to plain GPIO. GP0 is UART transmit and
is driven from boot, so it must never meet an encoder output: the encoder
goes on the **white** wire and the yellow wire stays unconnected.

The right line sensor sat on GP1 first and did not read there, so it moved
to Grove 2. If the right encoder never counts either, suspect GP1 or the
Grove 1 cable before the encoder.

| Cable   | Red      | Black     | Yellow             | White          |
|---------|----------|-----------|--------------------|----------------|
| Grove 1 | motor white, supply | motor blue, ground | nothing, tape it | motor red, A phase |
| Grove 2 | IR VCC   | IR GND    | IR DO, right line sensor | nothing  |
| Grove 7 | motor white, supply | motor blue, ground | motor red, A phase | nothing |

Test it in this order, wheels off the ground:

1. `./flash.sh line_barcode`, then open `screen`.
2. After two seconds you should see `line bench: pins L6 B26 R2`, then a
   status line every 200 ms or so.
3. The mask prints right, barcode, left. Hold black tape under the Grove 2
   sensor only: the first digit must change and `error` must read 2. If
   the digit changes the wrong way, flip `LINE_SENSOR_DARK_LEVEL`.
4. Do the same under the Grove 5 sensor, last digit and error -2, and the
   Grove 6 sensor, middle digit only, with `error` unchanged because the
   barcode sensor never steers. Every sensor now has a known port.
5. Build the car, flash, and confirm `motion ready` and `line ready` at
   boot. From here the bring up order above applies.

Leave `MOTION_OPEN_LOOP` at 1 until the encoders are physically wired.
Only when they are, set it to 0 and rerun the motion bench: it reports
`encoders seen: left 1 right 1`, and the 500 mm drive then measures rather
than times itself.

## Kernel notes

- Single core. `SMP=1` exists but the port documents I2C, ADC, PWM and
  UART as single owner resources under SMP, and nothing here needs the
  second core.
- The tick is `CNF_TIMER_PERIOD` in `firmware/config/config.h`, 10 ms.
  Task delays round up to it.
- The kernel releases only its own blocks from reset. `common/car_hw.c`
  releases PWM and TIMER on first use and sets the timer tick to one
  microsecond, idempotently, so the order the modules initialise in does
  not matter.
- Interrupts are registered with `tk_def_int()` and `EnableInt()`, never
  by writing the vector table. The encoders share NVIC line 13.
- A file that includes `<tk/tkernel.h>` must not include `<stddef.h>`,
  `<stdio.h>`, `<string.h>` or `<stdlib.h>`: the kernel typedefs its own
  `size_t` and the two collide. Every module includes the kernel only
  outside `CAR_HOST_TEST` and keeps its hardware calls in small static
  functions at the bottom of the file with a host fake beside them.
- lwIP runs `NO_SYS=1` on the radio task. The MQTT phase is the only lwIP
  caller the car adds, and application tasks reach it through ring buffers.
- Edits to the vendored kernel, and the only ones: `build_make/pico_rp2040.mk`
  gained a `pico_util` include path in two places for SDK 2.3 and the
  `WIFI_MQTT` profile; `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/.gitkeep`
  exists because the USB console rule needs that directory on macOS make
  3.81; `lib/libnet/lwip/include/lwipopts.h` enables TCP and sizes the
  pools for MQTT; `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` polls
  the MQTT phase once DHCP has an address; `lib/libnet/lwip/lwip_utk_mqtt.*`
  is new; `app_program/subdir.mk` is ours; and every board but
  `pico_rp2040` was deleted on 2026-09-25, along with the branches that
  selected them in `build_make/makefile`, `build_make/mtkernel_3/device/*/subdir.mk`,
  `include/sys/machine.h` and `config/config.h`. Git history has them if
  another board is ever needed.
