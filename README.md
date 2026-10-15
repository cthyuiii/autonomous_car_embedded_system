# Autonomous Car Firmware

Autonomous line following car on a Raspberry Pi Pico W carried by a Cytron
Robo Pico, running micro T-Kernel 3.0. Five subsystems, one vehicle
controller, one Pico on the car. The other Picos are development benches,
one per person.

## Layout

`firmware/` is the kernel port, vendored from
[sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040)
at commit 2e8e6ad, the Raspberry Pi Pico port listed in the course template
[sirfonzie/mtk3-hub](https://github.com/sirfonzie/mtk3-hub), under its own licences (see `firmware/LICENSE` and `firmware/LICENSES/`).
Everything of ours lives in `firmware/app_program/`. Do not edit the kernel
tree except the places listed under Kernel notes.

| Path                                | Owns                                   |
|-------------------------------------|----------------------------------------|
| `app_program/car_main.c`            | `usermain()`, the tasks, mission state |
| `app_program/common/`               | shared types, every config knob, logging, the hardware layer, the clock |
| `app_program/comms/`                | MQTT telemetry, heartbeat, commands    |
| `app_program/motion/`               | motors, encoders, PID, distance, turns, steering |
| `app_program/line_barcode/`         | three IR sensors, position, junctions, Code 39 barcodes |
| `app_program/imu_terrain/`          | LSM303DLHC, tilt, humps, events, collision |
| `app_program/scanning/`             | servo, HC-SR04, profiling, avoidance, recovery |
| `build_make/mtkernel_3/app_program/`| `subdir.mk`, how our code is built     |
| `flash.sh`                          | build one image and flash it, in one step |
| `run_host_tests.sh`                 | every host test, no board needed       |
| `lib/libnet/lwip/lwip_utk_mqtt.*`   | the MQTT phase on the kernel's radio task |

Each subsystem folder holds its public header under `include/`, the
implementation, a host test, a hardware bench and a README with its
calibration table. The two written deliverables sit with their code:
`motion/tuning_report.md` (PID tuning and motion accuracy) and
`imu_terrain/terrain_report.md` (terrain analysis).

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

Every file you own says so in its header comment. Change a header only by
agreement with the team. Add to a host test, never remove from it. Extend
your bench as you like. Record your measurements in your README's table.

The code follows the Barr C coding standard. Each module's public
functions carry the module's name, so `line.c` exports `line_*`, `imu.c`
exports `imu_*` and `scan.c` exports `scan_*`, while the folders keep the
subsystem names.

## Status

| Capability | State on the car |
|---|---|
| Motors over PWM: forward, reverse, turn on the spot | Working |
| Encoders, both phases, speed control | Working: 639 pulses and 205 mm per wheel turn, left tyre 993 and right 1007 per mille of that (tuning runs, 2026-10-14) |
| Straight line correction on distance moves | Working: 500 mm drives land 500 to 515 mm, straight (2026-10-14) |
| Line following, two sensors straddling the line | Working; speed-ups ramp at 200 mm/s², and a centred car no longer slows |
| Back onto the line after a detour or search: cross, creep, turn on the spot | Working up to 90 degrees (`CAR_REENTRY_TEST`, 2026-10-14) |
| Stop at a junction, then turn as the barcode says | Working on the course (2026-10-14); creep cut to 100 mm after a 50 mm overshoot, to rerun |
| Barcode decoding, Code 39 A, B, C, D | Working: all four on the bench, A on the course; a missed barcode is reread by backing up |
| Tilt from the IMU | Working |
| Hump detection, count and heights | Built and host tested, not yet run over a hump |
| Control loops every 10 ms, woken by the kernel tick | Working: 9 ms measured |
| Current action and terrain status over MQTT | Working, shown on the dashboard |
| Collision from the IMU | Working: hits reported in any state and acted on |
| Stuck detection: a wheel driven but not turning | Built, not yet run on the car |
| Servo and ultrasonic ranging at 3.3 V | Working |
| Obstacle profile and box detour | Runs; flat boxes profile narrow, so detours plan for at least 220 mm. The length is walked with a look every 50 mm: a 220 by 195 mm box cleared on the bench (2026-10-14, at 25 mm) |
| Search for the line after a bypass | Built, not yet proven on a full run |
| Wi-Fi telemetry, heartbeat and remote commands | Working end to end |
| Hardware layer on the Pico C SDK (`common/car_hw.c`) | Register for register the same as the kernel BSP calls it replaced; rerun every bench |

**Resources**, which the write-up asks to be used prudently (WiFi car
build, 2026-10-15):

| Resource | Used |
|---|---|
| Flash | 348 KB of 2 MB (17 %) |
| RAM, static | 65 KB of 264 KB (25 %), task stacks 4 KB each |
| Processing | control loops every 10 ms tick, 9 ms measured; the radio on its own task |
| Radio | telemetry, terrain and heartbeat every 5 s, about 0.1 KB/s, plus events as they happen |

The `TODO:` comments left in the code mark values still to be measured or
confirmed on the car.

## How the car behaves

- **motion** drives the board's H-bridge over PWM. Each wheel has a speed
  loop, a feedforward term plus PID, on the speed measured from the time
  between encoder pulses. Phase B gives the direction, and a reversal only
  counts once 4 pulses in a row agree. Distance moves also hold their
  heading (`MOTION_STRAIGHT_KP`). Turns on the spot run at
  `MOTION_TURN_SPEED_MM_PER_SEC`. Every speed-up is gradual: the faster wheel's
  setpoint climbs at `MOTION_ACCEL_MM_PER_SEC2` (200, so 0 to 150 mm/s in
  0.75 s) and the other wheel keeps its share, so steering works from rest;
  slowing and stopping are immediate. Each wheel has its own
  share of the circumference (`WHEEL_LEFT_PERMILLE`, `WHEEL_RIGHT_PERMILLE`),
  so the car runs straight on ground, not on pulse counts. Both encoders are
  required: a wheel whose encoder never pulses reads zero speed and is
  driven hard.
- **line_barcode** reads three MH-Sensor-Series modules as a 3 bit mask.
  Left (Grove 5) and right (Grove 2) straddle the line and steer. The
  third (Grove 6) sits off to the left, beside the line, and only reads
  barcodes; a timer interrupt samples it every 500 us. At start-up every
  sensor must read floor, and the console names any that do not. Sensor
  placement is under Line sensors and encoders.
- **imu_terrain** runs the LSM303DLHC on Grove 3 and moves the kernel's
  I2C unit off the motor pins at init. Pitch is relative to a level
  reference taken at boot and is frozen while the car shakes or
  accelerates. See `terrain_report.md` for how each quantity is worked out.
- **scanning** runs the HC-SR04 at 3.3 V from Grove 4, with no echo
  divider, and the servo on S1 across 65 to 115 degrees.
- **comms** publishes JSON over MQTT through a phase on the kernel's radio
  task, only in the `WIFI_MQTT=1` build: telemetry on `car/telemetry`,
  including what the car is doing as text (`action`), the terrain status
  on `car/terrain` and a heartbeat, each every 5 s. Anything that happens
  in between, such as a hit, a hump, a barcode, an obstacle scan or a new
  action, goes out at once on `car/event`. `comms/README.md` lists every
  key. Without the radio build the controller logs the radio as
  absent and drives anyway.
- **car_main** calibrates before starting the tasks, then follows the line,
  stops at every junction, drives a box detour round an obstacle, and
  searches for the line afterwards. In the radio build it first stands still
  until the broker has its first telemetry, so the run is reported from the
  start; with no broker after `CAR_BROKER_WAIT_MSEC` (60 s) it drives anyway.
  Start the broker and the dashboard before switching the car on.

**Barcodes and junctions.** The Code 39 characters that carry each command
(`BARCODE_CHAR_*`) follow the write-up's table: A Left, B Right, C
Straight, D U-turn. The line is 18 mm wide. Each barcode is 141 mm long
and sits along the track, beside the line on the left, 20 mm from its
edge. Junctions are crosses, so a junction puts the line under both line
sensors: at once when the car meets it square, one just after the other
when it meets it skewed. Each sensor dark within the last
`LINE_JUNCTION_WINDOW_MSEC` (100 ms) counts, so a skewed cross that only
darkens both for an instant is not missed. A barcode is held until the next cross, however far
away; a later barcode replaces an earlier one. Every junction has a barcode
just before it, so a junction reached with nothing read means the side
sensor missed it: the car backs up at 60 mm/s, reading as it goes, until it
reads the barcode or the barcode sensor has passed its bars and 50 mm of
plain floor (600 mm at most, `CAR_REREAD_MAX_MM`), then follows the line
back to the junction just as slowly, once per junction. With still nothing
read it goes straight on, so no distance needs measuring.

To test barcodes and junctions on their own, set `CAR_BARCODE_TEST` to 1
in `car_config.h`: the sonar is never used, so hands, walls and boxes near
the course cannot stop the car. Set it back to 0 for runs; the console says
so at start-up while it is on. Benches built with `--wifi` wait for the
broker before they start, like the car, so the dashboard sees them from
their first line.
- **At the junction:** the car stops for `CAR_JUNCTION_WAIT_MSEC`, taking
  any command that arrives in that time.
- **A or B:** it creeps `CAR_JUNCTION_CREEP_MM`, so its wheels sit over
  the crossing, and turns 90 degrees.
- **D:** it spins 180 degrees where it stands.
- **C, or no barcode:** it goes straight on.

A barcode still under the barcode sensor when the car stops at a cross is
lost: the stop stretches one bar past `BARCODE_MAX_ELEMENT_MSEC` and the
decoder drops the symbol. So no barcode may overlap a cross.

**Obstacles.** One sonar reading per `CAR_SONAR_CHECK_PERIOD_MSEC` is taken
above the state switch, and every state reads the same answer, so a turn,
a detour leg and the line search all give way to something in the path.
The exceptions are the scan phase of avoidance, which sweeps the whole arc
itself, and `HALTED`.

While driving, the sonar sweeps centre, right, centre, left
(`SCAN_SWEEP_WHILE_MOVING`). A close reading at any of the three angles
stops the car, and the fine scan that follows decides whether anything is
really in the way. Straight ahead is therefore ranged on every other
reading. If obstacles are seen too late, raise `SCAN_OBSTACLE_RANGE_MM`,
drop the speed, or set the flag to 0 to look straight ahead only.

**The detour** leaves the line, which the write-up allows, and every way
out of it ends in `RECOVER_LINE`, which the write-up requires.
- **Ending early:** it ends as soon as the line reappears after leg
  `CAR_DETOUR_PAST_INDEX`, and a leg that pings blocked is abandoned and
  replanned.
- **Walking the length:** the sonar sees only the obstacle's front, so
  once beside it the car steps along `SCAN_DETOUR_STEP_MM` (50 mm) at a
  time and turns `SCAN_DETOUR_LOOK_DEG` (65°) toward it at each step to
  look. With the sonar at its 25° limit, that meets the side square on. While
  something is nearer than the line it steps on; once clear it drives
  `SCAN_DETOUR_PAST_MM` (100 mm) more and turns in. The sideways leg leaves
  room for the nose to swing when it turns to look (`SCAN_NOSE_SWING_MM`).
- **Giving up:** `CAR_MAX_DETOUR_ATTEMPTS` limits how often the car goes
  round the same obstacle before it reverses instead. The count clears
  only after `CAR_DETOUR_CLEAR_MM` of following with nothing ahead.

**Getting back on the line.** After a detour or a search the car usually
meets the line at an angle. Steering from two straddling sensors then only
turns it further across, so it does what it does at a junction:
1. drives straight on until the second sensor touches the line;
2. creeps until its wheels are over the line, a distance worked out from
   how far apart the two touches were;
3. turns on the spot toward the second sensor and stops as soon as that
   sensor touches the line, which leaves room for the spin to coast; normal
   steering finishes from there.

Below about 5 degrees it skips the creep and the turn, and normal steering
takes over.

**Collisions.** A hit outranks every state. It is checked above the state
switch, so it can interrupt a detour leg, a turn or a search mid move.
State 7 then:
1. stops and holds still for `CAR_COLLISION_SETTLE_MSEC`;
2. backs straight off `CAR_COLLISION_BACKOFF_MM`;
3. hands over to avoidance to scan and decide.

It backs off blind because nothing says where the hit came from; the way
the car just came is the one direction known to be clear. After three hits
in one run it halts for good.

A wheel that is driven but has not turned for `MOTION_STALL_FAULT_MSEC`
counts as a hit too, which catches obstacles in the sonar's blind spot.

**Terrain speed.** `terrain_speed()` adjusts the commanded speed from the
IMU's motion event:
- **Climbing:** the car keeps its speed and the speed loop
  adds power as the slope slows the wheels.
- **Descending, or rough ground:** it slows to
  `CAR_DESCEND_SPEED_MM_PER_SEC`.

The event comes from pitch, so it holds still whenever pitch is frozen.

## Build

Needs `arm-none-eabi-gcc`, GNU make, a host C++ compiler for the UF2 tool
on first run, and `PICO_SDK_PATH` for every profile. The hardware layer
uses the SDK's hardware headers, and the USB console and Wi-Fi use its
libraries. The VS Code Pico extension installs the SDK under
`~/.pico-sdk/sdk/`.

    export PICO_SDK_PATH=~/.pico-sdk/sdk/2.3.0
    cd firmware/build_make
    make CONSOLE=usb_cdc -j8                 # the car, no radio
    make CONSOLE=usb_cdc BENCH=motion -j8    # one bench

`firmware/flash.sh` wraps both of these and the flashing step; see Flash
below. Use it for everyday work.

For the radio and MQTT:
1. Copy `firmware/config/wifi_credentials.example.h` to
   `wifi_credentials.h` and fill it in. Git ignores the copy.
2. Set `COMMS_MQTT_BROKER_HOST` in `car_config.h`.
3. Start the broker on the laptop in a terminal of its own and leave it
   open. Mosquitto 2 listens only to the laptop itself unless told
   otherwise, and the Pico W has no login, so save these two lines as
   `~/mosquitto-car.conf`:

       listener 1883 0.0.0.0
       allow_anonymous true

   then run `mosquitto -c ~/mosquitto-car.conf -v`. Don't use `brew
   services` for the car: it reads a different config file.
4. Build:

       make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
            WIFI_MQTT=1 -j8

`WIFI_MQTT` needs only DHCP. It does not pull in the port's DNS, UDP and
TCP echo test phases, so no echo server config files are needed. The image
name gains a `_mqtt` suffix. Changing any profile flag rebuilds
everything.

Switching between the car and a bench relinks by itself. `subdir.mk` keeps
a stamp under `build_make/mtkernel_3/app_program/` recording which
selection the image was built from, and removes the image when it changes.

WARNING: the macOS `make` is GNU Make 3.81, which compares whole seconds.
If a script edits `car_config.h` in the same second a build finished, the
next build can reuse stale objects. After scripted edits, delete
`build_make/mtkernel_3/app_program/**/*.o` before building.

WARNING: do not run `make -n` for a different profile or bench than the
last build. Both build stamps delete objects and images from a `$(shell)`
call while the makefile is read, even under `-n`.

## Flash

Every profile links to the same file name, so a bench built after the car
replaces it on disk. Build and flash in one step, so the board always gets
the image you just asked for:

    cd firmware
    ./flash.sh                  # the whole car
    ./flash.sh motion           # the motion bench
    ./flash.sh encoders         # count encoder pulses by hand, motors off
    ./flash.sh tuning           # step response and accuracy runs
    ./flash.sh line_barcode     # the IR sensors and the barcode decoder
    ./flash.sh imu_terrain      # the IMU
    ./flash.sh servo            # servo only, straight at the PWM block
    ./flash.sh scanning         # servo and sonar
    ./flash.sh detour           # drive, meet an obstacle, go round it
    ./flash.sh --wifi           # the car with the radio and MQTT
    ./flash.sh --wifi comms     # the comms bench, which needs the radio
    ./flash.sh --build-only motion

It builds first and refuses to flash a failed build. Needs `picotool`,
which installs with `brew install picotool`.

**The first flash of a Pico needs the BOOTSEL button.** Hold it down while
plugging in the USB cable, then run the script. After that `picotool`
reboots the board into the loader itself, because every profile here
carries the USB console.

To flash by hand, hold BOOTSEL while plugging in, wait for the `RPI-RP2`
volume, and copy the image onto it:

    cp firmware/build_make/mtk3pico_smp0_usb_cdc.uf2 /Volumes/RPI-RP2/

Or, on a board already running any build from this tree:

    picotool load -f -x firmware/build_make/mtk3pico_smp0_usb_cdc.uf2

`-f` reboots the running board into the loader and `-x` starts the program
after loading.

WARNING: iCloud leaves conflict copies beside the build output, named
`mtk3pico_smp0_usb_cdc 2.uf2` and so on. They are stale. Never copy one
onto `RPI-RP2`; delete them, the build recreates what it needs.

## Serial output

    ls /dev/cu.usbmodem*
    screen /dev/cu.usbmodemXXXX 115200       # quit: Ctrl-A, then K, then Y

Every bench waits 2 seconds before its first line, long enough to attach.
The car prints at boot, and USB serial drops anything sent before a
terminal connects, so open `screen` first and press the board's RESET
button to see the startup lines. The car logs every state change as
`state A -> B`, using these numbers:

| N | State | What it does | How it leaves |
|---|---|---|---|
| 0 | INIT | Nothing; calibration already ran before any task existed | Straight to 1 |
| 1 | FOLLOW_LINE | Gentle steering, crossing, creeping and turning on the spot to get back on the line after a detour or search, speed adapted for terrain | 4 obstacle, 2 barcode, 3 junction, 5 line lost, 7 hit or stuck |
| 2 | DECODE_BARCODE | One tick: take the command and hold it for the next junction | 1 |
| 3 | EXECUTE_TURN | At a junction: stops, waits for a command, creeps and turns, or goes straight on | 1 |
| 4 | AVOID_OBSTACLE | Scans, decides, drives the box, probes both sides | 1 if clear, 5 after a bypass, 6 if no lane |
| 5 | RECOVER_LINE | Search pattern, one step per tick | 1 when reacquired, 6 when exhausted |
| 6 | HALTED | `motion_stop()` every tick | Never; power cycle |
| 7 | COLLISION | Stops, settles, backs off, hands to 4. Entered on an IMU hit, or on `wheels driven but not turning, backing off` | 4 after backing off, 6 after three hits |

One halt arrives with no `state ->` line, because it is set from outside
the machine: `subsystem init failed, halting` at bring up.

## Benches

`firmware/run_host_tests.sh` needs no board. It compiles each module for
the Mac with fake hardware underneath and checks the maths and the
contracts. Run it after every edit and before every push.

A bench needs one subsystem's hardware and ignores anything else plugged
in. Every bench builds and flashes the same way:

```sh
cd firmware
./flash.sh <bench>          # build it and flash it
./flash.sh <bench> --build-only
screen $(ls /dev/cu.usbmodem* | head -1) 115200
```

**Off the cable.** Any bench, and the car, can run with the USB cable
unplugged. Build it with `--wifi` and every console line is also
published on `car/log`, so record the run from the laptop instead of
`screen`:

```sh
./flash.sh --wifi tuning    # flash it on the cable, then unplug
mosquitto_sub -h 192.168.50.131 -t car/log | tee tuning_run.txt
```

- Start `mosquitto_sub` before switching the car on. Nothing waits for the
  radio: lines printed before the car connects, or while the broker or
  Wi-Fi is away, are held (up to 8 KB) and sent in order once it is back,
  so an outage delays the log instead of cutting a hole in it. A line only
  reaches subscribers listening when it is sent, so keep `mosquitto_sub`
  running, or start it again, before the car reconnects.
- The battery must be on before the cable comes out, or the board loses
  power.
- The console still gets every line. With the cable out, the Pico keeps
  the first 4 KB and drops the rest; nothing blocks either way.
- The recorded file is the same text as the console, so the tables the
  benches print paste straight into the reports.

| Bench | Hardware | Power | Status |
|---|---|---|---|
| `run_host_tests.sh` | none | none | passing |
| car, nothing wired | Pico alone | USB | boots |
| `motion` | motors on M1, M2, both encoders | battery | working |
| `encoders` | both encoders, motors held off | battery | working |
| `duty` | motors and encoders, on the floor | battery | run 2026-09-26: `MOTOR_MIN_DUTY` 70 |
| `tuning` | motors and encoders, then the floor | battery | not yet run |
| `servo` | servo on S1 | battery | working |
| `scanning` | servo on S1, HC-SR04 on Grove 4 | battery | working |
| `detour` | motors, servo, HC-SR04 | battery | rerun: now drives the car's own legs; probe path not yet run |
| `imu_terrain` | GY-511 on Grove 3 | USB, battery to drive | tilt working; hump not yet run |
| `comms` | Pico W radio, a broker | USB | working end to end |
| `line_barcode` | 3 IR modules | USB | sensors working; barcode not yet run |

The motor, servo and sonar benches need the battery. The motor rail and
the servo header both run from it, and USB alone sags and resets the board
even when the console output looks fine.

### Per buddy, every command

Run these from `firmware/`.

| Buddy | Commands |
|---|---|
| all | `./run_host_tests.sh` before every push |
| 1 comms | `./flash.sh --wifi comms`, then `mosquitto_sub -h <broker> -t 'car/#' -v` and `mosquitto_pub -h <broker> -t car/command -m L` |
| 2 motion | `./flash.sh encoders`, `./flash.sh motion`, `./flash.sh duty` for `MOTOR_MIN_DUTY`, then `./flash.sh tuning` for `motion/tuning_report.md` |
| 3 line_barcode | `./flash.sh line_barcode` |
| 4 imu_terrain | `./flash.sh imu_terrain`, and again with `BENCH_DRIVE` 1 for `imu_terrain/terrain_report.md` |
| 5 scanning | `./flash.sh servo`, `./flash.sh scanning`, `./flash.sh detour` |
| team | `./flash.sh`, `./flash.sh --wifi` |

Two benches have a flag to set in the source first: `BENCH_DRIVE` in
`bench_imu_terrain.c` and `BENCH_FIND_CENTRE` in `bench_servo.c`.

### `encoders`

The motors stay off, so leave the battery on; the encoders may need it for
their supply. Mark a wheel, turn it exactly one revolution by hand in the
direction it rolls when driving forward, and read its column:
- **The count's change** is `ENCODER_SLOTS_PER_REV`.
- **The direction** must read `fwd`. If it reads `back`, flip that wheel's
  `ENCODER_*_B_FORWARD`.

A count that moves while nothing turns is electrical noise on that pin.

### `motion`

Wheels off the ground for the first run.
1. **Each motor alone.** If a wheel turns in one phase but not the other,
   the fault is that motor, its screw terminal, or that channel of the
   board. The board's M1A and M2A buttons tell you which.
2. **Both together,** printing each wheel's speed every 100 ms.
   - A negative speed while driving forward means that wheel's
     `ENCODER_*_B_FORWARD` is backwards.
   - A reading that jumps while the wheel sounds steady is encoder noise.
   - A smooth swing either side of the target is the speed loop hunting.
3. **A 500 mm drive,** printing the counts.

### `tuning`

Part 1 steps both wheels from rest to 100, 200 and 300 mm/s with the
wheels off the ground. Part 2 runs three 500 mm drives and three 90 degree
turns each way on the floor, with 8 seconds before each one to measure the
last and put the car back on its mark. Both parts print Markdown table
rows for `motion/tuning_report.md`, which also explains how to read them
and which knob each result changes.

### `servo`

Use this when the horn does not move. It drives the PWM block through
`car_hw` without calling `scan.c`, so our scanning code is not involved.
Three phases:
1. the centre, held for four seconds;
2. a one degree sweep across the band and back;
3. the signal off for two seconds, which lets a holding servo go slack.

Tape a pointer to the horn first. The horn points straight ahead at
1744 us (`SERVO_CENTRE_PULSE_USEC`), so the band runs 1469 to 2019 us for
65 to 115 degrees.

A servo reports nothing about where it is, so the first pulse throws the
horn from wherever it was left to the commanded centre. One big swing at
startup is normal. To find the centre of a remounted horn, set
`BENCH_FIND_CENTRE` to 1 at the top of `bench_servo.c`. The bench then
walks the pulse across a wide range, printing each step, and you read the
value off into `SERVO_CENTRE_PULSE_USEC`.

If the horn moves here but not in `scanning`, the fault is in `scan.c`. If
it moves in neither, check in this order:
1. the battery is connected and the board is on;
2. the three pin lead is on the right header and the right way round;
3. the horn is not jammed against its mount;
4. the servo itself, by swapping in a spare.

### `scanning`

Servo first, sonar second, so the two faults cannot be confused.
1. It parks the horn at each end and the centre, a second apart, and takes
   one reading at each.
2. It pings ten times without moving and counts the echoes.

A horn that moves with zero echoes is a sonar problem. No movement and no
echoes usually means the battery. The HC-SR04 runs from the Grove port's
3.3 V with no divider, which costs range but keeps the echo pin safe; this
bench measures what that range is.

### `detour`

Drives straight and goes round whatever it meets, with no line sensors. It
stops when something comes inside `SCAN_OBSTACLE_RANGE_MM`, prints what
each of the three scan angles sees and the decision, then drives the box
announcing each leg:

```
obstacle at 180 mm, stopping to look
   65 deg    812 mm  clear
   90 deg    180 mm  blocked
  115 deg    790 mm  clear
profile: valid 1 bearing 0 closest 180 width 61, clear left 790 right 812
detour sized for 61 mm wide: side 219 depth 386
decision: go RIGHT (action 3)
  leg 1/7  right 45 deg
  leg 2/7  forward 219 mm
```

The legs are the car's own, sized by `scan_detour_plan()` in the
scanning module from `CAR_WIDTH_MM` and `CAR_LENGTH_MM`, 150 and 200:
- **Sideways leg:** half the obstacle, plus half the car, plus 50 mm,
  divided by sin 45 for the diagonal.
- **Forward leg:** the obstacle's width plus the car's whole length,
  because the back wheels are still beside the obstacle when the bumper is
  past it.

A 100 mm obstacle gives side 247 and depth 425.

`SCAN_CLEARANCE_MIN_MM` is `CAR_WIDTH_MM` plus 100, so a gap under 250 mm
does not count as a lane. The three range lines are all the planner sees:
- **It always reverses:** both side readings are under
  `SCAN_CLEARANCE_MIN_MM`. The numbers show whether that is a wide
  obstacle or the sonar catching the floor.
- **A driving leg pings blocked part way round:** the lane is a dead end.
  The bench undoes the legs already driven, newest first, which returns
  the car to where the detour started, then tries the other side:

```
  leg 4/7 blocked at 140 mm, backing out
  backing out 3 leg(s)
  undo 3/7  right 45 deg
  undo 2/7  back 219 mm
  undo 1/7  left 45 deg
back at the start, trying left
```

Undoing turns the same amount the other way and drives the same distance
in reverse, so it is only as accurate as the odometry.
`CAR_MAX_DETOUR_ATTEMPTS` stops it after four tries. After a detour the
car drives off in whatever direction the box left it pointing, so give it
floor space.

### `imu_terrain`

Every 10 samples the bench prints one line with everything Buddy 4 owns:

```
cal 1 | pitch 3 ok 1 mag 1004 raw 1012 | hump 0 peak 0 mm | event STILL |
hit 0 | rate 0 dps | heading 187 | terrain STABLE rough 11
```

| Column | Meaning |
|---|---|
| `cal` | Calibrated at start-up |
| `pitch`, `ok`, `mag` | Tilt; whether it can be believed; the filtered vector length in milli g |
| `jolt` | The largest jolt since the last line: how far a raw sample sat from the filtered vector, in any direction. The only column a knock shows in |
| `hump`, `peak` | On a hump now; the run's highest hump in mm |
| `event` | Motion class |
| `hit` | Collision, latched and cleared when read, so it shows on one line only |
| `rate`, `heading` | Turn rate, magnetometer heading |
| `terrain`, `rough` | STABLE or ROUGH, and the roughness in milli g |

**Pitch.** An accelerometer cannot tell a tilt from a push, so pitch is
frozen whenever `mag` is more than `IMU_PITCH_TRUST_BAND_MILLI_G` from one
g, and `ok` shows 0.
- **Motor running, `mag` swinging, `ok 0`:** correct behaviour.
- **`ok 0` with the car standing still:** the mount passes too much
  vibration, or the band is too tight.

**Collision.** Tap the bumper harder and harder and watch `jolt`. Set
`IMU_COLLISION_THRESHOLD_MILLI_G`, 600 mg of jolt today, just above the
hardest knock that should not count. `mag` cannot show a knock, because
its filter takes 32 samples to respond. If `jolt` barely rises when you
knock the bumper hard, the mount is absorbing the impact.

**Hump height.** It needs the car moving, because height is sin(pitch)
integrated over ground distance.
1. With `BENCH_DRIVE` 0, tilt the car nose up by hand; pitch must read
   positive, else flip `IMU_PITCH_SIGN`.
2. Tilt it to about the hump's angle; pitch must pass
   `IMU_HUMP_PITCH_THRESHOLD_DEG`, 5 degrees.
3. Set `BENCH_DRIVE` to 1, battery on, and drive over the hump. Each time
   a hump ends, the bench prints `hump over: N mm, count C, run peak P mm`.
4. Compare with a ruler, and fill in section 4 of `terrain_report.md`.

Watch `ok` in step 3. If it drops to 0 on the climb, pitch froze and the
height is under-counted: widen `IMU_PITCH_TRUST_BAND_MILLI_G` until pitch
survives the climb, at the cost of letting more vibration through.

### `comms`

Needs the radio profile, `./flash.sh --wifi comms`; it refuses without it.
It shows the join, the MQTT connect and the command echo. Watch from a
second terminal:

```sh
mosquitto_sub -h <broker ip> -t 'car/#' -v
mosquitto_pub -h <broker ip> -t car/command -m L
```

The broker's own `-v` log prints protocol lines and byte counts, never
payloads, so use `mosquitto_sub` to see the contents. `python3
firmware/watch.py` shows the same messages one block each, every key named
with its unit and events marked `>>>`.

For a cleaner view, `python3 firmware/dashboard.py` opens a live page at
http://127.0.0.1:8765: the state, the last command and the link at the top,
every telemetry figure in one table, a timing diagram of the action, line
sensors, speed and pitch with events marked on it, the obstacle profile, the
terrain and hump table, the console log, and L, R, S, U buttons. A hit turns
the latest-event box red. Every message is kept in `firmware/runs/<date>.jsonl`
(not committed), and Export by date downloads a day as a zip of CSV files
for the reports. It needs no internet. Start the broker first; `watch.py`
stays as the terminal fallback and can run beside it. Point at the timing
diagram to read what the car was doing at that moment, which is how to read
an action too short to be labelled. When a bench is running instead of the
car, the page says which and shows its console across the middle.

### `line_barcode`

| Column | Meaning |
|---|---|
| `raw` | The electrical level on each pin before the dark level mapping |
| `changes` | Every flip per sensor since boot |
| `health` | CAPITAL per sensor once it has been seen both dark and light |

Wave a hand across one sensor at a time and watch `changes` climb; that
shows the sensor is wired, powered and detecting. A counter stuck at zero
means nothing reaches the pin.
- **`mask 111` with three CAPITALS:** a real junction.
- **`mask 111` with lower case letters:** sensors that have never changed.
- **`mask 000 health rcl` with nothing under the car:** the normal resting
  state.

## Known limits

- **The PID gains were set while the loops ran every 20 ms.** They now run
  every 10 ms, so re-tune them with `./flash.sh tuning` before trusting
  speed control; `motion/tuning_report.md` explains how.
- **The left encoder is noisy at speed.** It threw away 54 impossible
  edges on one step test against 1 on the right, and still shows speed
  spikes. Its extra counts make the straight line correction slow the left
  wheel, so the car curves left. Check its wiring (Pin map below).
- **The IMU has no gyroscope.** See `imu_terrain/terrain_report.md`,
  section 6.

## Bring up order

Prove each thing before the next depends on it. Keep the wheels off the
ground until line following behaves.

1. Board alone, nothing plugged in: `./flash.sh`, then watch the console
   for the startup lines, `motion ready` and `line ready`. Missing sensors
   log as absent rather than failing.
2. Motors on the battery. Press the board's M1A and M2A test buttons before
   running any code. Then `./flash.sh motion`, which drives each motor
   alone before both. Swap a terminal pair if a wheel runs backwards, and
   check M1 is the left motor.
3. `./flash.sh encoders`: one hand turn per wheel, 639 counts and `fwd`.
4. `./flash.sh imu_terrain`: tilt nose up, pitch positive.
5. `./flash.sh servo`: the horn must move. Everything below depends on it.
6. `./flash.sh scanning`: the horn moves in phase 1, echoes come back in
   phase 2. Then place a box at 300 mm for the sweep.
7. `./flash.sh line_barcode`: every health letter CAPITAL, then slide the
   car sideways across the line so the mask walks 001, 000, 100 (right,
   barcode, left) and the error reads -2, 0, 2.
8. `./flash.sh detour`: a box on the floor, off to one side. The printed
   ranges must match where the box is, the decision must pick the side
   with more room, and leg 1 must be a clear 45 degrees, not a spin.
9. `./flash.sh --wifi comms`: `connected 1`, then publish to
   `car/command` from a laptop and watch it echo.
10. The whole car on the floor, then `./flash.sh tuning` and the
    `BENCH_DRIVE` IMU run for the two reports.

## Pin map

Motor, servo and board pins come from the board maker's example code.
Grove port numbers and pairs come from the board documentation.

| Function                  | Pins            | Where                       |
|---------------------------|-----------------|-----------------------------|
| Motor left M1A, M1B       | GP8, GP9        | M1 terminal                 |
| Motor right M2A, M2B      | GP10, GP11      | M2 terminal                 |
| Scan servo                | GP12            | header S1                   |
| Console mirror, UART0     | GP0, GP1        | Grove 1, leave empty        |
| Line sensor right         | GP3             | Grove 2, yellow             |
| I2C0 SDA, SCL to IMU      | GP4, GP5        | Grove 3                     |
| Sonar trigger, echo       | GP16, GP17      | Grove 4                     |
| Line sensor left          | GP26            | Grove 5, yellow             |
| Barcode sensor            | GP27            | Grove 6, yellow             |
| Encoder right A, B        | GP7, GP28       | Grove 7, white and yellow   |
| Encoder left A, B         | GP19, GP6       | header, Pico pins 25 and 9  |
| Board: NeoPixels          | GP18            | leave alone                 |
| Board: buttons            | GP20, GP21      | spare inputs                |
| Board: buzzer             | GP22            | leave alone                 |
| Pico W radio              | GP23 to 25, 29  | not available               |

On every Grove port the **white** wire is the lower GPIO and the
**yellow** the higher. GP26 is both Grove 5's yellow (the left line
sensor) and Grove 6's white, and GP6 is both Grove 5's white and the left
encoder's B phase, so leave those two white wires unconnected at their
sensors. Which physical sensor is left, barcode and right depends on where
each module is mounted; swap the three `LINE_SENSOR_*_PIN` numbers to
match.

## Line sensors and encoders

Sensor placement, measured from the line's centre with the car centred on
it. The line is 18 mm wide and the barcode starts 20 mm beyond its edge,
29 mm from its centre:

| Sensor | Where its eye goes | Why |
|---|---|---|
| Left, Grove 5 | 12 to 14 mm left | Sees floor while centred, the line after a small drift, and 15 mm clear of the barcode |
| Right, Grove 2 | 12 to 14 mm right | The same |
| Barcode, Grove 6 | Over the barcode, more than 29 mm left, outside the left sensor | Must pass over every bar and never over the line |

Grove 1 is UART0, the kernel's serial console. Leave it empty: its white
wire is GP0, the console's transmit pin, so a sensor there can never read.

Each encoder brings both phases. A is counted, and B is sampled at every A
edge to tell which way the wheel turned. The left encoder goes on the pin
header beside the Pico, because no Grove port is free.

| Cable   | Red      | Black     | Yellow             | White          |
|---------|----------|-----------|--------------------|----------------|
| Grove 2 | IR VCC   | IR GND    | IR DO, right line sensor (GP3) | nothing (GP2) |
| Grove 5 | IR VCC   | IR GND    | IR DO, left line sensor (GP26) | nothing (GP6, the left encoder's B) |
| Grove 6 | IR VCC   | IR GND    | IR DO, barcode sensor (GP27) | nothing (GP26, the left line sensor) |
| Grove 7 | right motor white, supply | right motor blue, ground | right motor green, B phase (GP28) | right motor red, A phase (GP7) |

| Left motor wire | Header pin              |
|-----------------|-------------------------|
| white, supply   | 3V3 (OUT), Pico pin 36  |
| blue, ground    | GND, Pico pin 38        |
| red, A phase    | GP19, Pico pin 25       |
| green, B phase  | GP6, Pico pin 9         |

Pin numbers follow the Pico: USB end up, chip side towards you, pins 1 to
20 run down the left row and 21 to 40 up the right, so pin 9 is on the
other row from the rest. Pins 37 (3V3_EN), 39 (VSYS) and 40 (VBUS) sit
right beside the supply and ground pins. 3V3_EN to ground turns the
board's 3.3 V off, and VSYS or VBUS would feed 5 V into the encoder and
back into GP19 and GP6.

Before powering a replacement motor, check white is supply and blue is
ground against its label: a Hall sensor does not survive reversed supply.

To check the line sensors, wheels off the ground:

1. `./flash.sh line_barcode`, then open `screen`.
2. After two seconds you see `line bench: pins L26 B27 R3`, then a status
   line every 200 ms or so.
3. The mask prints right, barcode, left. Hold black tape under the Grove 2
   sensor only: the first digit must change and `error` must read 2. If
   the digit changes the wrong way, flip `LINE_SENSOR_DARK_LEVEL`.
4. Do the same under the Grove 5 sensor, last digit and error -2. Then
   the Grove 6 sensor: middle digit only, with `error` unchanged, because
   the barcode sensor never steers.
5. Build the car, flash, and confirm `motion ready` and `line ready` at
   boot. The bring up order above takes over from here.

## Resource use

Measured with `arm-none-eabi-size` on the current build. The Pico W has
2 MB of flash and 264 KB of RAM.

| Image | Flash | Static RAM |
|---|---|---|
| Car, `./flash.sh` | 66 KB, 3 % | 17 KB, 6 % |
| Car with Wi-Fi and MQTT, `./flash.sh --wifi` | 344 KB, 17 % | 65 KB, 25 % |
| Our own code, `app_program/`, within those | 16 KB | 1.3 KB |

Most of the Wi-Fi image is not ours: 225 KB of its flash is the radio
chip's firmware, and about 25 KB of its RAM is lwIP's packet buffers. On
top of the static RAM, the five car tasks take 4 KB of stack each, 20 KB
in all, from the kernel's memory pool at start-up. How much of each stack
is actually used has not been measured.

**Processor**
- One 10 ms kernel timer wakes the motion, line, IMU and mission tasks.
  Between ticks every task sleeps: nothing polls or waits in a loop.
- The only fast interrupt is the barcode sampler, every 500 us. It reads
  one pin and records a timestamp only when the level changes, into a
  32 entry ring; decoding runs later in the line task.
- Each encoder pulse records one timestamp in its interrupt; speed and
  distance are worked out in the motion task.
- No floating point anywhere. The RP2040's cores have no floating point
  unit, so it would run in software. Pitch and heading use an integer
  arctangent approximation, and hump height the small angle sine.

**Memory**
- No heap. Every buffer is a fixed size set at compile time, so the car
  cannot run out of memory mid-run.
- The largest buffers of ours are the log held for an outage, 8 KB, and
  the MQTT send ring, 8 messages of up to 480 bytes.

**Radio**

| Topic | Size | Rate | Data |
|---|---|---|---|
| `car/telemetry` | 297 bytes typical, 365 at most | every 5 s | about 0.06 KB/s |
| `car/terrain` | 124 bytes typical, 144 at most | every 5 s | under 0.03 KB/s |
| `car/heartbeat` | about 45 bytes | every 5 s | about 0.01 KB/s |
| `car/event` | 27 to 101 bytes typical, 117 at most | as it happens | a few a second at most, during a detour |
| `car/log` | whole console lines, up to 480 bytes a message | as printed | a few lines a second at most |

- Messages go at QoS 0 with no resends: a lost telemetry message is
  replaced by the next one 5 s later, so nothing out of date is sent
  twice. An event is held until it is queued, so one made while the
  broker is away is still sent.
- The control tasks never wait on the radio. A message is copied into
  the send ring and the radio task sends it.
- `CAR_TELEMETRY_PERIOD_MSEC` sets the largest share of the steady
  traffic. Events cost nothing while nothing happens.

**Power**
- A stopped motor gets no power at all.
- The sonar pings at most once every `CAR_SONAR_CHECK_PERIOD_MSEC`
  (60 ms) while following.
- The servo sweeps while driving (`SCAN_SWEEP_WHILE_MOVING`), which costs
  servo current; 0 keeps it still and looks straight ahead only.

## Kernel notes

- **Single core.** `SMP=1` exists, but the port documents I2C, ADC, PWM
  and UART as single owner resources under SMP, and nothing here needs the
  second core.
- **The tick** is `CNF_TIMER_PERIOD` in `firmware/config/config.h`, 10 ms.
  A task delay waits the ticks asked for plus one, so `tk_dly_tsk(10)`
  runs a loop every 20 ms. The motion, line, IMU and mission loops are
  woken by one cyclic handler every tick instead (`common/car_time.c`),
  and `car_main.c` refuses to build if a loop's period constant differs
  from the tick.
- **Resets.** The kernel releases only its own blocks from reset.
  `common/car_hw.c` releases PWM and TIMER on first use and sets the timer
  tick to one microsecond. Both are idempotent, so the order the modules
  initialise in does not matter.
- **Pico C SDK.** Every register access in our code is in
  `common/car_hw.c`, through the SDK's hardware API:
  - `hardware/gpio.h` (`gpio_get`, `gpio_put`, `gpio_set_dir`);
  - `hardware/pwm.h` and `hardware/resets.h`;
  - the register structs `io_bank0_hw`, `pads_bank0_hw`, `timer_hw` and
    `watchdog_hw`.

  Only header-only parts are used, so nothing of the SDK is linked for
  them. The kernel keeps interrupt registration (`tk_def_int`), time
  (`tk_get_otm`, in `common/car_time.c`), the I2C driver and the console.
  The Wi-Fi stack is the SDK's cyw43 driver and lwIP under the kernel's
  radio task.

  Some SDK parts stay unused on purpose: `gpio.c`, `irq.c`, `i2c.c` and
  the timer alarm functions. They bring the SDK's own interrupt layer,
  vector table and clock state (`clock_get_hz`), all of which the kernel
  owns. `car_hw.c` includes no kernel header, because the kernel and the
  SDK both define `size_t` and the block base addresses.
- **Interrupts** are registered with `tk_def_int()` and `EnableInt()`,
  never by writing the vector table. The encoders share NVIC line 13; the
  barcode sampler uses TIMER alarm 0.
- **Standard headers.** A file that includes `<tk/tkernel.h>` must not
  include `<stddef.h>`, `<stdio.h>`, `<string.h>` or `<stdlib.h>`: the
  kernel typedefs its own `size_t` and the two collide. Every module
  includes the kernel only outside `CAR_HOST_TEST`, and keeps its hardware
  calls in small static functions at the bottom of the file with a host
  fake beside them.
- **lwIP** runs `NO_SYS=1` on the radio task. The MQTT phase is the only
  lwIP caller the car adds, and application tasks reach it through ring
  buffers.
- **Edits to the vendored kernel,** and the only ones:
  - `build_make/pico_rp2040.mk` has a `pico_util` include path in two
    places for SDK 2.3, and the `WIFI_MQTT` profile.
  - `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/.gitkeep`
    exists because the USB console rule needs that directory with macOS
    make 3.81.
  - `lib/libnet/lwip/include/lwipopts.h` enables TCP and sizes the pools
    for MQTT.
  - `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` polls the MQTT phase
    once DHCP has an address.
  - `lib/libnet/lwip/lwip_utk_mqtt.*` is new.
  - `build_make/mtkernel_3/app_program/subdir.mk` is ours, including the
    SDK include paths for `car_hw.c`.
  - Only the `pico_rp2040` board is kept. The other boards, and the
    branches that selected them in `build_make/makefile`,
    `build_make/mtkernel_3/device/*/subdir.mk`, `include/sys/machine.h`
    and `config/config.h`, are removed; git history has them.
