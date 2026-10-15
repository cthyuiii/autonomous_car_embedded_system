# Autonomous Car Firmware

A line following car on a Raspberry Pi Pico W and a Cytron Robo Pico,
running micro T-Kernel 3.0. Five subsystems and one vehicle controller run
on the car's Pico. The other Picos are benches, one per person.

## Layout

`firmware/` is the Raspberry Pi Pico port from the course template
[sirfonzie/mtk3-hub](https://github.com/sirfonzie/mtk3-hub):
[sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040) at
commit 2e8e6ad, under its own licences (`firmware/LICENSE`,
`firmware/LICENSES/`). Our code lives in `firmware/app_program/`. Leave the
kernel tree alone apart from the edits listed under Kernel notes.

| Path                                | Owns                                   |
|-------------------------------------|----------------------------------------|
| `app_program/car_main.c`            | `usermain()`, the tasks, mission state |
| `app_program/common/`               | shared types, every config value, logging, the hardware layer, the clock |
| `app_program/comms/`                | MQTT telemetry, heartbeat, commands    |
| `app_program/motion/`               | motors, encoders, PID, distance, turns, steering |
| `app_program/line_barcode/`         | three IR sensors, position, junctions, Code 39 barcodes |
| `app_program/imu_terrain/`          | LSM303DLHC, tilt, humps, events, collision |
| `app_program/scanning/`             | servo, HC-SR04, profiling, avoidance, recovery |
| `build_make/mtkernel_3/app_program/`| `subdir.mk`, how our code is built     |
| `flash.sh`                          | build one image and flash it           |
| `run_host_tests.sh`                 | every host test, no board needed       |
| `lib/libnet/lwip/lwip_utk_mqtt.*`   | the MQTT phase on the kernel's radio task |
| `dashboard.py`, `watch.py`          | the live dashboard and the terminal viewer |

Each subsystem folder has its public header in `include/`, the code, a host
test, a bench and a README with its calibration table. The two reports sit
with their code: `motion/tuning_report.md` (PID tuning and motion accuracy)
and `imu_terrain/terrain_report.md` (terrain analysis).

## Who edits what

| Owner   | Files                                                     |
|---------|-----------------------------------------------------------|
| Buddy 1 | `app_program/comms/`, `lib/libnet/lwip/lwip_utk_mqtt.*`, the `COMMS_*` values |
| Buddy 2 | `app_program/motion/`, the `MOTOR_*`, `ENCODER_*`, `MOTION_*` values |
| Buddy 3 | `app_program/line_barcode/`, the `LINE_*`, `TRACK_*`, `BARCODE_*` values |
| Buddy 4 | `app_program/imu_terrain/`, the `IMU_*` values             |
| Buddy 5 | `app_program/scanning/`, the `SONAR_*`, `SERVO_*`, `SCAN_*` values |
| Team    | `car_main.c`, `common/`, `CAR_*`, `subdir.mk`             |
| Nobody  | Everything else under `firmware/`, which is the kernel    |

Each file names its owner in its header. Change a header only with the
team's agreement. Add to host tests, never remove from them. Record
measurements in your README's table.

The code follows the Barr C coding standard. Public functions carry their
module's name: `line_*`, `imu_*`, `scan_*` and so on.

## Status

| Capability | On the car |
|---|---|
| Motors: forward, reverse, turn on the spot | Working |
| Encoders and speed control | Working: 639 pulses and 205 mm per wheel turn, left tyre 993 and right 1007 per mille of that |
| Straight-line correction on distance moves | Working: 500 mm drives land at 500 to 515 mm, straight |
| Line following, two sensors straddling the line | Working, speed-ups ramped at 200 mm/s² |
| Back onto the line after a detour or search | Working up to 90 degrees |
| Stop at a junction and turn as the barcode says | Working |
| Barcode decoding, Code 39 A, B, C and D | Working: all four on the bench, A on the course. A missed barcode is reread by backing up |
| Tilt from the IMU | Working |
| Hump detection, count and heights | Built and host tested |
| Control loops every 10 ms on the kernel tick | Working, 9 ms measured |
| Telemetry, events, heartbeat and remote commands over MQTT | Working, shown on the dashboard |
| Collision from the IMU | Working: hits reported in any state and acted on |
| Stuck wheel detection | Built |
| Servo and ultrasonic ranging at 3.3 V | Working |
| Obstacle profile and detour | Working: a 220 by 195 mm box cleared on the bench by walking its length |
| Line search after a bypass | Built |
| Hardware layer on the Pico C SDK (`common/car_hw.c`) | Working |

## How the car behaves

- **motion** drives the board's H-bridge over PWM. Each wheel has a speed
  loop, feedforward plus PID, on the speed measured between encoder
  pulses. Phase B gives the direction, and a reversal counts only once 4
  pulses in a row agree. Distance moves hold their heading
  (`MOTION_STRAIGHT_KP`). Turns on the spot run at
  `MOTION_TURN_SPEED_MM_PER_SEC`. Speed-ups climb at
  `MOTION_ACCEL_MM_PER_SEC2`, 0 to 150 mm/s in 0.75 s, and the slower wheel
  keeps its share, so the car steers from rest. Slowing and stopping are
  immediate. Each wheel has its own share of the circumference
  (`WHEEL_LEFT_PERMILLE`, `WHEEL_RIGHT_PERMILLE`), so the car runs straight
  on the ground, not on pulse counts. Both encoders are required: a wheel
  whose encoder never pulses reads zero speed and is driven hard.
- **line_barcode** reads three IR modules as a 3 bit mask. Left (Grove 5)
  and right (Grove 2) straddle the line and steer. The third (Grove 6) sits
  to the left over the barcodes, sampled by a timer interrupt every
  500 µs. At start-up every sensor must read floor, and the console names
  any that do not.
- **imu_terrain** runs the LSM303DLHC on Grove 3. Pitch is relative to a
  level reference taken at boot and is frozen while the car shakes or
  accelerates. `terrain_report.md` explains each quantity.
- **scanning** runs the HC-SR04 at 3.3 V on Grove 4 and the servo on S1,
  across 65 to 115 degrees.
- **comms** publishes JSON over MQTT from the kernel's radio task, in the
  `WIFI_MQTT=1` build only. Telemetry (`car/telemetry`, with the current
  `action`), terrain (`car/terrain`) and a heartbeat go out every 5 s.
  Hits, humps, barcodes, obstacle scans and new actions go out at once on
  `car/event`. `comms/README.md` lists every key. Without the radio the car
  logs it as absent and drives anyway.
- **car_main** calibrates, then follows the line, stops at every junction,
  goes round obstacles and searches for the line afterwards. In the radio
  build it waits until the broker has its first telemetry, up to
  `CAR_BROKER_WAIT_MSEC` (60 s), so the whole run is reported. Start the
  broker and the dashboard before switching the car on.

**Barcodes and junctions.** A means left, B right, C straight and D U-turn
(`BARCODE_CHAR_*`). The line is 18 mm wide. Each barcode is 141 mm long and
sits beside the line on the left, 20 mm from its edge. Junctions are
crosses. A junction counts when each line sensor has been dark within the
last `LINE_JUNCTION_WINDOW_MSEC` (100 ms), so a car meeting the cross at an
angle still sees it. The last barcode read is held until the next cross.

- **At the junction:** the car stops for `CAR_JUNCTION_WAIT_MSEC`, taking
  any command that arrives.
- **A or B:** it creeps `CAR_JUNCTION_CREEP_MM` so its wheels sit over the
  crossing, then turns until the line sensor on that side has crossed the
  new branch.
- **D:** it turns about 180 degrees where it stands, ending the same way.
- **C, or no barcode:** it goes straight on.
- **Nothing read:** every junction has a barcode just before it, so the car
  backs up at 60 mm/s, reading, until it reads the barcode or passes its
  bars and 50 mm of plain floor, 600 mm at most (`CAR_REREAD_MAX_MM`). It
  then follows the line back to the junction, once per junction.

No barcode may overlap a cross: one still under the sensor when the car
stops is lost.

To test barcodes and junctions alone, set `CAR_BARCODE_TEST` to 1 in
`car_config.h`. The sonar is then not used. Set it back to 0 for runs. The
console reports it at start-up.

**Obstacles.** One sonar reading every `CAR_SONAR_CHECK_PERIOD_MSEC` is
shared by every state, so turns, detour legs and the line search all stop
for something in the path. While driving, the sonar sweeps centre, right,
centre, left (`SCAN_SWEEP_WHILE_MOVING`), so straight ahead is ranged every
other reading. If obstacles are seen too late, raise
`SCAN_OBSTACLE_RANGE_MM`, drop the speed, or set the flag to 0.

**The detour** may leave the line, and every way out of it ends in
`RECOVER_LINE`.
- **Walking the length:** the sonar sees only the obstacle's front, so
  beside it the car steps `SCAN_DETOUR_STEP_MM` (50 mm) and turns
  `SCAN_DETOUR_LOOK_DEG` (65°) toward it to look. While something is nearer
  than the line it steps on. Once clear it drives `SCAN_DETOUR_PAST_MM`
  (100 mm) more and turns in. The sideways leg leaves room for the nose to
  swing (`SCAN_NOSE_SWING_MM`).
- **Ending early:** it ends when the line reappears after leg
  `CAR_DETOUR_PAST_INDEX`. A leg that pings blocked is abandoned and
  replanned.
- **Giving up:** after `CAR_MAX_DETOUR_ATTEMPTS` tries the car reverses
  instead. The count clears after `CAR_DETOUR_CLEAR_MM` of clear following.

**Getting back on the line.** After a detour or search the car meets the
line at an angle, where steering would only turn it further across. So it:
1. drives straight on until the second sensor touches the line,
2. creeps until its wheels are over the line, a distance worked out from
   how far apart the two touches were,
3. turns on the spot toward the second sensor until that sensor touches
   the line, and normal steering finishes the turn.

Below about 6 degrees it goes straight to normal steering.

**Collisions.** A hit outranks every state and can interrupt any move. The
car then:
1. stops for `CAR_COLLISION_SETTLE_MSEC`,
2. backs straight off `CAR_COLLISION_BACKOFF_MM` the way it came, the one
   direction known to be clear, or forward if it was reversing,
3. hands over to avoidance to scan.

On the fourth hit in one run it halts. A wheel that is driven but has not
turned for `MOTION_STALL_FAULT_MSEC` counts as a hit, which catches
obstacles in the sonar's blind spot.

**Terrain speed.** Climbing keeps the speed and the speed loop adds power.
Descending or rough ground slows the car to `CAR_DESCEND_SPEED_MM_PER_SEC`.

## Build

Needs `arm-none-eabi-gcc`, GNU make, a host C++ compiler for the UF2 tool
on the first build, and `PICO_SDK_PATH`. The VS Code Pico extension
installs the SDK under `~/.pico-sdk/sdk/`.

    export PICO_SDK_PATH=~/.pico-sdk/sdk/2.3.0
    cd firmware/build_make
    make CONSOLE=usb_cdc -j8                 # the car, no radio
    make CONSOLE=usb_cdc BENCH=motion -j8    # one bench

`firmware/flash.sh` does both and flashes too. Use it day to day.

For the radio and MQTT:
1. Copy `firmware/config/wifi_credentials.example.h` to
   `wifi_credentials.h` and fill it in. Git ignores the copy.
2. Set `COMMS_MQTT_BROKER_HOST` in `car_config.h` to the laptop's address.
3. Start the broker in its own terminal and leave it open. Save these two
   lines as `~/mosquitto-car.conf`, since Mosquitto 2 otherwise listens
   only to the laptop and the Pico W has no login:

       listener 1883 0.0.0.0
       allow_anonymous true

   then run `mosquitto -c ~/mosquitto-car.conf -v`. Do not use `brew
   services`, which reads a different config file.
4. Build:

       make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
            WIFI_MQTT=1 -j8

`WIFI_MQTT` needs only DHCP, so none of the port's DNS, UDP or TCP test
phases are built. The image name gains a `_mqtt` suffix. Changing any
profile flag rebuilds everything.

Switching between the car and a bench relinks by itself: `subdir.mk` keeps
a stamp of what the image was built from.

WARNING: macOS ships GNU Make 3.81, which compares whole seconds. If a
script edits `car_config.h` in the same second a build finished, the next
build can reuse stale objects. After scripted edits, delete
`build_make/mtkernel_3/app_program/**/*.o` first.

WARNING: Do not run `make -n` for a different profile or bench than the
last build. The build stamps delete objects and images while the makefile
is read, even under `-n`.

## Flash

Every profile links to the same file name, so build and flash in one step:

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

It refuses to flash a failed build. It needs `picotool` (`brew install
picotool`).

**Every flash needs the BOOTSEL button.** Hold it while plugging in the USB
cable, then run the script.

To flash by hand, hold BOOTSEL while plugging in, wait for the `RPI-RP2`
drive and copy the image onto it:

    cp firmware/build_make/mtk3pico_smp0_usb_cdc.uf2 /Volumes/RPI-RP2/

WARNING: iCloud leaves conflict copies beside the build output, named
`mtk3pico_smp0_usb_cdc 2.uf2` and so on. They are stale. Delete them and
never copy one onto `RPI-RP2`.

## Serial output

    ls /dev/cu.usbmodem*
    screen /dev/cu.usbmodemXXXX 115200       # quit: Ctrl-A, then K, then Y

Every bench waits 2 seconds before its first line, so there is time to
attach. USB serial drops anything sent before a terminal connects, so open
`screen` first and press RESET to see the start-up lines. The car logs each
state change as `state A -> B`:

| N | State | What it does | How it leaves |
|---|---|---|---|
| 0 | INIT | Nothing, calibration ran before the tasks started | Straight to 1 |
| 1 | FOLLOW_LINE | Steers on the line, gets back onto it after a detour or search, adapts speed to the terrain | 4 obstacle, 2 barcode, 3 junction, 5 line lost, 7 hit or stuck |
| 2 | DECODE_BARCODE | One tick: holds the command for the next junction | 1 |
| 3 | EXECUTE_TURN | At a junction: stops, waits for a command, rereads a missed barcode, creeps and turns, or goes straight on | 1 |
| 4 | AVOID_OBSTACLE | Scans, decides, drives the detour, probes both sides | 1 if clear, 5 after a bypass, 6 if no lane |
| 5 | RECOVER_LINE | Search pattern, one step per tick | 1 when found, 6 when exhausted |
| 6 | HALTED | `motion_stop()` every tick | Never, power cycle |
| 7 | COLLISION | Stops, settles, backs off, hands to 4. Entered on an IMU hit or a stuck wheel | 4 after backing off, 6 on the fourth hit |

A failed bring-up halts with `subsystem init failed, halting` and no
`state ->` line.

## Dashboard

    python3 firmware/dashboard.py

Opens http://127.0.0.1:8765, or the next free port. It shows the state, the
last command and the link, every telemetry figure, a timing diagram of the
action, line sensors, speed and pitch with events marked, the obstacle
profile, the terrain table, the console, and L, R, S and U buttons. A hit
turns the latest-event box red. Every message is kept in
`firmware/runs/<date>.jsonl` (not committed), and Export by date downloads
a day as CSV files. It needs no internet. Start the broker first.

`python3 firmware/watch.py` shows the same messages in the terminal, one
block each, with events marked `>>>`.

## Benches

`firmware/run_host_tests.sh` needs no board. It compiles each module for
the Mac on fake hardware and checks the maths and the contracts. Run it
before every push.

A bench uses one subsystem's hardware and ignores anything else plugged in:

```sh
cd firmware
./flash.sh <bench>
screen $(ls /dev/cu.usbmodem* | head -1) 115200
```

**Off the cable.** Build any bench, or the car, with `--wifi` and every
console line is also published on `car/log`:

```sh
./flash.sh --wifi tuning    # flash it on the cable, then unplug
mosquitto_sub -h 192.168.50.131 -t car/log | tee tuning_run.txt
```

- Benches built with `--wifi` wait for the broker before they start, so
  nothing is missed.
- Lines printed while the broker or WiFi is away are held, up to 8 KB, and
  sent in order once it is back.
- Switch the battery on before pulling the cable, or the board loses
  power.
- The recorded text matches the console, so bench tables paste straight
  into the reports.

| Bench | Hardware | Power | Status |
|---|---|---|---|
| `run_host_tests.sh` | none | none | passing |
| car, nothing wired | Pico alone | USB | boots |
| `motion` | motors on M1, M2, both encoders | battery | working |
| `encoders` | both encoders, motors held off | battery | working |
| `duty` | motors and encoders, on the floor | battery | working: `MOTOR_MIN_DUTY` 70 |
| `tuning` | motors and encoders, then the floor | battery | working |
| `servo` | servo on S1 | battery | working |
| `scanning` | servo on S1, HC-SR04 on Grove 4 | battery | working |
| `detour` | motors, servo, HC-SR04 | battery | working |
| `imu_terrain` | GY-511 on Grove 3 | USB, battery to drive | working |
| `comms` | Pico W radio, a broker | USB | working |
| `line_barcode` | 3 IR modules | USB | working |

The motor, servo and sonar benches need the battery. USB alone sags and
resets the board even when the console looks fine.

### Per buddy

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

The motors stay off, but keep the battery on for the encoder supply. Turn
a marked wheel exactly once by hand, the way it rolls forward:
- **The count** must change by `ENCODER_SLOTS_PER_REV`.
- **The direction** must read `fwd`. If it reads `back`, flip that wheel's
  `ENCODER_*_B_FORWARD`.

A count that moves while nothing turns is electrical noise on that pin.

### `motion`

Wheels off the ground for the first run.
1. **Each motor alone.** If a wheel turns one way but not the other, the
   fault is that motor, its terminal or that board channel. The board's
   M1A and M2A buttons tell which.
2. **Both together,** each wheel's speed every 100 ms. A negative speed
   going forward means that wheel's `ENCODER_*_B_FORWARD` is backwards. A
   reading that jumps while the wheel sounds steady is encoder noise. A
   smooth swing about the target is the speed loop hunting.
3. **A 500 mm drive,** with the counts.

### `tuning`

Part 1 steps both wheels from rest to 100, 200 and 300 mm/s, wheels off
the ground. Part 2 runs three 500 mm drives and three 90 degree turns each
way on the floor, with 15 seconds before each to measure the last one and
put the car back on its mark. Both print table rows for
`motion/tuning_report.md`, which explains how to read them.

### `servo`

Use this when the horn does not move. It drives the PWM block without
`scan.c`. Three phases:
1. the centre, held for four seconds,
2. a one degree sweep across the band and back,
3. the signal off for two seconds, so the servo goes slack.

The horn points straight ahead at 1744 µs (`SERVO_CENTRE_PULSE_USEC`), so
the band runs 1469 to 2019 µs for 65 to 115 degrees. One big swing at
start-up is normal: a servo has no position feedback. To find the centre
of a remounted horn, set `BENCH_FIND_CENTRE` to 1 in `bench_servo.c` and
read the value off the printed steps.

If the horn moves here but not in `scanning`, the fault is in `scan.c`. If
it moves in neither, check:
1. the battery is on,
2. the lead is on the right header, the right way round,
3. the horn is not jammed,
4. the servo, by swapping in a spare.

### `scanning`

Servo first, sonar second, so the two faults cannot be confused.
1. It parks the horn at each end and the centre and takes a reading at
   each.
2. It pings ten times without moving and counts the echoes.

A horn that moves with no echoes is a sonar problem. No movement and no
echoes usually means the battery.

### `detour`

Drives straight and goes round whatever it meets, with no line sensors.
It stops inside `SCAN_OBSTACLE_RANGE_MM`, prints the three scan angles and
the decision, then drives the detour leg by leg:

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

The legs are the car's own, from `scan_detour_plan()`. Beside the obstacle
it walks the length with a look every 50 mm and prints each look. A gap
under `SCAN_CLEARANCE_MIN_MM` (250 mm) is not a lane:
- **It always reverses:** both side readings are under 250 mm. The numbers
  show whether that is a wide obstacle or the sonar catching the floor.
- **A leg pings blocked:** the lane is a dead end. The bench undoes the
  legs already driven, newest first, and tries the other side:

```
  leg 4/7 blocked at 140 mm, backing out
  backing out 3 leg(s)
  undo 3/7  right 45 deg
  undo 2/7  back 219 mm
  undo 1/7  left 45 deg
back at the start, trying left
```

It stops after `CAR_MAX_DETOUR_ATTEMPTS` tries. Give it floor space.

### `imu_terrain`

Every 20 samples, 200 ms, the bench prints one line:

```
cal 1 | pitch 3 ok 1 mag 1004 jolt 12 | hump 0 peak 0 mm | event STILL |
hit 0 | rate 0 dps | heading 187 | terrain STABLE rough 11
```

| Column | Meaning |
|---|---|
| `cal` | Calibrated at start-up |
| `pitch`, `ok`, `mag` | Tilt, whether it can be believed, the filtered vector length in milli g |
| `jolt` | The largest jolt since the last line, in any direction. The only column a knock shows in |
| `hump`, `peak` | On a hump now, and the run's highest hump in mm |
| `event` | Motion class |
| `hit` | Collision, shown on one line only |
| `rate`, `heading` | Turn rate and magnetometer heading |
| `terrain`, `rough` | STABLE or ROUGH, and the roughness in milli g |

**Pitch** freezes, `ok 0`, whenever `mag` is more than
`IMU_PITCH_TRUST_BAND_MILLI_G` from 1 g. That is correct with the motor
running. `ok 0` with the car standing still means the mount passes too
much vibration.

**Collision.** Tap the bumper harder each time and watch `jolt`. Set
`IMU_COLLISION_THRESHOLD_MILLI_G` (600 mg) just above the hardest knock
that should not count.

**Hump height** needs the car moving, as height is sin(pitch) over ground
distance.
1. With `BENCH_DRIVE` 0, tilt the nose up. Pitch must read positive, or
   flip `IMU_PITCH_SIGN`.
2. Tilt it to the hump's angle. Pitch must pass
   `IMU_HUMP_PITCH_THRESHOLD_DEG` (5 degrees).
3. Set `BENCH_DRIVE` to 1, battery on, and drive over the hump. Each hump
   prints `hump over: N mm, count C, run peak P mm`.
4. Compare with a ruler in section 4 of `terrain_report.md`.

If `ok` drops to 0 on the climb, widen `IMU_PITCH_TRUST_BAND_MILLI_G`.

### `comms`

Needs the radio: `./flash.sh --wifi comms`. It shows the join, the MQTT
connect and the command echo. Watch from a second terminal:

```sh
mosquitto_sub -h <broker ip> -t 'car/#' -v
mosquitto_pub -h <broker ip> -t car/command -m L
```

The broker's own `-v` log shows byte counts, not payloads.

### `line_barcode`

| Column | Meaning |
|---|---|
| `raw` | The level on each pin |
| `changes` | Every flip per sensor since boot |
| `health` | CAPITAL per sensor once it has been seen both dark and light |

Wave a hand across one sensor at a time and watch `changes` climb. A
counter stuck at zero means nothing reaches the pin.
- **`mask 111` with three CAPITALS:** a real junction.
- **`mask 111` with lower case letters:** sensors that never changed.
- **`mask 000 health rcl`, nothing under the car:** normal at rest.

## Known limits

- **The left encoder is noisy above 200 mm/s.** The car never drives that
  fast.
- **The IMU has no gyroscope.** See section 6 of
  `imu_terrain/terrain_report.md`.

## Bring-up order

Prove each part before the next depends on it. Keep the wheels off the
ground until line following behaves.

1. Board alone: `./flash.sh`, then look for `motion ready` and
   `line ready`. Missing sensors log as absent.
2. Motors on the battery. Press the M1A and M2A buttons first, then
   `./flash.sh motion`. Swap a terminal pair if a wheel runs backwards, and
   check M1 is the left motor.
3. `./flash.sh encoders`: one hand turn per wheel, 639 counts and `fwd`.
4. `./flash.sh imu_terrain`: nose up, pitch positive.
5. `./flash.sh servo`: the horn must move.
6. `./flash.sh scanning`: the horn moves, then echoes come back. Then a
   box at 300 mm for the sweep.
7. `./flash.sh line_barcode`: every health letter CAPITAL. Slide the car
   across the line: the mask walks 001, 000, 100 (right, barcode, left)
   and the error reads -2, 0, 2.
8. `./flash.sh detour`: a box to one side. The ranges must match where the
   box is, and the decision must pick the side with more room.
9. `./flash.sh --wifi comms`: `connected 1`, then publish to
   `car/command` and watch it echo.
10. The whole car on the floor, then `./flash.sh tuning` and the
    `BENCH_DRIVE` IMU run for the two reports.

## Pin map

Motor, servo and board pins come from the board maker's example code.

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
**yellow** the higher. GP26 is both Grove 5 yellow (the left line sensor)
and Grove 6 white. GP6 is both Grove 5 white and the left encoder's B
phase. Leave those two white wires unconnected at their sensors. To swap
which module is left, barcode or right, swap the three
`LINE_SENSOR_*_PIN` numbers.

## Line sensors and encoders

Sensor placement, measured from the line's centre with the car centred on
it. The line is 18 mm wide and the barcode starts 20 mm beyond its edge,
29 mm from its centre:

| Sensor | Where its eye goes | Why |
|---|---|---|
| Left, Grove 5 | 17.5 mm left, eyes 35 mm apart | Sees floor while centred, the line after 8.5 mm of drift, and stays 11.5 mm clear of the barcode |
| Right, Grove 2 | 17.5 mm right | The same, without the barcode |
| Barcode, Grove 6 | Over the barcode, more than 29 mm left | Passes over every bar and never over the line |

Grove 1 is the kernel's serial console. Leave it empty: its white wire is
GP0, the console's transmit pin.

Each encoder brings both phases. A is counted and B is read at every A
edge for the direction. The left encoder goes on the pin header, because
no Grove port is free.

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

Pin numbers follow the Pico with the USB end up: pins 1 to 20 run down
the left row and 21 to 40 up the right. Keep clear of pins 37 (3V3_EN),
39 (VSYS) and 40 (VBUS) beside the supply pins. 3V3_EN to ground turns the
3.3 V off, and VSYS or VBUS would feed 5 V into the encoder.

Before powering a replacement motor, check white is supply and blue is
ground against its label. A Hall sensor does not survive reversed supply.

To check the line sensors, wheels off the ground:
1. `./flash.sh line_barcode`, then open `screen`.
2. After two seconds: `line bench: pins L26 B27 R3`, then a status line
   every 200 ms or so.
3. The mask prints right, barcode, left. Black tape under the Grove 2
   sensor must change the first digit and read `error` 2. If the digit
   changes the wrong way, flip `LINE_SENSOR_DARK_LEVEL`.
4. Under the Grove 5 sensor: the last digit, error -2. Under the Grove 6
   sensor: the middle digit only, error unchanged.

## Resource use

Measured with `arm-none-eabi-size`. The Pico W has 2 MB of flash and
264 KB of RAM.

| Image | Flash | Static RAM |
|---|---|---|
| Car, `./flash.sh` | 70 KB, 3 % | 17 KB, 6 % |
| Car with WiFi and MQTT, `./flash.sh --wifi` | 348 KB, 17 % | 65 KB, 25 % |
| Our code, `app_program/`, within those | 22 KB | 1.8 KB |

Most of the WiFi image is not ours: 220 KB of flash is the radio chip's
firmware and about 25 KB of RAM is lwIP's packet buffers. The five car
tasks also take 4 KB of stack each from the kernel's memory pool.

**Processor**
- One 10 ms kernel timer wakes the motion, line, IMU and mission tasks.
  Between ticks every task sleeps.
- The only fast interrupt is the barcode sampler, every 500 µs. It records
  a timestamp only when the level changes, into a 32 entry ring, and the
  line task decodes later.
- Each encoder pulse records one timestamp. The motion task works out
  speed and distance.
- No floating point. The RP2040 has no floating point unit. Pitch and
  heading use an integer arctangent, and hump height the small angle sine.

**Memory**
- No heap. Every buffer has a fixed size, so the car cannot run out of
  memory mid-run.
- Our largest buffers are the 8 KB log held during an outage and the MQTT
  send ring of 8 messages of up to 480 bytes.

**Radio**

| Topic | Size | Rate | Data |
|---|---|---|---|
| `car/telemetry` | 297 bytes typical, 365 at most | every 5 s | about 0.06 KB/s |
| `car/terrain` | 124 bytes typical, 144 at most | every 5 s | under 0.03 KB/s |
| `car/heartbeat` | about 45 bytes | every 5 s | about 0.01 KB/s |
| `car/event` | 27 to 101 bytes typical, 117 at most | as it happens | a few a second at most, during a detour |
| `car/log` | whole console lines, up to 480 bytes a message | as printed | a few lines a second at most |

- QoS 0 with no resends. A lost telemetry message is replaced 5 s later.
  An event made while the broker is away waits until it is queued.
- The control tasks never wait on the radio. A message is copied into the
  send ring and the radio task sends it.

**Power**
- A stopped motor gets no power.
- The sonar pings at most every 60 ms while following.
- The servo sweeps while driving (`SCAN_SWEEP_WHILE_MOVING`). 0 keeps it
  still.

## Kernel notes

- **Single core.** `SMP=1` exists, but the port treats I2C, ADC, PWM and
  UART as single owner resources under SMP, and nothing here needs the
  second core.
- **The tick** is `CNF_TIMER_PERIOD` in `firmware/config/config.h`, 10 ms.
  A task delay waits one tick longer than asked, so `tk_dly_tsk(10)` runs
  a loop every 20 ms. The control loops are woken by one cyclic handler
  every tick instead (`common/car_time.c`), and `car_main.c` refuses to
  build if a loop's period differs from the tick.
- **Resets.** The kernel releases only its own blocks from reset.
  `common/car_hw.c` releases PWM and TIMER on first use and sets the timer
  tick to one microsecond.
- **Pico C SDK.** Every register access in our code is in
  `common/car_hw.c`, through the SDK's hardware headers: `hardware/gpio.h`,
  `hardware/pwm.h`, `hardware/resets.h` and the register structs
  `io_bank0_hw`, `pads_bank0_hw`, `timer_hw` and `watchdog_hw`. The kernel
  keeps interrupts (`tk_def_int`), time (`tk_get_otm`), the I2C driver and
  the console. WiFi is the SDK's cyw43 driver and lwIP under the kernel's
  radio task. The SDK's `gpio.c`, `irq.c`, `i2c.c` and timer alarm
  functions stay unused, because they bring their own interrupt layer and
  clock state, which the kernel owns. `car_hw.c` includes no kernel
  header, as both define `size_t` and the block base addresses.
- **Interrupts** are registered with `tk_def_int()` and `EnableInt()`. The
  encoders share NVIC line 13. The barcode sampler uses TIMER alarm 0.
- **Standard headers.** A file that includes `<tk/tkernel.h>` must not
  include `<stddef.h>`, `<stdio.h>`, `<string.h>` or `<stdlib.h>`, as both
  define `size_t`. Each module includes the kernel only outside
  `CAR_HOST_TEST`, with its hardware calls in small functions at the
  bottom of the file and a host fake beside them.
- **lwIP** runs `NO_SYS=1` on the radio task. The MQTT phase is the only
  lwIP caller we add, and tasks reach it through ring buffers.
- **Edits to the vendored kernel,** and the only ones:
  - `build_make/pico_rp2040.mk`: a `pico_util` include path for SDK 2.3,
    and the `WIFI_MQTT` profile.
  - `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/.gitkeep`:
    the USB console rule needs that directory with macOS make 3.81.
  - `lib/libnet/lwip/include/lwipopts.h`: TCP on and the pools sized for
    MQTT.
  - `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`: rejoins WiFi and polls
    the MQTT phase once DHCP has an address.
  - `lib/libnet/lwip/lwip_utk_mqtt.*`: new.
  - `build_make/mtkernel_3/app_program/subdir.mk`: ours, including the SDK
    include paths for `car_hw.c`.
  - Only the `pico_rp2040` board is kept. The other boards, and the
    branches that selected them, are removed. Git history has them.
