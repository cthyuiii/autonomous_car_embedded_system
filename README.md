# Firmware

Autonomous line following car on a Raspberry Pi Pico W carried by a Cytron
Maker Pi Pico base. Five subsystems, one vehicle controller, one Pico on the
car. The other Picos are development benches, one per person.

## Layout

| Path            | Owns                                               |
|-----------------|----------------------------------------------------|
| `common/`       | shared types, every config knob, logging           |
| `comms/`        | WiFi, MQTT, telemetry, heartbeat, remote commands  |
| `motion/`       | motors, encoders, PID, distance and turns          |
| `line_barcode/` | three IR sensors, line position, barcode decoding  |
| `imu_terrain/`  | LSM303DLHC, tilt, humps, motion events, collision  |
| `scanning/`     | servo, HC-SR04, obstacle profiling, avoidance      |
| `app/`          | the vehicle controller and mission state machine   |

## Every file and its status

Three words are used below. **Finished** means there is nothing to write.
**Stub** means the signature, doc comment and input guards are real but the
algorithm returns `CAR_ERR_NOT_IMPLEMENTED` with a `TODO` naming what to
write. **Skeleton** means the structure runs but the decisions are `TODO`.

### Top level

- `CMakeLists.txt`, finished. Imports the SDK, sets `PICO_BOARD=pico_w`,
  builds `car_common` and the five subsystem libraries and links them into
  `car_firmware`. The target build is C11 because the SDK headers need it.
  Our own code stays C99, which the host script enforces.
- `pico_sdk_import.cmake`, finished. Upstream shim, copied verbatim from
  the SDK. Do not edit.
- `run_host_tests.sh`, finished. Compiles and runs the five host tests
  with `-std=c99 -Wall -Wextra -Wconversion`. Exit status is the number of
  failing subsystems, so 0 means all green.
- `ABBREVIATIONS.md`, finished. The abbreviation table the coding standard
  requires. Add to it before using a new one in code.

### `common/`

- `car_types.h`, finished. The contract every subsystem shares: status
  codes, navigation commands, motion events, avoidance actions, mission
  states, and the hump, obstacle and telemetry structs. Change it only by
  agreement, since every module and every test depends on it.
- `car_config.h`, finished as a file, but every value is a guess marked
  `TODO: confirm` or `TODO: tune`. Pins, gains, thresholds, timings, MQTT
  topics, and the two design switches `LINE_SENSOR_IS_ANALOG` and
  `IMU_TURN_RATE_FROM_ENCODERS`. Modules never hardcode a number.
- `car_log.h` and `car_log.c`, finished and working. `car_log_write()` is
  a vprintf wrapper gated by `CAR_LOG_LEVEL`, with debug output compiled
  out under `NDEBUG`.

### Each subsystem folder

The same six files live in `comms/`, `motion/`, `line_barcode/`,
`imu_terrain/` and `scanning/`:

- `include/<name>.h`, finished. The public API with units, contracts and
  the hardware warnings. The only header other modules may include. A
  changed signature means updating the test, the bench and `car_main.c`.
- `<name>.c`, stub. NULL and range guards are real. `comms_is_connected()`
  and `motion_is_busy()` return real flags. `motion_get_encoder_counts()`
  reads the volatile counters. Every algorithm is a `TODO`.
- `test_<name>.c`, finished. One `assert` per guarantee. Fails today at
  the first stub. This is the definition of done: make it pass without
  deleting asserts.
- `bench_<name>.c`, finished as a fixture. A `main()` for a spare Pico
  with only that hardware wired. Calls the public API only and prints
  readings for calibration. Prints zeros until the module is implemented.
- `CMakeLists.txt`, finished. The library plus the bench executable, which
  is `EXCLUDE_FROM_ALL` so it never enters the car build. `comms/` carries
  a `TODO` for the lwIP and MQTT libraries to link when implementing.
- `README.md`, finished. What the module owns, what done means for it, and
  an empty calibration table to fill from the bench.

### `app/`

- `car_main.c`, skeleton. Real: init of all five, the fixed period loop,
  the three per tick calls, telemetry every `CAR_TELEMETRY_PERIOD_MSEC`,
  the `switch` over mission state, and halt on any init failure. `TODO`:
  the transition inside each state.

## Build

Needs `PICO_SDK_PATH`. The VS Code Pico extension installs the SDK under
`~/.pico-sdk/sdk/<version>`.

    export PICO_SDK_PATH=~/.pico-sdk/sdk/2.3.0
    cd firmware
    cmake -S . -B build -G Ninja
    cmake --build build                          # car_firmware only
    cmake --build build --target bench_motion    # one bench, on demand

Outputs are `build/car_firmware.uf2` and `build/<name>/bench_<name>.uf2`.
The `bench_*` targets are excluded from the default build on purpose.
`build/` is ignored by git.

## Flash

With the button: hold BOOTSEL while plugging in USB, a volume `RPI-RP2`
appears, copy the `.uf2` onto it and the board reboots into it.

    cp build/motion/bench_motion.uf2 /Volumes/RPI-RP2/

Without the button, once a Pico is running any firmware from this tree:

    picotool load -f -x build/motion/bench_motion.uf2

`-f` reboots the running Pico into BOOTSEL and `-x` runs the program after
loading.

## Serial output

    ls /dev/cu.usbmodem*
    screen /dev/cu.usbmodemXXXX 115200       # quit: Ctrl-A, then K, then Y

Every bench waits 2 seconds before its first line, long enough to attach.
`car_firmware` prints at boot, and USB serial drops anything sent before a
terminal connects, so open `screen` first and press the board's RESET
button to see its startup lines.

## Smoke test with nothing wired

Do this first on every Pico. It proves the toolchain, flashing and serial
work before any wiring exists. It is safe: every `*_init` is a stub, so no
GPIO is configured or driven. Expected output today:

- `car_firmware`: `car firmware starting`, then
  `subsystem init failed, halting`, then silence. State is HALTED and the
  loop keeps running.
- `bench_motion`: the header, `motion_init failed`, then silence.
- `bench_comms`: the header, `comms_init failed`, then `connected 0` once
  a second.
- `bench_line_barcode`: the header, `line_init failed`, then
  `mask 000 error 0 junction 0` every 100 ms.
- `bench_imu_terrain`: the header, `imu init or calibrate failed`, then
  `pitch 0 heading 0 event 0 hump 0` every 100 ms.
- `bench_scanning`: the header, `scan_init failed`, then
  `angle 0 range 4000 status 5` through `angle 180`, one every 60 ms,
  repeating each second.

Status numbers printed by the benches follow `car_status_t`:

| Value | Status                    |
|-------|---------------------------|
| 0     | `CAR_OK`                  |
| 1     | `CAR_ERR_TIMEOUT`         |
| 2     | `CAR_ERR_RANGE`           |
| 3     | `CAR_ERR_HARDWARE`        |
| 4     | `CAR_ERR_NO_DATA`         |
| 5     | `CAR_ERR_NOT_IMPLEMENTED` |

All six binaries run on a plain Pico or a Pico W today. Once comms is
implemented, `bench_comms` and `car_firmware` need a Pico W.

## One subsystem at a time

The loop for each person: implement the `.c`, run `./run_host_tests.sh`
until your line says PASS, build your bench, flash it to your own Pico with
only your hardware wired, and fill in your README table from what it
prints.

**`bench_motion`.** Wheels off the ground for the first run. The H-bridge
has its own battery with ground tied to the Pico. Never route motor
current through the Pico. Defaults: bridge on GP6 to GP9, encoders on GP2
and GP3. Prints the pulse counts for a 500 mm move, which sets
`WHEEL_CIRCUMFERENCE_MM` and exposes a left to right duty mismatch.

**`bench_line_barcode`.** Power the LM393 modules from 3V3 so their output
is 3.3 V and goes straight to a GPIO. Defaults GP26, GP27 and GP0. Slide
the car across the line by hand: the mask should walk 001, 011, 010, 110,
100 and the error should change sign at centre.

**`bench_imu_terrain`.** The LSM303DLHC is a 3.3 V part on a Grove I2C
port. Defaults GP4 and GP5; confirm the Grove port from the underside of
the board. Tilt by hand and pitch should follow. Then mount it, run the
motors with the car held still, and note how far heading moves. That is
the motor disturbance the header warns about.

**`bench_scanning`.** The HC-SR04 runs on 5 V and its Echo output is 5 V.
Divide it down, for example 1k over 2k, before GP17. Trigger on GP16 can
go direct. Servo signal on GP1, servo power from an external 5 V supply
with common ground, never the Pico 3V3 pin. Put a box at a known distance
and bearing and check the printed range and the angle it appears at.

**`bench_comms`.** Needs a Pico W, an MQTT broker on the same network
(`mosquitto` on a laptop is enough), and real values in `COMMS_WIFI_SSID`,
`COMMS_WIFI_PASSWORD` and `COMMS_MQTT_BROKER_HOST`. Those are placeholders.
Keep real ones in an uncommitted change or a build flag.

## Host tests once SDK calls appear

The moment a `.c` includes `hardware/gpio.h` it stops compiling on the
host. Keep every SDK call inside a few small `static` functions at the
bottom of the file, wrap only those in `#ifndef CAR_HOST_TEST` with a
trivial fake in the `#else`, and add `-DCAR_HOST_TEST` to `FLAGS` in
`run_host_tests.sh`. The algorithm above them stays plain C and stays
testable. PID maths, the barcode state machine and the hump estimator are
what is worth testing on the host anyway.

## The full car

All five wired, flash `car_firmware`. Any failed init halts before the
wheels move, so a partially built car halts. For incremental integration,
treat `CAR_ERR_NOT_IMPLEMENTED` from an init as absent and continue, and
only `CAR_ERR_HARDWARE` as fatal. That change lives in `main()` and is
about four lines.

## Pin budget

The board commits GP10 to GP22 and GP28 to onboard peripherals. Free: GP0
to GP9, GP26, GP27, which is 12 pins. Needed: 14. GP16 and GP17 are taken
back from the unused ESP-01 socket. Fill in the right column as wired.

| Function                 | Pins needed | Assigned in car_config.h | As wired |
|--------------------------|-------------|--------------------------|----------|
| Motor left, DRV8833 style| 2           | GP8, GP9                 |          |
| Motor right              | 2           | GP6, GP7                 |          |
| Encoder left, right      | 2           | GP2, GP3                 |          |
| IR left, centre, right   | 3           | GP26, GP27, GP0          |          |
| I2C SDA, SCL to IMU      | 2           | GP4, GP5                 |          |
| Sonar trigger, echo      | 2           | GP16, GP17 from ESP-01   |          |
| Servo PWM                | 1           | GP1                      |          |
| Total                    | 14          |                          |          |

Choices that cost pins: an L298N driver needs 6 motor pins not 4, and
quadrature encoders need 4 not 2. Either forces reclaiming GP10 to GP15
from the micro SD slot, which is free when no card is fitted.
