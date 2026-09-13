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
tree except the three places listed under Kernel notes.

| Path                                | Owns                                   |
|-------------------------------------|----------------------------------------|
| `app_program/car_main.c`            | `usermain()`, the tasks, mission state |
| `app_program/common/`               | shared types, every config knob, loggin|
| `app_program/comms/`                | WiFi, MQTT, telemetry, heartbeat       |
| `app_program/motion/`               | motors, encoders, PID, distance, turns |
| `app_program/line_barcode/`         | three IR sensors, position, barcodes   |
| `app_program/imu_terrain/`          | LSM303DLHC, tilt, humps, events        |
| `app_program/scanning/`             | servo, HC-SR04, profiling, avoidance   |
| `build_make/mtkernel_3/app_program/`| `subdir.mk`, how our code is built     |

Each subsystem folder holds its public header under `include/`, the stubs,
a host test, a hardware bench and a README with its calibration table.

## Every file and its status

Three words are used below. **Finished** means there is nothing to write.
**Stub** means the signature, doc comment and input guards are real but the
algorithm returns `CAR_ERR_NOT_IMPLEMENTED` with a `TODO` naming what to
write. **Skeleton** means the structure runs but the decisions are `TODO`.

### Top level of `firmware/`

- `run_host_tests.sh`, finished. Compiles and runs the five host tests
  with `-std=c99 -Wall -Wextra -Wconversion -DCAR_HOST_TEST`. Exit status
  is the number of failing subsystems, so 0 means all green.
- `ABBREVIATIONS.md`, finished. The abbreviation table the coding standard
  requires. Add to it before using a new one in code.
- `build_make/mtkernel_3/app_program/subdir.mk`, finished. Globs our
  sources, excludes benches and tests, adds our include paths and
  warnings, and implements `BENCH=`.
- Everything else under `firmware/` is the kernel. Its own README is at
  `docs/README_PORT.md`.

### `app_program/common/`

- `car_types.h`, finished. The contract every subsystem shares: status
  codes, navigation commands, motion events, avoidance actions, mission
  states, and the hump, obstacle and telemetry structs. Change it only by
  agreement, since every module and every test depends on it.
- `car_config.h`, finished as a file. Motor and servo pins are verified
  from the board maker's examples; every other value is a guess marked
  `TODO: confirm` or `TODO: tune`. Also holds the two design switches
  `LINE_SENSOR_IS_ANALOG` and `IMU_TURN_RATE_FROM_ENCODERS`.
- `car_log.h`, finished. `CAR_LOG(level, fmt, ...)` gated by
  `CAR_LOG_LEVEL`, routed to the kernel console, or to `printf` under
  `CAR_HOST_TEST`. A macro because the kernel's `tm_printf` has no va_list
  entry to forward to; the header says so.

### Each subsystem folder

The same six files live in `comms/`, `motion/`, `line_barcode/`,
`imu_terrain/` and `scanning/`:

- `include/<name>.h`, finished. The public API with units, contracts and
  the hardware warnings. The only header other modules may include. A
  changed signature means updating the test, the bench and `car_main.c`.
- `<name>.c`, stub. NULL and range guards are real. `comms_is_connected()`
  and `motion_is_busy()` return real flags. `motion_get_encoder_counts()`
  reads the volatile counters. Every algorithm is a `TODO` naming the
  kernel BSP or device call to use.
- `test_<name>.c`, finished. One `assert` per guarantee. Fails today at
  the first stub. This is the definition of done: make it pass without
  deleting asserts.
- `bench_<name>.c`, finished as a fixture. A `usermain()` for a spare Pico
  with only that hardware wired. Calls the public API only and prints
  readings for calibration. Prints zeros until the module is implemented.
- `README.md`, finished. What the module owns, what done means for it, and
  an empty calibration table to fill from the bench.

### `app_program/car_main.c`

Skeleton. Real: the mutex, subsystem init, five tasks (motion, IMU, line,
mission, comms) each pacing itself with `tk_dly_tsk()`, the telemetry
snapshot under the mutex, halt on any init failure, and the `switch` over
mission state. `TODO`: the transition inside each state.

## Build

Needs `arm-none-eabi-gcc`, GNU make, a host C++ compiler for the UF2 tool
on first run, and `PICO_SDK_PATH` for the USB console and WiFi profiles.
The VS Code Pico extension installs the SDK under `~/.pico-sdk/sdk/`.

    export PICO_SDK_PATH=~/.pico-sdk/sdk/2.3.0
    cd firmware/build_make
    make CONSOLE=usb_cdc -j8                 # the car
    make CONSOLE=usb_cdc BENCH=motion -j8    # one bench

Output is `firmware/build_make/mtk3pico_smp0_usb_cdc.uf2` either way, so
the last build wins. Flash straight after building. `make clean` between a
car build and a bench build is not needed; the object lists differ.

For WiFi, copy `firmware/config/wifi_credentials.example.h` to
`wifi_credentials.h`, fill it in (git ignores it), and add
`WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1` to the make line. MQTT
also needs `WIFI_TCP=1` and the profiles it requires.

## Flash

With the button: hold BOOTSEL while plugging in USB, a volume `RPI-RP2`
appears, copy the `.uf2` onto it and the board reboots into it.

    cp firmware/build_make/mtk3pico_smp0_usb_cdc.uf2 /Volumes/RPI-RP2/

Without the button, once a Pico is running any USB console build:

    picotool load -f -x firmware/build_make/mtk3pico_smp0_usb_cdc.uf2

`-f` reboots the running Pico into BOOTSEL and `-x` runs the program after
loading.

## Serial output

    ls /dev/cu.usbmodem*
    screen /dev/cu.usbmodemXXXX 115200       # quit: Ctrl-A, then K, then Y

Every bench waits 2 seconds before its first line, long enough to attach.
The car prints at boot, and USB serial drops anything sent before a
terminal connects, so open `screen` first and press the board's RESET
button to see its startup lines.

## Smoke test with nothing wired

Do this first on every Pico. It proves the toolchain, flashing and serial
work before any wiring exists. It is safe: every `*_init` is a stub, so no
GPIO is configured or driven. Expected output today:

- car: `car firmware starting`, then `subsystem init failed, halting`,
  then silence. State is HALTED and the tasks keep running.
- `BENCH=motion`: the header, `motion_init failed`, then silence.
- `BENCH=comms`: the header, `comms_init failed`, then `connected 0` once
  a second.
- `BENCH=line_barcode`: the header, `line_init failed`, then
  `mask 000 error 0 junction 0` every 100 ms.
- `BENCH=imu_terrain`: the header, `imu init or calibrate failed`, then
  `pitch 0 heading 0 event 0 hump 0` every 100 ms.
- `BENCH=scanning`: the header, `scan_init failed`, then
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

## One subsystem at a time

The loop for each person: implement the `.c`, run
`firmware/run_host_tests.sh` until your line says PASS, build your bench,
flash it to your own Pico on your own Robo Pico with only your hardware
wired, and fill in your README table from what it prints.

**`BENCH=motion`.** Wheels off the ground for the first run. Motors go to
the M1 and M2 screw terminals, and the board must run from the battery,
since USB power alone sags under motor load. Encoders on GP6 and GP7.
Prints the pulse counts for a 500 mm move, which sets
`WHEEL_CIRCUMFERENCE_MM` and exposes a left to right duty mismatch.

**`BENCH=line_barcode`.** Sensors on GP26, GP27 and GP28, which are the
ADC pins, so either sensor type fits. Power the LM393 modules from 3V3 so
their output is 3.3 V. Slide the car across the line by hand: the mask
should walk 001, 011, 010, 110, 100 and the error should change sign at
the centre.

**`BENCH=imu_terrain`.** The LSM303DLHC is a 3.3 V part on the I2C0 Grove
port, GP4 and GP5. Tilt by hand and pitch should follow. Then mount it,
run the motors with the car held still, and note how far heading moves.
That is the motor disturbance the header warns about.

**`BENCH=scanning`.** The HC-SR04 runs on 5 V and its Echo output is 5 V.
Divide it down, for example 1k over 2k, before GP17. Trigger on GP16 can
go direct. Servo on header S1, which the board powers from the motor rail,
so again the battery. Put a box at a known distance and bearing and check
the printed range and the angle it appears at.

**`BENCH=comms`.** Needs a Pico W, the WiFi make flags above, a filled in
`wifi_credentials.h`, and an MQTT broker on the same network (`mosquitto`
on a laptop is enough). Set `COMMS_MQTT_BROKER_HOST` in `car_config.h`.

## Host tests once kernel calls appear

The moment a `.c` includes `<tk/tkernel.h>` or `<bsp/libbsp.h>` it stops
compiling on the host. Keep every kernel and BSP call inside a few small
`static` functions at the bottom of the file, wrap only those in
`#ifndef CAR_HOST_TEST` with a trivial fake in the `#else`, and the host
script's `-DCAR_HOST_TEST` does the rest. The algorithm above them stays
plain C and stays testable. PID maths, the barcode state machine and the
hump estimator are what is worth testing on the host anyway.

## The full car

All five wired, build without `BENCH`. Any failed init halts before the
wheels move, so a partially built car halts. For incremental integration,
treat `CAR_ERR_NOT_IMPLEMENTED` from an init as absent and continue, and
only `CAR_ERR_HARDWARE` as fatal. That change lives in `usermain()` and is
about four lines.

## Pin map

Verified pins come from the board maker's example code. Grove port numbers
for the sensor pins are on the board's silkscreen; fill them in as wired.

| Function                  | Pins            | Source                    |
|---------------------------|-----------------|---------------------------|
| Motor left M1A, M1B       | GP8, GP9        | verified, board terminal  |
| Motor right M2A, M2B      | GP10, GP11      | verified, board terminal  |
| Scan servo, header S1     | GP12            | verified                  |
| Kernel console UART0      | GP0, GP1        | Grove 1, do not reuse     |
| Encoder left, right       | GP6, GP7        | Grove port, confirm       |
| I2C0 SDA, SCL to IMU      | GP4, GP5        | Grove port, confirm       |
| Sonar trigger, echo       | GP16, GP17      | Grove port, confirm       |
| IR left, centre, right    | GP26, GP27, GP28| Grove port, confirm, ADC  |
| Board: NeoPixels          | GP18            | verified, leave alone     |
| Board: buttons            | GP20, GP21      | verified, spare inputs    |
| Board: buzzer             | GP22            | verified, leave alone     |
| Pico W radio              | GP23 to 25, 29  | not available             |

Spare after all of the above: GP2, GP3 (Maker port), GP13 to GP15 (servo
headers S2 to S4), GP19.

## Kernel notes

- Single core. `SMP=1` exists but the port documents I2C, ADC, PWM and
  UART as single owner resources under SMP, and nothing here needs the
  second core.
- The tick is `CNF_TIMER_PERIOD` in `firmware/config/config.h`, 10 ms.
  Task delays round up to it.
- Interrupts are registered with `tk_def_int()` and `EnableInt()`, never
  by writing the vector table. Peripheral IRQs run on core 0.
- A file that includes `<tk/tkernel.h>` must not include `<stddef.h>`,
  `<stdio.h>`, `<string.h>` or `<stdlib.h>`: the kernel typedefs its own
  `size_t` and the two collide. Use the kernel's `NULL` and its `tstdlib`
  string functions instead. `<stdint.h>` and `<stdbool.h>` are fine.
- lwIP runs `NO_SYS=1`, so every call into the comms module must come from
  the comms task. Its MQTT client is lwIP's `apps/mqtt`, already in the
  Pico SDK's lwIP tree the port builds from.
- Three edits were made to the vendored kernel, and are the only ones:
  `build_make/pico_rp2040.mk` gained a `pico_util` include path in two
  places, without which the WiFi profile does not build against SDK 2.3;
  `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/.gitkeep`
  exists because the USB console rule needs that directory and the port
  never creates it on macOS make 3.81; and `app_program/subdir.mk` is
  ours.
