# Firmware

Autonomous line following car on a Raspberry Pi Pico W carried by a Cytron
Maker Pi Pico base. Five subsystems, one vehicle controller, one Pico on the
car. The other Picos are development benches, one per person.

## Layout

| Path           | Owns                                                |
|----------------|-----------------------------------------------------|
| `common/`      | shared types, every config knob, logging            |
| `comms/`       | WiFi, MQTT, telemetry, heartbeat, remote commands   |
| `motion/`      | motors, encoders, PID, distance and turns           |
| `line_barcode/`| three IR sensors, line position, barcode decoding   |
| `imu_terrain/` | LSM303DLHC, tilt, humps, motion events, collision   |
| `scanning/`    | servo, HC-SR04, obstacle profiling, avoidance       |
| `app/`         | the vehicle controller and mission state machine    |

Each subsystem folder holds its public header under `include/`, the stubs,
a host test, a hardware bench and a README with its calibration table.

## Build

Host tests, no SDK needed: `./run_host_tests.sh`

Car firmware, with `PICO_SDK_PATH` set:

    cmake -S . -B build && cmake --build build

Flash `build/car_firmware.uf2`. One bench, flashed to a spare Pico:

    cmake --build build --target bench_motion

The `bench_*` targets are excluded from the default build on purpose.

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
