# scanning

Owner: Buddy 5, ultrasonic scanning and obstacle profiling.

Owns the scan servo, HC-SR04 ranging, coarse and fine scans, obstacle
profiling, avoidance planning and line recovery. API is
`include/scanning.h`. Knobs are `SONAR_*`, `SERVO_*` and `SCAN_*` in
`common/car_config.h`. Read the two warnings in the header before wiring.

Scan budget: 60 ms minimum per ranging, from the datasheet, plus settle per
move. Five coarse angles is 5 x 60 = 300 ms standing still before travel.
Fine scan over 60 degrees at 15 degree steps is 5 more. Plan for it.

Done means `test_scanning.c` passes on the host, and `bench_scanning` puts
a box at 300 mm within 10 mm and 15 degrees of its true bearing.

| Calibration              | Measured | How                                 |
|--------------------------|----------|-------------------------------------|
| SERVO_SETTLE_MSEC        |          | reduce until readings smear         |
| Range error at 100, 300 mm|         | tape measure vs printed             |
| SERVO_PULSE_MIN and MAX  |          | find the mechanical end stops       |
