# imu_terrain

Owner: Buddy 4, IMU based motion and terrain monitoring.

Owns the LSM303DLHC, tilt, hump detection and peak height, motion event
classification, collision detection and turn rate. API is
`include/imu_terrain.h`. Knobs are `IMU_*` in `common/car_config.h`.
Read the header first: this part has no gyroscope, which shapes everything.

Done means `test_imu_terrain.c` passes on the host, `bench_imu_terrain`
tracks hand tilt within 2 degrees, and the peak height of the course hump
is reported within 5 mm of a ruler measurement on three runs.

| Calibration                    | Measured | How                         |
|--------------------------------|----------|-----------------------------|
| Gravity reference, milli g     |          | level, motors on            |
| Magnetometer offsets per axis  |          | full turn, motors on        |
| Heading shift from motors, deg |          | stationary, motors on vs off|
| Hump pitch threshold, deg      |          | lowest hump on the course   |
