# imu_terrain

Owner: Buddy 4, IMU based motion and terrain monitoring.

Owns the LSM303DLHC, tilt, hump detection and peak height, motion event
classification, collision detection and turn rate. API is
`include/imu_terrain.h`. Knobs are `IMU_*` in `common/car_config.h`.
Read the header first: this part has no gyroscope, which shapes everything.

Implemented. Pitch comes from the gravity vector relative to the level
reference taken at boot, and is **frozen whenever that vector is not one
g**. An accelerometer cannot tell a tilt from a push, so a running motor
would otherwise swing pitch by several degrees on a level car. The bench
prints `mag`, the vector length in milli g, and `ok`, which is 1 while
`mag` is within `IMU_PITCH_TRUST_BAND_MILLI_G` of 1000. Motor running,
`mag` moving and `ok 0` is correct behaviour.

Heading is the horizontal magnetometer angle with a hard iron offset
tracked from running extremes, so it settles after the first full turn. Hump height integrates sin(pitch) over distance the
controller feeds in from the motion snapshot. Turn rate comes from the
wheel speed difference while `IMU_TURN_RATE_FROM_ENCODERS` is 1. The bus
is moved from the driver's default GP8 and GP9 to Grove 3 at init.

Done means `test_imu_terrain.c` passes on the host, `bench_imu_terrain`
tracks hand tilt within 2 degrees, and the peak height of the course hump
is reported within 5 mm of a ruler measurement on three runs.

| Calibration                    | Measured | How                         |
|--------------------------------|----------|-----------------------------|
| IMU_PITCH_SIGN                 |          | nose up must read positive  |
| Heading shift from motors, deg |          | stationary, motors on vs off|
| Heading direction              |          | turn right, heading must rise|
| Hump pitch threshold, deg      |          | lowest hump on the course   |
| Collision threshold, milli g   |          | tap the bumper, read event  |
