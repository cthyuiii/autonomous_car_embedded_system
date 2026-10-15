# imu_terrain

Owner: Buddy 4, IMU based motion and terrain monitoring.

Owns the LSM303DLHC, tilt, hump detection and peak height, motion events,
collision detection and turn rate. The API is in `include/imu.h`, the
values are `IMU_*` in `common/car_config.h`. The part has no gyroscope,
which shapes everything here.

**Pitch** comes from the gravity vector relative to a level reference
taken at boot, and is frozen whenever that vector is not 1 g. An
accelerometer cannot tell a tilt from a push, so a running motor would
otherwise swing pitch by several degrees on a level car. The bench prints
`mag`, the vector length in milli g, and `ok`, which is 1 while `mag` is
within `IMU_PITCH_TRUST_BAND_MILLI_G` of 1000.

**Heading** is the horizontal magnetometer angle with a hard iron offset
tracked from the running extremes, so it settles after the first full
turn.

**Hump height** adds sin(pitch) over the ground distance from motion.
**Turn rate** comes from the wheel speed difference. The I2C bus moves from
the driver's default GP8 and GP9 to Grove 3 at init.

Tests: `test_imu_terrain.c` on the host, `./flash.sh imu_terrain` on the
car.

| Calibration                     | Value | How                                  |
|---------------------------------|-------|--------------------------------------|
| IMU_FORWARD_AXIS                | 1     | the module is turned 90 degrees      |
| IMU_PITCH_SIGN                  | 1     | nose up reads positive               |
| IMU_PITCH_TRUST_BAND_MILLI_G    | 300   | the mast mount shakes                |
| IMU_HUMP_PITCH_THRESHOLD_DEG    | 5     | tilt to the hump's angle             |
| IMU_COLLISION_THRESHOLD_MILLI_G | 600   | tap the bumper and watch `jolt`      |

`terrain_report.md` is the terrain analysis report: how each quantity is
worked out and what the bench measured.
