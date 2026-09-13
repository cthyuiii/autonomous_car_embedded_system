# IMU subsystem: LSM303DLHC, tilt, humps, motion events, collision.
add_library(imu_terrain STATIC imu_terrain.c)
target_include_directories(imu_terrain PUBLIC include)
target_link_libraries(imu_terrain PUBLIC car_common hardware_i2c)

# Bench: flashed alone to one Pico. See motion/CMakeLists.txt for why
# EXCLUDE_FROM_ALL. Build: cmake --build build --target bench_imu_terrain
add_executable(bench_imu_terrain EXCLUDE_FROM_ALL bench_imu_terrain.c)
target_link_libraries(bench_imu_terrain PRIVATE imu_terrain)
pico_enable_stdio_usb(bench_imu_terrain 1)
pico_enable_stdio_uart(bench_imu_terrain 0)
pico_add_extra_outputs(bench_imu_terrain)
