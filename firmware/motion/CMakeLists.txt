# Motion subsystem: motors, encoders, PID speed control.
add_library(motion STATIC motion.c)
target_include_directories(motion PUBLIC include)
target_link_libraries(motion PUBLIC car_common hardware_pwm)

# Bench: flashed alone to one Pico to exercise this module on hardware.
# EXCLUDE_FROM_ALL keeps it out of the default build, so car_firmware never
# links it. Build on demand: cmake --build build --target bench_motion
add_executable(bench_motion EXCLUDE_FROM_ALL bench_motion.c)
target_link_libraries(bench_motion PRIVATE motion)
pico_enable_stdio_usb(bench_motion 1)
pico_enable_stdio_uart(bench_motion 0)
pico_add_extra_outputs(bench_motion)
