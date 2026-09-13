# Scanning subsystem: servo, HC-SR04, profiling, avoidance, recovery.
add_library(scanning STATIC scanning.c)
target_include_directories(scanning PUBLIC include)
target_link_libraries(scanning PUBLIC car_common hardware_pwm)

# Bench: flashed alone to one Pico. See motion/CMakeLists.txt for why
# EXCLUDE_FROM_ALL. Build: cmake --build build --target bench_scanning
add_executable(bench_scanning EXCLUDE_FROM_ALL bench_scanning.c)
target_link_libraries(bench_scanning PRIVATE scanning)
pico_enable_stdio_usb(bench_scanning 1)
pico_enable_stdio_uart(bench_scanning 0)
pico_add_extra_outputs(bench_scanning)
