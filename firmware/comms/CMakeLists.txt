# Comms subsystem: WiFi, MQTT, telemetry, heartbeat.
add_library(comms STATIC comms.c)
target_include_directories(comms PUBLIC include)
# TODO: When implementing, link pico_cyw43_arch_lwip_threadsafe_background
#       and pico_lwip_mqtt, and add an lwipopts.h beside this file. They are
#       left out while the module is a stub so the tree configures without
#       them.
target_link_libraries(comms PUBLIC car_common)

# Bench: flashed alone to one Pico W. See motion/CMakeLists.txt for why
# EXCLUDE_FROM_ALL. Build: cmake --build build --target bench_comms
add_executable(bench_comms EXCLUDE_FROM_ALL bench_comms.c)
target_link_libraries(bench_comms PRIVATE comms)
pico_enable_stdio_usb(bench_comms 1)
pico_enable_stdio_uart(bench_comms 0)
pico_add_extra_outputs(bench_comms)
