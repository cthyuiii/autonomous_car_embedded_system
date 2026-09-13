# Line and barcode subsystem: three IR sensors, position, junctions, decode.
add_library(line_barcode STATIC line_barcode.c)
target_include_directories(line_barcode PUBLIC include)
target_link_libraries(line_barcode PUBLIC car_common hardware_adc)

# Bench: flashed alone to one Pico. See motion/CMakeLists.txt for why
# EXCLUDE_FROM_ALL. Build: cmake --build build --target bench_line_barcode
add_executable(bench_line_barcode EXCLUDE_FROM_ALL bench_line_barcode.c)
target_link_libraries(bench_line_barcode PRIVATE line_barcode)
pico_enable_stdio_usb(bench_line_barcode 1)
pico_enable_stdio_uart(bench_line_barcode 0)
pico_add_extra_outputs(bench_line_barcode)
