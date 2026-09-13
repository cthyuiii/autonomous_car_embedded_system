cmake_minimum_required(VERSION 3.13)

# Target the Pico W. Must be set before the SDK is imported.
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in the Pico SDK from PICO_SDK_PATH, or fetch it from git if
# PICO_SDK_FETCH_FROM_GIT is set.
include(pico_sdk_import.cmake)

project(autonomous_car C CXX ASM)

# NOTE: The SDK headers use C11 static_assert and its interface sources
# compile inside our targets, so the target build must be C11. Our own code
# stays within C99, which run_host_tests.sh enforces with -std=c99.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# NOTE: run_host_tests.sh also enables -Wconversion. It is left off here
# because SDK interface sources compile into every target and are not clean
# under it, which would bury our own warnings.
add_compile_options(-Wall -Wextra)

# Shared contract, config and logging. Every subsystem links this.
add_library(car_common STATIC common/car_log.c)
target_include_directories(car_common PUBLIC common)
target_link_libraries(car_common PUBLIC pico_stdlib)

add_subdirectory(comms)
add_subdirectory(motion)
add_subdirectory(line_barcode)
add_subdirectory(imu_terrain)
add_subdirectory(scanning)

# The car firmware. This is the only target that links all five subsystems.
# The bench_* executables are EXCLUDE_FROM_ALL in each subsystem and are
# never linked here.
add_executable(car_firmware app/car_main.c)
target_link_libraries(car_firmware PRIVATE
    car_common comms motion line_barcode imu_terrain scanning)
pico_enable_stdio_usb(car_firmware 1)
pico_enable_stdio_uart(car_firmware 0)
pico_add_extra_outputs(car_firmware)
