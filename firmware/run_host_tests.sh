#!/bin/sh
# Compiles and runs every host contract test. No kernel or SDK needed.
# Exit status is the number of failing subsystems, so 0 means all green.
# Each test aborts on its first failing assert, which today is the first
# stub that returns CAR_ERR_NOT_IMPLEMENTED. That is the expected start.
#
# CAR_HOST_TEST selects the host branch of CAR_LOG in car_log.h. Guard any
# kernel or BSP call in a module the same way so the module stays host
# buildable once it is implemented.

set -u
cd "$(dirname "$0")" || exit 1
mkdir -p build_host

CC="${CC:-cc}"
FLAGS="-std=c99 -Wall -Wextra -Wconversion -DCAR_HOST_TEST"
APP=app_program
fail=0

for name in comms motion line_barcode imu_terrain scanning; do
    bin="build_host/test_$name"
    if $CC $FLAGS -I "$APP/common" -I "$APP/$name/include" \
        "$APP/$name/$name.c" "$APP/$name/test_$name.c" -o "$bin" \
        && "./$bin" 2>/dev/null; then
        echo "PASS  $name"
    else
        echo "FAIL  $name"
        fail=$((fail + 1))
    fi
done

exit "$fail"
