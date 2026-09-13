#!/bin/sh
# Compiles and runs every host contract test. No Pico SDK needed.
# Exit status is the number of failing subsystems, so 0 means all green.
# Each test aborts on its first failing assert, which today is the first
# stub that returns CAR_ERR_NOT_IMPLEMENTED. That is the expected start.

set -u
cd "$(dirname "$0")" || exit 1
mkdir -p build/host

CC="${CC:-cc}"
FLAGS="-std=c99 -Wall -Wextra -Wconversion -I common"
fail=0

for name in comms motion line_barcode imu_terrain scanning; do
    bin="build/host/test_$name"
    if $CC $FLAGS -I "$name/include" \
        "$name/$name.c" "$name/test_$name.c" common/car_log.c -o "$bin" \
        && "./$bin" 2>/dev/null; then
        echo "PASS  $name"
    else
        echo "FAIL  $name"
        fail=$((fail + 1))
    fi
done

exit "$fail"
