#!/bin/sh
# Build one image and flash it to the Pico in a single step.
#
# Every profile links to the same file name, so a bench built after the car
# silently replaces it. Building and flashing together is the only way to be
# sure the image on the board is the one you just asked for.
#
#   ./flash.sh              the whole car
#   ./flash.sh motion       the motion bench
#   ./flash.sh line_barcode the line and barcode bench
#   ./flash.sh imu_terrain  the IMU bench
#   ./flash.sh scanning     the scanning bench
#   ./flash.sh comms        the comms bench, needs the radio profile below
#   ./flash.sh --wifi       the car with the radio and MQTT
#   ./flash.sh --no-recover the car, but it never searches for the line
#   ./flash.sh --build-only motion   build it, do not flash
#
# The first flash of a blank Pico needs the BOOTSEL button: hold it while
# plugging in the USB cable, then run this. Afterwards picotool reboots the
# board itself and the button is never needed again.

set -u
cd "$(dirname "$0")/build_make" || exit 1

PICO_SDK_PATH="${PICO_SDK_PATH:-$HOME/.pico-sdk/sdk/2.3.0}"
export PICO_SDK_PATH

BENCH=""
WIFI=0
FLASH=1
NORECOVER=0

for arg in "$@"; do
    case "$arg" in
        --wifi)       WIFI=1 ;;
        --no-recover) NORECOVER=1 ;;
        --build-only) FLASH=0 ;;
        -*)           echo "unknown option $arg" >&2; exit 2 ;;
        *)            BENCH="$arg" ;;
    esac
done

if [ -n "$BENCH" ] \
   && [ -z "$(ls ../app_program/*/bench_"$BENCH".c 2>/dev/null)" ]; then
    echo "no bench called '$BENCH'. Available:" >&2
    ls ../app_program/*/bench_*.c 2>/dev/null \
        | sed 's|.*/bench_||; s|\.c$||; s|^|  |' >&2
    exit 2
fi

ARGS="CONSOLE=usb_cdc"
IMAGE="mtk3pico_smp0_usb_cdc.uf2"

if [ "$WIFI" = "1" ]; then
    if [ ! -f ../config/wifi_credentials.h ]; then
        echo "config/wifi_credentials.h is missing." >&2
        echo "Copy config/wifi_credentials.example.h to it and fill it in." >&2
        exit 2
    fi
    ARGS="$ARGS WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1"
    IMAGE="mtk3pico_smp0_usb_cdc_wifi_dhcp_mqtt.uf2"
fi

[ -n "$BENCH" ] && ARGS="$ARGS BENCH=$BENCH"

if [ "$NORECOVER" = "1" ]; then
    if [ -n "$BENCH" ]; then
        echo "--no-recover is a car option; benches do not run car_main" >&2
        exit 2
    fi
    ARGS="$ARGS NO_RECOVER=1"
fi

if [ "$BENCH" = "comms" ] && [ "$WIFI" = "0" ]; then
    echo "the comms bench needs the radio: ./flash.sh --wifi comms" >&2
    exit 2
fi

WHAT="${BENCH:-the car}"
[ "$WIFI" = "1" ] && WHAT="$WHAT with the radio"
[ "$NORECOVER" = "1" ] && WHAT="$WHAT, no line search"
echo "building $WHAT"
# shellcheck disable=SC2086
if ! make $ARGS -j8; then
    echo "build failed, nothing flashed" >&2
    exit 1
fi

[ "$FLASH" = "0" ] && { echo "built $IMAGE, not flashing"; exit 0; }

if [ ! -f "$IMAGE" ]; then
    echo "expected $IMAGE after the build and it is not there" >&2
    exit 1
fi

echo "flashing $IMAGE"
# -f reboots a running board into BOOTSEL, -x runs the program afterwards.
if ! picotool load -f -x "$IMAGE"; then
    echo >&2
    echo "picotool could not reach a board." >&2
    echo "If this Pico has never been flashed, hold BOOTSEL while plugging" >&2
    echo "in the USB cable, then run this again." >&2
    exit 1
fi

echo
echo "flashed $WHAT. Watch it with:"
echo "  screen \$(ls /dev/cu.usbmodem* | head -1) 115200"
echo "Quit screen with Ctrl-A then K then Y."
