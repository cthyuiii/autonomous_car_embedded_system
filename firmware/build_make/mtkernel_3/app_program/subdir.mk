################################################################################
# Application build: the car firmware and its benches.
#
# Every .c directly under app_program/ and one level of subfolders is
# compiled, except bench_*.c and test_*.c. `make BENCH=motion` swaps
# car_main.c for motion/bench_motion.c so one subsystem can be flashed alone.
# The image keeps the kernel's name (mtk3pico_smp0_usb_cdc.uf2), so the last
# build wins; flash straight after building.
################################################################################

APP_SRCS := $(wildcard ../app_program/*.c ../app_program/*/*.c)
APP_SRCS := $(filter-out $(wildcard ../app_program/*/bench_*.c \
                                    ../app_program/*/test_*.c), $(APP_SRCS))

ifdef BENCH
# Found by search rather than by folder, so a diagnostic bench can sit in
# whichever subsystem owns the hardware it pokes at.
BENCH_SRC := $(wildcard ../app_program/*/bench_$(BENCH).c)
ifeq ($(BENCH_SRC),)
$(error No bench called '$(BENCH)'. Look for app_program/*/bench_*.c)
endif
APP_SRCS := $(filter-out ../app_program/car_main.c, $(APP_SRCS)) $(BENCH_SRC)
endif

# Switching between the car and a bench changes the object list but
# touches no source, and make only compares timestamps,
# so it would happily reuse the previous objects and flash the wrong
# program. A stamp records which selection the objects were built from; when
# it differs our objects and the image go, so both rules run again. Only our
# objects, never this file or the kernel's, so it costs a few seconds.
APP_SELECTED := mtkernel_3/app_program/.selected_$(if $(BENCH),$(BENCH),car)
ifeq ($(wildcard $(APP_SELECTED)),)
$(shell mkdir -p mtkernel_3/app_program; \
        rm -f mtkernel_3/app_program/.selected_* $(EXE_FILE).elf \
              $(EXE_FILE).uf2; \
        find mtkernel_3/app_program -name '*.o' -delete; \
        touch $(APP_SELECTED))
endif

# Our headers. The kernel's INCPATH already covers include/ and config/.
APP_INCPATH := -I"../app_program/common" \
               -I"../app_program/comms/include" \
               -I"../app_program/motion/include" \
               -I"../app_program/line_barcode/include" \
               -I"../app_program/imu_terrain/include" \
               -I"../app_program/scanning/include"

# The Pico C SDK hardware headers common/car_hw.c drives the chip through,
# whichever console is built. Header-only parts: nothing of the SDK is
# compiled or linked for them. The pico/ stubs for the generated
# config_autogen.h and version.h sit in the USB console folder.
PICO_SDK_PATH ?= ../../sdk/pico-sdk
APP_SDK := $(PICO_SDK_PATH)/src
APP_INCPATH += -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
               -I"$(APP_SDK)/common/pico_base_headers/include" \
               -I"$(APP_SDK)/rp2040/pico_platform/include" \
               -I"$(APP_SDK)/rp2_common/pico_platform_compiler/include" \
               -I"$(APP_SDK)/rp2_common/pico_platform_sections/include" \
               -I"$(APP_SDK)/rp2_common/pico_platform_panic/include" \
               -I"$(APP_SDK)/rp2_common/pico_platform_common/include" \
               -I"$(APP_SDK)/rp2_common/hardware_base/include" \
               -I"$(APP_SDK)/rp2_common/hardware_irq/include" \
               -I"$(APP_SDK)/rp2_common/hardware_gpio/include" \
               -I"$(APP_SDK)/rp2_common/hardware_pwm/include" \
               -I"$(APP_SDK)/rp2_common/hardware_resets/include" \
               -I"$(APP_SDK)/rp2040/hardware_regs/include" \
               -I"$(APP_SDK)/rp2040/hardware_structs/include"

APP_OBJS := $(subst ../, ./mtkernel_3/, $(APP_SRCS:.c=.o))
OBJS     += $(APP_OBJS)
C_DEPS   += $(APP_OBJS:.o=.d)

# Warnings apply to our code only. The kernel keeps its own flags.
mtkernel_3/app_program/%.o: ../app_program/%.c
	@mkdir -p "$(@D)"
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -Wall -Wextra -D$(TARGET) $(INCPATH) $(APP_INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
