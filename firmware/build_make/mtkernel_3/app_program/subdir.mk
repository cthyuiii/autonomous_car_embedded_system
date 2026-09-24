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

# make NO_RECOVER=1 builds the car so it never enters RECOVER_LINE, and
# make NO_ENCODERS=1 forces MOTION_OPEN_LOOP to 1. Both are folded into the
# stamp below, because they change no source file either.
APP_DEFS :=
APP_TAG  :=
ifeq ($(NO_RECOVER),1)
APP_DEFS += -DCAR_SKIP_LINE_RECOVERY=1
APP_TAG  := $(APP_TAG)_norecover
endif
ifeq ($(NO_ENCODERS),1)
APP_DEFS += -DMOTION_OPEN_LOOP=1u
APP_TAG  := $(APP_TAG)_noencoders
endif

# Switching between the car, a bench and these flags changes the object list
# or the defines, but touches no source, and make only compares timestamps,
# so it would happily reuse the previous objects and flash the wrong
# program. A stamp records which selection the objects were built from; when
# it differs our objects and the image go, so both rules run again. Only our
# objects, never this file or the kernel's, so it costs a few seconds.
APP_SELECTED := mtkernel_3/app_program/.selected_$(if $(BENCH),$(BENCH),car)$(APP_TAG)
ifeq ($(wildcard $(APP_SELECTED)),)
$(shell mkdir -p mtkernel_3/app_program; \
        rm -f mtkernel_3/app_program/.selected_* $(EXE_FILE).elf \
              $(EXE_FILE).uf2; \
        find mtkernel_3/app_program -name '*.o' -delete; \
        touch $(APP_SELECTED))
endif

# Our headers only. The kernel's INCPATH already covers include/ and config/.
APP_INCPATH := -I"../app_program/common" \
               -I"../app_program/comms/include" \
               -I"../app_program/motion/include" \
               -I"../app_program/line_barcode/include" \
               -I"../app_program/imu_terrain/include" \
               -I"../app_program/scanning/include"

APP_OBJS := $(subst ../, ./mtkernel_3/, $(APP_SRCS:.c=.o))
OBJS     += $(APP_OBJS)
C_DEPS   += $(APP_OBJS:.o=.d)

# Warnings apply to our code only. The kernel keeps its own flags.
mtkernel_3/app_program/%.o: ../app_program/%.c
	@mkdir -p "$(@D)"
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -Wall -Wextra -D$(TARGET) $(APP_DEFS) $(INCPATH) $(APP_INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
