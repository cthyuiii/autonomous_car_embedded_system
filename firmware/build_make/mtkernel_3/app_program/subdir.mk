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
APP_SRCS := $(filter-out ../app_program/car_main.c, $(APP_SRCS)) \
            ../app_program/$(BENCH)/bench_$(BENCH).c
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
	$(GCC) $(CFLAGS) -Wall -Wextra -D$(TARGET) $(INCPATH) $(APP_INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
