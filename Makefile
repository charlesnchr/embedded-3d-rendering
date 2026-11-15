# Builds the firmware for the desktop and runs it against a model of the
# display. The README describes what each target is for.

CC	?= cc
CFLAGS	?= -O2
BUILD	?= build

FIRMWARE_DIR	:= Implementations/C
SIM_OBJECTS	:= $(BUILD)/sim/main.o $(BUILD)/sim/ksdk.o $(BUILD)/sim/ssd1331.o $(BUILD)/sim/gif.o
SIMULATORS	:= $(BUILD)/firmware $(BUILD)/firmware-novert $(BUILD)/firmware-noztest $(BUILD)/rasterise

# The firmware is compiled as it is. -Dmain renames its entry point so that
# sim/main.c can call it, and -Isim/ksdk makes its SDK includes find the
# stand-in headers. Its warnings are left off, the source is from 2018.
# -ffp-contract=off stops the compiler from merging a multiply and an add into
# one instruction, which would round differently from one machine to the next.
FIRMWARE_FLAGS	:= -std=gnu99 -w -ffp-contract=off -Isim/ksdk -Dmain=firmware_main

all: $(SIMULATORS)

$(BUILD)/sim/%.o: sim/%.c $(wildcard sim/*.h sim/ksdk/*.h)
	@mkdir -p $(BUILD)/sim
	$(CC) $(CFLAGS) -std=c99 -Wall -Wextra -Isim -Isim/ksdk -c -o $@ $<

$(BUILD)/firmware: $(FIRMWARE_DIR)/MainImplementation/i2c_comm_master.c $(SIM_OBJECTS)
	$(CC) $(CFLAGS) $(FIRMWARE_FLAGS) -o $@ $^

$(BUILD)/firmware-novert: $(FIRMWARE_DIR)/NoVert/i2c_comm_master.c $(SIM_OBJECTS)
	$(CC) $(CFLAGS) $(FIRMWARE_FLAGS) -o $@ $^

$(BUILD)/firmware-noztest: $(FIRMWARE_DIR)/NoZtest/i2c_comm_master.c $(SIM_OBJECTS)
	$(CC) $(CFLAGS) $(FIRMWARE_FLAGS) -o $@ $^

$(BUILD)/rasterise: rasterise.c $(SIM_OBJECTS)
	$(CC) $(CFLAGS) $(FIRMWARE_FLAGS) -o $@ $^

# Play the model turning in the terminal until Ctrl-C.
run: $(BUILD)/firmware
	$(BUILD)/firmware

# One full turn, 36 frames of 10 degrees, as an animated GIF.
gif: $(BUILD)/firmware
	@mkdir -p out
	$(BUILD)/firmware --gif out/spin.gif

# Compare every frame with the hashes recorded in sim/expected.
check: all
	$(BUILD)/firmware --hash | diff - sim/expected/firmware.txt
	$(BUILD)/rasterise --hash | diff - sim/expected/firmware.txt
	$(BUILD)/firmware-novert --hash | diff - sim/expected/firmware-novert.txt
	$(BUILD)/firmware-noztest --hash | diff - sim/expected/firmware-noztest.txt
	@echo "all frames match"

# Rendering speed on this machine, over 1000 turns.
bench: $(BUILD)/firmware
	$(BUILD)/firmware --bench --frames 36000

# Code size and stack use on the Cortex-M0+, the core of the KL03. The SDK is
# not in this repository, so its functions are replaced by empty stubs. What is
# left is the firmware source and the floating point routines from libgcc.
ARM_PREFIX	?= arm-none-eabi-
ARM_FLAGS	:= -mcpu=cortex-m0plus -mthumb

size:
	@mkdir -p $(BUILD)/arm
	$(ARM_PREFIX)gcc $(ARM_FLAGS) $(ARM_CFLAGS) -Os -std=gnu99 -w -ffunction-sections -fdata-sections \
		-fstack-usage -Isim/ksdk -c -o $(BUILD)/arm/firmware.o $(FIRMWARE_DIR)/MainImplementation/i2c_comm_master.c
	$(ARM_PREFIX)nm -u $(BUILD)/arm/firmware.o \
		| awk '$$2 !~ /^__aeabi/ { print ".thumb_func\n.global " $$2 "\n" $$2 ":" } END { print "\tbx lr" }' \
		> $(BUILD)/arm/stubs.s
	$(ARM_PREFIX)gcc $(ARM_FLAGS) -nostdlib -Wl,--gc-sections,-e,main -o $(BUILD)/arm/firmware.elf \
		$(BUILD)/arm/firmware.o $(BUILD)/arm/stubs.s -lgcc
	@echo
	@echo "Firmware source alone, then linked with software floating point:"
	@$(ARM_PREFIX)size $(BUILD)/arm/firmware.o $(BUILD)/arm/firmware.elf
	@echo
	@echo "Stack frame of each function in bytes:"
	@awk -F'[:\t]' '{ printf "%8d  %s\n", $$5, $$4 }' $(BUILD)/arm/firmware.su

clean:
	rm -rf $(BUILD) out

.PHONY: all run gif check bench size clean
