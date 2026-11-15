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

clean:
	rm -rf $(BUILD) out

.PHONY: all run gif check bench clean
