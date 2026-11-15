# Builds the firmware for the desktop, to run against a model of the display.

CC	?= cc
CFLAGS	?= -O2
BUILD	?= build

FIRMWARE_DIR	:= Implementations/C
SIM_OBJECTS	:= $(BUILD)/sim/main.o $(BUILD)/sim/ksdk.o $(BUILD)/sim/ssd1331.o
SIMULATORS	:= $(BUILD)/firmware $(BUILD)/firmware-novert $(BUILD)/firmware-noztest

# The firmware is compiled as it is. -Dmain renames its entry point so that
# sim/main.c can call it, and -Isim/ksdk makes its SDK includes find the
# stand-in headers. Its warnings are left off, the source is from 2018.
FIRMWARE_FLAGS	:= -std=gnu99 -w -Isim/ksdk -Dmain=firmware_main

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

clean:
	rm -rf $(BUILD)

.PHONY: all clean
