/* What the SDK stand-ins in ksdk.c share with the driver in main.c. */
#ifndef SIM_SIM_H
#define SIM_SIM_H

#include "ssd1331.h"

extern ssd1331_t simDisplay;

/* Non-zero to print what the firmware writes to its debug console. */
extern int simConsole;

/*
 * Called after every SPI transfer of pixel data. framesCompleted is the number
 * of times the transfer filled the last pixel of the display.
 */
void sim_pixels_written(unsigned framesCompleted);

#endif
