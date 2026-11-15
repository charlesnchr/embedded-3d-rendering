/*
 * A model of the part of the SSD1331 OLED controller that the firmware uses:
 * the display RAM, the address window and its write pointer, and the command
 * parser. It takes the bytes the firmware clocks out over SPI, so the picture
 * in the display RAM is the one the panel would show.
 */
#ifndef SIM_SSD1331_H
#define SIM_SSD1331_H

#include <stddef.h>
#include <stdint.h>

#define SSD1331_WIDTH	96
#define SSD1331_HEIGHT	64

typedef struct {
	uint16_t	ram[SSD1331_HEIGHT][SSD1331_WIDTH];	/* RGB565, one entry per pixel */
	uint8_t		colStart, colEnd, rowStart, rowEnd;	/* address window */
	uint8_t		col, row;				/* write pointer */
	uint8_t		remap;					/* argument of command 0xA0 */
	uint8_t		displayOn;

	uint8_t		command;				/* command waiting for arguments */
	uint8_t		args[32];
	uint8_t		argCount, argsNeeded;

	uint8_t		highByte;				/* first byte of a pixel */
	uint8_t		haveHighByte;
} ssd1331_t;

void ssd1331_reset(ssd1331_t *d);

/* Bytes received with the D/C line low. */
void ssd1331_write_command(ssd1331_t *d, const uint8_t *bytes, size_t count);

/*
 * Bytes received with the D/C line high. Returns the number of times the write
 * pointer wrapped back to the start of the window, which is the number of
 * frames completed by this call.
 */
unsigned ssd1331_write_data(ssd1331_t *d, const uint8_t *bytes, size_t count);

/* Colour of the pixel at (x, y) as 8-bit red, green and blue. */
void ssd1331_pixel(const ssd1331_t *d, int x, int y, uint8_t rgb[3]);

#endif
