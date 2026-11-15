#include <string.h>

#include "ssd1331.h"

void
ssd1331_reset(ssd1331_t *d)
{
	memset(d, 0, sizeof(*d));
	d->colEnd = SSD1331_WIDTH - 1;
	d->rowEnd = SSD1331_HEIGHT - 1;
	d->remap = 0x40;	/* 65k colours, the reset value */
}

unsigned
ssd1331_write_data(ssd1331_t *d, const uint8_t *bytes, size_t count)
{
	unsigned frames = 0;
	size_t i;

	for (i = 0; i < count; i++) {
		/* In the 65k colour format a pixel is two bytes, high byte first. */
		if (!d->haveHighByte) {
			d->highByte = bytes[i];
			d->haveHighByte = 1;
			continue;
		}
		d->haveHighByte = 0;
		d->ram[d->row][d->col] = (uint16_t)(d->highByte << 8 | bytes[i]);

		/* Horizontal address increment, wrapping inside the window. */
		if (d->col != d->colEnd) {
			d->col++;
			continue;
		}
		d->col = d->colStart;
		if (d->row != d->rowEnd) {
			d->row++;
			continue;
		}
		d->row = d->rowStart;
		frames++;
	}

	return frames;
}

void
ssd1331_pixel(const ssd1331_t *d, int x, int y, uint8_t rgb[3])
{
	uint16_t p = d->ram[y][x];
	uint8_t a = (p >> 11) & 0x1F, b = (p >> 5) & 0x3F, c = p & 0x1F;
	uint8_t red = a, blue = c;

	/* Bit 2 of the remap setting swaps the red and blue sub-pixels. */
	if (d->remap & 0x04) {
		red = c;
		blue = a;
	}

	/* Expand 5 and 6 bits to 8 by repeating the high bits in the low ones. */
	rgb[0] = (uint8_t)(red << 3 | red >> 2);
	rgb[1] = (uint8_t)(b << 2 | b >> 4);
	rgb[2] = (uint8_t)(blue << 3 | blue >> 2);
}
