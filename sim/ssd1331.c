#include <string.h>

#include "ssd1331.h"

/*
 * Number of argument bytes that follow each command, from section 9 of the
 * SSD1331 datasheet (Datasheets/SSD1331_1.2.pdf). Commands that are not listed
 * take none.
 */
static uint8_t
argument_count(uint8_t command)
{
	switch (command) {
	case 0x15: return 2;	/* set column address */
	case 0x75: return 2;	/* set row address */
	case 0x21: return 7;	/* draw line */
	case 0x22: return 10;	/* draw rectangle */
	case 0x23: return 6;	/* copy */
	case 0x24: return 4;	/* dim window */
	case 0x25: return 4;	/* clear window */
	case 0x26: return 1;	/* fill enable */
	case 0x27: return 5;	/* scrolling setup */
	case 0x81: case 0x82: case 0x83: return 1;	/* contrast A, B, C */
	case 0x87: return 1;	/* master current */
	case 0x8A: case 0x8B: case 0x8C: return 1;	/* second pre-charge speed */
	case 0xA0: return 1;	/* remap and colour depth */
	case 0xA1: return 1;	/* display start line */
	case 0xA2: return 1;	/* display offset */
	case 0xA8: return 1;	/* multiplex ratio */
	case 0xAB: return 5;	/* dim mode */
	case 0xAD: return 1;	/* master configuration */
	case 0xB0: return 1;	/* power save */
	case 0xB1: return 1;	/* phase 1 and 2 period */
	case 0xB3: return 1;	/* clock divider */
	case 0xB8: return 32;	/* grey scale table */
	case 0xBB: return 1;	/* pre-charge level */
	case 0xBE: return 1;	/* VCOMH */
	case 0xFD: return 1;	/* command lock */
	default:   return 0;
	}
}

void
ssd1331_reset(ssd1331_t *d)
{
	memset(d, 0, sizeof(*d));
	d->colEnd = SSD1331_WIDTH - 1;
	d->rowEnd = SSD1331_HEIGHT - 1;
	d->remap = 0x40;	/* 65k colours, the reset value */
}

static uint8_t
clamp(uint8_t value, uint8_t max)
{
	return value > max ? max : value;
}

/*
 * Only the commands that change what ends up in the display RAM have an
 * effect. The analogue settings (contrast, pre-charge, clock) are parsed and
 * dropped. The drawing commands 0x21 to 0x24 are parsed and not carried out,
 * because the firmware never sends them.
 */
static void
execute(ssd1331_t *d)
{
	const uint8_t *a = d->args;
	int x, y;

	switch (d->command) {
	case 0x15:
		d->colStart = clamp(a[0], SSD1331_WIDTH - 1);
		d->colEnd = clamp(a[1], SSD1331_WIDTH - 1);
		d->col = d->colStart;
		d->haveHighByte = 0;
		break;
	case 0x75:
		d->rowStart = clamp(a[0], SSD1331_HEIGHT - 1);
		d->rowEnd = clamp(a[1], SSD1331_HEIGHT - 1);
		d->row = d->rowStart;
		d->haveHighByte = 0;
		break;
	case 0x25:
		for (y = clamp(a[1], SSD1331_HEIGHT - 1); y <= clamp(a[3], SSD1331_HEIGHT - 1); y++)
			for (x = clamp(a[0], SSD1331_WIDTH - 1); x <= clamp(a[2], SSD1331_WIDTH - 1); x++)
				d->ram[y][x] = 0;
		break;
	case 0xA0:
		d->remap = a[0];
		break;
	case 0xAE:
		d->displayOn = 0;
		break;
	case 0xAF:
		d->displayOn = 1;
		break;
	default:
		break;
	}
}

void
ssd1331_write_command(ssd1331_t *d, const uint8_t *bytes, size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		if (d->argCount < d->argsNeeded) {
			d->args[d->argCount++] = bytes[i];
		} else {
			d->command = bytes[i];
			d->argCount = 0;
			d->argsNeeded = argument_count(bytes[i]);
		}
		if (d->argCount == d->argsNeeded) {
			execute(d);
			d->argCount = d->argsNeeded = 0;
		}
	}
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
