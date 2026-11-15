#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gif.h"

typedef struct {
	FILE		*file;
	uint32_t	bits;		/* bits waiting to be written, lowest first */
	int		bitCount;
	uint8_t		block[255];	/* image data is stored in blocks of up to 255 bytes */
	int		blockLength;
} bit_writer_t;

static void
put_byte(bit_writer_t *w, uint8_t byte)
{
	w->block[w->blockLength++] = byte;
	if (w->blockLength == 255) {
		fputc(255, w->file);
		fwrite(w->block, 1, 255, w->file);
		w->blockLength = 0;
	}
}

static void
put_code(bit_writer_t *w, unsigned code, int codeSize)
{
	w->bits |= (uint32_t)code << w->bitCount;
	w->bitCount += codeSize;
	while (w->bitCount >= 8) {
		put_byte(w, w->bits & 0xFF);
		w->bits >>= 8;
		w->bitCount -= 8;
	}
}

static void
flush(bit_writer_t *w)
{
	if (w->bitCount > 0)
		put_byte(w, w->bits & 0xFF);
	if (w->blockLength > 0) {
		fputc(w->blockLength, w->file);
		fwrite(w->block, 1, w->blockLength, w->file);
	}
	fputc(0, w->file);	/* end of the image data */
}

/*
 * LZW compression as GIF defines it. table[c][k] is the code for the string
 * of code c followed by the pixel k, or 0 if that string has no code yet.
 */
static void
lzw_encode(FILE *file, const uint8_t *pixels, size_t count, int minCodeSize)
{
	static uint16_t table[4096][256];
	const unsigned clearCode = 1u << minCodeSize, endCode = clearCode + 1;
	unsigned maxCode = endCode, current, pixel;
	int codeSize = minCodeSize + 1;
	bit_writer_t w = { .file = file };
	size_t i;

	memset(table, 0, sizeof(table));
	fputc(minCodeSize, file);
	put_code(&w, clearCode, codeSize);

	current = pixels[0];
	for (i = 1; i < count; i++) {
		pixel = pixels[i];
		if (table[current][pixel]) {
			current = table[current][pixel];
			continue;
		}

		put_code(&w, current, codeSize);
		table[current][pixel] = (uint16_t)++maxCode;
		if (maxCode >= (1u << codeSize))
			codeSize++;
		if (maxCode == 4095) {
			put_code(&w, clearCode, codeSize);
			memset(table, 0, sizeof(table));
			codeSize = minCodeSize + 1;
			maxCode = endCode;
		}
		current = pixel;
	}

	put_code(&w, current, codeSize);
	put_code(&w, endCode, codeSize);
	flush(&w);
}

static void
put_word(FILE *file, unsigned value)
{
	fputc(value & 0xFF, file);
	fputc(value >> 8 & 0xFF, file);
}

int
gif_write(const char *path, int width, int height, uint8_t *const *frames, int frameCount,
	  int scale, int delay)
{
	uint8_t palette[256][3];
	int colours = 0, paletteBits = 2, status = 0;
	const size_t pixelCount = (size_t)width * height;
	const size_t scaledCount = pixelCount * scale * scale;
	uint8_t *index = malloc((size_t)frameCount * pixelCount);
	uint8_t *scaled = malloc(scaledCount);
	FILE *file = fopen(path, "wb");
	int f, c, x, y;
	size_t i;

	if (!index || !scaled || !file) {
		status = -1;
		goto done;
	}

	/* One palette for the whole animation, holding every colour that occurs. */
	for (f = 0; f < frameCount; f++) {
		for (i = 0; i < pixelCount; i++) {
			const uint8_t *rgb = &frames[f][3 * i];

			for (c = 0; c < colours; c++)
				if (memcmp(palette[c], rgb, 3) == 0)
					break;
			if (c == colours) {
				if (colours == 256) {
					status = -2;
					goto done;
				}
				memcpy(palette[colours++], rgb, 3);
			}
			index[f * pixelCount + i] = (uint8_t)c;
		}
	}
	while ((1 << paletteBits) < colours)
		paletteBits++;
	for (c = colours; c < 256; c++)
		memset(palette[c], 0, 3);

	fwrite("GIF89a", 1, 6, file);
	put_word(file, width * scale);
	put_word(file, height * scale);
	fputc(0xF0 | (paletteBits - 1), file);	/* global palette of 2^paletteBits colours */
	fputc(0, file);				/* background colour */
	fputc(0, file);				/* pixel aspect ratio, unused */
	fwrite(palette, 3, (size_t)1 << paletteBits, file);

	/* Application extension that makes the animation loop forever. */
	fwrite("\x21\xFF\x0BNETSCAPE2.0\x03\x01\x00\x00\x00", 1, 19, file);

	for (f = 0; f < frameCount; f++) {
		const uint8_t *source = &index[f * pixelCount];

		for (y = 0; y < height * scale; y++)
			for (x = 0; x < width * scale; x++)
				scaled[(size_t)y * width * scale + x] = source[(y / scale) * width + x / scale];

		/* Graphic control extension with the frame delay. */
		fwrite("\x21\xF9\x04\x00", 1, 4, file);
		put_word(file, delay);
		fwrite("\x00\x00", 1, 2, file);

		/* Image descriptor for a frame that covers the whole canvas. */
		fputc(0x2C, file);
		put_word(file, 0);
		put_word(file, 0);
		put_word(file, width * scale);
		put_word(file, height * scale);
		fputc(0, file);

		lzw_encode(file, scaled, scaledCount, paletteBits);
	}

	fputc(0x3B, file);
	if (ferror(file))
		status = -1;

done:
	if (file && fclose(file) != 0)
		status = -1;
	free(index);
	free(scaled);
	return status;
}
