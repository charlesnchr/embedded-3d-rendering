/*
 * Runs the firmware on a desktop. The firmware source is compiled as it is,
 * with its main renamed to firmware_main and the Kinetis SDK headers replaced
 * by the ones in sim/ksdk. Everything it sends to the display ends up in the
 * SSD1331 model, and this file turns the display contents into a GIF or PPM
 * files.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gif.h"
#include "sim.h"

#define WIDTH			SSD1331_WIDTH
#define HEIGHT			SSD1331_HEIGHT

int firmware_main(void);

static struct {
	int		frames;		/* frames to run before stopping */
	int		scale;
	int		fps;
	const char	*gifPath, *ppmPrefix;
} option = { .frames = 36, .scale = 8, .fps = 12 };

static uint8_t **captured;
static int capturedCount;

static void
read_display(uint8_t *rgb)
{
	int x, y;

	for (y = 0; y < HEIGHT; y++)
		for (x = 0; x < WIDTH; x++)
			ssd1331_pixel(&simDisplay, x, y, &rgb[3 * (y * WIDTH + x)]);
}

static int
write_ppm(const char *path, const uint8_t *rgb, int scale)
{
	FILE *file = fopen(path, "wb");
	int x, y;

	if (!file)
		return -1;
	fprintf(file, "P6\n%d %d\n255\n", WIDTH * scale, HEIGHT * scale);
	for (y = 0; y < HEIGHT * scale; y++)
		for (x = 0; x < WIDTH * scale; x++)
			fwrite(&rgb[3 * ((y / scale) * WIDTH + x / scale)], 1, 3, file);
	return fclose(file);
}

static void
finish(void)
{
	char path[4096];
	int i, status;

	for (i = 0; option.ppmPrefix && i < capturedCount; i++) {
		snprintf(path, sizeof(path), "%s%03d.ppm", option.ppmPrefix, i);
		if (write_ppm(path, captured[i], option.scale) != 0) {
			fprintf(stderr, "cannot write %s\n", path);
			exit(1);
		}
	}
	if (option.ppmPrefix && capturedCount > 0)
		printf("wrote %s000.ppm to %s%03d.ppm\n", option.ppmPrefix, option.ppmPrefix, capturedCount - 1);

	if (option.gifPath && capturedCount > 0) {
		status = gif_write(option.gifPath, WIDTH, HEIGHT, captured, capturedCount, option.scale,
				   (100 + option.fps / 2) / option.fps);
		if (status != 0) {
			fprintf(stderr, status == -2 ? "more than 256 colours, use --ppm instead of --gif\n"
						     : "cannot write %s\n", option.gifPath);
			exit(1);
		}
		printf("wrote %s (%d frames, %dx%d)\n", option.gifPath, capturedCount,
		       WIDTH * option.scale, HEIGHT * option.scale);
	}

	exit(0);
}

void
sim_pixels_written(unsigned framesCompleted)
{
	if (framesCompleted == 0)
		return;

	captured[capturedCount] = malloc(3 * WIDTH * HEIGHT);
	if (!captured[capturedCount]) {
		fprintf(stderr, "out of memory\n");
		exit(1);
	}
	read_display(captured[capturedCount]);

	if (++capturedCount == option.frames)
		finish();
}

static void
usage(const char *program)
{
	printf("usage: %s [options]\n"
	       "\n"
	       "Runs the firmware against a model of the SSD1331 display and writes\n"
	       "what the display shows after each frame.\n"
	       "\n"
	       "  --gif FILE      write an animated GIF\n"
	       "  --ppm PREFIX    write PREFIX000.ppm, PREFIX001.ppm, ...\n"
	       "  --frames N      stop after N frames (default 36, one full turn)\n"
	       "  --scale N       enlarge GIF and PPM output N times (default 8)\n"
	       "  --fps N         frame rate of the GIF (default 12)\n"
	       "  --console       show what the firmware prints to its debug console\n",
	       program);
}

static int
number(const char *text, int min, int max)
{
	char *end;
	long value = strtol(text, &end, 10);

	if (*text == '\0' || *end != '\0' || value < min || value > max) {
		fprintf(stderr, "expected a number from %d to %d, got '%s'\n", min, max, text);
		exit(2);
	}
	return (int)value;
}

int
main(int argc, char **argv)
{
	int i;

	for (i = 1; i < argc; i++) {
		const char *a = argv[i];
		const int hasValue = i + 1 < argc;

		if (strcmp(a, "--console") == 0)
			simConsole = 1;
		else if (strcmp(a, "--gif") == 0 && hasValue)
			option.gifPath = argv[++i];
		else if (strcmp(a, "--ppm") == 0 && hasValue)
			option.ppmPrefix = argv[++i];
		else if (strcmp(a, "--frames") == 0 && hasValue)
			option.frames = number(argv[++i], 1, 1000000);
		else if (strcmp(a, "--scale") == 0 && hasValue)
			option.scale = number(argv[++i], 1, 32);
		else if (strcmp(a, "--fps") == 0 && hasValue)
			option.fps = number(argv[++i], 1, 1000);
		else {
			usage(argv[0]);
			return strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0 ? 0 : 2;
		}
	}

	if (!option.gifPath && !option.ppmPrefix) {
		usage(argv[0]);
		return 2;
	}

	captured = calloc((size_t)option.frames, sizeof(*captured));
	if (!captured) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}

	ssd1331_reset(&simDisplay);	/* the state of the controller after power-on */
	firmware_main();

	/* Some builds of the firmware draw one frame and return. */
	finish();

	return 0;
}
