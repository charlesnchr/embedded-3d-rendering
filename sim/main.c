/*
 * Runs the firmware on a desktop. The firmware source is compiled as it is,
 * with its main renamed to firmware_main and the Kinetis SDK headers replaced
 * by the ones in sim/ksdk. Everything it sends to the display ends up in the
 * SSD1331 model, and this file turns the display contents into a GIF, PPM
 * files or an animation in the terminal.
 */
#define _POSIX_C_SOURCE 200809L

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "gif.h"
#include "sim.h"

#define WIDTH			SSD1331_WIDTH
#define HEIGHT			SSD1331_HEIGHT
#define TRIANGLES_PER_FRAME	8	/* Ngeom in the firmware */

int firmware_main(void);

static struct {
	int		frames;		/* frames to run before stopping, 0 for no limit */
	int		scale;
	int		fps;
	int		everyRow;	/* capture after each row instead of each frame */
	int		play, hash, bench, stats;
	const char	*gifPath, *ppmPrefix;
} option = { .scale = 8, .fps = 12 };

static uint8_t **captured;
static int capturedCount;
static struct timespec startTime;

static double
seconds_since(const struct timespec *start)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (double)(now.tv_sec - start->tv_sec) + (double)(now.tv_nsec - start->tv_nsec) / 1e9;
}

static void
read_display(uint8_t *rgb)
{
	int x, y;

	for (y = 0; y < HEIGHT; y++)
		for (x = 0; x < WIDTH; x++)
			ssd1331_pixel(&simDisplay, x, y, &rgb[3 * (y * WIDTH + x)]);
}

/* 32-bit FNV-1a hash of the display RAM, to compare frames between builds. */
static uint32_t
display_hash(void)
{
	uint32_t hash = 2166136261u;
	int x, y;

	for (y = 0; y < HEIGHT; y++) {
		for (x = 0; x < WIDTH; x++) {
			hash = (hash ^ (simDisplay.ram[y][x] >> 8)) * 16777619u;
			hash = (hash ^ (simDisplay.ram[y][x] & 0xFF)) * 16777619u;
		}
	}
	return hash;
}

/* Reset the colours and show the cursor again. */
static const char restoreTerminal[] = "\x1b[0m\x1b[?25h\n";

static void
on_interrupt(int signal)
{
	(void)signal;
	if (write(STDOUT_FILENO, restoreTerminal, sizeof(restoreTerminal) - 1) < 0)
		_exit(1);
	_exit(130);
}

/*
 * Draws the display in the terminal with 24-bit colour. Each character cell
 * shows two pixels, the upper half block in the foreground colour over the
 * background colour. Colours are only sent when they change.
 */
static void
draw_in_terminal(const uint8_t *rgb)
{
	static char buffer[HEIGHT / 2 * (WIDTH * 44 + 8) + 16];
	const uint8_t *lastTop, *lastBottom;
	char *p = buffer;
	int x, y;

	p += sprintf(p, "\x1b[H");
	for (y = 0; y < HEIGHT; y += 2) {
		lastTop = lastBottom = NULL;
		for (x = 0; x < WIDTH; x++) {
			const uint8_t *top = &rgb[3 * (y * WIDTH + x)];
			const uint8_t *bottom = &rgb[3 * ((y + 1) * WIDTH + x)];

			if (!lastTop || memcmp(top, lastTop, 3) != 0)
				p += sprintf(p, "\x1b[38;2;%d;%d;%dm", top[0], top[1], top[2]);
			if (!lastBottom || memcmp(bottom, lastBottom, 3) != 0)
				p += sprintf(p, "\x1b[48;2;%d;%d;%dm", bottom[0], bottom[1], bottom[2]);
			p += sprintf(p, "\xE2\x96\x80");
			lastTop = top;
			lastBottom = bottom;
		}
		p += sprintf(p, "\x1b[0m\n");
	}
	fwrite(buffer, 1, (size_t)(p - buffer), stdout);
	fflush(stdout);
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
	const double elapsed = seconds_since(&startTime);
	char path[4096];
	int i, status;

	if (option.play) {
		fputs(restoreTerminal, stdout);
		fflush(stdout);
	}

	if (option.bench && capturedCount > 0) {
		printf("%d frames in %.3f s\n", capturedCount, elapsed);
		printf("%.0f frames per second\n", capturedCount / elapsed);
		printf("%.0f triangles per second\n", capturedCount * TRIANGLES_PER_FRAME / elapsed);
	}

	if (option.stats) {
		printf("%llu command bytes and %llu data bytes over SPI\n",
		       (unsigned long long)simDisplay.commandBytes, (unsigned long long)simDisplay.dataBytes);
		if (capturedCount > 0 && !option.everyRow)
			printf("%llu data bytes per frame\n",
			       (unsigned long long)simDisplay.dataBytes / capturedCount);
	}

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
	static uint8_t rgb[3 * WIDTH * HEIGHT];
	struct timespec frameTime = { 0, 1000000000L / option.fps };

	if (!option.everyRow && framesCompleted == 0)
		return;

	if (option.hash)
		printf("%08x\n", display_hash());

	if (option.play) {
		read_display(rgb);
		draw_in_terminal(rgb);
		nanosleep(&frameTime, NULL);
	}

	if (option.gifPath || option.ppmPrefix) {
		captured[capturedCount] = malloc(sizeof(rgb));
		if (!captured[capturedCount]) {
			fprintf(stderr, "out of memory\n");
			exit(1);
		}
		read_display(captured[capturedCount]);
	}

	if (++capturedCount == option.frames)
		finish();
}

static void
usage(FILE *file, const char *program)
{
	fprintf(file,
		"usage: %s [options]\n"
		"\n"
		"Runs the firmware against a model of the SSD1331 display. Without\n"
		"options it plays the animation in the terminal until you press Ctrl-C.\n"
		"\n"
		"  --play          play in the terminal (needs 24-bit colour, 96 columns)\n"
		"  --gif FILE      write an animated GIF\n"
		"  --ppm PREFIX    write PREFIX000.ppm, PREFIX001.ppm, ...\n"
		"  --frames N      stop after N frames (default 36, one full turn, or no\n"
		"                  limit when only playing)\n"
		"  --scale N       enlarge GIF and PPM output N times (default 8)\n"
		"  --fps N         frame rate of the GIF and of --play (default 12)\n"
		"  --rows          capture after every row sent to the display\n"
		"  --hash          print a hash of each frame\n"
		"  --bench         print how fast the frames were rendered\n"
		"  --stats         print how many bytes went over SPI\n"
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

		if (strcmp(a, "--play") == 0)
			option.play = 1;
		else if (strcmp(a, "--rows") == 0)
			option.everyRow = 1;
		else if (strcmp(a, "--hash") == 0)
			option.hash = 1;
		else if (strcmp(a, "--bench") == 0)
			option.bench = 1;
		else if (strcmp(a, "--stats") == 0)
			option.stats = 1;
		else if (strcmp(a, "--console") == 0)
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
		else if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) {
			usage(stdout, argv[0]);
			return 0;
		} else {
			fprintf(stderr, "%s: unknown option or missing value: %s\n\n", argv[0], a);
			usage(stderr, argv[0]);
			return 2;
		}
	}

	if (!option.gifPath && !option.ppmPrefix && !option.hash && !option.bench && !option.stats)
		option.play = 1;
	if (option.frames == 0 && (option.gifPath || option.ppmPrefix || option.hash || option.bench || option.stats))
		option.frames = 36;

	if (option.gifPath || option.ppmPrefix) {
		captured = calloc((size_t)option.frames, sizeof(*captured));
		if (!captured) {
			fprintf(stderr, "out of memory\n");
			return 1;
		}
	}

	if (option.play) {
		signal(SIGINT, on_interrupt);
		fputs("\x1b[2J\x1b[?25l", stdout);
	}

	ssd1331_reset(&simDisplay);	/* the state of the controller after power-on */
	clock_gettime(CLOCK_MONOTONIC, &startTime);
	firmware_main();

	/* Some builds of the firmware draw one frame and return. */
	finish();

	return 0;
}
