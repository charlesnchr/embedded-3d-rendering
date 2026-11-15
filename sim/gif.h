/* A small GIF89a writer for animations with at most 256 colours. */
#ifndef SIM_GIF_H
#define SIM_GIF_H

#include <stdint.h>

/*
 * Writes frameCount frames of width x height pixels, each frame 3 bytes per
 * pixel in RGB order, enlarged by an integer scale without smoothing. delay is
 * the time each frame is shown, in hundredths of a second. The animation
 * loops. Returns 0 on success, -1 if the file cannot be written and -2 if the
 * frames use more than 256 colours.
 */
int gif_write(const char *path, int width, int height, uint8_t *const *frames, int frameCount,
	      int scale, int delay);

#endif
