"""Regenerate the README figures from the firmware running in the simulator.

    make figures

Every rendered pixel in these figures was read from the display model after the
firmware had sent it there. This script enlarges, crops and labels the frames
and draws nothing of its own inside them. Needs Pillow.
"""

import argparse
import os
import subprocess

from PIL import Image, ImageChops, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

WIDTH, HEIGHT = 96, 64
BG = (13, 17, 23)
PANEL = (22, 27, 34)
BORDER = (48, 54, 61)
INK = (240, 246, 252)
MUTED = (139, 148, 158)
ACCENT = (57, 211, 215)


def font(size, weight="Regular", mono=False):
    if mono:
        candidates = ["/usr/share/fonts/truetype/dejavu/DejaVuSansMono%s.ttf" % ("-Bold" if weight == "Bold" else "")]
    else:
        candidates = ["/usr/share/fonts/truetype/lato/Lato-%s.ttf" % weight,
                      "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if weight != "Regular" else "")]
    for path in candidates:
        if os.path.isfile(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def simulate(build, binary, tmp, name, frames, rows=False):
    """Run a simulator and return what the display showed, one 96x64 image per capture."""
    prefix = os.path.join(tmp, name + "-")
    command = [os.path.join(build, binary), "--ppm", prefix, "--scale", "1", "--frames", str(frames)]
    if rows:
        command.append("--rows")
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
    images = []
    for i in range(frames):
        path = "%s%03d.ppm" % (prefix, i)
        if not os.path.isfile(path):
            break
        images.append(Image.open(path).convert("RGB"))
    return images


def replicate(image, size):
    """Enlarge by an integer factor without smoothing. BOX repeats pixels exactly, like NEAREST."""
    return image.resize(size, Image.BOX)


def enlarge(image, scale, grid=True):
    """Enlarge without smoothing. With grid, darken one line between pixels, like the gaps of an OLED."""
    big = replicate(image, (image.width * scale, image.height * scale))
    if grid and scale >= 4:
        cell = Image.new("L", (scale, scale), 255)
        cell.paste(102, (scale - 1, 0, scale, scale))
        cell.paste(102, (0, scale - 1, scale, scale))
        lines = Image.new("L", big.size)
        for y in range(0, big.height, scale):
            for x in range(0, big.width, scale):
                lines.paste(cell, (x, y))
        big = ImageChops.multiply(big, Image.merge("RGB", (lines, lines, lines)))
    return big


def framed(canvas, image, xy, radius=10, pad=10):
    """Paste a display image on the canvas with a bezel around it."""
    x, y = xy
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle((x - pad, y - pad, x + image.width + pad - 1, y + image.height + pad - 1),
                           radius=radius, fill=(0, 0, 0), outline=BORDER, width=2)
    canvas.paste(image, xy)


def save_gif(path, frames, durations):
    """Write an animated GIF with one palette shared by all frames."""
    strip = Image.new("RGB", (frames[0].width, frames[0].height * min(len(frames), 12)))
    step = max(1, len(frames) // 12)
    for i, frame in enumerate(frames[::step][:12]):
        strip.paste(frame, (0, i * frame.height))
    palette = strip.quantize(colors=128, method=Image.MEDIANCUT, dither=Image.NONE)
    indexed = [frame.quantize(palette=palette, dither=Image.NONE) for frame in frames]
    indexed[0].save(path, save_all=True, append_images=indexed[1:], duration=durations, loop=0,
                    optimize=False, disposal=1)


def make_hero(spin, path):
    """One full turn of the model on the 96x64 display."""
    scale, margin, footer = 9, 30, 46
    size = (WIDTH * scale + 2 * margin, HEIGHT * scale + 2 * margin + footer)
    label, small = font(19, mono=True), font(19, mono=True)
    frames = []
    for i, image in enumerate(spin):
        canvas = Image.new("RGB", size, BG)
        framed(canvas, enlarge(image, scale), (margin, margin))
        draw = ImageDraw.Draw(canvas)
        y = margin + HEIGHT * scale + 30
        draw.text((margin, y), "96 x 64 pixel OLED, 16-bit colour", font=label, fill=MUTED, anchor="lm")
        draw.text((size[0] - margin, y), "rotation %3d°" % (i * 10), font=small, fill=INK, anchor="rm")
        frames.append(canvas)
    save_gif(path, frames, 90)


def make_scanlines(rows, path):
    """The display filling up one row at a time, next to the single row the firmware holds in RAM."""
    scale, margin = 6, 30
    strip_h, gap, header = 30, 74, 34
    size = (WIDTH * scale + 2 * margin + 20, header + HEIGHT * scale + gap + strip_h + 2 * margin + 30)
    title, mono = font(19, "Bold"), font(17, mono=True)
    x0 = margin + 20
    frames, durations = [], []
    for i, image in enumerate(rows):
        row = i % HEIGHT
        canvas = Image.new("RGB", size, BG)
        draw = ImageDraw.Draw(canvas)

        y0 = margin + header
        draw.text((x0, margin + 6), "Display RAM in the SSD1331", font=title, fill=INK, anchor="lm")
        draw.text((x0 + WIDTH * scale, margin + 6), "12,288 bytes", font=mono, fill=MUTED, anchor="rm")
        framed(canvas, enlarge(image, scale), (x0, y0))

        # Marker beside the row that has just arrived.
        ym = y0 + row * scale + scale // 2
        draw.polygon([(x0 - 26, ym - 7), (x0 - 14, ym), (x0 - 26, ym + 7)], fill=ACCENT)

        y1 = y0 + HEIGHT * scale + gap
        draw.text((x0, y1 - 24), "The row in the microcontroller's RAM", font=title, fill=INK, anchor="lm")
        draw.text((x0 + WIDTH * scale, y1 - 24), "192 bytes", font=mono, fill=ACCENT, anchor="rm")
        strip = replicate(image.crop((0, row, WIDTH, row + 1)), (WIDTH * scale, strip_h))
        framed(canvas, strip, (x0, y1), radius=8, pad=6)
        draw.text((x0, y1 + strip_h + 28), "row %2d of %d, frame %d" % (row + 1, HEIGHT, i // HEIGHT + 1),
                  font=mono, fill=MUTED, anchor="lm")

        frames.append(canvas)
        durations.append(900 if row == HEIGHT - 1 else 40)
    save_gif(path, frames, durations)


def depth_values(image):
    """Recover z from a frame of the depth-view build, which stores it as a 16-bit pixel."""
    values = []
    pixels = image.load()
    for y in range(image.height):
        for x in range(image.width):
            r, g, b = pixels[x, y]
            values.append(((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3)) / 256.0)
    return values


def make_depth_test(no_test, depth, tested, path):
    """Three views of the same frame: without the depth test, the depth itself, and with the test."""
    z = depth_values(depth)
    covered = [v for v in z if v > 0]
    near, far = min(covered), max(covered)
    grey = Image.new("RGB", (WIDTH, HEIGHT))
    grey.putdata([(0, 0, 0) if v == 0 else (int(255 - 190 * (v - near) / (far - near)),) * 3 for v in z])

    # Crop all three to the same box around the model.
    boxes = [image.getbbox() for image in (no_test, grey, tested)]
    left, top = min(b[0] for b in boxes) - 4, min(b[1] for b in boxes) - 4
    right, bottom = max(b[2] for b in boxes) + 4, max(b[3] for b in boxes) + 4

    scale, margin, gap, header, footer = 10, 30, 34, 40, 40
    w, h = (right - left) * scale, (bottom - top) * scale
    size = (3 * w + 2 * gap + 2 * margin, h + header + footer + 2 * margin)
    canvas = Image.new("RGB", size, BG)
    draw = ImageDraw.Draw(canvas)
    title, note = font(22, "Bold"), font(18)
    panels = [
        (no_test, "Depth test off", "the triangle drawn last wins"),
        (grey, "Depth per pixel", "brighter is nearer, never stored"),
        (tested, "Depth test on", "the nearest triangle wins"),
    ]
    for i, (image, heading, caption) in enumerate(panels):
        x = margin + i * (w + gap)
        draw.text((x + w // 2, margin + 8), heading, font=title, fill=INK, anchor="mm")
        framed(canvas, enlarge(image.crop((left, top, right, bottom)), scale), (x, margin + header))
        draw.text((x + w // 2, margin + header + h + 30), caption, font=note, fill=MUTED, anchor="mm")
    canvas.save(path, optimize=True)
    return near, far


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", default=os.path.join(ROOT, "build"), help="folder with the simulators")
    args = parser.parse_args()

    tmp = os.path.join(args.build, "fig")
    os.makedirs(tmp, exist_ok=True)

    spin = simulate(args.build, "firmware", tmp, "spin", 36)
    rows = simulate(args.build, "firmware", tmp, "rows", 2 * HEIGHT, rows=True)
    no_test = simulate(args.build, "firmware-noztest", tmp, "noztest", 1)[0]
    depth = simulate(args.build, "firmware-depth", tmp, "depth", 1)[0]

    make_hero(spin, os.path.join(HERE, "hero.gif"))
    make_scanlines(rows, os.path.join(HERE, "scanlines.gif"))
    near, far = make_depth_test(no_test, depth, spin[0], os.path.join(HERE, "depth-test.png"))
    print("depth in the first frame runs from %.2f to %.2f" % (near, far))


if __name__ == "__main__":
    main()
