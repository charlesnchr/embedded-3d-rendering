<h1 align="center">embedded-3d-rendering</h1>

<p align="center">
  <b>A 3D graphics pipeline for a microcontroller with 2 kB of RAM.</b><br>
  Vertex transform, scanline rasteriser and a depth test for every pixel, in one C file.<br>
  No GPU, no FPU and no frame buffer.
</p>

<p align="center">
  <a href="https://github.com/charlesnchr/embedded-3d-rendering/actions/workflows/build.yml"><img src="https://github.com/charlesnchr/embedded-3d-rendering/actions/workflows/build.yml/badge.svg" alt="Build status"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-blue" alt="MIT license"></a>
  <img src="https://img.shields.io/badge/target-Cortex--M0%2B-0091bd" alt="Target: ARM Cortex-M0+">
  <img src="https://img.shields.io/badge/RAM-2%20kB-39d3d7" alt="2 kB of RAM">
  <a href="Reports/Charles%20Christensen%20-%20Final%20project.pdf"><img src="https://img.shields.io/badge/report-PDF-b31b1b" alt="Project report"></a>
</p>

<p align="center">
  <img src="fig/hero.gif" alt="Two tetrahedra turning on a 96 by 64 pixel display, one passing in front of the other" width="760">
</p>
<p align="center">
  <sub>The firmware from 2018, compiled as it is and run against a model of the display. <code>make run</code> plays this in your terminal.</sub>
</p>

One frame for this display takes 12,288 bytes. The microcontroller has 2,048 bytes of RAM. So the renderer never holds a frame. It works out one row of 96 pixels, sends those 192 bytes to the display over SPI and starts on the next row. A depth buffer would not fit either, so there is none. For every pixel the rasteriser computes the depth of each triangle that covers it, keeps the nearest and forgets the number.

I wrote this in 2018 for the Cambridge course 4B25, Embedded Systems for the Internet of Things. OpenGL ES assumes a GPU and far more memory than a small microcontroller has, and I wanted to know how much of that kind of pipeline fits on a Freedom KL03 board, a 48 MHz Cortex-M0+ with 32 kB of flash. Most of it does.

- **192 bytes of pixels.** The only pixels in RAM are the row being drawn. The full frame is in the display controller, which the firmware fills from the top down.
- **Depth test without a depth buffer.** Triangles that overlap come out right, at the cost of one interpolation per triangle per pixel.
- **784 bytes of stack for the whole renderer.** That covers the 8 triangles and their colours, two matrices, the edge table, the active edge list and the row buffer.
- **4 kB of code.** The firmware source compiles to 4,114 bytes of Thumb code at `-Os`. Software floating point from libgcc brings it to 17.5 kB.
- **One file to read.** [`rasterise.c`](rasterise.c) is 449 lines of C. The only hardware it touches is an SPI port and three GPIO pins.
- **Runs on a laptop.** `make` builds the same source against a model of the display, so you can watch it without the board. One desktop core renders more than 10,000 frames per second.
- **Measured on the board.** The KL03 draws the scene at 2.6 to 5.8 frames per second, depending on which stages are switched on.

## Try it

```bash
git clone https://github.com/charlesnchr/embedded-3d-rendering && cd embedded-3d-rendering
make
make run
```

You need a C compiler and make, nothing else. `make run` plays the turning model in the terminal until you press Ctrl-C. The terminal has to support 24-bit colour and be at least 96 columns wide.

`make gif` writes one full turn to `out/spin.gif`. For single frames, `build/firmware --ppm frame- --frames 36` writes `frame-000.ppm` to `frame-035.ppm`, and `build/firmware --help` lists the other options.

## How it works

<p align="center">
  <img src="fig/pipeline.svg" alt="Diagram of the pipeline. Once per frame: triangles, vertex stage, edge table. Once per row: active edges, depth test per pixel, a 192-byte row buffer that is sent over SPI to the display." width="900">
</p>

The scene is 8 triangles, each vertex stored as three bytes. Once per frame the vertex stage rotates the vertices, multiplies them by a view and projection matrix, divides by w and scales the result to the 96 by 64 pixel screen. The screen coordinates go back into bytes. To turn the model the firmware applies one fixed matrix for 10 degrees as many times as needed, which is slow but avoids a sine table.

From the transformed triangles it builds an edge table. Every edge that is not horizontal gets its top row, its bottom row, the x position at the top and its slope.

Then the rasteriser walks down the screen one row at a time. Edges that start on the current row join the active list. The list is sorted by triangle, so the two edges of a triangle that cross the row end up next to each other and bound the span of pixels that the triangle covers. When the row is done, each active edge moves along its slope by one row and edges that have ended drop out.

<p align="center">
  <img src="fig/scanlines.gif" alt="The display filling up one row at a time while a strip below shows the single row held in the microcontroller's RAM" width="560">
</p>

The display controller is what makes this possible. An SSD1331 has its own RAM for the full frame and a write pointer that advances by itself, so the firmware sets the drawing window once at start-up and after that only sends pixels. Each row is 192 bytes of 16-bit colour and a frame is 64 of them.

For each pixel in a row, the rasteriser goes through the spans that cover it. It interpolates z from the three vertices of the triangle with barycentric weights and keeps the colour of the nearest one. The middle panel below shows that depth, which the firmware computes and throws away. It comes from a build with a [three-line patch](fig/depth-view.patch) that writes z into the pixel in place of the colour.

<p align="center">
  <img src="fig/depth-test.png" alt="The same frame three times: without the depth test the far tetrahedron is drawn over the near one, then the depth of each pixel in grey, then the correct picture with the depth test" width="900">
</p>

The fragment stage is the part that did not fit. Every triangle has one flat colour, and the shading in the pictures comes from giving each face its own brightness.

## On the board

<p align="center">
  <img src="fig/hardware.jpg" alt="A Freedom KL03 board on a breadboard, wired to a small OLED display that shows the model in white and to a current sensor" width="620">
</p>

The target is an NXP FRDM-KL03Z board with an Adafruit 0.96 inch colour OLED, which is driven by an SSD1331 over SPI. The numbers below are from the [project report](Reports/Charles%20Christensen%20-%20Final%20project.pdf), measured in 2018 at an SPI rate of 4000 kBd and averaged over 50 frames. I read them off the chart, which has three runs per configuration.

| Vertex stage | Depth test | Frames per second |
| --- | --- | :---: |
| Skipped, vertices transformed in advance | Off | 5.6 to 5.8 |
| On | Off | 4.4 to 5.0 |
| On, with a 300 degree rotation | Off | 4.0 |
| Skipped, vertices transformed in advance | On | 3.7 to 4.0 |
| On | On | 3.2 to 3.4 |
| On, with a 300 degree rotation | On | 2.6 |

<p align="center">
  <img src="fig/benchmark.png" alt="Bar chart of the frame rate for the six configurations in the table, three measurements each" width="760">
</p>

The depth test costs more than the vertex stage, because it runs for every pixel and the vertex stage runs for 24 vertices. The variants of the firmware in `Implementations/C` are three of these configurations.

Two more results from the report:

- At a slow SPI clock the transfer is the bottleneck. Raising the rate from 50 kBd to 6000 kBd took the frame rate from about 0.4 to 3.6 frames per second, and a fit to Amdahl's law puts 91% of the frame time at 50 kBd in the transfer.
- The current was about 10 mA with the model on a black background, against about 57 mA with every pixel white. An OLED spends its power on lit pixels, so a small model on black is cheap to show.

## Size and speed

| | | How to reproduce |
| --- | --- | --- |
| Pixels held in RAM | 192 bytes, one row of 96 pixels | `#define N 192` in the source |
| One frame in the display | 12,288 bytes, 6 times the RAM of the KL03 | `build/firmware --stats` |
| Stack frame of the renderer | 784 bytes | `make size` |
| Code of the firmware source | 4,114 bytes of Thumb code | `make size` |
| With libgcc software floating point | 17,512 bytes | `make size` |
| Speed on a desktop | more than 10,000 frames or 80,000 triangles per second | `make bench` |
| Speed on the KL03 | 2.6 to 5.8 frames per second | project report, 2018 |

The size figures are for the Cortex-M0+ with `arm-none-eabi-gcc` 14.2 at `-Os`. They leave out the Kinetis SDK drivers and `printf`, which are not in this repository. The desktop speed includes the display model and uses one core. A Ryzen 7 5800X manages 10,000 to 14,000 frames per second depending on how busy the other cores are, and the Linux runners of the build workflow about 21,000.

## What `make` runs

The simulator does not contain a second copy of the renderer. It compiles the firmware files in `Implementations/C` without changing them, and replaces what is underneath.

- `sim/ksdk/` has stand-in headers for the Kinetis SDK calls that the firmware makes. GPIO writes set a pin level, the delays return at once.
- `sim/ssd1331.c` is a model of the display controller. It parses the command bytes, keeps the address window and the write pointer, and stores pixel data in a 96 by 64 array, as the chip does.
- `sim/main.c` reads that array after each frame and writes a GIF, PPM files or the terminal animation.

So the picture is built from the bytes that the firmware would put on the SPI wire, 48 command bytes at start-up and then 12,288 data bytes per frame. `make check` renders a full turn with every variant of the firmware and compares a hash of each frame with the recorded one. The build workflow runs it with GCC and Clang on Linux and with Clang on macOS, and it compiles the firmware for the Cortex-M0+ to report the sizes above.

The model covers what decides the picture. It does not model timing, the analogue settings of the panel, chip select or the reset line.

## The MATLAB prototype

I wrote the pipeline in MATLAB first, in [`Implementations/MATLAB/rasterise.m`](Implementations/MATLAB/rasterise.m), and ported it to C once it worked. The prototype uses the same algorithm with a full frame and a z-buffer in memory, so it renders at any resolution. Below is its output at the display's 96 by 64 pixels and at 960 by 640.

<p align="center">
  <img src="Figures/geom.png" alt="The two tetrahedra rendered by the MATLAB prototype, as vector outlines and as raster images at 96 by 64 and 960 by 640 pixels" width="700">
</p>

## Repository layout
(1) The three reports related to the project are present in the `Reports` folder. Most importantly the final project report showing the results of the project. The project proposal and interim report contain more details on the project's subgoals. 

(2) Plots of results, diagram of pipeline and pictures of hardware setup are present in the folder `Figures`.

(3) The source code is in `Implementations/C` and a more platform-independent version stripped of SDK-specific calls is in the root folder, `rasterise.c`. There are a few versions of the implementation. A version without vertex shader based on pre-computed static vertices are in `Implementations/C/NoVert/i2c_comm_master.c`, and a version without Z depth testing is in `Implementations/C/NoZtest/i2c_comm_master.c`.

(4) A MATLAB implementation of the exact same pipeline, which was developed as a prototype before the embedded version, is in `Implementations/MATLAB/rasterise.m`. 
