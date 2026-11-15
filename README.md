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

The implementation currently renders the geometry below (images generated by identical MATLAB implementation):

![render](Figures/geom.png "Rendered geometry")

## Repository layout
(1) The three reports related to the project are present in the `Reports` folder. Most importantly the final project report showing the results of the project. The project proposal and interim report contain more details on the project's subgoals. 

(2) Plots of results, diagram of pipeline and pictures of hardware setup are present in the folder `Figures`.

(3) The source code is in `Implementations/C` and a more platform-independent version stripped of SDK-specific calls is in the root folder, `rasterise.c`. There are a few versions of the implementation. A version without vertex shader based on pre-computed static vertices are in `Implementations/C/NoVert/i2c_comm_master.c`, and a version without Z depth testing is in `Implementations/C/NoZtest/i2c_comm_master.c`.

(4) A MATLAB implementation of the exact same pipeline, which was developed as a prototype before the embedded version, is in `Implementations/MATLAB/rasterise.m`. 
