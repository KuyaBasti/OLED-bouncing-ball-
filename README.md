# OLED Bouncing Ball

<p align="center"><img src="docs/system-overview.svg" alt="System overview of the OLED Bouncing Ball firmware on a CC3200 Cortex-M4. main.c does the bus bring-up and runs the frame loop: erase, read tilt, move, draw. At boot it uses pin_mux_config.c for pin routing and calls the Adafruit_OLED.c SSD1351 driver directly to initialise and clear the screen. Every frame it gets X and Y tilt bytes from the polled I2C master in i2c_if.c, prints an X/Y readout through Report() in uart_if.c, and erases and draws a radius-4 ball through Adafruit_GFX.c, which turns fillCircle into vertical lines for Adafruit_OLED.c. The oled_test.c demo suite uses both graphics files but is compiled and never called. Below the firmware: a PC serial terminal over USB on UART0 at 115200 8N1, the on-board BMA222 accelerometer on 400 kHz I2C at address 0x18 with a burst read of registers 0x02 to 0x05, and an external 128×128 RGB565 SSD1351 OLED on 100 kHz mode-0 SPI plus GPIO DC, CS and RST lines." width="100%"></p>

A tilt-controlled ball on real hardware. A **TI CC3200 LaunchPad** (Cortex-M4) polls its **on-board BMA222 accelerometer over 400 kHz I2C**, folds the tilt into ball velocity with **friction and energy-losing bounces**, and animates the ball on a **128×128 SSD1351 color OLED** driven over a **100 kHz SPI link** — while streaming the raw acceleration readings to a **115200-baud UART console**. The firmware banner calls itself *"Sliding Ball"*; everything runs from a single polled `while(FOREVER)` loop with no interrupt handlers, no timers, and no framebuffer.

The interesting part isn't the physics — it's the **bandwidth budget**. At 100 kHz SPI, repainting the full screen moves 32,768 data bytes: **at least 2.6 seconds of raw shift time**. So the whole game is built around never repainting: each frame erases the ball by drawing a black circle over its old position and draws a white one at the new position — roughly **520 bytes per frame (340 of them pixel data) instead of 32,768**. Even the "friction" is shaped by the hardware: velocity lives in an `int8_t`, and the `× 0.99` decay truncates back to integer, which turns nominal 1%-per-frame air resistance into a flat −1 px/frame linear decay at every practical speed.

---

## Table of Contents

1. [Wiring](#wiring)
2. [How a Frame Happens](#how-a-frame-happens)
3. [Repository Map](#repository-map)
4. [The Physics — Integer Truncation Is the Real Friction](#the-physics--integer-truncation-is-the-real-friction)
5. [The Sensor Path](#the-sensor-path)
6. [The Display Stack](#the-display-stack)
7. [The Pin Map](#the-pin-map)
8. [Build & Flash](#build--flash)
9. [Known Limitations & Sharp Edges](#known-limitations--sharp-edges)
10. [Provenance](#provenance)

---

## Wiring

![Wiring diagram. The CC3200 LaunchPad sits in the middle. PIN_01 SCL and PIN_02 SDA go to the on-board BMA222 accelerometer (I2C 400 kHz, address 0x18); PIN_55 TX goes to RX and PIN_57 RX to TX of the on-board USB debug UART (UART0, 115200 8N1). Over GSPI at 100 kHz to the external SSD1351 128×128 OLED: PIN_05 SCLK to SCLK, PIN_07 MOSI to DIN, PIN_18 CS to CS, PIN_45 DC to DC, PIN_08 RST to RST, 3V3 to VCC and GND to GND. PIN_06 MISO and PIN_50 CS are muxed in code but not connected.](docs/wiring-diagram.svg)

Every pin comes straight from [pin_mux_config.c](pin_mux_config.c) and the GPIO writes in [Adafruit_OLED.c](Adafruit_OLED.c). The BMA222 and the USB debug UART are on the LaunchPad itself; only the OLED is external. Two muxed SPI pins go nowhere: the SSD1351 is write-only, so `GSPI_MISO` (PIN_06) has nothing to say, and the hardware `GSPI_CS` (PIN_50) is muxed but unused — the display's actual chip select is bit-banged on PIN_18.

## How a Frame Happens

<p align="center"><img src="docs/how-a-frame-happens.svg" alt="Flowchart of one frame, from main.c. Boot runs once in call order: BoardInit, PinMuxConfig, UART0 8N1 via InitTerm (115200 per the docs), I2C at 400 kHz, the Sliding Ball banner, SPI at 100 kHz master mode 0 with 8-bit words; then Adafruit_Init, fillScreen(BLACK) and a first white ball at (64, 64) with zero velocity. Then every while (FOREVER) pass, with no delay or timer: erase the old ball with fillCircle using a literal radius 4 in black; ReadAccData writes register 0x02 to I2C address 0x18 without a stop and burst-reads 4 bytes, keeping byte 3 as raw x and byte 1 as raw y; scale each to a = (int8_t)(raw / 64 × 6), so −12 to +11 px/frame²; Report prints the raw bytes, not the scaled values, over UART0 to the debug console; velocity becomes (v + a) × 0.99 stored as int8_t, which drops 1 toward zero; position += velocity; x and then y are checked against the walls: at or below 4 clamps to 4, above 123 clamps to 123, and either flips that axis's velocity by × −0.95, truncated; finally fillCircle draws the white ball at the new position. Erase plus draw is 26 vertical-line writes, 522 SPI bytes, about 42 ms of shifting at 100 kHz." width="100%"></p>

There is no frame timer: the loop runs as fast as its I/O completes. An r = 4 `fillCircle` is 13 vertical lines covering ≈85 pixel writes (Bresenham with overdraw) at 2 bytes each, and every line first sends 7 bytes of `SETCOLUMN`/`SETROW`/`WRITERAM` window setup, so erase + draw ≈ 522 bytes ≈ 42 ms of raw SPI shift time — a ~24 fps ceiling before per-byte chip-select overhead and the per-frame UART print (~2 ms on the wire, mostly absorbed by the TX FIFO) are added. The pacing of the game *is* the latency of its peripherals.

## Repository Map

```text
OLED-bouncing-ball-/
├── README.md               # you are here
├── SYSTEM-DESIGN.md        # the architecture-level view
├── docs/
│   ├── system-overview.svg          # top-of-README map: firmware files, buses, hardware
│   ├── wiring-diagram.svg           # board-level schematic (pins verified from pin_mux_config.c)
│   ├── how-a-frame-happens.svg      # boot + one loop pass, step by step
│   ├── system-design-flowchart.svg  # end-to-end flowchart in SYSTEM-DESIGN.md
│   ├── one-frame.svg                # one frame as a call sequence, with its SPI byte count
│   └── byte-on-the-wire.svg         # timing of one writeCommand / writeData byte
├── main.c                  # the game: bring-up, ReadAccData, the physics loop
├── pin_mux_config.c / .h   # TI PinMux-generated muxing — the wiring source of truth
├── Adafruit_OLED.c         # SSD1351 driver: SPI transport (the lab's TODO 1–3) + init + fills
├── Adafruit_SSD1351.h      # SSD1351 command set, 128×128 panel dimensions
├── Adafruit_GFX.c / .h     # Adafruit graphics primitives, ported from Arduino C++ to C
├── glcdfont.h              # classic 5×7 ASCII font table (255 glyphs × 5 bytes)
├── oled_test.c / .h        # display demo suite — compiled but never called from main()
├── i2c_if.c                # TI SDK common: polled I2C master (STD 100k / FST 400k)
├── uart_if.c               # TI SDK common: UART console (InitTerm, Report, GetCmd)
├── cc3200v1p32.cmd         # linker script — code + data entirely in SRAM at 0x20004000
├── .project / .cproject / .ccsproject   # CCS project (cloned from the SDK spi_demo example;
│                           #   links startup_ccs.c from the SDK — it is not in this repo)
├── .launches/ .settings/ targetConfigs/ # IDE + Stellaris ICDI debug-probe config
├── README.html             # TI's spi_demo docs page — SDK leftover, not this project's docs
└── Debug/                  # build output (spi_demo.bin / .out / .map) — artifacts
```

## The Physics — Integer Truncation Is the Real Friction

The state is four small integers: `ballPosition[2]` (`int`), `ballVelocity[2]` (`int8_t`), starting at the screen center (64, 64) with zero velocity. Per frame, in [main.c](main.c):

```c
ballVelocity[0] = (ballVelocity[0] + xAcc) * 0.99;   // promoted to double, truncated back
ballPosition[0] += ballVelocity[0];
```

- **Acceleration** — `(accData / 64) * 6`: the divide-by-64 normalizes the BMA222's ±2g 8-bit reading to g units (64 LSB per g), and ×6 converts to pixels-per-frame². A full 90° tilt injects about ±6 px/frame²; the sensor's ±2g ceiling caps it at −12…+11, because the `(int8_t)` cast truncates (which also zeroes readings with |raw| ≤ 10, about 0.16 g).
- **Friction** — nominally `× 0.99`, but the product is truncated back into an `int8_t`, and `(int)(v * 0.99)` is `v − 1` for every v from 1 to 99 and `v + 1` for every v from −1 to −99. The *actual* friction law is "lose 1 px/frame of speed every frame, toward zero" — linear decay wearing an exponential costume. It also means the ball genuinely stops (velocity 1 truncates to 0) instead of asymptotically creeping.
- **Bounce** — walls clamp the position into `[4, 123]` (`BALL_RADIUS` to `SCREEN − BALL_RADIUS − 1`) and reflect velocity with `×= −0.95`, again truncated, so slow balls die at the wall quickly.

## The Sensor Path

`ReadAccData()` writes register offset `0x02` to I2C address `0x18` (decimal 24 in the code) without a stop bit, then burst-reads 4 bytes — registers `0x02–0x05`, the BMA222's X LSB/MSB and Y LSB/MSB. It keeps only the two MSBs, and it **crosses the axes on purpose**: `data[0]` (the screen-X force) is byte 3 = register `0x05`, the accelerometer's *Y* axis, and `data[1]` (screen-Y) is byte 1 = register `0x03`, the *X* axis — matching how the LaunchPad is held relative to the display's `0x74` remap. Each frame's raw readings are also printed over UART (`X Acc: %d, Y Acc: %d`), which doubles as the calibration tool: watch the numbers while tilting to see the axis mapping live.

## The Display Stack

Three layers, top to bottom:

1. **[Adafruit_GFX.c](Adafruit_GFX.c)** — device-independent primitives: Bresenham lines and circles, rectangles, triangles, 5×7 font rendering. `fillCircle` is the only one the game uses: a center vertical line plus `fillCircleHelper`'s per-column vertical lines.
2. **[Adafruit_OLED.c](Adafruit_OLED.c)** — the SSD1351 driver. `fillRect`/`drawFastVLine`/`drawFastHLine` are "hardware accelerated": set a GRAM window with `SETCOLUMN`/`SETROW`, issue `WRITERAM`, then stream `w × h` RGB565 pixels — the controller advances the write pointer itself. `Adafruit_Init` runs the 20-step SSD1351 bring-up (command unlock `0x12`/`0xB1`, mux ratio 127, remap `0x74`, contrast `C8/80/C8`, VSL `A0/B5/55`, display on).
3. **The SPI transport** — the lab's actual assignment (`TODO 1–2` in the file; `TODO 3` is the reset-pin note on `Adafruit_Init`): `writeCommand`/`writeData` drive DC (PIN_45) and CS (PIN_18) as GPIOs around a single-byte SPI transaction, with a dummy `SPIDataGet` to drain the RX FIFO. Every byte pays the full CS-toggle ceremony.

## The Pin Map

Verified line-by-line from [pin_mux_config.c](pin_mux_config.c) (generated by TI PinMux 4.0.1543) and the GPIO base/mask pairs in [Adafruit_OLED.c](Adafruit_OLED.c):

| CC3200 pin | Configured as | Role |
|---|---|---|
| PIN_05 / PIN_07 | GSPI CLK / MOSI (mode 7) | OLED clock + data |
| PIN_06 | GSPI MISO (mode 7) | muxed, unused — the SSD1351 is write-only |
| PIN_50 | GSPI CS (mode 9) | muxed, unused — CS is bit-banged instead |
| PIN_18 | GPIO 28 out (GPIOA3, 0x10) | OLED chip select |
| PIN_45 | GPIO 31 out (GPIOA3, 0x80) | OLED data/command select |
| PIN_08 | GPIO 17 out (GPIOA2, 0x02) | OLED reset |
| PIN_01 / PIN_02 | I2C SCL / SDA (mode 1) | BMA222 (on-board) |
| PIN_55 / PIN_57 | UART0 TX / RX (mode 3) | debug console, 115200 8N1 |

## Build & Flash

This is embedded firmware — you need a **CC3200 LaunchPad**, an **SSD1351 OLED wired as above**, **Code Composer Studio** (the project was built with CCS 12.5.0 and TI ARM compiler 20.2.7.LTS), and the **CC3200 SDK 1.5.0**. There is no hand-written Makefile; the build lives in the CCS project files (the checked-in `Debug/makefile` is CCS-generated and hardcodes the compiler and SDK paths under `/Applications/TI`).

1. Install the CC3200 SDK and fix the paths: [.project](.project) hardcodes `CC3200_SDK_ROOT` as `/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk` (a macOS path). Point the `CC3200_SDK_ROOT` variable at your SDK or the linked `startup_ccs.c` and the SDK headers (`uart_if.h`, `i2c_if.h`, driverlib) will not resolve.
2. **File → Import → CCS Projects**, select this directory. The project imports as **`spi_demo`** — it was cloned from the SDK's spi_demo example and keeps the name.
3. **Project → Build All**. The linker script [cc3200v1p32.cmd](cc3200v1p32.cmd) places everything in SRAM (code at `0x20004000`), so a debug load runs immediately without flashing.
4. Debug via the LaunchPad's onboard Stellaris ICDI probe ([targetConfigs/CC3200.ccxml](targetConfigs/CC3200.ccxml)), or flash `Debug/spi_demo.bin` with UniFlash.
5. Open the LaunchPad's serial port at **115200 8N1** to see the banner and the per-frame accelerometer readings.

To run the display demos in [oled_test.c](oled_test.c) (`testlines`, `testfillcircles`, `lcdTestPattern`, `testHelloWorld`, …), add `#include "oled_test.h"` and call them from `main()` — nothing invokes them in the current build.

## Known Limitations & Sharp Edges

Honest notes — all verified in the code:

- **The erase uses a literal `4`, not `BALL_RADIUS`.** `fillCircle(ballPosition[0], ballPosition[1], 4, BLACK)` erases while the draw uses `BALL_RADIUS`. Grow the radius past 4 and every frame leaves a ring of trail behind.
- **`ReadAccData`'s error path returns an integer as a pointer.** `RET_IF_ERR` does `return iRetVal;` inside a function returning `int8_t*`, so an I2C failure returns `-1` converted to a pointer — which `main` then dereferences. On healthy hardware it never trips; on a wiring fault it faults instead of degrading.
- **`main.c` never includes the graphics headers.** `Adafruit_Init`, `fillScreen`, and `fillCircle` are implicitly declared (the includes list has no `Adafruit_*.h`). It links because the real signatures happen to take `int`-compatible arguments — but it's a C89-style trap.
- **The reset-pin comment is stale.** `Adafruit_Init`'s comment says RESET is on "GPIO28, pin 18", but the code drives RESET on PIN_08 (GPIO17) and uses PIN_18 (GPIO28) as chip select. Rewire from the comment and the display stays dead.
- **Init-sequence oddities inherited from Adafruit** — `CLOCKDIV`'s `0xF1`, `PRECHARGE`'s `0x32`, and `VCOMH`'s `0x05` are sent with `writeCommand` instead of `writeData` (the panel tolerates it after the `0xB1` command unlock), and the clamp math in `fillRect`/`drawFastVLine`/`drawFastHLine` is off by one (`HEIGHT − y − 1`), silently dropping the last row/column of clipped shapes.
- **Two chip selects, one connected.** The SPI is configured `SPI_SW_CTRL_CS | SPI_CS_ACTIVEHIGH` and every byte calls `SPICSEnable`/`SPICSDisable` on the unused hardware CS (PIN_50) while the wired CS is the PIN_18 GPIO — harmless, but confusing to anyone probing pins.
- **Heap and stack churn in the hot loop** — `Report()` mallocs and frees a 256-byte buffer every frame, `ReadAccData` burns a 256-byte stack buffer for a 4-byte read, and `main` declares an unused `char acCmdStore[512]` and `int iRetVal`.
- **No timestep** — physics speed is whatever the SPI + UART latency allows; faster I/O would make the ball faster, not smoother.
- **Boot repaint is slow by design** — the single `fillScreen(BLACK)` at startup is a 32,768-byte transfer: expect a couple of seconds of visible wipe at 100 kHz.
- **`oled_test.c` is dead code in this build**, and its `delay()` comment ("delays 3*ulCount cycles") doesn't match its body (a 65,535-iteration inner loop per count).

## Provenance

This is coursework built on an embedded-systems lab scaffold — the evidence is in the files: [Adafruit_OLED.c](Adafruit_OLED.c) carries `TODO 1/2/3` prompts ("Write a function to send a command byte…"), [oled_test.h](oled_test.h) is headed `Author: rtsang` (Jan 2024), and the CCS project was cloned from the CC3200 SDK's `spi_demo` example (its TI docs page survives as [README.html](README.html)). The layers:

- **TI SDK scaffolding** — [i2c_if.c](i2c_if.c), [uart_if.c](uart_if.c), [cc3200v1p32.cmd](cc3200v1p32.cmd), the project files, and the SDK-linked `startup_ccs.c`.
- **Adafruit-derived** — [Adafruit_GFX.c](Adafruit_GFX.c)/[.h](Adafruit_GFX.h) (Copyright 2013 Adafruit Industries, BSD), [Adafruit_SSD1351.h](Adafruit_SSD1351.h) (written by Limor Fried/Ladyada), [glcdfont.h](glcdfont.h), and the drawing/init halves of [Adafruit_OLED.c](Adafruit_OLED.c); [oled_test.c](oled_test.c) is based on Adafruit's Arduino `test.ino`.
- **Implemented here** — the SPI transport (`writeCommand`, `writeData`, `Adafruit_Init`'s GPIO reset/CS handling), the pin-mux selections in [pin_mux_config.c](pin_mux_config.c), and everything gameplay in [main.c](main.c): `ReadAccData` and the physics loop.

Thanks to **Adafruit Industries** (graphics library and OLED driver, BSD license — retained in the source headers) and **Texas Instruments** (CC3200 SDK, driverlib, and tooling).

See [SYSTEM-DESIGN.md](SYSTEM-DESIGN.md) for the architecture-level view: the full data-flow diagram, the ideas behind the design, and the numbers that matter.
