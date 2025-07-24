# OLED Bouncing Ball Demo for CC3200

A physics-based bouncing ball simulation for the Texas Instruments CC3200 microcontroller, featuring accelerometer-controlled movement on a 128x128 SSD1351 OLED display.

![Project Demo](https://img.shields.io/badge/Platform-CC3200-blue) ![Language](https://img.shields.io/badge/Language-C-brightgreen) ![Display](https://img.shields.io/badge/Display-SSD1351%20OLED-orange)

## 🎯 Overview

This project demonstrates real-time physics simulation on embedded hardware by creating a bouncing ball that responds to device tilt using an accelerometer. The ball exhibits realistic physics with:

- **Accelerometer-based control**: Tilt the device to apply forces to the ball
- **Physics simulation**: Velocity, acceleration, friction, and collision detection
- **Real-time graphics**: Smooth animation on a 128x128 color OLED display
- **Boundary collision**: Ball bounces off screen edges with energy loss

## 🏗️ Hardware Requirements

### Primary Components
- **Texas Instruments CC3200 Development Board**
- **1.5" 128x128 RGB OLED Display (SSD1351 driver)**
- **I2C Accelerometer** (connected to address 0x18)

### Pin Configuration
| Function | CC3200 Pin | Connection |
|----------|------------|------------|
| SPI MOSI | Pin 7      | OLED Data  |
| SPI CLK  | Pin 5      | OLED Clock |
| SPI CS   | Pin 8      | OLED CS    |
| I2C SDA  | Pin 2      | Accelerometer SDA |
| I2C SCL  | Pin 1      | Accelerometer SCL |
| UART TX  | Pin 55     | Debug Output |
| UART RX  | Pin 57     | Debug Input |

## 🛠️ Software Architecture

### Core Components

#### 1. **Main Application** (`main.c`)
- **Physics Engine**: Implements velocity-based movement with acceleration input
- **Collision Detection**: Boundary checking with realistic bounce physics
- **Sensor Interface**: I2C communication with accelerometer
- **Display Management**: Real-time screen updates with efficient rendering

#### 2. **Graphics Library** (`Adafruit_GFX.c/h`)
- **Primitive Drawing**: Lines, rectangles, circles, triangles
- **Text Rendering**: Built-in font support with configurable scaling
- **Color Management**: 16-bit RGB565 color space
- **Optimized Algorithms**: Fast drawing routines for embedded performance

#### 3. **OLED Driver** (`Adafruit_OLED.c`, `Adafruit_SSD1351.h`)
- **SSD1351 Controller**: Full driver implementation for 128x128 OLED
- **SPI Communication**: High-speed data transfer to display
- **Color Support**: 65,536 colors (RGB565 format)
- **Hardware Acceleration**: Optimized for real-time graphics

#### 4. **Test Suite** (`oled_test.c/h`)
- **Display Testing**: Comprehensive test patterns and demos
- **Performance Benchmarks**: Frame rate and rendering tests
- **Color Verification**: RGB color space validation
- **Font Testing**: Character set and text rendering validation

### Key Features

#### Physics Simulation
```c
// Core physics loop
ballVelocity[0] = (ballVelocity[0] + xAcc) * 0.99;  // Friction applied
ballVelocity[1] = (ballVelocity[1] + yAcc) * 0.99;
ballPosition[0] += ballVelocity[0];                 // Position update
ballPosition[1] += ballVelocity[1];
```

#### Collision Detection
```c
// Boundary collision with energy loss
if (ballPosition[0] <= BALL_RADIUS) {
    ballPosition[0] = BALL_RADIUS;
    ballVelocity[0] *= -0.95;  // 5% energy loss on bounce
}
```

#### Accelerometer Integration
```c
// Scale accelerometer data to reasonable force values
int8_t xAcc = (int8_t)(((double)accData[0] / 64) * 6);
int8_t yAcc = (int8_t)(((double)accData[1] / 64) * 6);
```

## 🚀 Quick Start

### Prerequisites
- **Code Composer Studio (CCS)** v12.0 or later
- **CC3200 SDK** installed and configured
- **Hardware setup** as described above

### Build Instructions

1. **Clone/Download** this repository to your local machine

2. **Import Project** into Code Composer Studio:
   ```bash
   File → Import → Code Composer Studio → CCS Projects
   Select the project directory
   ```

3. **Configure Target**:
   - Right-click project → Properties
   - Select CC3200 target configuration
   - Verify linker command file: `cc3200v1p32.cmd`

4. **Build Project**:
   ```bash
   Project → Build All
   ```
   Or use keyboard shortcut: `Ctrl+B`

5. **Flash and Run**:
   - Connect CC3200 via USB
   - Debug → Debug As → Code Composer Studio → CC3200
   - Press F8 to run

### Configuration Options

#### Display Settings
```c
#define SSD1351WIDTH 128
#define SSD1351HEIGHT 128
#define BALL_RADIUS 4
#define SCREEN 128
```

#### Physics Parameters
```c
#define FRICTION_FACTOR 0.99    // Air resistance
#define BOUNCE_DAMPING 0.95     // Energy loss on collision
#define MAX_ACCELERATION 6      // Maximum force from accelerometer
```

#### Communication Settings
```c
#define SPI_IF_BIT_RATE 100000  // SPI speed (100 kHz)
#define I2C_MASTER_MODE_FST     // I2C fast mode
```

## 📁 File Structure

```
├── main.c                  # Main application and physics engine
├── oled_test.c            # Display test functions and demos
├── oled_test.h            # Test function prototypes and color definitions
├── Adafruit_GFX.c         # Graphics library implementation
├── Adafruit_GFX.h         # Graphics library header
├── Adafruit_OLED.c        # OLED driver implementation
├── Adafruit_SSD1351.h     # SSD1351 controller definitions
├── glcdfont.h             # Bitmap font data
├── i2c_if.c               # I2C interface implementation
├── uart_if.c              # UART interface for debugging
├── pin_mux_config.c       # Pin multiplexer configuration
├── pin_mux_config.h       # Pin configuration header
├── cc3200v1p32.cmd        # Linker command file
├── .ccsproject            # Code Composer Studio project
├── .cproject              # C/C++ project configuration
├── .project               # Eclipse project file
├── Debug/                 # Build output directory
├── .settings/             # IDE configuration
├── .launches/             # Debug launch configurations
└── targetConfigs/         # Target device configurations
```

## 🎮 Usage

### Basic Operation
1. **Power on** the CC3200 with connected OLED display
2. **Observe** the white ball appear in the center of the screen
3. **Tilt** the device to apply gravitational forces
4. **Watch** realistic physics as the ball bounces around

### Debug Output
Connect to UART (115200 baud) to see real-time accelerometer values:
```
X Acc: 23, Y Acc: -15
X Acc: 18, Y Acc: -12
X Acc: 25, Y Acc: -18
```

### Test Functions
Uncomment test calls in `main()` to run display demos:
```c
// Add before main physics loop
testlines(WHITE);           // Line drawing test
testfillcircles(10, BLUE); // Circle filling test
lcdTestPattern();          // Color bar test
testHelloWorld(GREEN);     // Text rendering test
```

## 🔧 Customization

### Modify Ball Physics
```c
// Adjust in main.c
int BALL_RADIUS = 6;           // Larger ball
double FRICTION = 0.95;        // More air resistance
double BOUNCE_ENERGY = 0.80;   // More energy loss on bounce
```

### Change Colors
```c
// Available colors in oled_test.h
#define BALL_COLOR    RED      // Red ball
#define BACKGROUND    BLUE     // Blue background
```

### Add Multiple Balls
Extend the arrays:
```c
int ballPosition[MAX_BALLS][2];
int8_t ballVelocity[MAX_BALLS][2];
```

### Modify Accelerometer Sensitivity
```c
// Scale factor for accelerometer input
int8_t xAcc = (int8_t)(((double)accData[0] / 32) * 12);  // More sensitive
```

## 🐛 Troubleshooting

### Common Issues

#### Display Not Working
- **Check SPI connections**: Verify MOSI, CLK, CS pins
- **Verify power supply**: Ensure 3.3V to OLED
- **Check initialization**: Call `Adafruit_Init()` before drawing

#### Accelerometer Not Responding
- **I2C address**: Verify accelerometer is at address 0x18
- **Pull-up resistors**: Ensure I2C lines have 4.7kΩ pull-ups
- **Check connections**: SDA and SCL properly connected

#### Ball Movement Issues
- **Accelerometer calibration**: May need offset correction
- **Physics parameters**: Adjust friction and bounce factors
- **Boundary checking**: Verify screen dimensions match constants

#### Build Errors
- **SDK path**: Verify CC3200 SDK is properly installed
- **Include paths**: Check all header files are accessible
- **Linker script**: Ensure `cc3200v1p32.cmd` is in project

### Performance Optimization

#### Frame Rate Improvements
```c
// Reduce delay in main loop
// Use hardware acceleration where available
// Minimize floating-point operations
```

#### Memory Usage
```c
// Current usage: ~2KB RAM
// Stack usage: ~1KB
// Heap usage: Minimal (static allocation)
```

## 📚 API Reference

### Graphics Functions
```c
void fillScreen(unsigned int color);
void fillCircle(int x, int y, int radius, unsigned int color);
void drawLine(int x0, int y0, int x1, int y1, unsigned int color);
void drawRect(int x, int y, int w, int h, unsigned int color);
void fillRect(int x, int y, int w, int h, unsigned int color);
```

### Accelerometer Functions
```c
int8_t* ReadAccData();  // Returns [x, y] acceleration values
```

### Test Functions
```c
void testlines(unsigned int color);
void testfillcircles(unsigned char radius, unsigned int color);
void lcdTestPattern(void);
void testHelloWorld(unsigned int color);
```

## 🤝 Contributing

1. **Fork** the repository
2. **Create** a feature branch: `git checkout -b feature/new-physics`
3. **Commit** changes: `git commit -am 'Add gravity simulation'`
4. **Push** to branch: `git push origin feature/new-physics`
5. **Submit** a Pull Request

### Development Guidelines
- **Follow** existing code style and formatting
- **Add** comments for new physics algorithms
- **Test** on actual hardware before submitting
- **Update** documentation for new features

## 📄 License

This project is based on Adafruit libraries and follows their BSD license terms. All original code additions are provided under the same BSD license.

```
BSD License - See individual source files for full license text
Adafruit contributions: Copyright (c) Adafruit Industries
Project modifications: Open source contributions welcome
```

## 🙏 Acknowledgments

- **Adafruit Industries** - Graphics libraries and OLED driver
- **Texas Instruments** - CC3200 SDK and development tools
- **Contributors** - Community improvements and bug fixes

---

**Project Status**: ✅ Active Development  
**Last Updated**: January 2024  
**Tested Platforms**: CC3200 LaunchPad, Custom CC3200 boards
