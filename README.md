# **Driver Lab – Embedded Systems**
**Author:** Alejandro Fluitt Martinez  
**Date:** October 26, 2025  
 

 (:

## **1. Overview**
This project implements two hardware drivers for the Feather RP2040 board:
- **LIS3DH Driver:** communicates with the onboard accelerometer over the I2C bus (pins SDA = GPIO2, SCL = GPIO3, using `i2c1` instance).  
- **NeoMatrix Driver:** controls an 8×8 WS2812 (NeoPixel) LED matrix using PIO on GPIO7 for data (DIN) and GPIO10 for power enable (EN).

These drivers are integrated into a demo application called **level.cpp**, which turns the system into a simple **digital bubble level**.  
When the board is flat, the center LEDs light up green; when tilted, a red LED moves according to the accelerometer readings.

---

## **2. Project Structure**
whatever library using the pico extention/
├── libraries/
│ ├── LIS3DH.cpp # Accelerometer driver 
│ ├── LIS3DH.h
│ ├── neomatrix.cpp # NeoMatrix driver 
│ ├── neomatrix.h
│ └── neopixel.pio # PIO assembly for LED timing
│
├── src/
│ └── level.cpp # Main demo application 
│
├── CMakeLists.txt # Pico build configuration
├── pico_sdk_import.cmake
├── README.md # Documentation (this file)
## **3. Build Instructions**

**Prereqs**
- Pico SDK located **one level up** at `../pico-sdk`
- RP2040 toolchain on PATH: `arm-none-eabi-gcc`, `arm-none-eabi-objcopy` (and newlib)
- A build tool (macOS: run `xcode-select --install` once to get `make`)

**Build**
```bash
rm -rf build
cmake -S . -B build -G "Unix Makefiles"
cmake --build build -j


FULL TRANSPARENCY IT WOULD NOT BUILD LOCALLY FOR ME DUE TO WEIRD PROBLEMS WITH THE ARM TOOLCHAIN AND MY VERSION OF APPLE COMPUTER PLEASE IF IT DOESNT WORK CAN YOU TRY THE EXTENSION AND GIVE ME SOME CREDIT IM SORRY ):

## **4. How It Works**
- **LIS3DH Driver:**  
Uses I2C1 to read acceleration data from the LIS3DH sensor at 400kHz.  
Each axis (X, Y, Z) is scaled to represent ±2g range.  

- **NeoMatrix Driver:**  
Uses a PIO state machine to generate WS2812 LED data at ~800kHz.  
Data is written in GRB order, 24 bits per pixel.  
Power (EN) is controlled through GPIO10 to enable the 5V rail to the matrix.  

- **Level Application:**  
Continuously reads accelerometer values, applies smoothing with a low-pass filter, and updates the LED position to visualize tilt.  
When both X and Y acceleration are within ±0.1g, the center LEDs turn green (level zone).  
