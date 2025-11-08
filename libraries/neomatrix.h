#pragma once
#include <cstdint>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/pio.h"

// Default pins; override via -DNEOMATRIX_PIN=... -DNEOMATRIX_PWR_PIN=...
#ifndef NEOMATRIX_PIN
#define NEOMATRIX_PIN 7        // GPIO to NeoPixel DIN
#endif
#ifndef NEOMATRIX_PWR_PIN
#define NEOMATRIX_PWR_PIN 10     // PropMaker enable (drive HIGH)
#endif

class NeoMatrix {
public:
    NeoMatrix(uint8_t width, uint8_t height);

    // Set up power-enable pin and PIO WS2812 driver
    bool init();

    // Pixel API (RGB packed as 0xRRGGBB)
    void clear_pixels();
    void set_pixel(uint8_t row, uint8_t col, uint32_t rgb);
    void write();  // push current buffer to LEDs

    static inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
        return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
    }

private:
    uint8_t w, h;
    std::vector<uint32_t> buf;

    PIO pio = pio0;
    int sm = -1;
    uint offset = 0;

    uint32_t index_for(uint8_t row, uint8_t col) const; // serpentine mapping
    void put_pixel_grb(uint32_t grb);
};
