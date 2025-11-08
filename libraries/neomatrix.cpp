#include "neomatrix.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "neopixel.pio.h"   // generated from libraries/neopixel.pio

static inline uint32_t to_grb(uint32_t rgb) {
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8)  & 0xFF;
    uint8_t b =  rgb        & 0xFF;
    return (uint32_t(g) << 16) | (uint32_t(r) << 8) | uint32_t(b);
}

NeoMatrix::NeoMatrix(uint8_t width, uint8_t height)
: w(width), h(height), buf(width * height, 0) {}

bool NeoMatrix::init() {
    // Power the NeoPixels first (PropMaker EN must be HIGH)
    gpio_init(NEOMATRIX_PWR_PIN);
    gpio_set_dir(NEOMATRIX_PWR_PIN, GPIO_OUT);
    gpio_put(NEOMATRIX_PWR_PIN, 1);

    // Use PIO0 and one state machine
    pio = pio0;
    offset = pio_add_program(pio, &ws2812_program);
    sm = pio_claim_unused_sm(pio, true);

    // 800 kHz, RGB (not RGBW)
    ws2812_program_init(pio, sm, offset, NEOMATRIX_PIN, 800000.0f, false);
    return true;
}

void NeoMatrix::clear_pixels() {
    std::fill(buf.begin(), buf.end(), 0);
}

uint32_t NeoMatrix::index_for(uint8_t row, uint8_t col) const {
    if (row >= h || col >= w) return 0;
    return (row % 2 == 0) ? (row * w + col) : (row * w + (w - 1 - col));
}

void NeoMatrix::set_pixel(uint8_t row, uint8_t col, uint32_t color) {
    uint32_t idx = index_for(row, col);
    if (idx < buf.size()) buf[idx] = color;
}

void NeoMatrix::put_pixel_grb(uint32_t grb) {
    while (pio_sm_is_tx_fifo_full(pio, sm)) tight_loop_contents();
    pio_sm_put_blocking(pio, sm, grb << 8u); // left-align 24-bit
}

void NeoMatrix::write() {
    for (uint32_t c : buf) put_pixel_grb(to_grb(c));
    sleep_us(80); // latch
}
