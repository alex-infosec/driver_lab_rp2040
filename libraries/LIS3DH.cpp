#include "LIS3DH.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

static constexpr uint8_t CTRL_REG1 = 0x20;
static constexpr uint8_t CTRL_REG4 = 0x23;

LIS3DH::LIS3DH(i2c_inst_t* bus, uint8_t addr) : i2c_(bus), addr_(addr) {}

void LIS3DH::set_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c_, addr_, buf, 2, false);
}

uint8_t LIS3DH::read_reg(uint8_t reg) {
    i2c_write_blocking(i2c_, addr_, &reg, 1, true);
    uint8_t v = 0;
    i2c_read_blocking(i2c_, addr_, &v, 1, false);
    return v;
}

bool LIS3DH::init() {
    // 1. ODR 1.344 kHz, enable X/Y/Z
    set_reg(CTRL_REG1, 0x97);
    // 2. High-resolution, ±2g (no temp/ADC)
    set_reg(CTRL_REG4, 0x08);
    sleep_ms(5);
    return true;
}

void LIS3DH::update() {
    // Burst read 6 bytes starting at OUT_X_L with auto-increment
    uint8_t start = 0x28 | 0x80;
    uint8_t raw[6];
    i2c_write_blocking(i2c_, addr_, &start, 1, true);
    i2c_read_blocking(i2c_, addr_, raw, 6, false);

    auto s16 = [&](int lo, int hi) -> int16_t {
        return (int16_t)((raw[hi] << 8) | raw[lo]);
    };

    // High-res ±2g ≈ 0.98 mg/LSB (adjust if you change range/mode)
    const float mg_per_lsb = 0.98f;
    x = s16(0,1) * mg_per_lsb / 1000.0f;
    y = s16(2,3) * mg_per_lsb / 1000.0f;
    z = s16(4,5) * mg_per_lsb / 1000.0f;
}
