#pragma once
#include "hardware/i2c.h"
#include <cstdint>

class LIS3DH {
public:
  // x,y,z in g
  float x{0}, y{0}, z{0};

  // pass the I2C block (i2c0 or i2c1) and the 7-bit addr (0x18 or 0x19)
  LIS3DH(i2c_inst_t* bus, uint8_t addr = 0x18);

  bool init();      // config sensor
  void update();    // read 6 bytes -> x,y,z
  void set_reg(uint8_t reg, uint8_t val);
  uint8_t read_reg(uint8_t reg);

private:
  i2c_inst_t* i2c_;
  uint8_t addr_;
};
