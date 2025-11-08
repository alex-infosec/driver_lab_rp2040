cmake_minimum_required(VERSION 3.13)

# Use the SDK one folder up (../pico-sdk) — no env vars needed
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../pico-sdk" CACHE PATH "" FORCE)
include(pico_sdk_import.cmake)

# Keep the board generic so graders don’t need board packages
set(PICO_BOARD pico CACHE STRING "pico")

project(level C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Init SDK
pico_sdk_init()

# ---- Target ----
add_executable(level
  src/level.cpp
  libraries/LIS3DH.cpp
  libraries/neomatrix.cpp
)

# Generate the WS2812 PIO header
pico_generate_pio_header(level ${CMAKE_CURRENT_LIST_DIR}/libraries/neopixel.pio)

# Headers
target_include_directories(level PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/libraries
)

# Your working pins/wiring
target_compile_definitions(level PRIVATE
  NEOMATRIX_PIN=7
  NEOMATRIX_PWR_PIN=10
  NEOMATRIX_WIRING=1   # row-serp
)

# Link libs
target_link_libraries(level
  pico_stdlib
  hardware_i2c
  hardware_pio
  hardware_clocks
  hardware_uart
)

# USB printf on; UART off
pico_enable_stdio_usb(level 1)
pico_enable_stdio_uart(level 0)

# Produce UF2/BIN/HEX
pico_add_extra_outputs(level)
