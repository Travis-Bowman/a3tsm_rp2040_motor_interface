#pragma once
#include <cstdint>

// MCP2515 SPI pins
namespace neopixel_config {
    static constexpr uint8_t NEOPIXEL_DATA_PIN  = 21; // GPIO21
    static constexpr uint8_t NEOPIXEL_POWER_PIN = 20; // GPIO20
}