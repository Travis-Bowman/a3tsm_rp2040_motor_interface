#pragma once
#include <cstdint>

// MCP2515 SPI pins
namespace mcp25125_config {
    static constexpr uint8_t PIN_CAN_SCK     = 14;
    static constexpr uint8_t PIN_CAN_MOSI    = 15;
    static constexpr uint8_t PIN_CAN_MISO    = 8;
    static constexpr uint8_t PIN_CAN_CS      = 19;
    static constexpr uint8_t PIN_CAN_STANDBY = 16;
    static constexpr uint8_t PIN_CAN_RESET   = 18;

    static constexpr uint32_t CAN_BITRATE = 500000;
}