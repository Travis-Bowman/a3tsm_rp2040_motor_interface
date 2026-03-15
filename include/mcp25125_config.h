#pragma once
#include <cstdint>

// MCP2515 SPI pins
namespace mcp25125_config {
    // SPI pins
    static constexpr uint8_t PIN_CAN_SCK     = 14;
    static constexpr uint8_t PIN_CAN_MOSI    = 15;
    static constexpr uint8_t PIN_CAN_MISO    = 8;
    static constexpr uint8_t PIN_CAN_CS      = 19;
    static constexpr uint8_t PIN_CAN_STANDBY = 16;
    static constexpr uint8_t PIN_CAN_RESET   = 18;

    // CAN configuration -- TX IDs
    static constexpr uint32_t CAN_BITRATE = 500000; // match your bus
    static constexpr uint32_t CAN_ID_FL_TX = 0x120; // Front Left  (speed + steer)
    static constexpr uint32_t CAN_ID_FR_TX = 0x121; // Front Right (speed + steer)
    static constexpr uint32_t CAN_ID_RL_TX = 0x122; // Rear Left   (speed + steer)
    static constexpr uint32_t CAN_ID_RR_TX = 0x123; // Rear Right  (speed + steer)
    // CAN configuration -- RX IDs
    static constexpr uint32_t CAN_ID_FL_RX = 0x220; // Status from motor controllers
    static constexpr uint32_t CAN_ID_FR_RX = 0x221;
    static constexpr uint32_t CAN_ID_RL_RX = 0x222;
    static constexpr uint32_t CAN_ID_RR_RX = 0x223;
}