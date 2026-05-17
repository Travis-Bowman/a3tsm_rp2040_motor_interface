// ********************

// * a3tsm_rp2040_motor_interface  *

// ********************

// CAN frame format (8 bytes, RP2040 -> motor controller):
// Byte 0:   SOF byte 1 (0xAA)
// Byte 1:   SOF byte 2 (0x55)
// Byte 2:   Sequence number (uint8)
// Byte 3:   Flags (uint8 bit field)
// Byte 4:   left Speed low byte  (int16, little-endian, mm/s)
// Byte 5:   left Speed high byte
// Byte 6:   right Speed low byte  (int16, little-endian, mm/s)
// Byte 7:   right Speed high byte
// CAN IDs: FL=0x120, FR=0x121 (TX)
//          FL=0x220, FR=0x221 (RX feedback)

#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>
#include <Servo.h>
#include "mcp25125_config.h"
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>

// Node configuration — set per board:
// Front Left  = CAN_ID_FL_TX, Front Right = CAN_ID_FR_TX
static constexpr uint32_t NODE_CAN_ID   = mcp25125_config::CAN_ID_FR_TX;
static constexpr bool     IS_LEFT_SIDE  = (NODE_CAN_ID == mcp25125_config::CAN_ID_FL_TX);

// Motor control pin (Talon SRX servo-style PWM)
static constexpr int MOTOR_PWM_PIN = 5;

Servo talonSRX;

static constexpr unsigned long TIMEOUT_MS = 500;

struct Motor {
  int16_t  cmd_left;       // mm/s
  int16_t  cmd_right;      // mm/s
  unsigned long last_valid_ms;
  bool     timed_out;
};

Motor motor = {0, 0, 0, true};

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI,
                     mcp25125_config::PIN_CAN_MISO,
                     mcp25125_config::PIN_CAN_SCK);

void set_motor(int16_t left, int16_t right) {
  float cmd = (IS_LEFT_SIDE ? left : right) / 2000.0f;
  cmd = constrain(cmd, -1.0f, 1.0f);

// if (IS_LEFT_SIDE) {
//     Serial.println("Set LEFT motor cmd=" + String(cmd));
// } else {
//     Serial.println("Set RIGHT motor cmd=" + String(cmd));
// }

  int us = (int)(1500.0f + cmd * 500.0f);  // 1000-2000µs, 1500=stop
  talonSRX.writeMicroseconds(us);
}

// Returns true if packet matches this node and passes validation
bool decode_can_packet(uint32_t id, const uint8_t* data, uint8_t len) {
  if (id != NODE_CAN_ID) return false;
  if (len != 8) {
    Serial.print("ID 0x"); Serial.print(id, HEX);
    Serial.print(" unexpected DLC="); Serial.println(len);
    return false;
  }
  if (data[0] != 0xAA || data[1] != 0x55) return false;

  motor.cmd_left  = (int16_t)(data[4] | (data[5] << 8));
  motor.cmd_right = (int16_t)(data[6] | (data[7] << 8));

  uint8_t seq   = data[2];
  uint8_t flags = data[3];

  // Serial.print("seq="); Serial.print(seq);
  // Serial.print(" flags=0x"); Serial.print(flags, HEX);
  // Serial.print(" left="); Serial.print(motor.cmd_left);
  // Serial.print(" right="); Serial.println(motor.cmd_right);

  return true;
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) { delay(10); }

  Serial.println("Init CAN RX...");

  pinMode(mcp25125_config::PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_STANDBY, LOW);   // LOW = normal operation

  pinMode(mcp25125_config::PIN_CAN_RESET, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_RESET, HIGH);    // HIGH = not in reset (active-low)

  delay(10);

  

  if (!mcp.begin(mcp25125_config::CAN_BITRATE)) {
    Serial.println("CAN init failed");
    while (1) delay(10);
  }
  Serial.println("CAN initialized OK");

  pinMode(neopixel_config::NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(neopixel_config::NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(20);
  pixel.clear();
  pixel.show();

  pixel.setPixelColor(0, pixel.Color(0, 0, 255)); // blue = boot ok
  pixel.show();
  delay(80);
  pixel.clear();
  pixel.show();

  talonSRX.attach(MOTOR_PWM_PIN, 1000, 2000);
  talonSRX.writeMicroseconds(1500);  // neutral on boot
}

void loop() {
  int packetSize = mcp.parsePacket();

  if (packetSize > 0) {
    uint32_t id  = mcp.packetId();
    uint8_t  data[8] = {0};
    uint8_t  len = min((uint8_t)packetSize, (uint8_t)8);

    for (uint8_t i = 0; i < len; i++) {
      int c = mcp.read();
      if (c < 0) break;
      data[i] = (uint8_t)c;
    }

    if (decode_can_packet(id, data, len)) {
      motor.last_valid_ms = millis();
      motor.timed_out = false;
      set_motor(motor.cmd_left, motor.cmd_right);
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      pixel.show();
      pixel.clear();
    }
  }

  // Watchdog: stop motor if commands stop arriving
  if (!motor.timed_out && (millis() - motor.last_valid_ms > TIMEOUT_MS)) {
    Serial.println("Command timeout -> stopping motor");
    set_motor(0, 0);
    motor.timed_out = true;
    pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // yellow = timeout
    pixel.show();
    delay(80);
    pixel.clear();
    pixel.show();
  }
}