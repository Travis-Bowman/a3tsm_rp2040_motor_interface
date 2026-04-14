// ********************

// * a3tsm_rp2040_motor_interface  *

// ********************

// CAN frame format (8 bytes, RP2040 interface -> motor controller):
// Byte 0:   SOF byte 1 (0xAA)
// Byte 1:   SOF byte 2 (0x55)
// Byte 2:   Sequence number (uint8)
// Byte 3:   Flags (uint8 bit field)
// Byte 4:   Speed low byte  (int16, little-endian, mm/s)
// Byte 5:   Speed high byte
// Byte 6:   Steer low byte  (int16, little-endian, mrad)
// Byte 7:   Steer high byte
// CAN IDs: FL=0x120, FR=0x121, RL=0x122, RR=0x123 (commands in)
//          FL=0x220, FR=0x221, RL=0x222, RR=0x223 (feedback out)

#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>
#include "mcp25125_config.h"
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>

// Node configuration — set per board:
// Front Left  = CAN_ID_FL_TX, Front Right = CAN_ID_FR_TX
// Rear Left   = CAN_ID_RL_TX, Rear Right  = CAN_ID_RR_TX
static constexpr uint32_t NODE_CAN_ID  = mcp25125_config::CAN_ID_FL_TX;
static constexpr uint8_t  NODE_PURPOSE = 0; // 0=speed, 1=steer

// Motor control pins
static constexpr int MOTOR_PWM_PIN = 5;
static constexpr int MOTOR_DIR_PIN = 4;

static constexpr unsigned long TIMEOUT_MS = 500;

struct Motor {
  int16_t  cmd_speed;      // mm/s
  int16_t  cmd_steer;      // mrad
  unsigned long last_valid_ms;
  bool     timed_out;
};

Motor motor = {0, 0, 0, true};

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI,
                     mcp25125_config::PIN_CAN_MISO,
                     mcp25125_config::PIN_CAN_SCK);

void set_motor(int16_t speed, int16_t steer) {
  float cmd = (NODE_PURPOSE == 0) ? speed / 1000.0f : steer / 1000.0f;
  cmd = constrain(cmd, -1.0f, 1.0f);
  bool forward = cmd >= 0.0f;
  int duty = (int)(fabs(cmd) * 255.0f);
  digitalWrite(MOTOR_DIR_PIN, forward ? HIGH : LOW);
  analogWrite(MOTOR_PWM_PIN, duty);
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

  motor.cmd_speed = (int16_t)(data[4] | (data[5] << 8));
  motor.cmd_steer = (int16_t)(data[6] | (data[7] << 8));

  uint8_t seq   = data[2];
  uint8_t flags = data[3];

  Serial.print("seq="); Serial.print(seq);
  Serial.print(" flags=0x"); Serial.print(flags, HEX);
  Serial.print(" speed="); Serial.print(motor.cmd_speed);
  Serial.print(" steer="); Serial.println(motor.cmd_steer);

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

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);

  digitalWrite(MOTOR_DIR_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, 255);
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
      set_motor(motor.cmd_speed, motor.cmd_steer);
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