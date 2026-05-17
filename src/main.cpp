// ********************
// * a3tsm_rp2040_motor_interface  *
// ********************

// CAN frame format (8 bytes, RP2040 -> motor controller):
// Byte 0:   SOF byte 1 (0xAA)
// Byte 1:   SOF byte 2 (0x55)
// Byte 2:   Sequence number (uint8)
// Byte 3:   Flags (uint8 bit field)
// Byte 4-5: left Speed  (int16, little-endian, mm/s)
// Byte 6-7: right Speed (int16, little-endian, mm/s)
//
// CAN feedback frame (8 bytes, motor controller -> RP2040 not used here;
// this node TX's feedback on CAN_ID_*_RX):
// Byte 0-3: tick_count (int32, little-endian)
// Byte 4-5: reserved
// Byte 6:   flags (bit0 = magnet_detected)
// Byte 7:   AGC
//
// CAN IDs: FL=0x120, FR=0x121 (RX commands)
//          FL=0x220, FR=0x221 (TX feedback)

#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>
#include "mcp25125_config.h"
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>

// ---------- Node configuration ----------
static constexpr uint32_t NODE_CAN_ID    = mcp25125_config::CAN_ID_FR_TX;
static constexpr bool     IS_LEFT_SIDE   = (NODE_CAN_ID == mcp25125_config::CAN_ID_FL_TX);

// Feedback ID: 0x120 -> 0x220, 0x121 -> 0x221
static constexpr uint32_t FEEDBACK_CAN_ID =
    IS_LEFT_SIDE ? mcp25125_config::CAN_ID_FL_RX
                 : mcp25125_config::CAN_ID_FR_RX;

// ---------- Pins ----------
static constexpr int MOTOR_PWM_PIN = 5;
static constexpr int I2C_SDA       = 2;
static constexpr int I2C_SCL       = 3;

// ---------- Encoder calibration (per side) ----------
// TODO: re-run calibration on the right wheel and fill in
static constexpr float TICKS_PER_ROTATION =
    IS_LEFT_SIDE ? 24304.0f : 24304.0f;
static constexpr float WHEEL_CIRCUMFERENCE_IN = 50.265f;  // π × 16"
static constexpr int16_t COMMIT_THRESHOLD     = 40;

// ---------- Timing ----------
static constexpr unsigned long TIMEOUT_MS         = 500;
static constexpr unsigned long ENCODER_SAMPLE_MS  = 5;    // matches old delay(5)
static constexpr unsigned long FEEDBACK_TX_MS     = 50;   // 20 Hz feedback
static constexpr unsigned long DEBUG_PRINT_MS     = 200;

// ---------- State ----------
Servo talonSRX;

struct Motor {
  int16_t  cmd_left;
  int16_t  cmd_right;
  unsigned long last_valid_ms;
  bool     timed_out;
};
Motor motor = {0, 0, 0, true};

struct Encoder {
  uint16_t last_raw;
  long     tick_count;
  int32_t  accumulator;
  bool     ok;
} enc = {0, 0, 0, false};

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI,
                     mcp25125_config::PIN_CAN_MISO,
                     mcp25125_config::PIN_CAN_SCK);

AS5600 encoder(&Wire1);

// ---------- Motor ----------
void set_motor(int16_t left, int16_t right) {
  float cmd = (IS_LEFT_SIDE ? left : right) / 2000.0f;
  cmd = constrain(cmd, -1.0f, 1.0f);
  int us = (int)(1500.0f + cmd * 500.0f);
  talonSRX.writeMicroseconds(us);
}

// ---------- CAN RX ----------
bool decode_can_packet(uint32_t id, const uint8_t* data, uint8_t len) {
  if (id != NODE_CAN_ID) return false;
  if (len != 8) {
    Serial.print("ID 0x"); Serial.print(id, HEX);
    Serial.print(" unexpected DLC="); Serial.println(len);
    return false;
  }
  if (data[0] != 0xAA || data[1] != 0x55) return false;

  motor.cmd_left  = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
  motor.cmd_right = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
  return true;
}

// ---------- Encoder ----------
void encoder_update() {
  if (!enc.ok) return;

  uint16_t raw = encoder.readAngle();

  int16_t delta = (int16_t)(raw - enc.last_raw);
  if (delta >  2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  enc.last_raw = raw;

  enc.accumulator += delta;

  // NOTE: fixed the sign bug from the original (was += in both branches)
  while (enc.accumulator >= COMMIT_THRESHOLD) {
    enc.tick_count   += COMMIT_THRESHOLD;
    enc.accumulator  -= COMMIT_THRESHOLD;
  }
  while (enc.accumulator <= -COMMIT_THRESHOLD) {
    enc.tick_count   -= COMMIT_THRESHOLD;
    enc.accumulator  += COMMIT_THRESHOLD;
  }
}

// ---------- CAN TX feedback ----------
void send_feedback() {
  uint8_t buf[8] = {0};
  int32_t ticks = (int32_t)enc.tick_count;

  buf[0] = (uint8_t)(ticks       & 0xFF);
  buf[1] = (uint8_t)((ticks >> 8 ) & 0xFF);
  buf[2] = (uint8_t)((ticks >> 16) & 0xFF);
  buf[3] = (uint8_t)((ticks >> 24) & 0xFF);
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = enc.ok && encoder.magnetDetected() ? 0x01 : 0x00;
  buf[7] = enc.ok ? (uint8_t)encoder.readAGC() : 0;

  mcp.beginPacket(FEEDBACK_CAN_ID);
  mcp.write(buf, 8);
  mcp.endPacket();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  // while (!Serial) { delay(10); }

  Serial.println("Init CAN RX...");

  pinMode(mcp25125_config::PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_STANDBY, LOW);
  pinMode(mcp25125_config::PIN_CAN_RESET, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_RESET, HIGH);
  delay(10);

  if (!mcp.begin(mcp25125_config::CAN_BITRATE)) {
    Serial.println("CAN init failed");
    while (1) delay(10);
  }
  Serial.println("CAN initialized OK");

  // NeoPixel
  pinMode(neopixel_config::NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(neopixel_config::NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(20);
  pixel.clear();
  pixel.show();
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();
  delay(80);
  pixel.clear();
  pixel.show();

  // Motor
  talonSRX.attach(MOTOR_PWM_PIN, 1000, 2000);
  talonSRX.writeMicroseconds(1500);

  // Encoder (I2C1 on the RP2040)
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  Wire1.setClock(400000);

  encoder.begin();
  if (!encoder.isConnected()) {
    Serial.println("AS5600 NOT found — check wiring");
    enc.ok = false;
    pixel.setPixelColor(0, pixel.Color(255, 0, 255)); // magenta = encoder fault
    pixel.show();
  } else {
    enc.ok = true;
    enc.last_raw = encoder.readAngle();
    Serial.println("AS5600 found on bus");
    Serial.printf("Magnet detected: %s\n", encoder.magnetDetected() ? "yes" : "no");
    Serial.printf("AGC: %d (128 = ideal)\n", encoder.readAGC());
  }
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  // --- CAN RX ---
  int packetSize = mcp.parsePacket();
  if (packetSize > 0) {
    uint32_t id = mcp.packetId();
    uint8_t  data[8] = {0};
    uint8_t  expected = min((uint8_t)packetSize, (uint8_t)8);
    uint8_t  len = 0;

    for (; len < expected; len++) {
      int c = mcp.read();
      if (c < 0) break;
      data[len] = (uint8_t)c;
    }

    if (decode_can_packet(id, data, len)) {
      motor.last_valid_ms = now;
      motor.timed_out = false;
      set_motor(motor.cmd_left, motor.cmd_right);
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      pixel.show();
      pixel.clear();
      pixel.show();
    }
  }

  // --- Encoder sampling (non-blocking) ---
  static unsigned long last_enc_sample = 0;
  if (now - last_enc_sample >= ENCODER_SAMPLE_MS) {
    last_enc_sample = now;
    encoder_update();
  }

  // --- CAN TX feedback ---
  static unsigned long last_fb = 0;
  if (now - last_fb >= FEEDBACK_TX_MS) {
    last_fb = now;
    send_feedback();
  }

  // --- Debug print ---
  static unsigned long last_print = 0;
  if (now - last_print >= DEBUG_PRINT_MS) {
    last_print = now;
    float rotations   = enc.tick_count / TICKS_PER_ROTATION;
    float distance_in = rotations * WHEEL_CIRCUMFERENCE_IN;
    Serial.printf("Ticks: %ld   Rot: %.3f   Dist: %.2f in   cmd_L=%d cmd_R=%d\n",
                  enc.tick_count, rotations, distance_in,
                  motor.cmd_left, motor.cmd_right);
  }

  // --- Watchdog ---
  if (!motor.timed_out && (now - motor.last_valid_ms > TIMEOUT_MS)) {
    Serial.println("Command timeout -> stopping motor");
    set_motor(0, 0);
    motor.timed_out = true;
    pixel.setPixelColor(0, pixel.Color(255, 255, 0));
    pixel.show();
    delay(80);
    pixel.clear();
    pixel.show();
  }
}