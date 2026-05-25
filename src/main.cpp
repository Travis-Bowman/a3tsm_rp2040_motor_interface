// ********************
// * a3tsm_rp2040_motor_interface  *
// ********************
//
// Hi, Alyssa
//
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
// Byte 4-5: velocity   (int16, little-endian, mm/s)
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
static constexpr uint32_t NODE_CAN_ID    = mcp25125_config::CAN_ID_FL_TX;
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
// AS5600 is a 12-bit magnetic encoder: 4096 LSB per ENCODER revolution.
// This is fixed by the hardware and is what readAngle() / the wrap logic use.
static constexpr float LSB_PER_ENCODER_REV = 4096.0f;

// Gearbox between encoder (motor shaft) and wheel: 20:1.
// Encoder turns 20 times per wheel rotation.
//   - If the encoder were on the wheel axle, this would be 1.0.
//   - Verify 20:1 is the FULL reduction (include any secondary belt/chain stage).
static constexpr float GEAR_RATIO = 20.0f;

// Encoder LSB per one WHEEL rotation — measured empirically per side.
// Left measured: 82568.  Right: measure by driving one full wheel rotation
// and reading enc.tick_count delta, then fill in below.
static constexpr float TICKS_PER_WHEEL_ROTATION =
    IS_LEFT_SIDE ? 82568.0f : 82568.0f;  // TODO: replace right-side value

static constexpr float WHEEL_CIRCUMFERENCE_IN = 46.24f;  // π × 16" wheel — verify wheel size
static constexpr float WHEEL_CIRCUMFERENCE_MM = WHEEL_CIRCUMFERENCE_IN * 25.4f;

// Single conversion constant: mm of wheel travel per encoder LSB.
// Used everywhere velocity/distance is converted, so TX and debug can't drift apart.
static constexpr float MM_PER_LSB =
    WHEEL_CIRCUMFERENCE_MM / TICKS_PER_WHEEL_ROTATION;

// ---------- Filter / sampling config ----------
// IMPORTANT: keep SAMPLE_RATE_HZ in sync with ENCODER_SAMPLE_MS
//   5 ms  -> 200 Hz
//   1 ms  -> 1000 Hz
#define SAMPLE_RATE_HZ 200.0f
#define VEL_ALPHA      0.05f   // each stage ~95 ms; cascaded = 2nd-order ~135 ms effective

// Spike rejection: any single-sample delta larger than this is treated as
// an I2C glitch.
// 5 mph = 2235 mm/s → 2235 / (1174.5/82568) / 200 Hz ≈ 785 LSB/sample.
// 1000 gives ~27% headroom above max speed.
static constexpr int16_t MAX_PLAUSIBLE_DELTA = 1000;

// ---------- Timing ----------
static constexpr unsigned long TIMEOUT_MS         = 500;
static constexpr unsigned long ENCODER_SAMPLE_MS  = 5;     // 200 Hz
static constexpr unsigned long FEEDBACK_TX_MS     = 50;    // 20 Hz feedback
static constexpr unsigned long DEBUG_PRINT_MS     = 200;
static constexpr unsigned long LED_BLINK_MS       = 30;    // CAN RX blink length

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
  bool     ok;
  uint16_t last_raw;
  int32_t  tick_count;             // raw accumulated position (no filtering)
  float    ema1;                   // first EMA stage
  float    ema2;                   // second EMA stage (cascaded for 2nd-order rolloff)
  float    velocity_lsb_per_sec;   // derived from ema2
  uint32_t read_errors;
};
Encoder enc = {false, 0, 0, 0.0f, 0.0f, 0.0f, 0};

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI,
                     mcp25125_config::PIN_CAN_MISO,
                     mcp25125_config::PIN_CAN_SCK);

AS5600 encoder(&Wire1);

// ---------- LED helper (non-blocking blink) ----------
struct LedBlink {
  bool active;
  unsigned long off_at_ms;
} led = {false, 0};

void led_blink(uint8_t r, uint8_t g, uint8_t b, unsigned long now) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  led.active = true;
  led.off_at_ms = now + LED_BLINK_MS;
}

void led_service(unsigned long now) {
  if (led.active && (long)(now - led.off_at_ms) >= 0) {
    pixel.clear();
    pixel.show();
    led.active = false;
  }
}

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

  // Compute wrapped delta in LSB (encoder is 4096 counts/rev)
  int16_t delta = (int16_t)(raw - enc.last_raw);
  if (delta >  2048) delta -= 4096;
  if (delta < -2048) delta += 4096;

  // Spike rejection: top speed is ~720 LSB/sample at 200 Hz.
  // Anything beyond 800 is an I2C glitch — discard and resync.
  if (delta > MAX_PLAUSIBLE_DELTA || delta < -MAX_PLAUSIBLE_DELTA) {
    enc.read_errors++;
    enc.last_raw = raw;
    return;
  }
  enc.last_raw = raw;

  if (!IS_LEFT_SIDE) delta = -delta;

  // 7-sample median filter: rejects up to 3 consecutive bad reads that
  // pass the plausibility threshold.
  static int16_t med_buf[7] = {0, 0, 0, 0, 0, 0, 0};
  static uint8_t med_idx    = 0;
  med_buf[med_idx] = delta;
  med_idx = (med_idx + 1) % 7;
  int16_t s[7];
  memcpy(s, med_buf, sizeof(s));
  for (int i = 1; i < 7; i++) {
    int16_t key = s[i]; int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
    s[j+1] = key;
  }
  int16_t med = s[3];

  // Position: use raw delta so a single glitch doesn't corrupt tick_count.
  // Median has 2-sample lag; raw delta has none.
  enc.tick_count += delta;

  // Cascaded double EMA: two stages in series give 2nd-order rolloff
  // (much steeper noise rejection than a single EMA at the same alpha).
  enc.ema1 = VEL_ALPHA * (float)med + (1.0f - VEL_ALPHA) * enc.ema1;
  enc.ema2 = VEL_ALPHA * enc.ema1   + (1.0f - VEL_ALPHA) * enc.ema2;

  // Deadband on velocity output: anything below noise floor reads as zero.
  float vel_lsb = enc.ema2;
  if (vel_lsb > -1.5f && vel_lsb < 1.5f) vel_lsb = 0.0f;
  enc.velocity_lsb_per_sec = vel_lsb * SAMPLE_RATE_HZ;
}

// ---------- CAN TX feedback ----------
void send_feedback() {
  uint8_t buf[8] = {0};
  int32_t ticks = enc.tick_count;

  // Bytes 0-3: tick_count (int32 LE)
  buf[0] = (uint8_t)( ticks        & 0xFF);
  buf[1] = (uint8_t)((ticks >>  8) & 0xFF);
  buf[2] = (uint8_t)((ticks >> 16) & 0xFF);
  buf[3] = (uint8_t)((ticks >> 24) & 0xFF);

  // Bytes 4-5: velocity in mm/s (int16 LE)
  // velocity_mm_per_sec = velocity_lsb_per_sec * MM_PER_LSB
  //   where MM_PER_LSB = WHEEL_CIRCUMFERENCE_MM / (4096 * GEAR_RATIO)
  float vel_mm = enc.velocity_lsb_per_sec * MM_PER_LSB;
  // Clamp to int16 range to avoid wrap on overflow
  if (vel_mm >  32767.0f) vel_mm =  32767.0f;
  if (vel_mm < -32768.0f) vel_mm = -32768.0f;
  int16_t vel_mm_i = (int16_t)vel_mm;
  buf[4] = (uint8_t)( vel_mm_i       & 0xFF);
  buf[5] = (uint8_t)((vel_mm_i >> 8) & 0xFF);

  // Byte 6: status flags
  buf[6] = (enc.ok && encoder.magnetDetected()) ? 0x01 : 0x00;

  // Byte 7: AGC (signal quality)
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
  Wire1.setClock(50000);  // 50 kHz — more noise margin; raise back to 100000 if read_errors stay low

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
      led_blink(0, 255, 0, now);   // green flash on valid command
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
    float rotations   = (float)enc.tick_count / TICKS_PER_WHEEL_ROTATION;
    float distance_in = rotations * WHEEL_CIRCUMFERENCE_IN;
    float vel_mm_s    = enc.velocity_lsb_per_sec * MM_PER_LSB;
    float vel_mph     = vel_mm_s * 0.00223694f;
    Serial.printf("Ticks: %ld  Rot: %.3f  Dist: %.2f in  Vel: %.0f mm/s (%.2f mph)  "
                  "raw=%u  errs=%lu  cmd_L=%d cmd_R=%d\n",
                  (long)enc.tick_count, rotations, distance_in,
                  vel_mm_s, vel_mph,
                  enc.last_raw, (unsigned long)enc.read_errors,
                  motor.cmd_left, motor.cmd_right);

    uint16_t raw1 = encoder.readAngle();
    delayMicroseconds(100);
    uint16_t raw2 = encoder.readAngle();
    Serial.printf("  raw1=%u raw2=%u diff=%d\n", raw1, raw2, (int)raw2 - (int)raw1);
  }

  // --- Watchdog ---
  if (!motor.timed_out && (now - motor.last_valid_ms > TIMEOUT_MS)) {
    Serial.println("Command timeout -> stopping motor");
    set_motor(0, 0);
    motor.timed_out = true;
    led_blink(255, 255, 0, now);   // yellow flash on timeout
  }

  // --- Service non-blocking LED ---
  led_service(now);
}