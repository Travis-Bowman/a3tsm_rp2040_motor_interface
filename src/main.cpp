// ********************

// * a3tsm_rp2040_motor_interface  *

// ********************


#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>

#include "mcp25125_config.h"
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>

// Motor designation (set these according to which wheel this controller is attached to)
struct {
  int position = 0; // 0=front-left, 1=front-right, 2=rear-left, 3=rear-right
  int purpose = 0;  // 0=speed, 1=steer
}motor_designation;

// Motor control pins
static const int MOTOR_PWM_PIN = 5;
static const int MOTOR_DIR_PIN = 4;

unsigned long last_valid_packet = 0;
const unsigned long timeout_ms = 500;   // adjust as needed
bool motor_timed_out = true;
// Must match the TX side's CAN ID and payload format:
static constexpr uint32_t CAN_BITRATE = 500000;
// If you only care about this ID, set it here
static constexpr uint32_t CAN_ID_RX = 0x123;

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

// SPI-pin constructor (as in your TX code)
Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI,
                     mcp25125_config::PIN_CAN_MISO,
                     mcp25125_config::PIN_CAN_SCK);

static uint8_t crc8_atm(const uint8_t* data, size_t len, uint8_t poly = 0x07, uint8_t init = 0x00) {
  uint8_t crc = init;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ poly);
      else crc <<= 1;
    }
  }
  return crc;
}

static void neopixel_blink(uint8_t r, uint8_t g, uint8_t b, uint16_t ms = 20) {

  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(ms);
  pixel.clear();
  pixel.show();
}

void setMotor(float cmd)
{
  cmd = constrain(cmd, -1.0f, 1.0f);

  bool forward = cmd >= 0.0f;
  float mag = fabs(cmd);

  int duty = (int)(mag * 255.0f);

  digitalWrite(MOTOR_DIR_PIN, forward ? HIGH : LOW);
  analogWrite(MOTOR_PWM_PIN, duty);
}

// NOTE: Function below sends an 8-byte CAN frame containing linear velocity.
// If linear_velocity == 0, the receiver/control side can treat that as braking
// and set the NeoPixel purple there. I only addded the function without changing anything else.
// Purple LED for braking should be handled on the receive/control side
// where braking is actually detected and applied.

// CAN TX Helper (feature/braking)
bool sendCANDataField(Adafruit_MCP2515 &mcp, uint32_t can_id, int16_t linear_velocity) {
  uint8_t data[8] = {0};

  data[0] = 0xAA;
  data[1] = 0x55;
  data[2] = (uint8_t)(linear_velocity & 0xFF);
  data[3] = (uint8_t)((linear_velocity >> 8) & 0xFF);
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;

  if (mcp.beginPacket(can_id) != 0) {
    return false;
  }

  if (mcp.write(data, 8) != 8) {
    mcp.endPacket();
    return false;
  }
  
  return mcp.endPacket() == 0;
}

// Decode your 8-byte payload format for ID 0x123:
// [0]=seq [1]=flags [2..3]=lin_i16 [4..5]=ang_i16 [6]=rx_crc [7]=0
static void decode_if_matching(uint32_t id, const uint8_t* data, uint8_t len) {
  
  if (id != CAN_ID_RX) return;
  if (len != 8) {
    Serial.print("ID 0x");
    Serial.print(id, HEX);
    Serial.print(" unexpected DLC=");
    Serial.println(len);
    return;
  }

  // Validate SOF
  if (data[0] != 0xAA || data[1] != 0x55) return; // bad frame

  const uint8_t seq   = data[2];
  const uint8_t flags = data[3];

  int16_t driveValue = 0; // default if position/purpose invalid
  switch(motor_designation.position){
    case 0: 
      if(motor_designation.purpose == 0){  driveValue = 1000 * (int16_t)(data[4]  | (data[5]  << 8));}
      else {driveValue = 1000 * (int16_t)(data[6]  | (data[7]  << 8));}
      break;
    case 1:
      if(motor_designation.purpose == 0){  driveValue = 1000 * (int16_t)(data[8]  | (data[9]  << 8));}
      else {driveValue = 1000 * (int16_t)(data[10] | (data[11] << 8));}
      break;
    case 2:
      if(motor_designation.purpose == 0){  driveValue = 1000 * (int16_t)(data[12] | (data[13] << 8));}
      else {driveValue = 1000 * (int16_t)(data[14] | (data[15] << 8));}
      break;
    case 3: 
      if(motor_designation.purpose == 0){  driveValue = 1000 * (int16_t)(data[16] | (data[17] << 8));}
      else {driveValue = 1000 * (int16_t)(data[18] | (data[19] << 8));}
      break;
    default: Serial.println("Invalid motor position in motor_designation struct"); 
      break;
  }

  const uint8_t rx_crc = data[12];
  const uint8_t calc   = crc8_atm(data, 12); // CRC over first 12 bytes

  if (calc == rx_crc) {
    Serial.println(" CRC=OK");
    neopixel_blink(0, 255, 0); // green

    last_valid_packet = millis();
    motor_timed_out = false;
    setMotor(driveValue);

  } else {
    Serial.print(" CRC=BAD rx=");
    Serial.print(rx_crc, HEX);
    Serial.print(" calc=");
    Serial.println(calc, HEX);
    neopixel_blink(255, 0, 0, 60); // red
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("Init CAN RX...");

  // Wake transceiver / release reset (as in your code)
  pinMode(mcp25125_config::PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_STANDBY, LOW);   // normal (not standby)

  pinMode(mcp25125_config::PIN_CAN_RESET, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_RESET, HIGH);    // not in reset (active-low)

  delay(10);

  if (!mcp.begin(mcp25125_config::CAN_BITRATE)) {
    Serial.println("CAN init failed");
    while (1) delay(10);
  }
  Serial.println("CAN initialized OK");

  // NeoPixel setup
  pinMode(neopixel_config::NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(neopixel_config::NEOPIXEL_POWER_PIN, HIGH);

  pixel.begin();
  pixel.setBrightness(20);
  pixel.clear();
  pixel.show();

  neopixel_blink(0, 0, 255, 80); // blue = boot ok

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
}

void loop() {
  int packetSize = mcp.parsePacket();

  if (packetSize > 0) {
    uint32_t id = mcp.packetId();
    bool ext = mcp.packetExtended();
    bool rtr = mcp.packetRtr();

    uint8_t data[13] = {0};
    uint8_t len = (uint8_t)packetSize;
    if (len > 13) len = 13;

    for (uint8_t i = 0; i < len; i++) {
      int c = mcp.read();
      if (c < 0) break;
      data[i] = (uint8_t)c;
    }

    for (uint8_t i = 0; i < len; i++) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    decode_if_matching(id, data, len);
  }

  // Timeout watchdog always runs, even if no packet arrived
  if (!motor_timed_out && (millis() - last_valid_packet > timeout_ms)) {
    Serial.println("Command timeout -> stopping motor");
    setMotor(0.0f);
    motor_timed_out = true;
    neopixel_blink(255, 255, 0, 80); // yellow for timeout
  }
}
