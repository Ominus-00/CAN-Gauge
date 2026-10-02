#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/twai.h>

// 1 = Wokwi bench: software receives real frame bits at 1 kbit/s.
// 0 = physical ESP32-S3: native TWAI at 500 kbit/s.
// The slow bench mode is NOT a production CAN controller or OBD scanner.
#ifndef WOKWI_SOFT_CAN
#define WOKWI_SOFT_CAN 1
#endif
constexpr int CAN_TX = 1, CAN_RX = 2;
constexpr uint32_t BIT_US = 1000; // Must match ecu.chip.c: 1000 bit/s.
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool hasDisplay = false;
uint32_t received = 0, errors = 0, lastScreen = 0;
constexpr unsigned CHANNEL_COUNT = 6;
const char *labels[CHANNEL_COUNT] = {"IAT", "Boost", "Coolant", "Oil temp", "EGT", "Oil P"};
int16_t values[CHANNEL_COUNT] = {};
uint32_t updated[CHANNEL_COUNT] = {};
bool seen[CHANNEL_COUNT] = {};

struct Frame { uint16_t id; uint8_t len; uint8_t data[8]; };

#if WOKWI_SOFT_CAN
// Sample at each bit's center. Deadlines avoid accumulating timing error.
static void waitUntil(uint32_t deadline) {
  while ((int32_t)(micros() - deadline) < 0) {}
}
struct BitReader {
  uint32_t next;
  int previous = -1, run = 0;
  bool valid = true;
  uint16_t crc = 0;
  int raw() {
    waitUntil(next);
    // A late sample means timing was lost: discard, never display bad data.
    if ((int32_t)(micros() - next) > (int32_t)(BIT_US / 3)) valid = false;
    int b = digitalRead(CAN_RX);
    next += BIT_US;
    return b;
  }
  void stuffing() {
    if (run == 5) {
      int b = raw();
      if (b == previous) valid = false;
      previous = b;
      run = 1;
    }
  }
  unsigned field(unsigned count, bool includeCRC = true) {
    unsigned value = 0;
    while (count--) {
      stuffing();
      int b = raw();
      run = b == previous ? run + 1 : 1;
      previous = b;
      value = (value << 1) | b;
      if (includeCRC) {
        bool feedback = ((crc >> 14) & 1) ^ b;
        crc = (crc << 1) & 0x7fff;
        if (feedback) crc ^= 0x4599;
      }
    }
    return value;
  }
};

// Restricted to standard classical data frames on our two-node test bus.
// No arbitration, resynchronization, error flags, or CAN fault confinement.
static bool receiveBits(uint32_t start, Frame &f) {
  BitReader b;
  b.next = start + BIT_US / 2;
  if (b.field(1) != 0) return false; // SOF
  f.id = b.field(11);
  unsigned rtr = b.field(1), ide = b.field(1), reserved = b.field(1);
  if (rtr || ide || reserved) return false;
  f.len = b.field(4);
  if (f.len > 8) return false;
  for (unsigned i = 0; i < f.len; ++i) f.data[i] = b.field(8);
  uint16_t expected = b.crc;
  uint16_t actual = b.field(15, false);
  b.stuffing(); // A stuff bit can also follow the final CRC bit.
  int delimiter = b.raw();
  if (!b.valid || expected != actual || delimiter != HIGH) return false;

  // Acknowledge only a frame whose stuffing and CRC passed.
  // TXD LOW makes the transceiver drive a dominant ACK onto the bus.
  waitUntil(b.next - BIT_US / 2);
  digitalWrite(CAN_TX, LOW);
  b.raw(); // ACK slot
  waitUntil(b.next - BIT_US / 2);
  digitalWrite(CAN_TX, HIGH); // Release before ACK delimiter.
  for (unsigned i = 0; i < 11; ++i) // ACK delimiter + EOF + intermission
    if (b.raw() != HIGH) b.valid = false;
  return b.valid;
}

static bool receiveFrame(Frame &f) {
  static uint32_t idleSince = micros();
  static bool armed = false;
  uint32_t now = micros();
  if (digitalRead(CAN_RX) == HIGH) {
    if (now - idleSince >= 11 * BIT_US) armed = true;
    return false;
  }
  if (!armed) { idleSince = now; return false; }
  armed = false;
  bool ok = receiveBits(now, f);
  digitalWrite(CAN_TX, HIGH);
  idleSince = micros();
  if (!ok) ++errors;
  return ok;
}
#else
static bool receiveFrame(Frame &f) {
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) != ESP_OK) return false;
  if (message.extd || message.rtr || message.data_length_code > 8) return false;
  f.id = message.identifier;
  f.len = message.data_length_code;
  memcpy(f.data, message.data, f.len);
  return true;
}
#endif

static void drawScreen() {
  if (!hasDisplay) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(WOKWI_SOFT_CAN ? "SIM CAN 1 kbit/s" : "TWAI CAN 500 kbit/s");
  for (unsigned i = 0; i < CHANNEL_COUNT; ++i) {
    display.setCursor(0, 8 + i * 8);
    display.print(labels[i]);
    display.setCursor(54, 8 + i * 8);
    if (!seen[i] || millis() - updated[i] > 3000) {
      display.print("---");
    } else if (i == 1 || i == 5) {
      display.print(values[i] / 100.0f, 2);
      display.print(" bar");
    } else {
      display.print(values[i]);
      display.print(" C");
    }
  }
  display.setCursor(0, 56);
  display.printf("RX:%lu ERR:%lu", (unsigned long)received, (unsigned long)errors);
  display.display();
  lastScreen = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(CAN_TX, OUTPUT);
  digitalWrite(CAN_TX, HIGH); // Idle: do not hold the shared bus dominant.
  pinMode(CAN_RX, INPUT_PULLUP);
  Wire.begin(8, 9);
  hasDisplay = display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  Wire.setClock(400000);
#if WOKWI_SOFT_CAN
  Serial.println("Wokwi CAN bench: 1000 bit/s, GPIO1 TX / GPIO2 RX");
  Serial.println("Six pots -> ECU -> CANH/L -> TCAN3413 -> GPIO2 -> CRC -> OLED");
#else
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&general, &timing, &filter));
  ESP_ERROR_CHECK(twai_start());
  Serial.println("Native TWAI: 500000 bit/s");
#endif
  if (!hasDisplay) Serial.println("OLED not found at 0x3c; serial test continues.");
  drawScreen();
}

void loop() {
  Frame f;
  if (receiveFrame(f)) {
    if (f.id >= 0x123 && f.id < 0x123 + CHANNEL_COUNT && f.len == 2) {
      unsigned channel = f.id - 0x123;
      uint16_t raw = (uint16_t(f.data[0]) << 8) | f.data[1];
      values[channel] = raw < 0x8000 ? raw : (int32_t)raw - 65536;
      seen[channel] = true;
      updated[channel] = millis();
      ++received;
      Serial.printf("RX #%lu ID=0x%03x %s=", (unsigned long)received, f.id, labels[channel]);
      if (channel == 1 || channel == 5) {
        Serial.print(values[channel] / 100.0f, 2);
        Serial.print(" bar");
      } else {
        Serial.print(values[channel]); Serial.print(" C");
      }
      Serial.printf(" CRC=OK errors=%lu\r\n", (unsigned long)errors);
      drawScreen();
    }
    yield();
  }
  // The fake ECU leaves 100 ms between frames for OLED updates.
  // On an absent/broken bus this also clears stale displayed RPM.
  if (millis() - lastScreen > 1000) { drawScreen(); yield(); }
}


