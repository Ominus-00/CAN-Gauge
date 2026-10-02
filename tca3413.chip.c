// Beginner TCAN3413 model for the Wokwi custom-chip runtime.
// This file is NOT ESP32 firmware.
//
// Assumes VCC = VIO = 3.3 V and GND = 0 V throughout simulation.
// DIGITAL ENCODING ONLY (these are NOT physical CAN voltages):
//   dominant: CANH = 1, CANL = 0
//   idle:     CANH = 0, CANL = 1, established by weak pulls
// Each transmitter either drives dominant or releases BOTH wires.
// All simulated peers must follow the same encoding and release rule.
//
// Omits analog behavior, power supervision, delays, TXD timeout,
// thermal protection, and standby wake-up-pattern detection.

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
  pin_t txd;
  pin_t rxd;
  pin_t canh;
  pin_t canl;
  pin_t stb;
  bool driving_dominant;
} chip_state_t;

static void update_receiver(chip_state_t *chip) {
  // Simplified standby: no wake-up detector, so RXD stays high.
  if (pin_read(chip->stb) == HIGH) {
    pin_write(chip->rxd, HIGH);
    return;
  }

  // Read the shared bus, not TXD: another node may be dominant.
  bool dominant = pin_read(chip->canh) == HIGH &&
                  pin_read(chip->canl) == LOW;
  pin_write(chip->rxd, dominant ? LOW : HIGH);
}

static void update_transmitter(chip_state_t *chip) {
  bool dominant = pin_read(chip->stb) == LOW &&
                  pin_read(chip->txd) == LOW;

  // Avoid rewriting bus modes when only the received state changes.
  if (dominant != chip->driving_dominant) {
    chip->driving_dominant = dominant;
    if (dominant) {
      pin_mode(chip->canh, OUTPUT_HIGH);
      pin_mode(chip->canl, OUTPUT_LOW);
    } else {
      // Release the drivers. Weak pulls define our digital idle code.
      // Another node can still drive dominant against those weak pulls.
      pin_mode(chip->canh, INPUT_PULLDOWN);
      pin_mode(chip->canl, INPUT_PULLUP);
    }
  }
  update_receiver(chip);
}

static void on_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t *)user_data;
  (void)value; // Read the current pins instead of using this event's value.

  if (pin == chip->txd || pin == chip->stb) {
    update_transmitter(chip);
  } else {
    update_receiver(chip);
  }
}

void chip_init(void) {
  // Separate state for every instance of this chip in the diagram.
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  if (!chip) {
    return;
  }

  chip->txd  = pin_init("TXD", INPUT_PULLUP);
  chip->rxd  = pin_init("RXD", OUTPUT_HIGH);
  chip->canh = pin_init("CANH", INPUT_PULLDOWN);
  chip->canl = pin_init("CANL", INPUT_PULLUP);
  chip->stb  = pin_init("STB", INPUT_PULLUP);

  // Present for correct wiring; supply behavior is deliberately omitted.
  pin_init("VCC", INPUT);
  pin_init("VIO", INPUT);
  pin_init("GND", INPUT);

  const pin_watch_config_t watch = {
    .edge = BOTH,
    .pin_change = on_pin_change,
    .user_data = chip,
  };
  pin_watch(chip->txd, &watch);
  pin_watch(chip->stb, &watch);
  pin_watch(chip->canh, &watch);
  pin_watch(chip->canl, &watch);

  // Establish correct outputs even before the first pin-change event.
  update_transmitter(chip);
}
