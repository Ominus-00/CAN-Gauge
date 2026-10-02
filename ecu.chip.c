// Wokwi-only ECU stimulus source, compatible with our TCAN3413 model.
// One sender + one receiving ESP32-S3; NOT a complete CAN/OBD-II ECU.
// Six analog inputs -> IDs 0x123..0x128, DLC 2, signed 16-bit big-endian.
// CANH/CANL use our DIGITAL encoding, not real differential voltages.
// Recessive = release both wires; dominant = CANH HIGH, CANL LOW.
// VCC/GND are schematic pins; power loss is not simulated.

#include "wokwi-api.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

// Wokwi bench mode: match BIT_US=1000 in main.cpp (1 kbit/s).
#define CAN_BITRATE 1000u
#define FRAME_ID 0x123u
#define CHANNEL_COUNT 6u
// Wokwi's virtual pot/analog system uses a 0..5 V range.
// For a different source, set this to its full-scale voltage, e.g. 3.3f.
#define INPUT_FULL_SCALE_VOLTS 5.0f
#define FRAME_GAP_US 100000u
#define BIT_NS (1000000000ull / CAN_BITRATE)
#define SAMPLE_NS (BIT_NS * 3 / 4)

typedef enum { WAIT_FRAME, WAIT_IDLE, DRIVE_BIT, SAMPLE_BIT } phase_t;

// These are adjustable test ranges, not engine operating limits.
// Temperatures: degrees C. Pressures: hundredths of a bar.
typedef struct { const char *pin; int16_t min, max; } channel_t;
static const channel_t channels[CHANNEL_COUNT] = {
  { "IAT",      -40,  120 },
  { "BOOST",   -100,  300 }, // Gauge pressure: -1.00 .. +3.00 bar
  { "COOLANT",  -40,  140 },
  { "OIL_TEMP", -40,  180 },
  { "EGT",        0, 1100 },
  { "OIL_P",      0, 1000 }, // 0.00 .. 10.00 bar
};

typedef struct {
  pin_t inputs[CHANNEL_COUNT], en, canh, canl;
  timer_t timer;
  phase_t phase;
  uint8_t bits[160]; // More than enough for our stuffed 2-byte frame.
  unsigned count, index, ack_index, idle_count;
  int16_t value;
  unsigned channel;
  bool ack_seen;
  unsigned completed, aborted;
} chip_state_t;

static void release_bus(chip_state_t *c) {
  pin_mode(c->canh, INPUT_PULLDOWN);
  pin_mode(c->canl, INPUT_PULLUP);
}

static void drive_bit(chip_state_t *c, uint8_t bit) {
  if (bit == 0) {
    pin_mode(c->canh, OUTPUT_HIGH);
    pin_mode(c->canl, OUTPUT_LOW);
  } else {
    release_bus(c);
  }
}

static uint8_t read_bus(chip_state_t *c) {
  return (pin_read(c->canh) == HIGH && pin_read(c->canl) == LOW) ? 0 : 1;
}

// Append a field most-significant bit first, as CAN transmits it.
static void append_field(uint8_t *bits, unsigned *n, uint32_t value,
                         unsigned width) {
  for (unsigned i = width; i > 0; --i) {
    bits[(*n)++] = (value >> (i - 1)) & 1u;
  }
}

static void build_frame(chip_state_t *c) {
  float volts = pin_adc_read(c->inputs[c->channel]);
  if (!(volts >= 0)) volts = 0; // Also handles a NaN input defensively.
  if (volts > INPUT_FULL_SCALE_VOLTS) volts = INPUT_FULL_SCALE_VOLTS;
  const channel_t *input = &channels[c->channel];
  float scaled = input->min + volts * (input->max - input->min) / INPUT_FULL_SCALE_VOLTS;
  c->value = (int16_t)(scaled >= 0 ? scaled + 0.5f : scaled - 0.5f);

  uint8_t raw[64];
  unsigned n = 0;
  append_field(raw, &n, 0, 1);             // Start of frame
  append_field(raw, &n, FRAME_ID + c->channel, 11); // One ID per input
  append_field(raw, &n, 0, 1);             // RTR: data frame
  append_field(raw, &n, 0, 1);             // IDE: 11-bit identifier
  append_field(raw, &n, 0, 1);             // Reserved bit
  append_field(raw, &n, 2, 4);             // DLC: two data bytes
  append_field(raw, &n, (uint16_t)c->value, 16); // Two's complement, high byte first

  // Classical CAN CRC-15, polynomial 0x4599, initial value zero.
  // Compute over UNSTUFFED SOF through the end of data.
  uint16_t crc = 0;
  for (unsigned i = 0; i < n; ++i) {
    bool feedback = ((crc >> 14) & 1u) ^ raw[i];
    crc = (crc << 1) & 0x7fffu;
    if (feedback) crc ^= 0x4599u;
  }
  append_field(raw, &n, crc, 15);

  // Stuff SOF through CRC: after five equal bits insert the opposite bit.
  // Insert even when the fifth equal bit is the final CRC bit.
  c->count = 0;
  unsigned run = 0;
  uint8_t previous = 2;
  for (unsigned i = 0; i < n; ++i) {
    uint8_t bit = raw[i];
    c->bits[c->count++] = bit;
    run = (bit == previous) ? run + 1 : 1;
    previous = bit;
    if (run == 5) {
      previous = bit ^ 1u;
      c->bits[c->count++] = previous;
      run = 1;
    }
  }

  // These fixed fields are not stuffed.
  c->bits[c->count++] = 1;                // CRC delimiter
  c->ack_index = c->count;
  c->bits[c->count++] = 1;                // Release for receiver ACK
  c->bits[c->count++] = 1;                // ACK delimiter
  for (unsigned i = 0; i < 10; ++i) {
    c->bits[c->count++] = 1;              // Seven EOF + three intermission
  }
  c->index = 0;
  c->ack_seen = false;
}

static void wait_for_next_frame(chip_state_t *c) {
  release_bus(c);
  c->phase = WAIT_FRAME;
  timer_start(c->timer, FRAME_GAP_US, false);
}

static void on_timer(void *user_data) {
  chip_state_t *c = user_data;

  // EN can be tied high or controlled by a spare ESP32 GPIO.
  // Disabling mid-frame intentionally truncates it; the receiver may flag an error.
  if (pin_read(c->en) == LOW) {
    wait_for_next_frame(c);
    return;
  }

  switch (c->phase) {
    case WAIT_FRAME:
      c->idle_count = 0;
      c->phase = WAIT_IDLE;
      timer_start_ns(c->timer, BIT_NS, false);
      return;

    case WAIT_IDLE:
      // Simple idle gate for this single-sender bench setup.
      // This is not a general CAN receiver or a full bus-integration algorithm.
      c->idle_count = read_bus(c) ? c->idle_count + 1 : 0;
      if (c->idle_count < 11) {
        timer_start_ns(c->timer, BIT_NS, false);
        return;
      }
      build_frame(c);
      c->phase = DRIVE_BIT;
      // Fall through: begin SOF immediately after the idle check.
      /* fall through */
    case DRIVE_BIT:
      if (c->index == c->count) {
        ++c->completed;
        printf("ECU: %s raw=%d ID=0x%03x ACK=%s\n",
               channels[c->channel].pin, c->value, FRAME_ID + c->channel,
               c->ack_seen ? "yes" : "NO (check receiver)");
        c->channel = (c->channel + 1) % CHANNEL_COUNT;
        wait_for_next_frame(c);
        return;
      }
      drive_bit(c, c->bits[c->index]);
      c->phase = SAMPLE_BIT;
      timer_start_ns(c->timer, SAMPLE_NS, false);
      return;

    case SAMPLE_BIT: {
      uint8_t observed = read_bus(c);
      if (c->index == c->ack_index) {
        c->ack_seen = observed == 0;
        // Stimulus mode: finish the frame even without ACK. No CAN error flag
        // or immediate retry; report it and try fresh data after the gap.
      } else if (observed != c->bits[c->index]) {
        ++c->aborted;
        if (c->aborted == 1 || c->aborted % 10 == 0) {
          printf("ECU: bus mismatch at bit %u; releasing bus\n", c->index);
        }
        wait_for_next_frame(c);
        return;
      }
      ++c->index;
      c->phase = DRIVE_BIT;
      timer_start_ns(c->timer, BIT_NS - SAMPLE_NS, false);
      return;
    }
  }
}

void chip_init(void) {
  chip_state_t *c = calloc(1, sizeof(*c));
  if (!c) return;
  pin_init("VCC", INPUT);
  pin_init("GND", INPUT);
  for (unsigned i = 0; i < CHANNEL_COUNT; ++i)
    c->inputs[i] = pin_init(channels[i].pin, ANALOG);
  c->en = pin_init("EN", INPUT_PULLUP);
  c->canh = pin_init("CANH", INPUT_PULLDOWN);
  c->canl = pin_init("CANL", INPUT_PULLUP);

  const timer_config_t config = { .callback = on_timer, .user_data = c };
  c->timer = timer_init(&config);
  c->phase = WAIT_FRAME;
  // Give the gauge time to initialize before the first message.
  timer_start(c->timer, 1000000, false);
}

