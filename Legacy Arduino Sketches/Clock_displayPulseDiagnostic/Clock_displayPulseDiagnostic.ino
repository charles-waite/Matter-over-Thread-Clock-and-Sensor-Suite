#include <Arduino.h>
#include <Adafruit_TLC5947.h>

// =========================
// USER SETTINGS
// =========================
volatile uint32_t ROW_DWELL_US = 2400;

// Per-channel brightness (12-bit: 0..4095)
#define SEG_BRIGHT_NORMAL  1800
#define SEG_BRIGHT_RH      4095


// If we fall behind by more than this many row periods, resync timing
#define RESYNC_MULT        2

// =========================
// TLC5947 (your wiring)
// =========================
#define TLC_NUM   1
#define TLC_DATA  D10
#define TLC_CLK   D8
#define TLC_LAT   D9   // NOTE: OE is physically tied to LAT on your board

Adafruit_TLC5947 tlc(TLC_NUM, TLC_CLK, TLC_DATA, TLC_LAT);

// RH% cluster channel (your confirmed mapping)
const uint8_t CH_RH = 3;

// =========================
// TD62783 commons (your wiring)
// =========================
#define ROW1_PIN  D0   // CA1
#define ROW2_PIN  D1   // CA2
#define ROW3_PIN  D2   // CA3
#define ROW4_PIN  D3   // CA4
const uint8_t rowPins[4] = { ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };

// =========================
// Multiplex state
// =========================
uint8_t  currentRow      = 0;
uint32_t lastRowSwitchUs = 0;

// =========================
// Helpers
// =========================

static inline void programAllOnPattern() {
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, SEG_BRIGHT_NORMAL);

  // Boost RH% cluster only
  tlc.setPWM(CH_RH, SEG_BRIGHT_RH);

  // Latch once
  tlc.write();
}

static inline void renderRow(uint8_t rowIdx) {
  digitalWrite(rowPins[rowIdx], HIGH);
}

static inline void updateMultiplex() {
  uint32_t nowUs = micros();

  if (lastRowSwitchUs == 0) {
    lastRowSwitchUs = nowUs;
    renderRow(currentRow);
    return;
  }

  uint32_t elapsed = nowUs - lastRowSwitchUs;
  if (elapsed < ROW_DWELL_US) return;

  // If we fell way behind, don't "catch up" by rapid switching (looks like pulses).
  if (elapsed > (ROW_DWELL_US * RESYNC_MULT)) {
    lastRowSwitchUs = nowUs;  // resync
  } else {
    lastRowSwitchUs += ROW_DWELL_US;
  }

  currentRow = (currentRow + 1) & 0x03;
  renderRow(currentRow);
}

// Non-blocking-ish serial dwell tuning (still simple; avoid readStringUntil)
static inline void handleSerialDwell() {
  static char buf[16];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf[idx] = 0;
      idx = 0;
      long us = atol(buf);
      if (us >= 200 && us <= 20000) {
        ROW_DWELL_US = (uint32_t)us;
        Serial.print("ROW_DWELL_US=");
        Serial.println(ROW_DWELL_US);
      } else {
        Serial.println("Enter dwell in us (200..20000).");
      }
      return;
    }

    if (idx < sizeof(buf) - 1 && ((c >= '0' && c <= '9') || c == '-')) {
      buf[idx++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== TLC5947 ALL-ON / TD multiplex test (deadtime+resync) ===");
  Serial.println("OE is physically tied to LAT. No OE pin used.");
  Serial.println("Type a number to set ROW_DWELL_US (e.g. 2200)");

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], LOW);
  }

  tlc.begin();
  programAllOnPattern();   // latch once

  currentRow = 0;
  lastRowSwitchUs = 0;
}

void loop() {
  updateMultiplex();
  handleSerialDwell();
}