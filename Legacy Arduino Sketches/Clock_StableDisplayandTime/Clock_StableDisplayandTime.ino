#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_TLC5947.h>

// =========================
// I2C / RTC
// =========================
#define SDA_PIN  22
#define SCL_PIN  23
RTC_DS3231 rtc;

// =========================
// TLC5947 wiring
// OE is physically tied to LAT on your board
// =========================
#define TLC_NUM   1
#define TLC_DATA  D10
#define TLC_CLK   D8
#define TLC_LAT   D9
Adafruit_TLC5947 tlc(TLC_NUM, TLC_CLK, TLC_DATA, TLC_LAT);

// =========================
// TD62783 commons (rows)
// Row0->TD1, Row1->TD2, Row2->TD3, Row3->TD4 (active HIGH)
// =========================
#define ROW1_PIN  D0
#define ROW2_PIN  D1
#define ROW3_PIN  D2
#define ROW4_PIN  D3
static const uint8_t ROW_PINS[4] = { ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };

static inline void rowsOffAll() { for (int i = 0; i < 4; i++) digitalWrite(ROW_PINS[i], LOW); }
static inline void rowOn(uint8_t r)  { digitalWrite(ROW_PINS[r], HIGH); }
static inline void rowOff(uint8_t r) { digitalWrite(ROW_PINS[r], LOW);  }

// =========================
// Timing (serial-tunable row on time)
// =========================
static volatile uint32_t g_rowOnUs = 5000;   // default sweet spot
static uint32_t nextDeadlineUs = 0;

// =========================
// Brightness
// =========================
static constexpr uint16_t PWM_SEG = 1000;   // clock segments
static constexpr uint16_t PWM_OFF = 0;

// =========================
// Colon behavior
// =========================
enum ColonMode : uint8_t { COLON_OFF=0, COLON_ON=1, COLON_BLINK_1HZ=2 };
static constexpr ColonMode COLON_MODE = COLON_ON;   // default: static
static constexpr bool SHOW_PM_DOT = true;           // uses same dot-group channel as colon

// =========================
// Clock wiring mapping you verified
// row0=Digit2, row1=Digit4, row2=Digit1, row3=Digit3
// digits are indexed 0..3 for D1..D4
// =========================
static const int8_t rowToClockDigit[4] = { 1, 3, 0, 2 };
static constexpr uint8_t ROW_FOR_DIGIT1 = 2; // D1 -> row2
static constexpr uint8_t ROW_FOR_DIGIT3 = 3; // D3 -> row3

// =========================
// TLC channels (from your confirmed mapping)
// =========================
static constexpr uint8_t CLK_CH_A =  8;
static constexpr uint8_t CLK_CH_B =  9;
static constexpr uint8_t CLK_CH_C = 20;
static constexpr uint8_t CLK_CH_D = 21;
static constexpr uint8_t CLK_CH_E = 23;
static constexpr uint8_t CLK_CH_F = 10;
static constexpr uint8_t CLK_CH_G = 22;
static constexpr uint8_t CH_DOTGROUP = 11; // Colon / AMPM / upper dot group

// =========================
// 7-seg digit masks (abcdefg), bit0=A .. bit6=G
// =========================
static const uint8_t DIGIT_MASKS[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

// =========================
// Display model (clock only)
// =========================
static uint8_t clockDigits[4] = {0xFF, 0, 0, 0}; // D1..D4, 0xFF=blank
static bool colonOn = true;
static bool isPM = false;
static int lastSecond = -1;

// =========================
// Framebuffers: fb used by mux, bb built at 1Hz
// =========================
static volatile uint16_t fb[4][24];
static uint16_t          bb[4][24];
static volatile bool     swapPending = false;

// =========================
// Helpers
// =========================
static inline void clearRow(uint16_t row[24]) {
  for (uint8_t i = 0; i < 24; i++) row[i] = 0;
}

static inline void setCh(uint16_t row[24], uint8_t ch, uint16_t pwm) {
  if (ch < 24) row[ch] = pwm;
}

static inline void applyMask(uint16_t row[24], uint8_t mask, uint16_t pwm) {
  if (mask & (1 << 0)) setCh(row, CLK_CH_A, pwm);
  if (mask & (1 << 1)) setCh(row, CLK_CH_B, pwm);
  if (mask & (1 << 2)) setCh(row, CLK_CH_C, pwm);
  if (mask & (1 << 3)) setCh(row, CLK_CH_D, pwm);
  if (mask & (1 << 4)) setCh(row, CLK_CH_E, pwm);
  if (mask & (1 << 5)) setCh(row, CLK_CH_F, pwm);
  if (mask & (1 << 6)) setCh(row, CLK_CH_G, pwm);
}

static inline void applyDigit(uint16_t row[24], uint8_t v, uint16_t pwm) {
  if (v > 9) return;
  applyMask(row, DIGIT_MASKS[v], pwm);
}

// Compose one multiplex row into outRow (clock only)
static void composeClockRow(uint8_t rowIdx, uint16_t outRow[24]) {
  clearRow(outRow);

  int8_t clkIdx = rowToClockDigit[rowIdx]; // which of D1..D4 is on this row
  if (clkIdx >= 0 && clkIdx < 4) {
    uint8_t v = clockDigits[clkIdx];
    if (v != 0xFF) {
      if (v > 9) v = 0;
      applyDigit(outRow, v, PWM_SEG);
    }
  }

  // Colon / PM dot share the same TLC channel, but live on different rows.
  bool wantColon = false;
  if (COLON_MODE == COLON_ON) wantColon = true;
  else if (COLON_MODE == COLON_BLINK_1HZ) wantColon = colonOn;

  if (wantColon && rowIdx == ROW_FOR_DIGIT3) {
    setCh(outRow, CH_DOTGROUP, PWM_SEG);
  }

  if (SHOW_PM_DOT && isPM && rowIdx == ROW_FOR_DIGIT1) {
    setCh(outRow, CH_DOTGROUP, PWM_SEG);
  }
}

static void rebuildBackBuffer_ClockOnly() {
  for (uint8_t r = 0; r < 4; r++) {
    composeClockRow(r, bb[r]);
  }
  swapPending = true;
}

// =========================
// RTC -> clock digits (12h)
// =========================
static void updateClockFromRTC_1Hz() {
  DateTime now = rtc.now();
  if (now.second() == lastSecond) return;
  lastSecond = now.second();

  colonOn = (now.second() & 1) == 0;

  int h24 = now.hour();
  int m   = now.minute();

  isPM = (h24 >= 12);
  int h12 = h24 % 12;
  if (h12 == 0) h12 = 12;

  // HH:MM with leading blank for 1-9
  clockDigits[0] = (h12 >= 10) ? (h12 / 10) : 0xFF;
  clockDigits[1] = (uint8_t)(h12 % 10);
  clockDigits[2] = (uint8_t)(m / 10);
  clockDigits[3] = (uint8_t)(m % 10);
}

// =========================
// TLC push
// =========================
static inline void pushRowToTLC(const volatile uint16_t row[24]) {
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, row[ch]);
  tlc.write(); // OE tied to LAT -> blanks only here
}

// =========================
// Multiplex tick (NO catch-up; resync only)
// =========================
static volatile uint8_t activeRow = 0;

static void multiplexTick_NoCatchup() {
  uint32_t nowUs = micros();

  if (nextDeadlineUs == 0) {
    nextDeadlineUs = nowUs + g_rowOnUs;
    activeRow = 0;
    rowsOffAll();
    pushRowToTLC(fb[0]);
    rowOn(0);
    return;
  }

  // Not time yet
  if ((int32_t)(nowUs - nextDeadlineUs) < 0) return;

  // Time (or late): resync to *now + dwell* (never "catch up")
  nextDeadlineUs = nowUs + g_rowOnUs;

  // OFF current row
  rowOff(activeRow);

  // Next row
  uint8_t nextRow = (activeRow + 1) & 0x03;

  // Load PWM for next row and latch
  pushRowToTLC(fb[nextRow]);

  // ON next row
  rowOn(nextRow);
  activeRow = nextRow;

  // Swap only on frame boundary (after row3 -> row0)
  if (activeRow == 0 && swapPending) {
    noInterrupts();
    memcpy((void*)fb, (void*)bb, sizeof(bb));
    swapPending = false;
    interrupts();
  }
}

// =========================
// Serial: "on <us>" and "show"
// =========================
static void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.startsWith("on ")) {
        uint32_t v = (uint32_t) line.substring(3).toInt();
        if (v < 100) v = 100;
        if (v > 20000) v = 20000;
        g_rowOnUs = v;
        Serial.print("OK: row ON = "); Serial.print(g_rowOnUs); Serial.println(" us");
      } else if (line == "show") {
        Serial.print("row ON = "); Serial.print(g_rowOnUs); Serial.println(" us");
      } else if (line.length()) {
        Serial.println("Commands: on <us> | show");
      }
      line = "";
    } else {
      if (line.length() < 64) line += c;
    }
  }
}

// =========================
// setup / loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(150);

  // Row pins
  for (int i = 0; i < 4; i++) {
    pinMode(ROW_PINS[i], OUTPUT);
    digitalWrite(ROW_PINS[i], LOW);
  }

  // TLC
  tlc.begin();
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, 0);
  tlc.write();

  // I2C + RTC
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Clear buffers
  memset((void*)fb, 0, sizeof(fb));
  memset((void*)bb, 0, sizeof(bb));

  // Initial draw
  updateClockFromRTC_1Hz();
  rebuildBackBuffer_ClockOnly();
  noInterrupts();
  memcpy((void*)fb, (void*)bb, sizeof(bb));
  swapPending = false;
  interrupts();

  Serial.println("\nClock mux running. Commands: on <us> | show");
}

void loop() {
  multiplexTick_NoCatchup();
  handleSerial();

  // Lazy UI update: 1Hz max
  static uint32_t lastMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs >= 250) {           // poll RTC a few times/sec, but it only updates on second change
    lastMs = nowMs;
    updateClockFromRTC_1Hz();
    rebuildBackBuffer_ClockOnly();
  }
}