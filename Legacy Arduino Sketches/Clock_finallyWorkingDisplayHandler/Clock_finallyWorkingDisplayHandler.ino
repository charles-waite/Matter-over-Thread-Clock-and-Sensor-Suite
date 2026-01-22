#include <Arduino.h>
#include <Adafruit_TLC5947.h>

// ============================================================
// TLC5947 wiring (adjust pins if needed)
// ============================================================
#define TLC_NUM   1
#define TLC_DATA  D10
#define TLC_CLK   D8
#define TLC_LAT   D9   // OE tied to LAT on your board
Adafruit_TLC5947 tlc(TLC_NUM, TLC_CLK, TLC_DATA, TLC_LAT);

// ============================================================
// TD62783 commons (row index: 0..3)
// ============================================================
#define ROW1_PIN  D0
#define ROW2_PIN  D1
#define ROW3_PIN  D2
#define ROW4_PIN  D3
static const uint8_t ROW_PINS[4] = { ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };

static inline void rowsOffAll() { for (int i = 0; i < 4; i++) digitalWrite(ROW_PINS[i], LOW); }
static inline void rowOn(uint8_t r)  { digitalWrite(ROW_PINS[r], HIGH); }
static inline void rowOff(uint8_t r) { digitalWrite(ROW_PINS[r], LOW);  }

// ============================================================
// Modes
// ============================================================
static const uint8_t MODE_TLC_ONLY = 1;
static const uint8_t MODE_TD_ONLY  = 2;
static volatile uint8_t gMode = MODE_TLC_ONLY;

// Pick any row to hold ON in MODE_TLC_ONLY
static const uint8_t FIXED_ROW = 3;   // CA4

// Timing
static const uint32_t g_rowOnUs = 5000;   // default row ON time (us)
static volatile uint32_t g_rowOffUs = 0;     // optional, leave 0 for now
static const uint32_t MODE1_PATTERN_MS = 250;  // or whatever you were using before

// Brightness
static const uint16_t PWM_ON  = 4095;
static const uint16_t PWM_OFF = 0;

// ============================================================
// MODE 1 state
// ============================================================
static uint16_t frameA[24];
static uint16_t frameB[24];
static uint16_t frameCur[24];
static uint32_t lastToggleMs = 0;
static bool useA = true;

static void makeSparseFrame(uint16_t out[24], uint8_t onCount) {
  for (int i = 0; i < 24; i++) out[i] = PWM_OFF;

  for (uint8_t k = 0; k < onCount; k++) {
    uint8_t ch;
    do { ch = (uint8_t)random(0, 24); } while (out[ch] != PWM_OFF);
    out[ch] = PWM_ON;
  }
}

static void applyFrameDiff(const uint16_t next[24]) {
  for (uint8_t ch = 0; ch < 24; ch++) {
    if (frameCur[ch] != next[ch]) {
      tlc.setPWM(ch, next[ch]);
      frameCur[ch] = next[ch];
    }
  }
  tlc.write();
}

static void mode1_init() {
  rowsOffAll();
  rowOn(FIXED_ROW);

  for (int i = 0; i < 24; i++) frameCur[i] = 0xFFFF; // force first update

  makeSparseFrame(frameA, 8);
  makeSparseFrame(frameB, 10);

  lastToggleMs = millis();
  useA = true;
  applyFrameDiff(frameA);
}

static void mode1_service() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastToggleMs) >= MODE1_PATTERN_MS) {
    lastToggleMs += MODE1_PATTERN_MS;
    useA = !useA;
    applyFrameDiff(useA ? frameA : frameB);
  }
}

// ============================================================
// MODE 2 state
// ============================================================
static uint8_t curRow = 0;
static uint32_t rowStartUs = 0;

static void mode2_init() {
  // Latch one TLC pattern ONCE
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, PWM_OFF);
  for (uint8_t ch = 0; ch < 24; ch++) {
    if (ch % 2 == 0) tlc.setPWM(ch, PWM_ON);
  }
  tlc.write(); // ONLY write in Mode 2

  rowsOffAll();
  curRow = 0;
  rowOn(curRow);
  rowStartUs = micros();
}

static void mode2_service() {
  uint32_t us = micros();
  if ((uint32_t)(us - rowStartUs) >= g_rowOnUs) {
    rowStartUs = us;
    rowOff(curRow);
    curRow = (curRow + 1) & 0x03;
    rowOn(curRow);
  }
}

// ============================================================
// Mode switcher
// ============================================================
static void setMode(uint8_t m) {
  gMode = m;
  if (gMode == MODE_TLC_ONLY) mode1_init();
  else                       mode2_init();
}

// ===========================================
// Serial Handler for runtime input
// ===========================================
static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  on <us>     set row ON time in microseconds (ex: on 3200)");
  Serial.println("  off <us>    set row OFF/blank time in microseconds (ex: off 200)");
  Serial.println("  show        print current timings");
  Serial.println("  ?           help");
  Serial.println();
}

static void showTimings() {
  Serial.print("Row ON  (us): "); Serial.println(g_rowOnUs);
  Serial.print("Row OFF (us): "); Serial.println(g_rowOffUs);
}

static void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line.trim();
      if (line.startsWith("on ")) {
        uint32_t v = (uint32_t) line.substring(3).toInt();
        if (v < 50) v = 50;
        if (v > 20000) v = 20000;
        g_rowOnUs = v;
        Serial.print("OK: row ON = "); Serial.print(g_rowOnUs); Serial.println(" us");
      } else if (line == "show") {
        Serial.print("row ON = "); Serial.print(g_rowOnUs); Serial.println(" us");
      } else if (line == "?" || line == "help") {
        Serial.println("Commands: on <us> | show | ?");
      } else if (line.length()) {
        Serial.println("Unrecognized. Type '?' for help.");
      }
      line = "";
    } else {
      if (line.length() < 32) line += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  for (int i = 0; i < 4; i++) {
    pinMode(ROW_PINS[i], OUTPUT);
    digitalWrite(ROW_PINS[i], LOW);
  }

  tlc.begin();
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, 0);
  tlc.write();

  randomSeed((uint32_t)esp_random());

  setMode(MODE_TLC_ONLY);

  Serial.println("\nSend '1' = TLC-only, '2' = TD-only");
}

void loop() {
  handleSerial();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '1') { Serial.println("MODE 1"); setMode(MODE_TLC_ONLY); }
    if (c == '2') { Serial.println("MODE 2"); setMode(MODE_TD_ONLY);  }
  }

  if (gMode == MODE_TLC_ONLY) mode2_service();
  else                       mode1_service();
}