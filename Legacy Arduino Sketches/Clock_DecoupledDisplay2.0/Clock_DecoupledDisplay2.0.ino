#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_TLC5947.h>
#include <SE_BME680.h>   // from the README you uploaded

// =========================
// I2C
// =========================
#define SDA_PIN 22
#define SCL_PIN 23

// =========================
// TLC5947
// =========================
#define TLC_NUM   1
#define TLC_DATA  D10
#define TLC_CLK   D8
#define TLC_LAT   D9   // OE tied to LAT on your board
Adafruit_TLC5947 tlc(TLC_NUM, TLC_CLK, TLC_DATA, TLC_LAT);

// =========================
// TD62783 rows (active HIGH)
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
// Timing (serial adjustable)
// =========================
static volatile uint32_t g_rowOnUs = 5000;  // sweet spot default

// =========================
// Colon config
// =========================
enum ColonMode : uint8_t { COLON_STATIC_ON, COLON_STATIC_OFF, COLON_BLINK_1HZ };
static ColonMode COLON_MODE = COLON_STATIC_ON;

// =========================
// Brightness (as requested)
// =========================
static const uint16_t PWM_NORMAL = 1000;
static const uint16_t PWM_RH     = 4095;

// =========================
// RTC + BME680
// =========================
RTC_DS3231 rtc;
SE_BME680 bme;

// Optional: tweak this after you test in your enclosure
// (Offset is in Celsius; negative reduces reported temp)
static const float TEMP_OFFSET_C = -2.0f;

// =========================
// Your confirmed TLC channel mapping
// =========================
// Clock segments
const uint8_t CLK_CH_A =  8;
const uint8_t CLK_CH_B =  9;
const uint8_t CLK_CH_C = 20;
const uint8_t CLK_CH_D = 21;
const uint8_t CLK_CH_E = 23;
const uint8_t CLK_CH_F = 10;
const uint8_t CLK_CH_G = 22;

// Humidity segments
const uint8_t HUM_CH_A =  2;
const uint8_t HUM_CH_B =  5;
const uint8_t HUM_CH_C =  7;
const uint8_t HUM_CH_D =  4;
const uint8_t HUM_CH_E =  6;
const uint8_t HUM_CH_F =  0;
const uint8_t HUM_CH_G =  1;
const uint8_t CH_RH    =  3;   // %RH cluster

// Temperature segments
const uint8_t TMP_CH_A = 17;
const uint8_t TMP_CH_B = 14;
const uint8_t TMP_CH_C = 12;
const uint8_t TMP_CH_D = 15;
const uint8_t TMP_CH_E = 13;
const uint8_t TMP_CH_F = 19;
const uint8_t TMP_CH_G = 18;
const uint8_t CH_DEGREES = 16; // degrees/decimal dot group

// Shared dot group
const uint8_t CH_COLON = 11;   // colon / AMPM / upper dot group

// =========================
// Row <-> digit mapping you verified
// Clock: row0=Digit2, row1=Digit4, row2=Digit1, row3=Digit3
// =========================
const int8_t rowToClockDigit[4] = { 1, 3, 0, 2 };
// Humidity: Digit5 (tens) on CA4(row3), Digit6 (ones) on CA2(row1)
const int8_t rowToHumDigit[4]   = { -1, 1, -1, 0 };
// Temperature: Digit7 on CA3(row2), Digit8 on CA1(row0), Digit9 on CA4(row3), Digit10 on CA2(row1)
const int8_t rowToTempDigit[4]  = { 1, 3, 0, 2 };

// Rows used for PM + colon
const uint8_t digitToRow_clockDigit1 = 2; // Digit1 -> row2
const uint8_t digitToRow_clockDigit3 = 3; // Digit3 -> row3

// =========================
// 7-seg digit masks (abcdefg), bit0=A .. bit6=G
// =========================
const uint8_t DIGIT_MASKS[10] = {
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
// 'F' on 7-seg: A, E, F, G
const uint8_t MASK_F = 0b01110001;

// =========================
// Display model
// =========================
uint8_t clockDigits[4] = {0xFF, 0, 0, 0};        // Digit1..Digit4
uint8_t humDigits[2]   = {0xFF, 0xFF};           // Digit5..Digit6
uint8_t tempDigits[4]  = {0xFF, 0xFF, 0xFF, 0xFF}; // Digit7..Digit10 (idx3 is 'F')
bool isPM = false;

// =========================
// Framebuffer: 4 rows x 24 channels
// =========================
uint16_t fb[4][24];

// =========================
// Helpers (compose only; no IO)
// =========================
static inline void clearRow(uint16_t row[24]) { for (uint8_t i=0;i<24;i++) row[i]=0; }
static inline void setCh(uint16_t row[24], uint8_t ch, uint16_t pwm) { if (ch < 24) row[ch] = pwm; }

static inline void applyMask(uint16_t row[24], uint8_t mask,
                             uint8_t chA,uint8_t chB,uint8_t chC,uint8_t chD,uint8_t chE,uint8_t chF,uint8_t chG,
                             uint16_t pwm) {
  if (mask & (1 << 0)) setCh(row, chA, pwm);
  if (mask & (1 << 1)) setCh(row, chB, pwm);
  if (mask & (1 << 2)) setCh(row, chC, pwm);
  if (mask & (1 << 3)) setCh(row, chD, pwm);
  if (mask & (1 << 4)) setCh(row, chE, pwm);
  if (mask & (1 << 5)) setCh(row, chF, pwm);
  if (mask & (1 << 6)) setCh(row, chG, pwm);
}

static inline void applyDigit(uint16_t row[24], uint8_t value,
                              uint8_t chA,uint8_t chB,uint8_t chC,uint8_t chD,uint8_t chE,uint8_t chF,uint8_t chG,
                              uint16_t pwm) {
  if (value > 9) return;
  applyMask(row, DIGIT_MASKS[value], chA,chB,chC,chD,chE,chF,chG, pwm);
}

static bool colonStateForSecond(int sec) {
  switch (COLON_MODE) {
    case COLON_STATIC_ON:  return true;
    case COLON_STATIC_OFF: return false;
    case COLON_BLINK_1HZ:  return (sec % 2) == 0;
    default: return true;
  }
}

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

static void composeRow(uint8_t rowIdx, uint16_t outRow[24], bool colonOn) {
  clearRow(outRow);

  // ---- CLOCK ----
  int8_t clkIdx = rowToClockDigit[rowIdx];
  if (clkIdx >= 0 && clkIdx < 4) {
    uint8_t v = clockDigits[clkIdx];
    if (v != 0xFF) applyDigit(outRow, v, CLK_CH_A,CLK_CH_B,CLK_CH_C,CLK_CH_D,CLK_CH_E,CLK_CH_F,CLK_CH_G, PWM_NORMAL);

    bool colonThisRow = (rowIdx == digitToRow_clockDigit3) && colonOn;
    bool pmThisRow    = (rowIdx == digitToRow_clockDigit1) && isPM;
    if (colonThisRow || pmThisRow) setCh(outRow, CH_COLON, PWM_NORMAL);
  }

  // ---- HUMIDITY ----
  int8_t humIdx = rowToHumDigit[rowIdx];
  if (humIdx >= 0 && humIdx < 2) {
    uint8_t v = humDigits[humIdx];
    if (v != 0xFF) applyDigit(outRow, v, HUM_CH_A,HUM_CH_B,HUM_CH_C,HUM_CH_D,HUM_CH_E,HUM_CH_F,HUM_CH_G, PWM_NORMAL);
    if (rowIdx == 3) setCh(outRow, CH_RH, PWM_RH);  // RH cluster lives on row3
  }

  // ---- TEMPERATURE ----
  int8_t tmpIdx = rowToTempDigit[rowIdx];
  if (tmpIdx >= 0 && tmpIdx < 4) {
    if (tmpIdx == 3) {
      // render 'F'
      applyMask(outRow, MASK_F, TMP_CH_A,TMP_CH_B,TMP_CH_C,TMP_CH_D,TMP_CH_E,TMP_CH_F,TMP_CH_G, PWM_NORMAL);
      // degree dot shares CH_DEGREES
      setCh(outRow, CH_DEGREES, PWM_NORMAL);
    } else {
      uint8_t v = tempDigits[tmpIdx];
      if (v != 0xFF) applyDigit(outRow, v, TMP_CH_A,TMP_CH_B,TMP_CH_C,TMP_CH_D,TMP_CH_E,TMP_CH_F,TMP_CH_G, PWM_NORMAL);
      // decimal dot on tenths digit (tmpIdx==2)
      if (tmpIdx == 2) setCh(outRow, CH_DEGREES, PWM_NORMAL);
    }
  }
}

static void rebuildFrame(bool colonOn) {
  for (uint8_t r=0; r<4; r++) composeRow(r, fb[r], colonOn);
}

// =========================
// TLC push
// =========================
static inline void pushRowToTLC(const uint16_t row[24]) {
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, row[ch]);
  tlc.write(); // OE tied to LAT => blanks during latch automatically
}

// =========================
// Multiplex: resync-only
// =========================
static uint8_t  curRow = 0;
static uint32_t rowStartUs = 0;

static void multiplexService() {
  uint32_t us = micros();
  if (rowStartUs == 0) {
    rowStartUs = us;
    curRow = 0;
    rowsOffAll();
    pushRowToTLC(fb[curRow]);
    rowOn(curRow);
    return;
  }

  if ((uint32_t)(us - rowStartUs) >= g_rowOnUs) {
    rowStartUs = us;           // resync (never catch up)

    rowOff(curRow);
    curRow = (curRow + 1) & 0x03;

    pushRowToTLC(fb[curRow]);
    rowOn(curRow);
  }
}

// =========================
// Model updates (1Hz, lazy)
// =========================
static void updateClockFromRTC(DateTime now) {
  int hours24 = now.hour();
  int minutes = now.minute();

  isPM = (hours24 >= 12);
  int hours12 = hours24 % 12;
  if (hours12 == 0) hours12 = 12;

  // HH:MM with leading blank
  clockDigits[0] = (hours12 >= 10) ? (hours12 / 10) : 0xFF;
  clockDigits[1] = hours12 % 10;
  clockDigits[2] = minutes / 10;
  clockDigits[3] = minutes % 10;
}

static void updateEnvFromBME() {
  // This library expects performReading() cadence to be consistent.
  // We call it exactly once per second in the 1Hz tick.
  if (!bme.performReading()) return;

  // Prefer compensated values per the README
  float tc = bme.temperature_compensated;
  float hc = bme.humidity_compensated;

  // RH: integer 0-99
  if (isfinite(hc)) {
    int rh = (int)lroundf(hc);
    rh = constrain(rh, 0, 99);
    humDigits[0] = rh / 10;
    humDigits[1] = rh % 10;
  }

  // Temp: °F to 0.1, 0.0..99.9 => " 74.6F"
  if (isfinite(tc)) {
    float tF = c_to_f(tc);
    tF = constrain(tF, 0.0f, 99.9f);

    int t10 = (int)lroundf(tF * 10.0f);
    int ip  = t10 / 10;
    int th  = t10 % 10;

    int tens = ip / 10;
    int ones = ip % 10;

    tempDigits[0] = (tens > 0) ? tens : 0xFF;
    tempDigits[1] = ones;
    tempDigits[2] = th;
    tempDigits[3] = 0xFF; // 'F' is rendered by index
  }
}

// =========================
// Serial: row timing only
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
        if (v < 200) v = 200;
        if (v > 20000) v = 20000;
        g_rowOnUs = v;
        Serial.print("OK rowOnUs=");
        Serial.println(g_rowOnUs);
      } else if (line == "show") {
        Serial.print("rowOnUs=");
        Serial.println(g_rowOnUs);
      } else if (line.length()) {
        Serial.println("Commands: on <us> | show");
      }
      line = "";
    } else {
      if (line.length() < 40) line += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(150);

  // Rows
  for (int i = 0; i < 4; i++) {
    pinMode(ROW_PINS[i], OUTPUT);
    digitalWrite(ROW_PINS[i], LOW);
  }

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found");
  } else if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // BME680
  if (!bme.begin()) {
    Serial.println("BME680 setup failed");
  } else {
    bme.setTemperatureCompensation(TEMP_OFFSET_C); // TEMP_OFFSET_C is a float
  }

  // TLC
  tlc.begin();
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, 0);
  tlc.write();

  // Build initial frame
  DateTime now = rtc.now();
  updateClockFromRTC(now);
  updateEnvFromBME();
  bool colonOn = colonStateForSecond(now.second());
  rebuildFrame(colonOn);

  Serial.println("OK. Commands: on <us> | show");
}

void loop() {
  // real-time
  multiplexService();

  // optional tuning
  handleSerial();

  // lazy updates (1 Hz, no catch-up)
  static uint32_t lastTickMs = 0;
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastTickMs) >= 1000) {
    lastTickMs = nowMs; // resync

    DateTime now = rtc.now();
    updateClockFromRTC(now);
    updateEnvFromBME();

    bool colonOn = colonStateForSecond(now.second());
    rebuildFrame(colonOn);
  }
}