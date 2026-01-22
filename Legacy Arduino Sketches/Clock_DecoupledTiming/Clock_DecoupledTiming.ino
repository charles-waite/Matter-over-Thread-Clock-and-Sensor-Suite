#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "RTClib.h"
#include <Adafruit_TLC5947.h>
#include <bsec2.h>
#include <Preferences.h>

// ===  ======================
// PINS / HW
// =========================
#define SDA_PIN     22
#define SCL_PIN     23
#define BME_ADDR    0x77

// TLC5947 (OE is physically tied to LAT on your board)
#define TLC_NUM   1
#define TLC_DATA  D10
#define TLC_CLK   D8
#define TLC_LAT   D9

Adafruit_TLC5947 tlc(TLC_NUM, TLC_CLK, TLC_DATA, TLC_LAT);

// TD62783 commons
#define ROW1_PIN  D0   // CA1
#define ROW2_PIN  D1   // CA2
#define ROW3_PIN  D2   // CA3
#define ROW4_PIN  D3   // CA4
const uint8_t rowPins[4] = { ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };

// Fixed dwell
static constexpr uint32_t ROW_DWELL_US = 5000;

// =========================
// BRIGHTNESS (serial-tunable)
// =========================
volatile uint16_t SEG_BRIGHT_NORMAL = 1000; // serial: type "1600"
volatile uint16_t SEG_BRIGHT_RH     = 4095; // serial: type "rh 2000"

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
// TLC channels (confirmed)
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
// Display state (model)
// =========================
// Clock digits (Digit1..Digit4), 0xFF = blank
uint8_t clockDigits[4] = {0xFF, 0, 0, 0};
bool colonOn = true;
bool isPM    = false;

// Humidity: tens, ones
uint8_t humDigits[2] = {0xFF, 0xFF};

// Temp: [tens][ones][tenths][F]
uint8_t tempDigits[4] = {0xFF, 0xFF, 0xFF, 0xFF};

// =========================
// Framebuffers (renderer input)
// fb = front buffer used by multiplex
// bb = back buffer built by UI updates
// =========================
volatile uint16_t fb[4][24];
uint16_t          bb[4][24];
volatile bool     swapPending = false;

// =========================
// RTC
// =========================
RTC_DS3231 rtc;
int lastSecond = -1;

// =========================
// BSEC2 (BME680)
// =========================
Bsec2 env;
bsec_virtual_sensor_t sensorList[] = {
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
};

Preferences prefs;
const char* NVS_NS  = "bsec2";
const char* NVS_KEY = "state";
#define BSEC_STATE_SIZE 512

volatile float vTempC = NAN;
volatile float vHum   = NAN;

static void onBsecOutputs(const bme68xData, const bsecOutputs out, Bsec2) {
  for (uint8_t i = 0; i < out.nOutputs; i++) {
    const bsecData &o = out.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: vTempC = o.signal; break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:    vHum   = o.signal; break;
      default: break;
    }
  }
}

void loadBsecState() {
  if (!prefs.begin(NVS_NS, true)) return;
  size_t n = prefs.getBytesLength(NVS_KEY);
  if (n == BSEC_STATE_SIZE) {
    uint8_t buf[BSEC_STATE_SIZE];
    prefs.getBytes(NVS_KEY, buf, BSEC_STATE_SIZE);
    if (env.setState(buf)) Serial.println("[BSEC2] state restored");
  }
  prefs.end();
}

void saveBsecStateIfReady(uint32_t nowMs) {
  static const uint32_t EVERY = 5UL * 60UL * 1000UL;
  static uint32_t last = 0;
  if (nowMs - last < EVERY) return;

  uint8_t st[BSEC_STATE_SIZE];
  if (env.getState(st) && prefs.begin(NVS_NS, false)) {
    prefs.putBytes(NVS_KEY, st, BSEC_STATE_SIZE);
    prefs.end();
    last = nowMs;
    Serial.println("[BSEC2] state saved");
  }
}

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

// =========================
// Buffer helpers (NO hardware)
// =========================
static inline void clearRow(uint16_t row[24]) {
  for (uint8_t i = 0; i < 24; i++) row[i] = 0;
}

static inline void setCh(uint16_t row[24], uint8_t ch, uint16_t pwm) {
  if (ch < 24) row[ch] = pwm;
}

static inline void applyMaskRawBuf(uint16_t row[24], uint8_t mask,
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

static inline void applyDigitBuf(uint16_t row[24], uint8_t value,
                                 uint8_t chA,uint8_t chB,uint8_t chC,uint8_t chD,uint8_t chE,uint8_t chF,uint8_t chG,
                                 uint16_t pwm) {
  if (value > 9) return;
  applyMaskRawBuf(row, DIGIT_MASKS[value], chA,chB,chC,chD,chE,chF,chG, pwm);
}

// =========================
// Compose ONE row into bb[rowIdx][]
// (NO digitalWrite, NO tlc.setPWM, NO tlc.write)
// =========================
void composeRow(uint8_t rowIdx, uint16_t outRow[24]) {
  clearRow(outRow);

  const uint16_t pwmN  = SEG_BRIGHT_NORMAL;
  const uint16_t pwmRH = SEG_BRIGHT_RH;

  // ---- CLOCK ----
  int8_t clkIdx = rowToClockDigit[rowIdx];
  if (clkIdx >= 0 && clkIdx < 4) {
    uint8_t v = clockDigits[clkIdx];
    if (v != 0xFF) {
      if (v > 9) v = 0;
      applyDigitBuf(outRow, v, CLK_CH_A,CLK_CH_B,CLK_CH_C,CLK_CH_D,CLK_CH_E,CLK_CH_F,CLK_CH_G, pwmN);
    }

    bool colonThisRow = (rowIdx == digitToRow_clockDigit3) && colonOn;
    bool pmThisRow    = (rowIdx == digitToRow_clockDigit1) && isPM;
    if (colonThisRow || pmThisRow) {
      setCh(outRow, CH_COLON, pwmN);
    }
  }

  // ---- HUMIDITY ----
  int8_t humIdx = rowToHumDigit[rowIdx];
  if (humIdx >= 0 && humIdx < 2) {
    uint8_t v = humDigits[humIdx];
    if (v != 0xFF && v <= 9) {
      applyDigitBuf(outRow, v, HUM_CH_A,HUM_CH_B,HUM_CH_C,HUM_CH_D,HUM_CH_E,HUM_CH_F,HUM_CH_G, pwmN);
    }
    if (rowIdx == 3) { // RH% cluster lives on CA4(row3)
      setCh(outRow, CH_RH, pwmRH);
    }
  }

  // ---- TEMPERATURE ----
  int8_t tmpIdx = rowToTempDigit[rowIdx];
  if (tmpIdx >= 0 && tmpIdx < 4) {
    uint8_t v = tempDigits[tmpIdx];

    if (tmpIdx == 3) {
      applyMaskRawBuf(outRow, MASK_F, TMP_CH_A,TMP_CH_B,TMP_CH_C,TMP_CH_D,TMP_CH_E,TMP_CH_F,TMP_CH_G, pwmN);
    } else if (v != 0xFF && v <= 9) {
      applyDigitBuf(outRow, v, TMP_CH_A,TMP_CH_B,TMP_CH_C,TMP_CH_D,TMP_CH_E,TMP_CH_F,TMP_CH_G, pwmN);
    }

    // decimal dot (idx 2) and degree dot (idx 3) share CH_DEGREES
    if (tmpIdx == 2 || tmpIdx == 3) {
      setCh(outRow, CH_DEGREES, pwmN);
    }
  }
}

// Build all 4 rows into bb, then request a swap.
void rebuildBackBuffer() {
  for (uint8_t r = 0; r < 4; r++) {
    composeRow(r, bb[r]);
  }
  swapPending = true;
}

// =========================
// RTC -> clockDigits (12h + PM dot) + colon blink
// =========================
void updateTimeFromRTC_1Hz() {
  DateTime now = rtc.now();
  if (now.second() == lastSecond) return;

  lastSecond = now.second();
  colonOn = (now.second() % 2 == 0);

  int hours24 = now.hour();
  int minutes = now.minute();

  isPM = (hours24 >= 12);
  int hours12 = hours24 % 12;
  if (hours12 == 0) hours12 = 12;

  // HH:MM with leading blank for 1-9
  if (hours12 >= 10) clockDigits[0] = hours12 / 10;
  else               clockDigits[0] = 0xFF;

  clockDigits[1] = hours12 % 10;
  clockDigits[2] = minutes / 10;
  clockDigits[3] = minutes % 10;
}

// =========================
// Env -> digits (slow update)
// =========================
void updateEnvDigits_1Hz() {
  // Humidity 0-99
  if (isfinite(vHum)) {
    int rh = (int)lroundf(vHum);
    rh = constrain(rh, 0, 99);
    humDigits[0] = rh / 10;
    humDigits[1] = rh % 10;
  }

  // Temp °F to 1 decimal: " 74.6°F"
  if (isfinite(vTempC)) {
    float tF = c_to_f(vTempC);
    tF = constrain(tF, 0.0f, 99.9f);

    int t10 = (int)lroundf(tF * 10.0f); // 74.6 -> 746
    int ip  = t10 / 10;                 // 74
    int th  = t10 % 10;                 // 6

    int tens = ip / 10;                 // 7
    int ones = ip % 10;                 // 4

    tempDigits[0] = (tens > 0) ? tens : 0xFF;
    tempDigits[1] = ones;
    tempDigits[2] = th;
    tempDigits[3] = 0xFF;               // render 'F' based on index, not value
  }
}

// =========================
// Multiplex renderer (hardware-only)
// "Resync only" scheduler: never catches up.
// =========================
volatile uint8_t  activeRow = 0;
volatile uint32_t nextDeadlineUs = 0;

static inline void pushRowToTLC(const volatile uint16_t row[24]) {
  for (uint8_t ch = 0; ch < 24; ch++) tlc.setPWM(ch, row[ch]);
  tlc.write(); // OE is tied to LAT on your wiring => blanks during write
}

void multiplexTick() {
  uint32_t nowUs = micros();

  // First call init
  if (nextDeadlineUs == 0) {
    activeRow = 0;
    digitalWrite(rowPins[0], HIGH);
    pushRowToTLC(fb[0]);
    nextDeadlineUs = nowUs + ROW_DWELL_US;
    return;
  }

  // Not time yet
  if ((int32_t)(nowUs - nextDeadlineUs) < 0) return;

  // RESYNC ONLY (never catch up)
  nextDeadlineUs = nowUs + ROW_DWELL_US;

  // turn OFF current row
  digitalWrite(rowPins[activeRow], LOW);

  // next row
  uint8_t nextRow = (activeRow + 1) & 0x03;

  // latch next row PWM (single tlc.write per tick)
  pushRowToTLC(fb[nextRow]);

  // turn ON next row
  digitalWrite(rowPins[nextRow], HIGH);
  activeRow = nextRow;

  // swap only at frame boundary
  if (activeRow == 0 && swapPending) {
    noInterrupts();
    memcpy((void*)fb, (void*)bb, sizeof(bb));
    swapPending = false;
    interrupts();
  }
}

// =========================
// Serial brightness control
//  - "1600"     sets normal
//  - "rh 2000"  sets RH cluster
// =========================
void handleSerialBrightness() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.startsWith("rh")) {
    s.remove(0, 2);
    s.trim();
    long v = s.toInt();
    if (v >= 0 && v <= 4095) {
      SEG_BRIGHT_RH = (uint16_t)v;
      Serial.print("SEG_BRIGHT_RH=");
      Serial.println(SEG_BRIGHT_RH);
    } else {
      Serial.println("rh value must be 0..4095");
    }
    return;
  }

  long v = s.toInt();
  if (v >= 0 && v <= 4095) {
    SEG_BRIGHT_NORMAL = (uint16_t)v;
    Serial.print("SEG_BRIGHT_NORMAL=");
    Serial.println(SEG_BRIGHT_NORMAL);
  } else {
    Serial.println("value must be 0..4095 (or use: rh N)");
  }
}

// =========================
// setup / loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println("\n=== Clock + RTC + BSEC2 + TLC5947/TD62783 (buffered) ===");
  Serial.println("Dwell fixed at 5000us. Type 0..4095 to set normal brightness.");
  Serial.println("Type: rh 0..4095 to set RH% cluster brightness.");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else {
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting to compile time.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // BSEC2
  env.begin(BME_ADDR, Wire);
  loadBsecState();
  env.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  env.attachCallback(onBsecOutputs);

  // Row pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], LOW);
  }

  // TLC
  tlc.begin();

  // Initialize buffers to all-off
  memset((void*)fb, 0, sizeof(fb));
  memset((void*)bb, 0, sizeof(bb));
  tlc.write();

  // Build initial content
  updateTimeFromRTC_1Hz();
  updateEnvDigits_1Hz();
  rebuildBackBuffer();
  // Force immediate swap
  noInterrupts();
  memcpy((void*)fb, (void*)bb, sizeof(bb));
  swapPending = false;
  interrupts();
}

void loop() {
  // Real-time display
  multiplexTick();

  // BSEC2 state machine (can be called often; display is buffered so it won’t “pulse” it)
  env.run();

  // Serial brightness tuning
  handleSerialBrightness();

  // 1 Hz UI/model update
  static uint32_t lastUiMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastUiMs >= 1000) {
    lastUiMs = nowMs;

    updateTimeFromRTC_1Hz();
    updateEnvDigits_1Hz();
    rebuildBackBuffer();

    saveBsecStateIfReady(nowMs);
  }
}