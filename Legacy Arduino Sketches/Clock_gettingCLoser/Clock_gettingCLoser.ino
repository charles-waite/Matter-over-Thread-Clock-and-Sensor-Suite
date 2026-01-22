#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_TLC5947.h>
#include <bsec2.h>
#include <Preferences.h>

// ============================================================
// TLC5947 (your wiring)
// ============================================================
#define TLC_NUM   1
#define TLC_DATA  D10
#define TLC_CLK   D8
#define TLC_LAT   D9   // OE tied to LAT on your board
Adafruit_TLC5947 tlc = Adafruit_TLC5947(TLC_NUM, TLC_CLK, TLC_DATA, TLC_LAT);

// ============================================================
// TD62783 commons (your wiring)
// row index: 0=CA1, 1=CA2, 2=CA3, 3=CA4
// ============================================================
#define ROW1_PIN  D0   // CA1
#define ROW2_PIN  D1   // CA2
#define ROW3_PIN  D2   // CA3
#define ROW4_PIN  D3   // CA4
static const uint8_t ROW_PINS[4] = { ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };
struct SegMap { uint8_t a,b,c,d,e,f,g; };

// ============================================================
// I2C pins (from your earlier sketches)
// ============================================================
#define SDA_PIN   22
#define SCL_PIN   23
#define BME_ADDR  0x77

// ============================================================
// Row <-> digit mapping you verified
// Clock: row0=Digit2, row1=Digit4, row2=Digit1, row3=Digit3
// Humidity: Digit5 (tens) on CA4(row3), Digit6 (ones) on CA2(row1)
// Temperature: Digit7 on CA3(row2), Digit8 on CA1(row0), Digit9 on CA4(row3), Digit10 on CA2(row1)
// ============================================================
const int8_t rowToClockDigit[4] = { 1, 3, 0, 2 };
const int8_t rowToHumDigit[4]   = { -1, 1, -1, 0 };
const int8_t rowToTempDigit[4]  = { 1, 3, 0, 2 };

// Rows used for PM + colon clusters (same TLC channel, row-specific meaning)
const uint8_t rowForPmDot = 2;   // Clock Digit1 -> row2
const uint8_t rowForColon = 3;   // Clock Digit3 -> row3

// ============================================================
// TLC channels (confirmed from your sheet)
// ============================================================
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
const uint8_t CH_RH    =  3;   // %RH cluster (TLC channel 3)

// Temperature segments
const uint8_t TMP_CH_A = 17;
const uint8_t TMP_CH_B = 14;
const uint8_t TMP_CH_C = 12;
const uint8_t TMP_CH_D = 15;
const uint8_t TMP_CH_E = 13;
const uint8_t TMP_CH_F = 19;
const uint8_t TMP_CH_G = 18;
const uint8_t CH_DEGREES = 16; // degrees dot group

// Shared dot group (Colon on Digit3 row; AM/PM on Digit1 row)
const uint8_t CH_COLON = 11;

// ============================================================
// Brightness + timing
// ============================================================
static const uint32_t ROW_DWELL_US = 4650;   // fixed (your good value)

static volatile uint16_t SEG_BRIGHT = 1800;  // serial-adjustable
static volatile uint16_t DOT_BRIGHT = 4095;  // serial-adjustable
static volatile uint16_t RH_BRIGHT  = 4095;  // serial-adjustable

// ============================================================
// 7-seg bits/masks
// ============================================================
enum : uint8_t {
  SEG_A = 1 << 0,
  SEG_B = 1 << 1,
  SEG_C = 1 << 2,
  SEG_D = 1 << 3,
  SEG_E = 1 << 4,
  SEG_F = 1 << 5,
  SEG_G = 1 << 6,
};

static inline uint8_t segDigit(uint8_t d) {
  switch (d) {
    case 0: return SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F;
    case 1: return SEG_B|SEG_C;
    case 2: return SEG_A|SEG_B|SEG_D|SEG_E|SEG_G;
    case 3: return SEG_A|SEG_B|SEG_C|SEG_D|SEG_G;
    case 4: return SEG_B|SEG_C|SEG_F|SEG_G;
    case 5: return SEG_A|SEG_C|SEG_D|SEG_F|SEG_G;
    case 6: return SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G;
    case 7: return SEG_A|SEG_B|SEG_C;
    case 8: return SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G;
    case 9: return SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G;
    default: return 0;
  }
}
static inline uint8_t segCharF() { return SEG_A|SEG_E|SEG_F|SEG_G; }

// ============================================================
// RTC
// ============================================================
RTC_DS3231 rtc;

// ============================================================
// BSEC2 (BME680)
// ============================================================
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

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

void loadBsecState() {
  if (!prefs.begin(NVS_NS, true)) return;
  size_t n = prefs.getBytesLength(NVS_KEY);
  if (n == BSEC_STATE_SIZE) {
    uint8_t buf[BSEC_STATE_SIZE];
    prefs.getBytes(NVS_KEY, buf, BSEC_STATE_SIZE);
    env.setState(buf);
    Serial.println("[BSEC2] state restored");
  }
  prefs.end();
}

void saveBsecStatePeriodic(uint32_t nowMs) {
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

// ============================================================
// Display content state (digits)
// ============================================================
uint8_t clockDigits[4] = {0xFF, 0, 0, 0}; // Digit1..Digit4 (0xFF blank)
bool pm = false;
bool showHourTens = false;

uint8_t humTens = 0xFF, humOnes = 0xFF;
uint8_t tmpTens = 0xFF, tmpOnes = 0xFF;

// ============================================================
// Frame buffer: 4 rows × 24 channels PWM
// Multiplex loop only pushes these values.
// ============================================================
static uint16_t rowFrame[4][24];

// ============================================================
// Row helpers (NO global blank during refresh)
// ============================================================
static inline void rowsOff_setupOnly() {
  for (int i = 0; i < 4; i++) digitalWrite(ROW_PINS[i], LOW);
}
static inline void rowOn(uint8_t r)  { digitalWrite(ROW_PINS[r], HIGH); }
static inline void rowOff(uint8_t r) { digitalWrite(ROW_PINS[r], LOW); }

// ============================================================
// Apply a 7-seg mask into a given system map (writes into rowFrame[][])
// ============================================================

const SegMap MAP_CLK = { CLK_CH_A, CLK_CH_B, CLK_CH_C, CLK_CH_D, CLK_CH_E, CLK_CH_F, CLK_CH_G };
const SegMap MAP_HUM = { HUM_CH_A, HUM_CH_B, HUM_CH_C, HUM_CH_D, HUM_CH_E, HUM_CH_F, HUM_CH_G };
const SegMap MAP_TMP = { TMP_CH_A, TMP_CH_B, TMP_CH_C, TMP_CH_D, TMP_CH_E, TMP_CH_F, TMP_CH_G };

static inline void applySegMaskToFrame(uint8_t row, const SegMap &m, uint8_t mask, uint16_t pwm) {
  rowFrame[row][m.a] = (mask & SEG_A) ? pwm : 0;
  rowFrame[row][m.b] = (mask & SEG_B) ? pwm : 0;
  rowFrame[row][m.c] = (mask & SEG_C) ? pwm : 0;
  rowFrame[row][m.d] = (mask & SEG_D) ? pwm : 0;
  rowFrame[row][m.e] = (mask & SEG_E) ? pwm : 0;
  rowFrame[row][m.f] = (mask & SEG_F) ? pwm : 0;
  rowFrame[row][m.g] = (mask & SEG_G) ? pwm : 0;
}

// ============================================================
// Build frames for all rows (called ~1 Hz or when brightness changes)
// ============================================================
void rebuildAllRowFrames() {
  // clear all frames
  for (uint8_t r = 0; r < 4; r++) {
    for (uint8_t ch = 0; ch < 24; ch++) rowFrame[r][ch] = 0;
  }

  const uint16_t segPwm = SEG_BRIGHT;
  const uint16_t dotPwm = DOT_BRIGHT;
  const uint16_t rhPwm  = RH_BRIGHT;

  for (uint8_t row = 0; row < 4; row++) {
    // ---- CLOCK ----
    {
      int8_t d = rowToClockDigit[row];
      if (d >= 0 && d < 4) {
        if (d == 0 && !showHourTens) {
          // blank leading hour tens
        } else {
          applySegMaskToFrame(row, MAP_CLK, segDigit(clockDigits[d]), segPwm);
        }
      }

      // Colon static ON (row-specific)
      if (row == rowForColon) {
        rowFrame[row][CH_COLON] = dotPwm;
      }

      // PM dot on its row only
      if (row == rowForPmDot && pm) {
        rowFrame[row][CH_COLON] = dotPwm;
      }
    }

    // ---- HUMIDITY ----
    {
      int8_t hd = rowToHumDigit[row]; // 0=tens, 1=ones, -1 none
      if (hd == 0 && humTens <= 9) applySegMaskToFrame(row, MAP_HUM, segDigit(humTens), segPwm);
      if (hd == 1 && humOnes <= 9) applySegMaskToFrame(row, MAP_HUM, segDigit(humOnes), segPwm);

      // RH% cluster should be CA4 (row index 3) + TLC channel 3
      if (row == 3) rowFrame[row][CH_RH] = rhPwm;
    }

    // ---- TEMPERATURE ----
    {
      int8_t td = rowToTempDigit[row]; // 0..3 positions across rows
      if (td == 0 && tmpTens <= 9) applySegMaskToFrame(row, MAP_TMP, segDigit(tmpTens), segPwm);
      if (td == 1 && tmpOnes <= 9) applySegMaskToFrame(row, MAP_TMP, segDigit(tmpOnes), segPwm);

      // CH_DEGREES on td==2 row (per your earlier simplification)
      if (td == 2) rowFrame[row][CH_DEGREES] = dotPwm;

      // 'F' on td==3 row
      if (td == 3) applySegMaskToFrame(row, MAP_TMP, segCharF(), segPwm);
    }
  }
}

// ============================================================
// Update digits from RTC + sensor values (called ~1 Hz)
// ============================================================
void updateDigitsFromRTC() {
  DateTime now = rtc.now();
  uint8_t hh24 = now.hour();
  uint8_t mm   = now.minute();

  pm = (hh24 >= 12);

  uint8_t hh12 = hh24 % 12;
  if (hh12 == 0) hh12 = 12;

  showHourTens = (hh12 >= 10);

  clockDigits[0] = showHourTens ? (hh12 / 10) : 0xFF; // Digit1
  clockDigits[1] = hh12 % 10;                         // Digit2
  clockDigits[2] = mm / 10;                           // Digit3
  clockDigits[3] = mm % 10;                           // Digit4
}

void updateDigitsFromSensors() {
  // Humidity 0..99
  if (isfinite(vHum)) {
    int rh = (int)lroundf(vHum);
    rh = constrain(rh, 0, 99);
    humTens = (uint8_t)(rh / 10);
    humOnes = (uint8_t)(rh % 10);
  } else {
    humTens = humOnes = 0xFF;
  }

  // Temperature in F, integer (simple + stable)
  if (isfinite(vTempC)) {
    float tf = c_to_f(vTempC);
    int t = (int)lroundf(tf);
    t = constrain(t, 0, 99);
    tmpTens = (t >= 10) ? (uint8_t)(t / 10) : 0xFF; // blank leading 0
    tmpOnes = (uint8_t)(t % 10);
  } else {
    tmpTens = tmpOnes = 0xFF;
  }
}

// ============================================================
// Multiplex engine (tight + deterministic)
// - turn off previous row only
// - push prebuilt frame for next row
// - tlc.write() while dark (row off), then enable next row
// ============================================================
static uint8_t activeRow = 0;
static uint32_t lastRowUs = 0;

static inline void pushRowFrameToTLC(uint8_t row) {
  // Write all 24 channels from buffer (no recompute here)
  for (uint8_t ch = 0; ch < 24; ch++) {
    tlc.setPWM(ch, rowFrame[row][ch]);
  }
  tlc.write();
}

void serviceMultiplex() {
  uint32_t nowUs = micros();
  if (lastRowUs == 0) {
    lastRowUs = nowUs;
    // Prime row0
    pushRowFrameToTLC(activeRow);
    rowOn(activeRow);
    return;
  }

  if ((uint32_t)(nowUs - lastRowUs) < ROW_DWELL_US) return;
  lastRowUs += ROW_DWELL_US;

  uint8_t nextRow = (activeRow + 1) & 0x03;

  // Critical ordering for your “no-pulse” behavior:
  // 1) old row OFF
  rowOff(activeRow);

  // 2) write new TLC frame while dark
  pushRowFrameToTLC(nextRow);

  // 3) next row ON
  rowOn(nextRow);

  activeRow = nextRow;
}

// ============================================================
// Serial commands for brightness
// - "1800"        -> SEG_BRIGHT
// - "rh 4095"     -> RH_BRIGHT
// - "dot 4095"    -> DOT_BRIGHT
// ============================================================
void handleSerialBrightness() {
  static char line[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line[idx] = 0;
      idx = 0;

      // trim leading spaces
      char *p = line;
      while (*p == ' ' || *p == '\t') p++;

      if (strncmp(p, "rh", 2) == 0) {
        uint16_t v = (uint16_t)atoi(p + 2);
        if (v <= 4095) {
          RH_BRIGHT = v;
          Serial.print("RH_BRIGHT="); Serial.println(RH_BRIGHT);
          rebuildAllRowFrames();
        }
        return;
      }

      if (strncmp(p, "dot", 3) == 0) {
        uint16_t v = (uint16_t)atoi(p + 3);
        if (v <= 4095) {
          DOT_BRIGHT = v;
          Serial.print("DOT_BRIGHT="); Serial.println(DOT_BRIGHT);
          rebuildAllRowFrames();
        }
        return;
      }

      // default: number sets SEG_BRIGHT
      long v = atol(p);
      if (v >= 0 && v <= 4095) {
        SEG_BRIGHT = (uint16_t)v;
        Serial.print("SEG_BRIGHT="); Serial.println(SEG_BRIGHT);
        rebuildAllRowFrames();
      } else {
        Serial.println("Commands: <0..4095>,  rh <0..4095>,  dot <0..4095>");
      }
      return;
    }

    if (idx < sizeof(line) - 1) line[idx++] = c;
  }
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Clock + BSEC2 + TLC5947/TD multiplex (decoupled) ===");
  Serial.println("Commands: <0..4095> sets SEG_BRIGHT, 'rh N', 'dot N'");

  // Rows
  for (int i = 0; i < 4; i++) {
    pinMode(ROW_PINS[i], OUTPUT);
    digitalWrite(ROW_PINS[i], LOW);
  }

  // TLC
  tlc.begin();
  tlc.write();

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else if (rtc.lostPower()) {
    Serial.println("RTC lost power; setting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // BSEC2
  env.begin(BME_ADDR, Wire);
  loadBsecState();
  env.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  env.attachCallback(onBsecOutputs);

  // Initial digit build + frames
  updateDigitsFromRTC();
  updateDigitsFromSensors();
  rebuildAllRowFrames();

  // Prime multiplex
  rowsOff_setupOnly();
  activeRow = 0;
  lastRowUs = 0;
}

void loop() {
  // Keep BSEC ticking (non-blocking)
  env.run();

  // Update “content” at ~1 Hz (decoupled from refresh)
  static uint32_t lastContentMs = 0;
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastContentMs) >= 2000) {
    lastContentMs += 1000;
    
    updateDigitsFromRTC();
    updateDigitsFromSensors();
    rebuildAllRowFrames();

    //saveBsecStatePeriodic(nowMs);
  }

  // High-frequency refresh
  serviceMultiplex();

  // Serial tuning
//  handleSerialBrightness();
}