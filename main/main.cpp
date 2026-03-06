#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <time.h>
// For settimeofday
#include <sys/time.h>
// -------------------- Time Sync Kill Switch --------------------
// Set to 1 to completely disable Matter/network time sync.
// When disabled, the external DS3231 remains the only authority.
#if !defined(DISABLE_TIME_SYNC)
#define DISABLE_TIME_SYNC 0
#endif

// Compile-time default for verbose time-sync soak logs (`logtime on|off` can
// override at runtime).
#if !defined(LOGTIME_DEFAULT)
#define LOGTIME_DEFAULT 0
#endif

// Compile-time default for per-section heap diagnostics (`logheap on|off` can
// override at runtime).
#if !defined(LOGHEAP_DEFAULT)
#define LOGHEAP_DEFAULT 0
#endif

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_netif_sntp.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "soc/soc_caps.h"
#include "time_sync_manager.h"
#if __has_include("driver/usb_serial_jtag.h")
#include "driver/usb_serial_jtag.h"
#define ESP_CLOCK_HAS_USB_SERIAL_JTAG 1
#else
#define ESP_CLOCK_HAS_USB_SERIAL_JTAG 0
#endif
#include "nvs.h"
#include "nvs_flash.h"

#include <bsec2.h>

#if __has_include("esp_openthread.h")
#include "esp_openthread.h"
#if __has_include("esp_openthread_netif_glue.h")
#include "esp_openthread_netif_glue.h"
#endif
#include <openthread/link.h>
#include <openthread/thread.h>
#if __has_include(<openthread/thread_ftd.h>)
#include <openthread/thread_ftd.h>
#endif
#define ESP_CLOCK_HAS_OPENTHREAD 1
#else
#define ESP_CLOCK_HAS_OPENTHREAD 0
#endif

#if __has_include("esp_matter.h")
#include "esp_matter.h"
#include <app/server/Server.h>
#include <app/util/attribute-table.h>
#include <app-common/zap-generated/attribute-type.h>
#include <platform/PlatformManager.h>
#if __has_include("platform/ESP32/OpenthreadLauncher.h")
#include "platform/ESP32/OpenthreadLauncher.h"
#define ESP_CLOCK_HAS_OT_LAUNCHER 1
#else
#define ESP_CLOCK_HAS_OT_LAUNCHER 0
#endif
#define ESP_CLOCK_HAS_MATTER 1
#else
#define ESP_CLOCK_HAS_MATTER 0
#define ESP_CLOCK_HAS_OT_LAUNCHER 0
#endif

// -------------------- Logging --------------------
static const char* TAG = "esp_clock";

// -------------------- Board + I2C --------------------
static constexpr gpio_num_t SDA_PIN = GPIO_NUM_22; // D4
static constexpr gpio_num_t SCL_PIN = GPIO_NUM_23; // D5
static constexpr uint32_t I2C_HZ = 400000;

// -------------------- Sensors --------------------
static constexpr uint8_t BME_ADDR = 0x77;
static constexpr float TEMP_OFFSET_C = 4.1f;
static constexpr uint8_t DS3231_ADDR = 0x68;
static constexpr uint8_t VEML7700_ADDR = 0x10;
static constexpr const char* LOCAL_TZ = "PST8PDT,M3.2.0/2,M11.1.0/2";
static constexpr uint32_t RTC_SYNC_INTERVAL_MS = 7UL * 24UL * 60UL * 60UL * 1000UL;
static constexpr time_t RTC_VALID_EPOCH = 1700000000; // 2023-11-14
// Set to false if RTC stores local time instead of UTC.
static constexpr bool RTC_STORES_UTC = true;
static constexpr time_t RTC_MAX_DRIFT_SEC = 2;
static constexpr uint32_t RTC_SYNC_GRACE_MS = 60UL * 1000UL;
static constexpr const char* NTP_DEFAULT_SERVER_IPV4 = "192.168.1.29";
static constexpr const char* NTP_DEFAULT_SERVER_IPV6 = "fd7f:4975:d3c5:4f0a:bc3e:5442:1e1a:1911";
static constexpr uint16_t NTP_TEST_PORT = 123;
static constexpr uint32_t NTP_TEST_TIMEOUT_MS = 20000;
static constexpr time_t NTP_TEST_DRIFT_THRESHOLD_SEC = RTC_MAX_DRIFT_SEC;
static constexpr int64_t RTC_WRITE_ALIGN_WINDOW_US = 20000;     // 20 ms after second rollover
static constexpr uint32_t RTC_WRITE_ALIGN_TIMEOUT_MS = 1500;    // wait up to 1.5 s for alignment
static constexpr char DIAG_NS[] = "diag";
static constexpr char DIAG_KEY_BOOT_COUNT[] = "boot_count";
static constexpr char DIAG_KEY_LAST_RESET[] = "last_reset";
static constexpr char DIAG_KEY_LAST_RESET_EPOCH[] = "last_reset_epoch";
static constexpr char DIAG_KEY_RESET_HISTORY[] = "reset_hist";
static constexpr uint32_t DIAG_RESET_HISTORY_MAX = 20;
static constexpr uint32_t DIAG_RESET_HISTORY_LEGACY_MAX = 5;


// -------------------- Display HW --------------------
// TBD62783APG 4-channel common-anode driver pins (active HIGH).
static constexpr gpio_num_t TBD_PIN_CA1 = GPIO_NUM_0;   // D0
static constexpr gpio_num_t TBD_PIN_CA2 = GPIO_NUM_1;   // D1
static constexpr gpio_num_t TBD_PIN_CA3 = GPIO_NUM_2;   // D2
static constexpr gpio_num_t TBD_PIN_CA4 = GPIO_NUM_21;  // D3

// TLC5947 SPI pins (MOSI/SCLK + latch). OE is tied to latch.
static constexpr gpio_num_t TLC_SPI_MOSI = GPIO_NUM_18;  // D10
static constexpr gpio_num_t TLC_SPI_SCLK = GPIO_NUM_19;  // D8
static constexpr gpio_num_t TLC_SPI_LATCH = GPIO_NUM_20; // D9
static constexpr int32_t TLC_SPI_HZ = 1000000;
static constexpr uint16_t TLC_MAX = 4095;
static constexpr uint16_t TLC_ON = 1600;
static constexpr uint16_t RH_ON = 4095;
static constexpr uint8_t RH_CHANNEL = 3;
static constexpr uint16_t TLC_OFF = 0;

static constexpr uint32_t DISPLAY_PAGE_PERIOD_US = 2500; // 2.5ms/page -> 100 Hz frame
static constexpr bool DISPLAY_TEST_MODE = false;
static constexpr bool DISPLAY_RH_TEST_MODE = false;

// -------------------- BSEC2 --------------------
static Bsec2 env;
static constexpr char NVS_NS[] = "bsec2";
static constexpr char NVS_KEY[] = "state";
static constexpr size_t BSEC_STATE_SIZE = 512;

volatile float vTempC = NAN;
volatile float vHum = NAN;
volatile float vIAQ = NAN;
volatile float vCO2eq = NAN;
volatile uint8_t vIAQacc = 0;
static volatile bool gBrightnessOverride = false;
static TaskHandle_t gRtcSyncTask = nullptr;
static SemaphoreHandle_t gRtcSyncMutex = nullptr;
static volatile bool gThreadAttached = false;
static volatile bool gCommissioned = false;
static volatile bool gRtcInitialSyncDone = false;
static volatile bool gRtcInitialSyncDue = false;
static volatile bool gNetworkSyncTxnActive = false;
static volatile uint32_t gRtcSyncIntervalMs = RTC_SYNC_INTERVAL_MS;
static portMUX_TYPE gTimeDiagMux = portMUX_INITIALIZER_UNLOCKED;
static int64_t gLastNtpCompareEpoch = 0;
static double gLastNtpCompareDriftSec = NAN;
// Indicates whether network time is currently considered valid based on recent
// NTP fetch results. Auto RTC writes are attempted only through the guarded
// NTP sync pipeline.
static volatile bool gNetTimeValid = false;
static char gNtpServerIpv6[64] = "fd7f:4975:d3c5:4f0a:bc3e:5442:1e1a:1911";
static char gNtpServerIpv4[64] = "192.168.1.29";
static uint32_t gBootCount = 0;
static esp_reset_reason_t gPrevResetReason = ESP_RST_UNKNOWN;
static esp_reset_reason_t gCurrentResetReason = ESP_RST_UNKNOWN;
static int64_t gPrevResetEpoch = 0;
static int64_t gCurrentResetEpoch = 0;

static bool rtc_read_epoch(time_t* out);
static bool read_system_time_us(int64_t* outUsec);
static bool read_rtc_epoch_sec(int64_t* outSec);
static void format_epoch_us_utc(int64_t epochSec, int64_t usec, char* out, size_t outLen);
static int local_utc_offset_minutes_for_epoch(time_t epoch);
static void format_utc_offset(int offsetMin, char* out, size_t outLen);
static bool is_network_sync_transaction_active();
static bool is_commissioned();
static void log_time_sync_state(const char* reason, bool force);
static int64_t days_from_civil(int32_t year, uint32_t mon, uint32_t day);
static time_t utc_time_to_epoch(uint16_t year, uint8_t mon, uint8_t day,
                                uint8_t hour, uint8_t min, uint8_t sec);

typedef struct {
  uint32_t bootCount;
  uint32_t resetReason;
  int64_t epoch;
} ResetEvent;

typedef struct {
  uint32_t count;
  uint32_t next;
  ResetEvent entries[DIAG_RESET_HISTORY_MAX];
} ResetHistory;

typedef struct {
  uint32_t count;
  uint32_t next;
  ResetEvent entries[DIAG_RESET_HISTORY_LEGACY_MAX];
} ResetHistoryLegacy;

static ResetHistory gResetHistory = {};

enum class HeapDiagBucket : uint8_t {
  TimeSyncPoll = 0,
  BsecRun,
  MatterUpdate,
  AlsUpdate,
  DisplayUpdate,
  BsecStateSave,
  SerialCmd,
  StatusLogBlock,
  Count
};

typedef struct {
  int32_t netBytes;
  uint32_t calls;
  uint32_t lossEvents;
  uint32_t gainEvents;
  uint32_t worstLoss;
} HeapDiagStats;

static const char* kHeapDiagBucketNames[(size_t)HeapDiagBucket::Count] = {
  "time_sync_poll",
  "bsec_run",
  "matter_update",
  "als_update",
  "display_update",
  "bsec_state_save",
  "serial_cmd",
  "status_log"
};
static HeapDiagStats gHeapDiag[(size_t)HeapDiagBucket::Count] = {};
static bool gHeapDiagEnabled = (LOGHEAP_DEFAULT != 0);
static bool gLogTimeEnabled = (LOGTIME_DEFAULT != 0);
static size_t gHeapDiagLastFree = 0;
static size_t gHeapDiagLastLargest = 0;
#if ESP_CLOCK_HAS_USB_SERIAL_JTAG
static bool gUsbSerialReady = false;
#endif

static void heap_diag_reset() {
  memset(gHeapDiag, 0, sizeof(gHeapDiag));
}

static void heap_diag_account(HeapDiagBucket bucket, size_t before, size_t after) {
  if (!gHeapDiagEnabled) return;
  size_t idx = (size_t)bucket;
  if (idx >= (size_t)HeapDiagBucket::Count) return;
  int32_t delta = (int32_t)after - (int32_t)before;
  HeapDiagStats* s = &gHeapDiag[idx];
  s->calls++;
  s->netBytes += delta;
  if (delta < 0) {
    uint32_t loss = (uint32_t)(-delta);
    s->lossEvents++;
    if (loss > s->worstLoss) s->worstLoss = loss;
  } else if (delta > 0) {
    s->gainEvents++;
  }
}

static void heap_diag_log_and_reset(uint32_t intervalMs) {
  if (!gHeapDiagEnabled) return;
  const size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  const size_t minimum = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  const int32_t freeDelta = (gHeapDiagLastFree > 0) ? ((int32_t)freeHeap - (int32_t)gHeapDiagLastFree) : 0;
  const int32_t largestDelta = (gHeapDiagLastLargest > 0) ? ((int32_t)largest - (int32_t)gHeapDiagLastLargest) : 0;
  ESP_LOGI(TAG,
           "HeapDiag: free=%uB (%+d) largest=%uB (%+d) min=%uB window=%lu ms",
           (unsigned)freeHeap, (int)freeDelta,
           (unsigned)largest, (int)largestDelta,
           (unsigned)minimum, (unsigned long)intervalMs);
  gHeapDiagLastFree = freeHeap;
  gHeapDiagLastLargest = largest;

  for (size_t i = 0; i < (size_t)HeapDiagBucket::Count; i++) {
    const HeapDiagStats* s = &gHeapDiag[i];
    if (s->calls == 0) continue;
    if (s->netBytes == 0 && s->lossEvents == 0 && s->gainEvents == 0) continue;
    ESP_LOGI(TAG,
             "HeapDiag[%s]: calls=%u net=%+dB loss_events=%u gain_events=%u worst_loss=%uB",
             kHeapDiagBucketNames[i],
             (unsigned)s->calls, (int)s->netBytes,
             (unsigned)s->lossEvents, (unsigned)s->gainEvents, (unsigned)s->worstLoss);
  }
  heap_diag_reset();
}

static const char* resetReasonToStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_EFUSE: return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    default: return "UNMAPPED";
  }
}

static void formatEpochUtc(int64_t epoch, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  if (epoch <= 0) {
    snprintf(out, outLen, "unknown");
    return;
  }
  time_t t = (time_t)epoch;
  struct tm tmUtc = {};
  gmtime_r(&t, &tmUtc);
  snprintf(out, outLen, "%04d-%02d-%02d %02d:%02d:%02d UTC",
           tmUtc.tm_year + 1900, tmUtc.tm_mon + 1, tmUtc.tm_mday,
           tmUtc.tm_hour, tmUtc.tm_min, tmUtc.tm_sec);
}

static void format_epoch_us_utc(int64_t epochSec, int64_t usec, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  if (epochSec <= 0 || usec < 0) {
    snprintf(out, outLen, "unknown");
    return;
  }
  char utc[32] = {};
  formatEpochUtc(epochSec, utc, sizeof(utc));
  snprintf(out, outLen, "%s.%06" PRId64, utc, usec);
}

static int local_utc_offset_minutes_for_epoch(time_t epoch) {
  struct tm tmLocal = {};
  if (!localtime_r(&epoch, &tmLocal)) return 0;
  char z[8] = {};
  if (strftime(z, sizeof(z), "%z", &tmLocal) == 0) return 0;
  if (z[0] != '+' && z[0] != '-') return 0;
  if (z[1] < '0' || z[1] > '9' || z[2] < '0' || z[2] > '9' ||
      z[3] < '0' || z[3] > '9' || z[4] < '0' || z[4] > '9') {
    return 0;
  }
  const int sign = (z[0] == '-') ? -1 : 1;
  const int hh = ((z[1] - '0') * 10) + (z[2] - '0');
  const int mm = ((z[3] - '0') * 10) + (z[4] - '0');
  return sign * ((hh * 60) + mm);
}

static void format_utc_offset(int offsetMin, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  const int sign = (offsetMin < 0) ? -1 : 1;
  const int absMin = abs(offsetMin);
  const int hh = absMin / 60;
  const int mm = absMin % 60;
  snprintf(out, outLen, "%c%02d:%02d", (sign < 0) ? '-' : '+', hh, mm);
}

static void reset_history_append(ResetHistory* hist, const ResetEvent* ev) {
  if (!hist || !ev) return;
  uint32_t idx = hist->next % DIAG_RESET_HISTORY_MAX;
  hist->entries[idx] = *ev;
  hist->next = (idx + 1) % DIAG_RESET_HISTORY_MAX;
  if (hist->count < DIAG_RESET_HISTORY_MAX) hist->count++;
}

static void reset_history_log(const ResetHistory* hist, const char* context) {
  if (!hist) return;
  ESP_LOGI(TAG, "Reset history (%s): count=%lu", context ? context : "n/a",
           (unsigned long)hist->count);
  if (hist->count == 0) {
    ESP_LOGI(TAG, "Reset history: empty");
    return;
  }
  uint32_t oldest = (hist->next + DIAG_RESET_HISTORY_MAX - hist->count) % DIAG_RESET_HISTORY_MAX;
  for (uint32_t i = 0; i < hist->count; i++) {
    uint32_t idx = (oldest + i) % DIAG_RESET_HISTORY_MAX;
    const ResetEvent& e = hist->entries[idx];
    char ts[32] = {};
    formatEpochUtc(e.epoch, ts, sizeof(ts));
    ESP_LOGI(TAG, "Reset[%lu]: boot_count=%lu reason=%s(%lu) time=%s",
             (unsigned long)i,
             (unsigned long)e.bootCount,
             resetReasonToStr((esp_reset_reason_t)e.resetReason), (unsigned long)e.resetReason,
             ts);
  }
}

static void recordBootDiagnostics() {
  gCurrentResetReason = esp_reset_reason();
  nvs_handle_t h = 0;
  esp_err_t err = nvs_open(DIAG_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Boot diag NVS open failed: %s", esp_err_to_name(err));
    return;
  }

  uint32_t bootCount = 0;
  uint32_t prevReason = (uint32_t)ESP_RST_UNKNOWN;
  int64_t prevEpoch = 0;
  ResetHistory hist = {};
  size_t histSize = 0;
  (void)nvs_get_u32(h, DIAG_KEY_BOOT_COUNT, &bootCount);
  (void)nvs_get_u32(h, DIAG_KEY_LAST_RESET, &prevReason);
  (void)nvs_get_i64(h, DIAG_KEY_LAST_RESET_EPOCH, &prevEpoch);
  esp_err_t histErr = nvs_get_blob(h, DIAG_KEY_RESET_HISTORY, nullptr, &histSize);
  if (histErr == ESP_OK && histSize == sizeof(ResetHistory)) {
    size_t readSize = sizeof(hist);
    histErr = nvs_get_blob(h, DIAG_KEY_RESET_HISTORY, &hist, &readSize);
    if (histErr != ESP_OK || readSize != sizeof(hist) ||
        hist.count > DIAG_RESET_HISTORY_MAX || hist.next >= DIAG_RESET_HISTORY_MAX) {
      memset(&hist, 0, sizeof(hist));
    }
  } else if (histErr == ESP_OK && histSize == sizeof(ResetHistoryLegacy)) {
    ResetHistoryLegacy legacy = {};
    size_t readSize = sizeof(legacy);
    histErr = nvs_get_blob(h, DIAG_KEY_RESET_HISTORY, &legacy, &readSize);
    if (histErr == ESP_OK && readSize == sizeof(legacy) &&
        legacy.count <= DIAG_RESET_HISTORY_LEGACY_MAX &&
        legacy.next < DIAG_RESET_HISTORY_LEGACY_MAX) {
      memset(&hist, 0, sizeof(hist));
      uint32_t count = legacy.count;
      if (count > DIAG_RESET_HISTORY_LEGACY_MAX) count = DIAG_RESET_HISTORY_LEGACY_MAX;
      uint32_t oldest = (legacy.next + DIAG_RESET_HISTORY_LEGACY_MAX - count) % DIAG_RESET_HISTORY_LEGACY_MAX;
      for (uint32_t i = 0; i < count; i++) {
        uint32_t idx = (oldest + i) % DIAG_RESET_HISTORY_LEGACY_MAX;
        reset_history_append(&hist, &legacy.entries[idx]);
      }
      ESP_LOGI(TAG, "Migrated reset history from %lu to %lu entries",
               (unsigned long)DIAG_RESET_HISTORY_LEGACY_MAX,
               (unsigned long)DIAG_RESET_HISTORY_MAX);
    } else {
      memset(&hist, 0, sizeof(hist));
    }
  } else {
    memset(&hist, 0, sizeof(hist));
  }

  gBootCount = bootCount + 1;
  gPrevResetReason = (esp_reset_reason_t)prevReason;
  gPrevResetEpoch = prevEpoch;
  time_t rtcEpoch = 0;
  if (rtc_read_epoch(&rtcEpoch)) {
    gCurrentResetEpoch = (int64_t)rtcEpoch;
  } else {
    gCurrentResetEpoch = 0;
  }

  (void)nvs_set_u32(h, DIAG_KEY_BOOT_COUNT, gBootCount);
  (void)nvs_set_u32(h, DIAG_KEY_LAST_RESET, (uint32_t)gCurrentResetReason);
  if (gCurrentResetEpoch > 0) {
    (void)nvs_set_i64(h, DIAG_KEY_LAST_RESET_EPOCH, gCurrentResetEpoch);
  }
  ResetEvent ev = {};
  ev.bootCount = gBootCount;
  ev.resetReason = (uint32_t)gCurrentResetReason;
  ev.epoch = gCurrentResetEpoch;
  reset_history_append(&hist, &ev);
  gResetHistory = hist;
  (void)nvs_set_blob(h, DIAG_KEY_RESET_HISTORY, &hist, sizeof(hist));
  (void)nvs_commit(h);
  nvs_close(h);

  char prevTs[32] = {};
  char currTs[32] = {};
  formatEpochUtc(gPrevResetEpoch, prevTs, sizeof(prevTs));
  formatEpochUtc(gCurrentResetEpoch, currTs, sizeof(currTs));
  ESP_LOGI(TAG, "Boot diag: boot_count=%lu prev_reset=%s(%u) prev_time=%s current_reset=%s(%u) current_time=%s",
           (unsigned long)gBootCount,
           resetReasonToStr(gPrevResetReason), (unsigned)gPrevResetReason,
           prevTs,
           resetReasonToStr(gCurrentResetReason), (unsigned)gCurrentResetReason,
           currTs);
  reset_history_log(&gResetHistory, "boot");
}

static bsec_virtual_sensor_t sensorList[] = {
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  BSEC_OUTPUT_IAQ,
  BSEC_OUTPUT_CO2_EQUIVALENT,
};

static uint32_t millis_ms() {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void delay_microseconds_safe(uint32_t us) {
  const uint32_t lag = 5000;
  uint64_t start = esp_timer_get_time();
  if (us > lag) {
    vTaskDelay(pdMS_TO_TICKS((us - lag) / 1000UL));
    while ((esp_timer_get_time() - start) < (us - lag)) {
      vTaskDelay(1);
    }
  }
  while ((esp_timer_get_time() - start) < us) {
    esp_rom_delay_us(10);
  }
}

static i2c_master_bus_handle_t i2cBus = nullptr;
static i2c_master_dev_handle_t bmeDev = nullptr;
static i2c_master_dev_handle_t rtcDev = nullptr;
static i2c_master_dev_handle_t vemlDev = nullptr;
static bool gHasBme680 = false;
static bool gHasRtc = false;
static bool gHasVeml7700 = false;

static bool i2c_probe_addr(uint8_t addr) {
  if (!i2cBus) return false;
  return i2c_master_probe(i2cBus, addr, 50) == ESP_OK;
}

static int8_t i2c_read_bytes(uint8_t reg, uint8_t* data, uint32_t len, void*) {
  if (!bmeDev) return -1;
  esp_err_t err = i2c_master_transmit_receive(bmeDev, &reg, 1, data, len, 50);
  return (err == ESP_OK) ? 0 : -1;
}

static int8_t i2c_write_bytes(uint8_t reg, const uint8_t* data, uint32_t len, void*) {
  if (!bmeDev) return -1;
  uint8_t buf[1 + 32];
  if (len > 32) return -1;
  buf[0] = reg;
  memcpy(buf + 1, data, len);
  esp_err_t err = i2c_master_transmit(bmeDev, buf, len + 1, 50);
  return (err == ESP_OK) ? 0 : -1;
}

static bool i2c_read_dev(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, uint32_t len) {
  if (!dev) return false;
  return i2c_master_transmit_receive(dev, &reg, 1, data, len, 50) == ESP_OK;
}

static bool i2c_write_dev(i2c_master_dev_handle_t dev, uint8_t reg, const uint8_t* data, uint32_t len) {
  if (!dev) return false;
  uint8_t buf[1 + 32];
  if (len > 32) return false;
  buf[0] = reg;
  memcpy(buf + 1, data, len);
  return i2c_master_transmit(dev, buf, len + 1, 50) == ESP_OK;
}

static void delay_us(uint32_t period, void*) {
  delay_microseconds_safe(period);
}

static void onBsecOutputs(const bme68xData, const bsecOutputs out, Bsec2) {
  for (uint8_t i = 0; i < out.nOutputs; i++) {
    const bsecData& o = out.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: vTempC = o.signal; break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY: vHum = o.signal; break;
      case BSEC_OUTPUT_IAQ:
        vIAQ = o.signal;
        vIAQacc = o.accuracy;
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT: vCO2eq = o.signal; break;
      default: break;
    }
  }
}

static void loadBsecState() {
  nvs_handle_t handle;
  if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) return;
  size_t n = 0;
  if (nvs_get_blob(handle, NVS_KEY, nullptr, &n) == ESP_OK && n == BSEC_STATE_SIZE) {
    uint8_t buf[BSEC_STATE_SIZE];
    if (nvs_get_blob(handle, NVS_KEY, buf, &n) == ESP_OK) {
      env.setState(buf);
    }
  }
  nvs_close(handle);
}

static void saveBsecStateIfReady(uint32_t nowMs) {
  static const uint32_t EVERY = 5UL * 60UL * 1000UL;
  static uint32_t last = 0;
  if (nowMs - last < EVERY) return;
  uint8_t st[BSEC_STATE_SIZE];
  if (env.getState(st) != true) return;
  nvs_handle_t handle;
  if (nvs_open(NVS_NS, NVS_READWRITE, &handle) == ESP_OK) {
    nvs_set_blob(handle, NVS_KEY, st, BSEC_STATE_SIZE);
    nvs_commit(handle);
    nvs_close(handle);
    last = nowMs;
  }
}

static time_t build_time_epoch() {
  const char* date = __DATE__;
  const char* time = __TIME__;
  char mon_str[4] = {};
  int day = 0;
  int year = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  sscanf(date, "%3s %d %d", mon_str, &day, &year);
  sscanf(time, "%d:%d:%d", &hour, &min, &sec);
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* p = strstr(months, mon_str);
  int mon = p ? ((int)(p - months) / 3) + 1 : 1;
  struct tm tm_local = {};
  tm_local.tm_year = year - 1900;
  tm_local.tm_mon = mon - 1;
  tm_local.tm_mday = day;
  tm_local.tm_hour = hour;
  tm_local.tm_min = min;
  tm_local.tm_sec = sec;
  return mktime(&tm_local);
}

static time_t min_valid_epoch() {
  time_t floor = RTC_VALID_EPOCH;
  time_t build = build_time_epoch();
  if (build > floor) floor = build;
  return floor;
}

static void reset_ntp_servers_to_default() {
  strlcpy(gNtpServerIpv6, NTP_DEFAULT_SERVER_IPV6, sizeof(gNtpServerIpv6));
  strlcpy(gNtpServerIpv4, NTP_DEFAULT_SERVER_IPV4, sizeof(gNtpServerIpv4));
}

static bool set_ntp_server(char* dst, size_t dstLen, const char* value) {
  if (!dst || dstLen == 0 || !value) return false;
  while (*value == ' ') value++;
  if (*value == '\0') return false;
  size_t n = strnlen(value, dstLen);
  if (n >= dstLen) return false;
  strlcpy(dst, value, dstLen);
  return true;
}

// -------------------- Thread / Matter --------------------
static void configureThreadRouterEligibility() {
#if ESP_CLOCK_HAS_OPENTHREAD && CONFIG_ENABLE_MATTER_OVER_THREAD && CONFIG_OPENTHREAD_ENABLED
  otInstance* instance = esp_openthread_get_instance();
  if (!instance) return;

  otLinkModeConfig mode = otThreadGetLinkMode(instance);
  mode.mRxOnWhenIdle = true;
  mode.mDeviceType = true;
  mode.mNetworkData = true;
  otThreadSetLinkMode(instance, mode);
#if OPENTHREAD_FTD
  otThreadSetRouterEligible(instance, true);
#endif
#endif
}

#if ESP_CLOCK_HAS_OT_LAUNCHER
static void init_openthread_nvs() {
  esp_err_t err = nvs_flash_init_partition("ot");
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "OT NVS init failed (%s), erasing", esp_err_to_name(err));
    ESP_ERROR_CHECK(nvs_flash_erase_partition("ot"));
    err = nvs_flash_init_partition("ot");
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OT NVS init failed: %s", esp_err_to_name(err));
    return;
  }

  nvs_handle_t handle;
  err = nvs_open_from_partition("ot", "openthread", NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    nvs_close(handle);
    ESP_LOGI(TAG, "OT NVS partition ready");
  } else {
    ESP_LOGE(TAG, "OT NVS open failed: %s", esp_err_to_name(err));
  }
}

static void init_openthread_platform_config() {
#if ESP_CLOCK_HAS_OPENTHREAD && CONFIG_OPENTHREAD_ENABLED
  esp_openthread_platform_config_t config = {};
#if SOC_IEEE802154_SUPPORTED
  config.radio_config.radio_mode = RADIO_MODE_NATIVE;
#else
  config.radio_config.radio_mode = RADIO_MODE_UART_RCP;
#endif
  // No OT CLI host interface; avoids UART init with unset config.
  config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;
  config.port_config.storage_partition_name = "ot";
  config.port_config.netif_queue_size = 10;
  config.port_config.task_queue_size = 10;
  esp_err_t err = set_openthread_platform_config(&config);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "OpenThread platform config set");
  } else {
    ESP_LOGE(TAG, "set_openthread_platform_config failed: %s", esp_err_to_name(err));
  }
#endif
}
#endif

#if ESP_CLOCK_HAS_MATTER
static uint16_t matterTempEndpoint = 0;
static uint16_t matterHumEndpoint = 0;
static uint16_t matterAirEndpoint = 0;
static bool matterCo2Enabled = false;

static esp_err_t matter_attribute_cb(esp_matter::attribute::callback_type_t, uint16_t, uint32_t,
                                     uint32_t, esp_matter_attr_val_t*, void*) {
  return ESP_OK;
}

static esp_err_t matter_identify_cb(esp_matter::identification::callback_type_t, uint16_t, uint8_t,
                                    uint8_t, void*) {
  return ESP_OK;
}

static void request_rtc_sync(const char* reason);
static void schedule_initial_rtc_sync(const char* reason);
static void ensure_thread_default_netif(const char* reason);
static bool is_commissioned();
static void onTimeSyncReady(int64_t utc) {
  if (gLogTimeEnabled) {
    char utcBuf[40] = {};
    formatEpochUtc(utc, utcBuf, sizeof(utcBuf));
    ESP_LOGI(TAG, "logtime: Matter time sync callback ready (epoch=%" PRId64 " utc=\"%s\")", utc, utcBuf);
  }
  log_time_sync_state("matter-time-ready", false);
}

static uint8_t co2_to_air_quality_enum(float co2ppm) {
  if (!isfinite(co2ppm)) return 0;
  if (co2ppm <= 800.0f) return 1;
  if (co2ppm <= 1000.0f) return 2;
  if (co2ppm <= 1500.0f) return 3;
  if (co2ppm <= 2000.0f) return 4;
  return 4;
}

static void matter_event_callback(const ChipDeviceEvent* event, intptr_t) {
  if (!event) return;
  switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
      ESP_LOGI(TAG, "Matter commissioning complete");
      gCommissioned = true;
      schedule_initial_rtc_sync("commissioning-complete");
      log_time_sync_state("matter-commissioning-complete", false);
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
      ESP_LOGI(TAG, "Matter commissioning session started");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
      ESP_LOGI(TAG, "Matter commissioning session stopped");
      break;
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
      if (event->ThreadConnectivityChange.Result == chip::DeviceLayer::kConnectivity_Established) {
        gThreadAttached = true;
        ensure_thread_default_netif("thread-connected");
        if (gCommissioned || is_commissioned()) {
          // Reattach/reconnect path: schedule one grace-delayed correction.
          schedule_initial_rtc_sync("thread-reconnected");
        }
        log_time_sync_state("matter-thread-established", false);
      } else {
        gThreadAttached = false;
        log_time_sync_state("matter-thread-lost", false);
      }
      break;
    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkEnabled:
      ensure_thread_default_netif("operational-network");
      break;
    default:
      break;
  }
}

#endif

static void matter_init() {
#if ESP_CLOCK_HAS_MATTER
  using namespace esp_matter;
  using namespace esp_matter::endpoint;
  using namespace chip::app::Clusters;

  node::config_t node_cfg;
  node_t* node = node::create(&node_cfg, matter_attribute_cb, matter_identify_cb);
  if (!node) {
    ESP_LOGE(TAG, "Matter node create failed");
    return;
  }

  temperature_sensor::config_t temp_cfg;
  endpoint_t* temp_ep = temperature_sensor::create(node, &temp_cfg, ENDPOINT_FLAG_NONE, nullptr);
  if (temp_ep) matterTempEndpoint = endpoint::get_id(temp_ep);

  humidity_sensor::config_t hum_cfg;
  endpoint_t* hum_ep = humidity_sensor::create(node, &hum_cfg, ENDPOINT_FLAG_NONE, nullptr);
  if (hum_ep) matterHumEndpoint = endpoint::get_id(hum_ep);

  endpoint_t* aq_ep = endpoint::create(node, ENDPOINT_FLAG_NONE, nullptr);
  if (aq_ep) {
    esp_matter::endpoint::add_device_type(aq_ep,
                                          ESP_MATTER_AIR_QUALITY_SENSOR_DEVICE_TYPE_ID,
                                          ESP_MATTER_AIR_QUALITY_SENSOR_DEVICE_TYPE_VERSION);
    cluster::descriptor::config_t aq_desc_cfg;
    cluster::identify::config_t aq_id_cfg;
    cluster::descriptor::create(aq_ep, &aq_desc_cfg, CLUSTER_FLAG_SERVER);
    cluster::identify::create(aq_ep, &aq_id_cfg, CLUSTER_FLAG_SERVER);

    cluster_t* aq_cluster = cluster::create(aq_ep, AirQuality::Id, CLUSTER_FLAG_SERVER);
    if (aq_cluster) {
      cluster::global::attribute::create_feature_map(aq_cluster, 0);
      cluster::global::attribute::create_cluster_revision(aq_cluster, 1);
      esp_matter::attribute::create(aq_cluster, AirQuality::Attributes::AirQuality::Id,
                                    ATTRIBUTE_FLAG_NULLABLE, esp_matter_enum8(0));
    }

    cluster::carbon_dioxide_concentration_measurement::config_t co2_cfg;
    co2_cfg.feature_flags = cluster::concentration_measurement::feature::numeric_measurement::get_id();
    cluster::carbon_dioxide_concentration_measurement::create(aq_ep, &co2_cfg, CLUSTER_FLAG_SERVER);

    matterAirEndpoint = endpoint::get_id(aq_ep);
    matterCo2Enabled = true;
  }

  esp_err_t err = esp_matter::start(matter_event_callback);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Matter start failed: %s", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "Matter started (temp=%u hum=%u air=%u)",
           matterTempEndpoint, matterHumEndpoint, matterAirEndpoint);
#else
  ESP_LOGW(TAG, "esp-matter not configured; Matter disabled");
#endif
}

static void matter_update() {
#if ESP_CLOCK_HAS_MATTER
  if (!esp_matter::is_started()) return;
  static uint32_t lastMs = 0;
  uint32_t now = millis_ms();
  if (now - lastMs < 15000) return;
  lastMs = now;

  using namespace chip::app::Clusters;

  if (matterTempEndpoint && isfinite(vTempC)) {
    int16_t val = (int16_t)lroundf(vTempC * 100.0f);
    esp_matter_attr_val_t attr = esp_matter_int16(val);
    esp_matter::attribute::update(matterTempEndpoint, TemperatureMeasurement::Id,
                                  TemperatureMeasurement::Attributes::MeasuredValue::Id, &attr);
  }
  if (matterHumEndpoint && isfinite(vHum)) {
    int16_t val = (int16_t)lroundf(vHum * 100.0f);
    esp_matter_attr_val_t attr = esp_matter_int16(val);
    esp_matter::attribute::update(matterHumEndpoint, RelativeHumidityMeasurement::Id,
                                  RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &attr);
  }
  if (matterAirEndpoint) {
    if (matterCo2Enabled && isfinite(vCO2eq)) {
      esp_matter_attr_val_t attr = esp_matter_nullable_float(vCO2eq);
      esp_matter::attribute::update(matterAirEndpoint, CarbonDioxideConcentrationMeasurement::Id,
                                    CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id, &attr);
    }
    if (isfinite(vCO2eq)) {
      uint8_t aq = co2_to_air_quality_enum(vCO2eq);
      esp_matter_attr_val_t attr = esp_matter_nullable_enum8(nullable<uint8_t>(aq));
      esp_matter::attribute::update(matterAirEndpoint, AirQuality::Id,
                                    AirQuality::Attributes::AirQuality::Id, &attr);
    }
  }
#endif
}

// -------------------- Display Mapping --------------------
enum class DigitId : uint8_t {
  Clock1 = 0,
  Clock2 = 1,
  Clock3 = 2,
  Clock4 = 3,
  Hum5 = 4,
  Hum6 = 5,
  Temp7 = 6,
  Temp8 = 7,
  Temp9 = 8,
  Temp10 = 9,
  Count
};

enum class SegmentId : uint8_t { A = 0, B, C, D, E, F, G };

struct SegmentMap {
  uint8_t page;
  uint8_t channel;
};

static constexpr SegmentMap kDigitMap[(uint8_t)DigitId::Count][7] = {
  // Clock1 (CA3)
  { {2, 8}, {2, 9}, {2, 20}, {2, 21}, {2, 23}, {2, 10}, {2, 22} },
  // Clock2 (CA1)
  { {0, 8}, {0, 9}, {0, 20}, {0, 21}, {0, 23}, {0, 10}, {0, 22} },
  // Clock3 (CA4)
  { {3, 8}, {3, 9}, {3, 20}, {3, 21}, {3, 23}, {3, 10}, {3, 22} },
  // Clock4 (CA2)
  { {1, 8}, {1, 9}, {1, 20}, {1, 21}, {1, 23}, {1, 10}, {1, 22} },
  // Hum5 (CA4)
  { {3, 2}, {3, 5}, {3, 7}, {3, 4}, {3, 6}, {3, 0}, {3, 1} },
  // Hum6 (CA2)
  { {1, 2}, {1, 5}, {1, 7}, {1, 4}, {1, 6}, {1, 0}, {1, 1} },
  // Temp7 (CA3)
  { {2, 17}, {2, 14}, {2, 12}, {2, 15}, {2, 13}, {2, 19}, {2, 18} },
  // Temp8 (CA1)
  { {0, 17}, {0, 14}, {0, 12}, {0, 15}, {0, 13}, {0, 19}, {0, 18} },
  // Temp9 (CA4)
  { {3, 17}, {3, 14}, {3, 12}, {3, 15}, {3, 13}, {3, 19}, {3, 18} },
  // Temp10 (CA2)
  { {1, 17}, {1, 14}, {1, 12}, {1, 15}, {1, 13}, {1, 19}, {1, 18} },
};

enum class IndicatorId : uint8_t {
  Colon,
  UpperDot4,
  AmPm,
  Degrees7,
  Degrees10,
  TempDecimal,
  HumidityPercent,
  Count
};

static constexpr SegmentMap kIndicatorMap[(uint8_t)IndicatorId::Count] = {
  {3, 11}, // Colon
  {1, 11}, // UpperDot4
  {2, 11}, // AM/PM
  {2, 16}, // DegreesDot7
  {1, 16}, // DegreesDot10
  {3, 16}, // Decimal
  {3, 3},  // %RH
};

static constexpr uint8_t kDigitSegments[10] = {
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

static constexpr uint8_t kSegmentF = 0b01110001; // A + E + F + G

// -------------------- Display Driver --------------------
class DisplayDriver {
 public:
  void init() {
    anodePins_[0] = TBD_PIN_CA1;
    anodePins_[1] = TBD_PIN_CA2;
    anodePins_[2] = TBD_PIN_CA3;
    anodePins_[3] = TBD_PIN_CA4;

    for (uint8_t i = 0; i < 4; i++) {
      gpio_reset_pin(anodePins_[i]);
      gpio_set_direction(anodePins_[i], GPIO_MODE_OUTPUT);
      gpio_set_level(anodePins_[i], 0);
    }

    gpio_reset_pin(TLC_SPI_LATCH);
    gpio_set_direction(TLC_SPI_LATCH, GPIO_MODE_OUTPUT);
    gpio_set_level(TLC_SPI_LATCH, 0);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = TLC_SPI_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = TLC_SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

#if defined(HSPI_HOST)
    spi_host_device_t host = HSPI_HOST;
    hostName_ = "HSPI";
#else
    spi_host_device_t host = SPI2_HOST;
    hostName_ = "SPI2";
#endif
    spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = TLC_SPI_HZ;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 1;
    spi_bus_add_device(host, &devcfg, &spi_);

    clearBuffers(front_);
    clearBuffers(back_);
    swapPending_ = false;

    xTaskCreatePinnedToCore(
        &DisplayDriver::taskTrampoline,
        "display_refresh",
        4096,
        this,
        configMAX_PRIORITIES - 1,
        &taskHandle_,
        0);

    esp_timer_create_args_t targs = {};
    targs.callback = &DisplayDriver::timerTrampoline;
    targs.arg = this;
    targs.dispatch_method = ESP_TIMER_TASK;
    targs.name = "disp_tick";
    esp_timer_create(&targs, &timer_);
    esp_timer_start_periodic(timer_, DISPLAY_PAGE_PERIOD_US);
    pagePeriodUs_ = DISPLAY_PAGE_PERIOD_US;
  }

  void setDigit(DigitId digit, uint8_t value) {
    if (value > 9) value = 0;
    setSegments(digit, kDigitSegments[value]);
  }

  void setBlank(DigitId digit) {
    setSegments(digit, 0);
  }

  void setSegments(DigitId digit, uint8_t mask) {
    for (uint8_t s = 0; s < 7; s++) {
      bool on = (mask >> s) & 0x1;
      setSegmentMapped(kDigitMap[(uint8_t)digit][s], on);
    }
  }

  void setIndicator(IndicatorId id, bool on) {
    const SegmentMap& map = kIndicatorMap[(uint8_t)id];
    if (id == IndicatorId::HumidityPercent) {
      back_[map.page][map.channel] = on ? RH_ON : TLC_OFF;
    } else {
      setSegmentMapped(map, on);
    }
  }

  void commit() {
    taskENTER_CRITICAL(&spinlock_);
    swapPending_ = true;
    taskEXIT_CRITICAL(&spinlock_);
  }

  void clearBackBuffer() {
    clearBuffers(back_);
  }

  void setChannelAllPages(uint8_t channel, bool on) {
    if (channel >= 24) return;
    uint16_t value = on ? RH_ON : TLC_OFF;
    for (uint8_t p = 0; p < 4; p++) back_[p][channel] = value;
  }

  void setRhTestMode(bool enabled) { rhTestMode_ = enabled; }
  void setRhTestPage(uint8_t page) { rhTestPage_ = page & 0x03; }

  void setPagePeriodUs(uint32_t us) {
    if (us < 200) us = 200;
    if (us > 5000) us = 5000;
    if (!timer_) return;
    esp_timer_stop(timer_);
    esp_timer_start_periodic(timer_, us);
    pagePeriodUs_ = us;
  }

  uint32_t pagePeriodUs() const { return pagePeriodUs_; }
  const char* hostName() const { return hostName_; }
  void setBrightness(uint16_t level) { brightness_ = level > TLC_MAX ? TLC_MAX : level; }
  void setScaleRh(bool enabled) { scaleRh_ = enabled; }

 private:
  static void taskTrampoline(void* arg) {
    static_cast<DisplayDriver*>(arg)->refreshTask();
  }

  static void timerTrampoline(void* arg) {
    auto* self = static_cast<DisplayDriver*>(arg);
    xTaskNotifyGive(self->taskHandle_);
  }

  void refreshTask() {
    while (true) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      if (page_ == 0) {
        taskENTER_CRITICAL(&spinlock_);
        if (swapPending_) {
          swapBuffers();
          swapPending_ = false;
        }
        taskEXIT_CRITICAL(&spinlock_);
      }

      allAnodesOff();
      if (rhTestMode_) {
        writeTlcPage(front_[0]);
        setAnode(rhTestPage_);
      } else {
        writeTlcPage(front_[page_]);
        setAnode(page_);
        page_ = (page_ + 1) & 0x03;
      }

      if (rhTestMode_) continue;
    }
  }

  void setSegmentMapped(const SegmentMap& map, bool on) {
    back_[map.page][map.channel] = on ? TLC_ON : TLC_OFF;
  }

  void swapBuffers() {
    for (uint8_t p = 0; p < 4; p++) {
      for (uint8_t c = 0; c < 24; c++) {
        uint16_t tmp = front_[p][c];
        front_[p][c] = back_[p][c];
        back_[p][c] = tmp;
      }
    }
  }

  void clearBuffers(uint16_t buf[4][24]) {
    for (uint8_t p = 0; p < 4; p++) {
      for (uint8_t c = 0; c < 24; c++) buf[p][c] = TLC_OFF;
    }
  }

  void allAnodesOff() {
    for (uint8_t i = 0; i < 4; i++) gpio_set_level(anodePins_[i], 0);
  }

  void setAnode(uint8_t page) {
    if (page < 4) gpio_set_level(anodePins_[page], 1);
  }

  void writeTlcPage(const uint16_t* channels) {
    uint8_t payload[36] = {};
    int bitPos = 0;
    for (int ch = 23; ch >= 0; ch--) {
      uint32_t v = channels[ch] & 0x0FFF;
      if (ch != RH_CHANNEL || scaleRh_) {
        v = (v * brightness_) / TLC_MAX;
      }
      for (int b = 11; b >= 0; b--) {
        if (v & (1u << b)) payload[bitPos / 8] |= (1u << (7 - (bitPos % 8)));
        bitPos++;
      }
    }

    spi_transaction_t t = {};
    t.length = sizeof(payload) * 8;
    t.tx_buffer = payload;

    gpio_set_level(TLC_SPI_LATCH, 0);
    spi_device_polling_transmit(spi_, &t);
    gpio_set_level(TLC_SPI_LATCH, 1);
    gpio_set_level(TLC_SPI_LATCH, 0);
  }

  gpio_num_t anodePins_[4] = {};
  uint16_t front_[4][24] = {};
  uint16_t back_[4][24] = {};
  volatile bool swapPending_ = false;
  portMUX_TYPE spinlock_ = portMUX_INITIALIZER_UNLOCKED;
  TaskHandle_t taskHandle_ = nullptr;
  esp_timer_handle_t timer_ = nullptr;
  uint8_t page_ = 0;
  volatile bool rhTestMode_ = false;
  volatile uint8_t rhTestPage_ = 0;
  uint32_t pagePeriodUs_ = 0;
  spi_device_handle_t spi_ = nullptr;
  const char* hostName_ = "";
  volatile uint16_t brightness_ = TLC_MAX;
  volatile bool scaleRh_ = true;
};

static DisplayDriver display;

// -------------------- Display Formatting --------------------
typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool pm;
} ClockTime;

typedef struct {
  uint16_t year;
  uint8_t mon;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} RtcDateTime;

static uint8_t from_bcd(uint8_t v) {
  return (uint8_t)((v >> 4) * 10 + (v & 0x0F));
}

static bool read_rtc_utc(RtcDateTime* out) {
  uint8_t buf[7] = {};
  if (!out || !i2c_read_dev(rtcDev, 0x00, buf, sizeof(buf))) return false;
  out->sec = from_bcd(buf[0] & 0x7F);
  out->min = from_bcd(buf[1]);
  uint8_t hour = buf[2];
  if (hour & 0x40) {
    bool pm = hour & 0x20;
    uint8_t h12 = from_bcd(hour & 0x1F);
    if (h12 == 12 && !pm) h12 = 0;
    out->hour = pm ? (uint8_t)(h12 + 12) : h12;
  } else {
    out->hour = from_bcd(hour & 0x3F);
  }
  out->day = from_bcd(buf[4]);
  out->mon = from_bcd(buf[5] & 0x1F);
  out->year = (uint16_t)(2000 + from_bcd(buf[6]));
  return true;
}

static time_t rtc_time_to_epoch(const RtcDateTime& dt) {
  if (RTC_STORES_UTC) {
    return utc_time_to_epoch(dt.year, dt.mon, dt.day, dt.hour, dt.min, dt.sec);
  }
  struct tm t = {};
  t.tm_year = dt.year - 1900;
  t.tm_mon = dt.mon - 1;
  t.tm_mday = dt.day;
  t.tm_hour = dt.hour;
  t.tm_min = dt.min;
  t.tm_sec = dt.sec;
  return mktime(&t);
}

static ClockTime readClockTime() {
  static uint8_t lastSec = 0xFF;
  static ClockTime cached = {12, 0, 0, false};
  RtcDateTime dt = {};
  if (!read_rtc_utc(&dt)) return cached;
  if (dt.sec == lastSec) return cached;
  lastSec = dt.sec;

  time_t epoch = rtc_time_to_epoch(dt);
  struct tm local_tm = {};
  localtime_r(&epoch, &local_tm);
  cached.hour = (uint8_t)local_tm.tm_hour;
  cached.minute = (uint8_t)local_tm.tm_min;
  cached.second = (uint8_t)local_tm.tm_sec;
  cached.pm = (cached.hour >= 12);
  return cached;
}

static uint8_t to_bcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static int64_t days_from_civil(int32_t year, uint32_t mon, uint32_t day) {
  year -= (mon <= 2);
  const int32_t era = (year >= 0 ? year : year - 399) / 400;
  const uint32_t yoe = (uint32_t)(year - era * 400);
  const uint32_t doy = (153 * (mon + (mon > 2 ? (uint32_t)-3 : 9)) + 2) / 5 + day - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static time_t utc_time_to_epoch(uint16_t year, uint8_t mon, uint8_t day,
                                uint8_t hour, uint8_t min, uint8_t sec) {
  const int64_t days = days_from_civil((int32_t)year, (uint32_t)mon, (uint32_t)day);
  const int64_t sod = (int64_t)hour * 3600LL + (int64_t)min * 60LL + (int64_t)sec;
  return (time_t)(days * 86400LL + sod);
}

static bool rtc_set_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec) {
  if (!rtcDev) return false;
  ESP_LOGW(TAG, "RTC WRITE: explicit %04u-%02u-%02u %02u:%02u:%02u",
         year, mon, day, hour, min, sec);

  if (year < 2000) year = 2000;
  uint8_t data[7] = {
      to_bcd(sec),
      to_bcd(min),
      to_bcd(hour),
      0x01, // day of week (placeholder)
      to_bcd(day),
      to_bcd(mon),
      to_bcd((uint8_t)(year - 2000)),
  };
  if (!i2c_write_dev(rtcDev, 0x00, data, sizeof(data))) return false;
  uint8_t status = 0;
  if (i2c_read_dev(rtcDev, 0x0F, &status, 1)) {
    status &= ~0x80;
    i2c_write_dev(rtcDev, 0x0F, &status, 1);
  }
  return true;
}

static bool rtc_set_time_from_epoch(time_t epoch) {
  if (!rtcDev) return false;
  ESP_LOGW(TAG, "RTC WRITE: from EPOCH %ld", (long)epoch);
  struct tm tm_out = {};
  if (RTC_STORES_UTC) gmtime_r(&epoch, &tm_out);
  else localtime_r(&epoch, &tm_out);
  return rtc_set_time((uint16_t)(tm_out.tm_year + 1900),
                      (uint8_t)(tm_out.tm_mon + 1),
                      (uint8_t)tm_out.tm_mday,
                      (uint8_t)tm_out.tm_hour,
                      (uint8_t)tm_out.tm_min,
                      (uint8_t)tm_out.tm_sec);
}

static bool rtc_adjust_seconds(int32_t delta_sec) {
  time_t rtc_epoch = 0;
  if (!rtc_read_epoch(&rtc_epoch)) return false;
  rtc_epoch += (time_t)delta_sec;
  return rtc_set_time_from_epoch(rtc_epoch);
}

static bool rtc_read_epoch(time_t* out) {
  if (!out) return false;
  RtcDateTime dt = {};
  if (!read_rtc_utc(&dt)) return false;
  *out = rtc_time_to_epoch(dt);
  return true;
}

static void set_system_time_from_epoch(time_t epoch) {
  struct timeval tv = {};
  tv.tv_sec = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

// Set the system (software) time from the external RTC. If reading the RTC
// fails, this function does nothing. The timezone should already be set
// appropriately via setenv("TZ", ...) before calling this function.
static void set_system_time_from_rtc() {
  time_t rtc_epoch = 0;
  if (!rtc_read_epoch(&rtc_epoch)) {
    return;
  }
  // Ignore return value; on failure the system time remains unchanged.
  set_system_time_from_epoch(rtc_epoch);
}

static bool is_network_sync_transaction_active() {
  return gNetworkSyncTxnActive;
}

static void set_last_ntp_compare(int64_t compareEpoch, double driftSec) {
  taskENTER_CRITICAL(&gTimeDiagMux);
  gLastNtpCompareEpoch = compareEpoch;
  gLastNtpCompareDriftSec = driftSec;
  taskEXIT_CRITICAL(&gTimeDiagMux);
}

static void get_last_ntp_compare(int64_t* compareEpoch, double* driftSec) {
  if (!compareEpoch || !driftSec) return;
  taskENTER_CRITICAL(&gTimeDiagMux);
  *compareEpoch = gLastNtpCompareEpoch;
  *driftSec = gLastNtpCompareDriftSec;
  taskEXIT_CRITICAL(&gTimeDiagMux);
}

static bool read_system_time_us(int64_t* outUsec) {
  if (!outUsec) return false;
  struct timeval tv = {};
  gettimeofday(&tv, nullptr);
  *outUsec = ((int64_t)tv.tv_sec * 1000000LL) + (int64_t)tv.tv_usec;
  return true;
}

static bool read_rtc_epoch_sec(int64_t* outSec) {
  if (!outSec) return false;
  time_t rtc = 0;
  if (!rtc_read_epoch(&rtc)) return false;
  *outSec = (int64_t)rtc;
  return true;
}

static bool read_matter_time_epoch_sec(int64_t* outEpochSec) {
  if (!outEpochSec) return false;
#if !DISABLE_TIME_SYNC
  return time_sync_now_utc(outEpochSec);
#else
  return false;
#endif
}

static void log_time_sync_state(const char* reason, bool force) {
  if (!force && !gLogTimeEnabled) return;

  int64_t rtcEpoch = 0;
  int64_t sysUsec = 0;
  int64_t matterEpoch = 0;
  bool rtcValid = read_rtc_epoch_sec(&rtcEpoch);
  bool sysValid = read_system_time_us(&sysUsec);
  bool matterValid = read_matter_time_epoch_sec(&matterEpoch);

  const int64_t sysEpoch = sysValid ? (sysUsec / 1000000LL) : 0;
  const time_t floorEpoch = min_valid_epoch();
  const time_t offsetEpochBase = sysValid ? (time_t)sysEpoch : time(nullptr);
  const int utcToLocalOffsetMin = local_utc_offset_minutes_for_epoch(offsetEpochBase);
  char utcOffset[16] = {};
  format_utc_offset(utcToLocalOffsetMin, utcOffset, sizeof(utcOffset));

  char floorUtc[40] = {};
  char rtcUtc[40] = {};
  char sysUtc[40] = {};
  char matterUtc[40] = {};
  formatEpochUtc((int64_t)floorEpoch, floorUtc, sizeof(floorUtc));
  formatEpochUtc(rtcValid ? rtcEpoch : 0, rtcUtc, sizeof(rtcUtc));
  formatEpochUtc(sysValid ? sysEpoch : 0, sysUtc, sizeof(sysUtc));
  formatEpochUtc(matterValid ? matterEpoch : 0, matterUtc, sizeof(matterUtc));

  double driftMatterRtc = NAN;
  double driftMatterSys = NAN;
  if (matterValid && rtcValid) driftMatterRtc = (double)matterEpoch - (double)rtcEpoch;
  if (matterValid && sysValid) driftMatterSys = (double)matterEpoch - (double)sysEpoch;

  ESP_LOGI(TAG,
           "TIME_SYNC_STATE reason=%s commissioned=%d thread=%d net_valid=%d tx_active=%d "
           "interval_s=%lu floor_epoch=%ld floor_utc=\"%s\" rtc_epoch=%" PRId64 " rtc_utc=\"%s\" "
           "sys_epoch=%" PRId64 " sys_utc=\"%s\" matter_epoch=%" PRId64 " matter_utc=\"%s\" "
           "utc_to_local_apply=\"%s\" utc_to_local_offset_min=%d drift_matter_rtc_sec=%.3f "
           "drift_matter_sys_sec=%.3f ntp_v6=%s ntp_v4=%s",
           reason ? reason : "n/a",
           (int)(gCommissioned || is_commissioned()),
           (int)gThreadAttached,
           (int)gNetTimeValid,
           (int)is_network_sync_transaction_active(),
           (unsigned long)(gRtcSyncIntervalMs / 1000UL),
           (long)floorEpoch,
           floorUtc,
           rtcValid ? rtcEpoch : 0,
           rtcUtc,
           sysValid ? sysEpoch : 0,
           sysUtc,
           matterValid ? matterEpoch : 0,
           matterUtc,
           utcOffset,
           utcToLocalOffsetMin,
           driftMatterRtc,
           driftMatterSys,
           gNtpServerIpv6,
           gNtpServerIpv4);
}

typedef struct {
  const char* sourceServer;
  int64_t ntpEpochSec;
  int64_t ntpUsec;
  int64_t sysAtFetchSec;
  int64_t sysAtFetchUsec;
  int64_t rtcBeforeSec;
  int64_t rtcAfterSec;
  double driftRtcBeforeSec;
  double driftRtcAfterSec;
  double driftSysBeforeMs;
  double driftSysAfterMs;
  double writePhaseMs;
  bool fetched;
  bool rtcBeforeValid;
  bool rtcAfterValid;
  bool writeAttempted;
  bool writeSucceeded;
} NetworkSyncMetrics;

static bool wait_and_write_rtc_aligned_from_system(const char* prefix, double* outWritePhaseMs) {
  if (!prefix) prefix = "RTC sync";
  if (outWritePhaseMs) *outWritePhaseMs = NAN;
  if (!is_network_sync_transaction_active()) {
    ESP_LOGW(TAG, "%s: RTC write blocked (network sync transaction inactive)", prefix);
    return false;
  }

  const int64_t deadlineUs = esp_timer_get_time() + ((int64_t)RTC_WRITE_ALIGN_TIMEOUT_MS * 1000LL);
  while (esp_timer_get_time() < deadlineUs) {
    struct timeval now = {};
    gettimeofday(&now, nullptr);
    if (now.tv_usec <= RTC_WRITE_ALIGN_WINDOW_US) {
      if (outWritePhaseMs) *outWritePhaseMs = (double)now.tv_usec / 1000.0;
      time_t writeEpoch = now.tv_sec;
      ESP_LOGI(TAG, "%s: RTC aligned write target epoch=%ld (phase_ms=%.3f)",
               prefix, (long)writeEpoch, outWritePhaseMs ? *outWritePhaseMs : -1.0);
      return rtc_set_time_from_epoch(writeEpoch);
    }
    if (now.tv_usec >= (1000000 - RTC_WRITE_ALIGN_WINDOW_US)) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      const int32_t waitUs = (1000000 - (int32_t)now.tv_usec) - (int32_t)RTC_WRITE_ALIGN_WINDOW_US;
      if (waitUs > 2000) {
        vTaskDelay(pdMS_TO_TICKS((waitUs / 1000) - 1));
      } else {
        esp_rom_delay_us((uint32_t)waitUs);
      }
    }
  }

  struct timeval fallback = {};
  gettimeofday(&fallback, nullptr);
  if (outWritePhaseMs) *outWritePhaseMs = (double)fallback.tv_usec / 1000.0;
  time_t writeEpoch = fallback.tv_sec + ((fallback.tv_usec >= 500000) ? 1 : 0);
  ESP_LOGW(TAG, "%s: RTC write alignment timeout; using fallback epoch=%ld (phase_ms=%.3f)",
           prefix, (long)writeEpoch, outWritePhaseMs ? *outWritePhaseMs : -1.0);
  return rtc_set_time_from_epoch(writeEpoch);
}

static bool ntp_fetch_one_shot(const char* server, uint32_t timeoutMs, struct timeval* outTv, int64_t* outSysUsecAtFetch, const char* prefix) {
  if (!server || !outTv) return false;
  if (!prefix) prefix = "NTP test";
  *outTv = {};
  if (outSysUsecAtFetch) *outSysUsecAtFetch = 0;

  ESP_LOGI(TAG, "%s: starting one-shot fetch server=%s port=%u timeout=%lu ms",
           prefix,
           server, (unsigned)NTP_TEST_PORT, (unsigned long)timeoutMs);
  ensure_thread_default_netif("ntp-test");

  esp_netif_sntp_deinit();
  esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(server);
  cfg.start = true;
  cfg.server_from_dhcp = false;
  cfg.renew_servers_after_new_IP = false;

  const uint64_t t0 = esp_timer_get_time();
  esp_err_t initErr = esp_netif_sntp_init(&cfg);
  if (initErr != ESP_OK) {
    ESP_LOGW(TAG, "%s: init failed server=%s err=%s", prefix, server, esp_err_to_name(initErr));
    esp_netif_sntp_deinit();
    return false;
  }

  esp_err_t waitErr = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(timeoutMs));
  if (waitErr != ESP_OK) {
    const uint64_t elapsed = (esp_timer_get_time() - t0) / 1000ULL;
    ESP_LOGW(TAG, "%s: fetch failed server=%s err=%s elapsed=%llu ms",
             prefix,
             server, esp_err_to_name(waitErr), (unsigned long long)elapsed);
    esp_netif_sntp_deinit();
    return false;
  }

  gettimeofday(outTv, nullptr);
  time_t floor = min_valid_epoch();
  if (outTv->tv_sec < floor) {
    ESP_LOGW(TAG, "%s: fetched epoch=%" PRId64 " below validity floor=%ld (server=%s)",
             prefix, (int64_t)outTv->tv_sec, (long)floor, server);
    esp_netif_sntp_deinit();
    return false;
  }
  if (outSysUsecAtFetch) {
    (void)read_system_time_us(outSysUsecAtFetch);
  }
  const uint64_t elapsed = (esp_timer_get_time() - t0) / 1000ULL;
  ESP_LOGI(TAG, "%s: fetch success server=%s epoch=%" PRId64 ".%06ld elapsed=%llu ms",
           prefix,
           server, (int64_t)outTv->tv_sec, (long)outTv->tv_usec, (unsigned long long)elapsed);
  esp_netif_sntp_deinit();
  return true;
}

static bool matter_fetch_one_shot(struct timeval* outTv, int64_t* outSysUsecAtFetch, const char* prefix) {
#if !DISABLE_TIME_SYNC
  if (!outTv) return false;
  if (!prefix) prefix = "RTC sync";
  int64_t utc = 0;
  if (!time_sync_now_utc(&utc)) {
    if (gLogTimeEnabled) {
      ESP_LOGI(TAG, "logtime: %s Matter time unavailable (time_sync_now_utc=false)", prefix);
    }
    return false;
  }
  time_t floor = min_valid_epoch();
  if (utc < (int64_t)floor) {
    if (gLogTimeEnabled) {
      ESP_LOGW(TAG, "logtime: %s Matter time epoch=%" PRId64 " below validity floor=%ld", prefix, utc, (long)floor);
    }
    return false;
  }
  outTv->tv_sec = (time_t)utc;
  outTv->tv_usec = 0;
  if (outSysUsecAtFetch) {
    (void)read_system_time_us(outSysUsecAtFetch);
  }
  if (gLogTimeEnabled) {
    ESP_LOGI(TAG, "logtime: %s Matter time fetch success epoch=%" PRId64, prefix, utc);
  }
  return true;
#else
  (void)outTv;
  (void)outSysUsecAtFetch;
  (void)prefix;
  return false;
#endif
}

static bool run_network_rtc_sync(bool allowWrite, const char* reason, const char* prefix, bool verboseLogs) {
  if (!reason) reason = "unspecified";
  if (!prefix) prefix = "RTC sync";
  if (gRtcSyncMutex) {
    if (xSemaphoreTake(gRtcSyncMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
      ESP_LOGW(TAG, "%s: busy, skipping (reason=%s)", prefix, reason);
      return false;
    }
  }

  bool ok = false;
  do {
    gNetworkSyncTxnActive = true;
    log_time_sync_state(reason, false);
    if (verboseLogs) {
      ESP_LOGI(TAG, "%s: stage 1/4 start mode=%s timeout=%lu ms reason=%s",
               prefix, allowWrite ? "compare+sync" : "compare-only",
               (unsigned long)NTP_TEST_TIMEOUT_MS, reason);
    } else {
      ESP_LOGI(TAG, "%s: start mode=%s reason=%s",
               prefix, allowWrite ? "compare+sync" : "compare-only", reason);
    }

    NetworkSyncMetrics m = {};
    m.sourceServer = "none";
    m.writePhaseMs = NAN;

    int64_t rtcBefore = 0;
    m.rtcBeforeValid = read_rtc_epoch_sec(&rtcBefore);
    if (m.rtcBeforeValid) {
      m.rtcBeforeSec = rtcBefore;
      if (verboseLogs) {
        char rtcUtc[40] = {};
        formatEpochUtc(m.rtcBeforeSec, rtcUtc, sizeof(rtcUtc));
        ESP_LOGI(TAG, "%s: stage 2/4 RTC precheck epoch=%" PRId64 " utc=%s",
                 prefix, m.rtcBeforeSec, rtcUtc);
      }
    } else {
      ESP_LOGW(TAG, "%s: RTC precheck failed (unreadable)", prefix);
    }

    struct timeval tv = {};
    int64_t sysUsecAtFetch = 0;
    if (verboseLogs) {
      ESP_LOGI(TAG, "%s: stage 3/4 attempt 1/3 source=Matter-Time-Sync", prefix);
    }
    if (matter_fetch_one_shot(&tv, &sysUsecAtFetch, prefix)) {
      if (gLogTimeEnabled) {
        ESP_LOGI(TAG, "logtime: %s selected source=matter-time-sync", prefix);
      }
      m.fetched = true;
      m.sourceServer = "matter-time-sync";
      m.ntpEpochSec = (int64_t)tv.tv_sec;
      m.ntpUsec = (int64_t)tv.tv_usec;
      m.sysAtFetchSec = sysUsecAtFetch / 1000000LL;
      m.sysAtFetchUsec = sysUsecAtFetch % 1000000LL;
    } else {
      if (gLogTimeEnabled) {
        ESP_LOGI(TAG, "logtime: %s Matter source unavailable; falling back to NTP servers", prefix);
      }
      const char* servers[] = {gNtpServerIpv6, gNtpServerIpv4};
      for (size_t i = 0; i < (sizeof(servers) / sizeof(servers[0])); ++i) {
        if (!servers[i] || servers[i][0] == '\0') continue;
        if (verboseLogs) {
          ESP_LOGI(TAG, "%s: stage 3/4 attempt %u/%u server=%s",
                   prefix, (unsigned)(i + 2), 3U, servers[i]);
        }
        if (ntp_fetch_one_shot(servers[i], NTP_TEST_TIMEOUT_MS, &tv, &sysUsecAtFetch, prefix)) {
          m.fetched = true;
          m.sourceServer = servers[i];
          m.ntpEpochSec = (int64_t)tv.tv_sec;
          m.ntpUsec = (int64_t)tv.tv_usec;
          m.sysAtFetchSec = sysUsecAtFetch / 1000000LL;
          m.sysAtFetchUsec = sysUsecAtFetch % 1000000LL;
          break;
        }
      }
    }

    if (!m.fetched) {
      gNetTimeValid = false;
      ESP_LOGW(TAG, "%s: FAILURE no server returned time (reason=%s)", prefix, reason);
      break;
    }
    gNetTimeValid = true;
    if (verboseLogs) {
      char ntpUtcUs[56] = {};
      format_epoch_us_utc(m.ntpEpochSec, m.ntpUsec, ntpUtcUs, sizeof(ntpUtcUs));
      ESP_LOGI(TAG, "%s: stage 4/4 NTP sample source=%s utc=%s",
               prefix, m.sourceServer, ntpUtcUs);
    }

    const double ntpContinuousSec = (double)m.ntpEpochSec + ((double)m.ntpUsec / 1000000.0);
    m.driftSysBeforeMs = ((((double)m.sysAtFetchSec - (double)m.ntpEpochSec) * 1000000.0) +
                          ((double)m.sysAtFetchUsec - (double)m.ntpUsec)) / 1000.0;
    int64_t rtcCompare = 0;
    m.rtcBeforeValid = read_rtc_epoch_sec(&rtcCompare);
    if (m.rtcBeforeValid) {
      m.rtcBeforeSec = rtcCompare;
      m.driftRtcBeforeSec = ntpContinuousSec - (double)m.rtcBeforeSec;
      set_last_ntp_compare(m.ntpEpochSec, m.driftRtcBeforeSec);
      ESP_LOGI(TAG,
               "SYNC_METRIC reason=%s phase=pre source=%s ntp_epoch=%" PRId64 " ntp_usec=%" PRId64
               " sys_epoch=%" PRId64 " sys_usec=%" PRId64 " rtc_epoch=%" PRId64
               " drift_rtc_sec=%.4f drift_sys_ms=%.3f write_phase_ms=nan",
               reason, m.sourceServer, m.ntpEpochSec, m.ntpUsec,
               m.sysAtFetchSec, m.sysAtFetchUsec, m.rtcBeforeSec,
               m.driftRtcBeforeSec, m.driftSysBeforeMs);
      if (fabs(m.driftRtcBeforeSec) < 1.0) {
        ESP_LOGI(TAG, "%s: RTC is within 1 second variance of NTP", prefix);
      } else {
        ESP_LOGI(TAG, "%s: RTC is %.1f seconds %s NTP",
                 prefix, fabs(m.driftRtcBeforeSec), (m.driftRtcBeforeSec >= 0.0) ? "behind" : "ahead of");
      }
    } else {
      set_last_ntp_compare(m.ntpEpochSec, NAN);
      ESP_LOGI(TAG,
               "SYNC_METRIC reason=%s phase=pre source=%s ntp_epoch=%" PRId64 " ntp_usec=%" PRId64
               " sys_epoch=%" PRId64 " sys_usec=%" PRId64 " rtc_epoch=unknown"
               " drift_rtc_sec=nan drift_sys_ms=%.3f write_phase_ms=nan",
               reason, m.sourceServer, m.ntpEpochSec, m.ntpUsec, m.sysAtFetchSec, m.sysAtFetchUsec,
               m.driftSysBeforeMs);
      ESP_LOGI(TAG, "%s: RTC comparison skipped (RTC unreadable)", prefix);
    }

    if (!allowWrite) {
      ESP_LOGI(TAG, "%s: SUCCESS compare-only complete (reason=%s)", prefix, reason);
      ok = true;
      break;
    }

    bool needsWrite = !m.rtcBeforeValid;
    if (m.rtcBeforeValid) {
      needsWrite = fabs(m.driftRtcBeforeSec) > (double)NTP_TEST_DRIFT_THRESHOLD_SEC;
    }
    if (!needsWrite) {
      ESP_LOGI(TAG, "%s: RTC drift %.3f sec <= %ld sec, no write needed (reason=%s)",
               prefix, fabs(m.driftRtcBeforeSec), (long)NTP_TEST_DRIFT_THRESHOLD_SEC, reason);
      m.writeAttempted = false;
    } else {
      ESP_LOGI(TAG, "%s: syncing RTC to NTP time... (reason=%s)", prefix, reason);
      m.writeAttempted = true;
      set_system_time_from_epoch((time_t)m.ntpEpochSec);
      m.writeSucceeded = wait_and_write_rtc_aligned_from_system(prefix, &m.writePhaseMs);
      ESP_LOGI(TAG, "%s: RTC sync %s", prefix, m.writeSucceeded ? "SUCCEEDED" : "FAILED");
      if (!m.writeSucceeded) {
        ESP_LOGW(TAG, "%s: FAILURE RTC write failed (reason=%s)", prefix, reason);
        break;
      }
    }

    int64_t sysAfterUs = 0;
    (void)read_system_time_us(&sysAfterUs);
    m.sysAtFetchSec = sysAfterUs / 1000000LL;
    m.sysAtFetchUsec = sysAfterUs % 1000000LL;
    const double sysContinuousAfterSec = (double)m.sysAtFetchSec + ((double)m.sysAtFetchUsec / 1000000.0);
    m.driftSysAfterMs = ((((double)m.sysAtFetchSec - (double)m.ntpEpochSec) * 1000000.0) +
                         ((double)m.sysAtFetchUsec - (double)m.ntpUsec)) / 1000.0;

    int64_t rtcAfter = 0;
    m.rtcAfterValid = read_rtc_epoch_sec(&rtcAfter);
    if (m.rtcAfterValid) {
      m.rtcAfterSec = rtcAfter;
      m.driftRtcAfterSec = sysContinuousAfterSec - (double)m.rtcAfterSec;
      ESP_LOGI(TAG,
               "SYNC_METRIC reason=%s phase=post source=%s ntp_epoch=%" PRId64 " ntp_usec=%" PRId64
               " sys_epoch=%" PRId64 " sys_usec=%" PRId64 " rtc_epoch=%" PRId64
               " drift_rtc_sec=%.4f drift_sys_ms=%.3f write_phase_ms=%.3f",
               reason, m.sourceServer, m.ntpEpochSec, m.ntpUsec,
               m.sysAtFetchSec, m.sysAtFetchUsec, m.rtcAfterSec,
               m.driftRtcAfterSec, m.driftSysAfterMs, m.writePhaseMs);
      if (fabs(m.driftRtcAfterSec) < 1.0) {
        ESP_LOGI(TAG, "%s: post-sync RTC is within 1 second variance of NTP", prefix);
      } else {
        ESP_LOGI(TAG, "%s: post-sync RTC is %.1f seconds %s NTP",
                 prefix, fabs(m.driftRtcAfterSec), (m.driftRtcAfterSec >= 0.0) ? "behind" : "ahead of");
      }
      ESP_LOGI(TAG, "%s: SUCCESS compare+sync complete (reason=%s)", prefix, reason);
      ok = true;
      break;
    }

    ESP_LOGW(TAG,
             "SYNC_METRIC reason=%s phase=post source=%s ntp_epoch=%" PRId64 " ntp_usec=%" PRId64
             " sys_epoch=%" PRId64 " sys_usec=%" PRId64 " rtc_epoch=unknown"
             " drift_rtc_sec=nan drift_sys_ms=%.3f write_phase_ms=%.3f",
             reason, m.sourceServer, m.ntpEpochSec, m.ntpUsec,
             m.sysAtFetchSec, m.sysAtFetchUsec, m.driftSysAfterMs, m.writePhaseMs);
    ESP_LOGW(TAG, "%s: RTC re-read failed after sync attempt (reason=%s)", prefix, reason);
  } while (false);

  gNetworkSyncTxnActive = false;
  log_time_sync_state(ok ? "sync-finish-ok" : "sync-finish-fail", false);
  if (gRtcSyncMutex) xSemaphoreGive(gRtcSyncMutex);
  return ok;
}

static bool run_ntp_rtc_test(bool applyRtcSync) {
  return run_network_rtc_sync(applyRtcSync,
                              applyRtcSync ? "serial-ntptest-sync" : "serial-ntptest",
                              "NTP test",
                              true);
}

static void request_rtc_sync(const char* reason) {
  if (!gRtcSyncTask) return;
  ESP_LOGI(TAG, "RTC sync requested (%s)", reason);
  xTaskNotifyGive(gRtcSyncTask);
}

static void schedule_initial_rtc_sync(const char* reason) {
  gRtcInitialSyncDue = true;
  gRtcInitialSyncDone = false;
  request_rtc_sync(reason);
}

static bool is_commissioned() {
#if ESP_CLOCK_HAS_MATTER
  return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
#else
  return false;
#endif
}

static void ensure_thread_default_netif(const char* reason) {
#if ESP_CLOCK_HAS_OT_LAUNCHER
  esp_netif_t* thread_netif = esp_openthread_get_netif();
  if (!thread_netif) {
    ESP_LOGW(TAG, "Thread netif unavailable (%s)", reason);
    return;
  }
  esp_netif_t* current = esp_netif_get_default_netif();
  if (current == thread_netif) {
    ESP_LOGI(TAG, "Thread netif already default (%s)", reason);
    return;
  }
  esp_err_t err = esp_netif_set_default_netif(thread_netif);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set default netif (%s): %s", reason, esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Thread netif set as default (%s)", reason);
  }
#else
  (void)reason;
#endif
}

static void rtc_time_sync_task(void*) {
  gRtcSyncTask = xTaskGetCurrentTaskHandle();
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000)); // allow Matter to initialize or be triggered early
  while (true) {
    bool commissioned = gCommissioned || is_commissioned();
    if (!commissioned) {
      ESP_LOGI(TAG, "RTC sync skipped (not commissioned)");
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(gRtcSyncIntervalMs));
      continue;
    }
    if (!gThreadAttached) {
      ESP_LOGI(TAG, "RTC sync skipped (thread not attached)");
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(gRtcSyncIntervalMs));
      continue;
    }

    if (gRtcInitialSyncDue && !gRtcInitialSyncDone) {
      ESP_LOGI(TAG, "RTC initial sync: waiting grace period");
      vTaskDelay(pdMS_TO_TICKS(RTC_SYNC_GRACE_MS));
      bool ok = run_network_rtc_sync(true, "commissioning-auto", "RTC sync", false);
      if (ok) {
        gRtcInitialSyncDone = true;
        gRtcInitialSyncDue = false;
      } else {
        ESP_LOGW(TAG, "RTC initial sync failed");
        gRtcInitialSyncDone = true; // don't retry until periodic interval
        gRtcInitialSyncDue = false;
      }
    } else {
      run_network_rtc_sync(true, "periodic-weekly", "RTC sync", false);
    }
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(gRtcSyncIntervalMs));
  }
}

static void renderClock(const ClockTime& t) {
  uint8_t h = t.hour % 12;
  if (h == 0) h = 12;
  uint8_t hTens = h / 10;
  uint8_t hOnes = h % 10;
  uint8_t mTens = t.minute / 10;
  uint8_t mOnes = t.minute % 10;

  if (hTens == 0) display.setBlank(DigitId::Clock1);
  else display.setDigit(DigitId::Clock1, hTens);
  display.setDigit(DigitId::Clock2, hOnes);
  display.setDigit(DigitId::Clock3, mTens);
  display.setDigit(DigitId::Clock4, mOnes);

  display.setIndicator(IndicatorId::Colon, true);
  display.setIndicator(IndicatorId::AmPm, t.pm);
}

static void renderHumidity(float rh) {
  if (!isfinite(rh)) {
    display.setBlank(DigitId::Hum5);
    display.setBlank(DigitId::Hum6);
    display.setIndicator(IndicatorId::HumidityPercent, false);
    return;
  }
  int r = (int)lroundf(rh);
  if (r < 0) r = 0;
  if (r > 99) r = 99;
  display.setDigit(DigitId::Hum5, (uint8_t)(r / 10));
  display.setDigit(DigitId::Hum6, (uint8_t)(r % 10));
  display.setIndicator(IndicatorId::HumidityPercent, true);
}

static void renderTemperature(float tempC) {
  if (!isfinite(tempC)) {
    display.setBlank(DigitId::Temp7);
    display.setBlank(DigitId::Temp8);
    display.setBlank(DigitId::Temp9);
    display.setBlank(DigitId::Temp10);
    display.setIndicator(IndicatorId::TempDecimal, false);
    display.setIndicator(IndicatorId::Degrees7, false);
    display.setIndicator(IndicatorId::Degrees10, false);
    return;
  }

  float tempF = tempC * 9.0f / 5.0f + 32.0f;
  int whole = (int)lroundf(tempF);
  if (whole < 0) whole = 0;
  if (whole > 120) whole = 120;

  int hundreds = (whole / 100) % 10;
  int tens = (whole / 10) % 10;
  int ones = whole % 10;

  if (whole >= 100) display.setDigit(DigitId::Temp7, (uint8_t)hundreds);
  else display.setBlank(DigitId::Temp7);

  if (whole >= 10) display.setDigit(DigitId::Temp8, (uint8_t)tens);
  else display.setBlank(DigitId::Temp8);

  display.setDigit(DigitId::Temp9, (uint8_t)ones);
  display.setSegments(DigitId::Temp10, kSegmentF);

  display.setIndicator(IndicatorId::TempDecimal, false);
  display.setIndicator(IndicatorId::Degrees7, false);
  display.setIndicator(IndicatorId::Degrees10, true);
}

static void updateDisplayFromState() {
  static float dispTempC = NAN;
  static float dispHum = NAN;
  static uint32_t lastSensorMs = 0;
  uint32_t now = millis_ms();
  if (now - lastSensorMs >= 15000) {
    if (isfinite(vTempC)) dispTempC = vTempC;
    if (isfinite(vHum)) dispHum = vHum;
    lastSensorMs = now;
  }
  display.clearBackBuffer();
  renderClock(readClockTime());
  renderHumidity(dispHum);
  renderTemperature(dispTempC);
  display.commit();
}

static void updateDisplayTestPattern() {
  static uint32_t last = 0;
  static uint8_t step = 0;
  uint32_t now = millis_ms();
  if (now - last < 500) return;
  last = now;

  display.clearBackBuffer();
  if (step < 10) {
    for (uint8_t d = 0; d < (uint8_t)DigitId::Count; d++) {
      display.setDigit(static_cast<DigitId>(d), step);
    }
    display.setIndicator(IndicatorId::Colon, true);
    display.setIndicator(IndicatorId::AmPm, true);
    display.setIndicator(IndicatorId::HumidityPercent, true);
    display.setIndicator(IndicatorId::TempDecimal, true);
    display.setIndicator(IndicatorId::Degrees7, true);
    display.setIndicator(IndicatorId::Degrees10, true);
  } else {
    for (uint8_t d = 0; d < (uint8_t)DigitId::Count; d++) {
      display.setSegments(static_cast<DigitId>(d), 0x7F);
    }
  }
  display.commit();
  step = (step + 1) % 11;
}

static void updateDisplayRhTestPattern() {
  static uint32_t last = 0;
  static uint8_t page = 0;
  static bool onPhase = true;
  uint32_t now = millis_ms();
  if (now - last < 500) return;
  last = now;

  display.clearBackBuffer();
  display.setChannelAllPages(3, true); // TLC channel 3 only
  if (onPhase) {
    display.setRhTestMode(true);
    display.setRhTestPage(page);
    display.setBrightness(TLC_MAX);
    ESP_LOGI(TAG, "RH test: CA%u ON", page);
    onPhase = false;
  } else {
    display.setRhTestMode(false);
    display.setBrightness(0);
    ESP_LOGI(TAG, "RH test: OFF");
    onPhase = true;
    page = (page + 1) & 0x03;
  }
  display.commit();
}

static void print_serial_help() {
  printf("=======SERIAL COMMANDS=======\n");
  printf("=============================\n");
  printf("________RUNTIME DEBUG________\n");
    printf("help - Show this command list.\n");
    printf("loginfo on|off - Set esp_clock log level at runtime.\n");
    printf("logtime on|off - Control verbose Matter/NTP time-sync soak logs.\n");
    printf("logheap on|off|reset - Control per-section heap diagnostics.\n");
    printf("rebootcause - Print previous/current reset reason.\n");
    printf("reboothistory - Print stored reset history ring.\n");
  printf("__________RTC DEBUG__________\n");
    printf("rtc - Print RTC UTC time.\n");
    printf("rtc YYYY-MM-DD HH:MM:SS - Set RTC UTC time.\n");
    printf("rtc +/-SECONDS - Adjust RTC by a signed second delta.\n");
  printf("_______TIME SYNC DEBUG_______\n");
    printf("timesync - Show time sync state and interval.\n");
    printf("timesync now - Run immediate NTP->RTC sync flow.\n");
    printf("timesync interval <seconds> - Set runtime periodic sync interval.\n");
    printf("timesync interval default - Restore default periodic sync interval.\n");
  printf("__________NTP DEBUG__________\n");
    printf("ntptest - Run compare-only NTP vs RTC test.\n");
    printf("ntptest sync - Run NTP test and sync RTC if needed.\n");
    printf("ntpserver - Show configured fallback NTP servers.\n");
    printf("ntpserver v6 <host|ip> - Set fallback IPv6 NTP server.\n");
    printf("ntpserver v4 <host|ip> - Set fallback IPv4 NTP server.\n");
    printf("ntpserver default - Restore default fallback NTP servers.\n");
  printf("________DISPLAY DEBUG________\n");
    printf("pwm <1-4095> - Set fixed display PWM brightness.\n");
    printf("pwm auto - Re-enable automatic brightness control.\n");
    printf("refresh <microseconds> - Set display multiplex page period.\n");
    printf("display - Print display host/pin mapping.\n");

}

static void handleDisplaySerialCommands() {
  static char buf[64];
  static size_t idx = 0;
  uint8_t ch;
  while (true) {
    bool got = false;
#if ESP_CLOCK_HAS_USB_SERIAL_JTAG
    if (gUsbSerialReady && usb_serial_jtag_read_bytes(&ch, 1, 0) == 1) {
      got = true;
    }
#endif
    if (!got && uart_read_bytes(UART_NUM_0, &ch, 1, 0) == 1) {
      got = true;
    }
    if (!got) break;
    if (ch == '\n' || ch == '\r') {
      buf[idx] = '\0';
      if (idx > 0) {
        if (strcmp(buf, "help") == 0) {
          print_serial_help();
        } else if (strncmp(buf, "loginfo on", 10) == 0) {
          esp_log_level_set(TAG, ESP_LOG_INFO);
          printf("esp_clock log level: INFO\n");
        } else if (strncmp(buf, "loginfo off", 11) == 0) {
          esp_log_level_set(TAG, ESP_LOG_ERROR);
          printf("esp_clock log level: ERROR\n");
        } else if (strcmp(buf, "logtime on") == 0) {
          gLogTimeEnabled = true;
          ESP_LOGI(TAG, "logtime: enabled");
        } else if (strcmp(buf, "logtime off") == 0) {
          gLogTimeEnabled = false;
          ESP_LOGI(TAG, "logtime: disabled");
        } else if (strcmp(buf, "logheap on") == 0 || strcmp(buf, "heapdiag on") == 0) {
          gHeapDiagEnabled = true;
          gHeapDiagLastFree = heap_caps_get_free_size(MALLOC_CAP_8BIT);
          gHeapDiagLastLargest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
          heap_diag_reset();
          ESP_LOGI(TAG, "logheap: enabled");
        } else if (strcmp(buf, "logheap off") == 0 || strcmp(buf, "heapdiag off") == 0) {
          gHeapDiagEnabled = false;
          ESP_LOGI(TAG, "logheap: disabled");
        } else if (strcmp(buf, "logheap reset") == 0 || strcmp(buf, "heapdiag reset") == 0) {
          gHeapDiagLastFree = heap_caps_get_free_size(MALLOC_CAP_8BIT);
          gHeapDiagLastLargest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
          heap_diag_reset();
          ESP_LOGI(TAG, "logheap: counters reset");
        } else if (strncmp(buf, "pwm auto", 8) == 0) {
          gBrightnessOverride = false;
          display.setScaleRh(true);
          printf("pwm: auto (ALS)\n");
        } else if (strncmp(buf, "pwm ", 4) == 0) {
          char* p = buf + 4;
          while (*p == ' ') p++;
          uint32_t level = (uint32_t)strtoul(p, nullptr, 10);
          if (level < 1) level = 1;
          if (level > TLC_MAX) level = TLC_MAX;
          display.setBrightness((uint16_t)level);
          gBrightnessOverride = true;
          display.setScaleRh(false);
          ESP_LOGI(TAG, "PWM level = %u", (unsigned)level);
        } else if (strncmp(buf, "refresh", 7) == 0) {
          char* p = buf + 7;
          while (*p == ' ') p++;
          uint32_t us = (uint32_t)strtoul(p, nullptr, 10);
          if (us > 0) display.setPagePeriodUs(us);
          ESP_LOGI(TAG, "display page period = %u us (frame %.1f Hz)",
                   display.pagePeriodUs(),
                   1000000.0f / (display.pagePeriodUs() * 4.0f));
        } else if (strcmp(buf, "display") == 0) {
          ESP_LOGI(TAG, "display host=%s MOSI=%d SCK=%d LATCH=%d CA1=%d CA2=%d CA3=%d CA4=%d",
                   display.hostName(), TLC_SPI_MOSI, TLC_SPI_SCLK, TLC_SPI_LATCH,
                   TBD_PIN_CA1, TBD_PIN_CA2, TBD_PIN_CA3, TBD_PIN_CA4);
        } else if (strncmp(buf, "rtc ", 4) == 0) {
          char* p = buf + 4;
          while (*p == ' ') p++;
          if (*p == '+' || *p == '-') {
            int32_t delta = (int32_t)strtol(p, nullptr, 10);
            bool ok = rtc_adjust_seconds(delta);
            ESP_LOGI(TAG, "RTC adjust %+ld sec %s", (long)delta, ok ? "OK" : "FAILED");
          } else {
            int y, mo, d, h, mi, s;
            if (sscanf(p, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) == 6) {
              bool ok = rtc_set_time((uint16_t)y, (uint8_t)mo, (uint8_t)d,
                                     (uint8_t)h, (uint8_t)mi, (uint8_t)s);
              ESP_LOGI(TAG, "RTC set %s", ok ? "OK" : "FAILED");
            } else {
              ESP_LOGW(TAG, "RTC set format (UTC): rtc YYYY-MM-DD HH:MM:SS");
            }
          }
        } else if (strcmp(buf, "rtc") == 0) {
          RtcDateTime dt = {};
          if (read_rtc_utc(&dt)) {
            ESP_LOGI(TAG, "RTC UTC %04u-%02u-%02u %02u:%02u:%02u",
                     dt.year, dt.mon, dt.day, dt.hour, dt.min, dt.sec);
          } else {
            ESP_LOGW(TAG, "RTC read failed");
          }
        } else if (strncmp(buf, "ntpserver ", 10) == 0) {
          char* p = buf + 10;
          while (*p == ' ') p++;
          if (strcmp(p, "default") == 0) {
            reset_ntp_servers_to_default();
            ESP_LOGI(TAG, "NTP servers reset to defaults: v6=%s v4=%s", gNtpServerIpv6, gNtpServerIpv4);
          } else if (strncmp(p, "v6 ", 3) == 0) {
            bool ok = set_ntp_server(gNtpServerIpv6, sizeof(gNtpServerIpv6), p + 3);
            ESP_LOGI(TAG, "NTP v6 server set: %s", ok ? gNtpServerIpv6 : "FAILED");
          } else if (strncmp(p, "v4 ", 3) == 0) {
            bool ok = set_ntp_server(gNtpServerIpv4, sizeof(gNtpServerIpv4), p + 3);
            ESP_LOGI(TAG, "NTP v4 server set: %s", ok ? gNtpServerIpv4 : "FAILED");
          } else {
            ESP_LOGW(TAG, "NTP server format: ntpserver v6 <host|ip> | ntpserver v4 <host|ip> | ntpserver default");
          }
        } else if (strcmp(buf, "ntpserver") == 0) {
          ESP_LOGI(TAG, "NTP servers: v6=%s v4=%s", gNtpServerIpv6, gNtpServerIpv4);
        } else if (strcmp(buf, "timesync now") == 0) {
          bool ok = run_network_rtc_sync(true, "serial-timesync-now", "RTC sync", true);
          ESP_LOGI(TAG, "Time sync command result: %s", ok ? "SUCCESS" : "FAILURE");
        } else if (strcmp(buf, "ntptest sync") == 0) {
          bool ok = run_ntp_rtc_test(true);
          ESP_LOGI(TAG, "NTP test command result: %s", ok ? "SUCCESS" : "FAILURE");
        } else if (strcmp(buf, "ntptest") == 0) {
          bool ok = run_ntp_rtc_test(false);
          ESP_LOGI(TAG, "NTP test command result: %s", ok ? "SUCCESS" : "FAILURE");
        } else if (strncmp(buf, "timesync interval", 17) == 0) {
          char* p = buf + 17;
          while (*p == ' ') p++;
          if (strcmp(p, "default") == 0) {
            gRtcSyncIntervalMs = RTC_SYNC_INTERVAL_MS;
            ESP_LOGI(TAG, "Time sync interval reset to default (%lu ms)",
                     (unsigned long)gRtcSyncIntervalMs);
          } else {
            uint32_t sec = (uint32_t)strtoul(p, nullptr, 10);
            if (sec < 1) sec = 1;
            gRtcSyncIntervalMs = sec * 1000UL;
            ESP_LOGI(TAG, "Time sync interval set to %lu s (%lu ms)",
                     (unsigned long)sec, (unsigned long)gRtcSyncIntervalMs);
          }
        } else if (strcmp(buf, "timesync") == 0) {
          log_time_sync_state("serial-timesync", true);
          ESP_LOGI(TAG, "Time sync logging flags: logtime=%d logheap=%d",
                   (int)gLogTimeEnabled, (int)gHeapDiagEnabled);
        } else if (strcmp(buf, "rebootcause") == 0) {
          char prevTs[32] = {};
          char currTs[32] = {};
          formatEpochUtc(gPrevResetEpoch, prevTs, sizeof(prevTs));
          formatEpochUtc(gCurrentResetEpoch, currTs, sizeof(currTs));
          ESP_LOGI(TAG, "Boot diag: boot_count=%lu prev_reset=%s(%u) prev_time=%s current_reset=%s(%u) current_time=%s",
                   (unsigned long)gBootCount,
                   resetReasonToStr(gPrevResetReason), (unsigned)gPrevResetReason,
                   prevTs,
                   resetReasonToStr(gCurrentResetReason), (unsigned)gCurrentResetReason,
                   currTs);
        } else if (strcmp(buf, "reboothistory") == 0) {
          reset_history_log(&gResetHistory, "serial");
        }
      }
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = (char)ch;
    }
  }
}

static void init_i2c() {
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = SDA_PIN;
  bus_cfg.scl_io_num = SCL_PIN;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2cBus));

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.scl_speed_hz = I2C_HZ;

  dev_cfg.device_address = BME_ADDR;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBus, &dev_cfg, &bmeDev));

  dev_cfg.device_address = DS3231_ADDR;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBus, &dev_cfg, &rtcDev));

  dev_cfg.device_address = VEML7700_ADDR;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBus, &dev_cfg, &vemlDev));

  gHasBme680 = i2c_probe_addr(BME_ADDR);
  gHasRtc = i2c_probe_addr(DS3231_ADDR);
  gHasVeml7700 = i2c_probe_addr(VEML7700_ADDR);
  ESP_LOGI(TAG, "I2C probe: BME680=%s RTC(DS3231)=%s VEML7700=%s",
           gHasBme680 ? "present" : "missing",
           gHasRtc ? "present" : "missing",
           gHasVeml7700 ? "present" : "missing");
}

static void init_bsec() {
  if (!gHasBme680) {
    ESP_LOGW(TAG, "BME680 not detected; skipping BSEC init");
    return;
  }
  env.begin(BME68X_I2C_INTF, i2c_read_bytes, i2c_write_bytes, delay_us, nullptr, millis_ms);
  loadBsecState();
  env.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  env.attachCallback(onBsecOutputs);
  env.setTemperatureOffset(TEMP_OFFSET_C);
}

static void init_veml7700() {
  if (!vemlDev || !gHasVeml7700) {
    ESP_LOGW(TAG, "VEML7700 not detected; ALS auto-brightness disabled");
    return;
  }
  // ALS_CONF_0 register (0x00): set to default-ish, power on.
  // 0x0000 is a safe baseline on most boards.
  uint8_t cfg[2] = {0x00, 0x00};
  i2c_write_dev(vemlDev, 0x00, cfg, sizeof(cfg));
}

static uint16_t read_veml7700() {
  uint8_t buf[2] = {};
  if (!i2c_read_dev(vemlDev, 0x04, buf, sizeof(buf))) return 0;
  return (uint16_t)(buf[0] | (buf[1] << 8));
}

static uint16_t scale_brightness(uint16_t als) {
  // Map ALS raw to a usable brightness range.
  // Tune as needed based on your enclosure and LEDs.
  const uint16_t min = 50;
  const uint16_t max = 2000;
  // Minimum brightness at/under the darkest ALS reading.
  const float min_level = 0.12f;
  if (als <= min) return (uint16_t)(TLC_MAX * min_level);
  if (als >= max) return TLC_MAX;
  float t = (float)(als - min) / (float)(max - min);
  float level = min_level + t * (1.0f - min_level);
  return (uint16_t)(level * TLC_MAX);
}

static void sensor_task(void*) {
  uint32_t lastLuxMs = 0;
  uint32_t lastLogMs = 0;
  uint32_t lastTimeStateLogMs = 0;
  bool threadConfigured = false;
  while (true) {
    #if !DISABLE_TIME_SYNC
      size_t heapBeforeTimeSync = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      time_sync_poll();
      size_t heapAfterTimeSync = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      heap_diag_account(HeapDiagBucket::TimeSyncPoll, heapBeforeTimeSync, heapAfterTimeSync);
    #endif
    if (gHasBme680) {
      size_t heapBeforeBsecRun = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      bool bsecOk = env.run();
      size_t heapAfterBsecRun = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      heap_diag_account(HeapDiagBucket::BsecRun, heapBeforeBsecRun, heapAfterBsecRun);
      if (!bsecOk) {
        ESP_LOGW(TAG, "BSEC run returned false");
      }
    }
    if (!threadConfigured) {
#if ESP_CLOCK_HAS_MATTER && ESP_CLOCK_HAS_OPENTHREAD
      if (esp_matter::is_started()) {
        configureThreadRouterEligibility();
        threadConfigured = true;
      }
#endif
    }
    size_t heapBeforeMatter = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    matter_update();
    size_t heapAfterMatter = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    heap_diag_account(HeapDiagBucket::MatterUpdate, heapBeforeMatter, heapAfterMatter);
    uint32_t now = millis_ms();
    if (now - lastLuxMs > 500) {
      size_t heapBeforeAls = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      if (!DISPLAY_RH_TEST_MODE && !gBrightnessOverride) {
        uint16_t als = read_veml7700();
        display.setBrightness(scale_brightness(als));
      }
      size_t heapAfterAls = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      heap_diag_account(HeapDiagBucket::AlsUpdate, heapBeforeAls, heapAfterAls);
      lastLuxMs = now;
    }
    size_t heapBeforeDisplay = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    if (DISPLAY_RH_TEST_MODE) updateDisplayRhTestPattern();
    else if (DISPLAY_TEST_MODE) updateDisplayTestPattern();
    else updateDisplayFromState();
    size_t heapAfterDisplay = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    heap_diag_account(HeapDiagBucket::DisplayUpdate, heapBeforeDisplay, heapAfterDisplay);
    if (gHasBme680) {
      size_t heapBeforeBsecSave = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      saveBsecStateIfReady(millis_ms());
      size_t heapAfterBsecSave = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      heap_diag_account(HeapDiagBucket::BsecStateSave, heapBeforeBsecSave, heapAfterBsecSave);
    }
    size_t heapBeforeSerial = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    handleDisplaySerialCommands();
    size_t heapAfterSerial = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    heap_diag_account(HeapDiagBucket::SerialCmd, heapBeforeSerial, heapAfterSerial);

    if (now - lastLogMs > 5000) {
      size_t heapBeforeStatusLog = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      uint32_t logWindowMs = now - lastLogMs;
      ClockTime ct = readClockTime();
      int h12 = ct.hour % 12;
      if (h12 == 0) h12 = 12;
      int year = 0;
      int mon = 0;
      int day = 0;
      time_t rtcEpoch = 0;
      char rtcUtc[40] = {};
      if (rtc_read_epoch(&rtcEpoch)) {
        struct tm tmUtc = {};
        gmtime_r(&rtcEpoch, &tmUtc);
        year = tmUtc.tm_year + 1900;
        mon = tmUtc.tm_mon + 1;
        day = tmUtc.tm_mday;
        formatEpochUtc((int64_t)rtcEpoch, rtcUtc, sizeof(rtcUtc));
      } else {
        snprintf(rtcUtc, sizeof(rtcUtc), "unknown");
      }
      time_t sysEpoch = time(nullptr);
      struct tm sysUtcTm = {};
      gmtime_r(&sysEpoch, &sysUtcTm);
      char sysUtc[40] = {};
      formatEpochUtc((int64_t)sysEpoch, sysUtc, sizeof(sysUtc));
      const int utcToLocalOffsetMin = local_utc_offset_minutes_for_epoch(sysEpoch);
      char utcOffset[16] = {};
      format_utc_offset(utcToLocalOffsetMin, utcOffset, sizeof(utcOffset));
      const UBaseType_t hwmWords = uxTaskGetStackHighWaterMark(nullptr);
      char tempBuf[16] = "nan";
      char rhBuf[16] = "nan";
      if (isfinite(vTempC)) {
        int tempTenthsF = (int)lroundf((vTempC * 9.0f / 5.0f + 32.0f) * 10.0f);
        int tempWhole = tempTenthsF / 10;
        int tempFrac = abs(tempTenthsF % 10);
        snprintf(tempBuf, sizeof(tempBuf), "%d.%d", tempWhole, tempFrac);
      }
      if (isfinite(vHum)) {
        int rhTenths = (int)lroundf(vHum * 10.0f);
        int rhWhole = rhTenths / 10;
        int rhFrac = abs(rhTenths % 10);
        snprintf(rhBuf, sizeof(rhBuf), "%d.%d", rhWhole, rhFrac);
      }
      ESP_LOGI(TAG, "CLOCK_STATUS rtc_utc=\"%s\" rtc_date_utc=%04d-%02d-%02d display_12h=%02d:%02d:%02u(%s) display_24h=%02u:%02u:%02u sys_utc=\"%s\" sys_utc_hms=%02d:%02d:%02d utc_to_local_apply=\"%s\" utc_to_local_offset_min=%d temp_f=%s rh_pct=%s",
               rtcUtc,
               year, mon, day,
               h12, ct.minute, ct.second, ct.pm ? "PM" : "AM",
               ct.hour, ct.minute, ct.second,
               sysUtc,
               sysUtcTm.tm_hour, sysUtcTm.tm_min, sysUtcTm.tm_sec,
               utcOffset, utcToLocalOffsetMin,
               tempBuf, rhBuf);
      int64_t lastCompareEpoch = 0;
      double lastCompareDriftSec = NAN;
      get_last_ntp_compare(&lastCompareEpoch, &lastCompareDriftSec);
      if (lastCompareEpoch > 0) {
        char lastCompareUtc[40] = {};
        formatEpochUtc(lastCompareEpoch, lastCompareUtc, sizeof(lastCompareUtc));
        double daysAgo = ((double)sysEpoch - (double)lastCompareEpoch) / 86400.0;
        if (daysAgo < 0.0) daysAgo = 0.0;
        int32_t daysAgo100 = (int32_t)lround(daysAgo * 100.0);
        int32_t daysWhole = daysAgo100 / 100;
        int32_t daysFrac = abs(daysAgo100 % 100);
        if (isfinite(lastCompareDriftSec)) {
          int32_t driftMs = (int32_t)lround(fabs(lastCompareDriftSec) * 1000.0);
          int32_t driftWhole = driftMs / 1000;
          int32_t driftFrac = abs(driftMs % 1000);
          ESP_LOGI(TAG,
                   "Last NTP compare %ld.%02ld days ago (utc=%s) | Drift %ld.%03ld sec (%s)",
                   (long)daysWhole, (long)daysFrac, lastCompareUtc,
                   (long)driftWhole, (long)driftFrac,
                   (lastCompareDriftSec >= 0.0) ? "RTC behind NTP" : "RTC ahead of NTP");
        } else {
          ESP_LOGI(TAG,
                   "Last NTP compare %ld.%02ld days ago (utc=%s) | Drift unavailable",
                   (long)daysWhole, (long)daysFrac, lastCompareUtc);
        }
      } else {
        ESP_LOGI(TAG, "Last NTP compare: none yet");
      }
      if (gLogTimeEnabled && (now - lastTimeStateLogMs > 30000)) {
        log_time_sync_state("periodic-soak", false);
        lastTimeStateLogMs = now;
      }
      if (gHeapDiagEnabled) {
        size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI(TAG, "Health: free_heap=%uB sensor_stack_hwm=%uB",
                 (unsigned)freeHeap, (unsigned)(hwmWords * sizeof(StackType_t)));
        size_t heapAfterStatusLog = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        heap_diag_account(HeapDiagBucket::StatusLogBlock, heapBeforeStatusLog, heapAfterStatusLog);
        heap_diag_log_and_reset(logWindowMs);
      }
      lastLogMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

extern "C" void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  {
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
    }
  }
  esp_log_level_set(TAG, ESP_LOG_INFO);
  gRtcSyncMutex = xSemaphoreCreateMutex();
  if (!gRtcSyncMutex) {
    ESP_LOGW(TAG, "RTC sync mutex create failed; sync operations may overlap");
  }
#if ESP_CLOCK_HAS_OT_LAUNCHER
  init_openthread_nvs();
#endif
  uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
#if ESP_CLOCK_HAS_USB_SERIAL_JTAG
  usb_serial_jtag_driver_config_t usb_cfg = {};
  usb_cfg.rx_buffer_size = 1024;
  usb_cfg.tx_buffer_size = 1024;
  esp_err_t usbErr = usb_serial_jtag_driver_install(&usb_cfg);
  if (usbErr == ESP_OK) {
    gUsbSerialReady = true;
    ESP_LOGI(TAG, "USB Serial JTAG ready; UART0 remains fallback");
  } else {
    gUsbSerialReady = false;
    ESP_LOGW(TAG, "USB Serial JTAG init failed (%s); using UART0", esp_err_to_name(usbErr));
  }
#endif
#if ESP_CLOCK_HAS_OT_LAUNCHER
  init_openthread_platform_config();
#endif

  init_i2c();
  recordBootDiagnostics();
  init_bsec();
  init_veml7700();
  gHeapDiagLastFree = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  gHeapDiagLastLargest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  heap_diag_reset();
  setenv("TZ", LOCAL_TZ, 1);
  tzset();

  uint8_t status = 0;
  if (i2c_read_dev(rtcDev, 0x0F, &status, 1)) {
    // OSF bit indicates the RTC lost power and its time is invalid.
    if (status & 0x80) {
      ESP_LOGW(TAG, "RTC lost power (OSF=1); RTC time marked invalid until Matter sync");
    } else {
      ESP_LOGI(TAG, "RTC status OK (OSF=0)");
    }
  } else {
    ESP_LOGW(TAG, "RTC status read failed");
  }
  // Startup clock-source hierarchy (best to worst):
  // 1) RTC (if plausible and readable)
  // 2) Unsynced (no compile-time fallback)
  // Matter/network time is correction-only and updates RTC after validation.
  {
    time_t rtc_epoch = 0;
    time_t floor = min_valid_epoch();
    ESP_LOGI(TAG, "Time validity floor epoch: %ld", (long)floor);
    if (rtc_read_epoch(&rtc_epoch)) {
      if (rtc_epoch >= floor) {
        ESP_LOGI(TAG, "Clock source selected: RTC");
        set_system_time_from_rtc();
      } else {
        ESP_LOGW(TAG, "RTC epoch %ld is below validity floor %ld; clock remains unsynced",
                 (long)rtc_epoch, (long)floor);
      }
    } else {
      ESP_LOGW(TAG, "RTC read failed; clock remains unsynced");
    }
  }
  #if !DISABLE_TIME_SYNC
    time_sync_set_ready_callback(onTimeSyncReady);
    time_sync_init();
  #else
    ESP_LOGW(TAG, "Time sync disabled (DISABLE_TIME_SYNC=1)");
  #endif
  matter_init();
  gCommissioned = is_commissioned();
#if ESP_CLOCK_HAS_MATTER
  ESP_LOGI(TAG, "Boot commissioning state: commissioned=%s fabrics=%lu",
           gCommissioned ? "yes" : "no",
           (unsigned long)chip::Server::GetInstance().GetFabricTable().FabricCount());
#else
  ESP_LOGI(TAG, "Boot commissioning state: Matter disabled");
#endif

  display.init();

  ESP_LOGI(TAG, "display host=%s MOSI=%d SCK=%d LATCH=%d CA1=%d CA2=%d CA3=%d CA4=%d",
           display.hostName(), TLC_SPI_MOSI, TLC_SPI_SCLK, TLC_SPI_LATCH,
           TBD_PIN_CA1, TBD_PIN_CA2, TBD_PIN_CA3, TBD_PIN_CA4);
  ESP_LOGI(TAG, "display page period = %u us (frame %.1f Hz)",
           display.pagePeriodUs(),
           1000000.0f / (display.pagePeriodUs() * 4.0f));

  xTaskCreate(sensor_task, "sensor_task", 8192, nullptr, 5, nullptr);
  #if !DISABLE_TIME_SYNC
  xTaskCreate(rtc_time_sync_task, "rtc_time_sync", 4096, nullptr, 4, nullptr);
  #endif
}
