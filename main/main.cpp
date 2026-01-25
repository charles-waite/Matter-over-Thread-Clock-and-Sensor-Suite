#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#ifndef ESP_CLOCK_VERBOSE_LOGS
#define ESP_CLOCK_VERBOSE_LOGS 1
#endif

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_sntp.h"
#include "soc/soc_caps.h"
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
#include <openthread/link.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include <openthread/dataset.h>
#if __has_include(<openthread/thread_ftd.h>)
#include <openthread/thread_ftd.h>
#endif
#define ESP_CLOCK_HAS_OPENTHREAD 1
#else
#define ESP_CLOCK_HAS_OPENTHREAD 0
#endif

#if __has_include("esp_matter.h")
#include "esp_matter.h"
#include <app/util/attribute-table.h>
#include <app-common/zap-generated/attribute-type.h>
#include <platform/PlatformManager.h>
#include <lib/support/logging/CHIPLogging.h>
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

#if ESP_CLOCK_HAS_MATTER
static void chipLogRedirect(const char * module, uint8_t category, const char * msg, va_list args) {
  const char * cat = "I";
  if (category == chip::Logging::kLogCategory_Error) cat = "E";
  else if (category == chip::Logging::kLogCategory_Detail) cat = "D";
  else if (category == chip::Logging::kLogCategory_Automation) cat = "A";
  printf("%s chip[%s]: ", cat, module);
  vprintf(msg, args);
  printf("\n");
}
#endif

// -------------------- Board + I2C --------------------
static constexpr gpio_num_t SDA_PIN = GPIO_NUM_22; // D4
static constexpr gpio_num_t SCL_PIN = GPIO_NUM_23; // D5
static constexpr uint32_t I2C_HZ = 400000;

// -------------------- Sensors --------------------
static constexpr uint8_t BME_ADDR = 0x77;
static constexpr float TEMP_OFFSET_C = 4.0f;
static constexpr uint8_t DS3231_ADDR = 0x68;
static constexpr uint8_t VEML7700_ADDR = 0x10;
static constexpr const char* LOCAL_TZ = "PST8PDT,M3.2.0/2,M11.1.0/2";
static constexpr uint32_t RTC_SYNC_INTERVAL_MS = 30UL * 24UL * 60UL * 60UL * 1000UL;
static constexpr uint32_t RTC_SYNC_MATTER_TIMEOUT_MS = 60UL * 1000UL;
static constexpr uint32_t RTC_SYNC_SNTP_TIMEOUT_MS = 30UL * 1000UL;
static constexpr time_t RTC_VALID_EPOCH = 1700000000; // 2023-11-14
static constexpr gpio_num_t BUTTON_PIN = GPIO_NUM_9;
static constexpr uint32_t BUTTON_DECOMMISSION_MS = 5000;
static constexpr bool CLEAR_THREAD_DATASET_ON_BOOT = true;

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
#if ESP_CLOCK_HAS_MATTER
static volatile bool gChipLogRedirectEnabled = ESP_CLOCK_VERBOSE_LOGS;
#endif
static TaskHandle_t gSntpWaiter = nullptr;
static volatile bool gSntpSynced = false;

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
static const char* threadRoleStr(otDeviceRole role) {
  switch (role) {
    case OT_DEVICE_ROLE_DISABLED: return "disabled";
    case OT_DEVICE_ROLE_DETACHED: return "detached";
    case OT_DEVICE_ROLE_CHILD: return "child";
    case OT_DEVICE_ROLE_ROUTER: return "router";
    case OT_DEVICE_ROLE_LEADER: return "leader";
    default: return "unknown";
  }
}

static void logThreadState(const char* label);

static void logDatasetInfo(const otOperationalDataset * dataset, const char * label) {
  if (!dataset) return;
  char name[OT_NETWORK_NAME_MAX_SIZE + 1] = {0};
  char prefix[OT_IP6_ADDRESS_STRING_SIZE] = {0};
  if (dataset->mComponents.mIsNetworkNamePresent) {
    size_t len = strnlen(dataset->mNetworkName.m8, OT_NETWORK_NAME_MAX_SIZE);
    memcpy(name, dataset->mNetworkName.m8, len);
    name[len] = '\0';
  }
  if (dataset->mComponents.mIsMeshLocalPrefixPresent) {
    otIp6AddressToString(reinterpret_cast<const otIp6Address *>(&dataset->mMeshLocalPrefix), prefix, sizeof(prefix));
  }
  ESP_LOGI(TAG, "Thread dataset (%s): name=%s channel=%u panid=0x%04x",
           label,
           dataset->mComponents.mIsNetworkNamePresent ? name : "(unset)",
           dataset->mComponents.mIsChannelPresent ? dataset->mChannel : 0,
           dataset->mComponents.mIsPanIdPresent ? dataset->mPanId : 0);
  if (dataset->mComponents.mIsExtendedPanIdPresent) {
    const uint8_t * ext = dataset->mExtendedPanId.m8;
    ESP_LOGI(TAG, "Thread dataset (%s): extpanid=%02x%02x%02x%02x%02x%02x%02x%02x",
             label, ext[0], ext[1], ext[2], ext[3], ext[4], ext[5], ext[6], ext[7]);
  }
  if (dataset->mComponents.mIsMeshLocalPrefixPresent) {
    ESP_LOGI(TAG, "Thread dataset (%s): mesh-local=%s", label, prefix);
  }
}

static void openthread_event_handler(void * arg, esp_event_base_t base, int32_t id, void * data) {
  (void)arg;
  (void)base;
  switch (id) {
    case OPENTHREAD_EVENT_ATTACHED:
      ESP_LOGI(TAG, "Thread event: attached");
      logDatasetInfo(reinterpret_cast<otOperationalDataset *>(data), "attached");
      logThreadState("attached");
      break;
    case OPENTHREAD_EVENT_DETACHED:
      ESP_LOGI(TAG, "Thread event: detached");
      logThreadState("detached");
      break;
    case OPENTHREAD_EVENT_ROLE_CHANGED: {
      auto * evt = reinterpret_cast<esp_openthread_role_changed_event_t *>(data);
      if (evt) {
        ESP_LOGI(TAG, "Thread role change: %s -> %s",
                 threadRoleStr(evt->previous_role), threadRoleStr(evt->current_role));
      }
      logThreadState("role-change");
      break;
    }
    case OPENTHREAD_EVENT_DATASET_CHANGED: {
      auto * evt = reinterpret_cast<esp_openthread_dataset_changed_event_t *>(data);
      if (evt) {
        ESP_LOGI(TAG, "Thread dataset changed: %s",
                 evt->type == OPENTHREAD_ACTIVE_DATASET ? "active" : "pending");
        logDatasetInfo(&evt->new_dataset, "dataset-change");
        if (evt->type == OPENTHREAD_ACTIVE_DATASET) {
          otInstance* instance = esp_openthread_get_instance();
          if (instance && otDatasetIsCommissioned(instance)) {
            otDeviceRole role = otThreadGetDeviceRole(instance);
            if (role == OT_DEVICE_ROLE_DISABLED || role == OT_DEVICE_ROLE_DETACHED) {
              otError err = otIp6SetEnabled(instance, true);
              ESP_LOGI(TAG, "Thread dataset active; ip6 enable -> %s", otThreadErrorToString(err));
              err = otThreadSetEnabled(instance, true);
              ESP_LOGI(TAG, "Thread dataset active; thread enable -> %s", otThreadErrorToString(err));
            }
          }
        }
      }
      break;
    }
    default:
      break;
  }
}

static void logThreadState(const char* label) {
#if ESP_CLOCK_HAS_OPENTHREAD && CONFIG_OPENTHREAD_ENABLED
  otInstance* instance = esp_openthread_get_instance();
  if (!instance) {
    ESP_LOGW(TAG, "Thread state (%s): no instance", label);
    return;
  }
  otDeviceRole role = otThreadGetDeviceRole(instance);
  bool commissioned = otDatasetIsCommissioned(instance);
  uint16_t rloc16 = otThreadGetRloc16(instance);
  char addr[OT_IP6_ADDRESS_STRING_SIZE];

  const otIp6Address* linkLocal = otThreadGetLinkLocalIp6Address(instance);
  const otIp6Address* meshLocal = otThreadGetMeshLocalEid(instance);
  const otIp6Address* rloc = otThreadGetRloc(instance);
  ESP_LOGI(TAG, "Thread state (%s): role=%s commissioned=%s rloc16=0x%04x",
           label, threadRoleStr(role), commissioned ? "yes" : "no", rloc16);
  if (linkLocal) {
    otIp6AddressToString(linkLocal, addr, sizeof(addr));
    ESP_LOGI(TAG, "Thread addr (%s): link-local=%s", label, addr);
  }
  if (meshLocal) {
    otIp6AddressToString(meshLocal, addr, sizeof(addr));
    ESP_LOGI(TAG, "Thread addr (%s): mesh-local=%s", label, addr);
  }
  if (rloc) {
    otIp6AddressToString(rloc, addr, sizeof(addr));
    ESP_LOGI(TAG, "Thread addr (%s): rloc=%s", label, addr);
  }

  for (const otNetifAddress* a = otIp6GetUnicastAddresses(instance); a; a = a->mNext) {
    if (a->mAddress.mFields.m8[0] == 0xfe && (a->mAddress.mFields.m8[1] & 0xc0) == 0x80) {
      continue; // skip link-local (already logged)
    }
    otIp6AddressToString(&a->mAddress, addr, sizeof(addr));
    ESP_LOGI(TAG, "Thread addr (%s): unicast=%s", label, addr);
  }
#endif
}

static void maybeClearThreadDataset() {
#if ESP_CLOCK_HAS_OPENTHREAD && CONFIG_OPENTHREAD_ENABLED
  if (!CLEAR_THREAD_DATASET_ON_BOOT) return;
  otInstance* instance = esp_openthread_get_instance();
  if (!instance) return;
  if (otDatasetIsCommissioned(instance)) return;
  otError err = otInstanceErasePersistentInfo(instance);
  ESP_LOGW(TAG, "Thread dataset not commissioned; erased persistent info (%d)", err);
#endif
}

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

static void matter_event_callback(const ChipDeviceEvent* event, intptr_t) {
  if (!event) return;
  switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
      ESP_LOGI(TAG, "Matter commissioning complete");
      logThreadState("commissioning-complete");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
      ESP_LOGI(TAG, "Matter commissioning session started");
      logThreadState("commissioning-start");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
      ESP_LOGI(TAG, "Matter commissioning session stopped");
      break;
    default:
      break;
  }
}

static uint8_t iaq_to_air_quality_enum(float iaq) {
  if (!isfinite(iaq)) return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kUnknown);
  if (iaq <= 50.0f) return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kGood);
  if (iaq <= 100.0f) return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kFair);
  if (iaq <= 150.0f) return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kModerate);
  if (iaq <= 200.0f) return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kPoor);
  if (iaq <= 300.0f) return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kVeryPoor);
  return static_cast<uint8_t>(chip::app::Clusters::AirQuality::AirQualityEnum::kExtremelyPoor);
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

  air_quality_sensor::config_t aq_cfg;
  endpoint_t* aq_ep = air_quality_sensor::create(node, &aq_cfg, ENDPOINT_FLAG_NONE, nullptr);
  if (aq_ep) {
    matterAirEndpoint = endpoint::get_id(aq_ep);
    cluster::carbon_dioxide_concentration_measurement::config_t co2_cfg;
    co2_cfg.feature_flags = cluster::concentration_measurement::feature::numeric_measurement::get_id();
    cluster::carbon_dioxide_concentration_measurement::create(aq_ep, &co2_cfg, CLUSTER_FLAG_SERVER);
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
    if (isfinite(vIAQ) && vIAQacc >= 2) {
      uint8_t aq = iaq_to_air_quality_enum(vIAQ);
      esp_matter_attr_val_t attr = esp_matter_enum8(aq);
      esp_matter::attribute::update(matterAirEndpoint, AirQuality::Id,
                                    AirQuality::Attributes::AirQuality::Id, &attr);
    }
    if (matterCo2Enabled && isfinite(vCO2eq)) {
      esp_matter_attr_val_t attr = esp_matter_nullable_float(vCO2eq);
      esp_matter::attribute::update(matterAirEndpoint, CarbonDioxideConcentrationMeasurement::Id,
                                    CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id, &attr);
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

static time_t rtc_utc_to_epoch(const RtcDateTime& dt) {
  char tz_buf[64] = {};
  const char* tz_old = getenv("TZ");
  if (tz_old) strncpy(tz_buf, tz_old, sizeof(tz_buf) - 1);
  setenv("TZ", "UTC0", 1);
  tzset();
  struct tm t = {};
  t.tm_year = dt.year - 1900;
  t.tm_mon = dt.mon - 1;
  t.tm_mday = dt.day;
  t.tm_hour = dt.hour;
  t.tm_min = dt.min;
  t.tm_sec = dt.sec;
  time_t epoch = mktime(&t);
  if (tz_old) setenv("TZ", tz_buf, 1);
  else unsetenv("TZ");
  tzset();
  return epoch;
}

static ClockTime readClockTime() {
  static uint8_t lastSec = 0xFF;
  static ClockTime cached = {12, 0, false};
  RtcDateTime dt = {};
  if (!read_rtc_utc(&dt)) return cached;
  if (dt.sec == lastSec) return cached;
  lastSec = dt.sec;

  time_t epoch = rtc_utc_to_epoch(dt);
  struct tm local_tm = {};
  localtime_r(&epoch, &local_tm);
  cached.hour = (uint8_t)local_tm.tm_hour;
  cached.minute = (uint8_t)local_tm.tm_min;
  cached.pm = (cached.hour >= 12);
  return cached;
}

static uint8_t to_bcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static bool rtc_set_time_compile() {
  if (!rtcDev) return false;
  const char* date = __DATE__; // "Mmm dd yyyy"
  const char* time = __TIME__; // "hh:mm:ss"
  char mon_str[4] = {};
  int day = 0;
  int year = 0;
  int hour = 0, min = 0, sec = 0;
  sscanf(date, "%3s %d %d", mon_str, &day, &year);
  sscanf(time, "%d:%d:%d", &hour, &min, &sec);
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* p = strstr(months, mon_str);
  int mon = p ? ((int)(p - months) / 3) + 1 : 1;

  struct tm local_tm = {};
  local_tm.tm_year = year - 1900;
  local_tm.tm_mon = mon - 1;
  local_tm.tm_mday = day;
  local_tm.tm_hour = hour;
  local_tm.tm_min = min;
  local_tm.tm_sec = sec;
  time_t epoch = mktime(&local_tm);
  struct tm utc_tm = {};
  gmtime_r(&epoch, &utc_tm);

  uint8_t data[7] = {
      to_bcd((uint8_t)utc_tm.tm_sec),
      to_bcd((uint8_t)utc_tm.tm_min),
      to_bcd((uint8_t)utc_tm.tm_hour),
      0x01,
      to_bcd((uint8_t)utc_tm.tm_mday),
      to_bcd((uint8_t)(utc_tm.tm_mon + 1)),
      to_bcd((uint8_t)(utc_tm.tm_year - 100)),
  };
  if (!i2c_write_dev(rtcDev, 0x00, data, sizeof(data))) return false;

  uint8_t status = 0;
  if (i2c_read_dev(rtcDev, 0x0F, &status, 1)) {
    status &= ~0x80; // clear OSF
    i2c_write_dev(rtcDev, 0x0F, &status, 1);
  }
  return true;
}

static bool rtc_set_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec) {
  if (!rtcDev) return false;
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
  struct tm utc_tm = {};
  gmtime_r(&epoch, &utc_tm);
  return rtc_set_time((uint16_t)(utc_tm.tm_year + 1900),
                      (uint8_t)(utc_tm.tm_mon + 1),
                      (uint8_t)utc_tm.tm_mday,
                      (uint8_t)utc_tm.tm_hour,
                      (uint8_t)utc_tm.tm_min,
                      (uint8_t)utc_tm.tm_sec);
}

static bool system_time_valid() {
  time_t now = time(nullptr);
  return now >= RTC_VALID_EPOCH;
}

static bool sync_rtc_from_system_time(const char* source) {
  time_t now = time(nullptr);
  if (now < RTC_VALID_EPOCH) return false;
  bool ok = rtc_set_time_from_epoch(now);
  ESP_LOGI(TAG, "RTC sync from %s: %s", source, ok ? "OK" : "FAILED");
  return ok;
}

static void sntp_time_sync_cb(struct timeval* tv) {
  (void)tv;
  gSntpSynced = true;
  if (gSntpWaiter) xTaskNotifyGive(gSntpWaiter);
}

static bool try_matter_time_sync() {
  uint32_t start = millis_ms();
  while (millis_ms() - start < RTC_SYNC_MATTER_TIMEOUT_MS) {
    if (system_time_valid()) return sync_rtc_from_system_time("Matter/system");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  return false;
}

static bool try_sntp_time_sync() {
  gSntpWaiter = xTaskGetCurrentTaskHandle();
  gSntpSynced = false;
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_set_time_sync_notification_cb(sntp_time_sync_cb);
  esp_sntp_init();

  uint32_t start = millis_ms();
  while (!gSntpSynced && millis_ms() - start < RTC_SYNC_SNTP_TIMEOUT_MS) {
    if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
      gSntpSynced = true;
      break;
    }
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
  }

  esp_sntp_stop();
  gSntpWaiter = nullptr;
  if (!gSntpSynced) return false;
  return sync_rtc_from_system_time("SNTP");
}

static void rtc_time_sync_task(void*) {
  vTaskDelay(pdMS_TO_TICKS(60000)); // allow Matter to initialize
  while (true) {
    bool ok = try_matter_time_sync();
    if (!ok) {
      ESP_LOGW(TAG, "Matter time sync unavailable; falling back to SNTP");
      try_sntp_time_sync();
    }
    vTaskDelay(pdMS_TO_TICKS(RTC_SYNC_INTERVAL_MS));
  }
}

static void button_task(void*) {
  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_pullup_en(BUTTON_PIN);

  const TickType_t poll = pdMS_TO_TICKS(50);
  uint32_t heldMs = 0;
  bool wasPressed = false;
  while (true) {
    bool pressed = gpio_get_level(BUTTON_PIN) == 0;
    if (pressed) {
      if (!wasPressed) {
        heldMs = 0;
        wasPressed = true;
      }
      heldMs += 50;
      if (heldMs >= BUTTON_DECOMMISSION_MS) {
        ESP_LOGW(TAG, "BOOT long-press detected; decommissioning");
#if ESP_CLOCK_HAS_MATTER
        esp_matter::factory_reset();
#else
        esp_restart();
#endif
        while (gpio_get_level(BUTTON_PIN) == 0) vTaskDelay(poll);
        heldMs = 0;
        wasPressed = false;
      }
    } else {
      heldMs = 0;
      wasPressed = false;
    }
    vTaskDelay(poll);
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

static void handleDisplaySerialCommands() {
  static char buf[64];
  static size_t idx = 0;
  uint8_t ch;
#if ESP_CLOCK_HAS_USB_SERIAL_JTAG
  while (usb_serial_jtag_read_bytes(&ch, 1, 0) == 1) {
#else
  while (uart_read_bytes(UART_NUM_0, &ch, 1, 0) == 1) {
#endif
    if (ch == '\n' || ch == '\r') {
      buf[idx] = '\0';
      if (idx > 0) {
        if (strncmp(buf, "loginfo on", 10) == 0) {
          esp_log_level_set(TAG, ESP_LOG_INFO);
          printf("esp_clock log level: INFO\n");
        } else if (strncmp(buf, "loginfo off", 11) == 0) {
          esp_log_level_set(TAG, ESP_LOG_ERROR);
          printf("esp_clock log level: ERROR\n");
#if ESP_CLOCK_HAS_MATTER && ESP_CLOCK_VERBOSE_LOGS
        } else if (strncmp(buf, "chiplog on", 10) == 0) {
          gChipLogRedirectEnabled = true;
          chip::Logging::SetLogRedirectCallback(chipLogRedirect);
          printf("chip log redirect: ON\n");
        } else if (strncmp(buf, "chiplog off", 11) == 0) {
          gChipLogRedirectEnabled = false;
          chip::Logging::SetLogRedirectCallback(nullptr);
          printf("chip log redirect: OFF\n");
#elif ESP_CLOCK_HAS_MATTER
        } else if (strncmp(buf, "chiplog ", 8) == 0) {
          printf("chip log redirect unavailable (ESP_CLOCK_VERBOSE_LOGS=0)\n");
#endif
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
          int y, mo, d, h, mi, s;
          if (sscanf(buf + 4, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) == 6) {
            bool ok = rtc_set_time((uint16_t)y, (uint8_t)mo, (uint8_t)d,
                                   (uint8_t)h, (uint8_t)mi, (uint8_t)s);
            ESP_LOGI(TAG, "RTC set %s", ok ? "OK" : "FAILED");
          } else {
            ESP_LOGW(TAG, "RTC set format (UTC): rtc YYYY-MM-DD HH:MM:SS");
          }
        } else if (strcmp(buf, "rtc") == 0) {
          RtcDateTime dt = {};
          if (read_rtc_utc(&dt)) {
            ESP_LOGI(TAG, "RTC UTC %04u-%02u-%02u %02u:%02u:%02u",
                     dt.year, dt.mon, dt.day, dt.hour, dt.min, dt.sec);
          } else {
            ESP_LOGW(TAG, "RTC read failed");
          }
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
}

static void init_bsec() {
  env.begin(BME68X_I2C_INTF, i2c_read_bytes, i2c_write_bytes, delay_us, nullptr, millis_ms);
  loadBsecState();
  env.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  env.attachCallback(onBsecOutputs);
  env.setTemperatureOffset(TEMP_OFFSET_C);
}

static void init_veml7700() {
  if (!vemlDev) return;
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
  if (als <= min) return (uint16_t)(TLC_MAX * 0.15f);
  if (als >= max) return TLC_MAX;
  float t = (float)(als - min) / (float)(max - min);
  float level = 0.15f + t * (1.0f - 0.15f);
  return (uint16_t)(level * TLC_MAX);
}

static void sensor_task(void*) {
  uint32_t lastLuxMs = 0;
  uint32_t lastLogMs = 0;
  bool threadConfigured = false;
  while (true) {
    if (!env.run()) {
      ESP_LOGW(TAG, "BSEC run returned false");
    }
    if (!threadConfigured) {
#if ESP_CLOCK_HAS_MATTER && ESP_CLOCK_HAS_OPENTHREAD
      if (esp_matter::is_started()) {
        configureThreadRouterEligibility();
        threadConfigured = true;
      }
#endif
    }
    matter_update();
    uint32_t now = millis_ms();
      if (now - lastLuxMs > 500) {
      if (!DISPLAY_RH_TEST_MODE && !gBrightnessOverride) {
        uint16_t als = read_veml7700();
        display.setBrightness(scale_brightness(als));
      }
      lastLuxMs = now;
    }
    if (DISPLAY_RH_TEST_MODE) updateDisplayRhTestPattern();
    else if (DISPLAY_TEST_MODE) updateDisplayTestPattern();
    else updateDisplayFromState();
    saveBsecStateIfReady(millis_ms());
    handleDisplaySerialCommands();

    if (now - lastLogMs > 5000) {
      ClockTime ct = readClockTime();
      int h12 = ct.hour % 12;
      if (h12 == 0) h12 = 12;
      ESP_LOGI(TAG, "RTC %02u:%02u (%s) | Display %02d:%02d | Temp %.1f F | RH %.1f%%",
               ct.hour, ct.minute, ct.pm ? "PM" : "AM",
               h12, ct.minute,
               (double)(vTempC * 9.0f / 5.0f + 32.0f),
               (double)vHum);
      lastLogMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

extern "C" void app_main() {
  printf("esp_clock: printf path OK\n");
  esp_err_t evt_err = esp_event_loop_create_default();
  if (evt_err != ESP_OK && evt_err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "event loop create failed: %s", esp_err_to_name(evt_err));
  }
  ESP_ERROR_CHECK(nvs_flash_init());
#if ESP_CLOCK_VERBOSE_LOGS
  esp_log_level_set("*", ESP_LOG_INFO);
#else
  esp_log_level_set("*", ESP_LOG_ERROR);
#endif
  esp_log_level_set("esp_clock", ESP_LOG_INFO);
  ESP_LOGI(TAG, "esp_clock log path OK");
#if ESP_CLOCK_HAS_MATTER && ESP_CLOCK_VERBOSE_LOGS
  if (gChipLogRedirectEnabled) {
    chip::Logging::SetLogRedirectCallback(chipLogRedirect);
  }
#endif
#if ESP_CLOCK_HAS_OT_LAUNCHER
  init_openthread_nvs();
#endif
  uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
#if ESP_CLOCK_HAS_USB_SERIAL_JTAG
  usb_serial_jtag_driver_config_t usb_cfg = {};
  usb_cfg.rx_buffer_size = 1024;
  usb_cfg.tx_buffer_size = 1024;
  usb_serial_jtag_driver_install(&usb_cfg);
#endif
#if ESP_CLOCK_HAS_OT_LAUNCHER
  init_openthread_platform_config();
  ESP_ERROR_CHECK(esp_event_handler_register(OPENTHREAD_EVENT, ESP_EVENT_ANY_ID, &openthread_event_handler, nullptr));
  maybeClearThreadDataset();
  logThreadState("boot");
#endif

  init_i2c();
  init_bsec();
  init_veml7700();
  setenv("TZ", LOCAL_TZ, 1);
  tzset();

  uint8_t status = 0;
  if (i2c_read_dev(rtcDev, 0x0F, &status, 1)) {
    if (status & 0x80) {
      ESP_LOGW(TAG, "RTC lost power (OSF=1); initializing from compile time");
      rtc_set_time_compile();
    } else {
      ESP_LOGI(TAG, "RTC status OK (OSF=0)");
    }
  } else {
    ESP_LOGW(TAG, "RTC status read failed");
  }
  matter_init();

  display.init();

  ESP_LOGI(TAG, "display host=%s MOSI=%d SCK=%d LATCH=%d CA1=%d CA2=%d CA3=%d CA4=%d",
           display.hostName(), TLC_SPI_MOSI, TLC_SPI_SCLK, TLC_SPI_LATCH,
           TBD_PIN_CA1, TBD_PIN_CA2, TBD_PIN_CA3, TBD_PIN_CA4);
  ESP_LOGI(TAG, "display page period = %u us (frame %.1f Hz)",
           display.pagePeriodUs(),
           1000000.0f / (display.pagePeriodUs() * 4.0f));

  xTaskCreate(sensor_task, "sensor_task", 8192, nullptr, 5, nullptr);
  xTaskCreate(rtc_time_sync_task, "rtc_time_sync", 4096, nullptr, 4, nullptr);
  xTaskCreate(button_task, "boot_button", 2048, nullptr, 4, nullptr);
}
