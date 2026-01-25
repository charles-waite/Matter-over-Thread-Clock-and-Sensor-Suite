# Matter Over Thread Clock and Sensor Suite

ESP32-C6 (Seeed XIAO) clock + environmental sensor suite with Matter over Thread, BME680/BSEC2, DS3231 RTC, VEML7700, and a TLC5947/TBD62783APG-muxed 7-seg display.

## Notes
- RTC is synced monthly using Matter/system time with SNTP-over-Thread fallback (pool.ntp.org).
- Display tuning defaults: `DISPLAY_PAGE_PERIOD_US=2500`, `TLC_ON=1600`, `RH_ON=4095` (see `main/main.cpp`).
- ALS brightness scaling is enabled by default; `pwm <x>` overrides it until `pwm auto`.
- Serial commands are documented in `AGENTS.md` (via USB-Serial/JTAG when available).

## Runtime Update Rates (Current Defaults)
The following are the current behavior in `main/main.cpp`, described in plain language:
- The BSEC2 engine runs in LP mode (nominal 3‑second cadence) and continuously updates IAQ/CO2 in the background.
- Matter attribute updates (temperature, humidity, IAQ, CO₂) are pushed every **15 seconds**.
- VEML7700 ambient‑light reads occur every **500 ms** (unless `pwm <x>` overrides ALS).
- Display refresh is timer‑driven; each page is held for `DISPLAY_PAGE_PERIOD_US` (default 2500 µs), giving a ~100 Hz frame rate.
- Status log lines print every **5 seconds**.

## Serial Commands (Quick Reference)
- `pwm <1..4095>` / `pwm auto`
- `refresh <us>` / `display`
- `rtc` / `rtc YYYY-MM-DD HH:MM:SS`
- `loginfo on|off`

## Build Flags & Tunables
- `DISPLAY_TEST_MODE`, `DISPLAY_RH_TEST_MODE`
- `DISPLAY_PAGE_PERIOD_US`, `TLC_ON`, `RH_ON`
- `RTC_SYNC_INTERVAL_MS`, `RTC_SYNC_MATTER_TIMEOUT_MS`, `RTC_SYNC_SNTP_TIMEOUT_MS`
- `BUTTON_DECOMMISSION_MS`, `BUTTON_PIN`

## Tunables (Defaults + Ranges)
These live near the top of `main/main.cpp`.

- `DISPLAY_PAGE_PERIOD_US` (default: 2500 µs)
  - Per‑page dwell time; frame rate = `1e6 / (4 * DISPLAY_PAGE_PERIOD_US)`.
  - Range: 200–5000 µs (enforced in `setPagePeriodUs`).
  - Reason: 2500 µs (~100 Hz frame) balances brightness and flicker.

- `TLC_ON` (default: 1600, range: 1–4095)
  - Base PWM level for all TLC channels except RH%.
  - Reason: tuned to match %RH brightness while keeping overall brightness acceptable.

- `RH_ON` (default: 4095, fixed)
  - PWM level for the %RH cluster (TLC channel 3).
  - Reason: RH% is a multi‑LED cluster and needs full scale.

- `TLC_MAX` (4095)
  - Absolute max PWM; used for scaling ALS and safety clamping.

- `RTC_SYNC_INTERVAL_MS` (default: 30 days)
  - How often RTC resync runs.

- `RTC_SYNC_MATTER_TIMEOUT_MS` (default: 60s)
  - How long we wait for Matter/system time before falling back.

- `RTC_SYNC_SNTP_TIMEOUT_MS` (default: 30s)
  - SNTP sync timeout during fallback.

- `RTC_VALID_EPOCH` (default: 1700000000)
  - System time must be after this to be considered valid.

- `DISPLAY_TEST_MODE` / `DISPLAY_RH_TEST_MODE` (default: false)
  - Build‑time test patterns for wiring/brightness validation.

- `BUTTON_PIN` / `BUTTON_DECOMMISSION_MS` (default: BOOT_PIN / 5000 ms)
  - Long‑press BOOT (5s) triggers Matter decommission (factory reset).

## TODO
- Persist BME680/BSEC2 calibration state to NVM periodically and restore on boot.
- Cache last temperature and humidity and restore on boot to avoid blank display.
- Fix IAQ endpoint reporting (currently logs `W (...) esp_clock: AirQuality write failed: 1` and no IAQ in AH/HA).
