# Matter Over Thread Clock and Sensor Suite

ESP32-C6 clock and environmental sensor suite for a multiplexed 7-segment display. The device runs Matter over Thread, reads local environmental sensors, keeps time with an external DS3231 RTC, and exposes measurements to Matter controllers.

## Current Project State

Working in the current firmware:

- Display driver and multiplexing are stable on the TLC5947 + TBD62783APG display chain.
- DS3231 RTC is the primary runtime clock source.
- Pacific local time display works, including DST via `PST8PDT,M3.2.0/2,M11.1.0/2`.
- Matter commissioning and Thread operation are supported.
- Temperature and relative humidity publish to Matter.
- Air Quality now publishes from BSEC static IAQ with accuracy gating.
- CO2 concentration publishes independently from BSEC eCO2.
- VEML7700 automatic brightness control is enabled by default.
- Serial command interface is available for display, RTC, time-sync, diagnostics, and telemetry.
- Reset history, heap diagnostics, and periodic health logging are available for soak testing.
- Local Thread/IPv6 NTP time correction is implemented and validated in the sync pipeline.

Current hardware target:

- Seeed XIAO ESP32-C6.
- Display: TLC5947 24-channel cathode sink over SPI plus TBD62783APG common-anode drivers.
- Sensors: BME680 with BSEC2, VEML7700 ambient light, DS3231 RTC.

## Timekeeping Model

The DS3231 RTC is the product source of truth. Normal display/runtime code reads from the RTC, not from build time or an unvalidated system clock.

Network time is correction-only:

- Runtime sync is enabled by default with `DISABLE_TIME_SYNC=0`.
- Sync only runs after Matter commissioning and Thread attachment.
- A one-shot initial sync is scheduled after commissioning or Thread reconnect, with a 60 second grace period.
- Periodic drift checks run on `RTC_SYNC_INTERVAL_MS` (currently 1 hour).
- RTC writes happen only inside the guarded NTP/Matter sync transaction.
- RTC writes require a valid network time candidate and either an unreadable RTC or drift greater than `RTC_MAX_DRIFT_SEC` (currently 2 seconds).
- No compile-time fallback time should be written to the RTC or system clock.

Sync source order:

1. Thread IPv6 NTP server, default `fd7f:4975:d3c5:4f0a:bc3e:5442:1e1a:1911`.
2. Optional IPv4 NTP fallback, default `192.168.1.29`, compiled only when `ENABLE_NTP_IPV4_FALLBACK=1`.
3. Matter Time Sync fallback through `time_sync_manager` if NTP does not return a valid sample.

The NTP path uses ESP-IDF SNTP. On successful fetch, firmware records machine-readable `SYNC_METRIC` lines with NTP epoch/usec, system clock sample, RTC epoch, RTC drift, system drift, and RTC write phase. When a write is needed, the system clock is first staged from the NTP sample, then the RTC write is aligned near a second boundary. The DS3231 is still second-resolution, so post-sync success is reported as "within 1 second variance" rather than pretending sub-second RTC readback exists.

The serial status snapshot also reports the last NTP/Matter comparison age, timestamp, and drift when available.

## Matter Behavior

Matter endpoints currently include:

- Temperature Measurement.
- Relative Humidity Measurement.
- Air Quality Sensor endpoint.
- Carbon Dioxide Concentration Measurement on the air endpoint.

Air Quality enum behavior:

- Uses BSEC `BSEC_OUTPUT_STATIC_IAQ`.
- Accuracy `< 1` or invalid IAQ reports `Unknown`.
- IAQ thresholds map to Matter AirQuality enum:
  - `<= 50`: Good
  - `<= 100`: Fair
  - `<= 150`: Moderate
  - `<= 200`: Poor
  - `<= 300`: VeryPoor
  - `> 300`: ExtremelyPoor

CO2 reporting is independent and uses BSEC `BSEC_OUTPUT_CO2_EQUIVALENT`.

If Matter endpoint schema, device types, descriptors, or cluster layout change, controllers may need a re-interview. Worst case, decommission and recommission the device.

## Runtime Cadence

Current defaults in `main/main.cpp`:

- BSEC2 runs in LP mode, nominal 3 second cadence.
- Matter sensor attributes update every 15 seconds.
- VEML7700 ambient light reads every 500 ms unless fixed PWM override is active.
- Display refresh is timer-driven; `DISPLAY_PAGE_PERIOD_US=2500` gives about 100 Hz frame rate.
- Clock/health status snapshot logs every 5 minutes.
- Verbose time state logs, when `logtime on`, are limited to about every 30 seconds.
- UDP telemetry emits on sync events and periodic 5 minute snapshots.

## Serial Commands

Use `help` on the serial console for the firmware's current command list.

Runtime diagnostics:

- `help`: print command list.
- `loginfo on|off`: set `esp_clock` log verbosity at runtime.
- `logtime on|off`: enable verbose Matter/NTP time-sync soak logs.
- `logheap on|off|reset`: control per-section heap diagnostics.
- `rebootcause`: print previous/current reset reason.
- `reboothistory`: print stored reset history ring.

RTC and time sync:

- `rtc`: print RTC UTC time.
- `rtc YYYY-MM-DD HH:MM:SS`: set RTC UTC time.
- `rtc +/-SECONDS`: adjust RTC by signed seconds.
- `timesync`: show sync state and interval.
- `timesync now`: run immediate NTP/Matter-to-RTC sync flow.
- `timesync interval <seconds>`: set runtime periodic sync interval.
- `timesync interval default`: restore default periodic sync interval.
- `ntptest`: compare NTP vs RTC without writing RTC.
- `ntptest sync`: compare and sync RTC if drift exceeds threshold.
- `ntpserver`: show configured NTP servers.
- `ntpserver v6 <host|ip>` / `ntpserver v4 <host|ip>`: set fallback server.
- `ntpserver default`: restore default NTP servers.

Display:

- `pwm <1-4095>`: set fixed display PWM brightness and disable ALS brightness updates.
- `pwm auto`: re-enable automatic brightness control.
- `refresh <microseconds>`: set display multiplex page period.
- `display`: print display SPI host and pin mapping.

UDP telemetry:

- `telemetry udp`: show UDP telemetry config/state.
- `telemetry udp host <ipv6|host>`: set destination host.
- `telemetry udp port <1..65535>`: set destination port.
- `telemetry udp on|off`: enable/disable UDP telemetry.
- `telemetry udp default`: restore default telemetry config.

## Build and Flash

Use the project wrapper so ESP-IDF is sourced when needed and ccache is enabled:

```bash
cd "/Users/cwaite/Documents/Coding Projects/ESP-Clock"
./tools/idf.sh set-target esp32c6
./tools/idf.sh build
./tools/idf.sh -p <PORT> flash monitor
```

The target is `esp32c6` from `sdkconfig.defaults`. Do not hand-edit generated `sdkconfig`; change `sdkconfig.defaults` for default config changes.

## Key Tunables

These live near the top of `main/main.cpp`.

- `DISABLE_TIME_SYNC` default `0`: set to `1` to disable all Matter/network time correction.
- `LOGTIME_DEFAULT` default `0`: compile-time default for verbose time-sync logging.
- `LOGHEAP_DEFAULT` default `0`: compile-time default for heap diagnostics.
- `ENABLE_NTP_IPV4_FALLBACK` default `0`: include IPv4 NTP fallback attempts when enabled.
- `LOCAL_TZ`: Pacific timezone/DST rule.
- `RTC_SYNC_INTERVAL_MS` default 1 hour.
- `RTC_SYSTEM_DRIFT_CHECK_MS` default 15 seconds.
- `RTC_SYSTEM_DRIFT_TRIGGER_SEC` default 2 seconds.
- `RTC_VALID_EPOCH` default `1700000000`.
- `RTC_MAX_DRIFT_SEC` default 2 seconds.
- `RTC_SYNC_GRACE_MS` default 60 seconds.
- `NTP_DEFAULT_SERVER_IPV6` / `NTP_DEFAULT_SERVER_IPV4`: default time sources.
- `NTP_TEST_TIMEOUT_MS` default 10 seconds.
- `DISPLAY_PAGE_PERIOD_US` default 2500 us.
- `TLC_ON` default 1600.
- `RH_ON` default 4095.
- `DISPLAY_TEST_MODE` / `DISPLAY_RH_TEST_MODE` default `false`.

## Project Guardrails

- Do not modify display driver/task routines casually; display stability is a priority.
- Do not modify `managed_components/` directly.
- Use `sdkconfig.defaults`, `main/idf_component.yml`, and local components for project-level changes.
- Keep periodic paths allocation-light; avoid per-loop timezone mutation (`setenv`, `unsetenv`, `tzset`).
- Validate time/loop/logging changes with `logheap on` soak testing and `reboothistory`.

## Known Caveats

- The firmware is hardware-first; there is no automated test suite.
- Full validation requires the display board, sensors, a commissioned Thread/Matter fabric, and a reachable local Thread IPv6 NTP source.
- BSEC IAQ depends on sensor warm-up and accuracy; Air Quality remains `Unknown` until BSEC accuracy is at least 1.
- IPv4 NTP fallback is compiled out by default for Thread-first deployments.
