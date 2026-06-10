# Core

- ESP32-C6 Matter-over-Thread clock + environmental sensor suite; main app is `main/main.cpp`.
- Top-level references: build/toolchain in `mem:tech_stack`; day-to-day commands in `mem:suggested_commands`; code style and safety invariants in `mem:conventions`; handoff checks in `mem:task_completion`.
- Source map: `main/` app entry/component, `components/bsec2/` Bosch BSEC2/BME680 support, `components/BME68x_Sensor_library/`, `components/matter_time_sync/`, `resources/` hardware logs/photos/mapping, `Legacy Arduino Sketches/` reference-only sketches.
- Do not modify `managed_components/`; use project config/manifests/defaults instead.
- Authoritative display mapping: `resources/ClockDisplayMapping/Master Mapping.csv`; verify any mux/mapping change against it.
- Target hardware: Seeed XIAO ESP32-C6; display chain TLC5947 cathode sink over SPI + TBD62783APG common-anode drivers multiplexing 10x 7-seg displays; sensors BME680/BSEC2, VEML7700, DS3231.
- DS3231 RTC is product source-of-truth. Matter/network/system time is correction-only and must not overwrite RTC unless candidate time is valid and drift policy allows it.
- No compile-time/build-time fallback time should be written to RTC/system clock.
- BOOT long-press ~5s triggers Matter decommission/factory reset.
- Reference patterns may be borrowed from `/Users/cwaite/Documents/Coding Projects/MiniSensor/ESP32-Matter-Environmental-Sensor`, but avoid bulk-porting true ESP-IDF patterns that could destabilize this Arduino/IDF-hybrid-adjacent project.