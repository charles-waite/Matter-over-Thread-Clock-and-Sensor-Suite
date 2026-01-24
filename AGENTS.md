# Repository Guidelines

## Project Structure & Module Organization
- `main/` contains the ESP-IDF application (current entry in `main/main.cpp`).
- `components/bsec2/` is the Bosch BSEC2 library for the BME680 sensor.
- `resources/` holds board photos and LED mapping data; use `resources/ClockDisplayMapping/Master Mapping.csv` for the authoritative map.
- `Legacy Arduino Sketches/` contains standalone sketches; each folder must match its `.ino` file name.
- Do not modify anything under `managed_components/`. Use `sdkconfig`/`sdkconfig.defaults` for feature toggles instead.
- `sdkconfig` is auto-generated; only edit `sdkconfig.defaults`.

## Build, Test, and Development Commands
- Primary toolchain is ESP-IDF (Matter over Thread, with Arduino-ESP32 components as needed).
- Typical flow:
  - `idf.py set-target esp32c6`
  - `idf.py build`
  - `idf.py -p <port> flash monitor`
- On this machine, ESP-Matter builds can take up to ~5 minutes; set build timeouts accordingly.
- Legacy sketches are reference-only; if needed, build via Arduino IDE or:
  - `arduino-cli compile --fqbn <board_fqbn> "Legacy Arduino Sketches/Clock_StableDisplayandTime"`
  - `arduino-cli upload --fqbn <board_fqbn> -p <port> "Legacy Arduino Sketches/Clock_StableDisplayandTime"`

## Coding Style & Naming Conventions
- Use 2-space indentation and K&R braces for C/C++.
- Prefer `camelCase` for functions/variables and `SCREAMING_SNAKE_CASE` for macros/constants.
- Keep sketch folder and `.ino` names identical when touching legacy code.

## Testing Guidelines
- No automated tests are defined.
- Validate on hardware: confirm mux stability (no flicker/uneven brightness), I2C sensors respond (BME680, VEML7700, DS3231), and TLC/TBD drive paths behave.
- If mapping or mux timing changes, verify against `resources/ClockDisplayMapping/Master Mapping.csv`.
- A display wiring test mode exists via `DISPLAY_TEST_MODE` in `main/main.cpp`; when enabled it cycles digits 0–9 and then lights all segments plus indicators.
- RH% test mode exists via `DISPLAY_RH_TEST_MODE` in `main/main.cpp`; it steps commons to verify the RH cluster wiring.
- Commissioning currently succeeds; IAQ endpoint is still not reporting correctly (open item).

## Hardware & Integration Notes
- Target board: Seeed XIAO ESP32-C6.
- Display chain: TLC5947 (24-channel cathode sink over SPI) + TBD62783APG (4 GPIO common-anode drivers) multiplexing 10x 7-segment displays.
- Sensors: BME680 (BSEC2), VEML7700 ambient light, DS3231 RTC.
- Prefer ESP-IDF drivers for I2C/GPIO/PWM; use Arduino APIs only when required by a component.
- RTC is synced monthly from Matter/system time with SNTP-over-Thread fallback (see `rtc_time_sync_task` in `main/main.cpp`).

## Serial Commands
- `refresh <us>`: set page period in microseconds.
- `display`: print current display pin mapping and SPI host.
- `rtc`: print RTC UTC time.
- `rtc YYYY-MM-DD HH:MM:SS`: set RTC UTC time.
- `pwm <1..4095>`: set global PWM brightness and lock ALS updates.
- `pwm auto`: re-enable ALS brightness control.
- `loginfo on|off`: toggle `esp_clock` INFO logs at runtime.

## Build Flags & Tunables
- `DISPLAY_TEST_MODE` and `DISPLAY_RH_TEST_MODE` in `main/main.cpp`.
- `DISPLAY_PAGE_PERIOD_US`, `TLC_ON`, and `RH_ON` set default frame timing and brightness.

## Reference Implementation (WALL-Env)
- Use `/Users/cwaite/Documents/Coding Projects/MiniSensor/ESP32-Matter-Environmental-Sensor` for ESP-IDF + Matter patterns (see `main/MainSensor.cpp`, `components/`, `sdkconfig.defaults`).
- Borrow patterns for BSEC2 state storage in NVS, Matter commissioning logs, and Thread router-eligibility configuration.

## Commit & Pull Request Guidelines
- This directory does not contain Git history, so no commit convention is documented.
- If you add Git, use short imperative subjects (e.g., “Fix display pulse timing”) and include a brief body for hardware-impacting changes.
- For PRs, include a summary, affected sketch path(s) or modules, and test evidence (compile logs or device notes).

## Security & Configuration Tips
- Keep Matter credentials, Wi-Fi credentials, and calibration state local or in untracked files.
- Avoid committing secrets or device identifiers to the repository.
