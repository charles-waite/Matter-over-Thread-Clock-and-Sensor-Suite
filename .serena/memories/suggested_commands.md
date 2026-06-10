# Suggested Commands

- Build wrapper is preferred over raw `idf.py`:
  - `cd "/Users/cwaite/Documents/Coding Projects/ESP-Clock"`
  - `./tools/idf.sh set-target esp32c6` first-time or after target changes.
  - `./tools/idf.sh build` canonical verification build; ccache enabled by wrapper.
  - `./tools/idf.sh -p <PORT> flash monitor` canonical flash+monitor.
- RTC helper:
  - `./tools/set_rtc_from_host.sh` sends current host UTC time to the serial RTC command parser.
  - `./tools/set_rtc_from_host.sh --check` sends set then read/check command.
- ESP-Matter builds can take several minutes even with ccache; use long command timeouts.
- Useful serial commands documented in `AGENTS.md`/README: `rtc`, `timesync`, `timesync now`, `timesync interval <seconds|default>`, `pwm <1..4095>`, `pwm auto`, `refresh <us>`, `display`, `loginfo on|off`, `logheap on|off|reset`.
- Legacy Arduino sketches are reference-only; only use Arduino CLI/IDE when explicitly working under `Legacy Arduino Sketches/`.
- Darwin/macOS path contains spaces; quote absolute paths in shell commands: `"/Users/cwaite/Documents/Coding Projects/ESP-Clock"`.