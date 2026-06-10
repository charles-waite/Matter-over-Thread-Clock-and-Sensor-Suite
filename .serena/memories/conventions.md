# Conventions

- C/C++ style: 2-space indentation, K&R braces, `camelCase` functions/variables, `SCREAMING_SNAKE_CASE` macros/constants.
- Keep edits narrowly scoped; avoid display driver/task changes unless explicitly requested. Display stability is a high-priority invariant.
- Prefer ESP-IDF drivers/APIs for I2C/GPIO/PWM; use Arduino APIs only when required by a component.
- Do not edit `managed_components/`; do not hand-edit generated `sdkconfig`; change `sdkconfig.defaults` for stable config defaults.
- Treat `partitions.csv`, `main/idf_component.yml`, OpenThread/Wi-Fi defaults in `sdkconfig.defaults`, and display pin/timing constants as high-impact; change only when requested or clearly required.
- Periodic/hot paths (`<=1s`) should be allocation-free where practical. Avoid `setenv()`, `unsetenv()`, `tzset()` in loops/tasks; configure timezone once at boot.
- Avoid float `%f` formatting in frequent logs; prefer fixed-point integer formatting in hot paths.
- RTC/local time conversion should use direct UTC epoch conversion logic, not process TZ mutation in loops.
- Matter/network/system time may correct RTC only after validity/drift checks; system clock must not become an unvalidated RTC source.
- When changing Matter endpoint schema/cluster/device type behavior, expect controllers may require re-interview or decommission/recommission to observe descriptor changes.
- Legacy sketch folder names must match `.ino` names when touching legacy code.