# Task Completion

- Minimum code-change verification: `./tools/idf.sh build` from project root.
- For build failures, report the first root-cause compiler/config error with exact `file:line` and failing command; avoid downstream-noise summaries.
- For display mapping/mux/timing changes: verify against `resources/ClockDisplayMapping/Master Mapping.csv` and hardware behavior (no flicker/uneven brightness, correct TLC/TBD paths).
- For I2C/sensor changes: hardware-check BME680/BSEC2, VEML7700, DS3231 detection/behavior as applicable.
- For Matter endpoint/schema/cluster changes: build plus hardware/controller validation; note if re-interview or recommissioning is required.
- For time/RTC/time-sync changes: verify DS3231 remains primary, no compile-time fallback writes, candidate network time is validity-gated, and RTC writes obey drift policy.
- For sensor task timing, RTC conversion, or frequent logging changes: soak with `logheap on` for at least 2x prior failure window (~60-90 min historically); confirm `Health`/`HeapDiag` trends do not monotonically decline and `reboothistory` has no new `PANIC` entries.
- No automated test suite is defined; hardware validation notes are expected when behavior touches display, sensors, Matter commissioning, Thread networking, or RTC/time sync.
- After memory updates, user can run `serena memories check` from the project root to validate memory references.