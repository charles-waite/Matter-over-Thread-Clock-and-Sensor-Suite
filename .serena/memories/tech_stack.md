# Tech Stack

- Primary language/backend: C++ (`cpp` in Serena), ESP-IDF project using CMake/idf.py.
- Target chip/default: `esp32c6` from `sdkconfig.defaults` (`CONFIG_IDF_TARGET="esp32c6"`).
- ESP-IDF app project: top-level `CMakeLists.txt` sets `EXTRA_COMPONENT_DIRS components`, includes `$IDF_PATH/tools/cmake/project.cmake`, project name `esp_clock`.
- Main component: `main/CMakeLists.txt` registers `main.cpp`, includes `.`, `REQUIRES bsec2 esp_matter matter_time_sync`, `PRIV_REQUIRES esp_timer driver nvs_flash openthread`.
- Matter dependency: `main/idf_component.yml` depends on `espressif/esp_matter: '*'`; lock state in `dependencies.lock`.
- Local components: BSEC2/BME68x sensor libraries and `components/matter_time_sync`.
- Config defaults live in `sdkconfig.defaults`; generated `sdkconfig` is not hand-edited.
- Custom partition table: `partitions.csv`; notable partitions include encrypted `esp_secure_cert`, app at `0x20000` size `0x3D0000`, OpenThread NVS `ot` at `0x3F0000` size `0x6000`.
- OpenThread/Thread-only device defaults: OpenThread enabled, SRP/DNS client enabled, Wi-Fi station/AP disabled, RX-on-when-idle enabled, custom IPv6/address settings in `sdkconfig.defaults`.
- Build wrapper `./tools/idf.sh` is canonical: bash shebang, `set -euo pipefail`, enables ccache via `IDF_CCACHE_ENABLE=1`, sets `CCACHE_BASEDIR` to project root, sources `$HOME/esp-idf/export.sh` if `idf.py` is not already available.
- Project scripts are bash scripts and executable: `tools/idf.sh`, `tools/set_rtc_from_host.sh`.