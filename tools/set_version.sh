#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

semver_to_number() {
    local version="$1"
    if [[ ! "${version}" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
        echo "Version must be X.Y.Z, got '${version}'" >&2
        return 2
    fi
    echo $((BASH_REMATCH[1] * 10000 + BASH_REMATCH[2] * 100 + BASH_REMATCH[3]))
}

current_version() {
    sed -n 's/^set(PROJECT_VER "\([^"]*\)")$/\1/p' "${ROOT_DIR}/CMakeLists.txt" | head -n 1
}

current_version_number() {
    sed -n 's/^set(PROJECT_VER_NUMBER \([0-9][0-9]*\))$/\1/p' "${ROOT_DIR}/CMakeLists.txt" | head -n 1
}

current_sdkconfig_version_number() {
    sed -n 's/^CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER=\([0-9][0-9]*\)$/\1/p' "${ROOT_DIR}/sdkconfig.defaults" | head -n 1
}

replace_line() {
    local file="$1"
    local pattern="$2"
    local replacement="$3"

    if ! grep -Eq "${pattern}" "${file}"; then
        echo "Missing expected version field in ${file}" >&2
        return 3
    fi

    perl -0pi -e "s/${pattern}/${replacement}/gm" "${file}"
}

version="$(current_version)"
version_number="$(current_version_number)"
sdkconfig_version_number="$(current_sdkconfig_version_number)"
if [[ -z "${version}" || -z "${version_number}" || -z "${sdkconfig_version_number}" ]]; then
    echo "Could not determine current version fields from CMakeLists.txt and sdkconfig.defaults" >&2
    exit 3
fi

expected_number="$(semver_to_number "${version}")"
echo "Current version fields:"
echo "  PROJECT_VER=${version}"
echo "  PROJECT_VER_NUMBER=${version_number}"
echo "  CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER=${sdkconfig_version_number}"
if [[ "${expected_number}" != "${version_number}" ]]; then
    echo "WARNING: PROJECT_VER_NUMBER does not match PROJECT_VER semantic conversion." >&2
fi
if [[ "${expected_number}" != "${sdkconfig_version_number}" ]]; then
    echo "WARNING: CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER does not match PROJECT_VER semantic conversion." >&2
fi

if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [X.Y.Z]" >&2
    exit 2
fi

new_version="${1:-}"
if [[ -z "${new_version}" ]]; then
    read -r -p "Enter new version (X.Y.Z): " new_version
fi
new_number="$(semver_to_number "${new_version}")"

replace_line \
    "${ROOT_DIR}/CMakeLists.txt" \
    '^set\(PROJECT_VER "[^"]*"\)$' \
    "set(PROJECT_VER \"${new_version}\")"

replace_line \
    "${ROOT_DIR}/CMakeLists.txt" \
    '^set\(PROJECT_VER_NUMBER [0-9]+\)$' \
    "set(PROJECT_VER_NUMBER ${new_number})"

replace_line \
    "${ROOT_DIR}/main/chip_project_config.h" \
    '^#define CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING ".*"$' \
    "#define CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING \"${new_version}\""

replace_line \
    "${ROOT_DIR}/sdkconfig.defaults" \
    '^CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER=[0-9]+$' \
    "CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER=${new_number}"

echo "Updated version sources:"
echo "  PROJECT_VER=${new_version}"
echo "  PROJECT_VER_NUMBER=${new_number}"
echo "  CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING=${new_version}"
echo "  CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER=${new_number}"
echo "Run ./tools/idf.sh build to regenerate derived config/output values."
