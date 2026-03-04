#!/usr/bin/env bash
set -euo pipefail

# Send current host UTC time to the ESP-Clock RTC command parser.
# Tries the two expected modem ports in order:
#   /dev/cu.usbmodem101
#   /dev/cu.usbmodem1101
#
# Usage:
#   tools/set_rtc_from_host.sh
#   tools/set_rtc_from_host.sh --check
#
# Notes:
# - Close `idf.py monitor` first, or writes may fail because the port is busy.

CHECK=0
if [[ "${1:-}" == "--check" ]]; then
  CHECK=1
fi

ts_utc="$(date -u '+%Y-%m-%d %H:%M:%S')"
cmd_set="rtc ${ts_utc}\r\n"
cmd_check="rtc\r\n"

ports=(
  "/dev/cu.usbmodem101"
  "/dev/cu.usbmodem1101"
)

present=0
for port in "${ports[@]}"; do
  if [[ -e "${port}" ]]; then
    present=1
    break
  fi
done

if [[ ${present} -eq 0 ]]; then
  echo "No expected modem ports found (/dev/cu.usbmodem101 or /dev/cu.usbmodem1101)." >&2
  exit 1
fi

ok_count=0
for port in "${ports[@]}"; do
  if [[ ! -e "${port}" ]]; then
    echo "Skipped ${port} (not present)."
    continue
  fi
  if printf '%b' "${cmd_set}" > "${port}" 2>/dev/null; then
    echo "Set RTC via ${port} -> ${ts_utc} UTC"
    ((ok_count += 1))
    if [[ ${CHECK} -eq 1 ]]; then
      printf '%b' "${cmd_check}" > "${port}" 2>/dev/null || true
      echo "Requested RTC readback on ${port}"
    fi
  else
    echo "Skipped ${port} (not writable/busy)."
  fi
done

if [[ ${ok_count} -eq 0 ]]; then
  echo "Failed to write RTC command to any candidate port." >&2
  exit 2
fi
