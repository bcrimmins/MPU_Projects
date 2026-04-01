#!/usr/bin/env bash
# Flash and monitor the ESP32-C6 Zigbee coordinator firmware.
#
# Usage:
#   ./flash.sh                  # auto-detect port, flash + monitor
#   ./flash.sh /dev/ttyUSB0     # specify port explicitly
#   ./flash.sh /dev/ttyUSB0 flash         # flash only
#   ./flash.sh /dev/ttyUSB0 monitor       # monitor only
#   ./flash.sh /dev/ttyUSB0 flash monitor # flash then monitor
#
# Prerequisites:
#   - ESP-IDF v5.1+ installed and sourced  (. $IDF_PATH/export.sh)
#   - ESP32-C6 connected via USB

set -euo pipefail

PORT="${1:-}"
shift || true
ACTIONS="${*:-flash monitor}"

if [[ -z "$PORT" ]]; then
    # Try to auto-detect
    for candidate in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
        if [[ -e "$candidate" ]]; then
            PORT="$candidate"
            echo "[flash.sh] Auto-detected port: $PORT"
            break
        fi
    done
fi

if [[ -z "$PORT" ]]; then
    echo "[flash.sh] ERROR: No serial port found. Plug in the ESP32-C6 or pass the port as the first argument."
    exit 1
fi

echo "[flash.sh] Target : ESP32-C6"
echo "[flash.sh] Port   : $PORT"
echo "[flash.sh] Actions: $ACTIONS"

# Build first if sdkconfig or build directory is missing
if [[ ! -f "build/esp32c6-zigbee-coordinator.bin" ]]; then
    echo "[flash.sh] Binary not found — building first..."
    idf.py set-target esp32c6
    idf.py build
fi

# shellcheck disable=SC2086
idf.py -p "$PORT" -b 460800 $ACTIONS
