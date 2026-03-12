#!/bin/bash
# Installs udev rules for OpenDubs hardware devices.
# Creates stable /dev symlinks:
#   /dev/rplidar0, /dev/rplidar1  -> SLLIDAR A1 (CP2102 USB-UART)
#   /dev/pixhawk  -> Pixhawk / ArduPilot FCU

set -e

RULES_FILE="opendubs.rules"
RULES_DEST="/etc/udev/rules.d/99-opendubs.rules"

# Locate the scripts/ directory relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f "$SCRIPT_DIR/$RULES_FILE" ]; then
    echo "[ERROR] Rules file not found: $SCRIPT_DIR/$RULES_FILE"
    exit 1
fi

echo "Installing $RULES_FILE to $RULES_DEST ..."
sudo cp "$SCRIPT_DIR/$RULES_FILE" "$RULES_DEST"

echo "Reloading udev rules ..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Done! Replug your devices and verify with:"
echo "  ls -l /dev/rplidar* /dev/pixhawk"
