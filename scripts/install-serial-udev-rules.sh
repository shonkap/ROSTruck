#!/usr/bin/env bash
# Run this on whichever machine physically plugs into the FC over USB (dev
# workstation for bench testing, or the Jetson once it's wired up) so your
# user can read/write the FC's serial port without sudo, and get a stable
# /dev/fc0 symlink regardless of ACM numbering.
set -euo pipefail

RULES_SRC="$(dirname "$0")/udev/99-fc-serial.rules"
RULES_DST="/etc/udev/rules.d/99-fc-serial.rules"

sudo cp "$RULES_SRC" "$RULES_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed $RULES_DST."
echo "Unplug/replug the FC (and RPLidar) to pick up new permissions and the /dev/fc0, /dev/rplidar symlinks."
echo "You may also need: sudo usermod -aG dialout \$USER (then re-login)."
