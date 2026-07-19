#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_PARENT="${1:-$HOME/nrs_ws/src/nrs_vision}"
TARGET_DIR="$TARGET_PARENT/nrs_scan_cpp"

mkdir -p "$TARGET_PARENT"
if [[ -e "$TARGET_DIR" ]]; then
  BACKUP_DIR="${TARGET_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
  mv "$TARGET_DIR" "$BACKUP_DIR"
  echo "Existing package backed up to: $BACKUP_DIR"
fi
cp -a "$PACKAGE_DIR" "$TARGET_DIR"

echo "Installed package to: $TARGET_DIR"
echo "Build from ~/nrs_ws after sourcing /opt/ros/humble and ~/dev_ws/install/setup.bash"
