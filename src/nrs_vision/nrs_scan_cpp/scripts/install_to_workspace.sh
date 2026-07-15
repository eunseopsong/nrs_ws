#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_WS="${1:-$HOME/nrs_vision}"

mkdir -p "$TARGET_WS/src"
rm -rf "$TARGET_WS/src/nrs_scan_cpp"
cp -a "$PACKAGE_DIR" "$TARGET_WS/src/nrs_scan_cpp"


echo "Installed package to: $TARGET_WS/src/nrs_scan_cpp"
echo "Next: source the workspace that contains y2_rob_motion_interfaces, then build nrs_scan_cpp."
