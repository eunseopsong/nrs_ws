#!/usr/bin/env bash
set -euo pipefail

SCAN_DIR="${1:?usage: check_outputs.sh <scan_dir>}"

find "$SCAN_DIR" -maxdepth 3 -type f \
  \( -name '*.png' -o -name '*.pcd' -o -name '*.stl' -o -name '*.txt' \) \
  -printf '%p\n' | sort

echo
printf 'Captured ROI PCD count: '
find "$SCAN_DIR/views" -name cloud_world_roi.pcd -type f | wc -l
