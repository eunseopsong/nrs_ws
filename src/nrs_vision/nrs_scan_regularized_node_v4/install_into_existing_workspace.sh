#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="${1:-$HOME/nrs_ws/src/nrs_vision/nrs_scan_cpp}"
WS_DIR="${2:-$HOME/nrs_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -d "$PKG_DIR" ]]; then
  echo "[ERROR] package directory not found: $PKG_DIR" >&2
  exit 1
fi

mkdir -p "$PKG_DIR/src" "$PKG_DIR/config"

for rel in \
  src/reconstruct_regularized_node.cpp \
  config/reconstruct_regularized.yaml
do
  if [[ -f "$PKG_DIR/$rel" ]]; then
    cp "$PKG_DIR/$rel" "$PKG_DIR/$rel.backup_$STAMP"
  fi
done

cp "$HERE/reconstruct_regularized_node.cpp" \
   "$PKG_DIR/src/reconstruct_regularized_node.cpp"
cp "$HERE/reconstruct_regularized.yaml" \
   "$PKG_DIR/config/reconstruct_regularized.yaml"

source /opt/ros/humble/setup.bash

if [[ -f "$HOME/dev_ws/install/setup.bash" ]]; then
  source "$HOME/dev_ws/install/setup.bash"
fi

cd "$WS_DIR"

colcon build \
  --symlink-install \
  --packages-select nrs_scan_cpp

echo
echo "[OK] v4 build completed."
echo "source $WS_DIR/install/setup.bash"
