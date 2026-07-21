#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="${1:-$HOME/nrs_ws/src/nrs_vision/nrs_scan_cpp}"
WS_DIR="${2:-$HOME/nrs_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "$PKG_DIR" ]]; then
  echo "[ERROR] package directory not found: $PKG_DIR" >&2
  exit 1
fi

mkdir -p "$PKG_DIR/src" "$PKG_DIR/config"

STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ -f "$PKG_DIR/src/reconstruct_regularized_node.cpp" ]]; then
  cp "$PKG_DIR/src/reconstruct_regularized_node.cpp" \
     "$PKG_DIR/src/reconstruct_regularized_node.cpp.backup_$STAMP"
fi

if [[ -f "$PKG_DIR/config/reconstruct_regularized.yaml" ]]; then
  cp "$PKG_DIR/config/reconstruct_regularized.yaml" \
     "$PKG_DIR/config/reconstruct_regularized.yaml.backup_$STAMP"
fi

cp "$HERE/reconstruct_regularized_node.cpp" \
   "$PKG_DIR/src/reconstruct_regularized_node.cpp"

cp "$HERE/reconstruct_regularized.yaml" \
   "$PKG_DIR/config/reconstruct_regularized.yaml"

echo "[INFO] Replaced source and YAML."
echo "[INFO] Building from workspace: $WS_DIR"

source /opt/ros/humble/setup.bash
if [[ -f "$HOME/dev_ws/install/setup.bash" ]]; then
  source "$HOME/dev_ws/install/setup.bash"
fi

cd "$WS_DIR"
colcon build \
  --symlink-install \
  --packages-select nrs_scan_cpp

echo
echo "[OK] Build completed."
echo "Run:"
echo "  source $WS_DIR/install/setup.bash"
echo "  CONFIG=$PKG_DIR/config/reconstruct_regularized.yaml"
echo "  ros2 run nrs_scan_cpp reconstruct_regularized_node \\"
echo "    --ros-args --params-file \"\$CONFIG\" -p scan_dir:=<SCAN_DIR>"
