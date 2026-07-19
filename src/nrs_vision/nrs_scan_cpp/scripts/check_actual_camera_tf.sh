#!/usr/bin/env bash
set -euo pipefail

WORLD_FRAME="${1:-world}"
CAMERA_FRAME="${2:-sim_camera}"

echo "=== PointCloud2 frame ==="
timeout 8s ros2 topic echo /camera --once --field header.frame_id || true

echo
echo "=== TF topics ==="
ros2 topic list | grep -E '^/tf$|^/tf_static$' || true

echo
echo "=== TF ${WORLD_FRAME} <- ${CAMERA_FRAME} (5 s) ==="
timeout 5s ros2 run tf2_ros tf2_echo "${WORLD_FRAME}" "${CAMERA_FRAME}" || true
