# reconstruct_regularized_node v3

기존 `reconstruct_node`의 바닥 제거/필터 순서를 그대로 적용하고,
그 뒤에 GP3 smoothing 및 decimation을 수행합니다.

## 파이프라인

```text
각 view PCD 로드
→ 각 view별 Z PassThrough crop [crop_min_z_m, crop_max_z_m]
→ 병합
→ VoxelGrid
→ SOR
→ ROR
→ Normal
→ GP3
→ Windowed-Sinc smoothing
→ Quadric decimation
```

## 기존 워크스페이스에 설치

```bash
cd ~/nrs_ws/src/nrs_vision

unzip -o ~/Downloads/nrs_scan_regularized_node_v3_old_ground_logic.zip

chmod +x nrs_scan_regularized_node_v3/install_into_existing_workspace.sh

./nrs_scan_regularized_node_v3/install_into_existing_workspace.sh   ~/nrs_ws/src/nrs_vision/nrs_scan_cpp   ~/nrs_ws
```

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
source ~/nrs_ws/install/setup.bash
export ROS_DOMAIN_ID=0

SCAN_DIR=/home/eunseop/urp_data/sim_scan/tail_lamp_proxy_run2
CONFIG=~/nrs_ws/src/nrs_vision/nrs_scan_cpp/config/reconstruct_regularized.yaml

ros2 run nrs_scan_cpp reconstruct_regularized_node   --ros-args   --params-file "$CONFIG"   -p scan_dir:="$SCAN_DIR"
```

## 먼저 확인할 파일

```text
merged_z_cropped_regularized.pcd
merged_filtered_regularized.pcd
reconstructed_gp3_raw.stl
reconstructed_gp3_regularized.stl
```
