# nrs_scan_cpp regularized reconstruction v4

기존 v3 파이프라인을 유지하면서 최종 STL 저장 전에
Blender의 **Recalculate Outside**에 해당하는 face winding 정리 단계를 추가합니다.

## 처리 순서

```text
각 view별 Z crop
→ merge
→ voxel
→ SOR
→ ROR
→ normal
→ GP3
→ smoothing
→ decimation
→ face adjacency 기반 winding 일관화
→ connected component별 outward orientation
→ STL 저장
```

## 설치

```bash
cd ~/nrs_ws/src/nrs_vision

unzip -o ~/Downloads/nrs_scan_regularized_node_v4_normals_fixed.zip

chmod +x \
  nrs_scan_regularized_node_v4/install_into_existing_workspace.sh

./nrs_scan_regularized_node_v4/install_into_existing_workspace.sh \
  ~/nrs_ws/src/nrs_vision/nrs_scan_cpp \
  ~/nrs_ws
```

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
source ~/nrs_ws/install/setup.bash
export ROS_DOMAIN_ID=0

SCAN_DIR=/home/eunseop/urp_data/sim_scan/tail_lamp_proxy_run2
CONFIG=~/nrs_ws/src/nrs_vision/nrs_scan_cpp/config/reconstruct_regularized.yaml

ros2 run nrs_scan_cpp reconstruct_regularized_node \
  --ros-args \
  --params-file "$CONFIG" \
  -p scan_dir:="$SCAN_DIR"
```

## 결과

- `reconstructed_gp3_regularized_unoriented.stl`
  - orientation 수정 전 비교용
- `reconstructed_gp3_regularized.stl`
  - orientation 수정 완료 최종본
  - 경로 생성 노드에는 이 파일을 사용

로그에는 다음 진단이 출력됩니다.

```text
Mesh orientation fix:
components=...
local_face_flips=...
global_component_flips=...
boundary_edges=...
nonmanifold_edges=...
winding_conflicts=...
```
