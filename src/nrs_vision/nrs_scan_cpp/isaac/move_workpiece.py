import omni.usd
from pxr import Gf, Usd, UsdGeom

# ---------------------------------------------------------
# 사용자가 수정할 값
# ---------------------------------------------------------
PRIM_PATH = "/World/workpiece_8"

# 새 workpiece 중심 위치 [m]
# Z는 기존 높이를 그대로 유지한다.
TARGET_CENTER_X_M = 0.650
TARGET_CENTER_Y_M = 0.300


def get_world_bbox(stage, prim):
    # Transform 수정 후 매번 새 BBoxCache를 만들어야 갱신된 값이 계산된다.
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
        ],
        useExtentsHint=True,
    )

    aligned_range = cache.ComputeWorldBound(prim).ComputeAlignedRange()

    minimum = aligned_range.GetMin()
    maximum = aligned_range.GetMax()
    center = (minimum + maximum) * 0.5
    size = maximum - minimum

    return minimum, maximum, center, size


stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError("현재 열린 USD Stage가 없습니다.")

prim = stage.GetPrimAtPath(PRIM_PATH)

if not prim.IsValid():
    raise RuntimeError(f"Workpiece prim을 찾지 못했습니다: {PRIM_PATH}")

old_minimum, old_maximum, old_center, old_size = get_world_bbox(
    stage,
    prim,
)

print("==================================================")
print("[BEFORE]")
print("prim path =", PRIM_PATH)
print("center    =", [float(value) for value in old_center])
print("size      =", [float(value) for value in old_size])
print("minimum   =", [float(value) for value in old_minimum])
print("maximum   =", [float(value) for value in old_maximum])

# 기존 Z 중심은 유지한다.
desired_center = Gf.Vec3d(
    TARGET_CENTER_X_M,
    TARGET_CENTER_Y_M,
    float(old_center[2]),
)

delta_world = desired_center - Gf.Vec3d(old_center)

print("\ndesired center =", [float(value) for value in desired_center])
print("world delta    =", [float(value) for value in delta_world])

xformable = UsdGeom.Xformable(prim)

translate_op = None

for operation in xformable.GetOrderedXformOps():
    if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        translate_op = operation
        break

if translate_op is None:
    translate_op = xformable.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble,
        "nrs_move",
    )
    current_translation = Gf.Vec3d(0.0, 0.0, 0.0)
else:
    current_value = translate_op.Get()

    if current_value is None:
        current_translation = Gf.Vec3d(0.0, 0.0, 0.0)
    else:
        current_translation = Gf.Vec3d(
            float(current_value[0]),
            float(current_value[1]),
            float(current_value[2]),
        )

new_translation = current_translation + delta_world
translate_op.Set(new_translation)

new_minimum, new_maximum, new_center, new_size = get_world_bbox(
    stage,
    prim,
)

print("\n[AFTER]")
print("translate =", [float(value) for value in new_translation])
print("center    =", [float(value) for value in new_center])
print("size      =", [float(value) for value in new_size])
print("minimum   =", [float(value) for value in new_minimum])
print("maximum   =", [float(value) for value in new_maximum])
print("==================================================")
print("[주의] 화면에서 위치를 확인한 뒤에만 USD를 저장하세요.")
