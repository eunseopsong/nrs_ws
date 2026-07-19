"""Move the workpiece AABB center in Isaac Sim and print matching ROS parameters.

Run this file from Isaac Sim Script Editor while the stage is open:

    exec(open('/path/to/move_workpiece.py').read())

The script intentionally uses Isaac Sim/USD Python because changing a live USD
stage is an Isaac-specific operation; the ROS scan and reconstruction pipeline
is implemented in C++.
"""

import omni.usd
from pxr import Gf, Usd, UsdGeom

PRIM_PATH = "/World/workpiece_8"
DESIRED_CENTER_X_M = 0.550
DESIRED_CENTER_Y_M = 0.300
PRESERVE_CURRENT_CENTER_Z = True
DESIRED_CENTER_Z_M = 0.06545485

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No USD stage is open")

prim = stage.GetPrimAtPath(PRIM_PATH)
if not prim.IsValid():
    raise RuntimeError(f"Workpiece prim not found: {PRIM_PATH}")

bbox_cache = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(),
    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    useExtentsHint=True,
)


def aligned_world_range():
    return bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()


old_range = aligned_world_range()
old_min = old_range.GetMin()
old_max = old_range.GetMax()
old_center = (old_min + old_max) * 0.5
old_size = old_max - old_min

desired_center_z = (
    float(old_center[2]) if PRESERVE_CURRENT_CENTER_Z else DESIRED_CENTER_Z_M
)
desired_center = Gf.Vec3d(
    DESIRED_CENTER_X_M,
    DESIRED_CENTER_Y_M,
    desired_center_z,
)
delta = desired_center - Gf.Vec3d(old_center)

xformable = UsdGeom.Xformable(prim)
translate_op = None
for op in xformable.GetOrderedXformOps():
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        translate_op = op
        break

if translate_op is None:
    translate_op = xformable.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble,
        "nrs_workpiece_move",
    )
    old_translation = Gf.Vec3d(0.0, 0.0, 0.0)
else:
    old_translation_value = translate_op.Get()
    old_translation = Gf.Vec3d(old_translation_value)

new_translation = old_translation + delta
translate_op.Set(new_translation)

# Recreate the cache after changing the stage.
bbox_cache = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(),
    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    useExtentsHint=True,
)
new_range = aligned_world_range()
new_min = new_range.GetMin()
new_max = new_range.GetMax()
new_center = (new_min + new_max) * 0.5
new_size = new_max - new_min

print("========== WORKPIECE MOVED ==========")
print("prim_path:", PRIM_PATH)
print("old_center_world_m:", [float(v) for v in old_center])
print("old_size_m:", [float(v) for v in old_size])
print("delta_world_m:", [float(v) for v in delta])
print("new_translate_op_m:", [float(v) for v in new_translation])
print("new_center_world_m:", [float(v) for v in new_center])
print("new_size_m:", [float(v) for v in new_size])
print()
print("Copy these values into scan YAML:")
print(
    "workpiece_center_m: ["
    + ", ".join(f"{float(v):.9f}" for v in new_center)
    + "]"
)
print(
    "workpiece_size_m: ["
    + ", ".join(f"{float(v):.9f}" for v in new_size)
    + "]"
)
print("=====================================")
