from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BARREL_RADIUS = 0.016
PLATE_THICKNESS = 0.010
MOUNT_PLATE_WIDTH = 0.070
MOUNT_PLATE_HEIGHT = 0.180
LEAF_THICKNESS = 0.008
LEAF_WIDTH = 0.160
LEAF_HEIGHT = 0.140
OUTER_FLANGE_WIDTH = 0.018
OUTER_FLANGE_DEPTH = 0.018
FIXED_KNUCKLE_LENGTH = 0.055
LEAF_KNUCKLE_LENGTH = 0.048
KNUCKLE_GAP = 0.004
BARREL_SUPPORT_HEIGHT = 0.050
BARREL_STACK_HEIGHT = (
    2.0 * FIXED_KNUCKLE_LENGTH + LEAF_KNUCKLE_LENGTH + 2.0 * KNUCKLE_GAP
)
TOP_KNUCKLE_Z = LEAF_KNUCKLE_LENGTH / 2.0 + KNUCKLE_GAP + FIXED_KNUCKLE_LENGTH / 2.0
BOTTOM_KNUCKLE_Z = -TOP_KNUCKLE_Z
LEAF_ROOT_X = 0.022
LEAF_BRIDGE_WIDTH = 0.012
LEAF_BRIDGE_CENTER_X = 0.018


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weld_on_gate_hinge")

    model.material("blackened_steel", rgba=(0.20, 0.20, 0.21, 1.0))
    model.material("zinc_steel", rgba=(0.63, 0.65, 0.68, 1.0))

    mount_plate = model.part("mount_plate")
    mount_plate.visual(
        Box((MOUNT_PLATE_WIDTH, PLATE_THICKNESS, MOUNT_PLATE_HEIGHT)),
        material="blackened_steel",
        name="mount_body",
        origin=Origin(xyz=(-(BARREL_RADIUS + MOUNT_PLATE_WIDTH / 2.0), 0.0, 0.0)),
    )
    mount_plate.visual(
        Box((BARREL_RADIUS * 1.20, PLATE_THICKNESS * 1.20, BARREL_SUPPORT_HEIGHT)),
        material="blackened_steel",
        name="top_support",
        origin=Origin(xyz=(-(BARREL_RADIUS * 0.60), 0.0, TOP_KNUCKLE_Z)),
    )
    mount_plate.visual(
        Box((BARREL_RADIUS * 1.20, PLATE_THICKNESS * 1.20, BARREL_SUPPORT_HEIGHT)),
        material="blackened_steel",
        name="bottom_support",
        origin=Origin(xyz=(-(BARREL_RADIUS * 0.60), 0.0, BOTTOM_KNUCKLE_Z)),
    )
    mount_plate.visual(
        Cylinder(radius=BARREL_RADIUS, length=FIXED_KNUCKLE_LENGTH),
        material="blackened_steel",
        name="top_knuckle",
        origin=Origin(xyz=(0.0, 0.0, TOP_KNUCKLE_Z)),
    )
    mount_plate.visual(
        Cylinder(radius=BARREL_RADIUS, length=FIXED_KNUCKLE_LENGTH),
        material="blackened_steel",
        name="bottom_knuckle",
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_KNUCKLE_Z)),
    )
    mount_plate.inertial = Inertial.from_geometry(
        Box((MOUNT_PLATE_WIDTH + 2.0 * BARREL_RADIUS, PLATE_THICKNESS * 1.2, MOUNT_PLATE_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(-MOUNT_PLATE_WIDTH / 2.0, 0.0, 0.0)),
    )

    swing_leaf = model.part("swing_leaf")
    swing_leaf.visual(
        Box((LEAF_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        material="zinc_steel",
        name="leaf_body",
        origin=Origin(xyz=(LEAF_ROOT_X + LEAF_WIDTH / 2.0, 0.0, 0.0)),
    )
    swing_leaf.visual(
        Box((LEAF_BRIDGE_WIDTH, LEAF_THICKNESS, LEAF_KNUCKLE_LENGTH)),
        material="zinc_steel",
        name="leaf_bridge",
        origin=Origin(xyz=(LEAF_BRIDGE_CENTER_X, 0.0, 0.0)),
    )
    swing_leaf.visual(
        Cylinder(radius=BARREL_RADIUS, length=LEAF_KNUCKLE_LENGTH),
        material="zinc_steel",
        name="leaf_knuckle",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    swing_leaf.visual(
        Box((OUTER_FLANGE_WIDTH, OUTER_FLANGE_DEPTH, LEAF_HEIGHT)),
        material="zinc_steel",
        name="outer_flange",
        origin=Origin(
            xyz=(
                LEAF_ROOT_X + LEAF_WIDTH - OUTER_FLANGE_WIDTH / 2.0,
                LEAF_THICKNESS / 2.0 + OUTER_FLANGE_DEPTH / 2.0 - 0.001,
                0.0,
            )
        ),
    )
    swing_leaf.inertial = Inertial.from_geometry(
        Box(
            (
                LEAF_WIDTH + BARREL_RADIUS,
                LEAF_THICKNESS + OUTER_FLANGE_DEPTH,
                LEAF_HEIGHT,
            )
        ),
        mass=2.6,
        origin=Origin(
            xyz=(
                (LEAF_WIDTH + BARREL_RADIUS) / 2.0 - BARREL_RADIUS / 2.0,
                OUTER_FLANGE_DEPTH / 2.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "mount_to_leaf",
        ArticulationType.REVOLUTE,
        parent=mount_plate,
        child=swing_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=0.0,
            upper=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    mount_plate = object_model.get_part("mount_plate")
    swing_leaf = object_model.get_part("swing_leaf")
    hinge = object_model.get_articulation("mount_to_leaf")

    ctx.check(
        "mount plate exists",
        mount_plate is not None,
        details="Expected grounded mounting plate part.",
    )
    ctx.check(
        "swing leaf exists",
        swing_leaf is not None,
        details="Expected swinging leaf part.",
    )
    ctx.check(
        "hinge axis is vertical",
        tuple(round(v, 4) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "hinge has outward swing range",
        hinge.motion_limits is not None
        and isclose(hinge.motion_limits.lower or 0.0, 0.0, abs_tol=1e-6)
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 1.8,
        details=f"limits={hinge.motion_limits}",
    )

    ctx.expect_overlap(
        swing_leaf,
        mount_plate,
        axes="z",
        min_overlap=0.120,
        name="leaf and mounting plate share the barrel height band",
    )

    closed_center = _aabb_center(ctx.part_world_aabb(swing_leaf))
    ctx.check(
        "closed leaf projects to the positive-X side of the barrel",
        closed_center is not None and closed_center[0] > 0.060,
        details=f"closed_center={closed_center}",
    )

    with ctx.pose({hinge: 1.45}):
        opened_center = _aabb_center(ctx.part_world_aabb(swing_leaf))

    ctx.check(
        "leaf swings outward toward +Y when opened",
        closed_center is not None
        and opened_center is not None
        and opened_center[1] > closed_center[1] + 0.070,
        details=f"closed_center={closed_center}, opened_center={opened_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
