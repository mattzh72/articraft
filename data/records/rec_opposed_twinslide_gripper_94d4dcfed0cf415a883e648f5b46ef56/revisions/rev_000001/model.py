from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HOUSING_W = 0.108
HOUSING_D = 0.058
HOUSING_H = 0.046
REAR_MOUNT_W = 0.044
REAR_MOUNT_D = 0.014
REAR_MOUNT_H = 0.034

CENTER_WEB = 0.004
GUIDE_LEN = 0.044
GUIDE_DEPTH = 0.028
GUIDE_HEIGHT = 0.020
JAW_TRAVEL = 0.008

REAR_BLOCK_D = 0.030
REAR_BLOCK_Y = -(GUIDE_DEPTH / 2.0 + REAR_BLOCK_D / 2.0)
TOP_CAP_D = 0.022
TOP_CAP_H = 0.008
SHELF_H = 0.008
SHELF_Z = -(GUIDE_HEIGHT / 2.0 + SHELF_H / 2.0)

FINGER_BODY_LEN = 0.024
FINGER_BODY_DEPTH = 0.018
FINGER_BODY_HEIGHT = 0.026

STEP_FINGER_LEN = 0.010
STEP_FINGER_DEPTH = 0.012
STEP_FINGER_HEIGHT = 0.018

LEFT_JAW_X = -HOUSING_W / 2.0
RIGHT_JAW_X = HOUSING_W / 2.0
LEFT_CHANNEL_CENTER_X = -(HOUSING_W / 4.0 + CENTER_WEB / 4.0)
RIGHT_CHANNEL_CENTER_X = -LEFT_CHANNEL_CENTER_X
CHANNEL_LEN = HOUSING_W / 2.0 - CENTER_WEB / 2.0


def _add_jaw(
    model: ArticulatedObject,
    *,
    name: str,
    body_material: str,
    tip_material: str,
):
    jaw = model.part(name)
    jaw.visual(
        Box((GUIDE_LEN, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_LEN / 2.0, 0.0, 0.0)),
        material=body_material,
        name="guide_rail",
    )
    jaw.visual(
        Box((FINGER_BODY_LEN, FINGER_BODY_DEPTH, FINGER_BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.028,
                GUIDE_DEPTH / 2.0 + FINGER_BODY_DEPTH / 2.0,
                0.0,
            )
        ),
        material=body_material,
        name="finger_body",
    )
    jaw.visual(
        Box((STEP_FINGER_LEN, STEP_FINGER_DEPTH, STEP_FINGER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.039,
                GUIDE_DEPTH / 2.0 + FINGER_BODY_DEPTH + STEP_FINGER_DEPTH / 2.0,
                0.0,
            )
        ),
        material=tip_material,
        name="step_finger",
    )
    return jaw


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_parallel_gripper")

    housing_material = model.material("housing_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    jaw_material = model.material("jaw_satin", rgba=(0.72, 0.74, 0.77, 1.0))
    tip_material = model.material("finger_tip_dark", rgba=(0.17, 0.18, 0.20, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_W, REAR_BLOCK_D, HOUSING_H)),
        origin=Origin(xyz=(0.0, REAR_BLOCK_Y, 0.0)),
        material=housing_material,
        name="rear_block",
    )
    housing.visual(
        Box((HOUSING_W, TOP_CAP_D, TOP_CAP_H)),
        origin=Origin(xyz=(0.0, 0.003, HOUSING_H / 2.0 - TOP_CAP_H / 2.0)),
        material=housing_material,
        name="top_cap",
    )
    housing.visual(
        Box((CHANNEL_LEN, GUIDE_DEPTH, SHELF_H)),
        origin=Origin(xyz=(LEFT_CHANNEL_CENTER_X, 0.0, SHELF_Z)),
        material=housing_material,
        name="left_shelf",
    )
    housing.visual(
        Box((CHANNEL_LEN, GUIDE_DEPTH, SHELF_H)),
        origin=Origin(xyz=(RIGHT_CHANNEL_CENTER_X, 0.0, SHELF_Z)),
        material=housing_material,
        name="right_shelf",
    )
    housing.visual(
        Box((CENTER_WEB, GUIDE_DEPTH, HOUSING_H - TOP_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, -TOP_CAP_H / 2.0)),
        material=housing_material,
        name="center_web",
    )
    housing.visual(
        Box((REAR_MOUNT_W, REAR_MOUNT_D, REAR_MOUNT_H)),
        origin=Origin(
            xyz=(0.0, REAR_BLOCK_Y - REAR_BLOCK_D / 2.0 - REAR_MOUNT_D / 2.0, 0.0)
        ),
        material=housing_material,
        name="rear_mount",
    )

    left_jaw = _add_jaw(
        model,
        name="left_carriage",
        body_material=jaw_material,
        tip_material=tip_material,
    )
    right_jaw = _add_jaw(
        model,
        name="right_carriage",
        body_material=jaw_material,
        tip_material=tip_material,
    )

    model.articulation(
        "housing_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_jaw,
        origin=Origin(xyz=(LEFT_JAW_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.12,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "housing_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_jaw,
        origin=Origin(xyz=(RIGHT_JAW_X, 0.0, 0.0), rpy=(0.0, 3.141592653589793, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.12,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left = object_model.get_part("left_carriage")
    right = object_model.get_part("right_carriage")
    left_joint = object_model.get_articulation("housing_to_left_carriage")
    right_joint = object_model.get_articulation("housing_to_right_carriage")

    left_shelf = housing.get_visual("left_shelf")
    right_shelf = housing.get_visual("right_shelf")
    left_guide = left.get_visual("guide_rail")
    right_guide = right.get_visual("guide_rail")
    left_step = left.get_visual("step_finger")
    right_step = right.get_visual("step_finger")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shared_local_closing_axis",
        left_joint.axis == (1.0, 0.0, 0.0) and right_joint.axis == (1.0, 0.0, 0.0),
        details=f"left axis={left_joint.axis}, right axis={right_joint.axis}",
    )

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_contact(
            left,
            housing,
            elem_a=left_guide,
            elem_b=left_shelf,
            name="left_guide_supported_open",
        )
        ctx.expect_contact(
            right,
            housing,
            elem_a=right_guide,
            elem_b=right_shelf,
            name="right_guide_supported_open",
        )
        ctx.expect_overlap(
            left,
            housing,
            axes="xy",
            elem_a=left_guide,
            elem_b=left_shelf,
            min_overlap=0.024,
            name="left_guide_overlap_open",
        )
        ctx.expect_overlap(
            right,
            housing,
            axes="xy",
            elem_a=right_guide,
            elem_b=right_shelf,
            min_overlap=0.024,
            name="right_guide_overlap_open",
        )
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem=right_step,
            negative_elem=left_step,
            min_gap=0.018,
            max_gap=0.022,
            name="open_step_finger_gap",
        )

    with ctx.pose({left_joint: JAW_TRAVEL, right_joint: JAW_TRAVEL}):
        ctx.expect_contact(
            left,
            housing,
            elem_a=left_guide,
            elem_b=left_shelf,
            name="left_guide_supported_closed",
        )
        ctx.expect_contact(
            right,
            housing,
            elem_a=right_guide,
            elem_b=right_shelf,
            name="right_guide_supported_closed",
        )
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem=right_step,
            negative_elem=left_step,
            min_gap=0.003,
            max_gap=0.006,
            name="closed_step_finger_gap",
        )

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        left_open_x = ctx.part_world_position(left)[0]
        right_open_x = ctx.part_world_position(right)[0]
    with ctx.pose({left_joint: JAW_TRAVEL, right_joint: JAW_TRAVEL}):
        left_closed_x = ctx.part_world_position(left)[0]
        right_closed_x = ctx.part_world_position(right)[0]

    ctx.check(
        "positive_q_closes_both_carriages",
        left_closed_x > left_open_x and right_closed_x < right_open_x,
        details=(
            f"left open/closed x={left_open_x:.4f}/{left_closed_x:.4f}, "
            f"right open/closed x={right_open_x:.4f}/{right_closed_x:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
