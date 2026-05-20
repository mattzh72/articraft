from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.180
BODY_H = 0.050
REAR_CAP_D = 0.030
REAR_CAP_Y = -0.020
MOUNT_W = 0.070
MOUNT_T = 0.014
MOUNT_H = 0.038
MOUNT_Y = -0.042
TOP_T = 0.012
CENTER_BRIDGE_W = 0.018
CENTER_BRIDGE_D = 0.060
CENTER_BRIDGE_H = BODY_H - TOP_T

GUIDE_L = 0.066
GUIDE_CENTER_X = 0.054
FLOOR_T = 0.016
RAIL_W = 0.007
SIDE_RAIL_H = BODY_H - FLOOR_T - TOP_T
SIDE_RAIL_Z = FLOOR_T + 0.5 * SIDE_RAIL_H

CARRIAGE_L = 0.038
CARRIAGE_D = 0.046
CARRIAGE_H = 0.022
FINGER_T = 0.012
FINGER_D = 0.014
FINGER_H = 0.082
FINGER_CENTER_X = 0.013
FINGER_CENTER_Y = 0.016
PAD_T = 0.003
PAD_D = 0.014
PAD_H = 0.050
PAD_CENTER_Z = 0.047

RAIL_Y = 0.5 * CARRIAGE_D + 0.5 * RAIL_W

OPEN_CARRIAGE_CENTER_X = 0.054
JAW_STROKE = 0.026
SLIDE_Z = FLOOR_T + 0.5 * CARRIAGE_H


def _pad_origin(side_sign: float) -> Origin:
    pad_x = side_sign * (FINGER_CENTER_X + 0.5 * FINGER_T + 0.5 * PAD_T)
    return Origin(xyz=(pad_x, FINGER_CENTER_Y, PAD_CENTER_Z))


def _axis_tuple(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(round(float(value), 6) for value in values)


def _limits_match(joint, lower: float, upper: float) -> bool:
    limits = joint.motion_limits
    return (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(float(limits.lower), lower, abs_tol=1e-6)
        and math.isclose(float(limits.upper), upper, abs_tol=1e-6)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parallel_jaw_gripper_head")

    body_material = model.material("body_anodized", rgba=(0.19, 0.21, 0.24, 1.0))
    carriage_material = model.material("carriage_steel", rgba=(0.68, 0.71, 0.74, 1.0))
    pad_material = model.material("grip_pad", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, REAR_CAP_D, TOP_T)),
        origin=Origin(xyz=(0.0, REAR_CAP_Y, BODY_H - 0.5 * TOP_T)),
        material=body_material,
        name="rear_cap",
    )
    body.visual(
        Box((CENTER_BRIDGE_W, CENTER_BRIDGE_D, CENTER_BRIDGE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * CENTER_BRIDGE_H)),
        material=body_material,
        name="center_bridge",
    )
    body.visual(
        Box((MOUNT_W, MOUNT_T, MOUNT_H)),
        origin=Origin(xyz=(0.0, MOUNT_Y, 0.5 * MOUNT_H)),
        material=body_material,
        name="mount_block",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_center = side_sign * GUIDE_CENTER_X
        body.visual(
            Box((GUIDE_L, CARRIAGE_D, FLOOR_T)),
            origin=Origin(xyz=(x_center, 0.0, 0.5 * FLOOR_T)),
            material=body_material,
            name=f"{side_name}_floor",
        )
        for rail_name, rail_y in (("front", -RAIL_Y), ("rear", RAIL_Y)):
            body.visual(
                Box((GUIDE_L, RAIL_W, SIDE_RAIL_H)),
                origin=Origin(
                    xyz=(
                        x_center,
                        rail_y,
                        SIDE_RAIL_Z,
                    )
                ),
                material=body_material,
                name=f"{side_name}_{rail_name}_rail",
            )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        Box((CARRIAGE_L, CARRIAGE_D, CARRIAGE_H)),
        material=carriage_material,
        name="carriage_shell",
    )
    left_carriage.visual(
        Box((FINGER_T, FINGER_D, FINGER_H)),
        origin=Origin(
            xyz=(
                FINGER_CENTER_X,
                FINGER_CENTER_Y,
                0.5 * CARRIAGE_H + 0.5 * FINGER_H,
            )
        ),
        material=carriage_material,
        name="finger",
    )
    left_carriage.visual(
        Box((PAD_T, PAD_D, PAD_H)),
        origin=_pad_origin(1.0),
        material=pad_material,
        name="grip_pad",
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        Box((CARRIAGE_L, CARRIAGE_D, CARRIAGE_H)),
        material=carriage_material,
        name="carriage_shell",
    )
    right_carriage.visual(
        Box((FINGER_T, FINGER_D, FINGER_H)),
        origin=Origin(
            xyz=(
                -FINGER_CENTER_X,
                FINGER_CENTER_Y,
                0.5 * CARRIAGE_H + 0.5 * FINGER_H,
            )
        ),
        material=carriage_material,
        name="finger",
    )
    right_carriage.visual(
        Box((PAD_T, PAD_D, PAD_H)),
        origin=_pad_origin(-1.0),
        material=pad_material,
        name="grip_pad",
    )

    model.articulation(
        "body_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(-OPEN_CARRIAGE_CENTER_X, 0.0, SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.15,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )
    model.articulation(
        "body_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(OPEN_CARRIAGE_CENTER_X, 0.0, SLIDE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.15,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("body_to_left_carriage")
    right_slide = object_model.get_articulation("body_to_right_carriage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=2e-4)
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
        "left_slide_is_prismatic",
        left_slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC, got {left_slide.articulation_type!r}",
    )
    ctx.check(
        "right_slide_is_prismatic",
        right_slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC, got {right_slide.articulation_type!r}",
    )
    ctx.check(
        "left_slide_axis",
        _axis_tuple(left_slide.axis) == (1.0, 0.0, 0.0),
        f"left slide axis should close along +X, got {left_slide.axis!r}",
    )
    ctx.check(
        "right_slide_axis",
        _axis_tuple(right_slide.axis) == (-1.0, 0.0, 0.0),
        f"right slide axis should close along -X, got {right_slide.axis!r}",
    )
    ctx.check(
        "left_slide_limits",
        _limits_match(left_slide, 0.0, JAW_STROKE),
        f"unexpected left slide limits: {left_slide.motion_limits!r}",
    )
    ctx.check(
        "right_slide_limits",
        _limits_match(right_slide, 0.0, JAW_STROKE),
        f"unexpected right slide limits: {right_slide.motion_limits!r}",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_contact(
            left_carriage,
            body,
            contact_tol=2e-4,
            name="left_carriage_supported_open",
        )
        ctx.expect_contact(
            right_carriage,
            body,
            contact_tol=2e-4,
            name="right_carriage_supported_open",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.060,
            max_gap=0.068,
            name="open_grip_gap",
        )
        ctx.expect_overlap(
            left_carriage,
            right_carriage,
            axes="yz",
            elem_a="grip_pad",
            elem_b="grip_pad",
            min_overlap=PAD_D - 0.001,
            name="open_pad_alignment",
        )
        ctx.expect_within(
            left_carriage,
            body,
            axes="y",
            margin=0.0,
            name="left_carriage_depth_contained_open",
        )
        ctx.expect_within(
            right_carriage,
            body,
            axes="y",
            margin=0.0,
            name="right_carriage_depth_contained_open",
        )

    with ctx.pose({left_slide: JAW_STROKE, right_slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="left_stroked_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=2e-4, name="left_stroked_no_floating")
        ctx.expect_contact(
            left_carriage,
            body,
            contact_tol=2e-4,
            name="left_carriage_supported_stroked",
        )

    with ctx.pose({left_slide: 0.0, right_slide: JAW_STROKE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_stroked_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=2e-4, name="right_stroked_no_floating")
        ctx.expect_contact(
            right_carriage,
            body,
            contact_tol=2e-4,
            name="right_carriage_supported_stroked",
        )

    with ctx.pose({left_slide: JAW_STROKE, right_slide: JAW_STROKE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="both_jaws_closed_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=2e-4, name="both_jaws_closed_no_floating")
        ctx.expect_contact(
            left_carriage,
            body,
            contact_tol=2e-4,
            name="left_carriage_supported_closed",
        )
        ctx.expect_contact(
            right_carriage,
            body,
            contact_tol=2e-4,
            name="right_carriage_supported_closed",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.010,
            max_gap=0.014,
            name="closed_grip_gap",
        )
        ctx.expect_overlap(
            left_carriage,
            right_carriage,
            axes="yz",
            elem_a="grip_pad",
            elem_b="grip_pad",
            min_overlap=PAD_D - 0.001,
            name="closed_pad_alignment",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="jaw_articulation_sweep_clear")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=24,
        contact_tol=2e-4,
        name="jaw_pose_sweep_no_floating",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
