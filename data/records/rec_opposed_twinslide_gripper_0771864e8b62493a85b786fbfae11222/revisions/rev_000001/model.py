from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_OUTER_W = 0.220
FRAME_BACK_T = 0.012
FRAME_H = 0.090
FRAME_BACK_Y = -0.026
FRAME_WINDOW_W = 0.130
FRAME_WINDOW_H = 0.052
FRAME_WINDOW_Z = 0.046

HOUSING_W = 0.058
HOUSING_D = 0.030
HOUSING_H = 0.052
HOUSING_Y = -0.005

GUIDE_L = 0.076
GUIDE_D = 0.016
GUIDE_H = 0.010
GUIDE_Y = 0.000
GUIDE_Z = 0.046
GUIDE_X = 0.070
GUIDE_TOP_Z = GUIDE_Z + (GUIDE_H / 2.0)

SHOE_W = 0.024
SHOE_D = 0.014
SHOE_H = 0.006
RISER_W = 0.018
RISER_D = 0.016
RISER_H = 0.014
BODY_W = 0.030
BODY_D = 0.020
BODY_H = 0.018
ARM_W = 0.054
ARM_D = 0.012
ARM_H = 0.010
FINGER_W = 0.012
FINGER_D = 0.014
FINGER_H = 0.032
PAD_W = 0.010
PAD_D = 0.016
PAD_H = 0.018

LEFT_OPEN_X = -0.092
RIGHT_OPEN_X = 0.092
JAW_TRAVEL = 0.020


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _frame_shape() -> cq.Workplane:
    rear_frame = _box(
        (FRAME_OUTER_W, FRAME_BACK_T, FRAME_H),
        (0.0, FRAME_BACK_Y, FRAME_H / 2.0),
    ).cut(
        _box(
            (FRAME_WINDOW_W, FRAME_BACK_T + 0.010, FRAME_WINDOW_H),
            (0.0, FRAME_BACK_Y, FRAME_WINDOW_Z),
        )
    )

    housing = _box(
        (HOUSING_W, HOUSING_D, HOUSING_H),
        (0.0, HOUSING_Y, HOUSING_H / 2.0),
    ).cut(
        _box(
            (0.022, 0.018, 0.022),
            (0.0, 0.004, 0.017),
        )
    )

    left_guide = _box((GUIDE_L, GUIDE_D, GUIDE_H), (-GUIDE_X, GUIDE_Y, GUIDE_Z))
    right_guide = _box((GUIDE_L, GUIDE_D, GUIDE_H), (GUIDE_X, GUIDE_Y, GUIDE_Z))

    left_web = _box(
        (0.024, 0.018, 0.032),
        (-0.040, -0.003, 0.025),
    )
    right_web = _box(
        (0.024, 0.018, 0.032),
        (0.040, -0.003, 0.025),
    )

    center_nose = _box(
        (0.030, 0.016, 0.016),
        (0.0, 0.014, 0.016),
    )

    return (
        rear_frame.union(housing)
        .union(left_guide)
        .union(right_guide)
        .union(left_web)
        .union(right_web)
        .union(center_nose)
    )


def _carriage_shape(sign: float) -> cq.Workplane:
    shoe = _box((SHOE_W, SHOE_D, SHOE_H), (0.0, 0.0, SHOE_H / 2.0))
    riser = _box((RISER_W, RISER_D, RISER_H), (0.0, 0.010, 0.013))
    body = _box((BODY_W, BODY_D, BODY_H), (0.0, 0.020, 0.017))
    jaw_arm = _box((ARM_W, ARM_D, ARM_H), (sign * 0.031, 0.030, 0.015))
    finger = _box((FINGER_W, FINGER_D, FINGER_H), (sign * 0.054, 0.033, 0.027))

    return shoe.union(riser).union(body).union(jaw_arm).union(finger)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_parallel_gripper")

    model.material("frame_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("carriage_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("jaw_pad", rgba=(0.12, 0.13, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_shell"),
        material="frame_graphite",
        name="frame_shell",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_W, 0.070, FRAME_H)),
        mass=3.4,
        origin=Origin(xyz=(0.0, -0.004, FRAME_H / 2.0)),
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        mesh_from_cadquery(_carriage_shape(1.0), "left_carriage_shell"),
        material="carriage_alloy",
        name="carriage_shell",
    )
    left_carriage.visual(
        Box((PAD_W, PAD_D, PAD_H)),
        origin=Origin(xyz=(0.060, 0.040, 0.020)),
        material="jaw_pad",
        name="inner_pad",
    )
    left_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.070)),
        mass=0.55,
        origin=Origin(xyz=(0.025, 0.028, -0.026)),
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        mesh_from_cadquery(_carriage_shape(-1.0), "right_carriage_shell"),
        material="carriage_alloy",
        name="carriage_shell",
    )
    right_carriage.visual(
        Box((PAD_W, PAD_D, PAD_H)),
        origin=Origin(xyz=(-0.060, 0.040, 0.020)),
        material="jaw_pad",
        name="inner_pad",
    )
    right_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.070)),
        mass=0.55,
        origin=Origin(xyz=(-0.025, 0.028, -0.026)),
    )

    model.articulation(
        "frame_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_carriage,
        origin=Origin(xyz=(LEFT_OPEN_X, GUIDE_Y, GUIDE_Z - (GUIDE_H / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "frame_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_carriage,
        origin=Origin(xyz=(RIGHT_OPEN_X, GUIDE_Y, GUIDE_Z - (GUIDE_H / 2.0))),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slider = object_model.get_articulation("frame_to_left_carriage")
    right_slider = object_model.get_articulation("frame_to_right_carriage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        frame,
        left_carriage,
        elem_a="frame_shell",
        elem_b="carriage_shell",
        reason="Left jaw carriage is modeled as a captured slide inside an enclosed guideway; the hidden running surfaces are simplified as an interpenetrating shell pair.",
    )
    ctx.allow_overlap(
        frame,
        right_carriage,
        elem_a="frame_shell",
        elem_b="carriage_shell",
        reason="Right jaw carriage is modeled as a captured slide inside an enclosed guideway; the hidden running surfaces are simplified as an interpenetrating shell pair.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "left_slider_axis_points_inward",
        tuple(left_slider.axis) == (1.0, 0.0, 0.0),
        f"expected +X axis for left jaw, got {left_slider.axis}",
    )
    ctx.check(
        "right_slider_axis_points_inward",
        tuple(right_slider.axis) == (-1.0, 0.0, 0.0),
        f"expected -X axis for right jaw, got {right_slider.axis}",
    )

    ctx.expect_contact(
        left_carriage,
        frame,
        name="left_carriage_supported_on_left_guideway",
    )
    ctx.expect_contact(
        right_carriage,
        frame,
        name="right_carriage_supported_on_right_guideway",
    )
    ctx.expect_gap(
        right_carriage,
        left_carriage,
        axis="x",
        min_gap=0.053,
        max_gap=0.055,
        positive_elem="inner_pad",
        negative_elem="inner_pad",
        name="open_pose_pad_gap",
    )
    ctx.expect_overlap(
        left_carriage,
        right_carriage,
        axes="yz",
        min_overlap=0.014,
        elem_a="inner_pad",
        elem_b="inner_pad",
        name="pads_share_a_facing_window",
    )

    left_open_x = ctx.part_world_position(left_carriage)[0]
    right_open_x = ctx.part_world_position(right_carriage)[0]
    with ctx.pose({left_slider: JAW_TRAVEL, right_slider: JAW_TRAVEL}):
        left_closed_x = ctx.part_world_position(left_carriage)[0]
        right_closed_x = ctx.part_world_position(right_carriage)[0]

        ctx.expect_contact(
            left_carriage,
            frame,
            name="left_carriage_remains_supported_when_closed",
        )
        ctx.expect_contact(
            right_carriage,
            frame,
            name="right_carriage_remains_supported_when_closed",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            min_gap=0.013,
            max_gap=0.015,
            positive_elem="inner_pad",
            negative_elem="inner_pad",
            name="closed_pose_pad_gap",
        )

    ctx.check(
        "left_carriage_moves_toward_center",
        left_closed_x > left_open_x + 0.015,
        f"left carriage x did not move inward enough: open={left_open_x:.4f}, closed={left_closed_x:.4f}",
    )
    ctx.check(
        "right_carriage_moves_toward_center",
        right_closed_x < right_open_x - 0.015,
        f"right carriage x did not move inward enough: open={right_open_x:.4f}, closed={right_closed_x:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
