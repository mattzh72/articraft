from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_BASE_X = 0.22
BODY_BASE_Y = 0.28
BODY_BASE_Z = 0.055
ROLL_AXIS_Z = 0.15
PITCH_AXIS_X = 0.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_pitch_roll_head")

    body_color = model.material("body_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    frame_color = model.material("frame_graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    cradle_color = model.material("cradle_silver", rgba=(0.71, 0.74, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_BASE_X, BODY_BASE_Y, BODY_BASE_Z)),
        material=body_color,
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z * 0.5)),
        name="body_shell",
    )
    body.visual(
        Box((0.022, 0.04, 0.15)),
        material=body_color,
        origin=Origin(xyz=(-0.067, 0.0, 0.13)),
        name="left_upright",
    )
    body.visual(
        Box((0.022, 0.04, 0.15)),
        material=body_color,
        origin=Origin(xyz=(0.067, 0.0, 0.13)),
        name="right_upright",
    )
    body.visual(
        Box((0.038, 0.06, 0.06)),
        material=body_color,
        origin=Origin(xyz=(-0.067, 0.0, 0.085)),
        name="left_gusset",
    )
    body.visual(
        Box((0.038, 0.06, 0.06)),
        material=body_color,
        origin=Origin(xyz=(0.067, 0.0, 0.085)),
        name="right_gusset",
    )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((0.072, 0.012, 0.084)),
        material=frame_color,
        origin=Origin(xyz=(PITCH_AXIS_X, -0.03, 0.0)),
        name="left_pitch_cheek",
    )
    outer_frame.visual(
        Box((0.072, 0.012, 0.084)),
        material=frame_color,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.03, 0.0)),
        name="right_pitch_cheek",
    )
    outer_frame.visual(
        Box((0.072, 0.048, 0.012)),
        material=frame_color,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, 0.036)),
        name="top_bridge",
    )
    outer_frame.visual(
        Box((0.072, 0.048, 0.012)),
        material=frame_color,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, -0.036)),
        name="bottom_bridge",
    )
    outer_frame.visual(
        Box((0.012, 0.022, 0.022)),
        material=frame_color,
        origin=Origin(xyz=(-0.032, 0.0, 0.021)),
        name="left_upper_roll_brace",
    )
    outer_frame.visual(
        Box((0.012, 0.022, 0.022)),
        material=frame_color,
        origin=Origin(xyz=(-0.032, 0.0, -0.021)),
        name="left_lower_roll_brace",
    )
    outer_frame.visual(
        Box((0.012, 0.022, 0.022)),
        material=frame_color,
        origin=Origin(xyz=(0.032, 0.0, 0.021)),
        name="right_upper_roll_brace",
    )
    outer_frame.visual(
        Box((0.012, 0.022, 0.022)),
        material=frame_color,
        origin=Origin(xyz=(0.032, 0.0, -0.021)),
        name="right_lower_roll_brace",
    )
    outer_frame.visual(
        Cylinder(radius=0.011, length=0.02),
        material=frame_color,
        origin=Origin(xyz=(-0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        name="left_roll_stub",
    )
    outer_frame.visual(
        Cylinder(radius=0.011, length=0.02),
        material=frame_color,
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        name="right_roll_stub",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.01, length=0.048),
        material=cradle_color,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        name="pitch_trunnion",
    )
    inner_cradle.visual(
        Box((0.036, 0.038, 0.044)),
        material=cradle_color,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        name="cradle_body",
    )
    inner_cradle.visual(
        Box((0.01, 0.022, 0.022)),
        material=cradle_color,
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        name="nose_block",
    )

    model.articulation(
        "body_to_outer_frame",
        ArticulationType.REVOLUTE,
        parent=body,
        child=outer_frame,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.55, upper=0.55),
    )

    model.articulation(
        "outer_frame_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=inner_cradle,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.45, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    outer_frame = object_model.get_part("outer_frame")
    inner_cradle = object_model.get_part("inner_cradle")
    roll_joint = object_model.get_articulation("body_to_outer_frame")
    pitch_joint = object_model.get_articulation("outer_frame_to_inner_cradle")

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
        "roll_joint_axis",
        tuple(roll_joint.axis) == (1.0, 0.0, 0.0),
        f"expected roll axis (1, 0, 0), got {roll_joint.axis}",
    )
    ctx.check(
        "pitch_joint_axis",
        tuple(pitch_joint.axis) == (0.0, -1.0, 0.0),
        f"expected pitch axis (0, -1, 0), got {pitch_joint.axis}",
    )
    ctx.check(
        "compact_roll_range",
        roll_joint.motion_limits is not None
        and roll_joint.motion_limits.lower is not None
        and roll_joint.motion_limits.upper is not None
        and roll_joint.motion_limits.lower < 0.0
        and roll_joint.motion_limits.upper > 0.0,
        "roll joint should support left and right motion about the centered frame",
    )
    ctx.check(
        "pitch_range_biases_upward",
        pitch_joint.motion_limits is not None
        and pitch_joint.motion_limits.lower is not None
        and pitch_joint.motion_limits.upper is not None
        and pitch_joint.motion_limits.lower < 0.0
        and pitch_joint.motion_limits.upper > 0.5,
        "pitch joint should allow a deeper upward nod than downward tuck",
    )

    ctx.expect_contact(body, outer_frame, name="outer_frame_supported_by_body")
    ctx.expect_contact(outer_frame, inner_cradle, name="inner_cradle_supported_by_outer_frame")
    ctx.expect_within(
        inner_cradle,
        outer_frame,
        axes="yz",
        margin=0.002,
        name="inner_cradle_nested_inside_outer_frame",
    )

    rest_aabb = ctx.part_world_aabb(inner_cradle)
    with ctx.pose({pitch_joint: pitch_joint.motion_limits.upper}):
        pitched_aabb = ctx.part_world_aabb(inner_cradle)
    if rest_aabb is None or pitched_aabb is None:
        ctx.fail("pitch_pose_measurement", "could not evaluate inner cradle bounds across pitch pose")
    else:
        ctx.check(
            "pitch_raises_cradle",
            pitched_aabb[1][2] > rest_aabb[1][2] + 0.008,
            f"expected pitched cradle top to rise by at least 0.008 m; rest top={rest_aabb[1][2]:.4f}, pitched top={pitched_aabb[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
