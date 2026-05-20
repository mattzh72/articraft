from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_PLATE_THICKNESS = 0.018
SUPPORT_PLATE_WIDTH = 0.150
SUPPORT_PLATE_HEIGHT = 0.180
SUPPORT_PLATE_X = -0.072

SUPPORT_WINDOW_X = 0.028
SUPPORT_WINDOW_Y = 0.094
SUPPORT_WINDOW_Z = 0.108
SUPPORT_SIDE_RIB_LENGTH = 0.016
SUPPORT_SIDE_RIB_WIDTH = 0.022
SUPPORT_SIDE_RIB_HEIGHT = 0.116
SUPPORT_SIDE_RIB_X = -0.064
SUPPORT_SIDE_RIB_Y = 0.052
SUPPORT_TOP_BRIDGE_LENGTH = 0.016
SUPPORT_TOP_BRIDGE_WIDTH = 0.116
SUPPORT_TOP_BRIDGE_HEIGHT = 0.020
SUPPORT_TOP_BRIDGE_X = -0.064
SUPPORT_TOP_BRIDGE_Z = 0.048
SUPPORT_BEARING_RADIUS = 0.022
SUPPORT_BEARING_LENGTH = 0.008
SUPPORT_BEARING_X = -0.060
SUPPORT_HUB_BLOCK_LENGTH = 0.012
SUPPORT_HUB_BLOCK_WIDTH = 0.032
SUPPORT_HUB_BLOCK_HEIGHT = 0.032
SUPPORT_HUB_BLOCK_X = -0.068
SUPPORT_HUB_WEB_LENGTH = 0.012
SUPPORT_HUB_WEB_WIDTH = 0.026
SUPPORT_HUB_WEB_HEIGHT = 0.030
SUPPORT_HUB_WEB_X = -0.068
SUPPORT_HUB_WEB_Z = 0.028

YAW_DRUM_RADIUS = 0.040
YAW_DRUM_HEIGHT = 0.056
YAW_CAP_RADIUS = 0.026
YAW_CAP_THICKNESS = 0.014
YAW_CAP_Z_OFFSET = 0.035
YAW_REAR_HUB_RADIUS = 0.020
YAW_REAR_HUB_LENGTH = 0.016
YAW_REAR_HUB_X = -0.048
YAW_EAR_LENGTH = 0.032
YAW_EAR_WIDTH = 0.018
YAW_EAR_HEIGHT = 0.052
YAW_EAR_X = 0.052
YAW_EAR_Y = 0.043
YAW_CLEVIS_BLOCK_LENGTH = 0.034
YAW_CLEVIS_BLOCK_WIDTH = 0.076
YAW_CLEVIS_BLOCK_HEIGHT = 0.036
YAW_CLEVIS_BLOCK_X = 0.030

PITCH_AXIS_X = 0.058

PITCH_TRUNNION_RADIUS = 0.013
PITCH_TRUNNION_LENGTH = 0.090
PITCH_BODY_LENGTH = 0.050
PITCH_BODY_WIDTH = 0.038
PITCH_BODY_HEIGHT = 0.046
PITCH_BODY_X = 0.036
PITCH_NOSE_RADIUS = 0.020
PITCH_NOSE_LENGTH = 0.038
PITCH_NOSE_X = 0.072

ROLL_AXIS_X = 0.074

ROLL_FLANGE_RADIUS = 0.026
ROLL_FLANGE_LENGTH = 0.010
ROLL_BODY_RADIUS = 0.020
ROLL_BODY_LENGTH = 0.052
ROLL_TIP_RADIUS = 0.013
ROLL_TIP_LENGTH = 0.018
ROLL_LUG_LENGTH = 0.022
ROLL_LUG_WIDTH = 0.020
ROLL_LUG_HEIGHT = 0.014
ROLL_LUG_X = 0.069
ROLL_LUG_Z = 0.027


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _rear_support_shape() -> cq.Workplane:
    support = (
        cq.Workplane("XY")
        .box(SUPPORT_PLATE_THICKNESS, SUPPORT_PLATE_WIDTH, SUPPORT_PLATE_HEIGHT)
        .translate((SUPPORT_PLATE_X, 0.0, 0.0))
    )
    support = support.cut(
        cq.Workplane("XY")
        .box(SUPPORT_WINDOW_X, SUPPORT_WINDOW_Y, SUPPORT_WINDOW_Z)
        .translate((SUPPORT_PLATE_X, 0.0, 0.0))
    )

    for y_sign in (-1.0, 1.0):
        y_center = y_sign * SUPPORT_SIDE_RIB_Y
        support = support.union(
            cq.Workplane("XY")
            .box(SUPPORT_SIDE_RIB_LENGTH, SUPPORT_SIDE_RIB_WIDTH, SUPPORT_SIDE_RIB_HEIGHT)
            .translate((SUPPORT_SIDE_RIB_X, y_center, 0.0))
        )

    support = support.union(
        cq.Workplane("XY")
        .box(SUPPORT_TOP_BRIDGE_LENGTH, SUPPORT_TOP_BRIDGE_WIDTH, SUPPORT_TOP_BRIDGE_HEIGHT)
        .translate((SUPPORT_TOP_BRIDGE_X, 0.0, SUPPORT_TOP_BRIDGE_Z))
    )
    support = support.union(
        cq.Workplane("XY")
        .box(SUPPORT_TOP_BRIDGE_LENGTH, SUPPORT_TOP_BRIDGE_WIDTH, SUPPORT_TOP_BRIDGE_HEIGHT)
        .translate((SUPPORT_TOP_BRIDGE_X, 0.0, -SUPPORT_TOP_BRIDGE_Z))
    )
    support = support.union(
        _cylinder_x(
            SUPPORT_BEARING_RADIUS,
            SUPPORT_BEARING_LENGTH,
            (SUPPORT_BEARING_X, 0.0, 0.0),
        )
    )
    support = support.union(
        cq.Workplane("XY")
        .box(SUPPORT_HUB_BLOCK_LENGTH, SUPPORT_HUB_BLOCK_WIDTH, SUPPORT_HUB_BLOCK_HEIGHT)
        .translate((SUPPORT_HUB_BLOCK_X, 0.0, 0.0))
    )
    support = support.union(
        cq.Workplane("XY")
        .box(SUPPORT_HUB_WEB_LENGTH, SUPPORT_HUB_WEB_WIDTH, SUPPORT_HUB_WEB_HEIGHT)
        .translate((SUPPORT_HUB_WEB_X, 0.0, SUPPORT_HUB_WEB_Z))
    )
    support = support.union(
        cq.Workplane("XY")
        .box(SUPPORT_HUB_WEB_LENGTH, SUPPORT_HUB_WEB_WIDTH, SUPPORT_HUB_WEB_HEIGHT)
        .translate((SUPPORT_HUB_WEB_X, 0.0, -SUPPORT_HUB_WEB_Z))
    )
    return support


def _yaw_stage_shape() -> cq.Workplane:
    stage = _cylinder_z(YAW_DRUM_RADIUS, YAW_DRUM_HEIGHT, (0.0, 0.0, 0.0))
    stage = stage.union(_cylinder_z(YAW_CAP_RADIUS, YAW_CAP_THICKNESS, (0.0, 0.0, YAW_CAP_Z_OFFSET)))
    stage = stage.union(_cylinder_z(YAW_CAP_RADIUS, YAW_CAP_THICKNESS, (0.0, 0.0, -YAW_CAP_Z_OFFSET)))
    stage = stage.union(
        _cylinder_x(YAW_REAR_HUB_RADIUS, YAW_REAR_HUB_LENGTH, (YAW_REAR_HUB_X, 0.0, 0.0))
    )
    stage = stage.union(
        cq.Workplane("XY")
        .box(YAW_CLEVIS_BLOCK_LENGTH, YAW_CLEVIS_BLOCK_WIDTH, YAW_CLEVIS_BLOCK_HEIGHT)
        .translate((YAW_CLEVIS_BLOCK_X, 0.0, 0.0))
    )
    stage = stage.union(
        cq.Workplane("XY")
        .box(YAW_EAR_LENGTH, YAW_EAR_WIDTH, YAW_EAR_HEIGHT)
        .translate((YAW_EAR_X, YAW_EAR_Y, 0.0))
    )
    stage = stage.union(
        cq.Workplane("XY")
        .box(YAW_EAR_LENGTH, YAW_EAR_WIDTH, YAW_EAR_HEIGHT)
        .translate((YAW_EAR_X, -YAW_EAR_Y, 0.0))
    )
    return stage


def _pitch_frame_shape() -> cq.Workplane:
    frame = _cylinder_y(PITCH_TRUNNION_RADIUS, PITCH_TRUNNION_LENGTH, (0.0, 0.0, 0.0))
    frame = frame.union(
        cq.Workplane("XY")
        .box(PITCH_BODY_LENGTH, PITCH_BODY_WIDTH, PITCH_BODY_HEIGHT)
        .translate((PITCH_BODY_X, 0.0, 0.0))
    )
    frame = frame.union(
        _cylinder_x(PITCH_NOSE_RADIUS, PITCH_NOSE_LENGTH, (PITCH_NOSE_X, 0.0, 0.0))
    )
    frame = frame.union(
        cq.Workplane("XY")
        .box(0.020, 0.032, 0.016)
        .translate((0.048, 0.0, -0.015))
    )
    return frame


def _roll_spindle_shape() -> cq.Workplane:
    spindle = _cylinder_x(ROLL_FLANGE_RADIUS, ROLL_FLANGE_LENGTH, (ROLL_FLANGE_LENGTH / 2.0, 0.0, 0.0))
    spindle = spindle.union(
        _cylinder_x(
            ROLL_BODY_RADIUS,
            ROLL_BODY_LENGTH,
            (ROLL_FLANGE_LENGTH + (ROLL_BODY_LENGTH / 2.0), 0.0, 0.0),
        )
    )
    spindle = spindle.union(
        _cylinder_x(
            ROLL_TIP_RADIUS,
            ROLL_TIP_LENGTH,
            (ROLL_FLANGE_LENGTH + ROLL_BODY_LENGTH + (ROLL_TIP_LENGTH / 2.0), 0.0, 0.0),
        )
    )
    spindle = spindle.union(
        cq.Workplane("XY")
        .box(ROLL_LUG_LENGTH, ROLL_LUG_WIDTH, ROLL_LUG_HEIGHT)
        .translate((ROLL_LUG_X, 0.0, ROLL_LUG_Z))
    )
    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_wrist")

    model.material("support_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("machined_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        origin=Origin(),
        material="support_dark",
        name="support_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage"),
        origin=Origin(),
        material="machined_alloy",
        name="yaw_shell",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_pitch_frame_shape(), "pitch_frame"),
        origin=Origin(),
        material="machined_alloy",
        name="pitch_shell",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(_roll_spindle_shape(), "roll_spindle"),
        origin=Origin(),
        material="brushed_steel",
        name="roll_shell",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.2, upper=2.2, effort=60.0, velocity=2.6),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.15, effort=38.0, velocity=2.2),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=18.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

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
    ctx.allow_overlap(
        yaw_stage,
        pitch_frame,
        reason="pitch trunnion is represented as a journal passing through the yaw-stage clevis",
    )
    ctx.allow_overlap(
        pitch_frame,
        roll_spindle,
        reason="roll spindle is represented as a short shaft captured inside the pitch-frame nose bearing",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(rear_support, yaw_stage, contact_tol=5e-4, name="support_mounts_yaw_stage")
    ctx.expect_contact(yaw_stage, pitch_frame, contact_tol=5e-4, name="yaw_stage_carries_pitch_frame")
    ctx.expect_contact(pitch_frame, roll_spindle, contact_tol=5e-4, name="pitch_frame_carries_roll_spindle")

    ctx.expect_origin_gap(
        pitch_frame,
        yaw_stage,
        axis="x",
        min_gap=0.056,
        max_gap=0.060,
        name="pitch_axis_sits_forward_of_yaw_stage",
    )
    ctx.expect_origin_gap(
        roll_spindle,
        pitch_frame,
        axis="x",
        min_gap=0.072,
        max_gap=0.076,
        name="roll_axis_sits_at_pitch_frame_tip",
    )

    ctx.check(
        "joint_axes_match_wrist_convention",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, -1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"expected axes ((0,0,1),(0,-1,0),(1,0,0)), got "
            f"{yaw_joint.axis}, {pitch_joint.axis}, {roll_joint.axis}"
        ),
    )

    with ctx.pose(yaw_joint=0.70):
        yawed_pitch_pos = ctx.part_world_position(pitch_frame)
    ctx.check(
        "positive_yaw_sweeps_pitch_frame_toward_positive_y",
        yawed_pitch_pos is not None and yawed_pitch_pos[1] > 0.035,
        details=f"pitch frame position under positive yaw was {yawed_pitch_pos}",
    )

    with ctx.pose(pitch_joint=0.70):
        pitched_roll_pos = ctx.part_world_position(roll_spindle)
    ctx.check(
        "positive_pitch_lifts_roll_tip",
        pitched_roll_pos is not None and pitched_roll_pos[2] > 0.040,
        details=f"roll spindle origin under positive pitch was {pitched_roll_pos}",
    )

    rest_roll_aabb = ctx.part_world_aabb(roll_spindle)
    with ctx.pose(roll_joint=1.20):
        rolled_roll_aabb = ctx.part_world_aabb(roll_spindle)
    roll_feature_swung = False
    if rest_roll_aabb is not None and rolled_roll_aabb is not None:
        rest_max_y = rest_roll_aabb[1][1]
        rolled_max_y = rolled_roll_aabb[1][1]
        roll_feature_swung = rolled_max_y > rest_max_y + 0.006
    ctx.check(
        "roll_joint_turns_offset_tip_feature",
        roll_feature_swung,
        details=f"rest aabb={rest_roll_aabb}, rolled aabb={rolled_roll_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
