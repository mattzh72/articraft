from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.120
BASE_WIDTH = 0.090
BASE_THICKNESS = 0.014

YAW_STAGE_RADIUS = 0.039
YAW_STAGE_HEIGHT = 0.036

PITCH_AXIS_X = 0.060
PITCH_AXIS_Z = 0.038

ROLL_AXIS_X = 0.060
ROLL_SPINDLE_LENGTH = 0.046


def _rear_support_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS).translate(
        (-0.018, 0.0, -0.043)
    )
    lower_bridge = cq.Workplane("XY").box(0.052, 0.052, 0.016).translate(
        (-0.047, 0.0, -0.010)
    )
    rear_web = cq.Workplane("XY").box(0.018, 0.052, 0.094).translate(
        (-0.072, 0.0, 0.005)
    )
    left_upright = cq.Workplane("XY").box(0.012, 0.012, 0.100).translate(
        (-0.060, 0.030, 0.024)
    )
    right_upright = cq.Workplane("XY").box(0.012, 0.012, 0.100).translate(
        (-0.060, -0.030, 0.024)
    )
    top_bridge = cq.Workplane("XY").box(0.018, 0.068, 0.012).translate(
        (-0.060, 0.0, 0.074)
    )
    rear_spine = cq.Workplane("XY").box(0.018, 0.026, 0.050).translate(
        (-0.078, 0.0, 0.022)
    )

    shape = (
        base.union(lower_bridge)
        .union(rear_web)
        .union(left_upright)
        .union(right_upright)
        .union(top_bridge)
        .union(rear_spine)
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.050, -0.028),
                (-0.050, 0.028),
                (0.014, -0.028),
                (0.014, 0.028),
            ]
        )
        .circle(0.0045)
        .extrude(0.024)
        .translate((0.0, 0.0, -0.055))
    )
    return shape.cut(mount_holes)


def _yaw_stage_shape() -> cq.Workplane:
    upper_disc = (
        cq.Workplane("XY").circle(0.031).extrude(0.018).translate((0.0, 0.0, 0.027))
    )
    forward_body = cq.Workplane("XY").box(
        0.040, 0.028, 0.018, centered=(False, True, False)
    ).translate((0.0, 0.0, 0.029))
    left_cheek = cq.Workplane("XY").box(
        0.023, 0.009, 0.052, centered=(False, True, False)
    ).translate((0.040, 0.0165, 0.012))
    right_cheek = cq.Workplane("XY").box(
        0.023, 0.009, 0.052, centered=(False, True, False)
    ).translate((0.040, -0.0165, 0.012))
    rear_cap = cq.Workplane("XY").box(
        0.014, 0.026, 0.016, centered=(False, True, False)
    ).translate((0.010, 0.0, 0.034))
    return (
        upper_disc.union(forward_body)
        .union(left_cheek)
        .union(right_cheek)
        .union(rear_cap)
    )


def _pitch_frame_shape() -> cq.Workplane:
    profile = (
        cq.Sketch()
        .rect(0.048, 0.044, tag="outer")
        .vertices(tag="outer")
        .fillet(0.006)
        .reset()
        .rect(0.026, 0.020, mode="s")
    )
    frame = (
        cq.Workplane("XZ")
        .center(0.032, 0.0)
        .placeSketch(profile)
        .extrude(0.009, both=True)
    )
    trunnion_block = cq.Workplane("XY").box(
        0.008, 0.024, 0.014, centered=(False, True, True)
    ).translate((0.0, 0.0, 0.0))
    neck = cq.Workplane("XY").box(
        0.012, 0.018, 0.016, centered=(False, True, True)
    ).translate((0.004, 0.0, 0.0))
    nose = cq.Workplane("XZ").center(0.050, 0.0).circle(0.010).extrude(0.009, both=True)
    return frame.union(trunnion_block).union(neck).union(nose)


def _roll_spindle_shape() -> cq.Workplane:
    main_body = cq.Workplane("YZ").circle(0.010).extrude(ROLL_SPINDLE_LENGTH)
    rear_collar = cq.Workplane("YZ").circle(0.013).extrude(0.005)
    front_flange = cq.Workplane("YZ").circle(0.014).extrude(0.006).translate(
        (0.036, 0.0, 0.0)
    )
    drive_lug = cq.Workplane("XY").box(
        0.018, 0.006, 0.010, centered=(False, True, False)
    ).translate((0.010, 0.0, 0.008))
    return main_body.union(rear_collar).union(front_flange).union(drive_lug)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_three_axis_wrist")

    model.material("support_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("stage_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("frame_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("spindle_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        material="support_gray",
        name="rear_support_shell",
    )
    rear_support.visual(
        Cylinder(radius=0.022, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="support_gray",
        name="yaw_support_post",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage"),
        material="stage_black",
        name="yaw_stage_shell",
    )
    yaw_stage.visual(
        Cylinder(radius=YAW_STAGE_RADIUS, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="stage_black",
        name="yaw_lower_hub",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_pitch_frame_shape(), "pitch_frame"),
        material="frame_silver",
        name="pitch_frame_shell",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(_roll_spindle_shape(), "roll_spindle"),
        material="spindle_steel",
        name="roll_spindle_shell",
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.2,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.8,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=5.0,
            lower=-pi,
            upper=pi,
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

    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")

    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    ctx.check(
        "three serial revolute joints are present",
        len(object_model.articulations) == 3
        and yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and roll_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_types={[joint.articulation_type for joint in object_model.articulations]}",
    )
    ctx.check(
        "joint axes match yaw pitch roll convention",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, -1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details=f"yaw={yaw_joint.axis}, pitch={pitch_joint.axis}, roll={roll_joint.axis}",
    )

    ctx.expect_overlap(
        rear_support,
        yaw_stage,
        axes="xy",
        min_overlap=0.040,
        name="yaw stage remains seated over the rear support footprint",
    )
    ctx.expect_overlap(
        pitch_frame,
        yaw_stage,
        axes="y",
        min_overlap=0.015,
        name="pitch frame stays captured within the yaw stage width",
    )

    rest_pos = ctx.part_world_position(roll_spindle)
    with ctx.pose({yaw_joint: 0.75}):
        yawed_pos = ctx.part_world_position(roll_spindle)
    ctx.check(
        "yaw swings the wrist laterally about the support axis",
        rest_pos is not None
        and yawed_pos is not None
        and abs(yawed_pos[1] - rest_pos[1]) > 0.070
        and yawed_pos[0] < rest_pos[0] - 0.020,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    with ctx.pose({pitch_joint: 0.70}):
        pitched_pos = ctx.part_world_position(roll_spindle)
    ctx.check(
        "positive pitch raises the roll spindle tip",
        rest_pos is not None
        and pitched_pos is not None
        and pitched_pos[2] > rest_pos[2] + 0.038,
        details=f"rest={rest_pos}, pitched={pitched_pos}",
    )

    rest_aabb = ctx.part_world_aabb(roll_spindle)
    with ctx.pose({roll_joint: pi / 2.0}):
        rolled_aabb = ctx.part_world_aabb(roll_spindle)

    def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        low, high = aabb
        return (high[0] - low[0], high[1] - low[1], high[2] - low[2])

    rest_span = _span(rest_aabb)
    rolled_span = _span(rolled_aabb)
    ctx.check(
        "roll rotates the spindle's asymmetric drive lug",
        rest_span is not None
        and rolled_span is not None
        and rolled_span[1] > rest_span[1] + 0.003
        and rolled_span[2] < rest_span[2] - 0.003,
        details=f"rest_span={rest_span}, rolled_span={rolled_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
