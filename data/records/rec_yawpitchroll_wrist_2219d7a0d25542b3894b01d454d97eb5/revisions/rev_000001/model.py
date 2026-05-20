from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_three_axis_wrist")

    cast_iron = model.material("cast_iron", color=(0.24, 0.25, 0.27, 1.0))
    dark_steel = model.material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
    yaw_blue = model.material("yaw_blue", color=(0.05, 0.23, 0.46, 1.0))
    pitch_orange = model.material("pitch_orange", color=(0.90, 0.43, 0.11, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.68, 0.70, 0.72, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.46, 0.34, 0.04)),
        origin=Origin(xyz=(-0.12, 0.0, 0.02)),
        material=cast_iron,
        name="foot_plate",
    )
    rear_support.visual(
        Box((0.07, 0.25, 0.38)),
        origin=Origin(xyz=(-0.31, 0.0, 0.22)),
        material=cast_iron,
        name="rear_column",
    )
    rear_support.visual(
        Box((0.13, 0.32, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, 0.125)),
        material=cast_iron,
        name="front_saddle",
    )
    rear_support.visual(
        Box((0.26, 0.045, 0.05)),
        origin=Origin(xyz=(-0.15, 0.135, 0.125)),
        material=cast_iron,
        name="bridge_rib_0",
    )
    rear_support.visual(
        Box((0.26, 0.045, 0.05)),
        origin=Origin(xyz=(-0.15, -0.135, 0.125)),
        material=cast_iron,
        name="bridge_rib_1",
    )
    rear_support.visual(
        Cylinder(radius=0.115, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=dark_steel,
        name="stationary_bearing",
    )
    rear_support.visual(
        Box((0.26, 0.19, 0.045)),
        origin=Origin(xyz=(-0.18, 0.0, 0.375)),
        material=cast_iron,
        name="rear_bridge_cap",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.105, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=yaw_blue,
        name="turntable",
    )
    yaw_stage.visual(
        Cylinder(radius=0.048, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=yaw_blue,
        name="central_post",
    )
    yaw_stage.visual(
        Box((0.17, 0.030, 0.045)),
        origin=Origin(xyz=(0.085, 0.060, 0.115)),
        material=yaw_blue,
        name="pitch_bridge_0",
    )
    yaw_stage.visual(
        Box((0.17, 0.030, 0.045)),
        origin=Origin(xyz=(0.085, -0.060, 0.115)),
        material=yaw_blue,
        name="pitch_bridge_1",
    )
    yaw_stage.visual(
        Box((0.070, 0.028, 0.145)),
        origin=Origin(xyz=(0.185, 0.075, 0.160)),
        material=yaw_blue,
        name="pitch_cheek_0",
    )
    yaw_stage.visual(
        Box((0.070, 0.028, 0.145)),
        origin=Origin(xyz=(0.185, -0.075, 0.160)),
        material=yaw_blue,
        name="pitch_cheek_1",
    )
    yaw_stage.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.185, 0.097, 0.160), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_boss_0",
    )
    yaw_stage.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.185, -0.097, 0.160), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_boss_1",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        Cylinder(radius=0.020, length=0.126),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_shaft",
    )
    pitch_frame.visual(
        Cylinder(radius=0.038, length=0.070),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=pitch_orange,
        name="pitch_hub",
    )
    pitch_frame.visual(
        Box((0.175, 0.024, 0.030)),
        origin=Origin(xyz=(0.085, 0.043, 0.0)),
        material=pitch_orange,
        name="side_rail_0",
    )
    pitch_frame.visual(
        Box((0.175, 0.024, 0.030)),
        origin=Origin(xyz=(0.085, -0.043, 0.0)),
        material=pitch_orange,
        name="side_rail_1",
    )
    pitch_frame.visual(
        Box((0.035, 0.110, 0.030)),
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=pitch_orange,
        name="front_crossbar",
    )
    pitch_frame.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_bearing",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.024, length=0.110),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="spindle_shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="output_flange",
    )
    roll_spindle.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.120, 0.0, 0.045)),
        material=dark_steel,
        name="drive_key",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.185, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=-0.55, upper=0.80),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=4.0, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(str(j.articulation_type).lower().endswith("revolute") for j in object_model.articulations)
        and yaw.parent == "rear_support"
        and yaw.child == "yaw_stage"
        and pitch.parent == "yaw_stage"
        and pitch.child == "pitch_frame"
        and roll.parent == "pitch_frame"
        and roll.child == "roll_spindle",
        details="Expected rear_support -> yaw_stage -> pitch_frame -> roll_spindle revolute chain.",
    )
    ctx.expect_gap(
        yaw_stage,
        rear_support,
        axis="z",
        positive_elem="turntable",
        negative_elem="stationary_bearing",
        max_gap=0.001,
        max_penetration=0.00001,
        name="yaw turntable sits on rear bearing",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_frame,
        elem_a="pitch_cheek_0",
        elem_b="trunnion_shaft",
        reason="The pitch trunnion is intentionally captured slightly inside the cheek bearing proxy.",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_frame,
        elem_a="pitch_cheek_1",
        elem_b="trunnion_shaft",
        reason="The pitch trunnion is intentionally captured slightly inside the cheek bearing proxy.",
    )
    ctx.expect_gap(
        yaw_stage,
        pitch_frame,
        axis="y",
        positive_elem="pitch_cheek_0",
        negative_elem="trunnion_shaft",
        max_gap=0.001,
        max_penetration=0.003,
        name="positive pitch cheek captures trunnion with clearance",
    )
    ctx.expect_gap(
        pitch_frame,
        yaw_stage,
        axis="y",
        positive_elem="trunnion_shaft",
        negative_elem="pitch_cheek_1",
        max_gap=0.001,
        max_penetration=0.003,
        name="negative pitch cheek captures trunnion with clearance",
    )
    ctx.expect_gap(
        roll_spindle,
        pitch_frame,
        axis="x",
        positive_elem="spindle_shaft",
        negative_elem="roll_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="roll shaft seats against pitch bearing",
    )

    rest_pitch_pos = ctx.part_world_position(pitch_frame)
    with ctx.pose({yaw: 0.8}):
        yawed_pitch_pos = ctx.part_world_position(pitch_frame)
    ctx.check(
        "yaw sweeps pitch frame sideways",
        rest_pitch_pos is not None
        and yawed_pitch_pos is not None
        and abs(yawed_pitch_pos[1] - rest_pitch_pos[1]) > 0.10,
        details=f"rest={rest_pitch_pos}, yawed={yawed_pitch_pos}",
    )

    rest_roll_aabb = ctx.part_world_aabb(roll_spindle)
    with ctx.pose({pitch: 0.65}):
        pitched_roll_aabb = ctx.part_world_aabb(roll_spindle)
    ctx.check(
        "pitch lifts roll spindle",
        rest_roll_aabb is not None
        and pitched_roll_aabb is not None
        and pitched_roll_aabb[1][2] > rest_roll_aabb[1][2] + 0.06,
        details=f"rest={rest_roll_aabb}, pitched={pitched_roll_aabb}",
    )

    rest_key_aabb = ctx.part_element_world_aabb(roll_spindle, elem="drive_key")
    with ctx.pose({roll: 1.2}):
        rolled_key_aabb = ctx.part_element_world_aabb(roll_spindle, elem="drive_key")
    if rest_key_aabb is not None and rolled_key_aabb is not None:
        rest_key_center_y = 0.5 * (rest_key_aabb[0][1] + rest_key_aabb[1][1])
        rolled_key_center_y = 0.5 * (rolled_key_aabb[0][1] + rolled_key_aabb[1][1])
    else:
        rest_key_center_y = rolled_key_center_y = 0.0
    ctx.check(
        "roll key rotates about spindle axis",
        rest_key_aabb is not None
        and rolled_key_aabb is not None
        and abs(rolled_key_center_y - rest_key_center_y) > 0.030,
        details=f"rest={rest_key_aabb}, rolled={rolled_key_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
