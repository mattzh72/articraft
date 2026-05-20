from __future__ import annotations

import math

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
    model = ArticulatedObject(name="under_slung_three_axis_wrist")

    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.055, 0.06, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.01, 0.012, 0.014, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.300, 0.210, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=dark_anodized,
        name="mounting_plate",
    )
    top_support.visual(
        Cylinder(radius=0.056, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="fixed_yaw_bearing",
    )
    top_support.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=brushed_steel,
        name="top_bearing_cap",
    )
    for i, (x, y) in enumerate(
        ((-0.110, -0.070), (-0.110, 0.070), (0.110, -0.070), (0.110, 0.070))
    ):
        top_support.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(x, y, 0.077)),
            material=brushed_steel,
            name=f"plate_screw_{i}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.045, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=graphite,
        name="rotating_yaw_hub",
    )
    yaw_stage.visual(
        Cylinder(radius=0.070, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=dark_anodized,
        name="yaw_turntable",
    )
    yaw_stage.visual(
        Box((0.046, 0.072, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=dark_anodized,
        name="drop_web",
    )
    yaw_stage.visual(
        Box((0.174, 0.054, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=dark_anodized,
        name="pitch_fork_bridge",
    )
    yaw_stage.visual(
        Box((0.018, 0.096, 0.122)),
        origin=Origin(xyz=(-0.075, 0.0, -0.205)),
        material=dark_anodized,
        name="pitch_fork_side_0",
    )
    yaw_stage.visual(
        Box((0.018, 0.096, 0.122)),
        origin=Origin(xyz=(0.075, 0.0, -0.205)),
        material=dark_anodized,
        name="pitch_fork_side_1",
    )
    for suffix, x in (("0", -0.088), ("1", 0.088)):
        yaw_stage.visual(
            Cylinder(radius=0.033, length=0.009),
            origin=Origin(xyz=(x, 0.0, -0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"pitch_bearing_cap_{suffix}",
        )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.026, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pitch_trunnion",
    )
    pitch_cradle.visual(
        Box((0.104, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=safety_orange,
        name="cradle_crosshead",
    )
    for suffix, x in (("0", -0.040), ("1", 0.040)):
        pitch_cradle.visual(
            Box((0.018, 0.040, 0.108)),
            origin=Origin(xyz=(x, 0.0, -0.068)),
            material=safety_orange,
            name=f"cradle_side_arm_{suffix}",
        )
    pitch_cradle.visual(
        Cylinder(radius=0.034, length=0.075),
        origin=Origin(xyz=(0.0, -0.0375, -0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="roll_bearing",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.0, 0.002, -0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="roll_face_ring",
    )

    roll_output = model.part("roll_output")
    roll_output.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="output_shaft",
    )
    roll_output.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(xyz=(0.0, 0.087, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="tool_flange",
    )
    roll_output.visual(
        Cylinder(radius=0.014, length=0.035),
        origin=Origin(xyz=(0.0, 0.114, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pilot_nose",
    )
    for i, (x, z) in enumerate(((0.022, 0.000), (-0.022, 0.000), (0.0, 0.022), (0.0, -0.022))):
        roll_output.visual(
            Cylinder(radius=0.0045, length=0.007),
            origin=Origin(xyz=(x, 0.100, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_output,
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top_support = object_model.get_part("top_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_output = object_model.get_part("roll_output")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    ctx.allow_overlap(
        pitch_cradle,
        roll_output,
        elem_a="roll_face_ring",
        elem_b="output_shaft",
        reason="The roll shaft is intentionally captured through the cradle's bearing face ring.",
    )

    ctx.check(
        "three serial revolute stages",
        len(object_model.parts) == 4
        and len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in (yaw_joint, pitch_joint, roll_joint)),
        details="Expected top support, yaw stage, pitch cradle, and roll output with three revolute joints.",
    )
    ctx.check("yaw axis is vertical", tuple(yaw_joint.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch axis is horizontal", tuple(pitch_joint.axis) == (1.0, 0.0, 0.0))
    ctx.check("roll axis follows tool axis", tuple(roll_joint.axis) == (0.0, 1.0, 0.0))

    ctx.expect_gap(
        top_support,
        yaw_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="fixed_yaw_bearing",
        negative_elem="rotating_yaw_hub",
        name="yaw hub is carried directly under the fixed support bearing",
    )
    ctx.expect_overlap(
        yaw_stage,
        top_support,
        axes="xy",
        min_overlap=0.040,
        elem_a="rotating_yaw_hub",
        elem_b="fixed_yaw_bearing",
        name="yaw bearing footprints are coaxial",
    )
    ctx.expect_within(
        pitch_cradle,
        yaw_stage,
        axes="x",
        margin=0.0,
        inner_elem="pitch_trunnion",
        outer_elem="pitch_fork_bridge",
        name="pitch trunnion fits between yaw fork cheeks",
    )
    ctx.expect_overlap(
        pitch_cradle,
        yaw_stage,
        axes="z",
        min_overlap=0.040,
        elem_a="pitch_trunnion",
        elem_b="pitch_fork_side_0",
        name="pitch trunnion height aligns with fork bearing caps",
    )
    ctx.expect_gap(
        roll_output,
        pitch_cradle,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="output_shaft",
        negative_elem="roll_bearing",
        name="roll output exits from the cradle bearing face",
    )
    ctx.expect_overlap(
        roll_output,
        pitch_cradle,
        axes="y",
        min_overlap=0.005,
        elem_a="output_shaft",
        elem_b="roll_face_ring",
        name="roll shaft is retained through the face ring",
    )

    rest_roll_origin = ctx.part_world_position(roll_output)
    with ctx.pose({pitch_joint: 0.60, yaw_joint: 0.35, roll_joint: 0.90}):
        pitched_roll_origin = ctx.part_world_position(roll_output)
        ctx.expect_overlap(
            roll_output,
            pitch_cradle,
            axes="y",
            min_overlap=0.010,
            elem_a="output_shaft",
            elem_b="roll_face_ring",
            name="roll shaft stays seated at an articulated pose",
        )
    ctx.check(
        "pitch motion swings the roll axis below the yaw stage",
        rest_roll_origin is not None
        and pitched_roll_origin is not None
        and pitched_roll_origin[1] > rest_roll_origin[1] + 0.040
        and pitched_roll_origin[2] > rest_roll_origin[2] + 0.010,
        details=f"rest={rest_roll_origin}, posed={pitched_roll_origin}",
    )

    return ctx.report()


object_model = build_object_model()
