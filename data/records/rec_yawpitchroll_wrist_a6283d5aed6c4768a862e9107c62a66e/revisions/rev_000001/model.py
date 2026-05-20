from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_roll_wrist")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.65, 0.67, 1.0))
    blue_anodized = Material("blue_anodized", rgba=(0.12, 0.30, 0.58, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    bolt_black = Material("bolt_black", rgba=(0.03, 0.03, 0.035, 1.0))

    base_yoke = model.part("base_yoke")
    base_yoke.visual(
        Box((0.36, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="ground_plate",
    )
    base_yoke.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=satin_steel,
        name="yaw_bearing_collar",
    )
    base_yoke.visual(
        Box((0.032, 0.230, 0.130)),
        origin=Origin(xyz=(-0.146, 0.0, 0.096)),
        material=dark_steel,
        name="rear_bridge",
    )
    for idx, y in enumerate((-0.114, 0.114)):
        base_yoke.visual(
            Box((0.280, 0.032, 0.200)),
            origin=Origin(xyz=(-0.010, y, 0.133)),
            material=dark_steel,
            name=f"yoke_cheek_{idx}",
        )
        base_yoke.visual(
            Cylinder(radius=0.016, length=0.280),
            origin=Origin(xyz=(-0.010, y, 0.228), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"rounded_cheek_{idx}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.024, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, -0.0065)),
        material=satin_steel,
        name="yaw_spigot",
    )
    yaw_stage.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=blue_anodized,
        name="lower_turntable",
    )
    yaw_stage.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=satin_steel,
        name="yaw_hub",
    )
    yaw_stage.visual(
        Box((0.160, 0.140, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.080)),
        material=blue_anodized,
        name="pitch_support_deck",
    )
    yaw_stage.visual(
        Box((0.075, 0.026, 0.110)),
        origin=Origin(xyz=(0.045, -0.070, 0.135)),
        material=blue_anodized,
        name="pitch_lug_0",
    )
    yaw_stage.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.045, -0.070, 0.150), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_bearing_0",
    )
    yaw_stage.visual(
        Box((0.075, 0.026, 0.110)),
        origin=Origin(xyz=(0.045, 0.070, 0.135)),
        material=blue_anodized,
        name="pitch_lug_1",
    )
    yaw_stage.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.045, 0.070, 0.150), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_bearing_1",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_trunnion",
    )
    for idx, y in enumerate((-0.034, 0.034)):
        pitch_cradle.visual(
            Box((0.090, 0.014, 0.060)),
            origin=Origin(xyz=(0.045, y, 0.0)),
            material=dark_steel,
            name=f"cradle_side_{idx}",
        )
    pitch_cradle.visual(
        Box((0.020, 0.082, 0.044)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=dark_steel,
        name="cradle_crosshead",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.040, length=0.065),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="roll_bearing_sleeve",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=0.026, length=0.145),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="cartridge_body",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.040, length=0.011),
        origin=Origin(xyz=(0.123, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_anodized,
        name="tip_flange",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.144, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="tool_pilot",
    )
    for idx, (y, z) in enumerate(((0.024, 0.024), (-0.024, 0.024), (-0.024, -0.024), (0.024, -0.024))):
        roll_cartridge.visual(
            Cylinder(radius=0.0042, length=0.005),
            origin=Origin(xyz=(0.130, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"flange_bolt_{idx}",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base_yoke,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-pi, upper=pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.045, 0.0, 0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_yoke = object_model.get_part("base_yoke")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.allow_overlap(
        base_yoke,
        yaw_stage,
        elem_a="yaw_bearing_collar",
        elem_b="yaw_spigot",
        reason="The yaw spigot is intentionally seated inside the fixed bearing collar.",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        elem_a="pitch_bearing_0",
        elem_b="pitch_trunnion",
        reason="The pitch trunnion is represented as captured in the bearing bore proxy.",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        elem_a="pitch_bearing_1",
        elem_b="pitch_trunnion",
        reason="The pitch trunnion is represented as captured in the bearing bore proxy.",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_cartridge,
        elem_a="roll_bearing_sleeve",
        elem_b="cartridge_body",
        reason="The roll cartridge body is intentionally nested inside the sleeve proxy.",
    )

    ctx.check(
        "three serial revolute joints",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and roll.articulation_type == ArticulationType.REVOLUTE
        and yaw.parent == "base_yoke"
        and yaw.child == "yaw_stage"
        and pitch.parent == "yaw_stage"
        and pitch.child == "pitch_cradle"
        and roll.parent == "pitch_cradle"
        and roll.child == "roll_cartridge",
        details=f"joints={(yaw.parent, yaw.child), (pitch.parent, pitch.child), (roll.parent, roll.child)}",
    )
    ctx.check(
        "yaw pitch roll axes",
        yaw.axis == (0.0, 0.0, 1.0)
        and pitch.axis == (0.0, -1.0, 0.0)
        and roll.axis == (1.0, 0.0, 0.0),
        details=f"axes={yaw.axis}, {pitch.axis}, {roll.axis}",
    )

    ctx.expect_gap(
        yaw_stage,
        base_yoke,
        axis="z",
        positive_elem="lower_turntable",
        negative_elem="yaw_bearing_collar",
        min_gap=0.0,
        max_gap=0.003,
        name="yaw turntable rests above base collar",
    )
    ctx.expect_within(
        yaw_stage,
        base_yoke,
        axes="xy",
        inner_elem="yaw_spigot",
        outer_elem="yaw_bearing_collar",
        margin=0.0,
        name="yaw spigot is centered inside collar",
    )
    ctx.expect_overlap(
        yaw_stage,
        base_yoke,
        axes="z",
        elem_a="yaw_spigot",
        elem_b="yaw_bearing_collar",
        min_overlap=0.010,
        name="yaw spigot has retained bearing insertion",
    )
    ctx.expect_gap(
        yaw_stage,
        pitch_cradle,
        axis="y",
        positive_elem="pitch_bearing_1",
        negative_elem="pitch_trunnion",
        max_penetration=0.006,
        name="positive pitch bearing captures trunnion",
    )
    ctx.expect_gap(
        pitch_cradle,
        yaw_stage,
        axis="y",
        positive_elem="pitch_trunnion",
        negative_elem="pitch_bearing_0",
        max_penetration=0.006,
        name="negative pitch bearing captures trunnion",
    )
    ctx.expect_within(
        roll_cartridge,
        pitch_cradle,
        axes="yz",
        inner_elem="cartridge_body",
        outer_elem="roll_bearing_sleeve",
        margin=0.0,
        name="roll cartridge stays coaxial in sleeve",
    )
    ctx.expect_overlap(
        roll_cartridge,
        pitch_cradle,
        axes="x",
        elem_a="cartridge_body",
        elem_b="roll_bearing_sleeve",
        min_overlap=0.060,
        name="roll cartridge retained through sleeve",
    )

    rest_pitch_pos = ctx.part_world_position(pitch_cradle)
    with ctx.pose({yaw: 0.65}):
        yawed_pitch_pos = ctx.part_world_position(pitch_cradle)
    ctx.check(
        "yaw swings pitch cradle around vertical axis",
        rest_pitch_pos is not None
        and yawed_pitch_pos is not None
        and abs(yawed_pitch_pos[1] - rest_pitch_pos[1]) > 0.020,
        details=f"rest={rest_pitch_pos}, yawed={yawed_pitch_pos}",
    )

    rest_roll_aabb = ctx.part_element_world_aabb(roll_cartridge, elem="tip_flange")
    with ctx.pose({pitch: 0.75}):
        pitched_roll_aabb = ctx.part_element_world_aabb(roll_cartridge, elem="tip_flange")
    ctx.check(
        "pitch tips the roll cartridge upward",
        rest_roll_aabb is not None
        and pitched_roll_aabb is not None
        and pitched_roll_aabb[1][2] > rest_roll_aabb[1][2] + 0.020,
        details=f"rest={rest_roll_aabb}, pitched={pitched_roll_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
