from __future__ import annotations

import math

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
    model = ArticulatedObject(name="cartridge_yoke_wrist")

    cast_iron = Material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    parkerized = Material("parkerized_steel", rgba=(0.23, 0.25, 0.27, 1.0))
    bearing_black = Material("black_bearing", rgba=(0.02, 0.025, 0.028, 1.0))
    blue_anodized = Material("blue_anodized", rgba=(0.05, 0.18, 0.38, 1.0))
    brushed = Material("brushed_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    brass = Material("brass_bushing", rgba=(0.84, 0.62, 0.26, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.20, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="floor_flange",
    )
    base.visual(
        Box((0.26, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=cast_iron,
        name="mount_plinth",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=parkerized,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.1975)),
        material=bearing_black,
        name="fixed_bearing",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.120, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=parkerized,
        name="turntable",
    )
    yaw_stage.visual(
        Cylinder(radius=0.048, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=parkerized,
        name="rotary_neck",
    )
    yaw_stage.visual(
        Box((0.170, 0.185, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=parkerized,
        name="yoke_foot",
    )
    yaw_stage.visual(
        Box((0.040, 0.185, 0.170)),
        origin=Origin(xyz=(-0.080, 0.0, 0.235)),
        material=parkerized,
        name="rear_bridge",
    )
    for y, suffix in ((0.083, "0"), (-0.083, "1")):
        yaw_stage.visual(
            Box((0.112, 0.026, 0.185)),
            origin=Origin(xyz=(0.010, y, 0.2475)),
            material=parkerized,
            name=f"yoke_cheek_{suffix}",
        )
        yaw_stage.visual(
            Cylinder(radius=0.036, length=0.014),
            origin=Origin(xyz=(0.0, y * 0.83, 0.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"pitch_bearing_{suffix}",
        )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.017, length=0.132),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="pitch_shaft",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.039, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_anodized,
        name="pitch_hub",
    )
    pitch_cradle.visual(
        Box((0.150, 0.080, 0.055)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=blue_anodized,
        name="cradle_spine",
    )
    for y, suffix in ((0.045, "0"), (-0.045, "1")):
        pitch_cradle.visual(
            Box((0.080, 0.018, 0.050)),
            origin=Origin(xyz=(0.170, y, 0.0)),
            material=blue_anodized,
            name=f"roll_side_rail_{suffix}",
        )
    pitch_cradle.visual(
        Cylinder(radius=0.057, length=0.052),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="roll_bearing",
    )
    pitch_cradle.visual(
        Box((0.070, 0.108, 0.022)),
        origin=Origin(xyz=(0.185, 0.0, 0.059)),
        material=blue_anodized,
        name="upper_clamp",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=0.018, length=0.076),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="roll_shaft",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.041, length=0.095),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_anodized,
        name="cartridge_body",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.055, length=0.026),
        origin=Origin(xyz=(0.1365, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="front_flange",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.030, length=0.058),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_black,
        name="tool_socket",
    )
    roll_cartridge.visual(
        Box((0.040, 0.012, 0.010)),
        origin=Origin(xyz=(0.077, 0.0, 0.045)),
        material=brushed,
        name="index_mark",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.3, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.allow_overlap(
        pitch_cradle,
        roll_cartridge,
        elem_a="roll_bearing",
        elem_b="roll_shaft",
        reason="The compact roll shaft is intentionally captured inside the pitch-cradle bearing proxy.",
    )

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check("yaw axis is vertical", tuple(yaw.axis) == (0.0, 0.0, 1.0), details=f"axis={yaw.axis}")
    ctx.check("pitch axis is a cross axis", tuple(pitch.axis) == (0.0, -1.0, 0.0), details=f"axis={pitch.axis}")
    ctx.check("roll axis is the tool axis", tuple(roll.axis) == (1.0, 0.0, 0.0), details=f"axis={roll.axis}")

    ctx.expect_contact(
        base,
        yaw_stage,
        elem_a="fixed_bearing",
        elem_b="turntable",
        contact_tol=0.001,
        name="yaw stage seats on the grounded bearing",
    )
    ctx.expect_within(
        roll_cartridge,
        pitch_cradle,
        axes="yz",
        inner_elem="roll_shaft",
        outer_elem="roll_bearing",
        margin=0.001,
        name="roll shaft is centered in the bearing bore",
    )
    ctx.expect_overlap(
        roll_cartridge,
        pitch_cradle,
        axes="x",
        elem_a="roll_shaft",
        elem_b="roll_bearing",
        min_overlap=0.045,
        name="roll shaft remains retained in the bearing",
    )

    rest_roll_pos = ctx.part_world_position(roll_cartridge)
    with ctx.pose({yaw: 0.75}):
        yawed_roll_pos = ctx.part_world_position(roll_cartridge)
    ctx.check(
        "yaw rotates the wrist about the base",
        rest_roll_pos is not None
        and yawed_roll_pos is not None
        and abs(yawed_roll_pos[1] - rest_roll_pos[1]) > 0.08,
        details=f"rest={rest_roll_pos}, yawed={yawed_roll_pos}",
    )

    with ctx.pose({pitch: 0.55}):
        pitched_roll_pos = ctx.part_world_position(roll_cartridge)
    ctx.check(
        "positive pitch lifts the roll cartridge",
        rest_roll_pos is not None
        and pitched_roll_pos is not None
        and pitched_roll_pos[2] > rest_roll_pos[2] + 0.07,
        details=f"rest={rest_roll_pos}, pitched={pitched_roll_pos}",
    )

    return ctx.report()


object_model = build_object_model()
