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
    model = ArticulatedObject(name="cartesian_fed_linkage")

    cast_iron = Material("cast_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    rail_steel = Material("ground_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_steel = Material("black_oxide", rgba=(0.04, 0.045, 0.05, 1.0))
    carriage_blue = Material("carriage_blue", rgba=(0.05, 0.20, 0.44, 1.0))
    arm_white = Material("powder_coat_white", rgba=(0.86, 0.88, 0.86, 1.0))
    safety_orange = Material("safety_orange", rgba=(0.95, 0.36, 0.06, 1.0))
    tool_yellow = Material("tool_yellow", rgba=(0.95, 0.72, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.20, 0.45, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cast_iron,
        name="machine_bed",
    )
    base.visual(
        Box((1.05, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.16, 0.0775)),
        material=rail_steel,
        name="rail_0",
    )
    base.visual(
        Box((1.05, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.16, 0.0775)),
        material=rail_steel,
        name="rail_1",
    )
    for x, name in ((-0.575, "end_stop_0"), (0.575, "end_stop_1")):
        base.visual(
            Box((0.05, 0.42, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.10)),
            material=dark_steel,
            name=name,
        )
    base.visual(
        Box((1.02, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.245, 0.0825)),
        material=dark_steel,
        name="cable_tray",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.26, 0.36, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=carriage_blue,
        name="saddle_plate",
    )
    for x, name in ((-0.085, "bearing_pad_0"), (0.085, "bearing_pad_1")):
        carriage.visual(
            Box((0.075, 0.32, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0525)),
            material=dark_steel,
            name=name,
        )
    carriage.visual(
        Cylinder(radius=0.080, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=carriage_blue,
        name="pivot_pedestal",
    )
    carriage.visual(
        Cylinder(radius=0.122, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1325)),
        material=rail_steel,
        name="lower_bearing",
    )

    rotary_arm = model.part("rotary_arm")
    rotary_arm.visual(
        Cylinder(radius=0.115, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=rail_steel,
        name="upper_hub",
    )
    rotary_arm.visual(
        Box((0.110, 0.56, 0.055)),
        origin=Origin(xyz=(0.0, 0.315, 0.0275)),
        material=arm_white,
        name="arm_beam",
    )
    rotary_arm.visual(
        Box((0.025, 0.42, 0.025)),
        origin=Origin(xyz=(-0.040, 0.34, 0.0675)),
        material=rail_steel,
        name="tool_rail_0",
    )
    rotary_arm.visual(
        Box((0.025, 0.42, 0.025)),
        origin=Origin(xyz=(0.040, 0.34, 0.0675)),
        material=rail_steel,
        name="tool_rail_1",
    )
    rotary_arm.visual(
        Box((0.16, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, 0.58, 0.045)),
        material=safety_orange,
        name="distal_stop",
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        Box((0.18, 0.16, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=safety_orange,
        name="slide_block",
    )
    tool_slide.visual(
        Box((0.15, 0.060, 0.035)),
        origin=Origin(xyz=(0.105, 0.070, 0.0375)),
        material=safety_orange,
        name="side_outrigger",
    )
    tool_slide.visual(
        Box((0.055, 0.050, 0.180)),
        origin=Origin(xyz=(0.155, 0.080, -0.055)),
        material=tool_yellow,
        name="tool_carrier",
    )
    tool_slide.visual(
        Cylinder(radius=0.025, length=0.200),
        origin=Origin(xyz=(0.155, 0.080, -0.190)),
        material=dark_steel,
        name="vertical_tool",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.35, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.40, lower=0.0, upper=0.55),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rotary_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.75 * pi, upper=0.75 * pi),
    )
    model.articulation(
        "arm_to_tool_slide",
        ArticulationType.PRISMATIC,
        parent=rotary_arm,
        child=tool_slide,
        origin=Origin(xyz=(0.0, 0.24, 0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    rotary_arm = object_model.get_part("rotary_arm")
    tool_slide = object_model.get_part("tool_slide")
    base_slide = object_model.get_articulation("base_to_carriage")
    pivot = object_model.get_articulation("carriage_to_arm")
    tool_axis = object_model.get_articulation("arm_to_tool_slide")

    ctx.check("base slider is prismatic", base_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("center joint is revolute", pivot.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("tool slide is prismatic", tool_axis.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="saddle_plate",
        negative_elem="rail_0",
        name="saddle rests on base rail",
    )
    ctx.expect_gap(
        rotary_arm,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="upper_hub",
        negative_elem="lower_bearing",
        name="rotary hub sits on bearing",
    )
    ctx.expect_gap(
        tool_slide,
        rotary_arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="slide_block",
        negative_elem="tool_rail_0",
        name="tool slide rides on distal rail",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: 0.55}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="saddle_plate",
            elem_b="rail_0",
            name="base slide retains rail engagement",
        )

    ctx.check(
        "base carriage travels along x",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.50,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_tool = ctx.part_world_position(tool_slide)
    with ctx.pose({tool_axis: 0.18}):
        extended_tool = ctx.part_world_position(tool_slide)
        ctx.expect_overlap(
            tool_slide,
            rotary_arm,
            axes="y",
            min_overlap=0.08,
            elem_a="slide_block",
            elem_b="tool_rail_0",
            name="distal slide retains rail engagement",
        )

    ctx.check(
        "tool slide feeds distally",
        rest_tool is not None
        and extended_tool is not None
        and extended_tool[1] > rest_tool[1] + 0.16,
        details=f"rest={rest_tool}, extended={extended_tool}",
    )

    with ctx.pose({pivot: 0.60}):
        turned_aabb = ctx.part_world_aabb(tool_slide)
    ctx.check(
        "center pivot swings the distal slide",
        turned_aabb is not None and turned_aabb[0][0] < -0.05,
        details=f"turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
