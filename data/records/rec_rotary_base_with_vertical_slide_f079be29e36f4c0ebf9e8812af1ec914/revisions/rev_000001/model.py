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
    model = ArticulatedObject(name="rotary_pedestal_runner")

    cast_iron = Material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    black = Material("blackened_steel", rgba=(0.02, 0.025, 0.03, 1.0))
    machined = Material("machined_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rail_steel = Material("polished_runner_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    blue = Material("painted_carriage_blue", rgba=(0.05, 0.20, 0.42, 1.0))
    brass = Material("brass_bolts", rgba=(0.78, 0.55, 0.22, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.42, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.24, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=black,
        name="bearing_boss",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.32, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=machined,
        name="turntable_disk",
    )
    lower_stage.visual(
        Cylinder(radius=0.34, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=black,
        name="raised_rim",
    )
    lower_stage.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=machined,
        name="center_cap",
    )

    guide_block = model.part("guide_block")
    guide_block.visual(
        Box((0.20, 0.26, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=black,
        name="mounting_foot",
    )
    guide_block.visual(
        Box((0.10, 0.20, 0.700)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=cast_iron,
        name="upright_block",
    )
    guide_block.visual(
        Cylinder(radius=0.012, length=0.660),
        origin=Origin(xyz=(0.060, -0.055, 0.370)),
        material=rail_steel,
        name="rail_0",
    )
    guide_block.visual(
        Cylinder(radius=0.012, length=0.660),
        origin=Origin(xyz=(0.060, 0.055, 0.370)),
        material=rail_steel,
        name="rail_1",
    )
    guide_block.visual(
        Box((0.036, 0.165, 0.025)),
        origin=Origin(xyz=(0.064, 0.0, 0.080)),
        material=rail_steel,
        name="lower_rail_clamp",
    )
    guide_block.visual(
        Box((0.036, 0.165, 0.025)),
        origin=Origin(xyz=(0.064, 0.0, 0.660)),
        material=rail_steel,
        name="upper_rail_clamp",
    )
    for x in (-0.060, 0.060):
        for y in (-0.085, 0.085):
            guide_block.visual(
                Cylinder(radius=0.013, length=0.012),
                origin=Origin(xyz=(x, y, 0.051)),
                material=brass,
                name=f"foot_bolt_{x}_{y}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.070, 0.230, 0.160)),
        origin=Origin(),
        material=blue,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.025, 0.170, 0.110)),
        origin=Origin(xyz=(0.0475, 0.0, 0.0)),
        material=machined,
        name="front_tool_plate",
    )
    carriage.visual(
        Box((0.078, 0.250, 0.018)),
        origin=Origin(xyz=(0.005, 0.0, -0.060)),
        material=black,
        name="lower_clamp_band",
    )
    carriage.visual(
        Box((0.078, 0.250, 0.018)),
        origin=Origin(xyz=(0.005, 0.0, 0.060)),
        material=black,
        name="upper_clamp_band",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_stage_to_guide_block",
        ArticulationType.FIXED,
        parent=lower_stage,
        child=guide_block,
        origin=Origin(xyz=(0.190, 0.0, 0.085)),
    )
    model.articulation(
        "guide_block_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_block,
        child=carriage,
        origin=Origin(xyz=(0.107, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.320),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    guide_block = object_model.get_part("guide_block")
    carriage = object_model.get_part("carriage")
    rotary = object_model.get_articulation("base_to_lower_stage")
    lift = object_model.get_articulation("guide_block_to_carriage")

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="turntable_disk",
        negative_elem="bearing_boss",
        name="turntable rests on bearing boss",
    )
    ctx.expect_gap(
        guide_block,
        lower_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="mounting_foot",
        negative_elem="raised_rim",
        name="guide block bolts to rotating stage",
    )
    ctx.expect_gap(
        carriage,
        guide_block,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="carriage_body",
        negative_elem="rail_0",
        name="carriage rides against the front rail",
    )
    ctx.expect_overlap(
        carriage,
        guide_block,
        axes="z",
        min_overlap=0.120,
        elem_a="carriage_body",
        elem_b="rail_0",
        name="short carriage overlaps the vertical runner",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.300}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            guide_block,
            axes="z",
            min_overlap=0.120,
            elem_a="carriage_body",
            elem_b="rail_0",
            name="raised carriage remains on the runner",
        )
    ctx.check(
        "prismatic joint lifts carriage vertically",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and abs(raised_carriage_pos[0] - rest_carriage_pos[0]) < 0.001
        and abs(raised_carriage_pos[1] - rest_carriage_pos[1]) < 0.001
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.25,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_guide_pos = ctx.part_world_position(guide_block)
    with ctx.pose({rotary: 1.0}):
        rotated_guide_pos = ctx.part_world_position(guide_block)
    ctx.check(
        "revolute base sweeps the upright runner",
        rest_guide_pos is not None
        and rotated_guide_pos is not None
        and abs(rest_guide_pos[2] - rotated_guide_pos[2]) < 0.001
        and rotated_guide_pos[1] > rest_guide_pos[1] + 0.07,
        details=f"rest={rest_guide_pos}, rotated={rotated_guide_pos}",
    )

    return ctx.report()


object_model = build_object_model()
