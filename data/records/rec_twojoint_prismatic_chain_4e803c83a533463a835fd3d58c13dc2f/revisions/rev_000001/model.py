from __future__ import annotations

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
    model = ArticulatedObject(name="nested_two_carriage_transfer_chain")

    dark = model.material("black_anodized", rgba=(0.03, 0.035, 0.04, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    rail = model.material("ground_linear_rail", rgba=(0.78, 0.80, 0.82, 1.0))
    blue = model.material("blue_first_carriage", rgba=(0.05, 0.18, 0.38, 1.0))
    amber = model.material("amber_output_carriage", rgba=(0.93, 0.55, 0.08, 1.0))
    rubber = model.material("red_rubber_stops", rgba=(0.72, 0.05, 0.03, 1.0))

    guide = model.part("guide_frame")
    # A welded, fixed base frame with two raised precision rails along +X.
    guide.visual(
        Box((1.36, 0.055, 0.080)),
        origin=Origin(xyz=(0.0, -0.300, 0.040)),
        material=dark,
        name="side_beam_0",
    )
    guide.visual(
        Box((1.36, 0.055, 0.080)),
        origin=Origin(xyz=(0.0, 0.300, 0.040)),
        material=dark,
        name="side_beam_1",
    )
    for i, x in enumerate((-0.580, 0.580)):
        guide.visual(
            Box((0.080, 0.660, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=dark,
            name=f"end_crossmember_{i}",
        )
    guide.visual(
        Box((1.12, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark,
        name="center_tie",
    )
    for name, y in (("guide_rail_0", -0.205), ("guide_rail_1", 0.205)):
        guide.visual(
            Box((1.16, 0.075, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.105)),
            material=steel,
            name=f"{name}_plinth",
        )
        guide.visual(
            Box((1.12, 0.040, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.150)),
            material=rail,
            name=name,
        )
    for i, x in enumerate((-0.555, 0.555)):
        guide.visual(
            Box((0.035, 0.505, 0.110)),
            origin=Origin(xyz=(x, 0.0, 0.205)),
            material=rubber,
            name=f"guide_stop_{i}",
        )

    first = model.part("first_carriage")
    # Broad first stage: a wide table riding on four guide bearings.
    first.visual(
        Box((0.560, 0.485, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="wide_plate",
    )
    for name, x, y in (
        ("guide_bearing_0_0", -0.175, -0.205),
        ("guide_bearing_0_1", -0.175, 0.205),
        ("guide_bearing_1_0", 0.175, -0.205),
        ("guide_bearing_1_1", 0.175, 0.205),
    ):
            first.visual(
                Box((0.120, 0.080, 0.055)),
                origin=Origin(xyz=(x, y, -0.045)),
                material=steel,
                name=name,
            )
    # The nested output-axis rails are mounted on top of the first carriage.
    for name, y in (("nested_rail_0", -0.110), ("nested_rail_1", 0.110)):
        first.visual(
            Box((0.470, 0.026, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=rail,
            name=name,
        )
    for i, x in enumerate((-0.250, 0.250)):
        first.visual(
            Box((0.026, 0.290, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.050)),
            material=rubber,
            name=f"nested_stop_{i}",
        )

    output = model.part("output_carriage")
    # Smaller output slide carried by the first stage.
    output.visual(
        Box((0.250, 0.235, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=amber,
        name="small_plate",
    )
    for name, x, y in (
        ("nested_bearing_0_0", -0.070, -0.110),
        ("nested_bearing_0_1", -0.070, 0.110),
        ("nested_bearing_1_0", 0.070, -0.110),
        ("nested_bearing_1_1", 0.070, 0.110),
    ):
            output.visual(
                Box((0.075, 0.045, 0.035)),
                origin=Origin(xyz=(x, y, -0.035)),
                material=steel,
                name=name,
            )
    output.visual(
        Box((0.120, 0.100, 0.025)),
        origin=Origin(xyz=(0.145, 0.0, 0.005)),
        material=amber,
        name="output_tongue",
    )
    output.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.035)),
        material=steel,
        name="tooling_boss",
    )

    model.articulation(
        "guide_to_first",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=first,
        origin=Origin(xyz=(-0.240, 0.0, 0.2425)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=0.0, upper=0.380),
    )
    model.articulation(
        "first_to_output",
        ArticulationType.PRISMATIC,
        parent=first,
        child=output,
        origin=Origin(xyz=(-0.100, 0.0, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_frame")
    first = object_model.get_part("first_carriage")
    output = object_model.get_part("output_carriage")
    first_slide = object_model.get_articulation("guide_to_first")
    output_slide = object_model.get_articulation("first_to_output")

    ctx.check(
        "two serial prismatic joints",
        first_slide.articulation_type == ArticulationType.PRISMATIC
        and output_slide.articulation_type == ArticulationType.PRISMATIC
        and first_slide.parent == "guide_frame"
        and first_slide.child == "first_carriage"
        and output_slide.parent == "first_carriage"
        and output_slide.child == "output_carriage",
        details=f"first={first_slide}, output={output_slide}",
    )
    ctx.check(
        "common slide axis",
        tuple(first_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(output_slide.axis) == (1.0, 0.0, 0.0),
        details=f"first axis={first_slide.axis}, output axis={output_slide.axis}",
    )

    ctx.expect_gap(
        first,
        guide,
        axis="z",
        positive_elem="guide_bearing_0_0",
        negative_elem="guide_rail_0",
        min_gap=0.0,
        max_gap=0.002,
        name="first carriage bearing sits on guide rail",
    )
    ctx.expect_gap(
        output,
        first,
        axis="z",
        positive_elem="nested_bearing_0_0",
        negative_elem="nested_rail_0",
        min_gap=0.0,
        max_gap=0.002,
        name="output carriage bearing sits on nested rail",
    )

    rest_first = ctx.part_world_position(first)
    rest_output = ctx.part_world_position(output)
    with ctx.pose({first_slide: 0.380}):
        extended_first = ctx.part_world_position(first)
        ctx.expect_overlap(
            first,
            guide,
            axes="x",
            elem_a="guide_bearing_1_0",
            elem_b="guide_rail_0",
            min_overlap=0.080,
            name="first carriage remains on guide at full travel",
        )
    with ctx.pose({output_slide: 0.220}):
        extended_output = ctx.part_world_position(output)
        ctx.expect_overlap(
            output,
            first,
            axes="x",
            elem_a="nested_bearing_1_0",
            elem_b="nested_rail_0",
            min_overlap=0.050,
            name="output carriage remains on nested rail at full travel",
        )

    ctx.check(
        "first carriage extends along positive x",
        rest_first is not None
        and extended_first is not None
        and extended_first[0] > rest_first[0] + 0.30,
        details=f"rest={rest_first}, extended={extended_first}",
    )
    ctx.check(
        "output carriage extends along positive x",
        rest_output is not None
        and extended_output is not None
        and extended_output[0] > rest_output[0] + 0.18,
        details=f"rest={rest_output}, extended={extended_output}",
    )

    return ctx.report()


object_model = build_object_model()
