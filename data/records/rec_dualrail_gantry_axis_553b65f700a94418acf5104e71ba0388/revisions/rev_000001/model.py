from __future__ import annotations

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
    model = ArticulatedObject(name="ladder_frame_gantry_axis")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    blue_steel = model.material("blue_steel", rgba=(0.06, 0.20, 0.55, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.74, 0.76, 0.76, 1.0))
    black = model.material("black_anodized", rgba=(0.01, 0.012, 0.014, 1.0))
    tool_metal = model.material("tool_metal", rgba=(0.45, 0.46, 0.44, 1.0))

    base = model.part("base_frame")
    # Floor pads and lower ladder frame.
    for i, (x, y) in enumerate(
        ((-0.78, -0.45), (-0.78, 0.45), (0.78, -0.45), (0.78, 0.45))
    ):
        base.visual(
            Box((0.18, 0.14, 0.06)),
            origin=Origin(xyz=(x, y, 0.03)),
            material=dark_steel,
            name=f"foot_pad_{i}",
        )
    for i, y in enumerate((-0.42, 0.42)):
        base.visual(
            Box((1.68, 0.09, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.11)),
            material=dark_steel,
            name=f"lower_side_rail_{i}",
        )
    for i, x in enumerate((-0.72, 0.0, 0.72)):
        base.visual(
            Box((0.10, 0.92, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.11)),
            material=dark_steel,
            name=f"lower_rung_{i}",
        )

    # Raised side beams carry the precision rails.  End rungs make the fixed
    # structure read as a ladder frame rather than two isolated rails.
    for i, y in enumerate((-0.42, 0.42)):
        base.visual(
            Box((1.62, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.46)),
            material=dark_steel,
            name=f"top_side_beam_{i}",
        )
        base.visual(
            Box((1.56, 0.040, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.535)),
            material=rail_steel,
            name=f"linear_rail_{i}",
        )

    for i, (x, y) in enumerate(
        (
            (-0.78, -0.42),
            (-0.78, 0.42),
            (0.78, -0.42),
            (0.78, 0.42),
            (0.00, -0.42),
            (0.00, 0.42),
        )
    ):
        base.visual(
            Box((0.09, 0.09, 0.35)),
            origin=Origin(xyz=(x, y, 0.285)),
            material=dark_steel,
            name=f"upright_{i}",
        )
    for i, x in enumerate((-0.78, 0.78)):
        base.visual(
            Box((0.09, 0.92, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.46)),
            material=dark_steel,
            name=f"top_rung_{i}",
        )
        base.visual(
            Box((0.035, 0.18, 0.06)),
            origin=Origin(xyz=(x, -0.42, 0.58)),
            material=black,
            name=f"end_stop_{2 * i}",
        )
        base.visual(
            Box((0.035, 0.18, 0.06)),
            origin=Origin(xyz=(x, 0.42, 0.58)),
            material=black,
            name=f"end_stop_{2 * i + 1}",
        )

    beam = model.part("gantry_beam")
    beam.visual(
        Box((0.18, 0.98, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=blue_steel,
        name="cross_beam",
    )
    beam.visual(
        Box((0.20, 0.035, 0.17)),
        origin=Origin(xyz=(0.0, -0.49, 0.17)),
        material=black,
        name="beam_end_plate_0",
    )
    beam.visual(
        Box((0.20, 0.035, 0.17)),
        origin=Origin(xyz=(0.0, 0.49, 0.17)),
        material=black,
        name="beam_end_plate_1",
    )
    for i, y in enumerate((-0.42, 0.42)):
        beam.visual(
            Box((0.14, 0.10, 0.17)),
            origin=Origin(xyz=(0.0, y, 0.105)),
            material=blue_steel,
            name=f"side_plate_{i}",
        )
        for j, x in enumerate((-0.060, 0.060)):
            beam.visual(
                Box((0.105, 0.120, 0.048)),
                origin=Origin(xyz=(x, y, 0.025)),
                material=black,
                name=f"rail_truck_{2 * i + j}",
            )
    for i, z in enumerate((0.155, 0.245)):
        beam.visual(
            Box((0.018, 0.86, 0.025)),
            origin=Origin(xyz=(-0.089, 0.0, z)),
            material=rail_steel,
            name=f"beam_guide_{i}",
        )

    carriage = model.part("tool_carriage")
    carriage.visual(
        Box((0.070, 0.240, 0.220)),
        origin=Origin(xyz=(-0.145, 0.0, 0.0)),
        material=black,
        name="carriage_plate",
    )
    for i, (y, z) in enumerate(
        ((-0.060, -0.045), (0.060, -0.045), (-0.060, 0.045), (0.060, 0.045))
    ):
        carriage.visual(
            Box((0.030, 0.075, 0.035)),
            origin=Origin(xyz=(-0.113, y, z)),
            material=black,
            name=f"guide_shoe_{i}",
        )
    carriage.visual(
        Box((0.080, 0.120, 0.180)),
        origin=Origin(xyz=(-0.165, 0.0, -0.185)),
        material=blue_steel,
        name="tool_holder",
    )
    carriage.visual(
        Cylinder(radius=0.030, length=0.210),
        origin=Origin(xyz=(-0.165, 0.0, -0.350)),
        material=tool_metal,
        name="spindle_nose",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.120),
        origin=Origin(xyz=(-0.165, 0.0, -0.515)),
        material=tool_metal,
        name="short_tool",
    )

    model.articulation(
        "base_to_beam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.549)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.0, lower=-0.50, upper=0.50),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-0.31, upper=0.31),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    beam = object_model.get_part("gantry_beam")
    carriage = object_model.get_part("tool_carriage")
    base_slide = object_model.get_articulation("base_to_beam")
    carriage_slide = object_model.get_articulation("beam_to_carriage")

    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="rail_truck_0",
        negative_elem="linear_rail_0",
        min_gap=0.0,
        max_gap=0.003,
        name="beam truck rides just above rail",
    )
    ctx.expect_within(
        beam,
        base,
        axes="x",
        inner_elem="rail_truck_1",
        outer_elem="linear_rail_0",
        margin=0.0,
        name="beam truck remains on fixed rail at center",
    )
    ctx.expect_gap(
        beam,
        carriage,
        axis="x",
        positive_elem="beam_guide_0",
        negative_elem="guide_shoe_0",
        min_gap=0.0,
        max_gap=0.002,
        name="carriage shoe kisses beam guide",
    )
    ctx.expect_within(
        carriage,
        beam,
        axes="y",
        inner_elem="guide_shoe_3",
        outer_elem="beam_guide_1",
        margin=0.006,
        name="centered carriage shoe is within beam guide",
    )

    beam_rest = ctx.part_world_position(beam)
    with ctx.pose({base_slide: 0.40}):
        ctx.expect_within(
            beam,
            base,
            axes="x",
            inner_elem="rail_truck_3",
            outer_elem="linear_rail_1",
            margin=0.0,
            name="advanced beam truck remains on rail",
        )
        beam_advanced = ctx.part_world_position(beam)
    ctx.check(
        "beam advances along base axis",
        beam_rest is not None
        and beam_advanced is not None
        and beam_advanced[0] > beam_rest[0] + 0.35,
        details=f"rest={beam_rest}, advanced={beam_advanced}",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.24}):
        ctx.expect_within(
            carriage,
            beam,
            axes="y",
            inner_elem="guide_shoe_3",
            outer_elem="beam_guide_1",
            margin=0.006,
            name="advanced carriage shoe remains on beam guide",
        )
        carriage_advanced = ctx.part_world_position(carriage)
    ctx.check(
        "carriage advances along orthogonal beam axis",
        carriage_rest is not None
        and carriage_advanced is not None
        and carriage_advanced[1] > carriage_rest[1] + 0.20,
        details=f"rest={carriage_rest}, advanced={carriage_advanced}",
    )

    return ctx.report()


object_model = build_object_model()
