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
    model = ArticulatedObject(name="wide_gantry_positioning_module")

    anodized = Material("dark_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    rail_steel = Material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    black = Material("black_polymer", rgba=(0.02, 0.022, 0.025, 1.0))
    blue = Material("blue_carriage_plate", rgba=(0.05, 0.18, 0.46, 1.0))
    tool_metal = Material("brushed_tool_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    accent = Material("orange_safety_accent", rgba=(0.95, 0.38, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.15, 1.60, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.095, 1.45, 0.060)),
        origin=Origin(xyz=(-0.47, 0.0, 0.070)),
        material=black,
        name="rail_bed_0",
    )
    base.visual(
        Box((0.044, 1.36, 0.035)),
        origin=Origin(xyz=(-0.47, 0.0, 0.1175)),
        material=rail_steel,
        name="rail_cap_0",
    )
    base.visual(
        Box((0.110, 0.045, 0.085)),
        origin=Origin(xyz=(-0.47, -0.705, 0.0925)),
        material=anodized,
        name="end_stop_0_0",
    )
    base.visual(
        Box((0.110, 0.045, 0.085)),
        origin=Origin(xyz=(-0.47, 0.705, 0.0925)),
        material=anodized,
        name="end_stop_0_1",
    )
    base.visual(
        Box((0.095, 1.45, 0.060)),
        origin=Origin(xyz=(0.47, 0.0, 0.070)),
        material=black,
        name="rail_bed_1",
    )
    base.visual(
        Box((0.044, 1.36, 0.035)),
        origin=Origin(xyz=(0.47, 0.0, 0.1175)),
        material=rail_steel,
        name="rail_cap_1",
    )
    base.visual(
        Box((0.110, 0.045, 0.085)),
        origin=Origin(xyz=(0.47, -0.705, 0.0925)),
        material=anodized,
        name="end_stop_1_0",
    )
    base.visual(
        Box((0.110, 0.045, 0.085)),
        origin=Origin(xyz=(0.47, 0.705, 0.0925)),
        material=anodized,
        name="end_stop_1_1",
    )
    base.visual(
        Box((1.02, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, -0.755, 0.0675)),
        material=anodized,
        name="rear_tie_bar",
    )
    base.visual(
        Box((1.02, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.755, 0.0675)),
        material=anodized,
        name="front_tie_bar",
    )

    beam = model.part("moving_beam")
    beam.visual(
        Box((0.135, 0.180, 0.060)),
        origin=Origin(xyz=(-0.47, 0.0, 0.030)),
        material=black,
        name="bearing_shoe_0",
    )
    beam.visual(
        Box((0.085, 0.125, 0.205)),
        origin=Origin(xyz=(-0.47, 0.0, 0.1425)),
        material=anodized,
        name="side_riser_0",
    )
    beam.visual(
        Box((0.135, 0.180, 0.060)),
        origin=Origin(xyz=(0.47, 0.0, 0.030)),
        material=black,
        name="bearing_shoe_1",
    )
    beam.visual(
        Box((0.085, 0.125, 0.205)),
        origin=Origin(xyz=(0.47, 0.0, 0.1425)),
        material=anodized,
        name="side_riser_1",
    )
    beam.visual(
        Box((1.08, 0.105, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=anodized,
        name="box_beam",
    )
    beam.visual(
        Box((0.86, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.063, 0.245)),
        material=rail_steel,
        name="beam_front_rail",
    )
    beam.visual(
        Box((0.92, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.066, 0.305)),
        material=black,
        name="top_cable_tray",
    )

    carriage = model.part("tool_carriage")
    carriage.visual(
        Box((0.170, 0.036, 0.122)),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=black,
        name="bearing_truck",
    )
    carriage.visual(
        Box((0.150, 0.072, 0.230)),
        origin=Origin(xyz=(0.0, -0.070, -0.040)),
        material=blue,
        name="front_tool_plate",
    )
    carriage.visual(
        Box((0.115, 0.035, 0.075)),
        origin=Origin(xyz=(0.0, -0.122, -0.018)),
        material=accent,
        name="tool_clamp",
    )
    carriage.visual(
        Cylinder(radius=0.027, length=0.205),
        origin=Origin(xyz=(0.0, -0.125, -0.110)),
        material=tool_metal,
        name="tool_body",
    )
    carriage.visual(
        Cylinder(radius=0.011, length=0.070),
        origin=Origin(xyz=(0.0, -0.125, -0.245)),
        material=tool_metal,
        name="tool_tip",
    )

    model.articulation(
        "base_to_beam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, -0.45, 0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.60, lower=0.0, upper=0.90),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(-0.32, -0.076, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.75, lower=0.0, upper=0.64),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("moving_beam")
    carriage = object_model.get_part("tool_carriage")
    base_to_beam = object_model.get_articulation("base_to_beam")
    beam_to_carriage = object_model.get_articulation("beam_to_carriage")

    ctx.check(
        "two orthogonal prismatic axes",
        base_to_beam.articulation_type == ArticulationType.PRISMATIC
        and beam_to_carriage.articulation_type == ArticulationType.PRISMATIC
        and tuple(base_to_beam.axis) == (0.0, 1.0, 0.0)
        and tuple(beam_to_carriage.axis) == (1.0, 0.0, 0.0),
        details=f"axes were {base_to_beam.axis} and {beam_to_carriage.axis}",
    )

    with ctx.pose({base_to_beam: 0.0, beam_to_carriage: 0.0}):
        ctx.expect_gap(
            beam,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="bearing_shoe_0",
            negative_elem="rail_cap_0",
            name="beam shoe sits on first base rail",
        )
        ctx.expect_gap(
            beam,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="bearing_shoe_1",
            negative_elem="rail_cap_1",
            name="beam shoe sits on second base rail",
        )
        ctx.expect_gap(
            beam,
            carriage,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="beam_front_rail",
            negative_elem="bearing_truck",
            name="carriage truck bears on beam rail",
        )
        ctx.expect_within(
            carriage,
            beam,
            axes="x",
            margin=0.002,
            inner_elem="bearing_truck",
            outer_elem="beam_front_rail",
            name="carriage starts within beam rail travel",
        )

    beam_rest = ctx.part_world_position(beam)
    with ctx.pose({base_to_beam: 0.90, beam_to_carriage: 0.0}):
        beam_extended = ctx.part_world_position(beam)
        ctx.expect_within(
            beam,
            base,
            axes="y",
            margin=0.002,
            inner_elem="bearing_shoe_0",
            outer_elem="rail_cap_0",
            name="beam remains on rail at far travel",
        )
        ctx.expect_gap(
            beam,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="bearing_shoe_1",
            negative_elem="rail_cap_1",
            name="beam still sits on rails at far travel",
        )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({base_to_beam: 0.0, beam_to_carriage: 0.64}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            beam,
            axes="x",
            margin=0.002,
            inner_elem="bearing_truck",
            outer_elem="beam_front_rail",
            name="carriage remains on beam rail at far travel",
        )
        ctx.expect_gap(
            beam,
            carriage,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="beam_front_rail",
            negative_elem="bearing_truck",
            name="carriage stays seated against beam rail at far travel",
        )

    ctx.check(
        "beam translates along long rails",
        beam_rest is not None and beam_extended is not None and beam_extended[1] > beam_rest[1] + 0.85,
        details=f"rest={beam_rest}, extended={beam_extended}",
    )
    ctx.check(
        "carriage translates across beam",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.60,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    return ctx.report()


object_model = build_object_model()
