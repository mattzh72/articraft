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


BASE_X = 1.05
BASE_Y = 0.72
PLATE_T = 0.030
RAIL_X = 0.36
RAIL_LEN = 0.62
RAIL_TOP_Z = 0.059
BEAM_TRAVEL = 0.30
BEAM_REST_Y = -0.15
BEAM_TOP_Z = 0.0965
CARRIAGE_TRAVEL = 0.54
CARRIAGE_REST_X = -0.27


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_dual_rail_gantry")

    cast_dark = Material("cast_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_edge = Material("dark_edge", rgba=(0.02, 0.023, 0.026, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    anodized_blue = Material("anodized_blue", rgba=(0.05, 0.18, 0.34, 1.0))
    carriage_black = Material("carriage_black", rgba=(0.055, 0.060, 0.065, 1.0))

    # The base carries two long guideways and end stops.  The small bolt heads
    # are slightly seated into the plate so they read mounted rather than
    # floating, while remaining clear of the moving bearing shoes.
    base = model.part("base")
    base.visual(
        Box((BASE_X, BASE_Y, PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T / 2.0)),
        material=cast_dark,
        name="base_plate",
    )
    base.visual(
        Box((BASE_X - 0.06, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T + 0.001)),
        material=dark_edge,
        name="center_recess",
    )

    for index, x in enumerate((-RAIL_X, RAIL_X)):
        base.visual(
            Box((0.060, RAIL_LEN, 0.020)),
            origin=Origin(xyz=(x, 0.0, PLATE_T - 0.0005 + 0.010)),
            material=satin_steel,
            name=f"guideway_{index}",
        )
        base.visual(
            Box((0.030, RAIL_LEN, 0.010)),
            origin=Origin(xyz=(x, 0.0, RAIL_TOP_Z - 0.005)),
            material=brushed_steel,
            name=f"guideway_cap_{index}",
        )
        base.visual(
            Box((0.090, 0.035, 0.025)),
            origin=Origin(xyz=(x, -0.31, RAIL_TOP_Z + 0.0125)),
            material=dark_edge,
            name=f"guideway_stop_{index}_0",
        )
        base.visual(
            Box((0.090, 0.035, 0.025)),
            origin=Origin(xyz=(x, 0.31, RAIL_TOP_Z + 0.0125)),
            material=dark_edge,
            name=f"guideway_stop_{index}_1",
        )

    bolt_y_positions = (-0.25, 0.0, 0.25)
    for index, x in enumerate((-0.48, -0.24, 0.24, 0.48)):
        for j, y in enumerate(bolt_y_positions):
            base.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, y, PLATE_T + 0.001)),
                material=dark_edge,
                name=f"plate_bolt_{index}_{j}",
            )

    # The moving gantry beam is a single supported assembly: two bearing shoes
    # sit on the guideways, short uprights carry the shallow cross beam, and a
    # second linear rail on top guides the cross-slide.
    beam = model.part("beam")
    for index, x in enumerate((-RAIL_X, RAIL_X)):
        beam.visual(
            Box((0.150, 0.110, 0.022)),
            origin=Origin(xyz=(x, 0.0, 0.011)),
            material=satin_steel,
            name=f"rail_shoe_{index}",
        )
        beam.visual(
            Box((0.150, 0.010, 0.018)),
            origin=Origin(xyz=(x, -0.056, 0.013)),
            material=dark_edge,
            name=f"shoe_wiper_{index}_0",
        )
        beam.visual(
            Box((0.150, 0.010, 0.018)),
            origin=Origin(xyz=(x, 0.056, 0.013)),
            material=dark_edge,
            name=f"shoe_wiper_{index}_1",
        )
        beam.visual(
            Box((0.120, 0.075, 0.027)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=anodized_blue,
            name=f"beam_upright_{index}",
        )

    beam.visual(
        Box((0.860, 0.065, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=anodized_blue,
        name="beam_bar",
    )
    beam.visual(
        Box((0.800, 0.030, 0.011)),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=brushed_steel,
        name="cross_rail",
    )
    for index, x in enumerate((-0.405, 0.405)):
        beam.visual(
            Box((0.025, 0.075, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.1035)),
            material=dark_edge,
            name=f"beam_stop_{index}",
        )

    # Compact cross-slide carriage.  The saddle plate rides on the beam rail,
    # side cheeks wrap below the beam edges with visible clearance, and the top
    # pad provides a tool-mounting face.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.180, 0.120, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=carriage_black,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.160, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, -0.055, -0.014)),
        material=carriage_black,
        name="side_cheek_0",
    )
    carriage.visual(
        Box((0.160, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.055, -0.014)),
        material=carriage_black,
        name="side_cheek_1",
    )
    carriage.visual(
        Box((0.125, 0.085, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=cast_dark,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.095, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=satin_steel,
        name="tool_pad",
    )
    for index, x in enumerate((-0.034, 0.034)):
        for j, y in enumerate((-0.020, 0.020)):
            carriage.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.076)),
                material=dark_edge,
                name=f"tool_bolt_{index}_{j}",
            )

    model.articulation(
        "base_to_beam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, BEAM_REST_Y, RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=BEAM_TRAVEL),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_REST_X, 0.0, BEAM_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=CARRIAGE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    beam_slide = object_model.get_articulation("base_to_beam")
    carriage_slide = object_model.get_articulation("beam_to_carriage")

    ctx.check(
        "two orthogonal prismatic axes",
        beam_slide.articulation_type == ArticulationType.PRISMATIC
        and carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(beam_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(carriage_slide.axis) == (1.0, 0.0, 0.0),
        details=f"beam axis={beam_slide.axis}, carriage axis={carriage_slide.axis}",
    )

    with ctx.pose({beam_slide: 0.0, carriage_slide: 0.0}):
        for index in (0, 1):
            ctx.expect_gap(
                beam,
                base,
                axis="z",
                positive_elem=f"rail_shoe_{index}",
                negative_elem=f"guideway_cap_{index}",
                min_gap=0.0,
                max_gap=0.001,
                name=f"beam shoe {index} sits on guideway",
            )
            ctx.expect_overlap(
                beam,
                base,
                axes="xy",
                elem_a=f"rail_shoe_{index}",
                elem_b=f"guideway_cap_{index}",
                min_overlap=0.025,
                name=f"beam shoe {index} overlaps guideway footprint",
            )
        ctx.expect_gap(
            carriage,
            beam,
            axis="z",
            positive_elem="saddle_plate",
            negative_elem="cross_rail",
            min_gap=0.0,
            max_gap=0.001,
            name="cross-slide saddle sits on beam rail",
        )
        ctx.expect_overlap(
            carriage,
            beam,
            axes="xy",
            elem_a="saddle_plate",
            elem_b="cross_rail",
            min_overlap=0.025,
            name="cross-slide saddle overlaps beam rail footprint",
        )

    rest_beam_position = ctx.part_world_position(beam)
    with ctx.pose({beam_slide: BEAM_TRAVEL, carriage_slide: 0.0}):
        extended_beam_position = ctx.part_world_position(beam)
        for index in (0, 1):
            ctx.expect_within(
                beam,
                base,
                axes="y",
                inner_elem=f"rail_shoe_{index}",
                outer_elem=f"guideway_cap_{index}",
                margin=0.002,
                name=f"beam shoe {index} remains on guideway at travel",
            )

    rest_carriage_position = ctx.part_world_position(carriage)
    with ctx.pose({beam_slide: 0.0, carriage_slide: CARRIAGE_TRAVEL}):
        extended_carriage_position = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            beam,
            axes="x",
            inner_elem="saddle_plate",
            outer_elem="cross_rail",
            margin=0.002,
            name="cross-slide remains on beam rail at travel",
        )

    ctx.check(
        "beam travels across base",
        rest_beam_position is not None
        and extended_beam_position is not None
        and extended_beam_position[1] > rest_beam_position[1] + 0.25,
        details=f"rest={rest_beam_position}, extended={extended_beam_position}",
    )
    ctx.check(
        "carriage travels across beam",
        rest_carriage_position is not None
        and extended_carriage_position is not None
        and extended_carriage_position[0] > rest_carriage_position[0] + 0.45,
        details=f"rest={rest_carriage_position}, extended={extended_carriage_position}",
    )

    return ctx.report()


object_model = build_object_model()
