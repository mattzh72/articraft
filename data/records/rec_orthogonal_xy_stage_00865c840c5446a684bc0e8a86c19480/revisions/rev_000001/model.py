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


def _bolt_head(part, *, x: float, y: float, z: float, radius: float, name: str, material):
    """Small seated screw head; embedded a hair into the supporting plate."""
    part.visual(
        Cylinder(radius=radius, length=0.004),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_fixture_xy_stage")

    aluminum = model.material("satin_aluminum", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_aluminum = model.material("black_anodized", rgba=(0.035, 0.038, 0.042, 1.0))
    steel = model.material("ground_steel", rgba=(0.33, 0.35, 0.36, 1.0))
    blue = model.material("blue_fixture_plate", rgba=(0.05, 0.17, 0.48, 1.0))
    rubber = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))

    # Base frame: broad service plate plus two fixed X guide rails.
    base = model.part("base")
    base.visual(
        Box((1.15, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=aluminum,
        name="base_plate",
    )
    for idx, y in enumerate((-0.105, 0.105)):
        base.visual(
            Box((1.02, 0.026, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.013)),
            material=steel,
            name=f"x_rail_{idx}",
        )

    # Fixed end stops bracket both rails and define the usable X travel.
    for idx, x in enumerate((-0.535, 0.535)):
        base.visual(
            Box((0.025, 0.31, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0275)),
            material=dark_aluminum,
            name=f"end_stop_{idx}",
        )

    for idx, (x, y) in enumerate(
        (
            (-0.46, -0.155),
            (-0.20, -0.155),
            (0.20, -0.155),
            (0.46, -0.155),
            (-0.46, 0.155),
            (-0.20, 0.155),
            (0.20, 0.155),
            (0.46, 0.155),
        )
    ):
        _bolt_head(base, x=x, y=y, z=0.0015, radius=0.010, name=f"base_bolt_{idx}", material=rubber)

    # Long X carriage: four bearing trucks ride on the fixed rails, with a
    # compact Y rail pair mounted to the top of the carriage plate.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.62, 0.27, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="carriage_plate",
    )
    truck_positions = (
        (-0.21, -0.105),
        (0.21, -0.105),
        (-0.21, 0.105),
        (0.21, 0.105),
    )
    for idx, (x, y) in enumerate(truck_positions):
        carriage.visual(
            Box((0.145, 0.052, 0.024)),
            origin=Origin(xyz=(x, y, -0.026)),
            material=steel,
            name=f"x_truck_{idx}",
        )
    for idx, x in enumerate((-0.070, 0.070)):
        carriage.visual(
            Box((0.024, 0.28, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.023)),
            material=steel,
            name=f"y_rail_{idx}",
        )

    # Orthogonal compact slide and fixture pad.  Its bearing blocks sit directly
    # on the Y rails carried by the X carriage.
    top_slide = model.part("top_slide")
    top_slide.visual(
        Box((0.22, 0.18, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="top_pad",
    )
    for idx, x in enumerate((-0.070, 0.070)):
        top_slide.visual(
            Box((0.052, 0.105, 0.022)),
            origin=Origin(xyz=(x, 0.0, -0.025)),
            material=steel,
            name=f"y_bearing_{idx}",
        )
    for idx, (x, y) in enumerate(((-0.075, -0.055), (0.075, -0.055), (-0.075, 0.055), (0.075, 0.055))):
        _bolt_head(top_slide, x=x, y=y, z=0.015, radius=0.008, name=f"pad_bolt_{idx}", material=rubber)

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=top_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=-0.07, upper=0.07),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    top_slide = object_model.get_part("top_slide")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")

    ctx.check(
        "two orthogonal prismatic axes",
        x_axis.articulation_type == ArticulationType.PRISMATIC
        and y_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_axis.axis) == (1.0, 0.0, 0.0)
        and tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        details=f"x_axis={x_axis.axis}, y_axis={y_axis.axis}",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="x_truck_0",
        negative_elem="x_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="x carriage truck seated on fixed rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="x_truck_0",
        elem_b="x_rail_0",
        min_overlap=0.020,
        name="x truck retained over fixed rail",
    )
    ctx.expect_gap(
        top_slide,
        carriage,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="top slide bearing seated on y rail",
    )
    ctx.expect_overlap(
        top_slide,
        carriage,
        axes="xy",
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        min_overlap=0.020,
        name="top slide retained over y rail",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({x_axis: 0.18}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            elem_a="x_truck_1",
            elem_b="x_rail_0",
            min_overlap=0.020,
            name="x carriage keeps rail engagement at positive travel",
        )
    ctx.check(
        "x joint translates the long carriage",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.17,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_slide = ctx.part_world_position(top_slide)
    with ctx.pose({y_axis: 0.07}):
        extended_slide = ctx.part_world_position(top_slide)
        ctx.expect_overlap(
            top_slide,
            carriage,
            axes="xy",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.020,
            name="y slide keeps rail engagement at positive travel",
        )
    ctx.check(
        "y joint translates the top pad",
        rest_slide is not None
        and extended_slide is not None
        and extended_slide[1] > rest_slide[1] + 0.06,
        details=f"rest={rest_slide}, extended={extended_slide}",
    )

    return ctx.report()


object_model = build_object_model()
