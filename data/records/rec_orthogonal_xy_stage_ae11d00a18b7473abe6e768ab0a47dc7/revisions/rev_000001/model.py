from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xy_stage")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    blue_slide = model.material("blue_anodized", rgba=(0.08, 0.22, 0.48, 1.0))
    dark_bearing = model.material("dark_bearing_blocks", rgba=(0.035, 0.04, 0.045, 1.0))
    slot_shadow = model.material("slot_shadow", rgba=(0.01, 0.012, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.46, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=cast_iron,
        name="base_plate",
    )
    for i, (x, y) in enumerate(
        ((-0.32, -0.17), (-0.32, 0.17), (0.32, -0.17), (0.32, 0.17))
    ):
        base.visual(
            Box((0.11, 0.08, 0.025)),
            origin=Origin(xyz=(x, y, -0.0125)),
            material=dark_bearing,
            name=f"foot_{i}",
        )
    base.visual(
        Box((0.66, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.145, 0.0925)),
        material=rail_steel,
        name="lower_rail_0",
    )
    base.visual(
        Box((0.66, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.145, 0.0925)),
        material=rail_steel,
        name="lower_rail_1",
    )
    for i, x in enumerate((-0.365, 0.365)):
        base.visual(
            Box((0.035, 0.38, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.100)),
            material=cast_iron,
            name=f"x_stop_{i}",
        )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.30, 0.34, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue_slide,
        name="carriage_block",
    )
    for name, x, y in (
        ("lower_bearing_0", -0.075, -0.145),
        ("lower_bearing_1", 0.075, -0.145),
        ("lower_bearing_2", -0.075, 0.145),
        ("lower_bearing_3", 0.075, 0.145),
    ):
        x_carriage.visual(
            Box((0.090, 0.060, 0.018)),
            origin=Origin(xyz=(x, y, 0.009)),
            material=dark_bearing,
            name=name,
        )
    x_carriage.visual(
        Box((0.032, 0.300, 0.025)),
        origin=Origin(xyz=(-0.090, 0.0, 0.0625)),
        material=rail_steel,
        name="y_rail_0",
    )
    x_carriage.visual(
        Box((0.032, 0.300, 0.025)),
        origin=Origin(xyz=(0.090, 0.0, 0.0625)),
        material=rail_steel,
        name="y_rail_1",
    )
    for i, y in enumerate((-0.170, 0.170)):
        x_carriage.visual(
            Box((0.25, 0.018, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0675)),
            material=cast_iron,
            name=f"y_stop_{i}",
        )

    top_table = model.part("top_table")
    for name, x, y in (
        ("y_bearing_0", -0.090, -0.060),
        ("y_bearing_1", -0.090, 0.060),
        ("y_bearing_2", 0.090, -0.060),
        ("y_bearing_3", 0.090, 0.060),
    ):
        top_table.visual(
            Box((0.058, 0.075, 0.018)),
            origin=Origin(xyz=(x, y, 0.009)),
            material=dark_bearing,
            name=name,
        )
    top_table.visual(
        Box((0.36, 0.26, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0385)),
        material=blue_slide,
        name="table_plate",
    )
    for i, y in enumerate((-0.060, 0.060)):
        top_table.visual(
            Box((0.300, 0.014, 0.003)),
            origin=Origin(xyz=(0.0, y, 0.0615)),
            material=slot_shadow,
            name=f"t_slot_{i}",
        )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=top_table,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=-0.085, upper=0.085),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    top_table = object_model.get_part("top_table")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    ctx.check(
        "slides are orthogonal prismatic axes",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"x_axis={x_slide.axis}, y_axis={y_slide.axis}",
    )

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="lower_bearing_0",
        negative_elem="lower_rail_0",
        min_gap=0.0,
        max_gap=0.0005,
        name="lower bearing runs just above X rail",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="x",
        elem_a="lower_bearing_0",
        elem_b="lower_rail_0",
        min_overlap=0.08,
        name="lower bearing remains supported on X rail at center",
    )
    ctx.expect_gap(
        top_table,
        x_carriage,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rail_0",
        min_gap=0.0,
        max_gap=0.0005,
        name="upper bearing runs just above Y rail",
    )
    ctx.expect_overlap(
        top_table,
        x_carriage,
        axes="y",
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        min_overlap=0.07,
        name="upper bearing remains supported on Y rail at center",
    )

    rest_x = ctx.part_world_position(x_carriage)
    with ctx.pose({x_slide: 0.12}):
        extended_x = ctx.part_world_position(x_carriage)
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="lower_bearing_0",
            elem_b="lower_rail_0",
            min_overlap=0.045,
            name="X carriage retains rail engagement at travel limit",
        )
    ctx.check(
        "X slide travels along world X",
        rest_x is not None and extended_x is not None and extended_x[0] > rest_x[0] + 0.10,
        details=f"rest={rest_x}, extended={extended_x}",
    )

    rest_y = ctx.part_world_position(top_table)
    with ctx.pose({y_slide: 0.085}):
        extended_y = ctx.part_world_position(top_table)
        ctx.expect_overlap(
            top_table,
            x_carriage,
            axes="y",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.045,
            name="top table retains Y rail engagement at travel limit",
        )
    ctx.check(
        "Y slide travels along world Y",
        rest_y is not None and extended_y is not None and extended_y[1] > rest_y[1] + 0.07,
        details=f"rest={rest_y}, extended={extended_y}",
    )

    return ctx.report()


object_model = build_object_model()
