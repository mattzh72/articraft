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
    model = ArticulatedObject(name="two_axis_positioning_stage")

    anodized_black = model.material("anodized_black", rgba=(0.03, 0.035, 0.04, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.82, 0.84, 0.82, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.005, 0.006, 0.007, 1.0))
    blue_carriage = model.material("blue_carriage", rgba=(0.08, 0.16, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.64, 0.36, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=anodized_black,
        name="base_plate",
    )
    for y, name in ((-0.115, "x_rail_bed_0"), (0.115, "x_rail_bed_1")):
        base.visual(
            Box((0.58, 0.040, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.062)),
            material=brushed_aluminum,
            name=name,
        )
    for y, name in ((-0.115, "x_rod_0"), (0.115, "x_rod_1")):
        base.visual(
            Cylinder(radius=0.012, length=0.560),
            origin=Origin(xyz=(0.0, y, 0.085), rpy=(0.0, 1.57079632679, 0.0)),
            material=satin_steel,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.007, length=0.575),
        origin=Origin(xyz=(0.0, 0.0, 0.088), rpy=(0.0, 1.57079632679, 0.0)),
        material=satin_steel,
        name="x_leadscrew",
    )
    for x, name in ((-0.290, "x_end_block_0"), (0.290, "x_end_block_1")):
        base.visual(
            Box((0.035, 0.280, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=anodized_black,
            name=name,
        )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.260, 0.240, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue_carriage,
        name="carriage_plate",
    )
    for y, name in ((-0.115, "x_bearing_0"), (0.115, "x_bearing_1")):
        x_carriage.visual(
            Box((0.120, 0.050, 0.033)),
            origin=Origin(xyz=(0.0, y, -0.0115)),
            material=brushed_aluminum,
            name=name,
        )
    x_carriage.visual(
        Box((0.095, 0.052, 0.033)),
        origin=Origin(xyz=(0.0, 0.0, -0.0115)),
        material=brushed_aluminum,
        name="x_nut_block",
    )
    x_carriage.visual(
        Box((0.235, 0.275, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=anodized_black,
        name="y_slide_base",
    )
    for x, name in ((-0.075, "y_rail_bed_0"), (0.075, "y_rail_bed_1")):
        x_carriage.visual(
            Box((0.030, 0.250, 0.015)),
            origin=Origin(xyz=(x, 0.0, 0.0625)),
            material=brushed_aluminum,
            name=name,
        )
    for x, name in ((-0.075, "y_rod_0"), (0.075, "y_rod_1")):
        x_carriage.visual(
            Cylinder(radius=0.009, length=0.250),
            origin=Origin(xyz=(x, 0.0, 0.079), rpy=(1.57079632679, 0.0, 0.0)),
            material=satin_steel,
            name=name,
        )
    x_carriage.visual(
        Cylinder(radius=0.0055, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.082), rpy=(1.57079632679, 0.0, 0.0)),
        material=satin_steel,
        name="y_leadscrew",
    )
    for y, name in ((-0.125, "y_end_block_0"), (0.125, "y_end_block_1")):
        x_carriage.visual(
            Box((0.185, 0.022, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.077)),
            material=anodized_black,
            name=name,
        )

    y_table = model.part("y_table")
    y_table.visual(
        Box((0.220, 0.160, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="top_table",
    )
    for x, name in ((-0.075, "y_bearing_0"), (0.075, "y_bearing_1")):
        y_table.visual(
            Box((0.040, 0.085, 0.025)),
            origin=Origin(xyz=(x, 0.0, -0.0195)),
            material=brushed_aluminum,
            name=name,
        )
    for x, name in ((-0.048, "table_slot_0"), (0.048, "table_slot_1")):
        y_table.visual(
            Box((0.010, 0.145, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.0165)),
            material=dark_slot,
            name=name,
        )
    for y, name in ((-0.060, "table_cross_mark_0"), (0.060, "table_cross_mark_1")):
        y_table.visual(
            Box((0.190, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.0165)),
            material=dark_slot,
            name=name,
        )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=-0.090, upper=0.090),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.16, lower=-0.055, upper=0.055),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_table = object_model.get_part("y_table")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    ctx.check(
        "two prismatic orthogonal slide joints",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"x_slide={x_slide.articulation_type} axis={x_slide.axis}; "
        f"y_slide={y_slide.articulation_type} axis={y_slide.axis}",
    )

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_bearing_0",
        negative_elem="x_rod_0",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="x bearing rides on first base rail",
    )
    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_bearing_1",
        negative_elem="x_rod_1",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="x bearing rides on second base rail",
    )
    ctx.expect_gap(
        y_table,
        x_carriage,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rod_0",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="y bearing rides on first turned rail",
    )
    ctx.expect_gap(
        y_table,
        x_carriage,
        axis="z",
        positive_elem="y_bearing_1",
        negative_elem="y_rod_1",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="y bearing rides on second turned rail",
    )

    ctx.expect_overlap(
        x_carriage,
        base,
        axes="x",
        elem_a="x_bearing_0",
        elem_b="x_rod_0",
        min_overlap=0.100,
        name="x carriage has retained rail engagement",
    )
    ctx.expect_overlap(
        y_table,
        x_carriage,
        axes="y",
        elem_a="y_bearing_0",
        elem_b="y_rod_0",
        min_overlap=0.075,
        name="y table has retained rail engagement",
    )

    rest_x = ctx.part_world_position(x_carriage)
    rest_y = ctx.part_world_position(y_table)
    with ctx.pose({x_slide: 0.085, y_slide: 0.050}):
        moved_x = ctx.part_world_position(x_carriage)
        moved_y = ctx.part_world_position(y_table)
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_bearing_0",
            elem_b="x_rod_0",
            min_overlap=0.100,
            name="x carriage remains on rails near travel limit",
        )
        ctx.expect_overlap(
            y_table,
            x_carriage,
            axes="y",
            elem_a="y_bearing_0",
            elem_b="y_rod_0",
            min_overlap=0.075,
            name="y table remains on turned rails near travel limit",
        )
    ctx.check(
        "upper pose moves x carriage along x and table along y",
        rest_x is not None
        and moved_x is not None
        and rest_y is not None
        and moved_y is not None
        and moved_x[0] > rest_x[0] + 0.080
        and abs(moved_x[1] - rest_x[1]) < 0.002
        and moved_y[1] > rest_y[1] + 0.045,
        details=f"rest_x={rest_x}, moved_x={moved_x}, rest_y={rest_y}, moved_y={moved_y}",
    )

    return ctx.report()


object_model = build_object_model()
