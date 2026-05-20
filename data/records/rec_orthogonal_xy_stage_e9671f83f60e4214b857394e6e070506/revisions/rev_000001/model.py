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
    mesh_from_cadquery,
)
import cadquery as cq


def _table_plate_mesh():
    """A real under-plate stage table: rounded aluminium plate, holes, and slots."""
    length = 0.260
    width = 0.200
    thickness = 0.018
    plate = cq.Workplane("XY").box(length, width, thickness)
    plate = plate.edges("|Z").fillet(0.012)

    # Four through mounting holes and two shallow longitudinal T-slot style grooves.
    hole_points = [(-0.090, -0.060), (-0.090, 0.060), (0.090, -0.060), (0.090, 0.060)]
    plate = plate.faces(">Z").workplane().pushPoints(hole_points).hole(0.012)
    groove_depth = 0.004
    for y in (-0.045, 0.045):
        cutter = cq.Workplane("XY").box(0.210, 0.010, groove_depth + 0.002).translate(
            (0.0, y, thickness / 2.0 - groove_depth / 2.0)
        )
        plate = plate.cut(cutter)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_plate_xy_stage")

    dark = Material("matte_black_anodized", rgba=(0.02, 0.022, 0.026, 1.0))
    black = Material("black_bearing_polymer", rgba=(0.006, 0.006, 0.007, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    steel = Material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    brass = Material("brass_lead_nut", rgba=(0.86, 0.62, 0.22, 1.0))
    slot_black = Material("dark_recesses", rgba=(0.0, 0.0, 0.0, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.480, 0.300, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="base_plate",
    )
    for i, y in enumerate((-0.095, 0.095)):
        base.visual(
            Cylinder(radius=0.008, length=0.420),
            origin=Origin(xyz=(0.0, y, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=("x_guide_0" if i == 0 else "x_guide_1"),
        )
        for j, x in enumerate((-0.180, 0.180)):
            base.visual(
                Box((0.030, 0.026, 0.024)),
                origin=Origin(xyz=(x, y, 0.045)),
                material=dark,
                name=f"x_rail_block_{i}_{j}",
            )
    base.visual(
        Cylinder(radius=0.005, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="x_lead_screw",
    )
    for i, x in enumerate((-0.210, 0.210)):
        base.visual(
            Box((0.022, 0.034, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.042)),
            material=dark,
            name=f"x_screw_bearing_{i}",
        )
    for i, (x, y) in enumerate(((-0.205, -0.125), (-0.205, 0.125), (0.205, -0.125), (0.205, 0.125))):
        base.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=black,
            name=f"foot_{i}",
        )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((0.350, 0.200, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="lower_plate",
    )
    # Four black saddle bearings straddle the base X guide rods without occupying
    # the rod volume, making the under-plate slide construction legible.
    for rail_i, y in enumerate((-0.095, 0.095)):
        for shoe_i, x in enumerate((-0.110, 0.110)):
            lower_carriage.visual(
                Box((0.060, 0.043, 0.008)),
                origin=Origin(xyz=(x, y, -0.016)),
                material=black,
                name=f"x_saddle_cap_{rail_i}_{shoe_i}",
            )
            for side_i, dy in enumerate((-0.017, 0.017)):
                lower_carriage.visual(
                    Box((0.060, 0.008, 0.026)),
                    origin=Origin(xyz=(x, y + dy, -0.023)),
                    material=black,
                    name=f"x_saddle_cheek_{rail_i}_{shoe_i}_{side_i}",
                )
    lower_carriage.visual(
        Box((0.055, 0.035, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=brass,
        name="x_lead_nut",
    )
    # The second stage guide rods run crosswise on top of the lower carriage.
    for i, x in enumerate((-0.095, 0.095)):
        lower_carriage.visual(
            Cylinder(radius=0.0075, length=0.220),
            origin=Origin(xyz=(x, 0.0, 0.031), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=("y_guide_0" if i == 0 else "y_guide_1"),
        )
        for j, y in enumerate((-0.105, 0.105)):
            lower_carriage.visual(
                Box((0.028, 0.020, 0.025)),
                origin=Origin(xyz=(x, y, 0.0195)),
                material=aluminum,
                name=f"y_rail_block_{i}_{j}",
            )
    lower_carriage.visual(
        Cylinder(radius=0.0045, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.022), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="y_lead_screw",
    )
    for i, y in enumerate((-0.105, 0.105)):
        lower_carriage.visual(
            Box((0.032, 0.020, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.014)),
            material=aluminum,
            name=f"y_screw_bearing_{i}",
        )

    upper_carriage = model.part("upper_carriage")
    upper_carriage.visual(
        Box((0.260, 0.220, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="upper_plate",
    )
    for rail_i, x in enumerate((-0.095, 0.095)):
        for shoe_i, y in enumerate((-0.060, 0.060)):
            upper_carriage.visual(
                Box((0.043, 0.060, 0.008)),
                origin=Origin(xyz=(x, y, -0.0125)),
                material=black,
                name=f"y_saddle_cap_{rail_i}_{shoe_i}",
            )
            for side_i, dx in enumerate((-0.015, 0.015)):
                upper_carriage.visual(
                    Box((0.008, 0.060, 0.028)),
                    origin=Origin(xyz=(x + dx, y, -0.027)),
                    material=black,
                    name=f"y_saddle_cheek_{rail_i}_{shoe_i}_{side_i}",
                )
    for i, x in enumerate((-0.020, 0.020)):
        upper_carriage.visual(
            Box((0.018, 0.045, 0.020)),
            origin=Origin(xyz=(x, 0.0, -0.026)),
            material=brass,
            name=f"y_lead_nut_cheek_{i}",
        )
    upper_carriage.visual(
        Box((0.058, 0.045, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=brass,
        name="y_lead_nut_bridge",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_table_plate_mesh(), "slotted_table_plate", tolerance=0.0007),
        origin=Origin(),
        material=aluminum,
        name="table_plate",
    )
    for i, y in enumerate((-0.045, 0.045)):
        table.visual(
            Box((0.205, 0.007, 0.0012)),
            origin=Origin(xyz=(0.0, y, 0.0046)),
            material=slot_black,
            name=f"slot_floor_{i}",
        )
    for i, (x, y) in enumerate(((-0.095, -0.070), (-0.095, 0.070), (0.095, -0.070), (0.095, 0.070))):
        table.visual(
            Cylinder(radius=0.010, length=0.013),
            origin=Origin(xyz=(x, y, -0.0155)),
            material=black,
            name=("standoff_0" if i == 0 else f"standoff_{i}"),
        )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=-0.055, upper=0.055),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=-0.045, upper=0.045),
    )
    model.articulation(
        "upper_to_table",
        ArticulationType.FIXED,
        parent=upper_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_carriage")
    table = object_model.get_part("table")
    base = object_model.get_part("base")
    x_slide = object_model.get_articulation("base_to_lower")
    y_slide = object_model.get_articulation("lower_to_upper")

    ctx.check(
        "two perpendicular prismatic slides",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(sum(a * b for a, b in zip(x_slide.axis, y_slide.axis))) < 1e-6,
        details=f"x_axis={x_slide.axis}, y_axis={y_slide.axis}",
    )
    ctx.expect_gap(
        lower,
        base,
        axis="z",
        positive_elem="lower_plate",
        negative_elem="x_guide_0",
        min_gap=0.004,
        max_gap=0.012,
        name="lower carriage clears base guide rods",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="y_guide_0",
        min_gap=0.002,
        max_gap=0.009,
        name="upper carriage clears cross guide rods",
    )
    ctx.expect_contact(
        table,
        upper,
        elem_a="standoff_0",
        elem_b="upper_plate",
        contact_tol=0.001,
        name="table standoffs land on upper carriage",
    )
    ctx.expect_overlap(
        table,
        upper,
        axes="xy",
        elem_a="table_plate",
        elem_b="upper_plate",
        min_overlap=0.15,
        name="top table footprint is carried by upper carriage",
    )

    rest_lower = ctx.part_world_position(lower)
    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({x_slide: 0.055}):
        extended_lower = ctx.part_world_position(lower)
        ctx.expect_overlap(
            lower,
            base,
            axes="xy",
            elem_a="lower_plate",
            elem_b="base_plate",
            min_overlap=0.16,
            name="lower carriage remains on base at x travel limit",
        )
    with ctx.pose({y_slide: 0.045}):
        extended_upper = ctx.part_world_position(upper)
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.12,
            name="upper carriage remains on lower stage at y travel limit",
        )
    ctx.check(
        "lower carriage travels along x",
        rest_lower is not None
        and extended_lower is not None
        and extended_lower[0] > rest_lower[0] + 0.045
        and abs(extended_lower[1] - rest_lower[1]) < 1e-6,
        details=f"rest={rest_lower}, extended={extended_lower}",
    )
    ctx.check(
        "upper carriage travels along y",
        rest_upper is not None
        and extended_upper is not None
        and extended_upper[1] > rest_upper[1] + 0.035
        and abs(extended_upper[0] - rest_upper[0]) < 1e-6,
        details=f"rest={rest_upper}, extended={extended_upper}",
    )

    return ctx.report()


object_model = build_object_model()
