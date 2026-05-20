from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="benchtop_xyz_positioning_stage")

    anodized_black = model.material("anodized_black", color=(0.03, 0.035, 0.04, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", color=(0.68, 0.70, 0.72, 1.0))
    dark_aluminum = model.material("dark_aluminum", color=(0.22, 0.24, 0.26, 1.0))
    polished_steel = model.material("polished_steel", color=(0.82, 0.84, 0.86, 1.0))
    blue_anodized = model.material("blue_anodized", color=(0.05, 0.17, 0.34, 1.0))
    screw_dark = model.material("black_oxide", color=(0.01, 0.01, 0.012, 1.0))

    x_base = model.part("x_base")
    x_base.visual(
        Box((0.74, 0.24, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=anodized_black,
        name="base_slab",
    )
    for x in (-0.335, 0.335):
        x_base.visual(
            Box((0.035, 0.24, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0725)),
            material=dark_aluminum,
            name=f"x_end_block_{'neg' if x < 0 else 'pos'}",
        )
    x_base.visual(
        Box((0.64, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, -0.075, 0.051)),
        material=dark_aluminum,
        name="x_rail_seat_neg",
    )
    x_base.visual(
        Cylinder(radius=0.011, length=0.62),
        origin=Origin(xyz=(0.0, -0.075, 0.065), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="x_guide_rod_neg",
    )
    x_base.visual(
        Box((0.64, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.075, 0.051)),
        material=dark_aluminum,
        name="x_rail_seat_pos",
    )
    x_base.visual(
        Cylinder(radius=0.011, length=0.62),
        origin=Origin(xyz=(0.0, 0.075, 0.065), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="x_guide_rod_pos",
    )
    for x in (-0.29, 0.29):
        x_base.visual(
            Box((0.026, 0.034, 0.032)),
            origin=Origin(xyz=(x, 0.0, 0.061)),
            material=dark_aluminum,
            name=f"x_screw_bearing_{'neg' if x < 0 else 'pos'}",
        )
    x_base.visual(
        Cylinder(radius=0.006, length=0.61),
        origin=Origin(xyz=(0.0, 0.0, 0.067), rpy=(0.0, pi / 2.0, 0.0)),
        material=screw_dark,
        name="x_lead_screw",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.074, 0.034, 0.014)),
        origin=Origin(xyz=(-0.055, -0.075, -0.003)),
        material=dark_aluminum,
        name="x_bearing_shoe_negx_negy",
    )
    y_carriage.visual(
        Box((0.074, 0.034, 0.014)),
        origin=Origin(xyz=(-0.055, 0.075, -0.003)),
        material=dark_aluminum,
        name="x_bearing_shoe_negx_posy",
    )
    y_carriage.visual(
        Box((0.074, 0.034, 0.014)),
        origin=Origin(xyz=(0.055, -0.075, -0.003)),
        material=dark_aluminum,
        name="x_bearing_shoe_posx_negy",
    )
    y_carriage.visual(
        Box((0.074, 0.034, 0.014)),
        origin=Origin(xyz=(0.055, 0.075, -0.003)),
        material=dark_aluminum,
        name="x_bearing_shoe_posx_posy",
    )
    y_carriage.visual(
        Box((0.22, 0.36, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=brushed_aluminum,
        name="x_saddle_plate",
    )
    y_carriage.visual(
        Box((0.026, 0.30, 0.012)),
        origin=Origin(xyz=(-0.062, 0.0, 0.044)),
        material=dark_aluminum,
        name="y_rail_seat_neg",
    )
    y_carriage.visual(
        Cylinder(radius=0.009, length=0.30),
        origin=Origin(xyz=(-0.062, 0.0, 0.057), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="y_guide_rod_neg",
    )
    y_carriage.visual(
        Box((0.026, 0.30, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, 0.044)),
        material=dark_aluminum,
        name="y_rail_seat_pos",
    )
    y_carriage.visual(
        Cylinder(radius=0.009, length=0.30),
        origin=Origin(xyz=(0.062, 0.0, 0.057), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="y_guide_rod_pos",
    )
    y_carriage.visual(
        Cylinder(radius=0.005, length=0.29),
        origin=Origin(xyz=(0.0, 0.0, 0.057), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=screw_dark,
        name="y_lead_screw",
    )
    for y in (-0.13, 0.13):
        y_carriage.visual(
            Box((0.03, 0.018, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.051)),
            material=dark_aluminum,
            name=f"y_screw_bearing_{'neg' if y < 0 else 'pos'}",
        )

    z_column = model.part("z_column")
    z_column.visual(
        Box((0.17, 0.095, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_aluminum,
        name="y_bearing_saddle",
    )
    z_column.visual(
        Box((0.125, 0.12, 0.04)),
        origin=Origin(xyz=(-0.055, 0.0, 0.048)),
        material=brushed_aluminum,
        name="column_foot",
    )
    z_column.visual(
        Box((0.045, 0.13, 0.31)),
        origin=Origin(xyz=(-0.09, 0.0, 0.195)),
        material=brushed_aluminum,
        name="vertical_column",
    )
    for z in (0.085, 0.315):
        z_column.visual(
            Box((0.048, 0.118, 0.018)),
            origin=Origin(xyz=(-0.054, 0.0, z)),
            material=dark_aluminum,
            name=f"z_rail_clamp_{'lower' if z < 0.2 else 'upper'}",
        )
    z_column.visual(
        Cylinder(radius=0.007, length=0.25),
        origin=Origin(xyz=(-0.035, -0.04, 0.20)),
        material=polished_steel,
        name="z_guide_rod_neg",
    )
    z_column.visual(
        Cylinder(radius=0.007, length=0.25),
        origin=Origin(xyz=(-0.035, 0.04, 0.20)),
        material=polished_steel,
        name="z_guide_rod_pos",
    )
    z_column.visual(
        Cylinder(radius=0.0045, length=0.24),
        origin=Origin(xyz=(-0.035, 0.0, 0.20)),
        material=screw_dark,
        name="z_lead_screw",
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        Box((0.034, 0.13, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_anodized,
        name="z_slider_block",
    )
    tool_slide.visual(
        Box((0.046, 0.046, 0.046)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=blue_anodized,
        name="tool_plate_neck",
    )
    tool_slide.visual(
        Box((0.018, 0.13, 0.13)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=brushed_aluminum,
        name="square_tool_plate",
    )
    for y in (-0.045, 0.045):
        for z in (-0.045, 0.045):
            tool_slide.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(0.075, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=screw_dark,
                name=f"tool_socket_{'neg' if y < 0 else 'pos'}y_{'neg' if z < 0 else 'pos'}z",
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=x_base,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_column,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=-0.085, upper=0.085),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=z_column,
        child=tool_slide,
        origin=Origin(xyz=(-0.011, 0.0, 0.17)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    x_base = object_model.get_part("x_base")
    y_carriage = object_model.get_part("y_carriage")
    z_column = object_model.get_part("z_column")
    tool_slide = object_model.get_part("tool_slide")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.expect_overlap(
        y_carriage,
        x_base,
        axes="xy",
        elem_a="x_saddle_plate",
        elem_b="base_slab",
        min_overlap=0.18,
        name="X carriage remains visually seated on the long base",
    )
    ctx.expect_within(
        z_column,
        y_carriage,
        axes="y",
        inner_elem="y_bearing_saddle",
        outer_elem="y_guide_rod_pos",
        margin=0.003,
        name="Y saddle sits within the crosswise guide span",
    )
    ctx.expect_gap(
        z_column,
        y_carriage,
        axis="z",
        positive_elem="y_bearing_saddle",
        negative_elem="y_guide_rod_pos",
        min_gap=0.0,
        max_gap=0.001,
        name="Y saddle is carried directly above the Y guide rods",
    )

    rest_x = ctx.part_world_position(y_carriage)
    rest_y = ctx.part_world_position(z_column)
    rest_z = ctx.part_world_position(tool_slide)
    with ctx.pose({x_slide: 0.16, y_slide: 0.07, z_slide: 0.10}):
        moved_x = ctx.part_world_position(y_carriage)
        moved_y = ctx.part_world_position(z_column)
        moved_z = ctx.part_world_position(tool_slide)
        ctx.expect_overlap(
            y_carriage,
            x_base,
            axes="x",
            elem_a="x_bearing_shoe_posx_posy",
            elem_b="x_guide_rod_pos",
            min_overlap=0.02,
            name="X bearing still engages its guide at travel",
        )
        ctx.expect_overlap(
            z_column,
            y_carriage,
            axes="y",
            elem_a="y_bearing_saddle",
            elem_b="y_guide_rod_pos",
            min_overlap=0.02,
            name="Y bearing still engages its guide at travel",
        )
        ctx.expect_overlap(
            tool_slide,
            z_column,
            axes="z",
            elem_a="z_slider_block",
            elem_b="z_guide_rod_pos",
            min_overlap=0.02,
            name="Z slider still engages the vertical guide at travel",
        )
    ctx.check(
        "X joint moves only along the base length",
        rest_x is not None
        and moved_x is not None
        and moved_x[0] > rest_x[0] + 0.15
        and abs(moved_x[1] - rest_x[1]) < 1e-6
        and abs(moved_x[2] - rest_x[2]) < 1e-6,
        details=f"rest={rest_x}, moved={moved_x}",
    )
    ctx.check(
        "Y joint moves only crosswise",
        rest_y is not None
        and moved_y is not None
        and moved_y[1] > rest_y[1] + 0.06
        and abs(moved_y[2] - rest_y[2]) < 1e-6,
        details=f"rest={rest_y}, moved={moved_y}",
    )
    ctx.check(
        "Z joint raises the square tool plate vertically",
        rest_z is not None
        and moved_z is not None
        and moved_z[2] > rest_z[2] + 0.09,
        details=f"rest={rest_z}, moved={moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
