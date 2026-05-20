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
    model = ArticulatedObject(name="slimline_split_lid_chest_cooler")

    shell_blue = Material("cooler_blue", color=(0.05, 0.34, 0.70, 1.0))
    rim_blue = Material("rim_blue", color=(0.03, 0.24, 0.55, 1.0))
    liner_white = Material("white_inner_liner", color=(0.92, 0.96, 0.98, 1.0))
    gasket_black = Material("black_rubber_gasket", color=(0.01, 0.012, 0.014, 1.0))
    hinge_dark = Material("dark_hinge_pin", color=(0.02, 0.022, 0.025, 1.0))

    length = 1.30
    width = 0.34
    wall = 0.035
    rim_top_z = 0.380
    lid_thickness = 0.035
    lid_center_z = rim_top_z + lid_thickness / 2.0

    body = model.part("body")

    # Slim insulated tub: long, narrow, open above the inner liner.
    body.visual(
        Box((length, width, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=shell_blue,
        name="outer_floor",
    )
    body.visual(
        Box((length, wall, 0.350)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, 0.200)),
        material=shell_blue,
        name="side_wall_0",
    )
    body.visual(
        Box((length, wall, 0.350)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, 0.200)),
        material=shell_blue,
        name="side_wall_1",
    )
    body.visual(
        Box((wall, width, 0.350)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, 0.200)),
        material=shell_blue,
        name="end_wall_0",
    )
    body.visual(
        Box((wall, width, 0.350)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, 0.200)),
        material=shell_blue,
        name="end_wall_1",
    )

    # Thin pale liner faces inside the cooler so open lids reveal a hollow chest.
    body.visual(
        Box((length - 2 * wall, 0.010, 0.260)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall - 0.005, 0.190)),
        material=liner_white,
        name="inner_liner_0",
    )
    body.visual(
        Box((length - 2 * wall, 0.010, 0.260)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall + 0.005, 0.190)),
        material=liner_white,
        name="inner_liner_1",
    )
    body.visual(
        Box((0.010, width - 2 * wall, 0.260)),
        origin=Origin(xyz=(length / 2.0 - wall - 0.005, 0.0, 0.190)),
        material=liner_white,
        name="inner_liner_2",
    )
    body.visual(
        Box((0.010, width - 2 * wall, 0.260)),
        origin=Origin(xyz=(-length / 2.0 + wall + 0.005, 0.0, 0.190)),
        material=liner_white,
        name="inner_liner_3",
    )

    # Continuous center divider/spine forms the two top hinge edges.
    body.visual(
        Box((length - 0.070, 0.030, 0.350)),
        origin=Origin(xyz=(0.0, 0.0, 0.2025)),
        material=liner_white,
        name="center_divider",
    )
    body.visual(
        Box((length, 0.042, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, rim_top_z - 0.013)),
        material=rim_blue,
        name="center_spine",
    )

    # Raised perimeter rim/gasket seats the two lid panels.
    body.visual(
        Box((length, 0.048, 0.026)),
        origin=Origin(xyz=(0.0, width / 2.0 - 0.024, rim_top_z - 0.013)),
        material=rim_blue,
        name="side_rim_0",
    )
    body.visual(
        Box((length, 0.048, 0.026)),
        origin=Origin(xyz=(0.0, -width / 2.0 + 0.024, rim_top_z - 0.013)),
        material=rim_blue,
        name="side_rim_1",
    )
    body.visual(
        Box((0.052, width, 0.026)),
        origin=Origin(xyz=(length / 2.0 - 0.026, 0.0, rim_top_z - 0.013)),
        material=rim_blue,
        name="end_rim_0",
    )
    body.visual(
        Box((0.052, width, 0.026)),
        origin=Origin(xyz=(-length / 2.0 + 0.026, 0.0, rim_top_z - 0.013)),
        material=rim_blue,
        name="end_rim_1",
    )
    body.visual(
        Box((length - 0.12, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.011, rim_top_z - 0.004)),
        material=gasket_black,
        name="center_gasket_0",
    )
    body.visual(
        Box((length - 0.12, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.011, rim_top_z - 0.004)),
        material=gasket_black,
        name="center_gasket_1",
    )

    # Integrated molded end grips and short rubber feet keep the cooler grounded.
    body.visual(
        Box((0.018, 0.180, 0.055)),
        origin=Origin(xyz=(length / 2.0 + 0.009, 0.0, 0.235)),
        material=rim_blue,
        name="end_grip_0",
    )
    body.visual(
        Box((0.018, 0.180, 0.055)),
        origin=Origin(xyz=(-length / 2.0 - 0.009, 0.0, 0.235)),
        material=rim_blue,
        name="end_grip_1",
    )
    for i, x in enumerate((-0.47, 0.47)):
        for j, y in enumerate((-0.115, 0.115)):
            body.visual(
                Box((0.090, 0.050, 0.020)),
                origin=Origin(xyz=(x, y, -0.010)),
                material=gasket_black,
                name=f"rubber_foot_{i}_{j}",
            )

    hinge_y = 0.021
    lid_width = 0.145
    lid_length = 1.220
    hinge_origin_z = lid_center_z
    hinge_cylinder_rpy = (0.0, math.pi / 2.0, 0.0)

    lid_0 = model.part("lid_0")
    lid_0.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(xyz=(0.0, lid_width / 2.0, 0.0)),
        material=shell_blue,
        name="lid_panel",
    )
    lid_0.visual(
        Box((lid_length - 0.09, lid_width - 0.040, 0.006)),
        origin=Origin(xyz=(0.0, lid_width / 2.0 + 0.006, lid_thickness / 2.0 + 0.003)),
        material=liner_white,
        name="raised_insert",
    )
    lid_0.visual(
        Box((lid_length - 0.12, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, lid_width - 0.010, -lid_thickness / 2.0 + 0.005)),
        material=gasket_black,
        name="outer_gasket",
    )
    lid_0.visual(
        Cylinder(radius=0.014, length=lid_length - 0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_cylinder_rpy),
        material=hinge_dark,
        name="hinge_barrel",
    )
    lid_0.visual(
        Box((lid_length - 0.18, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.012, -0.012)),
        material=hinge_dark,
        name="hinge_leaf",
    )

    lid_1 = model.part("lid_1")
    lid_1.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(xyz=(0.0, -lid_width / 2.0, 0.0)),
        material=shell_blue,
        name="lid_panel",
    )
    lid_1.visual(
        Box((lid_length - 0.09, lid_width - 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -lid_width / 2.0 - 0.006, lid_thickness / 2.0 + 0.003)),
        material=liner_white,
        name="raised_insert",
    )
    lid_1.visual(
        Box((lid_length - 0.12, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -lid_width + 0.010, -lid_thickness / 2.0 + 0.005)),
        material=gasket_black,
        name="outer_gasket",
    )
    lid_1.visual(
        Cylinder(radius=0.014, length=lid_length - 0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_cylinder_rpy),
        material=hinge_dark,
        name="hinge_barrel",
    )
    lid_1.visual(
        Box((lid_length - 0.18, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.012, -0.012)),
        material=hinge_dark,
        name="hinge_leaf",
    )

    lid_limits = MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75)
    model.articulation(
        "body_to_lid_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid_0,
        origin=Origin(xyz=(0.0, hinge_y, hinge_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=lid_limits,
    )
    model.articulation(
        "body_to_lid_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid_1,
        origin=Origin(xyz=(0.0, -hinge_y, hinge_origin_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=lid_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    hinge_0 = object_model.get_articulation("body_to_lid_0")
    hinge_1 = object_model.get_articulation("body_to_lid_1")

    ctx.expect_gap(
        lid_0,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="side_rim_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid_0 rests on its rim",
    )
    ctx.expect_gap(
        lid_1,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="side_rim_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid_1 rests on its rim",
    )
    ctx.expect_overlap(
        lid_0,
        lid_1,
        axes="x",
        elem_a="lid_panel",
        elem_b="lid_panel",
        min_overlap=1.15,
        name="split lid panels are equal length",
    )

    rest_0 = ctx.part_world_aabb(lid_0)
    rest_1 = ctx.part_world_aabb(lid_1)
    with ctx.pose({hinge_0: 1.20, hinge_1: 0.0}):
        open_0 = ctx.part_world_aabb(lid_0)
        still_1 = ctx.part_world_aabb(lid_1)
        ctx.expect_gap(
            lid_0,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="center_spine",
            min_gap=0.005,
            name="lid_0 swings upward from center hinge",
        )
    with ctx.pose({hinge_0: 0.0, hinge_1: 1.20}):
        open_1 = ctx.part_world_aabb(lid_1)

    ctx.check(
        "lid_0 opens independently",
        rest_0 is not None
        and open_0 is not None
        and still_1 is not None
        and open_0[1][2] > rest_0[1][2] + 0.070
        and abs(still_1[1][2] - rest_1[1][2]) < 0.002,
        details=f"rest_0={rest_0}, open_0={open_0}, rest_1={rest_1}, still_1={still_1}",
    )
    ctx.check(
        "lid_1 opens independently",
        rest_1 is not None
        and open_1 is not None
        and open_1[1][2] > rest_1[1][2] + 0.070,
        details=f"rest_1={rest_1}, open_1={open_1}",
    )

    return ctx.report()


object_model = build_object_model()
