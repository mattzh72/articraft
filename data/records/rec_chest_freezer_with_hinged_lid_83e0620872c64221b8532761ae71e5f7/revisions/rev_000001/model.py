from __future__ import annotations

import math

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
    model = ArticulatedObject(name="countertop_bar_chest_cooler")

    painted = model.material("gloss_midnight_blue", rgba=(0.03, 0.06, 0.10, 1.0))
    liner = model.material("white_poly_liner", rgba=(0.92, 0.94, 0.90, 1.0))
    gasket = model.material("black_rubber_gasket", rgba=(0.005, 0.005, 0.004, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.015, 0.014, 0.013, 1.0))

    body_depth = 0.360
    body_width = 0.520
    body_height = 0.300
    wall = 0.020
    bottom_thickness = 0.035

    hinge_x = -body_depth / 2.0 - 0.012
    hinge_z = body_height + 0.045
    lid_depth = 0.385
    lid_width = 0.550
    lid_thickness = 0.055
    lid_rear_offset = 0.012
    lid_center_x = lid_rear_offset + lid_depth / 2.0
    lid_center_z = -0.0075
    lid_front_x = lid_rear_offset + lid_depth

    body = model.part("body")
    body.visual(
        Box((body_depth, body_width, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=painted,
        name="bottom_pan",
    )
    body.visual(
        Box((wall, body_width, body_height)),
        origin=Origin(xyz=(body_depth / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=painted,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width, body_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=painted,
        name="rear_wall",
    )
    body.visual(
        Box((body_depth, wall, body_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall / 2.0, body_height / 2.0)),
        material=painted,
        name="side_wall_0",
    )
    body.visual(
        Box((body_depth, wall, body_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall / 2.0, body_height / 2.0)),
        material=painted,
        name="side_wall_1",
    )

    inner_depth = body_depth - 2.0 * wall
    inner_width = body_width - 2.0 * wall
    liner_thickness = 0.006
    liner_height = body_height - bottom_thickness - 0.012
    liner_z = bottom_thickness + liner_height / 2.0
    body.visual(
        Box((inner_depth, inner_width, liner_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness + liner_thickness / 2.0)),
        material=liner,
        name="liner_floor",
    )
    body.visual(
        Box((liner_thickness, inner_width, liner_height)),
        origin=Origin(xyz=(body_depth / 2.0 - wall - liner_thickness / 2.0, 0.0, liner_z)),
        material=liner,
        name="front_liner",
    )
    body.visual(
        Box((liner_thickness, inner_width, liner_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + wall + liner_thickness / 2.0, 0.0, liner_z)),
        material=liner,
        name="rear_liner",
    )
    body.visual(
        Box((inner_depth, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall - liner_thickness / 2.0, liner_z)),
        material=liner,
        name="side_liner_0",
    )
    body.visual(
        Box((inner_depth, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall + liner_thickness / 2.0, liner_z)),
        material=liner,
        name="side_liner_1",
    )

    gasket_h = 0.008
    gasket_z = body_height + gasket_h / 2.0
    body.visual(
        Box((0.030, body_width, gasket_h)),
        origin=Origin(xyz=(body_depth / 2.0 - 0.015, 0.0, gasket_z)),
        material=gasket,
        name="top_gasket_front",
    )
    body.visual(
        Box((0.030, body_width, gasket_h)),
        origin=Origin(xyz=(-body_depth / 2.0 + 0.015, 0.0, gasket_z)),
        material=gasket,
        name="top_gasket_rear",
    )
    body.visual(
        Box((body_depth, 0.030, gasket_h)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - 0.015, gasket_z)),
        material=gasket,
        name="top_gasket_side_0",
    )
    body.visual(
        Box((body_depth, 0.030, gasket_h)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + 0.015, gasket_z)),
        material=gasket,
        name="top_gasket_side_1",
    )

    for x in (-0.120, 0.120):
        for y in (-0.190, 0.190):
            body.visual(
                Cylinder(radius=0.030, length=0.018),
                origin=Origin(xyz=(x, y, -0.009)),
                material=rubber,
                name=f"rubber_foot_{x:+.2f}_{y:+.2f}",
            )

    body.visual(
        Box((0.006, 0.080, 0.052)),
        origin=Origin(xyz=(body_depth / 2.0 + 0.003, 0.0, 0.272)),
        material=chrome,
        name="catch_plate",
    )
    body.visual(
        Box((0.014, 0.052, 0.008)),
        origin=Origin(xyz=(body_depth / 2.0 + 0.011, 0.0, 0.253)),
        material=chrome,
        name="catch_loop",
    )

    hinge_width = 0.104
    knuckle_radius = 0.010
    outer_len = 0.028
    middle_len = 0.036
    hinge_centers = (-0.145, 0.145)
    hinge_axis_rpy = (math.pi / 2.0, 0.0, 0.0)
    for hinge_index, y0 in enumerate(hinge_centers):
        for seg_index, y_offset in enumerate((-0.036, 0.036)):
            body.visual(
                Cylinder(radius=knuckle_radius, length=outer_len),
                origin=Origin(xyz=(hinge_x, y0 + y_offset, hinge_z), rpy=hinge_axis_rpy),
                material=chrome,
                name=f"rear_knuckle_{hinge_index}_{seg_index}",
            )
            body.visual(
                Box((0.022, outer_len, 0.006)),
                origin=Origin(xyz=(hinge_x + 0.001, y0 + y_offset, hinge_z)),
                material=chrome,
                name=f"rear_hinge_wrap_{hinge_index}_{seg_index}",
            )
            body.visual(
                Box((0.004, outer_len, 0.058)),
                origin=Origin(xyz=(-body_depth / 2.0 - 0.002, y0 + y_offset, body_height + 0.018)),
                material=chrome,
                name=f"rear_hinge_leaf_{hinge_index}_{seg_index}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        origin=Origin(xyz=(lid_center_x, 0.0, lid_center_z)),
        material=painted,
        name="lid_shell",
    )
    lid.visual(
        Box((lid_depth - 0.040, lid_width - 0.050, 0.010)),
        origin=Origin(xyz=(lid_center_x + 0.010, 0.0, lid_center_z - lid_thickness / 2.0 + 0.005)),
        material=gasket,
        name="underside_seal",
    )

    rail_radius = 0.007
    rail_z = lid_center_z + lid_thickness / 2.0 + 0.035
    post_height = rail_z - (lid_center_z + lid_thickness / 2.0)
    rail_front_x = lid_rear_offset + lid_depth - 0.030
    rail_rear_x = lid_rear_offset + 0.030
    rail_side_y = lid_width / 2.0 - 0.035
    lid.visual(
        Cylinder(radius=rail_radius, length=lid_width - 0.070),
        origin=Origin(xyz=(rail_front_x, 0.0, rail_z), rpy=hinge_axis_rpy),
        material=chrome,
        name="front_rail",
    )
    lid.visual(
        Cylinder(radius=rail_radius, length=lid_width - 0.070),
        origin=Origin(xyz=(rail_rear_x, 0.0, rail_z), rpy=hinge_axis_rpy),
        material=chrome,
        name="rear_rail",
    )
    lid.visual(
        Cylinder(radius=rail_radius, length=lid_depth - 0.060),
        origin=Origin(xyz=(lid_center_x, rail_side_y, rail_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="side_rail_0",
    )
    lid.visual(
        Cylinder(radius=rail_radius, length=lid_depth - 0.060),
        origin=Origin(xyz=(lid_center_x, -rail_side_y, rail_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="side_rail_1",
    )
    for x in (rail_rear_x, rail_front_x):
        for y in (-rail_side_y, rail_side_y):
            lid.visual(
                Cylinder(radius=0.006, length=post_height),
                origin=Origin(xyz=(x, y, lid_center_z + lid_thickness / 2.0 + post_height / 2.0)),
                material=chrome,
                name=f"rail_post_{x:.2f}_{y:.2f}",
            )

    for hinge_index, y0 in enumerate(hinge_centers):
        lid.visual(
            Cylinder(radius=knuckle_radius, length=middle_len),
            origin=Origin(xyz=(0.0, y0, 0.0), rpy=hinge_axis_rpy),
            material=chrome,
            name=f"lid_knuckle_{hinge_index}",
        )
        lid.visual(
            Box((0.028, middle_len, 0.006)),
            origin=Origin(xyz=(lid_rear_offset / 2.0, y0, 0.0)),
            material=chrome,
            name=f"lid_hinge_wrap_{hinge_index}",
        )
        lid.visual(
            Box((0.060, middle_len, 0.004)),
            origin=Origin(xyz=(lid_rear_offset + 0.032, y0, lid_center_z + lid_thickness / 2.0 + 0.002)),
            material=chrome,
            name=f"lid_hinge_leaf_{hinge_index}",
        )

    latch_pivot_x = lid_front_x + 0.008
    latch_pivot_z = -0.018
    for y in (-0.033, 0.033):
        lid.visual(
            Box((0.016, 0.008, 0.024)),
            origin=Origin(xyz=(latch_pivot_x - 0.003, y, latch_pivot_z - 0.004)),
            material=chrome,
            name=f"latch_lug_{y:+.2f}",
        )

    front_latch = model.part("front_latch")
    front_latch.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(rpy=hinge_axis_rpy),
        material=chrome,
        name="latch_barrel",
    )
    front_latch.visual(
        Box((0.007, 0.050, 0.082)),
        origin=Origin(xyz=(0.006, 0.0, -0.043)),
        material=chrome,
        name="latch_plate",
    )
    front_latch.visual(
        Box((0.026, 0.042, 0.007)),
        origin=Origin(xyz=(-0.006, 0.0, -0.084)),
        material=chrome,
        name="lower_hook",
    )
    front_latch.visual(
        Box((0.005, 0.038, 0.018)),
        origin=Origin(xyz=(0.011, 0.0, -0.017)),
        material=chrome,
        name="thumb_tab",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=front_latch,
        origin=Origin(xyz=(latch_pivot_x, 0.0, latch_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.5, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("front_latch")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_hinge = object_model.get_articulation("lid_to_latch")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="lid_shell",
        negative_elem="top_gasket_front",
        name="closed lid sits just above gasket",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.300,
        elem_a="lid_shell",
        elem_b="bottom_pan",
        name="lid covers the rectangular chest footprint",
    )
    ctx.expect_overlap(
        latch,
        body,
        axes="yz",
        min_overlap=0.025,
        elem_a="latch_plate",
        elem_b="catch_plate",
        name="front latch aligns with center catch",
    )
    ctx.expect_gap(
        body,
        latch,
        axis="z",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="catch_loop",
        negative_elem="lower_hook",
        name="latch hook sits just below catch loop",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid hinge raises the front edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
    with ctx.pose({latch_hinge: 1.00}):
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
    closed_latch_x = None
    open_latch_x = None
    if closed_latch_aabb is not None:
        closed_latch_x = (closed_latch_aabb[0][0] + closed_latch_aabb[1][0]) / 2.0
    if open_latch_aabb is not None:
        open_latch_x = (open_latch_aabb[0][0] + open_latch_aabb[1][0]) / 2.0
    ctx.check(
        "flip latch rotates outward",
        closed_latch_x is not None and open_latch_x is not None and open_latch_x > closed_latch_x + 0.020,
        details=f"closed_x={closed_latch_x}, open_x={open_latch_x}",
    )

    return ctx.report()


object_model = build_object_model()
