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
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_W = 0.360
BODY_D = 0.220
BODY_H = 0.140
WALL = 0.012
HINGE_Y = 0.127
HINGE_Z = 0.153


def _cyl_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_sewing_box")

    warm_cherry = model.material("warm_cherry_wood", rgba=(0.55, 0.30, 0.14, 1.0))
    dark_cherry = model.material("dark_cherry_endgrain", rgba=(0.33, 0.17, 0.08, 1.0))
    brass = model.material("aged_brass", rgba=(0.86, 0.64, 0.28, 1.0))
    satin_lining = model.material("burgundy_satin_lining", rgba=(0.55, 0.06, 0.13, 1.0))
    felt = model.material("cream_wool_felt", rgba=(0.84, 0.77, 0.62, 1.0))
    red_thread = model.material("red_thread", rgba=(0.82, 0.06, 0.05, 1.0))
    blue_thread = model.material("blue_thread", rgba=(0.05, 0.20, 0.70, 1.0))
    ivory_thread = model.material("ivory_thread", rgba=(0.93, 0.88, 0.75, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    leather = model.material("dark_brown_leather", rgba=(0.20, 0.10, 0.04, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=warm_cherry,
        name="bottom_panel",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H - WALL)),
        origin=Origin(xyz=(-BODY_W / 2.0 + WALL / 2.0, 0.0, (BODY_H + WALL) / 2.0)),
        material=warm_cherry,
        name="side_wall_0",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H - WALL)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL / 2.0, 0.0, (BODY_H + WALL) / 2.0)),
        material=warm_cherry,
        name="side_wall_1",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL, WALL, BODY_H - WALL)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - WALL / 2.0, (BODY_H + WALL) / 2.0)),
        material=warm_cherry,
        name="back_wall",
    )

    # The front is built as real joinery around a sliding lower organizer drawer.
    body.visual(
        Box((0.060, WALL, BODY_H - WALL)),
        origin=Origin(xyz=(-0.150, -BODY_D / 2.0 + WALL / 2.0, (BODY_H + WALL) / 2.0)),
        material=warm_cherry,
        name="front_stile_0",
    )
    body.visual(
        Box((0.060, WALL, BODY_H - WALL)),
        origin=Origin(xyz=(0.150, -BODY_D / 2.0 + WALL / 2.0, (BODY_H + WALL) / 2.0)),
        material=warm_cherry,
        name="front_stile_1",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL, WALL, 0.013)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + WALL / 2.0, 0.0185)),
        material=warm_cherry,
        name="front_bottom_rail",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL, WALL, 0.032)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + WALL / 2.0, 0.124)),
        material=warm_cherry,
        name="front_top_rail",
    )

    # Darker miter-like corner posts and slim brass edge bands make the box read
    # as a finished household craft item rather than a plain crate.
    for x in (-0.181, 0.181):
        for y in (-0.111, 0.111):
            body.visual(
                Box((0.010, 0.010, BODY_H + 0.006)),
                origin=Origin(xyz=(x, y, (BODY_H + 0.006) / 2.0)),
                material=dark_cherry,
                name=f"corner_post_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
    body.visual(
        Box((0.350, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.113, 0.132)),
        material=brass,
        name="front_brass_band",
    )
    body.visual(
        Box((0.350, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.113, 0.132)),
        material=brass,
        name="rear_brass_band",
    )

    body.visual(
        Box((0.318, 0.184, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=satin_lining,
        name="upper_tray_floor",
    )
    body.visual(
        Box((0.006, 0.184, 0.035)),
        origin=Origin(xyz=(-0.055, 0.0, 0.1125)),
        material=satin_lining,
        name="tray_divider_long",
    )
    body.visual(
        Box((0.116, 0.006, 0.032)),
        origin=Origin(xyz=(-0.112, 0.005, 0.111)),
        material=satin_lining,
        name="tray_divider_short_0",
    )
    body.visual(
        Box((0.252, 0.006, 0.032)),
        origin=Origin(xyz=(0.058, -0.035, 0.111)),
        material=satin_lining,
        name="tray_divider_short_1",
    )
    for index, x in enumerate((-0.088, 0.088)):
        body.visual(
            Box((0.014, 0.150, 0.019)),
            origin=Origin(xyz=(x, -0.035, 0.0215)),
            material=dark_cherry,
            name=f"drawer_runner_{index}",
        )

    # A few mounted sewing notions in the upper organizer give the interior scale.
    for index, (x, material) in enumerate(((-0.125, red_thread), (-0.090, blue_thread), (-0.020, ivory_thread))):
        body.visual(
            Cylinder(radius=0.013, length=0.030),
            origin=Origin(xyz=(x, 0.052, 0.1095)),
            material=material,
            name=f"thread_spool_{index}",
        )
        body.visual(
            Cylinder(radius=0.0032, length=0.038),
            origin=Origin(xyz=(x, 0.052, 0.1095)),
            material=dark_cherry,
            name=f"spool_dowel_{index}",
        )
    body.visual(
        Box((0.070, 0.044, 0.018)),
        origin=Origin(xyz=(0.096, -0.058, 0.1035)),
        material=felt,
        name="pin_cushion",
    )
    for index, y in enumerate((-0.069, -0.058, -0.047)):
        body.visual(
            Cylinder(radius=0.0009, length=0.058),
            origin=Origin(xyz=(0.096, y, 0.1130), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"pin_{index}",
        )

    body.visual(
        Box((0.050, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, -0.112, 0.128)),
        material=brass,
        name="latch_keeper",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.054),
        origin=Origin(xyz=(0.0, -0.116, 0.124), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="latch_loop",
    )
    for x in (-0.145, 0.145):
        for y in (-0.085, 0.085):
            body.visual(
                Cylinder(radius=0.011, length=0.008),
                origin=Origin(xyz=(x, y, -0.004)),
                material=dark_cherry,
                name=f"foot_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    # Interleaved hinge knuckles on the fixed body side.
    for index, x in enumerate((-0.128, 0.0, 0.128)):
        body.visual(
            _cyl_x(0.0042, 0.044),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"hinge_knuckle_body_{index}",
        )
        body.visual(
            Box((0.044, 0.023, 0.016)),
            origin=Origin(xyz=(x, 0.1185, 0.144)),
            material=brass,
            name=f"hinge_leaf_body_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.374, 0.226, 0.018)),
        origin=Origin(xyz=(0.0, -0.135, 0.0)),
        material=warm_cherry,
        name="lid_panel",
    )
    lid.visual(
        Box((0.324, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.035, -0.016)),
        material=dark_cherry,
        name="inner_lip_rear",
    )
    lid.visual(
        Box((0.324, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.219, -0.016)),
        material=dark_cherry,
        name="inner_lip_front",
    )
    lid.visual(
        Box((0.008, 0.184, 0.014)),
        origin=Origin(xyz=(-0.162, -0.127, -0.016)),
        material=dark_cherry,
        name="inner_lip_0",
    )
    lid.visual(
        Box((0.008, 0.184, 0.014)),
        origin=Origin(xyz=(0.162, -0.127, -0.016)),
        material=dark_cherry,
        name="inner_lip_1",
    )
    lid.visual(
        Box((0.255, 0.136, 0.003)),
        origin=Origin(xyz=(0.0, -0.136, 0.0105)),
        material=dark_cherry,
        name="recessed_top_panel",
    )
    lid.visual(
        Box((0.070, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.244, -0.008)),
        material=brass,
        name="front_latch_plate",
    )
    for index, x in enumerate((-0.064, 0.064)):
        lid.visual(
            _cyl_x(0.0040, 0.044),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"hinge_knuckle_lid_{index}",
        )
        lid.visual(
            Box((0.044, 0.020, 0.004)),
            origin=Origin(xyz=(x, -0.013, -0.004)),
            material=brass,
            name=f"hinge_leaf_lid_{index}",
        )
    for x in (-0.132, 0.132):
        lid.visual(
            Box((0.030, 0.018, 0.016)),
            origin=Origin(xyz=(x, -0.106, 0.017)),
            material=brass,
            name=f"handle_mount_{0 if x < 0 else 1}",
        )
        lid.visual(
            Cylinder(radius=0.005, length=0.033),
            origin=Origin(xyz=(x, -0.106, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"handle_pivot_cap_{0 if x < 0 else 1}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.226, 0.012, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warm_cherry,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.196, 0.140, 0.006)),
        origin=Origin(xyz=(0.0, 0.075, -0.032)),
        material=warm_cherry,
        name="drawer_box_bottom",
    )
    drawer.visual(
        Box((0.006, 0.140, 0.040)),
        origin=Origin(xyz=(-0.101, 0.075, -0.009)),
        material=warm_cherry,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.006, 0.140, 0.040)),
        origin=Origin(xyz=(0.101, 0.075, -0.009)),
        material=warm_cherry,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.196, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.145, -0.009)),
        material=warm_cherry,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.006, 0.128, 0.030)),
        origin=Origin(xyz=(-0.030, 0.078, -0.006)),
        material=satin_lining,
        name="drawer_divider_long",
    )
    drawer.visual(
        Box((0.126, 0.006, 0.030)),
        origin=Origin(xyz=(0.035, 0.066, -0.006)),
        material=satin_lining,
        name="drawer_divider_short",
    )
    drawer.visual(
        Box((0.050, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.008, 0.001)),
        material=brass,
        name="drawer_pull_plate",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.045),
        origin=Origin(xyz=(0.0, -0.015, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="drawer_pull_bar",
    )

    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.132, 0.000, 0.008),
                (-0.132, -0.045, 0.008),
                (-0.082, -0.083, 0.008),
                (0.000, -0.092, 0.008),
                (0.082, -0.083, 0.008),
                (0.132, -0.045, 0.008),
                (0.132, 0.000, 0.008),
            ],
            radius=0.004,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "folding_carry_handle",
    )
    handle = model.part("handle")
    handle.visual(handle_mesh, material=leather, name="handle_loop")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.116, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, -0.106, 0.017)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    handle_hinge = object_model.get_articulation("lid_to_handle")

    for cap_name in ("handle_pivot_cap_0", "handle_pivot_cap_1"):
        ctx.allow_overlap(
            handle,
            lid,
            elem_a="handle_loop",
            elem_b=cap_name,
            reason="The fold-flat carry handle is intentionally captured on brass pivot caps.",
        )
        ctx.expect_gap(
            handle,
            lid,
            axis="z",
            positive_elem="handle_loop",
            negative_elem=cap_name,
            max_penetration=0.009,
            name=f"captured handle pivot depth {cap_name[-1]}",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="side_wall_0",
        min_gap=0.000,
        max_gap=0.004,
        name="closed lid sits just above body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="bottom_panel",
        min_overlap=0.18,
        name="lid covers compact storage body",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="drawer_front",
        elem_b="front_top_rail",
        min_overlap=0.20,
        name="drawer front fits within joinery",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_loop")
    drawer_rest = ctx.part_world_position(drawer)

    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "hinged lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({drawer_slide: 0.100}):
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_box_bottom",
            elem_b="bottom_panel",
            min_overlap=0.025,
            name="extended drawer remains retained in box",
        )
    ctx.check(
        "organizer drawer slides forward",
        drawer_rest is not None
        and drawer_extended is not None
        and float(drawer_extended[1]) < float(drawer_rest[1]) - 0.085,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    with ctx.pose({handle_hinge: 1.10}):
        raised_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_loop")
    ctx.check(
        "folding carry handle raises above lid",
        closed_handle_aabb is not None
        and raised_handle_aabb is not None
        and float(raised_handle_aabb[1][2]) > float(closed_handle_aabb[1][2]) + 0.055,
        details=f"stored={closed_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
