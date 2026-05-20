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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flute_case")

    shell = model.material("black_vinyl_shell", rgba=(0.015, 0.014, 0.013, 1.0))
    edge = model.material("black_edge_binding", rgba=(0.035, 0.033, 0.030, 1.0))
    felt = model.material("blue_plush_lining", rgba=(0.015, 0.055, 0.16, 1.0))
    hardware = model.material("brushed_nickel_hardware", rgba=(0.72, 0.68, 0.60, 1.0))
    rubber = model.material("dark_rubber_feet", rgba=(0.005, 0.005, 0.004, 1.0))

    length = 0.72
    width = 0.12
    wall = 0.008
    base_thickness = 0.012
    tray_wall_height = 0.028
    tray_top = base_thickness + tray_wall_height
    hinge_y = width / 2.0 + 0.005
    hinge_z = tray_top

    lower_tray = model.part("lower_tray")
    lower_tray.visual(
        Box((length, width, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=shell,
        name="bottom_panel",
    )
    lower_tray.visual(
        Box((length, wall, tray_wall_height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, base_thickness + tray_wall_height / 2.0)),
        material=shell,
        name="front_wall",
    )
    lower_tray.visual(
        Box((length, wall, tray_wall_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, base_thickness + tray_wall_height / 2.0)),
        material=shell,
        name="rear_wall",
    )
    lower_tray.visual(
        Box((wall, width - 2.0 * wall, tray_wall_height)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, base_thickness + tray_wall_height / 2.0)),
        material=shell,
        name="end_wall_0",
    )
    lower_tray.visual(
        Box((wall, width - 2.0 * wall, tray_wall_height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, base_thickness + tray_wall_height / 2.0)),
        material=shell,
        name="end_wall_1",
    )
    lower_tray.visual(
        Box((length - 0.055, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.024, base_thickness + 0.003)),
        material=felt,
        name="front_felt_rail",
    )
    lower_tray.visual(
        Box((length - 0.055, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.024, base_thickness + 0.003)),
        material=felt,
        name="rear_felt_rail",
    )
    lower_tray.visual(
        Box((length - 0.030, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.0025)),
        material=felt,
        name="center_felt_strip",
    )
    for x in (-0.245, 0.245):
        lower_tray.visual(
            Cylinder(radius=0.006, length=0.065),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"rear_hinge_knuckle_{0 if x < 0 else 1}",
        )
        lower_tray.visual(
            Box((0.065, 0.004, 0.018)),
            origin=Origin(xyz=(x, width / 2.0 + 0.001, tray_top - 0.009)),
            material=hardware,
            name=f"rear_hinge_leaf_{0 if x < 0 else 1}",
        )
    for x in (-0.30, 0.30):
        lower_tray.visual(
            Box((0.055, 0.020, 0.004)),
            origin=Origin(xyz=(x, -0.020, 0.002)),
            material=rubber,
            name=f"foot_{0 if x < 0 else 1}",
        )

    lid = model.part("lid")
    lid_depth = width
    top_thickness = 0.010
    lid_skirt_height = 0.022
    lid_top_center_z = lid_skirt_height + top_thickness / 2.0
    lid_center_y = -(lid_depth / 2.0 + 0.005)
    lid.visual(
        Box((length, lid_depth, top_thickness)),
        origin=Origin(xyz=(0.0, lid_center_y, lid_top_center_z)),
        material=shell,
        name="top_panel",
    )
    lid.visual(
        Box((length, wall, lid_skirt_height)),
        origin=Origin(xyz=(0.0, -lid_depth - 0.005 + wall / 2.0, lid_skirt_height / 2.0)),
        material=shell,
        name="front_lip",
    )
    lid.visual(
        Box((length, wall, lid_skirt_height)),
        origin=Origin(xyz=(0.0, -0.005 - wall / 2.0, lid_skirt_height / 2.0)),
        material=shell,
        name="rear_lip",
    )
    lid.visual(
        Box((wall, lid_depth - 2.0 * wall, lid_skirt_height)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, lid_center_y, lid_skirt_height / 2.0)),
        material=shell,
        name="end_lip_0",
    )
    lid.visual(
        Box((wall, lid_depth - 2.0 * wall, lid_skirt_height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, lid_center_y, lid_skirt_height / 2.0)),
        material=shell,
        name="end_lip_1",
    )
    lid.visual(
        Box((length - 0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, lid_center_y, lid_skirt_height + top_thickness + 0.003)),
        material=edge,
        name="raised_center_rib",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((0.110, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.004, 0.010)),
        material=hardware,
        name="center_hinge_leaf",
    )
    for i, x in enumerate((-0.075, 0.075)):
        lid.visual(
            Box((0.026, 0.006, 0.020)),
            origin=Origin(xyz=(x, -lid_depth - 0.008, 0.018)),
            material=hardware,
            name=f"pivot_block_{i}",
        )
        lid.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(xyz=(x, -lid_depth - 0.011, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"pivot_boss_{i}",
        )

    model.articulation(
        "tray_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_tray,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    front_handle = model.part("front_handle")
    handle_span = 0.150
    handle_drop = 0.032
    handle_radius = 0.0032
    for i, x in enumerate((-handle_span / 2.0, handle_span / 2.0)):
        front_handle.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"pivot_sleeve_{i}",
        )
        front_handle.visual(
            Cylinder(radius=handle_radius, length=handle_drop),
            origin=Origin(xyz=(x, 0.0, -handle_drop / 2.0)),
            material=hardware,
            name=f"handle_arm_{i}",
        )
        front_handle.visual(
            Sphere(radius=handle_radius * 1.12),
            origin=Origin(xyz=(x, 0.0, -handle_drop)),
            material=hardware,
            name=f"handle_elbow_{i}",
        )
    front_handle.visual(
        Cylinder(radius=0.0040, length=handle_span),
        origin=Origin(xyz=(0.0, 0.0, -handle_drop), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="grip_bar",
    )

    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=front_handle,
        origin=Origin(xyz=(0.0, -lid_depth - 0.016, 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_tray = object_model.get_part("lower_tray")
    lid = object_model.get_part("lid")
    front_handle = object_model.get_part("front_handle")
    lid_hinge = object_model.get_articulation("tray_to_lid")
    handle_pivot = object_model.get_articulation("lid_to_handle")

    for i in (0, 1):
        ctx.allow_overlap(
            lid,
            front_handle,
            elem_a=f"pivot_boss_{i}",
            elem_b=f"pivot_sleeve_{i}",
            reason="The handle sleeve is intentionally captured on the visible front pivot pin.",
        )
        ctx.expect_overlap(
            lid,
            front_handle,
            axes="x",
            min_overlap=0.010,
            elem_a=f"pivot_boss_{i}",
            elem_b=f"pivot_sleeve_{i}",
            name=f"handle sleeve {i} remains on its pivot pin",
        )

    with ctx.pose({lid_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_gap(
            lid,
            lower_tray,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="front_lip",
            negative_elem="front_wall",
            name="closed lid lip sits on the shallow tray front wall",
        )
        ctx.expect_overlap(
            lid,
            lower_tray,
            axes="xy",
            min_overlap=0.10,
            elem_a="top_panel",
            elem_b="bottom_panel",
            name="broad top lid covers the narrow rectangular tray",
        )

        closed_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
        closed_grip_aabb = ctx.part_element_world_aabb(front_handle, elem="grip_bar")

    with ctx.pose({lid_hinge: 1.10, handle_pivot: 0.0}):
        open_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")

    ctx.check(
        "rear hinge lifts the lid front edge",
        closed_lip_aabb is not None
        and open_lip_aabb is not None
        and open_lip_aabb[0][2] > closed_lip_aabb[0][2] + 0.055,
        details=f"closed={closed_lip_aabb}, open={open_lip_aabb}",
    )

    with ctx.pose({lid_hinge: 0.0, handle_pivot: 1.10}):
        raised_grip_aabb = ctx.part_element_world_aabb(front_handle, elem="grip_bar")

    ctx.check(
        "fold-flat handle swings outward from front pivots",
        closed_grip_aabb is not None
        and raised_grip_aabb is not None
        and raised_grip_aabb[0][1] < closed_grip_aabb[0][1] - 0.020,
        details=f"folded={closed_grip_aabb}, raised={raised_grip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
