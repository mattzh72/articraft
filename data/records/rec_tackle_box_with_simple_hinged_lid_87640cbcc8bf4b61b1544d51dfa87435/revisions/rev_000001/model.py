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
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    body_green = model.material("body_green", rgba=(0.08, 0.30, 0.18, 1.0))
    lid_green = model.material("lid_green", rgba=(0.10, 0.38, 0.22, 1.0))
    tray_tan = model.material("molded_tray_tan", rgba=(0.78, 0.72, 0.58, 1.0))
    dark_hardware = model.material("dark_hinge_hardware", rgba=(0.025, 0.028, 0.030, 1.0))

    width = 0.56
    depth = 0.32
    body_height = 0.18
    wall = 0.018
    floor_thickness = 0.018

    body = model.part("body")

    body.visual(
        Box((width, depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_green,
        name="floor",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_height / 2.0)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_height / 2.0)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=body_green,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=body_green,
        name="side_wall_1",
    )

    # A shallow fixed organizer tray sits inside the body.  The divider walls are
    # deliberately part of the body link, not separate joints.
    tray_floor_z = floor_thickness - 0.002
    body.visual(
        Box((0.505, 0.265, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, tray_floor_z + 0.005)),
        material=tray_tan,
        name="tray_floor",
    )
    body.visual(
        Box((0.014, 0.250, 0.060)),
        origin=Origin(xyz=(0.0, -0.005, tray_floor_z + 0.035)),
        material=tray_tan,
        name="long_divider",
    )
    for idx, y in enumerate((-0.085, 0.020, 0.105)):
        body.visual(
            Box((0.500, 0.012, 0.052)),
            origin=Origin(xyz=(0.0, y, tray_floor_z + 0.031)),
            material=tray_tan,
            name=f"cross_divider_{idx}",
        )
    for idx, (x, y) in enumerate(((-0.128, -0.033), (0.128, -0.033), (-0.128, 0.064), (0.128, 0.064))):
        body.visual(
            Box((0.012, 0.080, 0.045)),
            origin=Origin(xyz=(x, y, tray_floor_z + 0.027)),
            material=tray_tan,
            name=f"small_divider_{idx}",
        )

    hinge_radius = 0.013
    hinge_axis_y = depth / 2.0 + hinge_radius
    hinge_axis_z = body_height + 0.018

    for idx, x in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.130, 0.022, 0.017)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.003, body_height - 0.010)),
            material=dark_hardware,
            name=f"rear_hinge_bridge_{idx}",
        )
        body.visual(
            Box((0.130, 0.014, 0.040)),
            origin=Origin(xyz=(x, hinge_axis_y, body_height + 0.001)),
            material=dark_hardware,
            name=f"rear_hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.120),
            origin=Origin(xyz=(x, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_hardware,
            name=f"rear_hinge_barrel_{idx}",
        )

    lid = model.part("lid")
    lid_depth = depth + 0.036
    lid_width = width + 0.020
    lid_thickness = 0.024
    rear_clearance = hinge_radius

    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -rear_clearance - lid_depth / 2.0, -0.006)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.455, 0.205, 0.006)),
        origin=Origin(xyz=(0.0, -rear_clearance - lid_depth / 2.0, 0.009)),
        material=lid_green,
        name="raised_center_panel",
    )
    lid.visual(
        Box((lid_width - 0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -rear_clearance - lid_depth + 0.035, 0.010)),
        material=lid_green,
        name="front_lid_rib",
    )
    lid.visual(
        Box((lid_width - 0.060, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -rear_clearance - 0.045, 0.010)),
        material=lid_green,
        name="rear_lid_rib",
    )
    lid.visual(
        Box((0.010, lid_depth - 0.080, 0.008)),
        origin=Origin(xyz=(-lid_width / 2.0 + 0.035, -rear_clearance - lid_depth / 2.0, 0.010)),
        material=lid_green,
        name="side_lid_rib_0",
    )
    lid.visual(
        Box((0.010, lid_depth - 0.080, 0.008)),
        origin=Origin(xyz=(lid_width / 2.0 - 0.035, -rear_clearance - lid_depth / 2.0, 0.010)),
        material=lid_green,
        name="side_lid_rib_1",
    )
    lid.visual(
        Box((lid_width - 0.020, 0.012, 0.044)),
        origin=Origin(xyz=(0.0, -rear_clearance - lid_depth - 0.006, -0.021)),
        material=lid_green,
        name="front_skirt",
    )
    lid.visual(
        Box((0.012, lid_depth - 0.018, 0.040)),
        origin=Origin(xyz=(-lid_width / 2.0 - 0.002, -rear_clearance - lid_depth / 2.0 - 0.005, -0.020)),
        material=lid_green,
        name="side_skirt_0",
    )
    lid.visual(
        Box((0.012, lid_depth - 0.018, 0.040)),
        origin=Origin(xyz=(lid_width / 2.0 + 0.002, -rear_clearance - lid_depth / 2.0 - 0.005, -0.020)),
        material=lid_green,
        name="side_skirt_1",
    )
    lid.visual(
        Box((0.220, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, -0.026, -0.006)),
        material=dark_hardware,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="lid_hinge_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single rear lid hinge",
        len(object_model.articulations) == 1
        and hinge.axis == (-1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper > 1.5,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.00001,
            name="closed lid panel sits on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="floor",
            min_overlap=0.30,
            name="short deep lid covers the rectangular body footprint",
        )
        ctx.expect_within(
            body,
            body,
            axes="xy",
            inner_elem="tray_floor",
            outer_elem="floor",
            margin=0.0,
            name="fixed internal tray stays inside the body",
        )

        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({hinge: 1.35}):
        opened_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid opens upward about the rear horizontal axis",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.25,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
