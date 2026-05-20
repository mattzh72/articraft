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

    body_green = model.material("molded_dark_green", rgba=(0.06, 0.24, 0.12, 1.0))
    lid_green = model.material("slightly_lighter_lid", rgba=(0.09, 0.34, 0.16, 1.0))
    tray_gray = model.material("fixed_tray_gray", rgba=(0.68, 0.70, 0.64, 1.0))
    hinge_black = model.material("blackened_steel_hinge", rgba=(0.02, 0.025, 0.025, 1.0))

    width = 0.48
    depth = 0.30
    body_height = 0.11
    wall = 0.012
    floor = 0.012
    lid_thickness = 0.016
    hinge_offset = 0.012
    hinge_radius = 0.010
    rear_clearance = 0.003

    rear_x = -depth / 2.0
    front_x = depth / 2.0
    hinge_x = rear_x - hinge_offset
    hinge_z = body_height + lid_thickness / 2.0

    body = model.part("body")
    body.visual(
        Box((depth, width, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=body_green,
        name="bottom_floor",
    )
    body.visual(
        Box((wall, width, body_height)),
        origin=Origin(xyz=(front_x - wall / 2.0, 0.0, body_height / 2.0)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((wall, width, body_height)),
        origin=Origin(xyz=(rear_x + wall / 2.0, 0.0, body_height / 2.0)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((depth, wall, body_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, body_height / 2.0)),
        material=body_green,
        name="side_wall_0",
    )
    body.visual(
        Box((depth, wall, body_height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, body_height / 2.0)),
        material=body_green,
        name="side_wall_1",
    )

    # Fixed organizer tray molded inside the body: shallow floor, two long bins,
    # and short dividers.  It is part of the body link so the only moving
    # mechanism remains the lid hinge.
    body.visual(
        Box((depth - 2.0 * wall - 0.012, width - 2.0 * wall - 0.018, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, floor + 0.003)),
        material=tray_gray,
        name="tray_floor",
    )
    body.visual(
        Box((depth - 2.0 * wall - 0.016, 0.008, 0.040)),
        origin=Origin(xyz=(0.006, 0.0, floor + 0.020)),
        material=tray_gray,
        name="long_divider",
    )
    for idx, x in enumerate((-0.065, 0.045)):
        body.visual(
            Box((0.008, width - 2.0 * wall - 0.026, 0.036)),
            origin=Origin(xyz=(x, 0.0, floor + 0.019)),
            material=tray_gray,
            name=f"cross_divider_{idx}",
        )

    # Alternating fixed hinge knuckles and their leaves on the rear wall.
    body_knuckle_length = 0.115
    for idx, y in enumerate((-0.155, 0.155)):
        body.visual(
            Cylinder(radius=hinge_radius, length=body_knuckle_length),
            origin=Origin(
                xyz=(hinge_x, y, hinge_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_black,
            name=f"hinge_knuckle_{idx}",
        )
        body.visual(
            Box((0.004, body_knuckle_length, 0.044)),
            origin=Origin(xyz=(rear_x - 0.001, y, body_height - 0.010)),
            material=hinge_black,
            name=f"hinge_leaf_{idx}",
        )

    lid = model.part("lid")
    lid_depth = depth - rear_clearance
    lid.visual(
        Box((lid_depth, width + 0.014, lid_thickness)),
        # The lid part frame is the rear hinge axis.  The panel begins just in
        # front of the hinge leaves and extends forward over almost the full box.
        origin=Origin(xyz=(hinge_offset + rear_clearance + lid_depth / 2.0, 0.0, 0.0)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_depth - 0.030, width - 0.050, 0.004)),
        origin=Origin(xyz=(hinge_offset + rear_clearance + lid_depth / 2.0, 0.0, lid_thickness / 2.0 + 0.002)),
        material=lid_green,
        name="raised_lid_field",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.170),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_black,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.026, 0.170, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, -0.005)),
        material=hinge_black,
        name="lid_hinge_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "only the lid is articulated",
        len(object_model.articulations) == 1,
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.00001,
            positive_elem="lid_panel",
            negative_elem="front_wall",
            name="closed lid rests on body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.25,
            elem_a="lid_panel",
            elem_b="bottom_floor",
            name="lid panel spans most of the box width",
        )

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.45}):
        opened_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "positive hinge motion opens the lid upward",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][2] > closed_panel[1][2] + 0.18,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    return ctx.report()


object_model = build_object_model()
