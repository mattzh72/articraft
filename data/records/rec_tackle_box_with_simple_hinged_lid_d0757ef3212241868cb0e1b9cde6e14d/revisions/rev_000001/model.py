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
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    body_blue = model.material("molded_blue_plastic", rgba=(0.05, 0.18, 0.34, 1.0))
    lid_blue = model.material("slightly_lighter_lid", rgba=(0.06, 0.24, 0.46, 1.0))
    tray_gray = model.material("matte_gray_tray", rgba=(0.48, 0.52, 0.53, 1.0))
    metal = model.material("brushed_steel_hinge", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_shadow = model.material("dark_recess", rgba=(0.025, 0.030, 0.035, 1.0))

    # Realistic compact tackle-box proportions in meters.
    width = 0.44
    depth = 0.26
    height = 0.16
    wall = 0.014
    floor = 0.014

    hinge_y = depth / 2.0 + 0.034
    hinge_z = height + 0.016
    hinge_r = 0.011
    cyl_x = Origin(rpy=(0.0, pi / 2.0, 0.0))

    body = model.part("body")

    # Open rectangular box body: floor plus four connected walls, not a solid block.
    body.visual(
        Box((width, depth, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=body_blue,
        name="floor",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, height / 2.0)),
        material=body_blue,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=body_blue,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=body_blue,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=body_blue,
        name="side_wall_1",
    )

    # Slightly raised rim/lip around the opening for the closed plain lid to sit above.
    rim_z = height + 0.004
    body.visual(
        Box((width + 0.010, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.001, rim_z)),
        material=body_blue,
        name="front_lip",
    )
    body.visual(
        Box((width + 0.010, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.001, rim_z)),
        material=body_blue,
        name="rear_lip",
    )
    body.visual(
        Box((0.010, depth, 0.012)),
        origin=Origin(xyz=(-width / 2.0 - 0.001, 0.0, rim_z)),
        material=body_blue,
        name="side_lip_0",
    )
    body.visual(
        Box((0.010, depth, 0.012)),
        origin=Origin(xyz=(width / 2.0 + 0.001, 0.0, rim_z)),
        material=body_blue,
        name="side_lip_1",
    )

    # Fixed internal tray details: a shallow organizer floor, rim and dividers.
    tray_z = 0.084
    tray_w = width - 2.0 * wall + 0.004
    tray_d = depth - 2.0 * wall + 0.004
    body.visual(
        Box((tray_w, tray_d, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, tray_z)),
        material=tray_gray,
        name="tray_floor",
    )
    body.visual(
        Box((tray_w, 0.006, 0.025)),
        origin=Origin(xyz=(0.0, -tray_d / 2.0 + 0.003, tray_z + 0.011)),
        material=tray_gray,
        name="tray_front_rim",
    )
    body.visual(
        Box((tray_w, 0.006, 0.025)),
        origin=Origin(xyz=(0.0, tray_d / 2.0 - 0.003, tray_z + 0.011)),
        material=tray_gray,
        name="tray_rear_rim",
    )
    body.visual(
        Box((0.006, tray_d, 0.025)),
        origin=Origin(xyz=(-tray_w / 2.0 + 0.003, 0.0, tray_z + 0.011)),
        material=tray_gray,
        name="tray_side_rim_0",
    )
    body.visual(
        Box((0.006, tray_d, 0.025)),
        origin=Origin(xyz=(tray_w / 2.0 - 0.003, 0.0, tray_z + 0.011)),
        material=tray_gray,
        name="tray_side_rim_1",
    )
    body.visual(
        Box((0.006, tray_d - 0.012, 0.027)),
        origin=Origin(xyz=(0.0, 0.0, tray_z + 0.011)),
        material=tray_gray,
        name="tray_divider_long",
    )
    for i, x in enumerate((-0.115, 0.115)):
        body.visual(
            Box((0.006, tray_d - 0.032, 0.027)),
            origin=Origin(xyz=(x, 0.0, tray_z + 0.011)),
            material=tray_gray,
            name=f"tray_divider_cross_{i}",
        )
    body.visual(
        Box((width - 2.0 * wall, depth - 2.0 * wall, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, floor + 0.002)),
        material=dark_shadow,
        name="bottom_shadow",
    )

    # Rear hinge hardware is deliberately pulled outside the rear wall so the
    # horizontal hinge axis is obvious in silhouette.
    body.visual(
        Box((0.39, 0.007, 0.034)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.005, height - 0.004)),
        material=metal,
        name="rear_hinge_leaf",
    )
    body.visual(
        Box((0.39, hinge_y - depth / 2.0, 0.006)),
        origin=Origin(xyz=(0.0, (hinge_y + depth / 2.0) / 2.0, hinge_z - hinge_r - 0.003)),
        material=metal,
        name="hinge_standoff",
    )
    for i, x in enumerate((-0.155, 0.155)):
        body.visual(
            Cylinder(radius=hinge_r, length=0.095),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=cyl_x.rpy),
            material=metal,
            name=f"body_knuckle_{i}",
        )

    lid = model.part("lid")
    lid_depth = 0.300
    lid_thick = 0.018
    lid.visual(
        Box((width + 0.030, lid_depth, lid_thick)),
        # The child frame is the hinge axis; at q=0 the plain panel projects
        # forward from that rear hinge line and rests just above the body rim.
        origin=Origin(xyz=(0.0, -0.018 - lid_depth / 2.0, 0.006)),
        material=lid_blue,
        name="lid_panel",
    )
    lid.visual(
        Box((width + 0.020, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.018 - lid_depth + 0.006, -0.005)),
        material=lid_blue,
        name="front_lid_lip",
    )
    lid.visual(
        Box((0.13, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, 0.001)),
        material=metal,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_x.rpy),
        material=metal,
        name="lid_knuckle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single rear revolute lid joint",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.0005,
        max_gap=0.035,
        positive_elem="lid_panel",
        negative_elem="front_lip",
        name="closed lid sits just above body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.18,
        elem_a="lid_panel",
        elem_b="floor",
        name="lid spans the rectangular box footprint",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="y",
        min_gap=0.018,
        positive_elem="lid_knuckle",
        negative_elem="rear_wall",
        name="hinge barrel is pulled outward behind rear wall",
    )
    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "opened lid rotates upward about rear hinge",
            closed_lid_aabb is not None
            and opened_lid_aabb is not None
            and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
            details=f"closed_aabb={closed_lid_aabb}, opened_aabb={opened_lid_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
