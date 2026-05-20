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
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    green = Material("molded_deep_green", rgba=(0.05, 0.22, 0.15, 1.0))
    green_edge = Material("thick_lid_frame_green", rgba=(0.035, 0.16, 0.11, 1.0))
    tray_gray = Material("matte_tray_gray", rgba=(0.48, 0.51, 0.47, 1.0))
    hinge_black = Material("black_hinge_knuckles", rgba=(0.015, 0.017, 0.016, 1.0))

    width = 0.56
    depth = 0.36
    body_h = 0.20
    wall = 0.025
    floor_h = 0.025

    body = model.part("body")
    # Thick-walled open rectangular tub: floor plus four overlapping wall members.
    body.visual(
        Box((width, depth, floor_h)),
        origin=Origin(xyz=(0.0, 0.0, floor_h / 2.0)),
        material=green,
        name="floor",
    )
    body.visual(
        Box((width, wall, body_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_h / 2.0)),
        material=green,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, body_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_h / 2.0)),
        material=green,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth, body_h)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_h / 2.0)),
        material=green,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, body_h)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_h / 2.0)),
        material=green,
        name="side_wall_1",
    )

    # Fixed internal tray and compartment dividers, kept part of the body.
    tray_w = width - 2.0 * wall - 0.015
    tray_d = depth - 2.0 * wall - 0.012
    body.visual(
        Box((tray_w, tray_d, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=tray_gray,
        name="tray_plate",
    )
    body.visual(
        Box((0.012, tray_d, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=tray_gray,
        name="long_divider",
    )
    body.visual(
        Box((tray_w, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.055, 0.058)),
        material=tray_gray,
        name="cross_divider_0",
    )
    body.visual(
        Box((tray_w, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.060, 0.058)),
        material=tray_gray,
        name="cross_divider_1",
    )
    body.visual(
        Box((tray_w, 0.010, 0.035)),
        origin=Origin(xyz=(0.0, -tray_d / 2.0 + 0.005, 0.050)),
        material=tray_gray,
        name="tray_front_lip",
    )
    body.visual(
        Box((tray_w, 0.010, 0.035)),
        origin=Origin(xyz=(0.0, tray_d / 2.0 - 0.005, 0.050)),
        material=tray_gray,
        name="tray_rear_lip",
    )
    body.visual(
        Box((0.010, tray_d, 0.035)),
        origin=Origin(xyz=(-tray_w / 2.0 + 0.005, 0.0, 0.050)),
        material=tray_gray,
        name="tray_side_lip_0",
    )
    body.visual(
        Box((0.010, tray_d, 0.035)),
        origin=Origin(xyz=(tray_w / 2.0 - 0.005, 0.0, 0.050)),
        material=tray_gray,
        name="tray_side_lip_1",
    )

    hinge_y = depth / 2.0 + 0.015
    hinge_z = body_h + 0.030
    barrel_r = 0.012
    for x in (-0.195, 0.195):
        body.visual(
            Cylinder(radius=barrel_r, length=0.150),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_black,
            name=f"fixed_barrel_{0 if x < 0 else 1}",
        )
        body.visual(
            Box((0.150, 0.024, 0.030)),
            origin=Origin(xyz=(x, hinge_y - 0.009, hinge_z - 0.015)),
            material=hinge_black,
            name=f"fixed_leaf_{0 if x < 0 else 1}",
        )

    lid = model.part("lid")
    lid_w = width + 0.020
    lid_d = depth
    lid.visual(
        Box((lid_w, lid_d, 0.032)),
        # The child frame is on the hinge axis; the closed panel reaches forward
        # from the rear hinge line over the body.
        origin=Origin(xyz=(0.0, -0.201, -0.012)),
        material=green,
        name="panel",
    )
    lid.visual(
        Box((lid_w, 0.038, 0.018)),
        origin=Origin(xyz=(0.0, -0.345, 0.012)),
        material=green_edge,
        name="front_frame",
    )
    lid.visual(
        Box((lid_w, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, -0.060, 0.012)),
        material=green_edge,
        name="rear_frame",
    )
    lid.visual(
        Box((0.040, 0.300, 0.018)),
        origin=Origin(xyz=(-lid_w / 2.0 + 0.035, -0.200, 0.012)),
        material=green_edge,
        name="side_frame_0",
    )
    lid.visual(
        Box((0.040, 0.300, 0.018)),
        origin=Origin(xyz=(lid_w / 2.0 - 0.035, -0.200, 0.012)),
        material=green_edge,
        name="side_frame_1",
    )
    lid.visual(
        Cylinder(radius=barrel_r, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_black,
        name="moving_barrel",
    )
    lid.visual(
        Box((0.210, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, -0.018, -0.002)),
        material=hinge_black,
        name="moving_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.85),
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
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (-1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="panel",
            negative_elem="front_wall",
            min_gap=0.001,
            max_gap=0.006,
            name="closed lid panel rests just above box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="panel",
            elem_b="floor",
            min_overlap=0.32,
            name="lid panel covers rectangular body footprint",
        )
        ctx.expect_within(
            body,
            body,
            axes="xy",
            inner_elem="tray_plate",
            outer_elem="floor",
            margin=0.0,
            name="fixed tray stays inside the body footprint",
        )

        closed_panel = ctx.part_element_world_aabb(lid, elem="panel")

    with ctx.pose({hinge: 1.25}):
        open_panel = ctx.part_element_world_aabb(lid, elem="panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="panel",
            negative_elem="front_wall",
            min_gap=0.020,
            name="opened lid clears the front rim",
        )

    closed_max_z = closed_panel[1][2] if closed_panel else None
    open_max_z = open_panel[1][2] if open_panel else None
    ctx.check(
        "positive hinge rotation lifts the lid",
        closed_max_z is not None and open_max_z is not None and open_max_z > closed_max_z + 0.12,
        details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
