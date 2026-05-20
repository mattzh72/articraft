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

    body_green = Material("molded_green_plastic", rgba=(0.05, 0.26, 0.16, 1.0))
    rim_green = Material("darker_green_rim", rgba=(0.03, 0.16, 0.10, 1.0))
    tray_plastic = Material("cream_tray_plastic", rgba=(0.78, 0.72, 0.56, 1.0))
    lid_green = Material("lid_green_plastic", rgba=(0.07, 0.34, 0.20, 1.0))
    hinge_metal = Material("brushed_hinge_steel", rgba=(0.58, 0.58, 0.54, 1.0))

    # Realistic compact fishing tackle box proportions: about 46 cm long,
    # 24 cm wide, and 18 cm tall with a hollow molded tub.
    length = 0.46
    width = 0.24
    body_height = 0.160
    wall = 0.015
    floor = 0.014
    rim_height = 0.006

    body = model.part("body")
    body.visual(
        Box((length, width, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=body_green,
        name="floor",
    )
    body.visual(
        Box((wall, width, body_height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((wall, width, body_height)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((length, wall, body_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, body_height / 2.0)),
        material=body_green,
        name="side_wall_0",
    )
    body.visual(
        Box((length, wall, body_height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, body_height / 2.0)),
        material=body_green,
        name="side_wall_1",
    )

    # Molded top rim that the lid closes over without becoming a second joint.
    rim_z = body_height + rim_height / 2.0
    body.visual(
        Box((length, wall, rim_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, rim_z)),
        material=rim_green,
        name="top_rim_0",
    )
    body.visual(
        Box((length, wall, rim_height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, rim_z)),
        material=rim_green,
        name="top_rim_1",
    )
    body.visual(
        Box((wall, width, rim_height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, rim_z)),
        material=rim_green,
        name="front_rim",
    )
    body.visual(
        Box((wall, width, rim_height)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, rim_z)),
        material=rim_green,
        name="rear_rim",
    )

    # Fixed internal tray details: a shallow organizer and partitions molded or
    # clipped inside the tub, intentionally not articulated.
    tray_z = 0.094
    tray_thickness = 0.008
    tray_len = length - 2.0 * wall - 0.030
    tray_wid = width - 2.0 * wall - 0.025
    body.visual(
        Box((tray_len, tray_wid, tray_thickness)),
        origin=Origin(xyz=(0.010, 0.0, tray_z)),
        material=tray_plastic,
        name="tray_floor",
    )
    body.visual(
        Box((tray_len + 0.018, 0.010, 0.012)),
        origin=Origin(xyz=(0.010, tray_wid / 2.0 + 0.005, tray_z + 0.002)),
        material=tray_plastic,
        name="tray_side_0",
    )
    body.visual(
        Box((tray_len + 0.018, 0.010, 0.012)),
        origin=Origin(xyz=(0.010, -tray_wid / 2.0 - 0.005, tray_z + 0.002)),
        material=tray_plastic,
        name="tray_side_1",
    )
    body.visual(
        Box((0.010, tray_wid + 0.020, 0.012)),
        origin=Origin(xyz=(0.010 + tray_len / 2.0 + 0.005, 0.0, tray_z + 0.002)),
        material=tray_plastic,
        name="tray_front_end",
    )
    body.visual(
        Box((0.010, tray_wid + 0.020, 0.012)),
        origin=Origin(xyz=(0.010 - tray_len / 2.0 - 0.005, 0.0, tray_z + 0.002)),
        material=tray_plastic,
        name="tray_rear_end",
    )
    for divider_name, x in (
        ("tray_divider_0", -0.095),
        ("tray_divider_1", 0.035),
        ("tray_divider_2", 0.135),
    ):
        body.visual(
            Box((0.008, tray_wid, 0.030)),
            origin=Origin(xyz=(x, 0.0, tray_z + tray_thickness / 2.0 + 0.015)),
            material=tray_plastic,
            name=divider_name,
        )
    body.visual(
        Box((tray_len * 0.55, 0.007, 0.026)),
        origin=Origin(xyz=(-0.050, 0.0, tray_z + tray_thickness / 2.0 + 0.013)),
        material=tray_plastic,
        name="cross_divider",
    )

    # Short hinge hardware tucked against the rear wall. The two outer knuckles
    # are fixed to the body; the central knuckle belongs to the lid.
    hinge_x = -length / 2.0 - 0.010
    hinge_z = body_height + rim_height + 0.001
    hinge_radius = 0.006
    hinge_rpy = (-math.pi / 2.0, 0.0, 0.0)
    for idx, y in enumerate((-0.075, 0.075)):
        body.visual(
            Box((0.006, 0.060, 0.018)),
            origin=Origin(xyz=(hinge_x + hinge_radius + 0.002, y, hinge_z)),
            material=hinge_metal,
            name=f"hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.052),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=hinge_rpy),
            material=hinge_metal,
            name=f"hinge_barrel_{idx}",
        )

    lid = model.part("lid")
    lid_length = length
    lid_width = width + 0.012
    lid_thickness = 0.020
    lid_rear_offset = 0.010
    lid.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(
            xyz=(lid_rear_offset + lid_length / 2.0, 0.0, 0.001 + lid_thickness / 2.0)
        ),
        material=lid_green,
        name="lid_panel",
    )
    # Low molded stiffening ribs keep the panel plain but not featureless.
    lid.visual(
        Box((lid_length * 0.72, 0.010, 0.006)),
        origin=Origin(xyz=(lid_rear_offset + lid_length * 0.50, 0.0, 0.024)),
        material=lid_green,
        name="center_rib",
    )
    lid.visual(
        Box((0.010, lid_width - 0.030, 0.006)),
        origin=Origin(xyz=(lid_rear_offset + lid_length - 0.035, 0.0, 0.024)),
        material=lid_green,
        name="front_rib",
    )
    lid.visual(
        Box((lid_length - 0.055, 0.008, 0.006)),
        origin=Origin(
            xyz=(lid_rear_offset + lid_length / 2.0, lid_width / 2.0 - 0.018, 0.024)
        ),
        material=lid_green,
        name="side_rib_0",
    )
    lid.visual(
        Box((lid_length - 0.055, 0.008, 0.006)),
        origin=Origin(
            xyz=(lid_rear_offset + lid_length / 2.0, -lid_width / 2.0 + 0.018, 0.024)
        ),
        material=lid_green,
        name="side_rib_1",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
        material=hinge_metal,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.030, 0.066, 0.006)),
        origin=Origin(xyz=(0.015, 0.0, 0.004)),
        material=hinge_metal,
        name="hinge_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single rear revolute lid joint",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_rim",
        min_gap=0.001,
        max_gap=0.006,
        name="closed lid clears the fixed rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="floor",
        min_overlap=0.20,
        name="closed lid spans the rectangular body footprint",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="tray_divider_1",
        min_gap=0.025,
        name="fixed tray details stay below the lid",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "positive hinge motion opens lid upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
