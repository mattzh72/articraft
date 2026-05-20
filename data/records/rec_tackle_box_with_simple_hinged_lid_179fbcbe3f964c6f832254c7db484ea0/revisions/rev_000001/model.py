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

    dark_green = model.material("dark_green_plastic", rgba=(0.05, 0.24, 0.14, 1.0))
    tray_green = model.material("tray_green_plastic", rgba=(0.10, 0.42, 0.24, 1.0))
    lid_green = model.material("lid_green_plastic", rgba=(0.07, 0.32, 0.18, 1.0))

    # A compact, tall-narrow box: X is front/back, Y is the narrow width, Z is up.
    length = 0.46
    width = 0.24
    body_height = 0.30
    wall = 0.026
    floor_thickness = 0.026

    body = model.part("body")
    body.visual(
        Box((length, width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=dark_green,
        name="floor",
    )
    body.visual(
        Box((wall, width, body_height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=dark_green,
        name="front_wall",
    )
    body.visual(
        Box((0.062, width, body_height)),
        origin=Origin(xyz=(-length / 2.0 + 0.031, 0.0, body_height / 2.0)),
        material=dark_green,
        name="rear_wall",
    )
    body.visual(
        Box((length, wall, body_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, body_height / 2.0)),
        material=dark_green,
        name="side_wall_0",
    )
    body.visual(
        Box((length, wall, body_height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, body_height / 2.0)),
        material=dark_green,
        name="side_wall_1",
    )

    # Deep rear spine/frame below the hinge line, intentionally blockier than the
    # other sides so the hinge reads as being carried by a structural back rail.
    body.visual(
        Box((0.086, width + 0.012, 0.064)),
        origin=Origin(
            xyz=(-length / 2.0 + 0.020, 0.0, body_height - 0.032)
        ),
        material=dark_green,
        name="rear_hinge_frame",
    )
    # Fixed internal tray features: a raised shelf and grid dividers are part of
    # the body, not articulated tray links.
    tray_z = 0.168
    tray_floor_thickness = 0.012
    inner_length = length - 2.0 * wall + 0.010
    inner_width = width - 2.0 * wall + 0.010
    body.visual(
        Box((inner_length, inner_width, tray_floor_thickness)),
        origin=Origin(xyz=(0.012, 0.0, tray_z)),
        material=tray_green,
        name="fixed_tray_floor",
    )
    divider_height = 0.048
    divider_thickness = 0.010
    body.visual(
        Box((inner_length, divider_thickness, divider_height)),
        origin=Origin(
            xyz=(0.012, 0.0, tray_z + tray_floor_thickness / 2.0 + divider_height / 2.0)
        ),
        material=tray_green,
        name="long_divider",
    )
    body.visual(
        Box((divider_thickness, inner_width, divider_height)),
        origin=Origin(
            xyz=(-0.070, 0.0, tray_z + tray_floor_thickness / 2.0 + divider_height / 2.0)
        ),
        material=tray_green,
        name="cross_divider_0",
    )
    body.visual(
        Box((divider_thickness, inner_width, divider_height)),
        origin=Origin(
            xyz=(0.092, 0.0, tray_z + tray_floor_thickness / 2.0 + divider_height / 2.0)
        ),
        material=tray_green,
        name="cross_divider_1",
    )

    lid = model.part("lid")
    lid_length = length + 0.030
    lid_width = width + 0.052
    lid_thickness = 0.026
    skirt_thickness = 0.012
    skirt_height = 0.046
    lid.visual(
        Box((lid_length, lid_width, lid_thickness)),
        # The lid frame is the hinge axis; the closed panel extends forward in +X.
        origin=Origin(xyz=(lid_length / 2.0, 0.0, 0.0)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.014, length=lid_width + 0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lid_green,
        name="rear_hinge_barrel",
    )
    lid.visual(
        Box((lid_length, skirt_thickness, skirt_height)),
        origin=Origin(
            xyz=(
                lid_length / 2.0,
                lid_width / 2.0 - skirt_thickness / 2.0,
                -lid_thickness / 2.0 - skirt_height / 2.0,
            )
        ),
        material=lid_green,
        name="side_skirt_0",
    )
    lid.visual(
        Box((lid_length, skirt_thickness, skirt_height)),
        origin=Origin(
            xyz=(
                lid_length / 2.0,
                -lid_width / 2.0 + skirt_thickness / 2.0,
                -lid_thickness / 2.0 - skirt_height / 2.0,
            )
        ),
        material=lid_green,
        name="side_skirt_1",
    )
    lid.visual(
        Box((skirt_thickness, lid_width, skirt_height)),
        origin=Origin(
            xyz=(
                lid_length - skirt_thickness / 2.0,
                0.0,
                -lid_thickness / 2.0 - skirt_height / 2.0,
            )
        ),
        material=lid_green,
        name="front_skirt",
    )
    lid.visual(
        Box((0.030, lid_width - 0.030, 0.010)),
        origin=Origin(xyz=(lid_length / 2.0, 0.0, lid_thickness / 2.0 + 0.005)),
        material=lid_green,
        name="plain_lid_rib",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-length / 2.0, 0.0, body_height + 0.014)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.check(
        "single revolute lid joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "hinge axis runs along rear width",
        tuple(round(v, 3) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            min_gap=0.001,
            max_gap=0.004,
            name="closed lid sits just above rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="floor",
            min_overlap=0.20,
            name="closed lid covers box footprint",
        )
        ctx.expect_within(
            body,
            body,
            axes="xy",
            inner_elem="fixed_tray_floor",
            outer_elem="floor",
            margin=0.0,
            name="fixed tray stays inside body footprint",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="rear_hinge_barrel",
            negative_elem="rear_hinge_frame",
            max_penetration=0.00001,
            max_gap=0.004,
            name="lid barrel sits on deep rear hinge frame",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    closed_front_z = closed_aabb[1][2] if closed_aabb is not None else None
    with ctx.pose({hinge: 1.2}):
        opened_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        opened_front_z = opened_aabb[1][2] if opened_aabb is not None else None

    ctx.check(
        "positive hinge opens lid upward",
        closed_front_z is not None
        and opened_front_z is not None
        and opened_front_z > closed_front_z + 0.14,
        details=f"closed_front_z={closed_front_z}, opened_front_z={opened_front_z}",
    )

    return ctx.report()


object_model = build_object_model()
