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

    blue_plastic = Material("slightly_mottled_blue_plastic", rgba=(0.05, 0.22, 0.42, 1.0))
    dark_tray = Material("dark_gray_fixed_tray", rgba=(0.08, 0.09, 0.10, 1.0))
    hinge_plastic = Material("black_hinge_knuckles", rgba=(0.015, 0.018, 0.020, 1.0))

    body = model.part("body")

    width = 0.56
    depth = 0.34
    height = 0.18
    wall = 0.024

    # Hollow rectangular body: floor plus four continuous upright walls.
    body.visual(
        Box((width, depth, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=blue_plastic,
        name="floor",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-(width / 2.0 - wall / 2.0), 0.0, height / 2.0)),
        material=blue_plastic,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=((width / 2.0 - wall / 2.0), 0.0, height / 2.0)),
        material=blue_plastic,
        name="side_wall_1",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, -(depth / 2.0 - wall / 2.0), height / 2.0)),
        material=blue_plastic,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, (depth / 2.0 - wall / 2.0), height / 2.0)),
        material=blue_plastic,
        name="rear_wall",
    )

    # Fixed internal tray insert, kept as part of the body and supported by side ledges.
    body.visual(
        Box((0.045, 0.260, 0.012)),
        origin=Origin(xyz=(-0.2335, 0.0, 0.092)),
        material=blue_plastic,
        name="tray_ledge_0",
    )
    body.visual(
        Box((0.045, 0.260, 0.012)),
        origin=Origin(xyz=(0.2335, 0.0, 0.092)),
        material=blue_plastic,
        name="tray_ledge_1",
    )
    body.visual(
        Box((0.480, 0.250, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_tray,
        name="tray_floor",
    )
    body.visual(
        Box((0.010, 0.230, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=dark_tray,
        name="long_divider",
    )
    body.visual(
        Box((0.440, 0.010, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=dark_tray,
        name="cross_divider",
    )
    body.visual(
        Box((0.010, 0.108, 0.022)),
        origin=Origin(xyz=(-0.120, -0.061, 0.116)),
        material=dark_tray,
        name="small_divider_0",
    )
    body.visual(
        Box((0.010, 0.108, 0.022)),
        origin=Origin(xyz=(0.120, 0.061, 0.116)),
        material=dark_tray,
        name="small_divider_1",
    )

    # Two body-side hinge knuckles along the rear edge.
    for index, x in enumerate((-0.215, 0.215)):
        body.visual(
            Box((0.110, 0.008, 0.008)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.004, height - 0.008)),
            material=hinge_plastic,
            name=f"body_hinge_leaf_{index}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.110),
            origin=Origin(xyz=(x, depth / 2.0 + 0.010, height + 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_plastic,
            name=f"body_hinge_knuckle_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.585, 0.340, 0.024)),
        # The child frame is the hinge pin center.  The uninterrupted panel
        # extends forward from the rear hinge line in local -Y.
        origin=Origin(xyz=(0.0, -0.180, 0.0085)),
        material=blue_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.585, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.359, -0.016)),
        material=blue_plastic,
        name="front_skirt",
    )
    lid.visual(
        Box((0.018, 0.320, 0.050)),
        origin=Origin(xyz=(-0.3015, -0.185, -0.016)),
        material=blue_plastic,
        name="side_skirt_0",
    )
    lid.visual(
        Box((0.018, 0.320, 0.050)),
        origin=Origin(xyz=(0.3015, -0.185, -0.016)),
        material=blue_plastic,
        name="side_skirt_1",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_plastic,
        name="lid_hinge_knuckle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.010, height + 0.005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="lid_panel",
        negative_elem="rear_wall",
        name="closed lid rests just above rear rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.28,
        elem_a="lid_panel",
        elem_b="floor",
        name="single lid panel covers the rectangular body",
    )
    ctx.expect_within(
        body,
        body,
        axes="xy",
        inner_elem="tray_floor",
        outer_elem="floor",
        margin=0.0,
        name="fixed tray stays within body footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.045,
            positive_elem="lid_panel",
            negative_elem="tray_floor",
            name="opened lid clears fixed tray",
        )
    ctx.check(
        "positive hinge angle raises front edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
