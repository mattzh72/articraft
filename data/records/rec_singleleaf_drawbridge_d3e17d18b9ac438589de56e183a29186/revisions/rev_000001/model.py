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
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = Material("weathered_concrete", color=(0.55, 0.56, 0.52, 1.0))
    dark_concrete = Material("dark_counterweight_concrete", color=(0.38, 0.39, 0.36, 1.0))
    steel_blue = Material("painted_blue_steel", color=(0.08, 0.18, 0.30, 1.0))
    worn_deck = Material("worn_asphalt_deck", color=(0.09, 0.095, 0.09, 1.0))
    safety_yellow = Material("safety_yellow", color=(0.95, 0.70, 0.08, 1.0))
    glass = Material("smoky_window_glass", color=(0.10, 0.16, 0.20, 0.65))

    shore = model.part("shore_frame")
    shore.visual(
        Box((1.70, 4.00, 0.30)),
        origin=Origin(xyz=(-0.35, 0.55, 0.15)),
        material=concrete,
        name="shore_slab",
    )
    shore.visual(
        Box((1.35, 1.85, 0.56)),
        origin=Origin(xyz=(-0.83, 0.00, 0.58)),
        material=concrete,
        name="approach_pier",
    )
    shore.visual(
        Box((1.25, 1.78, 0.16)),
        origin=Origin(xyz=(-0.85, 0.00, 0.92)),
        material=worn_deck,
        name="approach_road",
    )

    # The bridge opening is deliberately close to the light side of the fixed frame.
    # The extra mass and machinery sit on +Y, making that side visually heavier.
    shore.visual(
        Box((0.52, 0.38, 0.78)),
        origin=Origin(xyz=(0.00, -1.23, 0.69)),
        material=concrete,
        name="light_pedestal",
    )
    shore.visual(
        Box((0.62, 0.52, 0.82)),
        origin=Origin(xyz=(0.00, 1.30, 0.71)),
        material=concrete,
        name="heavy_pedestal",
    )
    shore.visual(
        Cylinder(radius=0.25, length=0.10),
        origin=Origin(xyz=(0.00, -1.09, 1.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_blue,
        name="light_bearing_disc",
    )
    shore.visual(
        Cylinder(radius=0.31, length=0.12),
        origin=Origin(xyz=(0.00, 1.10, 1.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_blue,
        name="heavy_bearing_disc",
    )
    shore.visual(
        Box((0.56, 0.82, 1.18)),
        origin=Origin(xyz=(-0.28, 1.78, 0.89)),
        material=dark_concrete,
        name="machinery_tower",
    )
    shore.visual(
        Box((0.74, 0.96, 0.44)),
        origin=Origin(xyz=(-0.28, 1.78, 1.68)),
        material=steel_blue,
        name="machinery_house",
    )
    shore.visual(
        Box((0.76, 0.055, 0.19)),
        origin=Origin(xyz=(-0.28, 1.29, 1.69)),
        material=glass,
        name="house_window",
    )
    shore.visual(
        Box((0.72, 0.58, 0.44)),
        origin=Origin(xyz=(0.10, 1.55, 1.23)),
        material=dark_concrete,
        name="counterweight_block",
    )
    shore.visual(
        Box((0.16, 0.16, 0.92)),
        origin=Origin(xyz=(0.18, 1.20, 0.92)),
        material=steel_blue,
        name="heavy_side_post",
    )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Cylinder(radius=0.14, length=2.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_blue,
        name="hinge_shaft",
    )
    leaf.visual(
        Box((4.25, 1.80, 0.16)),
        origin=Origin(xyz=(2.205, 0.00, -0.11)),
        material=worn_deck,
        name="deck_panel",
    )
    leaf.visual(
        Box((4.24, 0.14, 0.34)),
        origin=Origin(xyz=(2.205, -0.94, -0.23)),
        material=steel_blue,
        name="side_girder_0",
    )
    leaf.visual(
        Box((4.24, 0.14, 0.34)),
        origin=Origin(xyz=(2.205, 0.94, -0.23)),
        material=steel_blue,
        name="side_girder_1",
    )
    for i, x in enumerate((0.55, 1.45, 2.35, 3.25, 4.05)):
        leaf.visual(
            Box((0.10, 1.92, 0.10)),
            origin=Origin(xyz=(x, 0.00, -0.30)),
            material=steel_blue,
            name=f"cross_beam_{i}",
        )
    leaf.visual(
        Box((4.10, 0.045, 0.018)),
        origin=Origin(xyz=(2.28, 0.00, -0.034)),
        material=safety_yellow,
        name="center_stripe",
    )
    for i, x in enumerate((0.45, 1.20, 1.95, 2.70, 3.45, 4.15)):
        leaf.visual(
            Box((0.07, 0.07, 0.60)),
            origin=Origin(xyz=(x, -0.78, 0.27)),
            material=steel_blue,
            name=f"rail_post_0_{i}",
        )
        leaf.visual(
            Box((0.07, 0.07, 0.60)),
            origin=Origin(xyz=(x, 0.78, 0.27)),
            material=steel_blue,
            name=f"rail_post_1_{i}",
        )
    leaf.visual(
        Box((3.90, 0.08, 0.08)),
        origin=Origin(xyz=(2.35, -0.78, 0.56)),
        material=steel_blue,
        name="guardrail_0",
    )
    leaf.visual(
        Box((3.90, 0.08, 0.08)),
        origin=Origin(xyz=(2.35, 0.78, 0.56)),
        material=steel_blue,
        name="guardrail_1",
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.03)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.35, lower=0.0, upper=1.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    ctx.expect_contact(
        leaf,
        shore,
        elem_a="hinge_shaft",
        elem_b="heavy_bearing_disc",
        contact_tol=0.003,
        name="leaf trunnion meets heavy bearing",
    )
    ctx.expect_contact(
        leaf,
        shore,
        elem_a="hinge_shaft",
        elem_b="light_bearing_disc",
        contact_tol=0.003,
        name="leaf trunnion meets light bearing",
    )
    ctx.expect_gap(
        leaf,
        shore,
        axis="x",
        min_gap=0.25,
        max_gap=0.50,
        positive_elem="deck_panel",
        negative_elem="approach_road",
        name="deck leaves expansion gap at shore",
    )

    tower_aabb = ctx.part_element_world_aabb(shore, elem="machinery_tower")
    opening_center_y = 0.0
    ctx.check(
        "fixed mass is offset to heavy side",
        tower_aabb is not None and tower_aabb[0][1] > opening_center_y + 1.0,
        details=f"machinery_tower_aabb={tower_aabb}",
    )

    closed_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({hinge: 1.18}):
        raised_aabb = ctx.part_world_aabb(leaf)
    ctx.check(
        "leaf opens upward about shore axis",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 2.0,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
