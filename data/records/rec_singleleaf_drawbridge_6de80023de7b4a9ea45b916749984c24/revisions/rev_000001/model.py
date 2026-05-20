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

    concrete = Material("weathered_concrete", rgba=(0.55, 0.55, 0.50, 1.0))
    road = Material("dark_asphalt", rgba=(0.04, 0.045, 0.045, 1.0))
    steel = Material("painted_steel_blue", rgba=(0.08, 0.20, 0.34, 1.0))
    dark_steel = Material("dark_greased_steel", rgba=(0.02, 0.025, 0.03, 1.0))
    hazard = Material("yellow_safety_edge", rgba=(0.95, 0.72, 0.08, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((1.35, 4.20, 0.25)),
        origin=Origin(xyz=(-0.40, 0.0, 0.125)),
        material=concrete,
        name="foundation_slab",
    )
    shore_frame.visual(
        Box((0.50, 4.20, 0.32)),
        origin=Origin(xyz=(-0.45, 0.0, 0.405)),
        material=concrete,
        name="shore_abutment",
    )
    shore_frame.visual(
        Box((0.70, 2.95, 0.035)),
        origin=Origin(xyz=(-0.58, 0.0, 0.582)),
        material=road,
        name="approach_road",
    )
    shore_frame.visual(
        Box((0.16, 3.62, 0.16)),
        origin=Origin(xyz=(0.34, 0.0, 0.480)),
        material=dark_steel,
        name="rest_sill",
    )
    shore_frame.visual(
        Box((0.54, 0.36, 0.22)),
        origin=Origin(xyz=(0.0, -1.96, 0.36)),
        material=concrete,
        name="bearing_pedestal_0",
    )
    shore_frame.visual(
        Cylinder(radius=0.24, length=0.22),
        origin=Origin(xyz=(0.0, -1.793, 0.65), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_housing_0",
    )
    shore_frame.visual(
        Box((0.22, 0.38, 0.12)),
        origin=Origin(xyz=(0.0, -1.96, 0.31)),
        material=steel,
        name="bearing_plinth_0",
    )
    shore_frame.visual(
        Box((0.54, 0.36, 0.22)),
        origin=Origin(xyz=(0.0, 1.96, 0.36)),
        material=concrete,
        name="bearing_pedestal_1",
    )
    shore_frame.visual(
        Cylinder(radius=0.24, length=0.22),
        origin=Origin(xyz=(0.0, 1.793, 0.65), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_housing_1",
    )
    shore_frame.visual(
        Box((0.22, 0.38, 0.12)),
        origin=Origin(xyz=(0.0, 1.96, 0.31)),
        material=steel,
        name="bearing_plinth_1",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((4.75, 3.25, 0.18)),
        # The child frame is the shore-side hinge axis; the deck extends forward.
        origin=Origin(xyz=(2.475, 0.0, 0.0)),
        material=steel,
        name="deck_panel",
    )
    bridge_leaf.visual(
        Box((4.55, 2.70, 0.025)),
        origin=Origin(xyz=(2.555, 0.0, 0.102)),
        material=road,
        name="road_surface",
    )
    bridge_leaf.visual(
        Box((0.12, 3.12, 0.13)),
        origin=Origin(xyz=(4.78, 0.0, -0.02)),
        material=hazard,
        name="nose_beam",
    )
    for suffix, y in (("0", -1.58), ("1", 1.58)):
        bridge_leaf.visual(
            Box((4.35, 0.15, 0.34)),
            origin=Origin(xyz=(2.65, y, -0.045)),
            material=steel,
            name=f"side_girder_{suffix}",
        )
        bridge_leaf.visual(
            Box((4.10, 0.065, 0.065)),
            origin=Origin(xyz=(2.86, y, 0.31)),
            material=steel,
            name=f"guard_rail_{suffix}",
        )
        for idx, x in enumerate((1.05, 2.25, 3.45, 4.45)):
            bridge_leaf.visual(
                Box((0.065, 0.065, 0.34)),
                origin=Origin(xyz=(x, y, 0.205)),
                material=steel,
                name=f"rail_post_{suffix}_{idx}",
            )
    for idx, x in enumerate((0.70, 1.55, 2.40, 3.25, 4.10)):
        bridge_leaf.visual(
            Box((0.10, 3.05, 0.12)),
            origin=Origin(xyz=(x, 0.0, -0.145)),
            material=dark_steel,
            name=f"cross_girder_{idx}",
        )
    bridge_leaf.visual(
        Cylinder(radius=0.16, length=3.70),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_tube",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        # The leaf geometry extends along +X from the hinge; -Y raises the free end.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.35, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.allow_overlap(
        bridge_leaf,
        shore_frame,
        elem_a="hinge_tube",
        elem_b="bearing_housing_0",
        reason="The leaf trunnion shaft is intentionally captured inside the fixed side bearing housing.",
    )
    ctx.allow_overlap(
        bridge_leaf,
        shore_frame,
        elem_a="hinge_tube",
        elem_b="bearing_housing_1",
        reason="The leaf trunnion shaft is intentionally captured inside the fixed side bearing housing.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="deck_panel",
            negative_elem="rest_sill",
            min_gap=0.0,
            max_gap=0.020,
            name="closed leaf bears just above the shore sill",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="y",
            elem_a="deck_panel",
            elem_b="rest_sill",
            min_overlap=3.0,
            name="leaf spans most of the wide shore frame",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="xyz",
            elem_a="hinge_tube",
            elem_b="bearing_housing_0",
            min_overlap=0.10,
            name="hinge tube is retained in one side bearing",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="xyz",
            elem_a="hinge_tube",
            elem_b="bearing_housing_1",
            min_overlap=0.10,
            name="hinge tube is retained in the other side bearing",
        )

        closed_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")

    with ctx.pose({hinge: 1.05}):
        raised_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")

    ctx.check(
        "positive hinge motion raises the free end",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 3.0,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
