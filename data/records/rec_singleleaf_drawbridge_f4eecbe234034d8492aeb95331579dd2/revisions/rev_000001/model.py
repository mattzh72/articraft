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
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", rgba=(0.47, 0.48, 0.46, 1.0))
    dark_steel = model.material("painted_dark_steel", rgba=(0.08, 0.12, 0.13, 1.0))
    bearing_metal = model.material("oiled_bearing_metal", rgba=(0.03, 0.035, 0.035, 1.0))
    deck_brown = model.material("timber_deck", rgba=(0.45, 0.27, 0.13, 1.0))
    road_black = model.material("worn_blacktop", rgba=(0.025, 0.025, 0.023, 1.0))
    safety_yellow = model.material("painted_yellow", rgba=(0.95, 0.73, 0.08, 1.0))
    water_blue = model.material("canal_water", rgba=(0.05, 0.18, 0.28, 0.72))

    shore = model.part("shore_frame")

    # Fixed abutment, approach deck, and canal surface.  The bridge leaf's hinge
    # line sits at z=0.65 m, matching the top surface of the approach deck.
    shore.visual(
        Box((1.25, 3.05, 0.20)),
        origin=Origin(xyz=(-0.40, 0.0, 0.10)),
        material=concrete,
        name="abutment_slab",
    )
    shore.visual(
        Box((0.90, 2.30, 0.16)),
        origin=Origin(xyz=(-0.57, 0.0, 0.57)),
        material=road_black,
        name="approach_deck",
    )
    shore.visual(
        Box((0.18, 2.90, 0.55)),
        origin=Origin(xyz=(-0.20, 0.0, 0.375)),
        material=concrete,
        name="abutment_wall",
    )
    shore.visual(
        Box((2.90, 2.85, 0.025)),
        origin=Origin(xyz=(1.42, 0.0, 0.035)),
        material=water_blue,
        name="water_plane",
    )

    # Paired heavy side cheeks frame the moving leaf and carry the bearing pads.
    for side, y, cheek_name, liner_name in (
        (0, -1.265, "side_cheek_0", "inner_bearing_liner_0"),
        (1, 1.265, "side_cheek_1", "inner_bearing_liner_1"),
    ):
        shore.visual(
            Box((0.50, 0.18, 0.78)),
            origin=Origin(xyz=(0.00, y, 0.59)),
            material=dark_steel,
            name=cheek_name,
        )
        shore.visual(
            Box((0.44, 0.24, 0.12)),
            origin=Origin(xyz=(-0.02, y, 0.26)),
            material=dark_steel,
            name=f"cheek_foot_{side}",
        )
        shore.visual(
            Cylinder(radius=0.18, length=0.16),
            origin=Origin(xyz=(0.0, y, 0.65), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"outer_bearing_housing_{side}",
        )
        liner_y = -1.185 if y < 0.0 else 1.185
        shore.visual(
            Cylinder(radius=0.115, length=0.020),
            origin=Origin(xyz=(0.0, liner_y, 0.65), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_metal,
            name=liner_name,
        )

    # A low rear tie makes the two cheeks read as one fixed shore frame.
    shore.visual(
        Box((0.14, 2.75, 0.18)),
        origin=Origin(xyz=(-0.23, 0.0, 0.78)),
        material=dark_steel,
        name="rear_tie_beam",
    )

    leaf = model.part("bridge_leaf")
    # The leaf part frame is exactly on the shore-side bearing axis.  At q=0
    # the panel extends along +X; positive revolute motion raises the free end.
    leaf.visual(
        Box((2.70, 2.02, 0.16)),
        origin=Origin(xyz=(1.35, 0.0, -0.08)),
        material=deck_brown,
        name="deck_panel",
    )
    leaf.visual(
        Box((2.66, 1.72, 0.030)),
        origin=Origin(xyz=(1.38, 0.0, 0.017)),
        material=road_black,
        name="road_surface",
    )
    for stripe_x in (0.55, 1.20, 1.85, 2.50):
        leaf.visual(
            Box((0.38, 0.045, 0.012)),
            origin=Origin(xyz=(stripe_x, 0.0, 0.0375)),
            material=safety_yellow,
            name=f"center_stripe_{stripe_x:.2f}",
        )
    for side, y, girder_name in (
        (0, -1.03, "side_girder_0"),
        (1, 1.03, "side_girder_1"),
    ):
        leaf.visual(
            Box((2.50, 0.12, 0.30)),
            origin=Origin(xyz=(1.40, y, -0.07)),
            material=dark_steel,
            name=girder_name,
        )
        leaf.visual(
            Box((2.25, 0.06, 0.08)),
            origin=Origin(xyz=(1.53, y, 0.17)),
            material=dark_steel,
            name=f"guard_rail_{side}",
        )
        for post_x in (0.45, 1.10, 1.75, 2.40):
            leaf.visual(
                Box((0.055, 0.075, 0.32)),
                origin=Origin(xyz=(post_x, y, 0.075)),
                material=dark_steel,
                name=f"rail_post_{side}_{post_x:.2f}",
            )
        leaf.visual(
            Box((0.16, 0.16, 0.18)),
            origin=Origin(xyz=(0.055, y, -0.055)),
            material=dark_steel,
            name=f"hinge_lug_{side}",
        )

    # The visible hinge tube and side trunnion stubs are part of the moving leaf.
    leaf.visual(
        Cylinder(radius=0.070, length=2.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_tube",
    )
    for side, y, stub_name in (
        (0, -1.1025, "bearing_stub_0"),
        (1, 1.1025, "bearing_stub_1"),
    ):
        leaf.visual(
            Cylinder(radius=0.072, length=0.145),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_metal,
            name=stub_name,
        )

    model.articulation(
        "shore_bearing",
        ArticulationType.REVOLUTE,
        parent=shore,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.35, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_bearing")

    limits = hinge.motion_limits
    ctx.check(
        "single revolute drawbridge leaf",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 1.0,
        details=f"articulations={object_model.articulations}, limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            shore,
            axis="x",
            positive_elem="deck_panel",
            negative_elem="approach_deck",
            min_gap=0.03,
            max_gap=0.16,
            name="closed leaf starts just beyond shore approach",
        )
        ctx.expect_overlap(
            leaf,
            shore,
            axes="yz",
            elem_a="deck_panel",
            elem_b="approach_deck",
            min_overlap=0.12,
            name="closed deck aligns with approach deck height and width",
        )
        ctx.expect_gap(
            shore,
            leaf,
            axis="y",
            positive_elem="side_cheek_1",
            negative_elem="side_girder_1",
            min_gap=0.03,
            max_gap=0.14,
            name="positive side cheek clears leaf girder",
        )
        ctx.expect_gap(
            leaf,
            shore,
            axis="y",
            positive_elem="side_girder_0",
            negative_elem="side_cheek_0",
            min_gap=0.03,
            max_gap=0.14,
            name="negative side cheek clears leaf girder",
        )
        ctx.expect_contact(
            leaf,
            shore,
            elem_a="bearing_stub_1",
            elem_b="inner_bearing_liner_1",
            contact_tol=0.002,
            name="positive trunnion seats in bearing liner",
        )
        ctx.expect_contact(
            leaf,
            shore,
            elem_a="bearing_stub_0",
            elem_b="inner_bearing_liner_0",
            contact_tol=0.002,
            name="negative trunnion seats in bearing liner",
        )

    closed_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({hinge: limits.upper if limits is not None and limits.upper is not None else 1.1}):
        raised_aabb = ctx.part_world_aabb(leaf)
        ctx.expect_gap(
            leaf,
            shore,
            axis="z",
            positive_elem="deck_panel",
            negative_elem="water_plane",
            min_gap=0.35,
            name="raised leaf remains above canal water",
        )

    ctx.check(
        "free end rises when drawbridge opens",
        closed_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > closed_aabb[1][2] + 1.0,
        details=f"closed_aabb={closed_aabb}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
