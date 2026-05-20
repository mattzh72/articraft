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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_deck_dock_plate")

    galvanized = model.material("galvanized_steel", rgba=(0.46, 0.49, 0.50, 1.0))
    dark_steel = model.material("dark_painted_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    worn_steel = model.material("worn_bearing_steel", rgba=(0.64, 0.62, 0.57, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.74, 0.05, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    ground_frame = model.part("ground_frame")
    ground_frame.visual(
        Box((2.80, 2.20, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="base_plate",
    )
    for y in (-0.92, 0.92):
        ground_frame.visual(
            Box((2.64, 0.080, 0.090)),
            origin=Origin(xyz=(0.0, y, 0.083)),
            material=dark_steel,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    for x in (-1.18, 0.0, 1.18):
        ground_frame.visual(
            Box((0.080, 1.76, 0.065)),
            origin=Origin(xyz=(x, 0.0, 0.071)),
            material=dark_steel,
            name=f"cross_rail_{len([v for v in ground_frame.visuals if v.name and v.name.startswith('cross_rail_')])}",
        )

    # Outboard bearing pedestals carry the central hinge trunnions while staying
    # clear of the deck's flat plate surface.
    for y, pedestal_name, warning_name in (
        (-0.98, "hinge_pedestal_0", "bearing_warning_0"),
        (0.98, "hinge_pedestal_1", "bearing_warning_1"),
    ):
        ground_frame.visual(
            Box((0.24, 0.15, 0.388)),
            origin=Origin(xyz=(0.0, y, 0.234)),
            material=dark_steel,
            name=pedestal_name,
        )
        ground_frame.visual(
            Box((0.34, 0.050, 0.095)),
            origin=Origin(xyz=(0.0, y, 0.382)),
            material=safety_yellow,
            name=warning_name,
        )
    ground_frame.visual(
        Cylinder(radius=0.082, length=0.120),
        origin=Origin(xyz=(0.0, -0.98, 0.510), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_bearing_0",
    )
    ground_frame.visual(
        Cylinder(radius=0.082, length=0.120),
        origin=Origin(xyz=(0.0, 0.98, 0.510), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_bearing_1",
    )

    # Square-tube guide sleeves for the two telescoping support columns.
    sleeve_x = 0.70
    sleeve_centers = (-0.45, 0.45)
    ground_frame.visual(
        Box((0.16, 0.16, 0.018)),
        origin=Origin(xyz=(sleeve_x, -0.45, 0.049)),
        material=dark_steel,
        name="sleeve_foot_0",
    )
    ground_frame.visual(
        Box((0.018, 0.112, 0.272)),
        origin=Origin(xyz=(sleeve_x - 0.047, -0.45, 0.174)),
        material=dark_steel,
        name="sleeve_0_xm",
    )
    ground_frame.visual(
        Box((0.018, 0.112, 0.272)),
        origin=Origin(xyz=(sleeve_x + 0.047, -0.45, 0.174)),
        material=dark_steel,
        name="sleeve_0_xp",
    )
    ground_frame.visual(
        Box((0.076, 0.018, 0.272)),
        origin=Origin(xyz=(sleeve_x, -0.45 - 0.047, 0.174)),
        material=dark_steel,
        name="sleeve_0_ym",
    )
    ground_frame.visual(
        Box((0.076, 0.018, 0.272)),
        origin=Origin(xyz=(sleeve_x, -0.45 + 0.047, 0.174)),
        material=dark_steel,
        name="sleeve_0_yp",
    )
    ground_frame.visual(
        Box((0.16, 0.16, 0.018)),
        origin=Origin(xyz=(sleeve_x, 0.45, 0.049)),
        material=dark_steel,
        name="sleeve_foot_1",
    )
    ground_frame.visual(
        Box((0.018, 0.112, 0.272)),
        origin=Origin(xyz=(sleeve_x - 0.047, 0.45, 0.174)),
        material=dark_steel,
        name="sleeve_1_xm",
    )
    ground_frame.visual(
        Box((0.018, 0.112, 0.272)),
        origin=Origin(xyz=(sleeve_x + 0.047, 0.45, 0.174)),
        material=dark_steel,
        name="sleeve_1_xp",
    )
    ground_frame.visual(
        Box((0.076, 0.018, 0.272)),
        origin=Origin(xyz=(sleeve_x, 0.45 - 0.047, 0.174)),
        material=dark_steel,
        name="sleeve_1_ym",
    )
    ground_frame.visual(
        Box((0.076, 0.018, 0.272)),
        origin=Origin(xyz=(sleeve_x, 0.45 + 0.047, 0.174)),
        material=dark_steel,
        name="sleeve_1_yp",
    )

    for idx, (x, y) in enumerate(
        (
            (-1.18, -0.92),
            (-1.18, 0.92),
            (1.18, -0.92),
            (1.18, 0.92),
        )
    ):
        ground_frame.visual(
            Cylinder(radius=0.035, length=0.014),
            origin=Origin(xyz=(x, y, 0.044)),
            material=worn_steel,
            name=f"anchor_bolt_{idx}",
        )

    deck = model.part("deck")
    deck.visual(
        Box((2.40, 1.50, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="deck_plate",
    )
    for y, idx in ((-0.730, 0), (0.730, 1)):
        deck.visual(
            Box((2.40, 0.040, 0.120)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=safety_yellow,
            name=f"side_curb_{idx}",
        )
    for i, x in enumerate((-0.90, -0.60, -0.30, 0.0, 0.30, 0.60, 0.90)):
        deck.visual(
            Box((0.034, 1.20, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.044)),
            material=worn_steel,
            name=f"tread_bar_{i}",
        )
    deck.visual(
        Cylinder(radius=0.055, length=0.180),
        origin=Origin(xyz=(0.0, -0.830, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="hinge_trunnion_0",
    )
    deck.visual(
        Cylinder(radius=0.055, length=0.180),
        origin=Origin(xyz=(0.0, 0.830, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="hinge_trunnion_1",
    )
    deck.visual(
        Box((0.30, 0.22, 0.014)),
        origin=Origin(xyz=(0.70, -0.45, -0.045)),
        material=worn_steel,
        name="wear_pad_0",
    )
    deck.visual(
        Box((0.30, 0.22, 0.014)),
        origin=Origin(xyz=(0.70, 0.45, -0.045)),
        material=worn_steel,
        name="wear_pad_1",
    )

    for idx, y in enumerate(sleeve_centers):
        leg = model.part(f"leg_{idx}")
        leg.visual(
            Box((0.076, 0.076, 0.320)),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=galvanized,
            name="post",
        )
        leg.visual(
            Sphere(radius=0.035),
            origin=Origin(xyz=(0.0, 0.0, 0.111)),
            material=worn_steel,
            name="swivel_head",
        )
        leg.visual(
            Box((0.082, 0.082, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=rubber_black,
            name="wipe_seal",
        )

        model.articulation(
            f"leg_slide_{idx}",
            ArticulationType.PRISMATIC,
            parent=ground_frame,
            child=leg,
            origin=Origin(xyz=(sleeve_x, y, 0.310)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12000.0, velocity=0.08, lower=0.0, upper=0.180),
        )

    model.articulation(
        "deck_hinge",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20000.0, velocity=0.35, lower=0.0, upper=0.260),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("ground_frame")
    deck = object_model.get_part("deck")
    leg_0 = object_model.get_part("leg_0")
    leg_1 = object_model.get_part("leg_1")
    deck_hinge = object_model.get_articulation("deck_hinge")
    leg_slide_0 = object_model.get_articulation("leg_slide_0")
    leg_slide_1 = object_model.get_articulation("leg_slide_1")

    ctx.expect_within(
        leg_0,
        deck,
        axes="xy",
        inner_elem="swivel_head",
        outer_elem="deck_plate",
        margin=0.0,
        name="leg 0 bears under the platform footprint",
    )
    ctx.expect_within(
        leg_1,
        deck,
        axes="xy",
        inner_elem="swivel_head",
        outer_elem="deck_plate",
        margin=0.0,
        name="leg 1 bears under the platform footprint",
    )
    ctx.expect_gap(
        deck,
        leg_0,
        axis="z",
        positive_elem="wear_pad_0",
        negative_elem="swivel_head",
        min_gap=0.0,
        max_gap=0.006,
        name="leg 0 head is just below its underside wear pad",
    )
    ctx.expect_gap(
        deck,
        leg_1,
        axis="z",
        positive_elem="wear_pad_1",
        negative_elem="swivel_head",
        min_gap=0.0,
        max_gap=0.006,
        name="leg 1 head is just below its underside wear pad",
    )
    ctx.expect_overlap(
        leg_0,
        frame,
        axes="z",
        elem_a="post",
        elem_b="sleeve_0_xp",
        min_overlap=0.18,
        name="leg 0 remains inserted in its guide sleeve",
    )
    ctx.expect_overlap(
        leg_1,
        frame,
        axes="z",
        elem_a="post",
        elem_b="sleeve_1_xp",
        min_overlap=0.18,
        name="leg 1 remains inserted in its guide sleeve",
    )
    ctx.expect_contact(
        deck,
        frame,
        elem_a="hinge_trunnion_0",
        elem_b="hinge_bearing_0",
        name="deck trunnion seats against hinge bearing 0",
    )
    ctx.expect_contact(
        deck,
        frame,
        elem_a="hinge_trunnion_1",
        elem_b="hinge_bearing_1",
        name="deck trunnion seats against hinge bearing 1",
    )
    ctx.expect_contact(
        leg_0,
        frame,
        elem_a="post",
        elem_b="sleeve_0_xp",
        name="leg 0 post is guided by its sleeve",
    )
    ctx.expect_contact(
        leg_1,
        frame,
        elem_a="post",
        elem_b="sleeve_1_xp",
        name="leg 1 post is guided by its sleeve",
    )

    rest_deck_aabb = ctx.part_world_aabb(deck)
    rest_leg_aabb = ctx.part_world_aabb(leg_0)
    with ctx.pose({deck_hinge: 0.22, leg_slide_0: 0.156, leg_slide_1: 0.156}):
        ctx.expect_contact(
            deck,
            leg_0,
            elem_a="wear_pad_0",
            elem_b="swivel_head",
            contact_tol=0.018,
            name="extended leg 0 reaches the tilted deck pad",
        )
        ctx.expect_contact(
            deck,
            leg_1,
            elem_a="wear_pad_1",
            elem_b="swivel_head",
            contact_tol=0.018,
            name="extended leg 1 reaches the tilted deck pad",
        )
        tilted_deck_aabb = ctx.part_world_aabb(deck)
        raised_leg_aabb = ctx.part_world_aabb(leg_0)

    ctx.check(
        "hinge tilt raises the supported edge",
        rest_deck_aabb is not None
        and tilted_deck_aabb is not None
        and tilted_deck_aabb[1][2] > rest_deck_aabb[1][2] + 0.12,
        details=f"rest={rest_deck_aabb}, tilted={tilted_deck_aabb}",
    )
    ctx.check(
        "prismatic support leg extends upward",
        rest_leg_aabb is not None
        and raised_leg_aabb is not None
        and raised_leg_aabb[1][2] > rest_leg_aabb[1][2] + 0.14,
        details=f"rest={rest_leg_aabb}, raised={raised_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
