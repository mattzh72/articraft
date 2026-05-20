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
    model = ArticulatedObject(name="mechanical_dock_leveler")

    steel = model.material("dark_galvanized_steel", color=(0.24, 0.27, 0.28, 1.0))
    worn_steel = model.material("worn_deck_steel", color=(0.34, 0.37, 0.38, 1.0))
    edge_steel = model.material("blackened_hinge_steel", color=(0.08, 0.09, 0.10, 1.0))
    safety_yellow = model.material("safety_yellow", color=(1.0, 0.74, 0.10, 1.0))

    # Fixed dock pit frame: a connected welded steel frame that the rear deck
    # hinge is mounted to.  Dimensions are full-scale for a small loading dock.
    frame = model.part("pit_frame")
    frame.visual(
        Box((0.18, 2.28, 0.09)),
        origin=Origin(xyz=(-0.08, 0.0, 0.045)),
        material=steel,
        name="rear_crossmember",
    )
    for y in (-1.08, 1.08):
        frame.visual(
            Box((2.70, 0.12, 0.10)),
            origin=Origin(xyz=(1.25, y, 0.050)),
            material=steel,
            name=f"side_rail_{'negative' if y < 0 else 'positive'}",
        )
        frame.visual(
            Box((0.08, 0.18, 0.18)),
            origin=Origin(xyz=(2.55, -1.20 if y < 0 else 1.20, 0.090)),
            material=safety_yellow,
            name=f"front_guard_{'negative' if y < 0 else 'positive'}",
        )

    # Alternating rear hinge knuckles on the stationary frame.
    rear_fixed_segments = [(-0.78, 0.40), (0.0, 0.40), (0.78, 0.40)]
    for index, (y, length) in enumerate(rear_fixed_segments):
        frame.visual(
            Box((0.055, length, 0.060)),
            origin=Origin(xyz=(-0.025, y, 0.118)),
            material=edge_steel,
            name=f"rear_hinge_lug_{index}",
        )
        frame.visual(
            Cylinder(radius=0.055, length=length),
            origin=Origin(xyz=(0.0, y, 0.170), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=edge_steel,
            name=f"rear_frame_knuckle_{index}",
        )
    frame.visual(
        Cylinder(radius=0.026, length=2.06),
        origin=Origin(xyz=(0.0, 0.0, 0.170), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=edge_steel,
        name="rear_hinge_pin",
    )

    # Moving steel deck.  Its frame is the rear hinge pin line, so the deck
    # plate extends forward along +X and positive joint motion raises its nose.
    deck = model.part("deck_plate")
    deck.visual(
        Box((2.30, 2.00, 0.060)),
        origin=Origin(xyz=(1.25, 0.0, 0.0)),
        material=worn_steel,
        name="deck_panel",
    )
    for y in (-0.68, 0.0, 0.68):
        deck.visual(
            Box((2.05, 0.070, 0.115)),
            origin=Origin(xyz=(1.25, y, -0.087)),
            material=edge_steel,
            name=f"underside_rib_{len(deck.visuals)}",
        )

    for index, x in enumerate((0.42, 0.72, 1.02, 1.32, 1.62, 1.92, 2.22)):
        deck.visual(
            Box((0.035, 1.72, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=edge_steel,
            name=f"deck_tread_{index}",
        )

    # Deck-side rear knuckles fill the gaps between the fixed knuckles.
    rear_deck_segments = [(-0.38, 0.28), (0.38, 0.28)]
    for index, (y, length) in enumerate(rear_deck_segments):
        deck.visual(
            Box((0.055, length, 0.020)),
            origin=Origin(xyz=(0.0775, y, 0.0)),
            material=edge_steel,
            name=f"rear_hinge_leaf_{index}",
        )
        deck.visual(
            Cylinder(radius=0.055, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=edge_steel,
            name=f"rear_deck_knuckle_{index}",
        )

    # Deck-side front knuckles for the folding lip hinge.
    front_deck_segments = [(-0.78, 0.40), (0.0, 0.40), (0.78, 0.40)]
    for index, (y, length) in enumerate(front_deck_segments):
        deck.visual(
            Box((0.060, length, 0.020)),
            origin=Origin(xyz=(2.415, y, 0.0)),
            material=edge_steel,
            name=f"front_hinge_leaf_{index}",
        )
        deck.visual(
            Cylinder(radius=0.055, length=length),
            origin=Origin(xyz=(2.500, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=edge_steel,
            name=f"front_deck_knuckle_{index}",
        )
    deck.visual(
        Cylinder(radius=0.026, length=2.06),
        origin=Origin(xyz=(2.500, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=edge_steel,
        name="front_hinge_pin",
    )

    # Folding dock lip.  Its local frame is the front hinge pin line; at q=0 it
    # projects forward as a bridge plate, and positive joint motion folds it
    # downward toward the truck bed / stored position.
    lip = model.part("folding_lip")
    lip.visual(
        Box((0.72, 1.86, 0.050)),
        origin=Origin(xyz=(0.440, 0.0, 0.005)),
        material=worn_steel,
        name="lip_panel",
    )
    for index, x in enumerate((0.20, 0.42, 0.64)):
        lip.visual(
            Box((0.030, 1.58, 0.007)),
            origin=Origin(xyz=(x, 0.0, 0.033)),
            material=edge_steel,
            name=f"lip_tread_{index}",
        )

    front_lip_segments = [(-0.38, 0.28), (0.38, 0.28)]
    for index, (y, length) in enumerate(front_lip_segments):
        lip.visual(
            Box((0.055, length, 0.018)),
            origin=Origin(xyz=(0.0775, y, 0.0)),
            material=edge_steel,
            name=f"lip_hinge_leaf_{index}",
        )
        lip.visual(
            Cylinder(radius=0.055, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=edge_steel,
            name=f"lip_knuckle_{index}",
        )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.35, lower=0.0, upper=0.35),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(2.500, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.80, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("pit_frame")
    deck = object_model.get_part("deck_plate")
    lip = object_model.get_part("folding_lip")
    deck_hinge = object_model.get_articulation("frame_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    ctx.check(
        "deck uses rear revolute hinge",
        deck_hinge.articulation_type == ArticulationType.REVOLUTE
        and deck_hinge.axis == (0.0, -1.0, 0.0)
        and deck_hinge.motion_limits is not None
        and deck_hinge.motion_limits.upper >= 0.30,
        details=f"type={deck_hinge.articulation_type}, axis={deck_hinge.axis}, limits={deck_hinge.motion_limits}",
    )
    ctx.check(
        "lip uses front downward revolute hinge",
        lip_hinge.articulation_type == ArticulationType.REVOLUTE
        and lip_hinge.axis == (0.0, 1.0, 0.0)
        and lip_hinge.motion_limits is not None
        and lip_hinge.motion_limits.upper >= 1.0,
        details=f"type={lip_hinge.articulation_type}, axis={lip_hinge.axis}, limits={lip_hinge.motion_limits}",
    )

    # The hinge pins are intentionally captured through the rotating knuckles.
    # These local, cylindrical interpenetrations are the physical support path,
    # not broad plate collisions.
    for index in (0, 1):
        ctx.allow_overlap(
            frame,
            deck,
            elem_a="rear_hinge_pin",
            elem_b=f"rear_deck_knuckle_{index}",
            reason="The rear steel hinge pin is captured inside the rotating deck knuckle.",
        )
        ctx.expect_within(
            frame,
            deck,
            axes="xz",
            inner_elem="rear_hinge_pin",
            outer_elem=f"rear_deck_knuckle_{index}",
            margin=0.0,
            name=f"rear pin sits inside deck knuckle {index}",
        )
        ctx.expect_overlap(
            frame,
            deck,
            axes="y",
            elem_a="rear_hinge_pin",
            elem_b=f"rear_deck_knuckle_{index}",
            min_overlap=0.24,
            name=f"rear pin spans deck knuckle {index}",
        )

        ctx.allow_overlap(
            deck,
            lip,
            elem_a="front_hinge_pin",
            elem_b=f"lip_knuckle_{index}",
            reason="The front lip hinge pin is captured inside the folding lip knuckle.",
        )
        ctx.expect_within(
            deck,
            lip,
            axes="xz",
            inner_elem="front_hinge_pin",
            outer_elem=f"lip_knuckle_{index}",
            margin=0.0,
            name=f"front pin sits inside lip knuckle {index}",
        )
        ctx.expect_overlap(
            deck,
            lip,
            axes="y",
            elem_a="front_hinge_pin",
            elem_b=f"lip_knuckle_{index}",
            min_overlap=0.24,
            name=f"front pin spans lip knuckle {index}",
        )

    ctx.expect_within(
        deck,
        frame,
        axes="y",
        inner_elem="deck_panel",
        outer_elem="rear_crossmember",
        margin=0.0,
        name="deck width sits inside dock frame",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        positive_elem="lip_panel",
        negative_elem="deck_panel",
        min_gap=0.16,
        max_gap=0.20,
        name="lip plate projects beyond deck front hinge",
    )

    rest_deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_panel")
    with ctx.pose({deck_hinge: deck_hinge.motion_limits.upper}):
        raised_deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_panel")
    rest_deck_center_z = (rest_deck_aabb[0][2] + rest_deck_aabb[1][2]) / 2.0
    raised_deck_center_z = (raised_deck_aabb[0][2] + raised_deck_aabb[1][2]) / 2.0
    ctx.check(
        "deck hinge raises the front deck plate",
        raised_deck_center_z > rest_deck_center_z + 0.25,
        details=f"rest_z={rest_deck_center_z}, raised_z={raised_deck_center_z}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_panel")
    with ctx.pose({lip_hinge: 1.10}):
        folded_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_panel")
    rest_lip_center_z = (rest_lip_aabb[0][2] + rest_lip_aabb[1][2]) / 2.0
    folded_lip_center_z = (folded_lip_aabb[0][2] + folded_lip_aabb[1][2]) / 2.0
    ctx.check(
        "lip hinge pivots the folding lip downward",
        folded_lip_center_z < rest_lip_center_z - 0.25,
        details=f"rest_z={rest_lip_center_z}, folded_z={folded_lip_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
