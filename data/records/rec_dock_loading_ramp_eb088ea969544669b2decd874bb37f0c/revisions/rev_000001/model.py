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
    model = ArticulatedObject(name="vertical_storing_dock_leveler")

    concrete = Material("poured_concrete", rgba=(0.55, 0.55, 0.52, 1.0))
    dark_steel = Material("dark_blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    deck_steel = Material("painted_blue_steel", rgba=(0.05, 0.17, 0.32, 1.0))
    worn_steel = Material("worn_lip_steel", rgba=(0.34, 0.35, 0.33, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(1.0, 0.74, 0.05, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    deck_length = 1.80
    deck_width = 2.10
    deck_thickness = 0.10
    lip_length = 0.55
    lip_width = 2.02
    lip_thickness = 0.060

    frame = model.part("dock_frame")
    # Concrete dock slab and face, with a steel hinge sill at the back edge.
    frame.visual(
        Box((1.05, 2.85, 0.12)),
        origin=Origin(xyz=(-0.64, 0.0, -0.06)),
        material=concrete,
        name="dock_floor",
    )
    frame.visual(
        Box((0.16, 2.85, 0.80)),
        origin=Origin(xyz=(-0.17, 0.0, -0.42)),
        material=concrete,
        name="dock_face",
    )
    frame.visual(
        Box((0.09, 2.55, 0.12)),
        origin=Origin(xyz=(-0.12, 0.0, -0.06)),
        material=dark_steel,
        name="hinge_sill",
    )
    frame.visual(
        Cylinder(radius=0.040, length=2.46),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="back_hinge_pin",
    )
    for idx, y in enumerate((-1.18, 1.18)):
        frame.visual(
            Box((0.16, 0.08, 0.16)),
            origin=Origin(xyz=(-0.04, y, -0.04)),
            material=dark_steel,
            name=f"hinge_bracket_{idx}",
        )
        frame.visual(
            Box((0.08, 0.10, 0.82)),
            origin=Origin(xyz=(-0.08, y, 0.41)),
            material=dark_steel,
            name=f"vertical_stop_{idx}",
        )
        frame.visual(
            Box((0.055, 0.18, 0.55)),
            origin=Origin(xyz=(-0.035, y, 0.36)),
            material=rubber,
            name=f"bumper_pad_{idx}",
        )

    deck = model.part("deck")
    deck.visual(
        Box((deck_thickness, deck_width, deck_length)),
        origin=Origin(xyz=(deck_thickness / 2.0, 0.0, deck_length / 2.0)),
        material=deck_steel,
        name="deck_plate",
    )
    deck.visual(
        Cylinder(radius=0.073, length=2.12),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=deck_steel,
        name="back_hinge_sleeve",
    )
    deck.visual(
        Cylinder(radius=0.046, length=2.08),
        origin=Origin(xyz=(0.0, 0.0, deck_length + 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_hinge_pin",
    )
    # Underside stiffeners are welded to the deck skin and make the heavy dock plate read as structural steel.
    for idx, y in enumerate((-0.72, 0.0, 0.72)):
        deck.visual(
            Box((0.12, 0.070, 1.50)),
            origin=Origin(xyz=(deck_thickness + 0.045, y, 0.88)),
            material=dark_steel,
            name=f"longitudinal_rib_{idx}",
        )
    for idx, z in enumerate((0.42, 1.08, 1.60)):
        deck.visual(
            Box((0.11, 1.86, 0.065)),
            origin=Origin(xyz=(deck_thickness + 0.040, 0.0, z)),
            material=dark_steel,
            name=f"cross_rib_{idx}",
        )
    for idx, z in enumerate((0.32, 0.64, 0.96, 1.28, 1.60)):
        deck.visual(
            Box((0.010, deck_width - 0.20, 0.022)),
            origin=Origin(xyz=(-0.002, 0.0, z)),
            material=dark_steel,
            name=f"tread_bar_{idx}",
        )
    for idx, y in enumerate((-deck_width / 2.0 + 0.06, deck_width / 2.0 - 0.06)):
        deck.visual(
            Box((0.012, 0.055, deck_length - 0.22)),
            origin=Origin(xyz=(-0.003, y, 0.95)),
            material=safety_yellow,
            name=f"edge_stripe_{idx}",
        )

    lip = model.part("lip")
    lip.visual(
        Cylinder(radius=0.052, length=2.04),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="lip_hinge_sleeve",
    )
    lip.visual(
        Box((lip_length, lip_width, lip_thickness)),
        origin=Origin(xyz=(0.065 + lip_length / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.020, lip_width, 0.075)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=worn_steel,
        name="lip_weld_neck",
    )
    lip.visual(
        Box((0.055, lip_width, 0.025)),
        origin=Origin(xyz=(0.065 + lip_length - 0.010, 0.0, -0.018)),
        material=dark_steel,
        name="beveled_tip",
    )
    for idx, x in enumerate((0.20, 0.36, 0.52)):
        lip.visual(
            Box((0.020, lip_width - 0.18, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=dark_steel,
            name=f"lip_tread_{idx}",
        )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.35, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.0, 0.0, deck_length + 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.60, lower=-math.pi / 2.0, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    deck_joint = object_model.get_articulation("frame_to_deck")
    lip_joint = object_model.get_articulation("deck_to_lip")

    ctx.allow_overlap(
        frame,
        deck,
        elem_a="back_hinge_pin",
        elem_b="back_hinge_sleeve",
        reason="The fixed hinge pin is intentionally captured inside the deck hinge sleeve.",
    )
    ctx.allow_overlap(
        frame,
        deck,
        elem_a="back_hinge_pin",
        elem_b="deck_plate",
        reason="The back edge of the welded deck plate locally wraps the hinge pin line.",
    )
    ctx.expect_within(
        frame,
        deck,
        axes="xz",
        inner_elem="back_hinge_pin",
        outer_elem="back_hinge_sleeve",
        margin=0.002,
        name="back hinge pin sits inside deck sleeve",
    )
    ctx.expect_overlap(
        frame,
        deck,
        axes="y",
        elem_a="back_hinge_pin",
        elem_b="back_hinge_sleeve",
        min_overlap=2.0,
        name="back hinge spans most of deck width",
    )
    ctx.expect_gap(
        deck,
        frame,
        axis="z",
        positive_elem="deck_plate",
        negative_elem="back_hinge_pin",
        max_penetration=0.045,
        name="deck plate only locally embeds at back hinge",
    )

    ctx.allow_overlap(
        deck,
        lip,
        elem_a="front_hinge_pin",
        elem_b="lip_hinge_sleeve",
        reason="The lip hinge sleeve rotates around the deck-mounted pin.",
    )
    ctx.allow_overlap(
        deck,
        lip,
        elem_a="deck_plate",
        elem_b="lip_hinge_sleeve",
        reason="The lip sleeve is seated into a shallow notch at the deck nose.",
    )
    ctx.expect_within(
        deck,
        lip,
        axes="xz",
        inner_elem="front_hinge_pin",
        outer_elem="lip_hinge_sleeve",
        margin=0.004,
        name="lip hinge sleeve captures the front pin",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="z",
        positive_elem="lip_hinge_sleeve",
        negative_elem="deck_plate",
        max_penetration=0.015,
        name="lip sleeve has shallow nose seating",
    )

    stored_aabb = ctx.part_world_aabb(deck)
    if stored_aabb is not None:
        stored_min, stored_max = stored_aabb
        stored_height = stored_max[2] - stored_min[2]
        stored_depth = stored_max[0] - stored_min[0]
        ctx.check(
            "deck stores vertically",
            stored_height > 1.65 and stored_depth < 0.35,
            details=f"stored_height={stored_height:.3f}, stored_depth={stored_depth:.3f}",
        )
    else:
        ctx.fail("deck stores vertically", "deck AABB was unavailable")

    with ctx.pose({deck_joint: math.pi / 2.0, lip_joint: 0.0}):
        working_aabb = ctx.part_world_aabb(deck)
        if working_aabb is not None:
            working_min, working_max = working_aabb
            working_length = working_max[0] - working_min[0]
            working_thickness = working_max[2] - working_min[2]
            ctx.check(
                "deck reaches horizontal working position",
                working_length > 1.65 and working_thickness < 0.28,
                details=f"working_length={working_length:.3f}, working_thickness={working_thickness:.3f}",
            )
        else:
            ctx.fail("deck reaches horizontal working position", "deck AABB was unavailable")

    with ctx.pose({deck_joint: math.pi / 2.0, lip_joint: -math.pi / 2.0}):
        deck_aabb = ctx.part_world_aabb(deck)
        lip_aabb = ctx.part_world_aabb(lip)
        if deck_aabb is not None and lip_aabb is not None:
            deck_min, deck_max = deck_aabb
            lip_min, lip_max = lip_aabb
            ctx.check(
                "lip extends beyond deck front",
                lip_max[0] > deck_max[0] + 0.40,
                details=f"deck_front={deck_max[0]:.3f}, lip_front={lip_max[0]:.3f}",
            )
            ctx.check(
                "extended lip stays near deck elevation",
                abs(lip_max[2] - deck_max[2]) < 0.12,
                details=f"deck_top={deck_max[2]:.3f}, lip_top={lip_max[2]:.3f}",
            )
        else:
            ctx.fail("lip extends beyond deck front", "deck or lip AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
