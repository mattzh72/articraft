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
    model = ArticulatedObject(name="dock_loading_ramp")

    # Base / Dock Frame
    base = model.part("base")
    base.visual(Box((2.2, 0.4, 0.2)), origin=Origin(xyz=(0.0, -0.25, 1.1)), name="base_frame")
    
    base.visual(Cylinder(radius=0.04, length=0.2), origin=Origin(xyz=(-0.8, 0.0, 1.2), rpy=(0.0, 1.5708, 0.0)), name="base_knuckle_L")
    base.visual(Box((0.2, 0.05, 0.08)), origin=Origin(xyz=(-0.8, -0.025, 1.2)), name="base_support_L")
    
    base.visual(Cylinder(radius=0.04, length=0.2), origin=Origin(xyz=(0.0, 0.0, 1.2), rpy=(0.0, 1.5708, 0.0)), name="base_knuckle_C")
    base.visual(Box((0.2, 0.05, 0.08)), origin=Origin(xyz=(0.0, -0.025, 1.2)), name="base_support_C")
    
    base.visual(Cylinder(radius=0.04, length=0.2), origin=Origin(xyz=(0.8, 0.0, 1.2), rpy=(0.0, 1.5708, 0.0)), name="base_knuckle_R")
    base.visual(Box((0.2, 0.05, 0.08)), origin=Origin(xyz=(0.8, -0.025, 1.2)), name="base_support_R")
    
    base.visual(Cylinder(radius=0.029, length=1.8), origin=Origin(xyz=(0.0, 0.0, 1.2), rpy=(0.0, 1.5708, 0.0)), name="rear_hinge_pin")

    # Deck
    deck = model.part("deck")
    # Main plate (Y from 0.05 to 2.45)
    deck.visual(Box((2.0, 2.4, 0.05)), origin=Origin(xyz=(0.0, 1.25, 0.025)), name="deck_plate")
    # Side rails
    deck.visual(Box((0.1, 2.4, 0.15)), origin=Origin(xyz=(0.95, 1.25, -0.075)), name="deck_rail_L")
    deck.visual(Box((0.1, 2.4, 0.15)), origin=Origin(xyz=(-0.95, 1.25, -0.075)), name="deck_rail_R")
    
    # Rear knuckles (Y=0)
    deck.visual(Cylinder(radius=0.04, length=0.5), origin=Origin(xyz=(-0.4, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="deck_rear_knuckle_L")
    deck.visual(Box((0.5, 0.05, 0.08)), origin=Origin(xyz=(-0.4, 0.025, 0.0)), name="deck_rear_support_L")
    deck.visual(Cylinder(radius=0.04, length=0.5), origin=Origin(xyz=(0.4, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="deck_rear_knuckle_R")
    deck.visual(Box((0.5, 0.05, 0.08)), origin=Origin(xyz=(0.4, 0.025, 0.0)), name="deck_rear_support_R")
    
    # Front knuckles (Y=2.5)
    deck.visual(Cylinder(radius=0.04, length=0.2), origin=Origin(xyz=(-0.8, 2.5, 0.0), rpy=(0.0, 1.5708, 0.0)), name="deck_front_knuckle_L")
    deck.visual(Box((0.2, 0.05, 0.08)), origin=Origin(xyz=(-0.8, 2.475, 0.0)), name="deck_front_support_L")
    deck.visual(Cylinder(radius=0.04, length=0.2), origin=Origin(xyz=(0.0, 2.5, 0.0), rpy=(0.0, 1.5708, 0.0)), name="deck_front_knuckle_C")
    deck.visual(Box((0.2, 0.05, 0.08)), origin=Origin(xyz=(0.0, 2.475, 0.0)), name="deck_front_support_C")
    deck.visual(Cylinder(radius=0.04, length=0.2), origin=Origin(xyz=(0.8, 2.5, 0.0), rpy=(0.0, 1.5708, 0.0)), name="deck_front_knuckle_R")
    deck.visual(Box((0.2, 0.05, 0.08)), origin=Origin(xyz=(0.8, 2.475, 0.0)), name="deck_front_support_R")
    
    deck.visual(Cylinder(radius=0.029, length=1.8), origin=Origin(xyz=(0.0, 2.5, 0.0), rpy=(0.0, 1.5708, 0.0)), name="front_hinge_pin")

    # Leg brackets under deck
    # Front leg brackets (Y=2.2)
    deck.visual(Box((0.05, 0.2, 0.2)), origin=Origin(xyz=(0.85, 2.2, -0.1)), name="deck_front_leg_bracket_L")
    deck.visual(Box((0.05, 0.2, 0.2)), origin=Origin(xyz=(-0.85, 2.2, -0.1)), name="deck_front_leg_bracket_R")
    # Mid leg brackets (Y=1.2)
    deck.visual(Box((0.05, 0.2, 0.2)), origin=Origin(xyz=(0.85, 1.2, -0.1)), name="deck_mid_leg_bracket_L")
    deck.visual(Box((0.05, 0.2, 0.2)), origin=Origin(xyz=(-0.85, 1.2, -0.1)), name="deck_mid_leg_bracket_R")

    # Lip
    lip = model.part("lip")
    # Lip plate (Y from 0.05 to 0.45)
    lip.visual(Box((2.0, 0.4, 0.04)), origin=Origin(xyz=(0.0, 0.25, 0.02)), name="lip_plate")
    # Lip bevel
    lip.visual(Box((2.0, 0.05, 0.01)), origin=Origin(xyz=(0.0, 0.475, 0.005)), name="lip_bevel")
    
    # Lip knuckles (Y=0 in lip frame)
    lip.visual(Cylinder(radius=0.04, length=0.5), origin=Origin(xyz=(-0.4, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="lip_knuckle_L")
    lip.visual(Box((0.5, 0.05, 0.08)), origin=Origin(xyz=(-0.4, 0.025, 0.0)), name="lip_support_L")
    lip.visual(Cylinder(radius=0.04, length=0.5), origin=Origin(xyz=(0.4, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="lip_knuckle_R")
    lip.visual(Box((0.5, 0.05, 0.08)), origin=Origin(xyz=(0.4, 0.025, 0.0)), name="lip_support_R")

    # Front Legs
    front_legs = model.part("front_legs")
    front_legs.visual(Box((0.1, 0.1, 1.05)), origin=Origin(xyz=(0.75, 0.0, -0.525)), name="front_leg_L")
    front_legs.visual(Box((0.1, 0.1, 1.05)), origin=Origin(xyz=(-0.75, 0.0, -0.525)), name="front_leg_R")
    front_legs.visual(Box((1.4, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, -0.7)), name="front_crossbar")
    front_legs.visual(Cylinder(radius=0.03, length=1.8), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="front_pin")

    # Mid Legs
    mid_legs = model.part("mid_legs")
    mid_legs.visual(Box((0.1, 0.1, 1.05)), origin=Origin(xyz=(0.75, 0.0, -0.525)), name="mid_leg_L")
    mid_legs.visual(Box((0.1, 0.1, 1.05)), origin=Origin(xyz=(-0.75, 0.0, -0.525)), name="mid_leg_R")
    mid_legs.visual(Box((1.4, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, -0.7)), name="mid_crossbar")
    mid_legs.visual(Cylinder(radius=0.03, length=1.8), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="mid_pin")

    # Articulations
    model.articulation(
        "base_to_deck",
        ArticulationType.REVOLUTE,
        parent=base,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 1.2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.2, upper=0.8),
    )

    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.0, 2.5, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.0, lower=-1.5, upper=0.2),
    )

    model.articulation(
        "deck_to_front_legs",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_legs,
        origin=Origin(xyz=(0.0, 2.2, -0.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-1.57, upper=0.0),
    )

    model.articulation(
        "deck_to_mid_legs",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=mid_legs,
        origin=Origin(xyz=(0.0, 1.2, -0.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-1.57, upper=0.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allowances for captured hinge pins inside brackets
    ctx.allow_overlap(
        "base", "deck",
        elem_a="rear_hinge_pin", elem_b="deck_rear_knuckle_L",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "base", "deck",
        elem_a="rear_hinge_pin", elem_b="deck_rear_knuckle_R",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "base", "deck",
        elem_a="rear_hinge_pin", elem_b="deck_rear_support_L",
        reason="Captured hinge pin in support block"
    )
    ctx.allow_overlap(
        "base", "deck",
        elem_a="rear_hinge_pin", elem_b="deck_rear_support_R",
        reason="Captured hinge pin in support block"
    )
    ctx.allow_overlap(
        "deck", "lip",
        elem_a="front_hinge_pin", elem_b="lip_knuckle_L",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "deck", "lip",
        elem_a="front_hinge_pin", elem_b="lip_knuckle_R",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "deck", "lip",
        elem_a="front_hinge_pin", elem_b="lip_support_L",
        reason="Captured hinge pin in support block"
    )
    ctx.allow_overlap(
        "deck", "lip",
        elem_a="front_hinge_pin", elem_b="lip_support_R",
        reason="Captured hinge pin in support block"
    )
    ctx.allow_overlap(
        "deck", "front_legs",
        elem_a="deck_front_leg_bracket_L", elem_b="front_pin",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "deck", "front_legs",
        elem_a="deck_front_leg_bracket_R", elem_b="front_pin",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "deck", "mid_legs",
        elem_a="deck_mid_leg_bracket_L", elem_b="mid_pin",
        reason="Captured hinge pin"
    )
    ctx.allow_overlap(
        "deck", "mid_legs",
        elem_a="deck_mid_leg_bracket_R", elem_b="mid_pin",
        reason="Captured hinge pin"
    )

    # Verify legs are deployed downward with proper clearance
    ctx.expect_gap("deck", "front_legs", axis="z", positive_elem="deck_plate", negative_elem="front_leg_L", min_gap=0.1)
    
    # Test poses
    with ctx.pose(base_to_deck=0.5):
        ctx.expect_origin_gap("deck", "base", axis="z", min_gap=0.0)

    with ctx.pose(deck_to_front_legs=-1.5, deck_to_mid_legs=-1.5):
        # When folded, legs should remain behind the lip hinge
        ctx.expect_gap("lip", "front_legs", axis="y", positive_elem="lip_plate", negative_elem="front_leg_L", min_gap=0.0)

    return ctx.report()

object_model = build_object_model()
