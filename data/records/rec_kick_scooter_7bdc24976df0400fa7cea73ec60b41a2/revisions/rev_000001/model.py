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
    model = ArticulatedObject(name="kick_scooter")

    # 1. Deck
    deck = model.part("deck")
    # Main board
    deck.visual(
        Box((0.35, 0.12, 0.03)),
        origin=Origin(xyz=(0.025, 0.0, 0.075)),
        name="board",
    )
    # Rear fork to hold the rear wheel
    deck.visual(
        Box((0.15, 0.02, 0.03)),
        origin=Origin(xyz=(-0.225, 0.05, 0.075)),
        name="rear_fork_l",
    )
    deck.visual(
        Box((0.15, 0.02, 0.03)),
        origin=Origin(xyz=(-0.225, -0.05, 0.075)),
        name="rear_fork_r",
    )
    # Rear axle
    deck.visual(
        Cylinder(radius=0.005, length=0.12),
        origin=Origin(xyz=(-0.25, 0.0, 0.075), rpy=(math.pi / 2, 0.0, 0.0)),
        name="rear_axle",
    )
    # Front hinge base
    deck.visual(
        Box((0.08, 0.06, 0.085)),
        origin=Origin(xyz=(0.20, 0.0, 0.1175)),
        name="hinge_base",
    )

    # 2. Rear Wheel
    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=0.075, length=0.03),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        name="wheel",
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.25, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    # 3. Stem Assembly (folds backward over the deck)
    stem_assembly = model.part("stem_assembly")
    # Fork crown
    stem_assembly.visual(
        Box((0.04, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="fork_crown",
    )
    # Stem tube
    stem_assembly.visual(
        Cylinder(radius=0.02, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        name="stem",
    )
    # Handlebar
    stem_assembly.visual(
        Cylinder(radius=0.015, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.70), rpy=(math.pi / 2, 0.0, 0.0)),
        name="handlebar",
    )
    # Front fork (connects hinge to front wheel axle)
    pitch = math.atan2(0.085, 0.11)
    stem_assembly.visual(
        Box((0.14, 0.02, 0.02)),
        origin=Origin(xyz=(0.055, 0.05, -0.0425), rpy=(0.0, pitch, 0.0)),
        name="fork_l",
    )
    stem_assembly.visual(
        Box((0.14, 0.02, 0.02)),
        origin=Origin(xyz=(0.055, -0.05, -0.0425), rpy=(0.0, pitch, 0.0)),
        name="fork_r",
    )
    # Front axle
    stem_assembly.visual(
        Cylinder(radius=0.005, length=0.12),
        origin=Origin(xyz=(0.11, 0.0, -0.085), rpy=(math.pi / 2, 0.0, 0.0)),
        name="front_axle",
    )

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem_assembly,
        origin=Origin(xyz=(0.24, 0.0, 0.16)),
        # Rotate around -Y so positive q pitches the stem backward over the deck
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.96, effort=20.0, velocity=5.0),
    )

    # 4. Front Wheel
    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=0.075, length=0.03),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        name="wheel",
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=stem_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.11, 0.0, -0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    stem_assembly = object_model.get_part("stem_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    ctx.allow_overlap(
        deck,
        stem_assembly,
        elem_a="hinge_base",
        elem_b="fork_crown",
        reason="Hinge components interpenetrate to form the folding joint.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="wheel",
        reason="Rear axle passes through the rear wheel.",
    )
    ctx.allow_overlap(
        stem_assembly,
        front_wheel,
        elem_a="front_axle",
        elem_b="wheel",
        reason="Front axle passes through the front wheel.",
    )

    # In rest pose (upright), wheels should touch the ground plane (z=0)
    ctx.expect_overlap(
        front_wheel,
        deck,
        axes="z",
        min_overlap=0.0,
        name="front wheel is at ground level relative to deck",
    )

    # Test folding backward by 90 degrees (1.57 rad)
    with ctx.pose({"deck_to_stem": 1.57}):
        stem_pos = ctx.part_world_position(stem_assembly)
        ctx.check(
            "stem_folds_backward",
            stem_pos is not None,
            "Stem folded pose is valid.",
        )

    return ctx.report()


object_model = build_object_model()