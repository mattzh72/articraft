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
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_cart")

    deck = model.part("deck")
    # Deck panel
    deck.visual(
        Box((1.0, 0.6, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        name="deck_panel",
    )
    # Handle posts
    deck.visual(
        Cylinder(radius=0.015, length=0.775),
        origin=Origin(xyz=(-0.45, 0.25, 0.6125)),
        name="handle_post_left",
    )
    deck.visual(
        Cylinder(radius=0.015, length=0.775),
        origin=Origin(xyz=(-0.45, -0.25, 0.6125)),
        name="handle_post_right",
    )
    # Handle crossbar
    deck.visual(
        Cylinder(radius=0.015, length=0.53),
        origin=Origin(xyz=(-0.45, 0.0, 1.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="handle_crossbar",
    )

    # Casters
    caster_positions = {
        "front_left": (0.4, 0.25),
        "front_right": (0.4, -0.25),
        "rear_left": (-0.4, 0.25),
        "rear_right": (-0.4, -0.25),
    }

    for name, (x, y) in caster_positions.items():
        fork = model.part(f"caster_fork_{name}")
        # Top plate of the fork
        fork.visual(
            Cylinder(radius=0.03, length=0.01),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            name=f"fork_top_{name}",
        )
        # Fork legs
        fork.visual(
            Box((0.06, 0.005, 0.12)),
            origin=Origin(xyz=(-0.02, 0.02, -0.065)),
            name=f"fork_leg_left_{name}",
        )
        fork.visual(
            Box((0.06, 0.005, 0.12)),
            origin=Origin(xyz=(-0.02, -0.02, -0.065)),
            name=f"fork_leg_right_{name}",
        )

        model.articulation(
            f"swivel_{name}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, 0.175)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=5.0),
        )

        wheel = model.part(f"caster_wheel_{name}")
        wheel.visual(
            Cylinder(radius=0.05, length=0.03),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            name=f"wheel_cyl_{name}",
        )
        wheel.visual(
            Cylinder(radius=0.01, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            name=f"wheel_hub_{name}",
        )

        model.articulation(
            f"axle_{name}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-0.02, 0.0, -0.125)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=5.0),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    deck = object_model.get_part("deck")
    
    for name in ["front_left", "front_right", "rear_left", "rear_right"]:
        fork = object_model.get_part(f"caster_fork_{name}")
        wheel = object_model.get_part(f"caster_wheel_{name}")
        
        # Fork is under the deck
        ctx.expect_gap(deck, fork, axis="z", min_gap=0.0, max_gap=0.001)
        
        # Wheel is in the fork
        ctx.expect_within(wheel, fork, axes="y")
        
    return ctx.report()

object_model = build_object_model()