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

    # Root part: front half of the deck
    front_deck = model.part("front_deck")
    front_deck.visual(
        Box((0.21, 0.15, 0.03)),
        origin=Origin(xyz=(0.095, 0.0, 0.15)),
        name="deck_board",
    )
    # Neck to connect to steering column
    front_deck.visual(
        Box((0.05, 0.04, 0.15)),
        origin=Origin(xyz=(0.225, 0.0, 0.225)),
        name="neck_base",
    )
    front_deck.visual(
        Box((0.15, 0.04, 0.05)),
        origin=Origin(xyz=(0.325, 0.0, 0.275)),
        name="neck_top",
    )

    # Rear half of the deck (folds up)
    rear_deck = model.part("rear_deck")
    rear_deck.visual(
        Box((0.21, 0.15, 0.03)),
        origin=Origin(xyz=(-0.095, 0.0, -0.015)),
        name="deck_board",
    )
    rear_deck.visual(
        Box((0.15, 0.02, 0.03)),
        origin=Origin(xyz=(-0.25, 0.03, -0.04)),
        name="fork_left",
    )
    rear_deck.visual(
        Box((0.15, 0.02, 0.03)),
        origin=Origin(xyz=(-0.25, -0.03, -0.04)),
        name="fork_right",
    )

    model.articulation(
        "deck_fold",
        ArticulationType.REVOLUTE,
        parent=front_deck,
        child=rear_deck,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.14),
    )

    # Rear wheel
    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=0.1, length=0.042),
        origin=Origin(rpy=(1.5708, 0.0, 0.0)),
        name="tire",
    )
    
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=rear_deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.3, 0.0, -0.065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # Rear fender brake
    rear_fender = model.part("rear_fender")
    rear_fender.visual(
        Box((0.15, 0.06, 0.01)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        name="shield",
    )
    # Add a mount to connect fender to deck to prevent floating
    rear_fender.visual(
        Box((0.05, 0.06, 0.05)),
        origin=Origin(xyz=(0.025, 0.0, -0.025)),
        name="fender_mount",
    )

    model.articulation(
        "fender_brake",
        ArticulationType.REVOLUTE,
        parent=rear_deck,
        child=rear_fender,
        origin=Origin(xyz=(-0.2, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.3),
    )

    # Steering column (lower stem)
    lower_stem = model.part("lower_stem")
    lower_stem.visual(
        Cylinder(radius=0.02, length=0.3),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        name="stem_tube",
    )
    lower_stem.visual(
        Box((0.02, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        name="fork_crown",
    )
    lower_stem.visual(
        Box((0.02, 0.02, 0.2)),
        origin=Origin(xyz=(0.0, 0.03, -0.1)),
        name="fork_left",
    )
    lower_stem.visual(
        Box((0.02, 0.02, 0.2)),
        origin=Origin(xyz=(0.0, -0.03, -0.1)),
        name="fork_right",
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=front_deck,
        child=lower_stem,
        origin=Origin(xyz=(0.4, 0.0, 0.3)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.5, upper=1.5),
    )

    # Front wheel
    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=0.1, length=0.042),
        origin=Origin(rpy=(1.5708, 0.0, 0.0)),
        name="tire",
    )

    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=lower_stem,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.2)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # Upper stem (folding handlebar)
    upper_stem = model.part("upper_stem")
    upper_stem.visual(
        Cylinder(radius=0.015, length=0.4),
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        name="pole",
    )
    upper_stem.visual(
        Cylinder(radius=0.015, length=0.4),
        origin=Origin(xyz=(0.0, 0.0, 0.4), rpy=(1.5708, 0.0, 0.0)),
        name="handlebar",
    )

    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=lower_stem,
        child=upper_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.3)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "front_deck",
        "lower_stem",
        elem_a="neck_top",
        elem_b="stem_tube",
        reason="Neck bracket wraps around the steering column tube.",
    )
    
    ctx.allow_overlap(
        "front_deck",
        "lower_stem",
        elem_a="neck_top",
        elem_b="fork_crown",
        reason="Neck bracket wraps around the steering column crown.",
    )
    
    ctx.allow_overlap(
        "front_deck",
        "rear_deck",
        elem_a="deck_board",
        elem_b="deck_board",
        reason="Deck halves overlap slightly at the hinge to ensure connectivity.",
    )
    
    ctx.allow_overlap(
        "rear_deck",
        "rear_fender",
        elem_a="deck_board",
        elem_b="fender_mount",
        reason="Fender mount embeds into the rear deck.",
    )

    with ctx.pose(deck_fold=3.14):
        ctx.expect_overlap(
            "front_deck",
            "rear_deck",
            axes="xy",
            elem_a="deck_board",
            elem_b="deck_board",
            name="folded deck boards overlap in footprint",
        )
        ctx.expect_contact(
            "front_deck",
            "rear_deck",
            elem_a="deck_board",
            elem_b="deck_board",
            name="folded deck boards touch",
        )

    with ctx.pose(stem_fold=1.57):
        # The stem folds backwards, so the handlebar should move backwards (-X) compared to q=0
        handlebar_aabb = ctx.part_element_world_aabb("upper_stem", elem="handlebar")
        if handlebar_aabb is not None:
            handlebar_center_x = (handlebar_aabb[0][0] + handlebar_aabb[1][0]) / 2.0
            ctx.check("stem_folds_backward", handlebar_center_x < 0.3, "handlebar should fold backward")
        else:
            ctx.fail("stem_folds_backward", "could not find handlebar aabb")

    with ctx.pose(fender_brake=0.3):
        ctx.expect_contact(
            "rear_fender",
            "rear_wheel",
            elem_a="shield",
            elem_b="tire",
            name="fender brake contacts the wheel",
        )

    return ctx.report()

object_model = build_object_model()