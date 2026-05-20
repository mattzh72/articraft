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
    model = ArticulatedObject(name="drawbridge")

    # -------------------------------------------------------------------------
    # Shore Frame (Fixed)
    # -------------------------------------------------------------------------
    shore = model.part("shore_frame")

    # Left bank (supports the hinge)
    shore.visual(
        Box((4.0, 6.0, 1.0)),
        origin=Origin(xyz=(-2.0, 0.0, 0.5)),
        name="left_bank",
    )

    # Right bank (the other side)
    shore.visual(
        Box((4.0, 6.0, 1.0)),
        origin=Origin(xyz=(6.4, 0.0, 0.5)),
        name="right_bank",
    )

    # Right ledge (where the bridge rests when closed)
    shore.visual(
        Box((0.4, 6.0, 0.7)),
        origin=Origin(xyz=(4.2, 0.0, 0.35)),
        name="right_ledge",
    )

    # Heavy side bearings for the hinge
    shore.visual(
        Box((0.8, 0.6, 0.4)),
        origin=Origin(xyz=(-0.2, 2.3, 1.0)),
        name="bearing_left",
    )
    shore.visual(
        Box((0.8, 0.6, 0.4)),
        origin=Origin(xyz=(-0.2, -2.3, 1.0)),
        name="bearing_right",
    )

    # Sea floor connecting the banks
    shore.visual(
        Box((12.4, 6.0, 0.2)),
        origin=Origin(xyz=(2.2, 0.0, -0.1)),
        name="sea_floor",
    )

    # -------------------------------------------------------------------------
    # Bridge Leaf
    # -------------------------------------------------------------------------
    leaf = model.part("bridge_leaf")

    # Deck
    leaf.visual(
        Box((4.36, 4.0, 0.1)),
        origin=Origin(xyz=(2.2, 0.0, -0.05)),
        name="deck",
    )

    # Under-deck support beams
    leaf.visual(
        Box((4.36, 0.2, 0.3)),
        origin=Origin(xyz=(2.2, 1.9, -0.15)),
        name="beam_left",
    )
    leaf.visual(
        Box((4.36, 0.2, 0.3)),
        origin=Origin(xyz=(2.2, -1.9, -0.15)),
        name="beam_right",
    )

    # Railings
    leaf.visual(
        Box((4.36, 0.1, 0.4)),
        origin=Origin(xyz=(2.2, 1.95, 0.2)),
        name="railing_left",
    )
    leaf.visual(
        Box((4.36, 0.1, 0.4)),
        origin=Origin(xyz=(2.2, -1.95, 0.2)),
        name="railing_right",
    )

    # Hinge shaft
    # Rotated to align with the Y-axis
    leaf.visual(
        Cylinder(radius=0.1, length=5.2),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        name="hinge_shaft",
    )

    # -------------------------------------------------------------------------
    # Articulation
    # -------------------------------------------------------------------------
    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=shore,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=0.5, lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shore = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    # Allowances for the captured hinge shaft
    ctx.allow_overlap(
        leaf,
        shore,
        elem_a="hinge_shaft",
        elem_b="left_bank",
        reason="Hinge shaft is partially embedded in the top edge of the left bank.",
    )
    ctx.allow_overlap(
        leaf,
        shore,
        elem_a="hinge_shaft",
        elem_b="bearing_left",
        reason="Hinge shaft is captured by the left bearing.",
    )
    ctx.allow_overlap(
        leaf,
        shore,
        elem_a="hinge_shaft",
        elem_b="bearing_right",
        reason="Hinge shaft is captured by the right bearing.",
    )

    # Rest pose checks
    with ctx.pose({hinge: 0.0}):
        # Beams should rest on the right ledge
        ctx.expect_gap(
            leaf,
            shore,
            axis="z",
            max_penetration=0.001,
            positive_elem="beam_left",
            negative_elem="right_ledge",
            name="Left beam rests on right ledge",
        )
        ctx.expect_gap(
            leaf,
            shore,
            axis="z",
            max_penetration=0.001,
            positive_elem="beam_right",
            negative_elem="right_ledge",
            name="Right beam rests on right ledge",
        )

        # Deck should have a small gap with the banks
        ctx.expect_gap(
            leaf,
            shore,
            axis="x",
            min_gap=0.01,
            positive_elem="deck",
            negative_elem="left_bank",
            name="Clearance between deck and left bank",
        )
        ctx.expect_gap(
            shore,
            leaf,
            axis="x",
            min_gap=0.01,
            positive_elem="right_bank",
            negative_elem="deck",
            name="Clearance between right bank and deck",
        )

    # Open pose check
    with ctx.pose({hinge: 1.5}):
        leaf_aabb = ctx.part_world_aabb(leaf)
        if leaf_aabb:
            ctx.check(
                "Leaf raises up",
                leaf_aabb[1][2] > 4.0,
                details=f"Leaf max Z: {leaf_aabb[1][2]}",
            )

    return ctx.report()


object_model = build_object_model()
