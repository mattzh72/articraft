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
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", rgba=(0.48, 0.49, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.18, 0.20, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.22, 0.34, 0.31, 1.0))
    deck_surface = model.material("gritted_deck", rgba=(0.30, 0.33, 0.32, 1.0))
    bronze = model.material("bronze_bearing", rgba=(0.58, 0.42, 0.20, 1.0))
    water = model.material("canal_water", rgba=(0.05, 0.22, 0.32, 0.82))
    hazard_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.12, 1.0))

    frame = model.part("shore_frame")

    # Low concrete and canal surface keep all fixed structure physically tied together.
    frame.visual(
        Box((5.35, 3.45, 0.16)),
        origin=Origin(xyz=(2.15, 0.0, 0.08)),
        material=concrete,
        name="foundation_slab",
    )
    frame.visual(
        Box((3.65, 2.20, 0.025)),
        origin=Origin(xyz=(2.65, 0.0, 0.1725)),
        material=water,
        name="canal_surface",
    )
    frame.visual(
        Box((0.58, 3.05, 0.56)),
        origin=Origin(xyz=(-0.36, 0.0, 0.44)),
        material=concrete,
        name="shore_abutment",
    )

    # Surrounding fixed frame: side beams, support posts, and a far cross tie.
    frame.visual(
        Box((4.90, 0.22, 0.32)),
        origin=Origin(xyz=(2.30, 1.31, 0.86)),
        material=dark_steel,
        name="side_beam_0",
    )
    frame.visual(
        Box((4.90, 0.22, 0.32)),
        origin=Origin(xyz=(2.30, -1.31, 0.86)),
        material=dark_steel,
        name="side_beam_1",
    )
    for y_pos, beam_name in ((1.31, "side_beam_0"), (-1.31, "side_beam_1")):
        for x_pos in (-0.05, 2.20, 4.45):
            frame.visual(
                Box((0.26, 0.22, 0.58)),
                origin=Origin(xyz=(x_pos, y_pos, 0.45)),
                material=dark_steel,
                name=f"{beam_name}_post_{x_pos:g}",
            )

    frame.visual(
        Box((0.24, 3.05, 0.28)),
        origin=Origin(xyz=(4.72, 0.0, 0.84)),
        material=dark_steel,
        name="far_cross_tie",
    )

    # Heavy side bearing housings flank the rotating trunnion without becoming
    # another moving part.  The circular faces are vertical, with their axes
    # along bridge width.
    frame.visual(
        Box((0.58, 0.28, 0.82)),
        origin=Origin(xyz=(0.0, 1.42, 0.61)),
        material=dark_steel,
        name="bearing_0_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.32, length=0.18),
        origin=Origin(xyz=(0.0, 1.25, 1.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_0",
    )
    frame.visual(
        Cylinder(radius=0.20, length=0.205),
        origin=Origin(xyz=(0.0, 1.25, 1.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="bearing_0_bush",
    )
    frame.visual(
        Box((0.58, 0.28, 0.82)),
        origin=Origin(xyz=(0.0, -1.42, 0.61)),
        material=dark_steel,
        name="bearing_1_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.32, length=0.18),
        origin=Origin(xyz=(0.0, -1.25, 1.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_1",
    )
    frame.visual(
        Cylinder(radius=0.20, length=0.205),
        origin=Origin(xyz=(0.0, -1.25, 1.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="bearing_1_bush",
    )

    leaf = model.part("bridge_leaf")

    # The leaf part frame is exactly on the shore-side hinge axis.  Its deck and
    # stiffeners extend along local +X from that trunnion line.
    leaf.visual(
        Cylinder(radius=0.155, length=2.32),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    leaf.visual(
        Box((4.35, 2.05, 0.16)),
        origin=Origin(xyz=(2.26, 0.0, -0.06)),
        material=deck_surface,
        name="deck_plate",
    )
    for y_pos, name in ((0.94, "edge_girder_0"), (-0.94, "edge_girder_1")):
        leaf.visual(
            Box((4.25, 0.12, 0.26)),
            origin=Origin(xyz=(2.33, y_pos, 0.08)),
            material=painted_steel,
            name=name,
        )
    for x_pos in (0.62, 1.62, 2.62, 3.62):
        leaf.visual(
            Box((0.11, 1.96, 0.08)),
            origin=Origin(xyz=(x_pos, 0.0, 0.055)),
            material=painted_steel,
            name=f"cross_stiffener_{x_pos:g}",
        )
    leaf.visual(
        Box((0.16, 2.05, 0.18)),
        origin=Origin(xyz=(4.51, 0.0, -0.035)),
        material=hazard_yellow,
        name="toe_plate",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.35, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    for bush_name in ("bearing_0_bush", "bearing_1_bush"):
        ctx.allow_overlap(
            leaf,
            frame,
            elem_a="hinge_barrel",
            elem_b=bush_name,
            reason="The rotating trunnion is intentionally seated a short distance inside the bronze bearing bush.",
        )

    ctx.check(
        "single revolute bridge leaf",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.parent == "shore_frame"
        and hinge.child == "bridge_leaf",
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "horizontal shore-side hinge axis",
        abs(hinge.axis[0]) < 1e-9 and abs(abs(hinge.axis[1]) - 1.0) < 1e-9 and abs(hinge.axis[2]) < 1e-9,
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        deck_aabb = ctx.part_element_world_aabb(leaf, elem="deck_plate")
        beam_0_aabb = ctx.part_element_world_aabb(frame, elem="side_beam_0")
        beam_1_aabb = ctx.part_element_world_aabb(frame, elem="side_beam_1")
        if deck_aabb and beam_0_aabb and beam_1_aabb:
            deck_min, deck_max = deck_aabb
            beam_0_min, beam_0_max = beam_0_aabb
            beam_1_min, beam_1_max = beam_1_aabb
            pos_inner = beam_0_min[1]
            neg_inner = beam_1_max[1]
            pos_clearance = pos_inner - deck_max[1]
            neg_clearance = deck_min[1] - neg_inner
            deck_center_y = 0.5 * (deck_min[1] + deck_max[1])
            ctx.check(
                "leaf centered between side beams",
                abs(deck_center_y) <= 0.01
                and pos_clearance > 0.12
                and neg_clearance > 0.12
                and abs(pos_clearance - neg_clearance) <= 0.02,
                details=(
                    f"center_y={deck_center_y}, pos_clearance={pos_clearance}, "
                    f"neg_clearance={neg_clearance}"
                ),
            )
        else:
            ctx.fail("leaf centered between side beams", "missing deck or side beam AABB")

        ctx.expect_contact(
            frame,
            leaf,
            elem_a="bearing_0",
            elem_b="hinge_barrel",
            name="positive bearing flanks trunnion",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="hinge_barrel",
            elem_b="bearing_1",
            name="negative bearing flanks trunnion",
        )
        for bush_name in ("bearing_0_bush", "bearing_1_bush"):
            ctx.expect_within(
                leaf,
                frame,
                axes="xz",
                inner_elem="hinge_barrel",
                outer_elem=bush_name,
                margin=0.0,
                name=f"trunnion centered in {bush_name}",
            )
            ctx.expect_overlap(
                leaf,
                frame,
                axes="y",
                elem_a="hinge_barrel",
                elem_b=bush_name,
                min_overlap=0.006,
                name=f"trunnion retained in {bush_name}",
            )

    rest_toe = ctx.part_element_world_aabb(leaf, elem="toe_plate")
    with ctx.pose({hinge: 1.25}):
        opened_toe = ctx.part_element_world_aabb(leaf, elem="toe_plate")
        ctx.expect_gap(
            frame,
            leaf,
            axis="y",
            positive_elem="side_beam_0",
            negative_elem="deck_plate",
            min_gap=0.12,
            name="open leaf remains clear of side frame",
        )

    if rest_toe and opened_toe:
        ctx.check(
            "positive hinge angle raises free end",
            opened_toe[0][2] > rest_toe[1][2] + 2.0,
            details=f"rest_toe={rest_toe}, opened_toe={opened_toe}",
        )
    else:
        ctx.fail("positive hinge angle raises free end", "missing toe plate AABB")

    return ctx.report()


object_model = build_object_model()
