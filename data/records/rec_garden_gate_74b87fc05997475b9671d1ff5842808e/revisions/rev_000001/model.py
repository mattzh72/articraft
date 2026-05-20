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
    model = ArticulatedObject(name="privacy_garden_gate")

    cedar = model.material("warm_cedar", rgba=(0.60, 0.36, 0.18, 1.0))
    dark_cedar = model.material("dark_endgrain", rgba=(0.32, 0.19, 0.09, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.02, 0.018, 0.015, 1.0))
    stone = model.material("stone_sill", rgba=(0.42, 0.40, 0.36, 1.0))

    posts = model.part("posts")
    posts.visual(
        Box((0.10, 0.10, 1.85)),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=cedar,
        name="hinge_post",
    )
    posts.visual(
        Box((0.10, 0.10, 1.85)),
        origin=Origin(xyz=(1.33, 0.0, 0.925)),
        material=cedar,
        name="latch_post",
    )
    posts.visual(
        Box((1.43, 0.16, 0.08)),
        origin=Origin(xyz=(0.665, 0.0, 0.04)),
        material=stone,
        name="ground_sill",
    )
    posts.visual(
        Box((0.14, 0.14, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.8725)),
        material=dark_cedar,
        name="hinge_post_cap",
    )
    posts.visual(
        Box((0.14, 0.14, 0.045)),
        origin=Origin(xyz=(1.33, 0.0, 1.8725)),
        material=dark_cedar,
        name="latch_post_cap",
    )

    # Two exposed hinge pins are fixed to the post with short strap plates.  The
    # leaf rotates about the centerline of these pins, offset forward of the gate
    # plane as a real garden-gate hinge would be.
    for z, name in ((0.55, "lower"), (1.23, "upper")):
        posts.visual(
            Box((0.20, 0.030, 0.16)),
            origin=Origin(xyz=(0.075, -0.063, z)),
            material=black_iron,
            name=f"{name}_post_hinge_plate",
        )
        posts.visual(
            Cylinder(radius=0.018, length=0.18),
            origin=Origin(xyz=(0.16, -0.075, z)),
            material=black_iron,
            name=f"{name}_hinge_pin",
        )

    posts.visual(
        Box((0.075, 0.012, 0.14)),
        origin=Origin(xyz=(1.295, -0.056, 0.92)),
        material=black_iron,
        name="keeper_plate",
    )

    leaf = model.part("leaf")
    wood_y = 0.075
    # Outer frame and the solid privacy lower panel.
    leaf.visual(
        Box((0.08, 0.070, 1.42)),
        origin=Origin(xyz=(0.06, wood_y, 0.91)),
        material=cedar,
        name="hinge_stile",
    )
    leaf.visual(
        Box((0.08, 0.070, 1.42)),
        origin=Origin(xyz=(1.04, wood_y, 0.91)),
        material=cedar,
        name="latch_stile",
    )
    leaf.visual(
        Box((1.06, 0.070, 0.08)),
        origin=Origin(xyz=(0.55, wood_y, 0.24)),
        material=cedar,
        name="bottom_rail",
    )
    leaf.visual(
        Box((1.06, 0.070, 0.08)),
        origin=Origin(xyz=(0.55, wood_y, 0.84)),
        material=cedar,
        name="middle_rail",
    )
    leaf.visual(
        Box((1.06, 0.070, 0.08)),
        origin=Origin(xyz=(0.55, wood_y, 1.58)),
        material=cedar,
        name="top_rail",
    )
    leaf.visual(
        Box((0.90, 0.042, 0.52)),
        origin=Origin(xyz=(0.55, wood_y, 0.55)),
        material=cedar,
        name="solid_panel",
    )
    for i, x in enumerate((0.25, 0.40, 0.55, 0.70, 0.85)):
        leaf.visual(
            Box((0.014, 0.008, 0.50)),
            origin=Origin(xyz=(x, wood_y - 0.027, 0.55)),
            material=dark_cedar,
            name=f"panel_seam_{i}",
        )

    # Open upper lattice: continuous slats touch the top/middle rails and both
    # stiles, leaving square openings instead of a filled panel.
    for i, x in enumerate((0.22, 0.385, 0.55, 0.715, 0.88)):
        leaf.visual(
            Box((0.035, 0.040, 0.66)),
            origin=Origin(xyz=(x, wood_y, 1.21)),
            material=cedar,
            name=f"lattice_vertical_{i}",
        )
    for i, z in enumerate((1.02, 1.21, 1.40)):
        leaf.visual(
            Box((0.90, 0.040, 0.035)),
            origin=Origin(xyz=(0.55, wood_y, z)),
            material=cedar,
            name=f"lattice_horizontal_{i}",
        )

    for z, name in ((0.55, "lower"), (1.23, "upper")):
        leaf.visual(
            Box((0.224, 0.060, 0.050)),
            origin=Origin(xyz=(0.129, 0.025, z)),
            material=black_iron,
            name=f"{name}_leaf_hinge_strap",
        )

    model.articulation(
        "post_to_leaf",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=leaf,
        origin=Origin(xyz=(0.16, -0.075, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.2),
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.11, 0.014, 0.030)),
        origin=Origin(xyz=(0.035, 0.003, 0.0)),
        material=black_iron,
        name="latch_bar",
    )
    latch.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="pivot_cap",
    )
    latch.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="pivot_pin",
    )

    model.articulation(
        "leaf_to_latch",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=latch,
        origin=Origin(xyz=(1.02, 0.031, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    leaf = object_model.get_part("leaf")
    latch = object_model.get_part("latch")
    gate_hinge = object_model.get_articulation("post_to_leaf")
    latch_pivot = object_model.get_articulation("leaf_to_latch")

    for name in ("lower", "upper"):
        ctx.allow_overlap(
            posts,
            leaf,
            elem_a=f"{name}_hinge_pin",
            elem_b=f"{name}_leaf_hinge_strap",
            reason="The hinge strap is intentionally seated around the fixed hinge pin.",
        )
        ctx.expect_gap(
            leaf,
            posts,
            axis="x",
            positive_elem=f"{name}_leaf_hinge_strap",
            negative_elem=f"{name}_hinge_pin",
            max_penetration=0.003,
            max_gap=0.002,
            name=f"{name} hinge strap captures pin locally",
        )

    ctx.allow_overlap(
        latch,
        leaf,
        elem_a="pivot_pin",
        elem_b="latch_stile",
        reason="The rotary latch pivot pin intentionally passes a short distance into the free stile.",
    )
    ctx.expect_gap(
        leaf,
        latch,
        axis="y",
        positive_elem="latch_stile",
        negative_elem="pivot_pin",
        max_penetration=0.016,
        max_gap=0.002,
        name="latch pivot pin is seated in the stile",
    )
    ctx.expect_overlap(
        latch,
        leaf,
        axes="xz",
        elem_a="pivot_pin",
        elem_b="latch_stile",
        min_overlap=0.020,
        name="latch pivot is centered on the free stile",
    )

    ctx.expect_gap(
        posts,
        leaf,
        axis="x",
        positive_elem="latch_post",
        negative_elem="latch_stile",
        min_gap=0.025,
        max_gap=0.065,
        name="closed leaf has latch-side clearance",
    )
    ctx.expect_gap(
        leaf,
        latch,
        axis="y",
        positive_elem="latch_stile",
        negative_elem="pivot_cap",
        max_penetration=0.0005,
        max_gap=0.004,
        name="latch cap sits on the free stile face",
    )
    ctx.expect_gap(
        latch,
        posts,
        axis="y",
        positive_elem="latch_bar",
        negative_elem="keeper_plate",
        min_gap=0.0,
        max_gap=0.004,
        name="rotary latch closes just behind keeper plate",
    )
    ctx.expect_overlap(
        latch,
        posts,
        axes="xz",
        elem_a="latch_bar",
        elem_b="keeper_plate",
        min_overlap=0.010,
        name="rotary latch aligns with the strike keeper",
    )

    closed_leaf_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({gate_hinge: 1.0}):
        opened_leaf_aabb = ctx.part_world_aabb(leaf)
    ctx.check(
        "leaf swings outward on vertical hinge",
        closed_leaf_aabb is not None
        and opened_leaf_aabb is not None
        and opened_leaf_aabb[1][1] > closed_leaf_aabb[1][1] + 0.45,
        details=f"closed={closed_leaf_aabb}, opened={opened_leaf_aabb}",
    )
    with ctx.pose({gate_hinge: 1.2}):
        ctx.expect_gap(
            leaf,
            posts,
            axis="x",
            positive_elem="hinge_stile",
            negative_elem="hinge_post",
            min_gap=0.005,
            name="opened leaf clears hinge post",
        )

    latched_bar_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    with ctx.pose({latch_pivot: 1.1}):
        lifted_bar_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    ctx.check(
        "latch rotates upward off the keeper",
        latched_bar_aabb is not None
        and lifted_bar_aabb is not None
        and lifted_bar_aabb[1][2] > latched_bar_aabb[1][2] + 0.055
        and lifted_bar_aabb[1][0] < latched_bar_aabb[1][0] - 0.020,
        details=f"latched={latched_bar_aabb}, lifted={lifted_bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
