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
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("dark_galvanized_steel", color=(0.18, 0.19, 0.18, 1.0))
    rail_metal = model.material("worn_black_track", color=(0.04, 0.045, 0.045, 1.0))
    concrete = model.material("cast_concrete", color=(0.48, 0.47, 0.43, 1.0))
    wheel_rubber = model.material("black_rubber_wheel", color=(0.015, 0.015, 0.014, 1.0))

    fixed_frame = model.part("fixed_frame")

    # One continuous sill ties the posts and bottom guide into the ground.
    fixed_frame.visual(
        Box((5.85, 0.32, 0.08)),
        origin=Origin(xyz=(1.30, 0.0, 0.04)),
        material=concrete,
        name="concrete_sill",
    )

    # Fixed receiving/latch post, opening-side support post, and far storage post.
    for x, name in (
        (-1.55, "latch_post"),
        (1.55, "guide_post"),
        (4.15, "storage_post"),
    ):
        fixed_frame.visual(
            Box((0.16, 0.16, 2.13)),
            origin=Origin(xyz=(x, -0.13, 1.065)),
            material=galvanized,
            name=name,
        )

    fixed_frame.visual(
        Box((5.85, 0.10, 0.12)),
        origin=Origin(xyz=(1.30, -0.13, 2.05)),
        material=galvanized,
        name="top_header",
    )

    # Long, simple U-shaped top channel: one roof plate and two continuous lips.
    fixed_frame.visual(
        Box((5.85, 0.18, 0.05)),
        origin=Origin(xyz=(1.30, 0.0, 2.045)),
        material=rail_metal,
        name="top_track_roof",
    )
    fixed_frame.visual(
        Box((5.85, 0.035, 0.18)),
        origin=Origin(xyz=(1.30, -0.075, 1.945)),
        material=rail_metal,
        name="top_rear_lip",
    )
    fixed_frame.visual(
        Box((5.85, 0.035, 0.18)),
        origin=Origin(xyz=(1.30, 0.075, 1.945)),
        material=rail_metal,
        name="top_front_lip",
    )

    # Matching bottom guide channel, kept as a few large pieces rather than many
    # small brackets along the travel path.
    fixed_frame.visual(
        Box((5.85, 0.18, 0.04)),
        origin=Origin(xyz=(1.30, 0.0, 0.09)),
        material=rail_metal,
        name="bottom_guide_base",
    )
    fixed_frame.visual(
        Box((5.85, 0.035, 0.18)),
        origin=Origin(xyz=(1.30, -0.075, 0.19)),
        material=rail_metal,
        name="bottom_rear_lip",
    )
    fixed_frame.visual(
        Box((5.85, 0.035, 0.18)),
        origin=Origin(xyz=(1.30, 0.075, 0.19)),
        material=rail_metal,
        name="bottom_front_lip",
    )

    gate_leaf = model.part("gate_leaf")

    # The gate leaf frame is authored around its sliding guide centerline.  At
    # q=0 it closes the opening between the fixed posts; positive q opens it by
    # sliding to the storage side.
    gate_leaf.visual(
        Box((2.70, 0.07, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=galvanized,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((2.70, 0.07, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.58)),
        material=galvanized,
        name="top_rail",
    )
    for x, name in ((-1.30, "stile_0"), (1.30, "stile_1")):
        gate_leaf.visual(
            Box((0.10, 0.07, 1.50)),
            origin=Origin(xyz=(x, 0.0, 0.87)),
            material=galvanized,
            name=name,
        )

    for i, x in enumerate((-1.02, -0.68, -0.34, 0.0, 0.34, 0.68, 1.02)):
        gate_leaf.visual(
            Cylinder(radius=0.018, length=1.36),
            origin=Origin(xyz=(x, 0.0, 0.87)),
            material=galvanized,
            name=f"bar_{i}",
        )

    gate_leaf.visual(
        Box((2.55, 0.045, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.68)),
        material=rail_metal,
        name="top_guide",
    )
    gate_leaf.visual(
        Box((2.55, 0.045, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=rail_metal,
        name="bottom_shoe",
    )

    for x, name in ((-0.88, "wheel_0"), (0.88, "wheel_1")):
        gate_leaf.visual(
            Cylinder(radius=0.055, length=0.045),
            origin=Origin(xyz=(x, 0.0, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name=name,
        )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=0.0, upper=2.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_leaf")

    ctx.expect_overlap(
        gate_leaf,
        fixed_frame,
        axes="x",
        elem_a="top_guide",
        elem_b="top_track_roof",
        min_overlap=2.40,
        name="closed leaf retained by top track",
    )
    ctx.expect_within(
        gate_leaf,
        fixed_frame,
        axes="y",
        inner_elem="top_guide",
        outer_elem="top_track_roof",
        margin=0.0,
        name="top guide centered inside track width",
    )
    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="y",
        positive_elem="top_front_lip",
        negative_elem="top_guide",
        min_gap=0.015,
        max_gap=0.060,
        name="front top lip clears guide",
    )
    ctx.expect_gap(
        gate_leaf,
        fixed_frame,
        axis="y",
        positive_elem="top_guide",
        negative_elem="top_rear_lip",
        min_gap=0.015,
        max_gap=0.060,
        name="rear top lip clears guide",
    )
    ctx.expect_within(
        gate_leaf,
        fixed_frame,
        axes="y",
        inner_elem="bottom_shoe",
        outer_elem="bottom_guide_base",
        margin=0.0,
        name="bottom shoe centered in guide",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 2.25}):
        ctx.expect_overlap(
            gate_leaf,
            fixed_frame,
            axes="x",
            elem_a="top_guide",
            elem_b="top_track_roof",
            min_overlap=2.40,
            name="opened leaf remains under top track",
        )
        ctx.expect_overlap(
            gate_leaf,
            fixed_frame,
            axes="x",
            elem_a="bottom_shoe",
            elem_b="bottom_guide_base",
            min_overlap=2.40,
            name="opened leaf remains in bottom guide",
        )
        ctx.expect_within(
            gate_leaf,
            fixed_frame,
            axes="y",
            inner_elem="bottom_shoe",
            outer_elem="bottom_guide_base",
            margin=0.0,
            name="opened bottom shoe remains centered",
        )
        opened_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "leaf slides along track axis",
        rest_pos is not None and opened_pos is not None and opened_pos[0] > rest_pos[0] + 2.0,
        details=f"rest={rest_pos}, opened={opened_pos}",
    )

    return ctx.report()


object_model = build_object_model()
