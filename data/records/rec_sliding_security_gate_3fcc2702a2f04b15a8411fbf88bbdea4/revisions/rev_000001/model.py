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

    galvanized = model.material("galvanized_steel", rgba=(0.48, 0.52, 0.53, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.03, 0.035, 0.035, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.48, 0.46, 0.42, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    safety_yellow = model.material("yellow_stop_bumpers", rgba=(0.95, 0.72, 0.08, 1.0))

    frame = model.part("frame")

    # A compact fixed portal: one stout fixed post, a lighter stop post, and
    # top/bottom U-shaped guides.  The base plate makes the fixed assembly one
    # continuous supported frame rather than separate floating rails.
    frame.visual(
        Box((2.08, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=concrete,
        name="ground_sill",
    )
    frame.visual(
        Box((0.12, 0.16, 1.60)),
        origin=Origin(xyz=(-0.95, 0.0, 0.83)),
        material=galvanized,
        name="fixed_post",
    )
    frame.visual(
        Box((0.08, 0.13, 1.53)),
        origin=Origin(xyz=(0.95, 0.0, 0.795)),
        material=galvanized,
        name="stop_post",
    )
    frame.visual(
        Box((1.90, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=galvanized,
        name="top_web",
    )
    frame.visual(
        Box((1.90, 0.025, 0.22)),
        origin=Origin(xyz=(0.0, -0.075, 1.44)),
        material=galvanized,
        name="top_lip_0",
    )
    frame.visual(
        Box((1.90, 0.025, 0.22)),
        origin=Origin(xyz=(0.0, 0.075, 1.44)),
        material=galvanized,
        name="top_lip_1",
    )
    frame.visual(
        Box((1.90, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=galvanized,
        name="bottom_channel_base",
    )
    frame.visual(
        Box((1.90, 0.025, 0.18)),
        origin=Origin(xyz=(0.0, -0.075, 0.17)),
        material=galvanized,
        name="bottom_lip_0",
    )
    frame.visual(
        Box((1.90, 0.025, 0.18)),
        origin=Origin(xyz=(0.0, 0.075, 0.17)),
        material=galvanized,
        name="bottom_lip_1",
    )
    frame.visual(
        Box((0.04, 0.18, 0.20)),
        origin=Origin(xyz=(-0.80, 0.0, 0.18)),
        material=safety_yellow,
        name="end_stop_0",
    )
    frame.visual(
        Box((0.04, 0.18, 0.20)),
        origin=Origin(xyz=(0.80, 0.0, 0.18)),
        material=safety_yellow,
        name="end_stop_1",
    )

    gate_leaf = model.part("gate_leaf")

    # The moving leaf is a welded rectangular bar gate.  Its part frame is at
    # the sliding carriage, so q=0 places the leaf partly closed in the compact
    # frame and positive prismatic travel slides it a short distance along +X.
    gate_leaf.visual(
        Box((1.00, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=dark_steel,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((1.00, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=dark_steel,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.07, 0.06, 0.96)),
        origin=Origin(xyz=(-0.465, 0.0, 0.76)),
        material=dark_steel,
        name="stile_0",
    )
    gate_leaf.visual(
        Box((0.07, 0.06, 0.96)),
        origin=Origin(xyz=(0.465, 0.0, 0.76)),
        material=dark_steel,
        name="stile_1",
    )
    for i, x in enumerate((-0.30, -0.15, 0.0, 0.15, 0.30)):
        gate_leaf.visual(
            Box((0.026, 0.035, 0.88)),
            origin=Origin(xyz=(x, 0.0, 0.76)),
            material=dark_steel,
            name=f"picket_{i}",
        )

    brace_angle = -math.atan2(0.80, 0.86)
    gate_leaf.visual(
        Box((1.18, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.76), rpy=(0.0, brace_angle, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )

    gate_leaf.visual(
        Box((0.04, 0.035, 0.16)),
        origin=Origin(xyz=(-0.30, 0.0, 1.31)),
        material=dark_steel,
        name="roller_hanger_0",
    )
    gate_leaf.visual(
        Cylinder(radius=0.04, length=0.045),
        origin=Origin(xyz=(-0.30, 0.0, 1.40), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_0",
    )
    gate_leaf.visual(
        Box((0.04, 0.035, 0.16)),
        origin=Origin(xyz=(0.30, 0.0, 1.31)),
        material=dark_steel,
        name="roller_hanger_1",
    )
    gate_leaf.visual(
        Cylinder(radius=0.04, length=0.045),
        origin=Origin(xyz=(0.30, 0.0, 1.40), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_1",
    )

    gate_leaf.visual(
        Box((0.88, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="bottom_guide_blade",
    )
    for i, x in enumerate((-0.32, 0.0, 0.32)):
        gate_leaf.visual(
            Box((0.04, 0.04, 0.14)),
            origin=Origin(xyz=(x, 0.0, 0.245)),
            material=dark_steel,
            name=f"guide_tab_{i}",
        )

    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(-0.25, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("leaf_slide")

    ctx.check(
        "gate leaf uses a prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type is {slide.articulation_type}",
    )
    ctx.check(
        "sliding travel is short and compact",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and 0.20 <= slide.motion_limits.upper <= 0.40,
        details=f"limits={slide.motion_limits}",
    )

    # At rest and at the short upper travel, the rolling carriage and lower
    # guide remain captured inside the fixed top track/bottom guide.
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="y",
        inner_elem="top_roller_0",
        outer_elem="top_web",
        margin=0.0,
        name="first top roller sits within top channel width",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="y",
        inner_elem="bottom_guide_blade",
        outer_elem="bottom_channel_base",
        margin=0.0,
        name="bottom guide blade sits within lower channel width",
    )
    ctx.expect_gap(
        gate_leaf,
        frame,
        axis="z",
        positive_elem="bottom_guide_blade",
        negative_elem="bottom_channel_base",
        max_gap=0.001,
        max_penetration=0.000001,
        name="bottom guide blade rides on the channel floor",
    )
    ctx.expect_overlap(
        gate_leaf,
        frame,
        axes="x",
        elem_a="bottom_guide_blade",
        elem_b="bottom_channel_base",
        min_overlap=0.70,
        name="guide blade is retained in the lower track at rest",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 0.32}):
        ctx.expect_within(
            gate_leaf,
            frame,
            axes="y",
            inner_elem="top_roller_1",
            outer_elem="top_web",
            margin=0.0,
            name="second top roller remains within top channel at travel",
        )
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="x",
            elem_a="bottom_guide_blade",
            elem_b="bottom_channel_base",
            min_overlap=0.70,
            name="guide blade is retained in the lower track at travel",
        )
        travel_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "positive joint value slides leaf along the track",
        rest_pos is not None
        and travel_pos is not None
        and travel_pos[0] > rest_pos[0] + 0.30
        and abs(travel_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, travel={travel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
