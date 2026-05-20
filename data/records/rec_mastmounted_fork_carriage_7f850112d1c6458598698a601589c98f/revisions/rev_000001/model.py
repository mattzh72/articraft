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
    model = ArticulatedObject(name="open_industrial_fork_mast")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.36, 0.37, 0.35, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.46, 0.08, 1.0))
    fork_black = model.material("fork_black", rgba=(0.06, 0.06, 0.055, 1.0))

    mast = model.part("mast")
    # Fixed outer mast: two tall uprights joined by top and bottom ties,
    # leaving the center open for the translating carriage.
    mast.visual(
        Box((0.10, 0.16, 2.20)),
        origin=Origin(xyz=(-0.42, 0.0, 1.10)),
        material=dark_steel,
        name="left_upright",
    )
    mast.visual(
        Box((0.10, 0.16, 2.20)),
        origin=Origin(xyz=(0.42, 0.0, 1.10)),
        material=dark_steel,
        name="right_upright",
    )
    mast.visual(
        Box((0.94, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 2.17)),
        material=dark_steel,
        name="top_tie",
    )
    mast.visual(
        Box((0.94, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_steel,
        name="bottom_tie",
    )
    mast.visual(
        Box((0.16, 0.30, 0.05)),
        origin=Origin(xyz=(-0.42, 0.04, 0.025)),
        material=worn_steel,
        name="left_base_foot",
    )
    mast.visual(
        Box((0.16, 0.30, 0.05)),
        origin=Origin(xyz=(0.42, 0.04, 0.025)),
        material=worn_steel,
        name="right_base_foot",
    )

    fork_frame = model.part("fork_frame")
    # The child frame origin is on the prismatic guide line.  Its local +Z
    # motion lifts the whole front carriage while the side blocks remain inside
    # the mast uprights.
    fork_frame.visual(
        Box((0.07, 0.08, 0.80)),
        origin=Origin(xyz=(-0.31, 0.0, 0.40)),
        material=safety_orange,
        name="left_side_stile",
    )
    fork_frame.visual(
        Box((0.07, 0.08, 0.80)),
        origin=Origin(xyz=(0.31, 0.0, 0.40)),
        material=safety_orange,
        name="right_side_stile",
    )
    fork_frame.visual(
        Box((0.72, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=safety_orange,
        name="lower_crossbar",
    )
    fork_frame.visual(
        Box((0.72, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=safety_orange,
        name="upper_crossbar",
    )
    # Wear blocks and rollers on both sides make the guided lift path visible.
    fork_frame.visual(
        Box((0.035, 0.07, 0.14)),
        origin=Origin(xyz=(-0.347, 0.015, 0.24)),
        material=worn_steel,
        name="left_lower_guide_block",
    )
    fork_frame.visual(
        Box((0.035, 0.07, 0.14)),
        origin=Origin(xyz=(-0.347, 0.015, 0.64)),
        material=worn_steel,
        name="left_upper_guide_block",
    )
    fork_frame.visual(
        Cylinder(radius=0.023, length=0.085),
        origin=Origin(xyz=(-0.347, 0.025, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_black,
        name="left_lower_roller",
    )
    fork_frame.visual(
        Cylinder(radius=0.023, length=0.085),
        origin=Origin(xyz=(-0.347, 0.025, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_black,
        name="left_upper_roller",
    )
    fork_frame.visual(
        Box((0.035, 0.07, 0.14)),
        origin=Origin(xyz=(0.347, 0.015, 0.24)),
        material=worn_steel,
        name="right_lower_guide_block",
    )
    fork_frame.visual(
        Box((0.035, 0.07, 0.14)),
        origin=Origin(xyz=(0.347, 0.015, 0.64)),
        material=worn_steel,
        name="right_upper_guide_block",
    )
    fork_frame.visual(
        Cylinder(radius=0.023, length=0.085),
        origin=Origin(xyz=(0.347, 0.025, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_black,
        name="right_lower_roller",
    )
    fork_frame.visual(
        Cylinder(radius=0.023, length=0.085),
        origin=Origin(xyz=(0.347, 0.025, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_black,
        name="right_upper_roller",
    )

    for x, name in ((-0.20, "fork_0"), (0.20, "fork_1")):
        fork_frame.visual(
            Box((0.10, 0.88, 0.08)),
            origin=Origin(xyz=(x, -0.55, -0.18)),
            material=fork_black,
            name=f"{name}_tine",
        )
        fork_frame.visual(
            Box((0.10, 0.10, 0.25)),
            origin=Origin(xyz=(x, -0.08, -0.05)),
            material=fork_black,
            name=f"{name}_heel",
        )

    model.articulation(
        "mast_to_fork_frame",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=fork_frame,
        origin=Origin(xyz=(0.0, -0.14, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.35, lower=0.0, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    fork_frame = object_model.get_part("fork_frame")
    slide = object_model.get_articulation("mast_to_fork_frame")

    ctx.check(
        "single vertical prismatic carriage joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper >= 0.75,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    ctx.expect_within(
        fork_frame,
        mast,
        axes="x",
        margin=0.0,
        name="moving fork frame stays between twin mast uprights",
    )
    ctx.expect_gap(
        mast,
        fork_frame,
        axis="x",
        positive_elem="right_upright",
        negative_elem="right_upper_guide_block",
        min_gap=0.002,
        max_gap=0.030,
        name="right guide block runs close to right upright",
    )
    ctx.expect_gap(
        fork_frame,
        mast,
        axis="x",
        positive_elem="left_upper_guide_block",
        negative_elem="left_upright",
        min_gap=0.002,
        max_gap=0.030,
        name="left guide block runs close to left upright",
    )
    ctx.expect_contact(
        fork_frame,
        mast,
        elem_a="right_upper_roller",
        elem_b="right_upright",
        contact_tol=0.001,
        name="right upper roller contacts mast rail",
    )
    ctx.expect_contact(
        fork_frame,
        mast,
        elem_a="left_upper_roller",
        elem_b="left_upright",
        contact_tol=0.001,
        name="left upper roller contacts mast rail",
    )

    rest_position = ctx.part_world_position(fork_frame)
    with ctx.pose({slide: 0.80}):
        ctx.expect_within(
            fork_frame,
            mast,
            axes="x",
            margin=0.0,
            name="raised fork frame remains laterally captured",
        )
        ctx.expect_overlap(
            fork_frame,
            mast,
            axes="z",
            min_overlap=0.45,
            name="raised frame remains engaged in mast height",
        )
        raised_position = ctx.part_world_position(fork_frame)

    ctx.check(
        "fork frame translates upward on the mast",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.75,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
