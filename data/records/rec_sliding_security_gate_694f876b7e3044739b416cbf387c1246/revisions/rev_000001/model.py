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

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_steel = model.material("dark_painted_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    worn_guide = model.material("worn_bottom_guide", rgba=(0.22, 0.22, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((0.14, 0.14, 1.96)),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=galvanized,
        name="lock_post",
    )
    fixed_frame.visual(
        Box((0.14, 0.14, 1.96)),
        origin=Origin(xyz=(5.45, 0.0, 0.98)),
        material=galvanized,
        name="end_post",
    )
    fixed_frame.visual(
        Box((5.60, 0.20, 0.08)),
        origin=Origin(xyz=(2.72, 0.0, 1.96)),
        material=galvanized,
        name="top_web",
    )
    fixed_frame.visual(
        Box((5.60, 0.035, 0.20)),
        origin=Origin(xyz=(2.72, -0.0925, 1.83)),
        material=galvanized,
        name="track_lip_0",
    )
    fixed_frame.visual(
        Box((5.60, 0.035, 0.20)),
        origin=Origin(xyz=(2.72, 0.0925, 1.83)),
        material=galvanized,
        name="track_lip_1",
    )
    fixed_frame.visual(
        Box((5.45, 0.055, 0.07)),
        origin=Origin(xyz=(2.725, 0.0, 0.045)),
        material=worn_guide,
        name="bottom_guide",
    )
    fixed_frame.visual(
        Box((0.30, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="post_foot_0",
    )
    fixed_frame.visual(
        Box((0.30, 0.22, 0.035)),
        origin=Origin(xyz=(5.45, 0.0, 0.0175)),
        material=dark_steel,
        name="post_foot_1",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((2.80, 0.08, 0.08)),
        origin=Origin(xyz=(1.40, 0.0, 1.62)),
        material=dark_steel,
        name="top_tube",
    )
    gate_leaf.visual(
        Box((2.80, 0.08, 0.08)),
        origin=Origin(xyz=(1.40, 0.0, 0.20)),
        material=dark_steel,
        name="bottom_tube",
    )
    gate_leaf.visual(
        Box((0.08, 0.08, 1.50)),
        origin=Origin(xyz=(0.04, 0.0, 0.91)),
        material=dark_steel,
        name="stile_0",
    )
    gate_leaf.visual(
        Box((0.08, 0.08, 1.50)),
        origin=Origin(xyz=(2.76, 0.0, 0.91)),
        material=dark_steel,
        name="stile_1",
    )

    for index, x in enumerate((0.39, 0.74, 1.09, 1.44, 1.79, 2.14, 2.49)):
        gate_leaf.visual(
            Box((0.035, 0.055, 1.40)),
            origin=Origin(xyz=(x, 0.0, 0.91)),
            material=dark_steel,
            name=f"picket_{index}",
        )

    brace_dx = 2.40
    brace_dz = 1.30
    brace_length = math.hypot(brace_dx, brace_dz)
    brace_angle = -math.atan2(brace_dz, brace_dx)
    gate_leaf.visual(
        Box((brace_length, 0.055, 0.055)),
        origin=Origin(xyz=(1.40, 0.0, 0.91), rpy=(0.0, brace_angle, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )

    gate_leaf.visual(
        Box((0.025, 0.115, 0.34)),
        origin=Origin(xyz=(0.005, -0.012, 1.04)),
        material=galvanized,
        name="latch_plate",
    )

    for index, x in enumerate((0.62, 2.18)):
        gate_leaf.visual(
            Box((0.050, 0.035, 0.23)),
            origin=Origin(xyz=(x, 0.0, 1.775)),
            material=galvanized,
            name=f"hanger_{index}",
        )
        gate_leaf.visual(
            Cylinder(radius=0.045, length=0.055),
            origin=Origin(xyz=(x, 0.0, 1.84), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"roller_{index}",
        )

    gate_leaf.visual(
        Box((0.16, 0.035, 0.09)),
        origin=Origin(xyz=(1.40, 0.0, 0.125)),
        material=galvanized,
        name="guide_shoe",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.65, lower=0.0, upper=2.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "gate leaf has one prismatic slide",
        gate_slide.articulation_type == ArticulationType.PRISMATIC
        and gate_slide.axis == (1.0, 0.0, 0.0)
        and gate_slide.motion_limits is not None
        and gate_slide.motion_limits.upper is not None
        and gate_slide.motion_limits.upper > 2.0,
        details=f"type={gate_slide.articulation_type}, axis={gate_slide.axis}, limits={gate_slide.motion_limits}",
    )

    for roller_name in ("roller_0", "roller_1"):
        ctx.expect_within(
            gate_leaf,
            fixed_frame,
            axes="y",
            inner_elem=roller_name,
            outer_elem="top_web",
            margin=0.0,
            name=f"{roller_name} is laterally captured by the top track",
        )
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="z",
            positive_elem="top_web",
            negative_elem=roller_name,
            min_gap=0.015,
            max_gap=0.040,
            name=f"{roller_name} clears underside of top web",
        )

    ctx.expect_overlap(
        gate_leaf,
        fixed_frame,
        axes="x",
        elem_a="top_tube",
        elem_b="top_web",
        min_overlap=2.70,
        name="closed gate leaf remains below the overhead track",
    )
    ctx.expect_gap(
        gate_leaf,
        fixed_frame,
        axis="z",
        positive_elem="bottom_tube",
        negative_elem="bottom_guide",
        min_gap=0.070,
        max_gap=0.090,
        name="bottom tube runs above the clean single guide",
    )
    ctx.expect_contact(
        gate_leaf,
        fixed_frame,
        elem_a="guide_shoe",
        elem_b="bottom_guide",
        contact_tol=0.001,
        name="gate leaf shoe rides on the bottom guide",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({gate_slide: 2.10}):
        ctx.expect_overlap(
            gate_leaf,
            fixed_frame,
            axes="x",
            elem_a="top_tube",
            elem_b="top_web",
            min_overlap=2.70,
            name="extended gate leaf is still carried by the overhead track",
        )
        for roller_name in ("roller_0", "roller_1"):
            ctx.expect_within(
                gate_leaf,
                fixed_frame,
                axes="y",
                inner_elem=roller_name,
                outer_elem="top_web",
                margin=0.0,
                name=f"extended {roller_name} stays inside track width",
            )
        extended_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "upper limit opens the gate along the rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 2.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
