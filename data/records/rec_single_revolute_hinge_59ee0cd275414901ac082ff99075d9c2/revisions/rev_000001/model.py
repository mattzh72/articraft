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
    model = ArticulatedObject(name="single_leaf_garden_gate")

    green = model.material("dark_green_painted_steel", rgba=(0.04, 0.22, 0.12, 1.0))
    black = model.material("black_latch_iron", rgba=(0.015, 0.014, 0.012, 1.0))
    galvanized = model.material("galvanized_hinge_metal", rgba=(0.56, 0.58, 0.56, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.48, 0.46, 0.40, 1.0))
    post_paint = model.material("painted_square_posts", rgba=(0.06, 0.17, 0.10, 1.0))

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((1.40, 0.18, 0.08)),
        origin=Origin(xyz=(0.54, 0.0, 0.04)),
        material=concrete,
        name="ground_sill",
    )
    fixed_frame.visual(
        Box((0.12, 0.12, 1.35)),
        origin=Origin(xyz=(-0.09, 0.0, 0.675)),
        material=post_paint,
        name="hinge_post",
    )
    fixed_frame.visual(
        Box((0.12, 0.12, 1.35)),
        origin=Origin(xyz=(1.17, 0.0, 0.675)),
        material=post_paint,
        name="latch_post",
    )
    fixed_frame.visual(
        Box((0.16, 0.16, 0.055)),
        origin=Origin(xyz=(-0.09, 0.0, 1.3775)),
        material=post_paint,
        name="hinge_post_cap",
    )
    fixed_frame.visual(
        Box((0.16, 0.16, 0.055)),
        origin=Origin(xyz=(1.17, 0.0, 1.3775)),
        material=post_paint,
        name="latch_post_cap",
    )

    # Fixed pintle plates carry vertical pins that sit inside the swinging hinge
    # knuckles.  Small under-collars and welded ears show how each pintle is
    # supported from the post instead of floating on the hinge axis.
    fixed_frame.visual(
        Box((0.060, 0.050, 0.120)),
        origin=Origin(xyz=(-0.035, -0.060, 0.35)),
        material=galvanized,
        name="pintle_plate_0",
    )
    fixed_frame.visual(
        Cylinder(radius=0.012, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=galvanized,
        name="pintle_pin_0",
    )
    fixed_frame.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        material=galvanized,
        name="pintle_collar_0",
    )
    fixed_frame.visual(
        Box((0.030, 0.100, 0.018)),
        origin=Origin(xyz=(0.0, -0.041, 0.260)),
        material=galvanized,
        name="pintle_ear_0",
    )
    fixed_frame.visual(
        Box((0.022, 0.028, 0.110)),
        origin=Origin(xyz=(-0.016, -0.060, 0.315)),
        material=galvanized,
        name="pintle_web_0",
    )
    fixed_frame.visual(
        Box((0.060, 0.050, 0.120)),
        origin=Origin(xyz=(-0.035, -0.060, 0.92)),
        material=galvanized,
        name="pintle_plate_1",
    )
    fixed_frame.visual(
        Cylinder(radius=0.012, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=galvanized,
        name="pintle_pin_1",
    )
    fixed_frame.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.834)),
        material=galvanized,
        name="pintle_collar_1",
    )
    fixed_frame.visual(
        Box((0.030, 0.100, 0.018)),
        origin=Origin(xyz=(0.0, -0.041, 0.830)),
        material=galvanized,
        name="pintle_ear_1",
    )
    fixed_frame.visual(
        Box((0.022, 0.028, 0.110)),
        origin=Origin(xyz=(-0.016, -0.060, 0.885)),
        material=galvanized,
        name="pintle_web_1",
    )

    fixed_frame.visual(
        Box((0.026, 0.080, 0.180)),
        origin=Origin(xyz=(1.105, -0.030, 0.720)),
        material=black,
        name="catch_plate",
    )
    fixed_frame.visual(
        Box((0.065, 0.030, 0.026)),
        origin=Origin(xyz=(1.075, -0.050, 0.775)),
        material=black,
        name="catch_upper_lip",
    )
    fixed_frame.visual(
        Box((0.065, 0.030, 0.026)),
        origin=Origin(xyz=(1.075, -0.050, 0.665)),
        material=black,
        name="catch_lower_lip",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.055, 0.050, 1.000)),
        origin=Origin(xyz=(0.080, 0.0, 0.660)),
        material=green,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((0.055, 0.050, 1.000)),
        origin=Origin(xyz=(1.025, 0.0, 0.660)),
        material=green,
        name="latch_stile",
    )
    gate_leaf.visual(
        Box((1.000, 0.050, 0.055)),
        origin=Origin(xyz=(0.552, 0.0, 1.1325)),
        material=green,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.000, 0.050, 0.055)),
        origin=Origin(xyz=(0.552, 0.0, 0.1875)),
        material=green,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.945, 0.042, 0.045)),
        origin=Origin(xyz=(0.552, 0.0, 0.660)),
        material=green,
        name="middle_rail",
    )

    brace_dx = 0.82
    brace_dz = 0.84
    brace_len = math.hypot(brace_dx, brace_dz)
    brace_angle = math.atan2(brace_dz, brace_dx)
    gate_leaf.visual(
        Box((brace_len, 0.036, 0.046)),
        origin=Origin(xyz=(0.550, 0.0, 0.660), rpy=(0.0, -brace_angle, 0.0)),
        material=green,
        name="diagonal_brace",
    )

    for i, x in enumerate((0.315, 0.552, 0.790)):
        gate_leaf.visual(
            Box((0.026, 0.036, 0.870)),
            origin=Origin(xyz=(x, 0.0, 0.660)),
            material=green,
            name=f"picket_{i}",
        )

    gate_leaf.visual(
        Cylinder(radius=0.028, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=galvanized,
        name="hinge_knuckle_0",
    )
    gate_leaf.visual(
        Box((0.180, 0.018, 0.070)),
        origin=Origin(xyz=(0.075, 0.032, 0.35)),
        material=galvanized,
        name="hinge_strap_0",
    )
    gate_leaf.visual(
        Cylinder(radius=0.028, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=galvanized,
        name="hinge_knuckle_1",
    )
    gate_leaf.visual(
        Box((0.180, 0.018, 0.070)),
        origin=Origin(xyz=(0.075, 0.032, 0.92)),
        material=galvanized,
        name="hinge_strap_1",
    )

    gate_leaf.visual(
        Box((0.240, 0.026, 0.036)),
        origin=Origin(xyz=(0.945, -0.034, 0.720)),
        material=black,
        name="latch_bar",
    )
    gate_leaf.visual(
        Box((0.050, 0.030, 0.170)),
        origin=Origin(xyz=(0.825, -0.040, 0.665)),
        material=black,
        name="latch_handle",
    )
    gate_leaf.visual(
        Box((0.090, 0.018, 0.090)),
        origin=Origin(xyz=(0.825, -0.028, 0.720)),
        material=black,
        name="latch_backplate",
    )

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    hinge = object_model.get_articulation("gate_hinge")

    ctx.allow_overlap(
        fixed_frame,
        gate_leaf,
        elem_a="pintle_pin_0",
        elem_b="hinge_knuckle_0",
        reason="The fixed vertical pintle is intentionally represented as captured inside the swinging hinge knuckle sleeve.",
    )
    ctx.allow_overlap(
        fixed_frame,
        gate_leaf,
        elem_a="pintle_pin_1",
        elem_b="hinge_knuckle_1",
        reason="The fixed vertical pintle is intentionally represented as captured inside the swinging hinge knuckle sleeve.",
    )
    ctx.expect_within(
        fixed_frame,
        gate_leaf,
        axes="xy",
        inner_elem="pintle_pin_0",
        outer_elem="hinge_knuckle_0",
        margin=0.001,
        name="lower pintle is centered in the hinge sleeve",
    )
    ctx.expect_overlap(
        gate_leaf,
        fixed_frame,
        axes="z",
        elem_a="hinge_knuckle_0",
        elem_b="pintle_pin_0",
        min_overlap=0.14,
        name="lower hinge hardware shares one vertical span",
    )
    ctx.expect_within(
        fixed_frame,
        gate_leaf,
        axes="xy",
        inner_elem="pintle_pin_1",
        outer_elem="hinge_knuckle_1",
        margin=0.001,
        name="upper pintle is centered in the hinge sleeve",
    )
    ctx.expect_overlap(
        gate_leaf,
        fixed_frame,
        axes="z",
        elem_a="hinge_knuckle_1",
        elem_b="pintle_pin_1",
        min_overlap=0.14,
        name="upper hinge hardware shares one vertical span",
    )
    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="x",
        positive_elem="latch_post",
        negative_elem="latch_stile",
        min_gap=0.02,
        max_gap=0.12,
        name="closed leaf leaves a latch-side clearance",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({hinge: 1.20}):
        open_pos = ctx.part_world_position(gate_leaf)
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="y",
            positive_elem="latch_stile",
            negative_elem="hinge_post",
            min_gap=0.15,
            name="open leaf swings outward from posts",
        )

    ctx.check(
        "gate has a single swinging leaf hinge",
        hinge.axis == (0.0, 0.0, 1.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 1.5,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "leaf origin remains on the hinge axis while opening",
        rest_pos is not None and open_pos is not None and abs(rest_pos[0] - open_pos[0]) < 1e-6 and abs(rest_pos[1] - open_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
