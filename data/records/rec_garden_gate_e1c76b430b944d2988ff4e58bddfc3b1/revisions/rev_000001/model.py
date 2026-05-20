from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_garden_gate")

    weathered_wood = Material("weathered_wood", rgba=(0.48, 0.28, 0.13, 1.0))
    lighter_wood = Material("lighter_picket_wood", rgba=(0.58, 0.36, 0.18, 1.0))
    dark_metal = Material("blackened_hinge_steel", rgba=(0.02, 0.022, 0.02, 1.0))
    dull_screw = Material("dull_screw_heads", rgba=(0.13, 0.13, 0.12, 1.0))
    ground = Material("dark_soil_foundation", rgba=(0.18, 0.15, 0.10, 1.0))
    rubber = Material("black_rubber_stop", rgba=(0.01, 0.01, 0.01, 1.0))

    posts = model.part("posts")
    posts.visual(
        Box((1.55, 0.20, 0.06)),
        origin=Origin(xyz=(0.575, 0.0, 0.03)),
        material=ground,
        name="ground_sill",
    )
    posts.visual(
        Box((0.14, 0.14, 1.35)),
        origin=Origin(xyz=(-0.095, 0.0, 0.675)),
        material=weathered_wood,
        name="hinge_post",
    )
    posts.visual(
        Box((0.14, 0.14, 1.35)),
        origin=Origin(xyz=(1.280, 0.0, 0.675)),
        material=weathered_wood,
        name="latch_post",
    )
    for x, name in [(-0.095, "hinge_post_cap"), (1.280, "latch_post_cap")]:
        posts.visual(
            Box((0.18, 0.18, 0.07)),
            origin=Origin(xyz=(x, 0.0, 1.385)),
            material=weathered_wood,
            name=name,
        )

    # Two fixed post-side hinge plates and barrels.  The moving leaf uses two
    # matching straps that rotate about these vertical barrel axes.
    for z, suffix in [(0.95, "upper"), (0.40, "lower")]:
        posts.visual(
            Box((0.020, 0.080, 0.220)),
            origin=Origin(xyz=(-0.015, -0.010, z)),
            material=dark_metal,
            name=f"{suffix}_post_plate",
        )
        posts.visual(
            Cylinder(radius=0.025, length=0.200),
            origin=Origin(xyz=(0.0, -0.010, z)),
            material=dark_metal,
            name=f"{suffix}_hinge_barrel",
        )

    posts.visual(
        Box((0.0725, 0.060, 0.180)),
        origin=Origin(xyz=(1.17375, 0.0, 0.68)),
        material=rubber,
        name="latch_stop",
    )
    posts.visual(
        Box((0.014, 0.070, 0.110)),
        origin=Origin(xyz=(1.203, -0.038, 0.83)),
        material=dark_metal,
        name="latch_keeper_plate",
    )

    gate = model.part("gate_leaf")
    # The gate leaf frame is authored in the child frame of the upper hinge:
    # x extends across the opening, z is measured relative to the upper barrel.
    gate.visual(
        Box((0.075, 0.055, 1.000)),
        origin=Origin(xyz=(0.080, 0.0, -0.270)),
        material=weathered_wood,
        name="hinge_stile",
    )
    gate.visual(
        Box((0.075, 0.055, 1.000)),
        origin=Origin(xyz=(1.100, 0.0, -0.270)),
        material=weathered_wood,
        name="latch_stile",
    )
    gate.visual(
        Box((1.095, 0.055, 0.075)),
        origin=Origin(xyz=(0.590, 0.0, 0.1925)),
        material=weathered_wood,
        name="top_rail",
    )
    gate.visual(
        Box((1.095, 0.055, 0.075)),
        origin=Origin(xyz=(0.590, 0.0, -0.7325)),
        material=weathered_wood,
        name="bottom_rail",
    )
    for index, x in enumerate((0.220, 0.345, 0.470, 0.595, 0.720, 0.845, 0.970)):
        gate.visual(
            Box((0.045, 0.038, 0.850)),
            origin=Origin(xyz=(x, 0.0, -0.270)),
            material=lighter_wood,
            name=f"picket_{index}",
        )

    brace_angle = math.atan2(0.85, 0.91)
    brace_length = math.hypot(0.91, 0.85)
    gate.visual(
        Box((brace_length, 0.040, 0.080)),
        origin=Origin(xyz=(0.595, -0.035, -0.275), rpy=(0.0, -brace_angle, 0.0)),
        material=weathered_wood,
        name="diagonal_brace",
    )

    gate.visual(
        Box((0.520, 0.012, 0.045)),
        origin=Origin(xyz=(0.300, -0.0335, 0.0)),
        material=dark_metal,
        name="upper_strap_leaf",
    )
    for index, x in enumerate((0.125, 0.295, 0.465)):
        gate.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.0425, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dull_screw,
            name=f"upper_screw_{index}",
        )
    gate.visual(
        Box((0.145, 0.014, 0.024)),
        origin=Origin(xyz=(1.085, -0.0365, -0.120)),
        material=dark_metal,
        name="latch_handle",
    )
    gate.visual(
        Box((0.090, 0.018, 0.028)),
        origin=Origin(xyz=(1.150, -0.0365, -0.120)),
        material=dark_metal,
        name="latch_bar",
    )
    gate.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(1.030, -0.046, -0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="latch_knob",
    )

    lower_strap = model.part("lower_strap")
    lower_strap.visual(
        Box((0.520, 0.012, 0.045)),
        origin=Origin(xyz=(0.300, -0.0750, 0.0)),
        material=dark_metal,
        name="strap_leaf",
    )
    lower_strap.visual(
        Box((0.065, 0.0415, 0.036)),
        origin=Origin(xyz=(0.080, -0.04825, 0.0)),
        material=dark_metal,
        name="mount_pad",
    )
    for index, x in enumerate((0.125, 0.295, 0.465)):
        lower_strap.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.0840, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dull_screw,
            name=f"screw_{index}",
        )

    swing_limits = MotionLimits(
        effort=120.0,
        velocity=1.0,
        lower=0.0,
        upper=math.radians(100.0),
    )
    model.articulation(
        "upper_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=gate,
        origin=Origin(xyz=(0.0, -0.010, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "lower_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=lower_strap,
        origin=Origin(xyz=(0.0, -0.010, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swing_limits,
        mimic=Mimic(joint="upper_hinge", multiplier=1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    gate = object_model.get_part("gate_leaf")
    lower_strap = object_model.get_part("lower_strap")
    upper = object_model.get_articulation("upper_hinge")
    lower = object_model.get_articulation("lower_hinge")

    ctx.check(
        "two revolute hinge joints",
        upper.articulation_type == ArticulationType.REVOLUTE
        and lower.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "hinge barrels share a vertical axis",
        upper.axis == (0.0, 0.0, 1.0)
        and lower.axis == (0.0, 0.0, 1.0)
        and abs(upper.origin.xyz[0] - lower.origin.xyz[0]) < 1e-6
        and abs(upper.origin.xyz[1] - lower.origin.xyz[1]) < 1e-6
        and upper.origin.xyz[2] > lower.origin.xyz[2],
    )
    ctx.check(
        "gate swing limit is about 100 degrees",
        upper.motion_limits is not None
        and upper.motion_limits.lower == 0.0
        and abs(upper.motion_limits.upper - math.radians(100.0)) < 1e-6,
    )

    ctx.expect_gap(
        posts,
        gate,
        axis="x",
        positive_elem="latch_stop",
        negative_elem="latch_stile",
        min_gap=0.0,
        max_gap=0.001,
        name="closed leaf rests against latch stop",
    )
    ctx.expect_overlap(
        gate,
        posts,
        axes="yz",
        elem_a="latch_stile",
        elem_b="latch_stop",
        min_overlap=0.04,
        name="latch stile meets the stop face",
    )
    ctx.expect_contact(
        lower_strap,
        gate,
        elem_a="mount_pad",
        elem_b="hinge_stile",
        contact_tol=0.001,
        name="lower strap is seated on gate stile",
    )

    closed_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
    with ctx.pose({upper: math.radians(100.0)}):
        open_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
        lower_open_aabb = ctx.part_element_world_aabb(lower_strap, elem="strap_leaf")

    if closed_aabb is not None and open_aabb is not None and lower_open_aabb is not None:
        closed_y = 0.5 * (closed_aabb[0][1] + closed_aabb[1][1])
        open_y = 0.5 * (open_aabb[0][1] + open_aabb[1][1])
        lower_open_y = 0.5 * (lower_open_aabb[0][1] + lower_open_aabb[1][1])
        ctx.check(
            "upper hinge swings leaf outward",
            open_y > closed_y + 0.80,
            details=f"closed_y={closed_y:.3f}, open_y={open_y:.3f}",
        )
        ctx.check(
            "lower strap follows the same swing",
            lower_open_y > 0.15,
            details=f"lower_strap_y={lower_open_y:.3f}",
        )
    else:
        ctx.fail("swing pose aabbs available", "Could not measure latch stile or lower strap AABBs.")

    return ctx.report()


object_model = build_object_model()
