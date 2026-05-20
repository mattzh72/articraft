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
    model = ArticulatedObject(name="revolute_prismatic_revolute_chain")

    dark_metal = model.material("dark_metal", color=(0.08, 0.09, 0.10, 1.0))
    blued_steel = model.material("blued_steel", color=(0.18, 0.23, 0.29, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.62, 0.66, 0.68, 1.0))
    safety_orange = model.material("safety_orange", color=(0.95, 0.38, 0.10, 1.0))
    rubber_black = model.material("rubber_black", color=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.28, 0.20, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=blued_steel,
        name="fixed_bearing",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed_steel,
        name="base_spindle",
    )
    for x in (-0.105, 0.105):
        for y in (-0.065, 0.065):
            base.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x, y, 0.020)),
                material=brushed_steel,
                name=f"mount_bolt_{x:+.3f}_{y:+.3f}",
            )

    swing_carriage = model.part("swing_carriage")
    swing_carriage.visual(
        Cylinder(radius=0.040, length=0.052),
        origin=Origin(),
        material=blued_steel,
        name="pivot_hub",
    )
    swing_carriage.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_metal,
        name="upper_thrust_plate",
    )
    # Four connected walls form a real rectangular slide sleeve instead of a
    # solid proxy, leaving clearance for the moving middle member.
    sleeve_x = 0.143
    sleeve_len = 0.215
    swing_carriage.visual(
        Box((sleeve_len, 0.068, 0.010)),
        origin=Origin(xyz=(sleeve_x, 0.0, 0.022)),
        material=blued_steel,
        name="sleeve_top",
    )
    swing_carriage.visual(
        Box((sleeve_len, 0.068, 0.010)),
        origin=Origin(xyz=(sleeve_x, 0.0, -0.022)),
        material=blued_steel,
        name="sleeve_bottom",
    )
    swing_carriage.visual(
        Box((sleeve_len, 0.010, 0.044)),
        origin=Origin(xyz=(sleeve_x, 0.027, 0.0)),
        material=blued_steel,
        name="sleeve_side_0",
    )
    swing_carriage.visual(
        Box((sleeve_len, 0.010, 0.044)),
        origin=Origin(xyz=(sleeve_x, -0.027, 0.0)),
        material=blued_steel,
        name="sleeve_side_1",
    )
    swing_carriage.visual(
        Box((0.120, 0.010, 0.004)),
        origin=Origin(xyz=(0.152, 0.0, 0.029)),
        material=brushed_steel,
        name="travel_scale",
    )

    middle_slide = model.part("middle_slide")
    middle_slide.visual(
        Box((0.278, 0.032, 0.022)),
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=brushed_steel,
        name="slide_bar",
    )
    middle_slide.visual(
        Box((0.060, 0.014, 0.007)),
        origin=Origin(xyz=(0.060, 0.0, 0.0135)),
        material=dark_metal,
        name="upper_wear_pad",
    )
    middle_slide.visual(
        Box((0.060, 0.014, 0.007)),
        origin=Origin(xyz=(0.060, 0.0, -0.0135)),
        material=dark_metal,
        name="lower_wear_pad",
    )
    middle_slide.visual(
        Box((0.024, 0.070, 0.026)),
        origin=Origin(xyz=(0.246, 0.0, 0.0)),
        material=brushed_steel,
        name="fork_mount_bridge",
    )
    middle_slide.visual(
        Box((0.046, 0.012, 0.052)),
        origin=Origin(xyz=(0.277, 0.028, 0.0)),
        material=blued_steel,
        name="pin_cheek_0",
    )
    middle_slide.visual(
        Box((0.046, 0.012, 0.052)),
        origin=Origin(xyz=(0.277, -0.028, 0.0)),
        material=blued_steel,
        name="pin_cheek_1",
    )
    middle_slide.visual(
        Cylinder(radius=0.006, length=0.078),
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="fork_pin",
    )
    middle_slide.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.280, 0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pin_cap_0",
    )
    middle_slide.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.280, -0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pin_cap_1",
    )
    middle_slide.visual(
        Box((0.012, 0.054, 0.034)),
        origin=Origin(xyz=(0.183, 0.0, 0.0)),
        material=safety_orange,
        name="slide_stop",
    )

    tool_fork = model.part("tool_fork")
    tool_fork.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="fork_hub",
    )
    tool_fork.visual(
        Box((0.042, 0.024, 0.016)),
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
        material=safety_orange,
        name="fork_tang",
    )
    tool_fork.visual(
        Box((0.016, 0.062, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=safety_orange,
        name="fork_bridge",
    )
    tool_fork.visual(
        Box((0.080, 0.011, 0.012)),
        origin=Origin(xyz=(0.108, 0.025, 0.0)),
        material=safety_orange,
        name="fork_tine_0",
    )
    tool_fork.visual(
        Box((0.080, 0.011, 0.012)),
        origin=Origin(xyz=(0.108, -0.025, 0.0)),
        material=safety_orange,
        name="fork_tine_1",
    )
    tool_fork.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.150, 0.025, 0.0)),
        material=rubber_black,
        name="tine_tip_0",
    )
    tool_fork.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.150, -0.025, 0.0)),
        material=rubber_black,
        name="tine_tip_1",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=swing_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-2.09439510239,
            upper=2.09439510239,
        ),
    )
    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=swing_carriage,
        child=middle_slide,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.100),
    )
    model.articulation(
        "fork_pin",
        ArticulationType.REVOLUTE,
        parent=middle_slide,
        child=tool_fork,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-1.308996939,
            upper=1.308996939,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    swing = object_model.get_part("swing_carriage")
    slide = object_model.get_part("middle_slide")
    fork = object_model.get_part("tool_fork")

    base_hinge = object_model.get_articulation("base_hinge")
    slide_joint = object_model.get_articulation("middle_slide")
    fork_pin = object_model.get_articulation("fork_pin")

    ctx.allow_overlap(
        base,
        swing,
        elem_a="base_spindle",
        elem_b="pivot_hub",
        reason="The fixed spindle is intentionally captured inside the rotating hub bore proxy.",
    )
    ctx.expect_within(
        base,
        swing,
        axes="xy",
        inner_elem="base_spindle",
        outer_elem="pivot_hub",
        margin=0.001,
        name="spindle is centered in base hinge hub",
    )
    ctx.expect_overlap(
        base,
        swing,
        axes="z",
        elem_a="base_spindle",
        elem_b="pivot_hub",
        min_overlap=0.045,
        name="base hinge spindle has captured length",
    )

    ctx.allow_overlap(
        slide,
        fork,
        elem_a="fork_pin",
        elem_b="fork_hub",
        reason="The fork pin is intentionally represented as passing through the fork hub bore.",
    )
    ctx.expect_within(
        slide,
        fork,
        axes="xz",
        inner_elem="fork_pin",
        outer_elem="fork_hub",
        margin=0.001,
        name="fork pin is centered in distal hub",
    )
    ctx.expect_overlap(
        slide,
        fork,
        axes="y",
        elem_a="fork_pin",
        elem_b="fork_hub",
        min_overlap=0.025,
        name="fork pin crosses the distal hub",
    )

    ctx.expect_within(
        slide,
        swing,
        axes="y",
        inner_elem="slide_bar",
        outer_elem="sleeve_top",
        margin=0.0,
        name="slide bar is centered within sleeve width",
    )
    ctx.expect_within(
        slide,
        swing,
        axes="z",
        inner_elem="slide_bar",
        outer_elem="sleeve_side_0",
        margin=0.0,
        name="slide bar clears sleeve height",
    )
    ctx.expect_overlap(
        slide,
        swing,
        axes="x",
        elem_a="slide_bar",
        elem_b="sleeve_top",
        min_overlap=0.150,
        name="collapsed slide remains retained in sleeve",
    )
    ctx.expect_contact(
        slide,
        swing,
        elem_a="upper_wear_pad",
        elem_b="sleeve_top",
        name="slide wear pad bears against sleeve",
    )

    with ctx.pose({slide_joint: 0.100}):
        ctx.expect_overlap(
            slide,
            swing,
            axes="x",
            elem_a="slide_bar",
            elem_b="sleeve_top",
            min_overlap=0.050,
            name="extended slide still has retained insertion",
        )

    ctx.check(
        "base hinge travel is about 120 degrees each way",
        base_hinge.motion_limits is not None
        and math.isclose(base_hinge.motion_limits.lower, -2.09439510239, abs_tol=1e-6)
        and math.isclose(base_hinge.motion_limits.upper, 2.09439510239, abs_tol=1e-6),
    )
    ctx.check(
        "middle slide travel is 100 mm",
        slide_joint.motion_limits is not None
        and math.isclose(slide_joint.motion_limits.lower, 0.0, abs_tol=1e-9)
        and math.isclose(slide_joint.motion_limits.upper, 0.100, abs_tol=1e-9),
    )
    ctx.check(
        "fork pin travel is about 75 degrees each way",
        fork_pin.motion_limits is not None
        and math.isclose(fork_pin.motion_limits.lower, -1.308996939, abs_tol=1e-6)
        and math.isclose(fork_pin.motion_limits.upper, 1.308996939, abs_tol=1e-6),
    )

    return ctx.report()


object_model = build_object_model()
