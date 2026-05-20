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
    model = ArticulatedObject(name="long_reach_service_tv_wall_mount")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_gray = model.material("dark_gray_powdercoat", rgba=(0.10, 0.105, 0.11, 1.0))
    pin_steel = model.material("brushed_pin_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    bolt_black = model.material("black_bolt_heads", rgba=(0.02, 0.02, 0.018, 1.0))

    # +X reaches away from the wall, +Z is vertical, and the root frame is the
    # first wall-side hinge axis.
    wall = model.part("wall_bracket")
    wall.visual(
        Box((0.035, 0.300, 0.560)),
        origin=Origin(xyz=(-0.066, 0.0, 0.0)),
        material=dark_gray,
        name="wall_plate",
    )
    wall.visual(
        Box((0.090, 0.150, 0.050)),
        origin=Origin(xyz=(-0.036, 0.0, 0.135)),
        material=dark_gray,
        name="upper_clevis",
    )
    wall.visual(
        Box((0.090, 0.150, 0.050)),
        origin=Origin(xyz=(-0.036, 0.0, -0.135)),
        material=dark_gray,
        name="lower_clevis",
    )
    wall.visual(
        Cylinder(radius=0.016, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_steel,
        name="wall_pin",
    )
    for i, (yy, zz) in enumerate(((-0.095, -0.205), (0.095, -0.205), (-0.095, 0.205), (0.095, 0.205))):
        wall.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(-0.046, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"lag_bolt_{i}",
        )

    arm_0 = model.part("arm_0")
    arm_0.visual(
        Cylinder(radius=0.036, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="proximal_sleeve",
    )
    arm_0.visual(
        Box((0.480, 0.076, 0.048)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=matte_black,
        name="long_link_body",
    )
    arm_0.visual(
        Box((0.110, 0.020, 0.080)),
        origin=Origin(xyz=(0.545, 0.046, 0.0)),
        material=matte_black,
        name="elbow_fork_0",
    )
    arm_0.visual(
        Box((0.110, 0.020, 0.080)),
        origin=Origin(xyz=(0.545, -0.046, 0.0)),
        material=matte_black,
        name="elbow_fork_1",
    )
    arm_1 = model.part("arm_1")
    arm_1.visual(
        Cylinder(radius=0.036, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="proximal_sleeve",
    )
    arm_1.visual(
        Box((0.433, 0.072, 0.044)),
        origin=Origin(xyz=(0.2485, 0.0, 0.0)),
        material=matte_black,
        name="long_link_body",
    )
    arm_1.visual(
        Box((0.095, 0.020, 0.074)),
        origin=Origin(xyz=(0.495, 0.044, 0.0)),
        material=matte_black,
        name="head_fork_0",
    )
    arm_1.visual(
        Box((0.095, 0.020, 0.074)),
        origin=Origin(xyz=(0.495, -0.044, 0.0)),
        material=matte_black,
        name="head_fork_1",
    )
    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.034, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="swivel_sleeve",
    )
    swivel.visual(
        Box((0.065, 0.065, 0.055)),
        origin=Origin(xyz=(0.0575, 0.0, 0.0)),
        material=matte_black,
        name="short_neck",
    )
    swivel.visual(
        Box((0.018, 0.132, 0.050)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=matte_black,
        name="neck_spreader",
    )
    swivel.visual(
        Box((0.070, 0.018, 0.050)),
        origin=Origin(xyz=(0.105, 0.055, 0.0)),
        material=matte_black,
        name="neck_strap_0",
    )
    swivel.visual(
        Box((0.070, 0.018, 0.050)),
        origin=Origin(xyz=(0.105, -0.055, 0.0)),
        material=matte_black,
        name="neck_strap_1",
    )
    swivel.visual(
        Box((0.035, 0.022, 0.118)),
        origin=Origin(xyz=(0.140, 0.055, 0.0)),
        material=matte_black,
        name="tilt_ear_0",
    )
    swivel.visual(
        Box((0.035, 0.022, 0.118)),
        origin=Origin(xyz=(0.140, -0.055, 0.0)),
        material=matte_black,
        name="tilt_ear_1",
    )
    swivel.visual(
        Cylinder(radius=0.012, length=0.145),
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="tilt_pin",
    )

    output = model.part("output_frame")
    output.visual(
        Cylinder(radius=0.026, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="tilt_barrel",
    )
    output.visual(
        Box((0.074, 0.056, 0.058)),
        origin=Origin(xyz=(0.063, 0.0, 0.0)),
        material=matte_black,
        name="tilt_lug",
    )
    output.visual(
        Box((0.024, 0.060, 0.190)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=dark_gray,
        name="vertical_spine",
    )
    output.visual(
        Box((0.024, 0.240, 0.034)),
        origin=Origin(xyz=(0.100, 0.0, 0.066)),
        material=dark_gray,
        name="upper_vesa_rail",
    )
    output.visual(
        Box((0.024, 0.240, 0.034)),
        origin=Origin(xyz=(0.100, 0.0, -0.066)),
        material=dark_gray,
        name="lower_vesa_rail",
    )
    for i, (yy, zz) in enumerate(((-0.085, -0.066), (0.085, -0.066), (-0.085, 0.066), (0.085, 0.066))):
        output.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(0.115, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"vesa_hole_{i}",
        )

    model.articulation(
        "wall_yaw",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=arm_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=-1.70, upper=1.70),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=arm_0,
        child=arm_1,
        origin=Origin(xyz=(0.550, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.0, lower=-2.65, upper=2.65),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=swivel,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=output,
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.8, lower=-0.40, upper=0.40),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_bracket")
    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    swivel = object_model.get_part("swivel")
    output = object_model.get_part("output_frame")
    wall_yaw = object_model.get_articulation("wall_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.allow_overlap(
        wall,
        arm_0,
        elem_a="wall_pin",
        elem_b="proximal_sleeve",
        reason="The wall hinge pin is intentionally captured inside the first arm sleeve.",
    )
    ctx.allow_overlap(
        swivel,
        output,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The horizontal tilt pin is intentionally captured inside the output-frame barrel.",
    )

    ctx.expect_within(
        wall,
        arm_0,
        axes="xy",
        inner_elem="wall_pin",
        outer_elem="proximal_sleeve",
        margin=0.002,
        name="wall pin is centered in first sleeve",
    )
    ctx.expect_overlap(
        wall,
        arm_0,
        axes="z",
        elem_a="wall_pin",
        elem_b="proximal_sleeve",
        min_overlap=0.160,
        name="wall hinge has retained vertical pin engagement",
    )
    ctx.expect_gap(
        arm_0,
        arm_1,
        axis="y",
        positive_elem="elbow_fork_0",
        negative_elem="proximal_sleeve",
        max_gap=0.004,
        max_penetration=0.00001,
        name="outer elbow fork closely captures the second sleeve",
    )
    ctx.expect_gap(
        arm_1,
        arm_0,
        axis="y",
        positive_elem="proximal_sleeve",
        negative_elem="elbow_fork_1",
        max_gap=0.004,
        max_penetration=0.00001,
        name="inner elbow fork closely captures the second sleeve",
    )
    ctx.expect_gap(
        arm_1,
        swivel,
        axis="y",
        positive_elem="head_fork_0",
        negative_elem="swivel_sleeve",
        max_gap=0.004,
        max_penetration=0.00001,
        name="outer head fork closely captures the swivel sleeve",
    )
    ctx.expect_gap(
        swivel,
        arm_1,
        axis="y",
        positive_elem="swivel_sleeve",
        negative_elem="head_fork_1",
        max_gap=0.004,
        max_penetration=0.00001,
        name="inner head fork closely captures the swivel sleeve",
    )
    ctx.expect_within(
        swivel,
        output,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_barrel",
        margin=0.002,
        name="tilt pin is centered in output barrel",
    )
    ctx.expect_overlap(
        swivel,
        output,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.075,
        name="tilt hinge has retained horizontal pin engagement",
    )

    ctx.expect_origin_distance(
        arm_0,
        arm_1,
        axes="x",
        min_dist=0.54,
        max_dist=0.56,
        name="first fold joint sits at the far end of the first link",
    )
    ctx.expect_origin_distance(
        arm_1,
        swivel,
        axes="x",
        min_dist=0.49,
        max_dist=0.51,
        name="head swivel sits at the far end of the second link",
    )
    ctx.check(
        "four revolute joints provide two arm folds plus swivel and tilt",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (wall_yaw, elbow_yaw, head_swivel, head_tilt)
        ),
        details="Expected wall_yaw, elbow_yaw, head_swivel, and head_tilt to be revolute.",
    )
    ctx.check(
        "arm links are distinctly longer than compact head support",
        0.50 > 3.0 * 0.14 and 0.55 > 3.0 * 0.14,
        details="Arm links are modeled as 0.55 m and 0.50 m spans; head neck is 0.14 m.",
    )

    rest_pos = ctx.part_world_position(output)
    with ctx.pose({elbow_yaw: -1.55, head_swivel: 0.70, head_tilt: 0.32}):
        folded_pos = ctx.part_world_position(output)
        ctx.expect_origin_distance(
            output,
            wall,
            axes="xy",
            min_dist=0.15,
            max_dist=0.95,
            name="folded pose keeps output frame carried by the arms",
        )

    ctx.check(
        "folding and head joints move the output frame",
        rest_pos is not None
        and folded_pos is not None
        and abs(folded_pos[1] - rest_pos[1]) > 0.15,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
