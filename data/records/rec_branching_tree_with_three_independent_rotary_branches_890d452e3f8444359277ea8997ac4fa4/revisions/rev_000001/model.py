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
    model = ArticulatedObject(name="three_branch_inspection_fixture_head")

    painted = model.material("painted_gunmetal", rgba=(0.28, 0.30, 0.31, 1.0))
    dark = model.material("black_oxide", rgba=(0.03, 0.032, 0.035, 1.0))
    machined = model.material("machined_steel", rgba=(0.66, 0.67, 0.64, 1.0))
    cap_blue = model.material("blue_anodized_pad", rgba=(0.12, 0.28, 0.62, 1.0))
    safety = model.material("safety_orange_pad", rgba=(0.92, 0.36, 0.08, 1.0))
    nickel = model.material("nickel_mounting_face", rgba=(0.78, 0.76, 0.68, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.38, 0.28, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="bench_sole_plate",
    )
    spine.visual(
        Box((0.12, 0.10, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=painted,
        name="rectangular_mast",
    )
    spine.visual(
        Box((0.16, 0.035, 0.46)),
        origin=Origin(xyz=(0.0, 0.067, 0.38)),
        material=painted,
        name="front_welded_strip",
    )
    spine.visual(
        Box((0.15, 0.035, 0.46)),
        origin=Origin(xyz=(0.0, -0.067, 0.87)),
        material=painted,
        name="rear_welded_strip",
    )
    for x in (-0.105, 0.105):
        spine.visual(
            Box((0.035, 0.18, 0.20)),
            origin=Origin(xyz=(x, 0.0, 0.105)),
            material=painted,
            name=f"base_gusset_{x:+.2f}",
        )

    # Low transverse-shaft trunnion for the side arm.
    spine.visual(
        Box((0.18, 0.040, 0.160)),
        origin=Origin(xyz=(0.0, 0.070, 0.340)),
        material=machined,
        name="low_collar_plate",
    )
    for x in (-0.066, 0.066):
        spine.visual(
            Box((0.034, 0.095, 0.110)),
            origin=Origin(xyz=(x, 0.130, 0.340)),
            material=machined,
            name=f"low_trunnion_lug_{x:+.2f}",
        )
        spine.visual(
            Cylinder(radius=0.034, length=0.012),
            origin=Origin(xyz=(x * 1.33, 0.130, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"low_bearing_cap_{x:+.2f}",
        )
    spine.visual(
        Box((0.13, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.095, 0.415)),
        material=dark,
        name="low_stop_bar",
    )

    # Mid-height vertical stub support for the forward arm.
    spine.visual(
        Box((0.045, 0.200, 0.160)),
        origin=Origin(xyz=(0.0825, 0.0, 0.620)),
        material=machined,
        name="mid_collar_plate",
    )
    spine.visual(
        Box((0.165, 0.250, 0.040)),
        origin=Origin(xyz=(0.160, 0.0, 0.570)),
        material=machined,
        name="mid_stub_pedestal",
    )
    spine.visual(
        Cylinder(radius=0.046, length=0.045),
        origin=Origin(xyz=(0.220, 0.0, 0.5825)),
        material=machined,
        name="mid_lower_bearing",
    )
    for y in (-0.112, 0.112):
        spine.visual(
            Box((0.030, 0.018, 0.125)),
            origin=Origin(xyz=(0.175, y, 0.627)),
            material=dark,
            name=f"mid_guard_post_{y:+.2f}",
        )

    # High longitudinal bearing block for the upper arm.
    spine.visual(
        Box((0.200, 0.040, 0.180)),
        origin=Origin(xyz=(0.0, -0.070, 0.950)),
        material=machined,
        name="high_collar_plate",
    )
    spine.visual(
        Box((0.120, 0.060, 0.120)),
        origin=Origin(xyz=(0.0, -0.120, 0.950)),
        material=machined,
        name="high_bearing_block",
    )
    spine.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(0.0, -0.1325, 0.950), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="high_bearing_cap",
    )
    spine.visual(
        Box((0.140, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, -0.108, 1.026)),
        material=dark,
        name="high_top_stop",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.035, length=0.098),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="transverse_hub",
    )
    lower_arm.visual(
        Box((0.055, 0.420, 0.045)),
        origin=Origin(xyz=(0.0, 0.245, 0.0)),
        material=painted,
        name="side_box_arm",
    )
    lower_arm.visual(
        Box((0.150, 0.120, 0.028)),
        origin=Origin(xyz=(0.0, 0.505, -0.002)),
        material=cap_blue,
        name="rectangular_pad",
    )
    for x in (-0.045, 0.045):
        lower_arm.visual(
            Cylinder(radius=0.012, length=0.005),
            origin=Origin(xyz=(x, 0.505, 0.0145)),
            material=dark,
            name=f"lower_pad_socket_{x:+.2f}",
        )

    forward_arm = model.part("forward_arm")
    forward_arm.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(),
        material=machined,
        name="vertical_turntable",
    )
    forward_arm.visual(
        Box((0.420, 0.060, 0.045)),
        origin=Origin(xyz=(0.255, 0.0, 0.020)),
        material=painted,
        name="forward_box_arm",
    )
    forward_arm.visual(
        Cylinder(radius=0.075, length=0.026),
        origin=Origin(xyz=(0.500, 0.0, 0.020)),
        material=safety,
        name="round_pad",
    )
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        forward_arm.visual(
            Cylinder(radius=0.010, length=0.005),
            origin=Origin(xyz=(0.500 + 0.043 * math.cos(angle), 0.043 * math.sin(angle), 0.0355)),
            material=dark,
            name=f"forward_pad_socket_{int(angle * 100):03d}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="longitudinal_hub",
    )
    upper_arm.visual(
        Box((0.055, 0.480, 0.050)),
        origin=Origin(xyz=(0.0, -0.290, 0.0)),
        material=painted,
        name="upper_box_arm",
    )
    upper_arm.visual(
        Box((0.160, 0.025, 0.140)),
        origin=Origin(xyz=(0.0, -0.535, 0.0)),
        material=nickel,
        name="flange_pad",
    )
    for x in (-0.050, 0.050):
        for z in (-0.043, 0.043):
            upper_arm.visual(
                Cylinder(radius=0.010, length=0.005),
                origin=Origin(xyz=(x, -0.550, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark,
                name=f"upper_pad_bolt_{x:+.2f}_{z:+.2f}",
            )

    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.130, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=0.85),
    )
    model.articulation(
        "forward_yaw",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=forward_arm,
        origin=Origin(xyz=(0.220, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=-0.62, upper=0.62),
    )
    model.articulation(
        "upper_roll",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_arm,
        origin=Origin(xyz=(0.0, -0.150, 0.950)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.0, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    lower_arm = object_model.get_part("lower_arm")
    forward_arm = object_model.get_part("forward_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_pivot = object_model.get_articulation("lower_pivot")
    forward_yaw = object_model.get_articulation("forward_yaw")
    upper_roll = object_model.get_articulation("upper_roll")

    ctx.expect_gap(
        lower_arm,
        spine,
        axis="y",
        min_gap=0.080,
        positive_elem="side_box_arm",
        negative_elem="rectangular_mast",
        name="lower side arm clears mast face",
    )
    ctx.expect_gap(
        forward_arm,
        spine,
        axis="x",
        min_gap=0.160,
        positive_elem="forward_box_arm",
        negative_elem="rectangular_mast",
        name="forward arm clears mast face",
    )
    ctx.expect_gap(
        spine,
        upper_arm,
        axis="y",
        min_gap=0.140,
        positive_elem="rectangular_mast",
        negative_elem="upper_box_arm",
        name="upper arm clears rear mast face",
    )

    lower_rest = ctx.part_world_aabb(lower_arm)
    with ctx.pose({lower_pivot: 0.85}):
        ctx.expect_gap(
            lower_arm,
            spine,
            axis="y",
            min_gap=0.070,
            positive_elem="side_box_arm",
            negative_elem="rectangular_mast",
            name="raised lower arm clears trunnion",
        )
        lower_raised = ctx.part_world_aabb(lower_arm)
    ctx.check(
        "lower pivot lifts side pad",
        lower_rest is not None
        and lower_raised is not None
        and lower_raised[1][2] > lower_rest[1][2] + 0.20,
        details=f"rest={lower_rest}, raised={lower_raised}",
    )

    for angle, label in ((-0.62, "negative"), (0.62, "positive")):
        with ctx.pose({forward_yaw: angle}):
            ctx.expect_gap(
                forward_arm,
                spine,
                axis="x",
                min_gap=0.160,
                positive_elem="forward_box_arm",
                negative_elem="rectangular_mast",
                name=f"{label} forward yaw clears mast",
            )
            ctx.expect_gap(
                forward_arm,
                lower_arm,
                axis="x",
                min_gap=0.090,
                positive_elem="forward_box_arm",
                negative_elem="rectangular_pad",
                name=f"{label} forward yaw stays ahead of lower arm",
            )

    with ctx.pose({upper_roll: 0.70}):
        ctx.expect_gap(
            spine,
            upper_arm,
            axis="y",
            min_gap=0.140,
            positive_elem="rectangular_mast",
            negative_elem="upper_box_arm",
            name="rolled upper arm clears bearing block",
        )
    with ctx.pose({upper_roll: -0.70}):
        ctx.expect_gap(
            spine,
            upper_arm,
            axis="y",
            min_gap=0.140,
            positive_elem="rectangular_mast",
            negative_elem="upper_box_arm",
            name="counter-rolled upper arm clears bearing block",
        )

    return ctx.report()


object_model = build_object_model()
