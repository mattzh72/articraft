from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_link_arm_with_output_slide")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    blue_paint = model.material("blue_paint", rgba=(0.05, 0.22, 0.62, 1.0))
    orange_paint = model.material("orange_paint", rgba=(0.94, 0.38, 0.06, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.58, 0.60, 0.62, 1.0))

    root = model.part("root_support")
    root.visual(
        Box((0.34, 0.28, 0.04)),
        origin=Origin(xyz=(-0.05, 0.0, -0.42)),
        material=cast_iron,
        name="floor_plate",
    )
    root.visual(
        Box((0.12, 0.10, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=cast_iron,
        name="pedestal",
    )
    root.visual(
        Box((0.085, 0.040, 0.20)),
        origin=Origin(xyz=(0.0, -0.065, -0.10)),
        material=cast_iron,
        name="side_column_0",
    )
    root.visual(
        Box((0.085, 0.040, 0.20)),
        origin=Origin(xyz=(0.0, 0.065, -0.10)),
        material=cast_iron,
        name="side_column_1",
    )
    root.visual(
        Box((0.105, 0.040, 0.18)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=cast_iron,
        name="pivot_cheek_0",
    )
    root.visual(
        Box((0.105, 0.040, 0.18)),
        origin=Origin(xyz=(0.0, 0.065, 0.0)),
        material=cast_iron,
        name="pivot_cheek_1",
    )
    root.visual(
        Cylinder(radius=0.064, length=0.026),
        origin=Origin(xyz=(0.0, -0.098, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="outer_boss_0",
    )
    root.visual(
        Cylinder(radius=0.064, length=0.026),
        origin=Origin(xyz=(0.0, 0.098, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="outer_boss_1",
    )

    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="root_hub",
    )
    upper.visual(
        Box((0.46, 0.050, 0.045)),
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        material=blue_paint,
        name="main_bar",
    )
    upper.visual(
        Box((0.055, 0.140, 0.045)),
        origin=Origin(xyz=(0.4625, 0.0, 0.0)),
        material=blue_paint,
        name="fork_bridge",
    )
    upper.visual(
        Box((0.16, 0.035, 0.045)),
        origin=Origin(xyz=(0.54, -0.055, 0.0)),
        material=blue_paint,
        name="fork_arm_0",
    )
    upper.visual(
        Box((0.16, 0.035, 0.045)),
        origin=Origin(xyz=(0.54, 0.055, 0.0)),
        material=blue_paint,
        name="fork_arm_1",
    )
    upper.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.62, -0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_ear_0",
    )
    upper.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.62, 0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_ear_1",
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.052, length=0.075),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="proximal_lug",
    )
    forelink.visual(
        Box((0.39, 0.045, 0.040)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=orange_paint,
        name="fore_bar",
    )
    forelink.visual(
        Box((0.030, 0.150, 0.070)),
        origin=Origin(xyz=(0.435, 0.0, 0.0)),
        material=orange_paint,
        name="slide_back_wall",
    )
    forelink.visual(
        Box((0.25, 0.030, 0.070)),
        origin=Origin(xyz=(0.575, -0.060, 0.0)),
        material=orange_paint,
        name="guide_side_0",
    )
    forelink.visual(
        Box((0.25, 0.030, 0.070)),
        origin=Origin(xyz=(0.575, 0.060, 0.0)),
        material=orange_paint,
        name="guide_side_1",
    )
    forelink.visual(
        Box((0.25, 0.150, 0.018)),
        origin=Origin(xyz=(0.575, 0.0, 0.044)),
        material=orange_paint,
        name="guide_top",
    )

    runner = model.part("runner")
    runner.visual(
        Box((0.260, 0.090, 0.070)),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material=ground_steel,
        name="slide_body",
    )
    runner.visual(
        Box((0.050, 0.075, 0.055)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=dark_steel,
        name="output_face",
    )
    runner.visual(
        Cylinder(radius=0.034, length=0.036),
        origin=Origin(xyz=(0.308, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="nose_boss",
    )

    model.articulation(
        "root_to_upper",
        ArticulationType.REVOLUTE,
        parent=root,
        child=upper,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=-0.75, upper=1.25),
        motion_properties=MotionProperties(damping=0.4, friction=0.05),
    )
    model.articulation(
        "upper_to_forelink",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forelink,
        origin=Origin(xyz=(0.62, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.8, lower=-1.25, upper=1.40),
        motion_properties=MotionProperties(damping=0.35, friction=0.04),
    )
    model.articulation(
        "forelink_to_runner",
        ArticulationType.PRISMATIC,
        parent=forelink,
        child=runner,
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.45, lower=0.0, upper=0.16),
        motion_properties=MotionProperties(damping=0.25, friction=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_support")
    upper = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    runner = object_model.get_part("runner")
    shoulder = object_model.get_articulation("root_to_upper")
    elbow = object_model.get_articulation("upper_to_forelink")
    slide = object_model.get_articulation("forelink_to_runner")

    ctx.expect_within(
        upper,
        root,
        axes="y",
        inner_elem="root_hub",
        margin=0.0,
        name="upper hub sits between root cheeks",
    )
    ctx.expect_overlap(
        upper,
        root,
        axes="xz",
        elem_a="root_hub",
        elem_b="pivot_cheek_1",
        min_overlap=0.070,
        name="root pivot cheeks straddle the shoulder axis",
    )
    ctx.expect_within(
        forelink,
        upper,
        axes="y",
        inner_elem="proximal_lug",
        margin=0.0,
        name="forelink lug sits inside upper fork",
    )
    ctx.expect_overlap(
        forelink,
        upper,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="elbow_ear_1",
        min_overlap=0.070,
        name="elbow fork and lug share the pivot axis",
    )
    ctx.expect_within(
        runner,
        forelink,
        axes="yz",
        inner_elem="slide_body",
        margin=0.0,
        name="runner is captured inside the distal guide envelope",
    )
    ctx.expect_contact(
        forelink,
        runner,
        elem_a="guide_top",
        elem_b="slide_body",
        name="slide body rides under the guide cap",
    )
    ctx.expect_contact(
        forelink,
        runner,
        elem_a="guide_side_1",
        elem_b="slide_body",
        name="slide body rides against the positive side rail",
    )
    ctx.expect_overlap(
        runner,
        forelink,
        axes="x",
        elem_a="slide_body",
        elem_b="guide_top",
        min_overlap=0.18,
        name="resting runner remains deeply inserted",
    )

    rest_runner_pos = ctx.part_world_position(runner)
    rest_forelink_pos = ctx.part_world_position(forelink)
    with ctx.pose({slide: 0.16}):
        ctx.expect_within(
            runner,
            forelink,
            axes="yz",
            inner_elem="slide_body",
            margin=0.0,
            name="extended runner stays captured in the guide envelope",
        )
        ctx.expect_overlap(
            runner,
            forelink,
            axes="x",
            elem_a="slide_body",
            elem_b="guide_top",
            min_overlap=0.050,
            name="extended runner retains insertion in the guide",
        )
        extended_runner_pos = ctx.part_world_position(runner)

    ctx.check(
        "prismatic runner extends distally",
        rest_runner_pos is not None
        and extended_runner_pos is not None
        and extended_runner_pos[0] > rest_runner_pos[0] + 0.14,
        details=f"rest={rest_runner_pos}, extended={extended_runner_pos}",
    )

    with ctx.pose({shoulder: 0.60, elbow: 0.55}):
        posed_forelink_pos = ctx.part_world_position(forelink)
    ctx.check(
        "revolute chain lifts the distal link",
        rest_forelink_pos is not None
        and posed_forelink_pos is not None
        and posed_forelink_pos[2] > rest_forelink_pos[2] + 0.10,
        details=f"rest={rest_forelink_pos}, posed={posed_forelink_pos}",
    )

    return ctx.report()


object_model = build_object_model()
