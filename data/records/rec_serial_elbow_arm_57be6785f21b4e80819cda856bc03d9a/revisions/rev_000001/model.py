from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="fork_root_elbow_module")

    painted_fork = model.material("painted_fork", rgba=(0.20, 0.22, 0.24, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.12, 0.27, 0.46, 1.0))
    forearm_paint = model.material("forearm_paint", rgba=(0.18, 0.37, 0.52, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_bore = model.material("dark_bore", rgba=(0.03, 0.035, 0.04, 1.0))

    y_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))

    root_fork = model.part("root_fork")
    root_fork.visual(
        Box((0.30, 0.24, 0.035)),
        origin=Origin(xyz=(-0.05, 0.0, -0.18)),
        material=painted_fork,
        name="base_plate",
    )
    root_fork.visual(
        Box((0.18, 0.13, 0.08)),
        origin=Origin(xyz=(-0.045, 0.0, -0.125)),
        material=painted_fork,
        name="pedestal",
    )
    root_fork.visual(
        Box((0.040, 0.18, 0.18)),
        origin=Origin(xyz=(-0.072, 0.0, -0.015)),
        material=painted_fork,
        name="rear_bridge",
    )
    root_fork.visual(
        Box((0.12, 0.025, 0.22)),
        origin=Origin(xyz=(0.0, 0.075, 0.0)),
        material=painted_fork,
        name="fork_cheek_pos",
    )
    root_fork.visual(
        Box((0.12, 0.025, 0.22)),
        origin=Origin(xyz=(0.0, -0.075, 0.0)),
        material=painted_fork,
        name="fork_cheek_neg",
    )
    root_fork.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=y_axis,
        material=steel,
        name="shoulder_pin",
    )
    root_fork.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(0.0, 0.091, 0.0), rpy=y_axis.rpy),
        material=steel,
        name="shoulder_cap_pos",
    )
    root_fork.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(0.0, -0.091, 0.0), rpy=y_axis.rpy),
        material=steel,
        name="shoulder_cap_neg",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.045, length=0.068),
        origin=y_axis,
        material=steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=y_axis.rpy),
        material=dark_bore,
        name="shoulder_bore_pos",
    )
    upper_arm.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=y_axis.rpy),
        material=dark_bore,
        name="shoulder_bore_neg",
    )
    upper_arm.visual(
        Box((0.38, 0.040, 0.032)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=arm_paint,
        name="upper_web",
    )
    upper_arm.visual(
        Box((0.34, 0.052, 0.010)),
        origin=Origin(xyz=(0.24, 0.0, 0.020)),
        material=arm_paint,
        name="upper_flange_top",
    )
    upper_arm.visual(
        Box((0.34, 0.052, 0.010)),
        origin=Origin(xyz=(0.24, 0.0, -0.020)),
        material=arm_paint,
        name="upper_flange_bottom",
    )
    upper_arm.visual(
        Box((0.035, 0.126, 0.056)),
        origin=Origin(xyz=(0.392, 0.0, 0.0)),
        material=arm_paint,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.115, 0.022, 0.095)),
        origin=Origin(xyz=(0.46, 0.052, 0.0)),
        material=arm_paint,
        name="elbow_cheek_pos",
    )
    upper_arm.visual(
        Box((0.115, 0.022, 0.095)),
        origin=Origin(xyz=(0.46, -0.052, 0.0)),
        material=arm_paint,
        name="elbow_cheek_neg",
    )
    upper_arm.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=y_axis.rpy),
        material=steel,
        name="elbow_pin",
    )
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.46, 0.067, 0.0), rpy=y_axis.rpy),
        material=steel,
        name="elbow_cap_pos",
    )
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.46, -0.067, 0.0), rpy=y_axis.rpy),
        material=steel,
        name="elbow_cap_neg",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=y_axis,
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.415, 0.036, 0.030)),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material=forearm_paint,
        name="forearm_web",
    )
    forearm.visual(
        Box((0.35, 0.046, 0.009)),
        origin=Origin(xyz=(0.255, 0.0, 0.019)),
        material=forearm_paint,
        name="forearm_flange_top",
    )
    forearm.visual(
        Box((0.35, 0.046, 0.009)),
        origin=Origin(xyz=(0.255, 0.0, -0.019)),
        material=forearm_paint,
        name="forearm_flange_bottom",
    )
    forearm.visual(
        Box((0.035, 0.14, 0.12)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=steel,
        name="end_plate",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=root_fork,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=-1.15, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=0.0, upper=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_fork = object_model.get_part("root_fork")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        root_fork,
        upper_arm,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The visible shoulder shaft is intentionally captured inside the upper-arm root hub.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The visible elbow shaft is intentionally captured inside the forearm root hub.",
    )

    ctx.check(
        "shoulder and elbow are revolutes",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
    )

    ctx.expect_overlap(
        root_fork,
        upper_arm,
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.030,
        name="shoulder pin passes through shoulder hub",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.025,
        name="elbow pin passes through forearm hub",
    )

    ctx.expect_overlap(
        upper_arm,
        root_fork,
        axes="xz",
        elem_a="shoulder_hub",
        elem_b="fork_cheek_pos",
        min_overlap=0.07,
        name="shoulder hub is captured by root fork cheek",
    )
    ctx.expect_gap(
        root_fork,
        upper_arm,
        axis="y",
        positive_elem="fork_cheek_pos",
        negative_elem="shoulder_hub",
        min_gap=0.015,
        max_gap=0.040,
        name="positive root cheek clears shoulder hub",
    )
    ctx.expect_gap(
        upper_arm,
        root_fork,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="fork_cheek_neg",
        min_gap=0.015,
        max_gap=0.040,
        name="negative root cheek clears shoulder hub",
    )

    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        elem_a="elbow_hub",
        elem_b="elbow_cheek_pos",
        min_overlap=0.065,
        name="forearm hub is captured by elbow fork cheek",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_pos",
        negative_elem="elbow_hub",
        min_gap=0.010,
        max_gap=0.030,
        name="positive elbow cheek clears forearm hub",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_neg",
        min_gap=0.010,
        max_gap=0.030,
        name="negative elbow cheek clears forearm hub",
    )

    with ctx.pose({elbow: 1.0}):
        ctx.expect_gap(
            forearm,
            upper_arm,
            axis="z",
            positive_elem="end_plate",
            negative_elem="upper_web",
            min_gap=0.25,
            name="elbow rotation raises the forearm end plate",
        )

    return ctx.report()


object_model = build_object_model()
