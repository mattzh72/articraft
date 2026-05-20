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


Y_AXIS_CYL = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
X_AXIS_CYL = Origin(rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_robotic_arm")

    aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.70, 1.0))
    dark = model.material("black_oxide", rgba=(0.03, 0.035, 0.04, 1.0))
    graphite = model.material("graphite_housings", rgba=(0.18, 0.19, 0.20, 1.0))
    blue = model.material("blue_anodized_links", rgba=(0.08, 0.22, 0.55, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.52, 0.42, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Box((0.26, 0.24, 0.38)),
        origin=Origin(xyz=(-0.04, 0.0, 0.245)),
        material=aluminum,
        name="pedestal",
    )
    base.visual(
        Box((0.18, 0.30, 0.08)),
        origin=Origin(xyz=(-0.035, 0.0, 0.405)),
        material=aluminum,
        name="shoulder_bridge",
    )
    for y, suffix in ((0.14, "0"), (-0.14, "1")):
        base.visual(
            Box((0.22, 0.04, 0.25)),
            origin=Origin(xyz=(0.0, y, 0.55)),
            material=aluminum,
            name=f"shoulder_cheek_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.115, length=0.030),
            origin=Origin(xyz=(0.0, y + (0.025 if y > 0 else -0.025), 0.55), rpy=Y_AXIS_CYL.rpy),
            material=dark,
            name=f"shoulder_bearing_{suffix}",
        )
    base.visual(
        Cylinder(radius=0.026, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.55), rpy=Y_AXIS_CYL.rpy),
        material=dark,
        name="shoulder_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.086, length=0.18),
        origin=Origin(rpy=Y_AXIS_CYL.rpy),
        material=dark,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Box((0.475, 0.085, 0.085)),
        origin=Origin(xyz=(0.2875, 0.0, 0.0)),
        material=blue,
        name="upper_link",
    )
    upper_arm.visual(
        Box((0.105, 0.24, 0.12)),
        origin=Origin(xyz=(0.475, 0.0, 0.0)),
        material=aluminum,
        name="elbow_bridge",
    )
    for y, suffix in ((0.11, "0"), (-0.11, "1")):
        upper_arm.visual(
            Box((0.145, 0.040, 0.17)),
            origin=Origin(xyz=(0.60, y, 0.0)),
            material=aluminum,
            name=f"elbow_cheek_{suffix}",
        )
        upper_arm.visual(
            Cylinder(radius=0.077, length=0.020),
            origin=Origin(xyz=(0.60, y + (0.025 if y > 0 else -0.025), 0.0), rpy=Y_AXIS_CYL.rpy),
            material=dark,
            name=f"elbow_bearing_{suffix}",
        )
    upper_arm.visual(
        Cylinder(radius=0.018, length=0.28),
        origin=Origin(xyz=(0.60, 0.0, 0.0), rpy=Y_AXIS_CYL.rpy),
        material=dark,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.068, length=0.15),
        origin=Origin(rpy=Y_AXIS_CYL.rpy),
        material=dark,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.365, 0.070, 0.070)),
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        material=blue,
        name="forearm_link",
    )
    forearm.visual(
        Box((0.085, 0.18, 0.095)),
        origin=Origin(xyz=(0.365, 0.0, 0.0)),
        material=aluminum,
        name="wrist_bridge",
    )
    for y, suffix in ((0.078, "0"), (-0.078, "1")):
        forearm.visual(
            Box((0.105, 0.030, 0.115)),
            origin=Origin(xyz=(0.45, y, 0.0)),
            material=aluminum,
            name=f"wrist_cheek_{suffix}",
        )
        forearm.visual(
            Cylinder(radius=0.052, length=0.015),
            origin=Origin(xyz=(0.45, y + (0.020 if y > 0 else -0.020), 0.0), rpy=Y_AXIS_CYL.rpy),
            material=dark,
            name=f"wrist_bearing_{suffix}",
        )
    forearm.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.45, 0.0, 0.0), rpy=Y_AXIS_CYL.rpy),
        material=dark,
        name="wrist_pin",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.045, length=0.09),
        origin=Origin(rpy=Y_AXIS_CYL.rpy),
        material=dark,
        name="wrist_barrel",
    )
    wrist.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=X_AXIS_CYL.rpy),
        material=aluminum,
        name="tool_neck",
    )
    wrist.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=X_AXIS_CYL.rpy),
        material=aluminum,
        name="tool_flange",
    )
    for y, z, suffix in (
        (0.038, 0.038, "0"),
        (-0.038, 0.038, "1"),
        (0.038, -0.038, "2"),
        (-0.038, -0.038, "3"),
    ):
        wrist.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.153, y, z), rpy=X_AXIS_CYL.rpy),
            material=dark,
            name=f"flange_bolt_{suffix}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.0, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=-1.9, upper=1.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist")

    for joint in (shoulder, elbow, wrist_joint):
        ctx.check(
            f"{joint.name} is a hinge",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    ctx.allow_overlap(
        base,
        upper_arm,
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        reason="The dark shoulder shaft is intentionally captured inside the rotating shoulder barrel.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        reason="The elbow hinge pin intentionally passes through the forearm barrel.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_pin",
        elem_b="wrist_barrel",
        reason="The wrist hinge pin intentionally passes through the wrist barrel.",
    )

    ctx.expect_within(
        base,
        upper_arm,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_barrel",
        margin=0.002,
        name="shoulder pin lies inside shoulder barrel bore",
    )
    ctx.expect_overlap(
        base,
        upper_arm,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        min_overlap=0.17,
        name="shoulder pin spans the barrel",
    )
    ctx.expect_within(
        upper_arm,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_barrel",
        margin=0.002,
        name="elbow pin lies inside elbow barrel bore",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        min_overlap=0.14,
        name="elbow pin spans the barrel",
    )
    ctx.expect_within(
        forearm,
        wrist,
        axes="xz",
        inner_elem="wrist_pin",
        outer_elem="wrist_barrel",
        margin=0.002,
        name="wrist pin lies inside wrist barrel bore",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="y",
        elem_a="wrist_pin",
        elem_b="wrist_barrel",
        min_overlap=0.08,
        name="wrist pin spans the barrel",
    )

    with ctx.pose({shoulder: -0.65, elbow: 1.1, wrist_joint: -0.8}):
        rest = ctx.part_world_position(wrist)
        ctx.check(
            "wrist remains carried by the three-hinge chain",
            rest is not None and rest[2] > 0.12,
            details=f"wrist position={rest}",
        )

    return ctx.report()


object_model = build_object_model()
