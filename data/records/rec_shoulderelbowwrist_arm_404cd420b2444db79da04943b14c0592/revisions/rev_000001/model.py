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
    model = ArticulatedObject(name="fork_root_three_joint_arm")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    blue_paint = model.material("blue_painted_link", rgba=(0.04, 0.18, 0.55, 1.0))
    orange_paint = model.material("orange_wrist_flange", rgba=(0.95, 0.38, 0.08, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))

    root = model.part("root_fork")
    root.visual(
        Box((0.46, 0.32, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="ground_plate",
    )
    root.visual(
        Box((0.20, 0.18, 0.17)),
        origin=Origin(xyz=(-0.02, 0.0, 0.13)),
        material=cast_iron,
        name="pedestal_block",
    )
    root.visual(
        Box((0.05, 0.18, 0.22)),
        origin=Origin(xyz=(-0.085, 0.0, 0.33)),
        material=cast_iron,
        name="fork_bridge",
    )
    root.visual(
        Box((0.14, 0.035, 0.30)),
        origin=Origin(xyz=(0.0, -0.080, 0.33)),
        material=cast_iron,
        name="shoulder_cheek_0",
    )
    root.visual(
        Box((0.14, 0.035, 0.30)),
        origin=Origin(xyz=(0.0, 0.080, 0.33)),
        material=cast_iron,
        name="shoulder_cheek_1",
    )
    for index, y in enumerate((-0.1015, 0.1015)):
        root.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"shoulder_bushing_{index}",
        )
    root.visual(
        Cylinder(radius=0.013, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_pin",
    )
    for index, (x, y) in enumerate(((-0.16, -0.105), (-0.16, 0.105), (0.16, -0.105), (0.16, 0.105))):
        root.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(x, y, 0.049)),
            material=black,
            name=f"base_bolt_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.050, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.45, 0.070, 0.060)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        material=blue_paint,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.050, 0.156, 0.065)),
        origin=Origin(xyz=(0.435, 0.0, 0.0)),
        material=blue_paint,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.17, 0.028, 0.115)),
        origin=Origin(xyz=(0.535, -0.064, 0.0)),
        material=blue_paint,
        name="elbow_cheek_0",
    )
    upper_arm.visual(
        Box((0.17, 0.028, 0.115)),
        origin=Origin(xyz=(0.535, 0.064, 0.0)),
        material=blue_paint,
        name="elbow_cheek_1",
    )
    for index, y in enumerate((-0.085, 0.085)):
        upper_arm.visual(
            Cylinder(radius=0.027, length=0.014),
            origin=Origin(xyz=(0.58, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"elbow_bushing_{index}",
        )
    upper_arm.visual(
        Cylinder(radius=0.011, length=0.180),
        origin=Origin(xyz=(0.58, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.042, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.34, 0.060, 0.050)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=blue_paint,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.28, 0.022, 0.066)),
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        material=blue_paint,
        name="forearm_center_rib",
    )
    forearm.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(0.405, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wrist_collar",
    )

    wrist_flange = model.part("wrist_flange")
    wrist_flange.visual(
        Cylinder(radius=0.068, length=0.025),
        origin=Origin(xyz=(0.0125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=orange_paint,
        name="flange_disk",
    )
    wrist_flange.visual(
        Cylinder(radius=0.037, length=0.055),
        origin=Origin(xyz=(0.0525, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="output_boss",
    )
    for index, (y, z) in enumerate(((0.045, 0.0), (0.0, 0.045), (-0.045, 0.0), (0.0, -0.045))):
        wrist_flange.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.029, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"flange_bolt_{index}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=root,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=-0.75, upper=1.55),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.8, lower=0.0, upper=2.25),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_flange,
        origin=Origin(xyz=(0.44, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_fork")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_flange = object_model.get_part("wrist_flange")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.allow_overlap(
        root,
        upper_arm,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The grounded fork's shoulder pin is intentionally seated through the upper arm hub.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow pin is intentionally captured through the forearm hub.",
    )

    ctx.check(
        "three revolute joints in sequence",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    ctx.expect_gap(
        root,
        upper_arm,
        axis="y",
        positive_elem="shoulder_cheek_1",
        negative_elem="shoulder_hub",
        min_gap=0.006,
        max_gap=0.016,
        name="upper arm clears positive shoulder cheek",
    )
    ctx.expect_gap(
        upper_arm,
        root,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_0",
        min_gap=0.006,
        max_gap=0.016,
        name="upper arm clears negative shoulder cheek",
    )
    ctx.expect_overlap(
        upper_arm,
        root,
        axes="xz",
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_1",
        min_overlap=0.075,
        name="shoulder hub is captured inside the fork cheek profile",
    )
    ctx.expect_within(
        root,
        upper_arm,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_hub",
        margin=0.001,
        name="shoulder pin passes through the hub bore line",
    )
    ctx.expect_overlap(
        root,
        upper_arm,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.095,
        name="shoulder pin spans the captured hub",
    )

    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_1",
        negative_elem="elbow_hub",
        min_gap=0.007,
        max_gap=0.018,
        name="forearm clears positive elbow cheek",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_0",
        min_gap=0.007,
        max_gap=0.018,
        name="forearm clears negative elbow cheek",
    )
    ctx.expect_within(
        upper_arm,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.001,
        name="elbow pin passes through the forearm hub bore line",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.070,
        name="elbow pin spans the forearm hub",
    )
    ctx.expect_contact(
        forearm,
        wrist_flange,
        elem_a="wrist_collar",
        elem_b="flange_disk",
        contact_tol=0.001,
        name="wrist flange seats on the forearm collar",
    )

    rest_forearm = ctx.part_world_position(forearm)
    rest_wrist = ctx.part_world_position(wrist_flange)
    with ctx.pose({shoulder: 0.85}):
        raised_forearm = ctx.part_world_position(forearm)
    with ctx.pose({elbow: 1.0}):
        folded_wrist = ctx.part_world_position(wrist_flange)
    ctx.check(
        "positive shoulder motion raises the elbow",
        rest_forearm is not None
        and raised_forearm is not None
        and raised_forearm[2] > rest_forearm[2] + 0.35,
        details=f"rest={rest_forearm}, raised={raised_forearm}",
    )
    ctx.check(
        "positive elbow motion folds the wrist upward",
        rest_wrist is not None and folded_wrist is not None and folded_wrist[2] > rest_wrist[2] + 0.25,
        details=f"rest={rest_wrist}, folded={folded_wrist}",
    )
    with ctx.pose({wrist: 1.0}):
        ctx.expect_contact(
            forearm,
            wrist_flange,
            elem_a="wrist_collar",
            elem_b="flange_disk",
            contact_tol=0.0015,
            name="wrist roll preserves flange seating",
        )

    return ctx.report()


object_model = build_object_model()
