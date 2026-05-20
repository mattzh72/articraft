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


STEEL_RPY_Y = (-math.pi / 2.0, 0.0, 0.0)
STEEL_RPY_X = (0.0, math.pi / 2.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_cantilever_support_arm")

    model.material("paint_dark", color=(0.10, 0.11, 0.12, 1.0))
    model.material("paint_grey", color=(0.33, 0.36, 0.38, 1.0))
    model.material("paint_orange", color=(0.95, 0.42, 0.08, 1.0))
    model.material("brushed_steel", color=(0.70, 0.72, 0.70, 1.0))
    model.material("black_oxide", color=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base_column")
    base.visual(
        Box((0.32, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="paint_dark",
        name="bench_foot",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material="paint_dark",
        name="column_tube",
    )
    base.visual(
        Box((0.180, 0.190, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material="paint_dark",
        name="top_saddle",
    )
    base.visual(
        Box((0.150, 0.024, 0.160)),
        origin=Origin(xyz=(0.035, 0.065, 0.740)),
        material="paint_dark",
        name="shoulder_cheek_0",
    )
    base.visual(
        Box((0.150, 0.024, 0.160)),
        origin=Origin(xyz=(0.035, -0.065, 0.740)),
        material="paint_dark",
        name="shoulder_cheek_1",
    )
    base.visual(
        Box((0.030, 0.154, 0.145)),
        origin=Origin(xyz=(-0.055, 0.0, 0.735)),
        material="paint_dark",
        name="shoulder_bridge",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.180),
        origin=Origin(xyz=(0.035, 0.0, 0.740), rpy=STEEL_RPY_Y),
        material="brushed_steel",
        name="shoulder_pin",
    )
    for i, (x, y) in enumerate(
        ((-0.115, -0.075), (-0.115, 0.075), (0.115, -0.075), (0.115, 0.075))
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 0.043)),
            material="black_oxide",
            name=f"mount_bolt_{i}",
        )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=0.038, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=STEEL_RPY_Y),
        material="paint_grey",
        name="root_boss",
    )
    shoulder.visual(
        Box((0.420, 0.050, 0.050)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material="paint_grey",
        name="main_beam",
    )
    shoulder.visual(
        Box((0.042, 0.130, 0.092)),
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        material="paint_grey",
        name="elbow_bridge",
    )
    shoulder.visual(
        Box((0.130, 0.022, 0.140)),
        origin=Origin(xyz=(0.480, 0.060, 0.0)),
        material="paint_grey",
        name="elbow_cheek_0",
    )
    shoulder.visual(
        Box((0.130, 0.022, 0.140)),
        origin=Origin(xyz=(0.480, -0.060, 0.0)),
        material="paint_grey",
        name="elbow_cheek_1",
    )
    shoulder.visual(
        Cylinder(radius=0.011, length=0.165),
        origin=Origin(xyz=(0.480, 0.0, 0.0), rpy=STEEL_RPY_Y),
        material="brushed_steel",
        name="elbow_pin",
    )

    elbow = model.part("elbow_link")
    elbow.visual(
        Cylinder(radius=0.036, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=STEEL_RPY_Y),
        material="paint_grey",
        name="root_boss",
    )
    elbow.visual(
        Box((0.315, 0.048, 0.048)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material="paint_grey",
        name="main_beam",
    )
    elbow.visual(
        Box((0.014, 0.144, 0.120)),
        origin=Origin(xyz=(0.357, 0.0, 0.0)),
        material="paint_grey",
        name="wrist_rear_plate",
    )
    elbow.visual(
        Box((0.014, 0.144, 0.120)),
        origin=Origin(xyz=(0.403, 0.0, 0.0)),
        material="paint_grey",
        name="wrist_front_plate",
    )
    elbow.visual(
        Box((0.082, 0.016, 0.120)),
        origin=Origin(xyz=(0.380, 0.080, 0.0)),
        material="paint_grey",
        name="wrist_side_rail_0",
    )
    elbow.visual(
        Box((0.082, 0.016, 0.120)),
        origin=Origin(xyz=(0.380, -0.080, 0.0)),
        material="paint_grey",
        name="wrist_side_rail_1",
    )
    elbow.visual(
        Cylinder(radius=0.009, length=0.104),
        origin=Origin(xyz=(0.380, 0.0, 0.0), rpy=STEEL_RPY_X),
        material="brushed_steel",
        name="wrist_pin",
    )

    wrist = model.part("wrist_plate")
    wrist.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=STEEL_RPY_X),
        material="brushed_steel",
        name="spindle_hub",
    )
    wrist.visual(
        Cylinder(radius=0.030, length=0.044),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=STEEL_RPY_X),
        material="paint_orange",
        name="front_boss",
    )
    wrist.visual(
        Box((0.014, 0.120, 0.090)),
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        material="paint_orange",
        name="tool_plate",
    )
    for i, (y, z) in enumerate(
        ((-0.038, -0.027), (-0.038, 0.027), (0.038, -0.027), (0.038, 0.027))
    ):
        wrist.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.071, y, z), rpy=STEEL_RPY_X),
            material="black_oxide",
            name=f"tool_bolt_{i}",
        )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.035, 0.0, 0.740)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.85, upper=1.15),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=elbow,
        child=wrist,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    wrist = object_model.get_part("wrist_plate")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    ctx.allow_overlap(
        base,
        shoulder,
        elem_a="shoulder_pin",
        elem_b="root_boss",
        reason="The steel shoulder pin is intentionally captured through the shoulder boss bore proxy.",
    )
    ctx.allow_overlap(
        shoulder,
        elbow,
        elem_a="elbow_pin",
        elem_b="root_boss",
        reason="The elbow pin is intentionally modeled through the elbow boss bore proxy.",
    )
    ctx.allow_overlap(
        elbow,
        wrist,
        elem_a="wrist_pin",
        elem_b="spindle_hub",
        reason="The wrist pin is intentionally captured inside the rotating spindle hub.",
    )
    ctx.allow_overlap(
        elbow,
        wrist,
        elem_a="wrist_pin",
        elem_b="front_boss",
        reason="The wrist retaining pin continues through the front boss bore proxy.",
    )
    ctx.allow_overlap(
        elbow,
        wrist,
        elem_a="wrist_front_plate",
        elem_b="front_boss",
        reason="The rotating front boss is intentionally seated through the front clevis bearing opening.",
    )
    ctx.allow_overlap(
        elbow,
        wrist,
        elem_a="wrist_rear_plate",
        elem_b="spindle_hub",
        reason="The spindle hub is intentionally carried through the rear clevis bearing opening.",
    )
    ctx.allow_overlap(
        elbow,
        wrist,
        elem_a="wrist_front_plate",
        elem_b="spindle_hub",
        reason="The spindle hub is intentionally carried through the front clevis bearing opening.",
    )

    ctx.check(
        "shoulder and elbow axes are parallel horizontal",
        shoulder_joint.axis == elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder_joint.axis}, elbow={elbow_joint.axis}",
    )
    ctx.check(
        "wrist axis follows the tool spindle",
        wrist_joint.axis == (1.0, 0.0, 0.0),
        details=f"wrist={wrist_joint.axis}",
    )

    ctx.expect_within(
        base,
        shoulder,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="root_boss",
        margin=0.002,
        name="shoulder pin runs through boss",
    )
    ctx.expect_overlap(
        base,
        shoulder,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="root_boss",
        min_overlap=0.060,
        name="shoulder boss is retained between clevis cheeks",
    )
    ctx.expect_within(
        shoulder,
        elbow,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="root_boss",
        margin=0.002,
        name="elbow pin runs through boss",
    )
    ctx.expect_overlap(
        shoulder,
        elbow,
        axes="y",
        elem_a="elbow_pin",
        elem_b="root_boss",
        min_overlap=0.055,
        name="elbow boss is retained between clevis cheeks",
    )
    ctx.expect_within(
        elbow,
        wrist,
        axes="yz",
        inner_elem="wrist_pin",
        outer_elem="spindle_hub",
        margin=0.001,
        name="wrist pin runs through spindle hub",
    )
    ctx.expect_within(
        elbow,
        wrist,
        axes="yz",
        inner_elem="wrist_pin",
        outer_elem="front_boss",
        margin=0.001,
        name="wrist pin stays centered in front boss",
    )
    ctx.expect_within(
        wrist,
        elbow,
        axes="yz",
        inner_elem="front_boss",
        outer_elem="wrist_front_plate",
        margin=0.001,
        name="front boss is centered in front clevis bearing",
    )
    ctx.expect_within(
        wrist,
        elbow,
        axes="yz",
        inner_elem="spindle_hub",
        outer_elem="wrist_rear_plate",
        margin=0.001,
        name="spindle hub is centered in rear clevis bearing",
    )
    ctx.expect_within(
        wrist,
        elbow,
        axes="yz",
        inner_elem="spindle_hub",
        outer_elem="wrist_front_plate",
        margin=0.001,
        name="spindle hub is centered in front clevis bearing",
    )

    rest_z = ctx.part_world_position(wrist)[2]
    with ctx.pose({shoulder_joint: 0.55, elbow_joint: 0.45, wrist_joint: 1.2}):
        raised_z = ctx.part_world_position(wrist)[2]
    ctx.check(
        "arm raises wrist at positive shoulder and elbow",
        raised_z > rest_z + 0.12,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
