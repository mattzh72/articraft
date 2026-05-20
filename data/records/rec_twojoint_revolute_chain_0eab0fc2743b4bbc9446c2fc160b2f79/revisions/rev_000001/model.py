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
    model = ArticulatedObject(name="simple_robot_arm")

    cast_aluminum = model.material("cast_aluminum", color=(0.58, 0.61, 0.63, 1.0))
    dark_bearing = model.material("dark_bearing", color=(0.04, 0.045, 0.05, 1.0))
    safety_orange = model.material("safety_orange", color=(0.95, 0.42, 0.08, 1.0))
    tool_blue = model.material("tool_blue", color=(0.10, 0.22, 0.42, 1.0))
    tool_face = model.material("tool_face", color=(0.015, 0.018, 0.022, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.22, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_aluminum,
        name="floor_disk",
    )
    base.visual(
        Cylinder(radius=0.075, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=cast_aluminum,
        name="pedestal",
    )
    for side, y in (("0", -0.0575), ("1", 0.0575)):
        base.visual(
            Box((0.13, 0.030, 0.20)),
            origin=Origin(xyz=(0.0, y, 0.30)),
            material=cast_aluminum,
            name=f"shoulder_yoke_{side}",
        )
        base.visual(
            Cylinder(radius=0.065, length=0.036),
            origin=Origin(xyz=(0.0, y * 1.55, 0.32), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_bearing,
            name=f"shoulder_bearing_{side}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.085, length=0.085),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_bearing,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.39, 0.055, 0.065)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=safety_orange,
        name="upper_link",
    )
    upper_arm.visual(
        Box((0.045, 0.170, 0.065)),
        origin=Origin(xyz=(0.435, 0.0, 0.0)),
        material=safety_orange,
        name="elbow_fork_bridge",
    )
    for side, y in (("0", -0.057), ("1", 0.057)):
        upper_arm.visual(
            Box((0.14, 0.034, 0.120)),
            origin=Origin(xyz=(0.505, y, 0.0)),
            material=safety_orange,
            name=f"elbow_fork_{side}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_bearing,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.34, 0.050, 0.055)),
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        material=safety_orange,
        name="forearm_link",
    )
    forearm.visual(
        Box((0.045, 0.100, 0.090)),
        origin=Origin(xyz=(0.3975, 0.0, 0.0)),
        material=cast_aluminum,
        name="tool_mount_plate",
    )

    tool = model.part("tool")
    tool.visual(
        Box((0.12, 0.10, 0.09)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=tool_blue,
        name="tool_block",
    )
    tool.visual(
        Box((0.008, 0.078, 0.068)),
        origin=Origin(xyz=(0.123, 0.0, 0.0)),
        material=tool_face,
        name="front_face",
    )

    shoulder = model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.5, lower=-0.75, upper=1.35),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=0.0, upper=2.15),
    )
    model.articulation(
        "tool_mount",
        ArticulationType.FIXED,
        parent=forearm,
        child=tool,
        origin=Origin(xyz=(0.42, 0.0, 0.0)),
    )

    # The shoulder variable is intentionally kept local for readability while the
    # named articulation is used by tests and probe tooling.
    _ = shoulder
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    tool_mount = object_model.get_articulation("tool_mount")

    ctx.check(
        "two user-facing revolute joints",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tool_mount.articulation_type == ArticulationType.FIXED,
        details=f"shoulder={shoulder.articulation_type}, elbow={elbow.articulation_type}, tool={tool_mount.articulation_type}",
    )
    ctx.expect_contact(
        "forearm",
        "tool",
        elem_a="tool_mount_plate",
        elem_b="tool_block",
        contact_tol=0.001,
        name="fixed tool block seats on forearm mount",
    )

    rest_tool = ctx.part_world_position("tool")
    with ctx.pose({shoulder: 0.70, elbow: 1.10}):
        raised_tool = ctx.part_world_position("tool")
    ctx.check(
        "shoulder and elbow move the tool",
        rest_tool is not None
        and raised_tool is not None
        and raised_tool[2] > rest_tool[2] + 0.20,
        details=f"rest={rest_tool}, raised={raised_tool}",
    )

    return ctx.report()


object_model = build_object_model()
