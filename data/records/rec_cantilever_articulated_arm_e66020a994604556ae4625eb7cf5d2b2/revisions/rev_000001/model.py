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
    model = ArticulatedObject(name="compact_service_cantilever_arm")

    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("column_base")
    base.visual(
        Box((0.46, 0.32, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.404),
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
        material=satin_steel,
        name="column_tube",
    )
    base.visual(
        Cylinder(radius=0.094, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.418)),
        material=dark_steel,
        name="top_collar",
    )
    base.visual(
        Box((0.165, 0.178, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.408)),
        material=dark_steel,
        name="shoulder_saddle",
    )
    base.visual(
        Box((0.124, 0.024, 0.136)),
        origin=Origin(xyz=(0.0, 0.066, 0.486)),
        material=safety_yellow,
        name="shoulder_cheek_0",
    )
    base.visual(
        Box((0.124, 0.024, 0.136)),
        origin=Origin(xyz=(0.0, -0.066, 0.486)),
        material=safety_yellow,
        name="shoulder_cheek_1",
    )
    for i, y in enumerate((0.086, -0.086)):
        base.visual(
            Cylinder(radius=0.044, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.486), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name=f"shoulder_pin_boss_{i}",
        )
    for i, (x, y) in enumerate(((0.17, 0.11), (0.17, -0.11), (-0.17, 0.11), (-0.17, -0.11))):
        base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.050)),
            material=black,
            name=f"anchor_bolt_{i}",
        )

    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=0.039, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_trunnion",
    )
    upper.visual(
        Box((0.408, 0.064, 0.056)),
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        material=safety_yellow,
        name="upper_box_beam",
    )
    upper.visual(
        Box((0.072, 0.126, 0.058)),
        origin=Origin(xyz=(0.404, 0.0, 0.0)),
        material=safety_yellow,
        name="elbow_fork_bridge",
    )
    upper.visual(
        Box((0.104, 0.024, 0.104)),
        origin=Origin(xyz=(0.480, 0.062, 0.0)),
        material=safety_yellow,
        name="elbow_cheek_0",
    )
    upper.visual(
        Box((0.104, 0.024, 0.104)),
        origin=Origin(xyz=(0.480, -0.062, 0.0)),
        material=safety_yellow,
        name="elbow_cheek_1",
    )
    for i, y in enumerate((0.081, -0.081)):
        upper.visual(
            Cylinder(radius=0.037, length=0.014),
            origin=Origin(xyz=(0.480, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name=f"elbow_pin_boss_{i}",
        )

    fore = model.part("forelink")
    fore.visual(
        Cylinder(radius=0.032, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_trunnion",
    )
    fore.visual(
        Box((0.300, 0.054, 0.050)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=safety_yellow,
        name="fore_box_beam",
    )
    fore.visual(
        Box((0.072, 0.104, 0.052)),
        origin=Origin(xyz=(0.304, 0.0, 0.0)),
        material=safety_yellow,
        name="wrist_fork_bridge",
    )
    fore.visual(
        Box((0.088, 0.018, 0.090)),
        origin=Origin(xyz=(0.380, 0.046, 0.0)),
        material=safety_yellow,
        name="wrist_cheek_0",
    )
    fore.visual(
        Box((0.088, 0.018, 0.090)),
        origin=Origin(xyz=(0.380, -0.046, 0.0)),
        material=safety_yellow,
        name="wrist_cheek_1",
    )
    for i, y in enumerate((0.061, -0.061)):
        fore.visual(
            Cylinder(radius=0.031, length=0.012),
            origin=Origin(xyz=(0.380, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name=f"wrist_pin_boss_{i}",
        )

    tool = model.part("tool_bracket")
    tool.visual(
        Cylinder(radius=0.026, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_trunnion",
    )
    tool.visual(
        Box((0.118, 0.046, 0.042)),
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
        material=dark_steel,
        name="short_tool_arm",
    )
    tool.visual(
        Box((0.032, 0.092, 0.126)),
        origin=Origin(xyz=(0.136, 0.0, -0.034)),
        material=dark_steel,
        name="tool_mount_plate",
    )
    for i, z in enumerate((0.010, -0.078)):
        tool.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.154, 0.026, z), rpy=(0.0, pi / 2, 0.0)),
            material=black,
            name=f"tool_bolt_{i}",
        )
        tool.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.154, -0.026, z), rpy=(0.0, pi / 2, 0.0)),
            material=black,
            name=f"tool_bolt_{i + 2}",
        )

    model.articulation(
        "shoulder_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.486)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=-0.45, upper=1.15),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=fore,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.8, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist_pivot",
        ArticulationType.REVOLUTE,
        parent=fore,
        child=tool,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.4, lower=-1.65, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder_pivot")
    elbow = object_model.get_articulation("elbow_pivot")
    wrist = object_model.get_articulation("wrist_pivot")

    ctx.check(
        "three revolute pivots",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, wrist)),
        details="The service arm must articulate at shoulder, elbow, and wrist revolute pivots.",
    )
    ctx.expect_within(
        "upper_link",
        "column_base",
        axes="y",
        inner_elem="shoulder_trunnion",
        outer_elem="shoulder_saddle",
        margin=0.010,
        name="shoulder trunnion sits inside base yoke span",
    )
    ctx.expect_within(
        "forelink",
        "upper_link",
        axes="y",
        inner_elem="elbow_trunnion",
        outer_elem="elbow_fork_bridge",
        margin=0.010,
        name="elbow trunnion sits inside upper fork span",
    )
    ctx.expect_within(
        "tool_bracket",
        "forelink",
        axes="y",
        inner_elem="wrist_trunnion",
        outer_elem="wrist_fork_bridge",
        margin=0.010,
        name="wrist trunnion sits inside forelink fork span",
    )

    tool = object_model.get_part("tool_bracket")
    rest_tip = ctx.part_world_position(tool)
    with ctx.pose({shoulder: 0.70}):
        raised_tip = ctx.part_world_position(tool)
    ctx.check(
        "shoulder lift raises terminal bracket",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.18,
        details=f"rest={rest_tip}, raised={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
