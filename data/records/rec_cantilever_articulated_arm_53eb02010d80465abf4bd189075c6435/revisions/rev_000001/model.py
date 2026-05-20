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
    model = ArticulatedObject(name="bench_cantilever_arm")

    blue = model.material("shop_blue", rgba=(0.03, 0.18, 0.48, 1.0))
    steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.57, 1.0))
    dark = model.material("blackened_hardware", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("dark_rubber_feet", rgba=(0.03, 0.028, 0.025, 1.0))

    # All pivot axes run horizontally across the bench-tool arm, parallel to Y.
    axis_y = (0.0, -1.0, 0.0)
    shoulder_x = 0.14
    shoulder_z = 0.62
    shoulder_len = 0.56
    elbow_len = 0.46

    column = model.part("ground_column")
    column.visual(
        Box((0.36, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=steel,
        name="bench_base",
    )
    column.visual(
        Cylinder(radius=0.057, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=steel,
        name="round_column",
    )
    column.visual(
        Cylinder(radius=0.072, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=steel,
        name="top_collar",
    )
    # Cantilever shoulder yoke: two outboard cheeks with a rear bridge to the
    # column, leaving a real central gap for the moving shoulder lug.
    for sign, name in ((1.0, "shoulder_cheek_0"), (-1.0, "shoulder_cheek_1")):
        column.visual(
            Box((0.14, 0.026, 0.12)),
            origin=Origin(xyz=(shoulder_x, sign * 0.062, shoulder_z)),
            material=steel,
            name=name,
        )
        column.visual(
            Cylinder(radius=0.046, length=0.018),
            origin=Origin(
                xyz=(shoulder_x, sign * 0.080, shoulder_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"shoulder_cap_{0 if sign > 0 else 1}",
        )
    column.visual(
        Box((0.070, 0.150, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, shoulder_z)),
        material=steel,
        name="shoulder_yoke_bridge",
    )
    column.visual(
        Cylinder(radius=0.015, length=0.150),
        origin=Origin(
            xyz=(shoulder_x, 0.0, shoulder_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark,
        name="shoulder_pin",
    )
    # Bench fasteners and rubber feet make the column read as grounded.
    for ix, x in enumerate((-0.125, 0.125)):
        for iy, y in enumerate((-0.085, 0.085)):
            column.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x, y, 0.038)),
                material=dark,
                name=f"base_bolt_{ix}_{iy}",
            )
            column.visual(
                Cylinder(radius=0.022, length=0.010),
                origin=Origin(xyz=(x, y, -0.005)),
                material=rubber,
                name=f"foot_{ix}_{iy}",
            )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="shoulder_lug",
    )
    shoulder.visual(
        Box((0.465, 0.045, 0.045)),
        origin=Origin(xyz=(0.2675, 0.0, 0.0)),
        material=blue,
        name="shoulder_beam",
    )
    shoulder.visual(
        Box((0.070, 0.140, 0.045)),
        origin=Origin(xyz=(shoulder_len - 0.090, 0.0, 0.0)),
        material=blue,
        name="elbow_fork_bridge",
    )
    for sign, name in ((1.0, "elbow_cheek_0"), (-1.0, "elbow_cheek_1")):
        shoulder.visual(
            Box((0.130, 0.026, 0.120)),
            origin=Origin(xyz=(shoulder_len, sign * 0.058, 0.0)),
            material=blue,
            name=name,
        )
        shoulder.visual(
            Cylinder(radius=0.044, length=0.020),
            origin=Origin(
                xyz=(shoulder_len, sign * 0.078, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"elbow_cap_{0 if sign > 0 else 1}",
        )
    shoulder.visual(
        Cylinder(radius=0.013, length=0.136),
        origin=Origin(
            xyz=(shoulder_len, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark,
        name="elbow_pin",
    )

    elbow = model.part("elbow_link")
    elbow.visual(
        Cylinder(radius=0.040, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="elbow_lug",
    )
    elbow.visual(
        Box((0.350, 0.040, 0.040)),
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        material=blue,
        name="elbow_beam",
    )
    elbow.visual(
        Box((0.062, 0.125, 0.040)),
        origin=Origin(xyz=(elbow_len - 0.077, 0.0, 0.0)),
        material=blue,
        name="wrist_fork_bridge",
    )
    for sign, name in ((1.0, "wrist_cheek_0"), (-1.0, "wrist_cheek_1")):
        elbow.visual(
            Box((0.110, 0.024, 0.100)),
            origin=Origin(xyz=(elbow_len, sign * 0.052, 0.0)),
            material=blue,
            name=name,
        )
        elbow.visual(
            Cylinder(radius=0.037, length=0.018),
            origin=Origin(
                xyz=(elbow_len, sign * 0.069, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"wrist_cap_{0 if sign > 0 else 1}",
        )
    elbow.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(
            xyz=(elbow_len, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark,
        name="wrist_pin",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_lug",
    )
    tool_plate.visual(
        Box((0.080, 0.044, 0.034)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=steel,
        name="tool_neck",
    )
    tool_plate.visual(
        Box((0.020, 0.180, 0.130)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=steel,
        name="mounting_plate",
    )
    for iy, y in enumerate((-0.055, 0.055)):
        for iz, z in enumerate((-0.040, 0.040)):
            tool_plate.visual(
                Cylinder(radius=0.009, length=0.008),
                origin=Origin(
                    xyz=(0.116, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=dark,
                name=f"plate_bolt_{iy}_{iz}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=column,
        child=shoulder,
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z)),
        axis=axis_y,
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.95, upper=1.10),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(shoulder_len, 0.0, 0.0)),
        axis=axis_y,
        motion_limits=MotionLimits(effort=65.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=elbow,
        child=tool_plate,
        origin=Origin(xyz=(elbow_len, 0.0, 0.0)),
        axis=axis_y,
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    for joint in (shoulder, elbow, wrist):
        ctx.check(
            f"{joint.name} is a horizontal pitch joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(joint.axis[0]) < 1e-9
            and abs(abs(joint.axis[1]) - 1.0) < 1e-9
            and abs(joint.axis[2]) < 1e-9,
            details=f"axis={joint.axis}, type={joint.articulation_type}",
        )

    ctx.allow_overlap(
        "ground_column",
        "shoulder_link",
        elem_a="shoulder_pin",
        elem_b="shoulder_lug",
        reason="The shoulder shaft is intentionally captured through the rotating lug.",
    )
    ctx.allow_overlap(
        "shoulder_link",
        "elbow_link",
        elem_a="elbow_pin",
        elem_b="elbow_lug",
        reason="The elbow shaft is intentionally captured through the rotating lug.",
    )
    ctx.allow_overlap(
        "elbow_link",
        "tool_plate",
        elem_a="wrist_pin",
        elem_b="wrist_lug",
        reason="The wrist shaft is intentionally captured through the tool-plate lug.",
    )

    with ctx.pose({"shoulder": 0.0, "elbow": 0.0, "wrist": 0.0}):
        shoulder_pos = ctx.part_world_position("shoulder_link")
        elbow_pos = ctx.part_world_position("elbow_link")
        plate_pos = ctx.part_world_position("tool_plate")
        ctx.check(
            "arm projects to one side at rest",
            shoulder_pos is not None
            and elbow_pos is not None
            and plate_pos is not None
            and elbow_pos[0] > shoulder_pos[0] + 0.50
            and plate_pos[0] > elbow_pos[0] + 0.40,
            details=f"shoulder={shoulder_pos}, elbow={elbow_pos}, plate={plate_pos}",
        )
        ctx.expect_overlap(
            "ground_column",
            "shoulder_link",
            axes="xyz",
            elem_a="shoulder_pin",
            elem_b="shoulder_lug",
            min_overlap=0.020,
            name="shoulder pin runs through lug",
        )
        ctx.expect_overlap(
            "shoulder_link",
            "elbow_link",
            axes="xyz",
            elem_a="elbow_pin",
            elem_b="elbow_lug",
            min_overlap=0.020,
            name="elbow pin runs through lug",
        )
        ctx.expect_overlap(
            "elbow_link",
            "tool_plate",
            axes="xyz",
            elem_a="wrist_pin",
            elem_b="wrist_lug",
            min_overlap=0.018,
            name="wrist pin runs through lug",
        )
        ctx.expect_gap(
            "shoulder_link",
            "ground_column",
            axis="y",
            positive_elem="shoulder_lug",
            negative_elem="shoulder_cheek_1",
            min_gap=0.010,
            name="shoulder lug clears one yoke cheek",
        )
        ctx.expect_gap(
            "ground_column",
            "shoulder_link",
            axis="y",
            positive_elem="shoulder_cheek_0",
            negative_elem="shoulder_lug",
            min_gap=0.010,
            name="shoulder lug clears opposite yoke cheek",
        )
        ctx.expect_gap(
            "elbow_link",
            "shoulder_link",
            axis="y",
            positive_elem="elbow_lug",
            negative_elem="elbow_cheek_1",
            min_gap=0.010,
            name="elbow lug clears fork cheek",
        )

    rest_plate = ctx.part_world_position("tool_plate")
    with ctx.pose({"shoulder": 0.55, "elbow": 0.0, "wrist": 0.0}):
        raised_plate = ctx.part_world_position("tool_plate")
    ctx.check(
        "positive shoulder motion raises the cantilevered arm",
        rest_plate is not None and raised_plate is not None and raised_plate[2] > rest_plate[2] + 0.20,
        details=f"rest={rest_plate}, raised={raised_plate}",
    )

    return ctx.report()


object_model = build_object_model()
