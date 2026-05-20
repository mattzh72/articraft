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
    model = ArticulatedObject(name="industrial_inspection_arm")

    safety_yellow = model.material("safety_yellow", color=(0.95, 0.68, 0.08, 1.0))
    graphite = model.material("graphite", color=(0.08, 0.085, 0.09, 1.0))
    dark_grey = model.material("dark_grey", color=(0.22, 0.23, 0.24, 1.0))
    steel = model.material("brushed_steel", color=(0.62, 0.63, 0.60, 1.0))
    black = model.material("black_rubber", color=(0.01, 0.012, 0.014, 1.0))
    lens_blue = model.material("coated_lens", color=(0.05, 0.25, 0.75, 0.85))

    base = model.part("base")
    base.visual(
        Box((0.58, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=graphite,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.12, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=dark_grey,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.6575)),
        material=steel,
        name="top_bearing",
    )
    for x in (-0.21, 0.21):
        for y in (-0.13, 0.13):
            base.visual(
                Cylinder(radius=0.022, length=0.012),
                origin=Origin(xyz=(x, y, 0.045)),
                material=steel,
                name=f"anchor_bolt_{x}_{y}",
            )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.155, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_rotor",
    )
    shoulder.visual(
        Cylinder(radius=0.105, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=dark_grey,
        name="shoulder_hub",
    )

    link_len = 0.72
    link_y = 0.145
    link_z = 0.11
    wall = 0.018
    link_center_z = 0.035
    link_center_x = 0.39
    shoulder.visual(
        Box((link_len, link_y, wall)),
        origin=Origin(xyz=(link_center_x, 0.0, link_center_z + link_z / 2 - wall / 2)),
        material=safety_yellow,
        name="upper_tube_top",
    )
    shoulder.visual(
        Box((link_len, link_y, wall)),
        origin=Origin(xyz=(link_center_x, 0.0, link_center_z - link_z / 2 + wall / 2)),
        material=safety_yellow,
        name="upper_tube_bottom",
    )
    shoulder.visual(
        Box((link_len, wall, link_z - 2 * wall)),
        origin=Origin(xyz=(link_center_x, link_y / 2 - wall / 2, link_center_z)),
        material=safety_yellow,
        name="upper_tube_side_0",
    )
    shoulder.visual(
        Box((link_len, wall, link_z - 2 * wall)),
        origin=Origin(xyz=(link_center_x, -link_y / 2 + wall / 2, link_center_z)),
        material=safety_yellow,
        name="upper_tube_side_1",
    )
    shoulder.visual(
        Box((0.055, 0.145, 0.115)),
        origin=Origin(xyz=(0.06, 0.0, 0.045)),
        material=safety_yellow,
        name="shoulder_bulkhead",
    )
    shoulder.visual(
        Cylinder(radius=0.135, length=0.04),
        origin=Origin(xyz=(0.78, 0.0, 0.07)),
        material=steel,
        name="elbow_bearing",
    )
    shoulder.visual(
        Box((0.065, 0.165, 0.10)),
        origin=Origin(xyz=(0.735, 0.0, 0.035)),
        material=safety_yellow,
        name="elbow_bulkhead",
    )
    for x in (0.20, 0.42, 0.64):
        for y in (-0.045, 0.045):
            shoulder.visual(
                Cylinder(radius=0.012, length=0.012),
                origin=Origin(xyz=(x, y, link_center_z + link_z / 2 + 0.005)),
                material=steel,
                name=f"upper_bolt_{x}_{y}",
            )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.122, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=steel,
        name="elbow_rotor",
    )
    forearm.visual(
        Cylinder(radius=0.078, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=dark_grey,
        name="elbow_hub",
    )
    fore_len = 0.68
    fore_y = 0.13
    fore_z = 0.10
    fore_center_x = 0.39
    fore_center_z = 0.14
    forearm.visual(
        Box((fore_len, fore_y, wall)),
        origin=Origin(xyz=(fore_center_x, 0.0, fore_center_z + fore_z / 2 - wall / 2)),
        material=safety_yellow,
        name="fore_tube_top",
    )
    forearm.visual(
        Box((fore_len, fore_y, wall)),
        origin=Origin(xyz=(fore_center_x, 0.0, fore_center_z - fore_z / 2 + wall / 2)),
        material=safety_yellow,
        name="fore_tube_bottom",
    )
    forearm.visual(
        Box((fore_len, wall, fore_z - 2 * wall)),
        origin=Origin(xyz=(fore_center_x, fore_y / 2 - wall / 2, fore_center_z)),
        material=safety_yellow,
        name="fore_tube_side_0",
    )
    forearm.visual(
        Box((fore_len, wall, fore_z - 2 * wall)),
        origin=Origin(xyz=(fore_center_x, -fore_y / 2 + wall / 2, fore_center_z)),
        material=safety_yellow,
        name="fore_tube_side_1",
    )
    forearm.visual(
        Box((0.07, 0.16, 0.115)),
        origin=Origin(xyz=(0.075, 0.0, 0.155)),
        material=safety_yellow,
        name="fore_bulkhead",
    )
    forearm.visual(
        Box((0.66, 0.018, 0.018)),
        origin=Origin(xyz=(0.38, 0.045, 0.199)),
        material=steel,
        name="rail_0",
    )
    forearm.visual(
        Box((0.66, 0.018, 0.018)),
        origin=Origin(xyz=(0.38, -0.045, 0.199)),
        material=steel,
        name="rail_1",
    )
    forearm.visual(
        Box((0.18, 0.17, 0.018)),
        origin=Origin(xyz=(0.61, 0.0, 0.211)),
        material=dark_grey,
        name="travel_stop",
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        Box((0.075, 0.038, 0.036)),
        origin=Origin(xyz=(0.01, 0.045, 0.226)),
        material=dark_grey,
        name="carriage_block_0",
    )
    tool_slide.visual(
        Box((0.075, 0.038, 0.036)),
        origin=Origin(xyz=(0.01, -0.045, 0.226)),
        material=dark_grey,
        name="carriage_block_1",
    )
    tool_slide.visual(
        Box((0.075, 0.038, 0.036)),
        origin=Origin(xyz=(0.13, 0.045, 0.226)),
        material=dark_grey,
        name="carriage_block_2",
    )
    tool_slide.visual(
        Box((0.075, 0.038, 0.036)),
        origin=Origin(xyz=(0.13, -0.045, 0.226)),
        material=dark_grey,
        name="carriage_block_3",
    )
    tool_slide.visual(
        Box((0.20, 0.145, 0.020)),
        origin=Origin(xyz=(0.07, 0.0, 0.252)),
        material=dark_grey,
        name="carriage_bridge",
    )
    tool_slide.visual(
        Box((0.42, 0.060, 0.055)),
        origin=Origin(xyz=(0.285, 0.0, 0.285)),
        material=safety_yellow,
        name="extension_tube",
    )
    tool_slide.visual(
        Box((0.11, 0.095, 0.085)),
        origin=Origin(xyz=(0.545, 0.0, 0.285)),
        material=graphite,
        name="sensor_head",
    )
    tool_slide.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.615, 0.0, 0.285), rpy=(0.0, math.pi / 2, 0.0)),
        material=lens_blue,
        name="optical_lens",
    )
    tool_slide.visual(
        Cylinder(radius=0.010, length=0.16),
        origin=Origin(xyz=(0.705, 0.0, 0.285), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="probe_stylus",
    )
    tool_slide.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.794, 0.0, 0.285)),
        material=steel,
        name="probe_tip",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-3.05, upper=3.05),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.78, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.4, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "tool_slide",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=tool_slide,
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.30, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder")
    forearm = object_model.get_part("forearm")
    tool_slide = object_model.get_part("tool_slide")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    slide_joint = object_model.get_articulation("tool_slide")

    ctx.check(
        "three primary axes are articulated",
        len(object_model.articulations) == 3
        and shoulder_yaw.articulation_type == ArticulationType.REVOLUTE
        and elbow_yaw.articulation_type == ArticulationType.REVOLUTE
        and slide_joint.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.expect_gap(
        shoulder,
        base,
        axis="z",
        positive_elem="shoulder_rotor",
        negative_elem="top_bearing",
        max_penetration=0.000001,
        max_gap=0.0015,
        name="shoulder rotary bearing is seated",
    )
    ctx.expect_gap(
        forearm,
        shoulder,
        axis="z",
        positive_elem="elbow_rotor",
        negative_elem="elbow_bearing",
        max_penetration=0.000001,
        max_gap=0.0015,
        name="elbow rotary bearing is seated",
    )
    ctx.expect_gap(
        tool_slide,
        forearm,
        axis="z",
        positive_elem="carriage_block_0",
        negative_elem="rail_0",
        max_penetration=0.000001,
        max_gap=0.0015,
        name="linear carriage rides on guide rail",
    )
    ctx.expect_overlap(
        tool_slide,
        forearm,
        axes="x",
        elem_a="carriage_block_2",
        elem_b="rail_0",
        min_overlap=0.07,
        name="resting carriage remains on rail",
    )

    rest_tip = ctx.part_element_world_aabb(tool_slide, elem="probe_tip")
    with ctx.pose({slide_joint: 0.20}):
        extended_tip = ctx.part_element_world_aabb(tool_slide, elem="probe_tip")
        ctx.expect_overlap(
            tool_slide,
            forearm,
            axes="x",
            elem_a="carriage_block_2",
            elem_b="rail_0",
            min_overlap=0.035,
            name="extended carriage remains on rail",
        )
    ctx.check(
        "prismatic tool extension moves outward",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[1][0] > rest_tip[1][0] + 0.18,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
