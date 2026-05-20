from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_module")

    dark_steel = Material("dark_burnished_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_aluminum = Material("satin_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    bearing_black = Material("black_bearing_rubber", rgba=(0.025, 0.025, 0.028, 1.0))
    safety_orange = Material("safety_orange_plate", rgba=(0.95, 0.36, 0.08, 1.0))

    ground_base = model.part("ground_base")
    ground_base.visual(
        Cylinder(radius=0.25, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="floor_disc",
    )
    ground_base.visual(
        Cylinder(radius=0.17, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=bearing_black,
        name="fixed_bearing_race",
    )

    yaw_fork = model.part("yaw_fork")
    yaw_fork.visual(
        Cylinder(radius=0.185, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=satin_aluminum,
        name="turntable_disc",
    )
    yaw_fork.visual(
        Cylinder(radius=0.085, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=satin_aluminum,
        name="center_boss",
    )
    yaw_fork.visual(
        Box((0.25, 0.34, 0.064)),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=satin_aluminum,
        name="fork_crossbar",
    )
    yaw_fork.visual(
        Box((0.065, 0.050, 0.335)),
        origin=Origin(xyz=(0.0, 0.1475, 0.303)),
        material=satin_aluminum,
        name="fork_arm_0",
    )
    yaw_fork.visual(
        Box((0.065, 0.050, 0.335)),
        origin=Origin(xyz=(0.0, -0.1475, 0.303)),
        material=satin_aluminum,
        name="fork_arm_1",
    )
    yaw_fork.visual(
        Cylinder(radius=0.048, length=0.025),
        origin=Origin(xyz=(0.0, 0.1335, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_black,
        name="bearing_0",
    )
    yaw_fork.visual(
        Cylinder(radius=0.048, length=0.025),
        origin=Origin(xyz=(0.0, -0.1335, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_black,
        name="bearing_1",
    )
    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Box((0.170, 0.180, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="cradle_block",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.022, length=0.031),
        origin=Origin(xyz=(0.0, 0.1055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_0",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.022, length=0.031),
        origin=Origin(xyz=(0.0, -0.1055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_1",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.040, length=0.037),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=satin_aluminum,
        name="top_pedestal",
    )
    pitch_cradle.visual(
        Box((0.150, 0.120, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0865)),
        material=safety_orange,
        name="top_plate",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=ground_base,
        child=yaw_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_fork,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_base = object_model.get_part("ground_base")
    yaw_fork = object_model.get_part("yaw_fork")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_axis = object_model.get_articulation("yaw_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    ctx.check(
        "yaw joint is a vertical revolute",
        yaw_axis.articulation_type == ArticulationType.REVOLUTE and tuple(yaw_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw_axis.articulation_type}, axis={yaw_axis.axis}",
    )
    ctx.check(
        "pitch joint is a horizontal revolute",
        pitch_axis.articulation_type == ArticulationType.REVOLUTE and tuple(pitch_axis.axis) == (0.0, 1.0, 0.0),
        details=f"type={pitch_axis.articulation_type}, axis={pitch_axis.axis}",
    )

    ctx.expect_gap(
        yaw_fork,
        ground_base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_disc",
        negative_elem="fixed_bearing_race",
        name="turntable bears on the grounded race",
    )
    ctx.expect_contact(
        pitch_cradle,
        yaw_fork,
        elem_a="trunnion_0",
        elem_b="bearing_0",
        contact_tol=0.0015,
        name="one trunnion is carried by its fork bearing",
    )
    ctx.expect_contact(
        pitch_cradle,
        yaw_fork,
        elem_a="trunnion_1",
        elem_b="bearing_1",
        contact_tol=0.0015,
        name="opposite trunnion is carried by its fork bearing",
    )
    ctx.expect_within(
        pitch_cradle,
        yaw_fork,
        axes="y",
        inner_elem="cradle_block",
        outer_elem="fork_crossbar",
        margin=0.0,
        name="cradle body sits centered inside the fork span",
    )

    rest_plate = ctx.part_element_world_aabb(pitch_cradle, elem="top_plate")
    with ctx.pose({pitch_axis: 0.55}):
        tilted_plate = ctx.part_element_world_aabb(pitch_cradle, elem="top_plate")
        ctx.expect_gap(
            pitch_cradle,
            yaw_fork,
            axis="z",
            min_gap=0.11,
            positive_elem="cradle_block",
            negative_elem="fork_crossbar",
            name="pitched cradle clears the lower fork bridge",
        )
    rest_z_span = rest_plate[1][2] - rest_plate[0][2] if rest_plate else 0.0
    tilted_z_span = tilted_plate[1][2] - tilted_plate[0][2] if tilted_plate else 0.0
    ctx.check(
        "top plate visibly pitches about the fork axis",
        tilted_z_span > rest_z_span + 0.050,
        details=f"rest_z_span={rest_z_span}, tilted_z_span={tilted_z_span}",
    )

    return ctx.report()


object_model = build_object_model()
