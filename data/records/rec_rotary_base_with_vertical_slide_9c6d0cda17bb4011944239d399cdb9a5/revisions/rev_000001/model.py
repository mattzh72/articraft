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
    model = ArticulatedObject(name="rotary_lift_module")

    charcoal = model.material("charcoal_powdercoat", rgba=(0.05, 0.055, 0.06, 1.0))
    dark = model.material("black_anodized", rgba=(0.01, 0.012, 0.014, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    rail_steel = model.material("polished_rail_steel", rgba=(0.82, 0.84, 0.82, 1.0))
    blue = model.material("blue_carriage", rgba=(0.05, 0.18, 0.34, 1.0))
    brass = model.material("bronze_bearing", rgba=(0.72, 0.48, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.38, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=charcoal,
        name="square_plinth",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=dark,
        name="round_pedestal",
    )
    base.visual(
        Cylinder(radius=0.135, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=steel,
        name="lower_bearing_race",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=0.145, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="turntable_disk",
    )
    rotary_stage.visual(
        Box((0.24, 0.22, 0.025)),
        origin=Origin(xyz=(0.0, -0.005, 0.0405)),
        material=charcoal,
        name="slide_mount_plate",
    )
    rotary_stage.visual(
        Box((0.17, 0.055, 0.044)),
        origin=Origin(xyz=(0.0, -0.035, 0.075)),
        material=dark,
        name="lower_rail_block",
    )
    rotary_stage.visual(
        Cylinder(radius=0.010, length=0.462),
        origin=Origin(xyz=(-0.055, -0.035, 0.283)),
        material=rail_steel,
        name="guide_rail_0",
    )
    rotary_stage.visual(
        Cylinder(radius=0.010, length=0.462),
        origin=Origin(xyz=(0.055, -0.035, 0.283)),
        material=rail_steel,
        name="guide_rail_1",
    )
    rotary_stage.visual(
        Cylinder(radius=0.007, length=0.420),
        origin=Origin(xyz=(0.0, -0.035, 0.303)),
        material=steel,
        name="lift_screw",
    )
    rotary_stage.visual(
        Box((0.038, 0.035, 0.462)),
        origin=Origin(xyz=(0.0, 0.037, 0.283)),
        material=dark,
        name="rear_spine",
    )
    rotary_stage.visual(
        Box((0.17, 0.055, 0.036)),
        origin=Origin(xyz=(0.0, -0.035, 0.532)),
        material=dark,
        name="top_rail_block",
    )

    lift_carriage = model.part("lift_carriage")
    lift_carriage.visual(
        Box((0.17, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, -0.085, 0.0)),
        material=blue,
        name="front_tool_plate",
    )
    lift_carriage.visual(
        Box((0.035, 0.031, 0.070)),
        origin=Origin(xyz=(-0.055, -0.0605, 0.0)),
        material=brass,
        name="bearing_shoe_0",
    )
    lift_carriage.visual(
        Box((0.035, 0.031, 0.070)),
        origin=Origin(xyz=(0.055, -0.0605, 0.0)),
        material=brass,
        name="bearing_shoe_1",
    )
    lift_carriage.visual(
        Box((0.115, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.073, 0.042)),
        material=blue,
        name="upper_bridge",
    )
    lift_carriage.visual(
        Box((0.115, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.073, -0.042)),
        material=blue,
        name="lower_bridge",
    )
    for x in (-0.055, 0.055):
        for z in (-0.027, 0.027):
            suffix = f"{0 if x < 0 else 1}_{0 if z < 0 else 1}"
            lift_carriage.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, -0.097, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"face_bolt_{suffix}",
            )

    model.articulation(
        "yaw_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
    )
    model.articulation(
        "lift_axis",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=lift_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotary_stage = object_model.get_part("rotary_stage")
    lift_carriage = object_model.get_part("lift_carriage")
    yaw_axis = object_model.get_articulation("yaw_axis")
    lift_axis = object_model.get_articulation("lift_axis")

    ctx.check(
        "module exposes exactly yaw and lift joints",
        {joint.name for joint in object_model.articulations} == {"yaw_axis", "lift_axis"},
    )
    ctx.check(
        "yaw is a vertical continuous rotary axis",
        yaw_axis.articulation_type == ArticulationType.CONTINUOUS and yaw_axis.axis == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "lift is a bounded vertical prismatic axis",
        lift_axis.articulation_type == ArticulationType.PRISMATIC
        and lift_axis.axis == (0.0, 0.0, 1.0)
        and lift_axis.motion_limits is not None
        and lift_axis.motion_limits.lower == 0.0
        and lift_axis.motion_limits.upper == 0.22,
    )

    ctx.expect_gap(
        rotary_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_disk",
        negative_elem="lower_bearing_race",
        name="turntable disk rests on bearing race",
    )
    ctx.expect_overlap(
        rotary_stage,
        base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="lower_bearing_race",
        min_overlap=0.20,
        name="rotary bearing footprints are concentric",
    )
    ctx.expect_overlap(
        lift_carriage,
        rotary_stage,
        axes="z",
        elem_a="bearing_shoe_0",
        elem_b="guide_rail_0",
        min_overlap=0.060,
        name="lowered carriage remains engaged with rail",
    )
    ctx.expect_gap(
        rotary_stage,
        lift_carriage,
        axis="y",
        positive_elem="guide_rail_0",
        negative_elem="bearing_shoe_0",
        max_gap=0.001,
        max_penetration=0.0001,
        name="carriage shoe runs against the guide rail",
    )

    rest_pos = ctx.part_world_position(lift_carriage)
    with ctx.pose({lift_axis: 0.22, yaw_axis: math.pi / 2.0}):
        ctx.expect_overlap(
            lift_carriage,
            rotary_stage,
            axes="z",
            elem_a="bearing_shoe_1",
            elem_b="guide_rail_1",
            min_overlap=0.060,
            name="raised carriage remains engaged with rail",
        )
        ctx.expect_gap(
            lift_carriage,
            rotary_stage,
            axis="z",
            positive_elem="front_tool_plate",
            negative_elem="lower_rail_block",
            min_gap=0.22,
            name="lifted carriage clears lower stop",
        )
        raised_pos = ctx.part_world_position(lift_carriage)

    ctx.check(
        "prismatic joint raises the carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
