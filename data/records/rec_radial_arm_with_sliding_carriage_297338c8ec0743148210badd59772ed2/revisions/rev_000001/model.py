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
    model = ArticulatedObject(name="turret_carriage_radial_arm")

    painted_steel = model.material("painted_steel", color=(0.17, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", color=(0.05, 0.055, 0.06, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    safety_yellow = model.material("safety_yellow", color=(0.95, 0.67, 0.08, 1.0))
    rubber = model.material("rubber_black", color=(0.015, 0.015, 0.014, 1.0))

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.34, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="floor_plate",
    )
    column.visual(
        Cylinder(radius=0.105, length=1.055),
        origin=Origin(xyz=(0.0, 0.0, 0.5725)),
        material=painted_steel,
        name="upright_tube",
    )
    column.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 1.1175)),
        material=brushed_steel,
        name="top_bearing_flange",
    )
    column.visual(
        Cylinder(radius=0.205, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=brushed_steel,
        name="base_collar",
    )
    for i, (x, y) in enumerate(
        ((0.22, 0.22), (-0.22, 0.22), (-0.22, -0.22), (0.22, -0.22))
    ):
        column.visual(
            Cylinder(radius=0.025, length=0.012),
            origin=Origin(xyz=(x, y, 0.051)),
            material=brushed_steel,
            name=f"anchor_bolt_{i}",
        )

    beam = model.part("beam")
    beam.visual(
        Cylinder(radius=0.198, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=brushed_steel,
        name="turntable_disk",
    )
    beam.visual(
        Cylinder(radius=0.13, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=painted_steel,
        name="turret_drum",
    )
    beam.visual(
        Box((1.42, 0.13, 0.12)),
        origin=Origin(xyz=(0.80, 0.0, 0.20)),
        material=safety_yellow,
        name="arm_tube",
    )
    beam.visual(
        Box((0.36, 0.12, 0.10)),
        origin=Origin(xyz=(-0.17, 0.0, 0.19)),
        material=painted_steel,
        name="counterweight_stub",
    )
    beam.visual(
        Box((1.25, 0.008, 0.034)),
        origin=Origin(xyz=(0.83, 0.069, 0.20)),
        material=brushed_steel,
        name="side_rail_0",
    )
    beam.visual(
        Box((1.25, 0.008, 0.034)),
        origin=Origin(xyz=(0.83, -0.069, 0.20)),
        material=brushed_steel,
        name="side_rail_1",
    )
    beam.visual(
        Box((0.035, 0.21, 0.17)),
        origin=Origin(xyz=(1.5275, 0.0, 0.225)),
        material=dark_steel,
        name="end_stop",
    )
    beam.visual(
        Box((0.36, 0.035, 0.16)),
        origin=Origin(xyz=(0.14, 0.083, 0.155), rpy=(0.0, 0.0, 0.48)),
        material=dark_steel,
        name="gusset_0",
    )
    beam.visual(
        Box((0.36, 0.035, 0.16)),
        origin=Origin(xyz=(0.14, -0.083, 0.155), rpy=(0.0, 0.0, -0.48)),
        material=dark_steel,
        name="gusset_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.26, 0.028, 0.25)),
        origin=Origin(xyz=(0.0, 0.089, 0.0)),
        material=painted_steel,
        name="side_plate_0",
    )
    carriage.visual(
        Box((0.26, 0.028, 0.25)),
        origin=Origin(xyz=(0.0, -0.089, 0.0)),
        material=painted_steel,
        name="side_plate_1",
    )
    carriage.visual(
        Box((0.27, 0.205, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=painted_steel,
        name="top_bridge",
    )
    carriage.visual(
        Box((0.27, 0.205, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.1125)),
        material=painted_steel,
        name="bottom_bridge",
    )
    for i, x in enumerate((-0.072, 0.072)):
        carriage.visual(
            Cylinder(radius=0.015, length=0.19),
            origin=Origin(xyz=(x, 0.0, 0.075), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"top_roller_{i}",
        )
        carriage.visual(
            Cylinder(radius=0.015, length=0.19),
            origin=Origin(xyz=(x, 0.0, -0.075), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"bottom_roller_{i}",
        )
    carriage.visual(
        Box((0.11, 0.13, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.17)),
        material=dark_steel,
        name="tool_mount_lug",
    )

    model.articulation(
        "column_to_beam",
        ArticulationType.REVOLUTE,
        parent=column,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 1.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.75, lower=-pi, upper=pi),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.36, 0.0, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=0.0, upper=0.88),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    turret_joint = object_model.get_articulation("column_to_beam")
    carriage_joint = object_model.get_articulation("beam_to_carriage")

    ctx.expect_contact(
        beam,
        column,
        elem_a="turntable_disk",
        elem_b="top_bearing_flange",
        name="turntable disk is seated on column bearing",
    )
    ctx.expect_overlap(
        beam,
        column,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="top_bearing_flange",
        min_overlap=0.28,
        name="turret footprint covers bearing flange",
    )
    ctx.expect_gap(
        carriage,
        beam,
        axis="z",
        positive_elem="top_roller_0",
        negative_elem="arm_tube",
        max_gap=0.002,
        max_penetration=0.001,
        name="upper carriage rollers ride on the beam",
    )
    ctx.expect_gap(
        beam,
        carriage,
        axis="z",
        positive_elem="arm_tube",
        negative_elem="bottom_roller_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="lower carriage rollers capture the beam",
    )
    ctx.expect_overlap(
        carriage,
        beam,
        axes="x",
        elem_a="top_bridge",
        elem_b="arm_tube",
        min_overlap=0.20,
        name="carriage is centered over the arm at home",
    )

    home_position = ctx.part_world_position(carriage)
    with ctx.pose({carriage_joint: 0.88}):
        extended_position = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            beam,
            axes="x",
            elem_a="top_bridge",
            elem_b="arm_tube",
            min_overlap=0.20,
            name="carriage remains on the beam at full travel",
        )
        ctx.expect_gap(
            carriage,
            beam,
            axis="z",
            positive_elem="top_roller_0",
            negative_elem="arm_tube",
            max_gap=0.002,
            max_penetration=0.001,
            name="upper rollers stay on the beam at full travel",
        )
    ctx.check(
        "prismatic joint moves carriage outward along beam",
        home_position is not None
        and extended_position is not None
        and extended_position[0] > home_position[0] + 0.80,
        details=f"home={home_position}, extended={extended_position}",
    )

    with ctx.pose({turret_joint: pi / 2.0}):
        rotated_position = ctx.part_world_position(carriage)
    ctx.check(
        "revolute turret swings radial arm about column axis",
        rotated_position is not None
        and abs(rotated_position[0]) < 0.02
        and rotated_position[1] > 0.34,
        details=f"rotated={rotated_position}",
    )

    return ctx.report()


object_model = build_object_model()
