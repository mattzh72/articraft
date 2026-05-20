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
    model = ArticulatedObject(name="rotary_column_lift_carriage")

    dark_cast = model.material("dark_cast_iron", rgba=(0.07, 0.075, 0.08, 1.0))
    black = model.material("black_bearing", rgba=(0.01, 0.011, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    rail_finish = model.material("ground_rail", rgba=(0.78, 0.79, 0.76, 1.0))
    orange = model.material("orange_carriage", rgba=(0.95, 0.38, 0.07, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.32, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_cast,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=black,
        name="bearing_pedestal",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.13, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="turntable_disk",
    )
    column.visual(
        Cylinder(radius=0.05, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=steel,
        name="round_mast",
    )
    column.visual(
        Box((0.035, 0.13, 0.56)),
        origin=Origin(xyz=(0.0675, 0.0, 0.50)),
        material=rail_finish,
        name="guide_rail",
    )
    column.visual(
        Box((0.065, 0.18, 0.04)),
        origin=Origin(xyz=(0.075, 0.0, 0.20)),
        material=black,
        name="lower_stop",
    )
    column.visual(
        Box((0.065, 0.18, 0.04)),
        origin=Origin(xyz=(0.075, 0.0, 0.80)),
        material=black,
        name="upper_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.08, 0.22, 0.16)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=orange,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.074, 0.024, 0.16)),
        origin=Origin(xyz=(0.012, 0.078, 0.0)),
        material=orange,
        name="guide_shoe_0",
    )
    carriage.visual(
        Box((0.074, 0.024, 0.16)),
        origin=Origin(xyz=(0.012, -0.078, 0.0)),
        material=orange,
        name="guide_shoe_1",
    )
    carriage.visual(
        Box((0.018, 0.16, 0.10)),
        origin=Origin(xyz=(0.089, 0.0, 0.0)),
        material=steel,
        name="front_tool_plate",
    )
    for index, y in enumerate((-0.055, 0.055)):
        for z in (-0.035, 0.035):
            carriage.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(
                    xyz=(0.102, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black,
                name=f"bolt_head_{index}_{0 if z < 0 else 1}",
            )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.085, 0.0, 0.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    rotate = object_model.get_articulation("base_to_column")
    lift = object_model.get_articulation("column_to_carriage")

    ctx.check(
        "column is revolute about base centerline",
        rotate.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in rotate.axis) == (0.0, 0.0, 1.0),
        details=f"type={rotate.articulation_type}, axis={rotate.axis}",
    )
    ctx.check(
        "carriage is vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating turntable sits on fixed bearing",
    )
    ctx.expect_gap(
        carriage,
        column,
        axis="x",
        positive_elem="carriage_block",
        negative_elem="guide_rail",
        max_gap=0.001,
        max_penetration=1e-6,
        name="carriage bears against the guide rail face",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="z",
        elem_a="carriage_block",
        elem_b="guide_rail",
        min_overlap=0.12,
        name="carriage remains engaged on guide at lower travel",
    )

    lower_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.32}):
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            elem_a="carriage_block",
            elem_b="guide_rail",
            min_overlap=0.12,
            name="carriage remains engaged on guide at upper travel",
        )
        upper_position = ctx.part_world_position(carriage)

    ctx.check(
        "lift travel raises the carriage",
        lower_position is not None
        and upper_position is not None
        and upper_position[2] > lower_position[2] + 0.30,
        details=f"lower={lower_position}, upper={upper_position}",
    )

    home_position = ctx.part_world_position(carriage)
    with ctx.pose({rotate: math.pi / 2.0}):
        turned_position = ctx.part_world_position(carriage)

    ctx.check(
        "column rotation carries the lift carriage around the base",
        home_position is not None
        and turned_position is not None
        and abs(home_position[0]) > 0.07
        and abs(turned_position[1]) > 0.07
        and abs(turned_position[0]) < 0.01,
        details=f"home={home_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
