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
    model = ArticulatedObject(name="radial_arm_mechanism")

    cast_iron = Material("dark_cast_iron", rgba=(0.12, 0.14, 0.15, 1.0))
    machine_blue = Material("machine_blue", rgba=(0.05, 0.17, 0.28, 1.0))
    machined = Material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    blackened = Material("blackened_hardware", rgba=(0.025, 0.025, 0.023, 1.0))
    safety_orange = Material("safety_orange", rgba=(0.92, 0.40, 0.06, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.82, 0.56, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.22, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=cast_iron,
        name="base_boss",
    )
    pedestal.visual(
        Cylinder(radius=0.090, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=machine_blue,
        name="column_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.140, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.885)),
        material=machined,
        name="top_bearing",
    )
    pedestal.visual(
        Box((0.34, 0.038, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=cast_iron,
        name="rib_x",
    )
    pedestal.visual(
        Box((0.038, 0.34, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=cast_iron,
        name="rib_y",
    )
    for i, (x, y) in enumerate(((-0.31, -0.20), (-0.31, 0.20), (0.31, -0.20), (0.31, 0.20))):
        pedestal.visual(
            Cylinder(radius=0.030, length=0.022),
            origin=Origin(xyz=(x, y, 0.091)),
            material=blackened,
            name=f"anchor_bolt_{i}",
        )

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        Cylinder(radius=0.155, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=machined,
        name="turntable",
    )
    swing_arm.visual(
        Cylinder(radius=0.074, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=machine_blue,
        name="pivot_post",
    )
    swing_arm.visual(
        Box((0.92, 0.12, 0.13)),
        origin=Origin(xyz=(0.52, 0.0, 0.165)),
        material=machine_blue,
        name="box_beam",
    )
    swing_arm.visual(
        Box((0.30, 0.19, 0.090)),
        origin=Origin(xyz=(0.145, 0.0, 0.145)),
        material=machine_blue,
        name="root_knuckle",
    )
    swing_arm.visual(
        Box((0.82, 0.028, 0.030)),
        origin=Origin(xyz=(0.58, -0.083, 0.107)),
        material=cast_iron,
        name="lower_flange_0",
    )
    swing_arm.visual(
        Box((0.82, 0.028, 0.030)),
        origin=Origin(xyz=(0.58, 0.083, 0.107)),
        material=cast_iron,
        name="lower_flange_1",
    )
    for i, y in enumerate((-0.076, 0.076)):
        swing_arm.visual(
            Cylinder(radius=0.016, length=0.78),
            origin=Origin(xyz=(0.59, y, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined,
            name=f"guide_rail_{i}",
        )
        for j, x in enumerate((0.24, 0.94)):
            swing_arm.visual(
                Box((0.060, 0.052, 0.074)),
                origin=Origin(xyz=(x, y, 0.089)),
                material=cast_iron,
                name=f"rail_clamp_{i}_{j}",
            )
    swing_arm.visual(
        Box((0.18, 0.20, 0.055)),
        origin=Origin(xyz=(0.965, 0.0, 0.166)),
        material=cast_iron,
        name="end_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.19, 0.108, 0.092)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=cast_iron,
        name="crosshead",
    )
    for i, y in enumerate((-0.076, 0.076)):
        carriage.visual(
            Box((0.17, 0.056, 0.040)),
            origin=Origin(xyz=(0.0, y, -0.036)),
            material=machined,
            name=f"bearing_pad_{i}",
        )
    carriage.visual(
        Box((0.165, 0.155, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=machine_blue,
        name="tool_head",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, -0.272)),
        material=machined,
        name="spindle_socket",
    )
    carriage.visual(
        Box((0.115, 0.026, 0.070)),
        origin=Origin(xyz=(0.0, -0.085, -0.176)),
        material=safety_orange,
        name="front_badge",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=swing_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.9, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=swing_arm,
        child=carriage,
        origin=Origin(xyz=(0.38, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.40),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    swing_arm = object_model.get_part("swing_arm")
    carriage = object_model.get_part("carriage")
    swing_joint = object_model.get_articulation("column_to_arm")
    slide_joint = object_model.get_articulation("arm_to_carriage")

    ctx.expect_gap(
        swing_arm,
        pedestal,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotating turntable is seated on the column bearing",
    )
    ctx.expect_overlap(
        swing_arm,
        pedestal,
        axes="xy",
        elem_a="turntable",
        elem_b="top_bearing",
        min_overlap=0.10,
        name="turntable footprint captures the vertical bearing",
    )
    for i in range(2):
        ctx.expect_gap(
            swing_arm,
            carriage,
            axis="z",
            positive_elem=f"guide_rail_{i}",
            negative_elem=f"bearing_pad_{i}",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"bearing pad {i} rides just under its guide rail",
        )
        ctx.expect_overlap(
            carriage,
            swing_arm,
            axes="x",
            elem_a=f"bearing_pad_{i}",
            elem_b=f"guide_rail_{i}",
            min_overlap=0.15,
            name=f"bearing pad {i} remains engaged with the rail at rest",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: 0.40}):
        extended_pos = ctx.part_world_position(carriage)
        for i in range(2):
            ctx.expect_gap(
                swing_arm,
                carriage,
                axis="z",
                positive_elem=f"guide_rail_{i}",
                negative_elem=f"bearing_pad_{i}",
                max_gap=0.003,
                max_penetration=0.001,
                name=f"bearing pad {i} stays on the rail at full travel",
            )
            ctx.expect_overlap(
                carriage,
                swing_arm,
                axes="x",
                elem_a=f"bearing_pad_{i}",
                elem_b=f"guide_rail_{i}",
                min_overlap=0.15,
                name=f"bearing pad {i} remains engaged at full travel",
            )
    ctx.check(
        "carriage travel moves outward along the arm",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({swing_joint: 0.90}):
        swung_pos = ctx.part_world_position(carriage)
    ctx.check(
        "arm rotation swings the carriage around the column",
        rest_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_pos[1] + 0.25,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
