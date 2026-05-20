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
    model = ArticulatedObject(name="radial_arm_sliding_carriage")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    blue = model.material("blue_enamel", rgba=(0.05, 0.16, 0.33, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    zinc = model.material("zinc_fasteners", rgba=(0.78, 0.78, 0.72, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.24, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cast_iron,
        name="floor_foot",
    )
    base.visual(
        Cylinder(radius=0.070, length=1.04),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=steel,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
        material=cast_iron,
        name="bearing_cap",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=cast_iron,
        name="column_foot_collar",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.125, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="turntable",
    )
    arm.visual(
        Cylinder(radius=0.078, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=steel,
        name="rotary_hub",
    )
    arm.visual(
        Box((1.16, 0.090, 0.070)),
        origin=Origin(xyz=(0.585, 0.0, 0.250)),
        material=blue,
        name="main_beam",
    )
    arm.visual(
        Box((0.28, 0.026, 0.240)),
        origin=Origin(xyz=(0.125, 0.0, 0.150)),
        material=blue,
        name="root_web",
    )
    arm.visual(
        Box((0.95, 0.008, 0.018)),
        origin=Origin(xyz=(0.60, 0.049, 0.250)),
        material=steel,
        name="guide_rail_0",
    )
    arm.visual(
        Box((0.95, 0.008, 0.018)),
        origin=Origin(xyz=(0.60, -0.049, 0.250)),
        material=steel,
        name="guide_rail_1",
    )
    arm.visual(
        Box((0.035, 0.140, 0.140)),
        origin=Origin(xyz=(1.175, 0.0, 0.250)),
        material=cast_iron,
        name="end_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.200, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=cast_iron,
        name="saddle_top",
    )
    carriage.visual(
        Box((0.200, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=cast_iron,
        name="saddle_bottom",
    )
    carriage.visual(
        Box((0.200, 0.018, 0.130)),
        origin=Origin(xyz=(0.0, 0.062, 0.0)),
        material=cast_iron,
        name="side_plate_0",
    )
    carriage.visual(
        Box((0.200, 0.018, 0.130)),
        origin=Origin(xyz=(0.0, -0.062, 0.0)),
        material=cast_iron,
        name="side_plate_1",
    )
    carriage.visual(
        Box((0.050, 0.110, 0.045)),
        origin=Origin(xyz=(0.092, 0.0, -0.073)),
        material=cast_iron,
        name="head_bracket",
    )
    carriage.visual(
        Box((0.022, 0.170, 0.200)),
        origin=Origin(xyz=(0.109, 0.0, -0.150)),
        material=blue,
        name="head_plate",
    )
    carriage.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.126, 0.0, -0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="round_mount",
    )
    for i, y in enumerate((-0.056, 0.056)):
        for j, z in enumerate((-0.210, -0.090)):
            carriage.visual(
                Cylinder(radius=0.0085, length=0.010),
                origin=Origin(xyz=(0.123, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=zinc,
                name=f"head_bolt_{i}_{j}",
            )
    carriage.visual(
        Box((0.030, 0.040, 0.020)),
        origin=Origin(xyz=(-0.088, 0.0, -0.075)),
        material=black,
        name="rear_bumper",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-2.7, upper=2.7),
    )

    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.450, 0.0, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    pivot = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="turntable",
        negative_elem="bearing_cap",
        name="turntable sits on column bearing",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        min_overlap=0.08,
        elem_a="turntable",
        elem_b="bearing_cap",
        name="turntable is concentric with column",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="saddle_top",
        negative_elem="main_beam",
        name="carriage clears top of beam",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="y",
        max_penetration=1e-6,
        max_gap=0.001,
        positive_elem="side_plate_0",
        negative_elem="guide_rail_0",
        name="carriage clears positive guide rail",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        max_penetration=1e-6,
        max_gap=0.001,
        positive_elem="guide_rail_1",
        negative_elem="side_plate_1",
        name="carriage clears negative guide rail",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="x",
        min_overlap=0.17,
        elem_a="saddle_top",
        elem_b="main_beam",
        name="carriage surrounds beam at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.55}):
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            min_overlap=0.17,
            elem_a="saddle_top",
            elem_b="main_beam",
            name="extended carriage remains on beam",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage slides along arm",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.50
        and abs(extended_pos[1] - rest_pos[1]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({pivot: 1.2}):
        swept_pos = ctx.part_world_position(carriage)
    ctx.check(
        "arm sweeps carriage in plan",
        rest_pos is not None
        and swept_pos is not None
        and swept_pos[1] > rest_pos[1] + 0.25
        and abs(swept_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
