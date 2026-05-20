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
    model = ArticulatedObject(name="linear_carriage_rotary_spindle")

    steel = Material("brushed_steel", color=(0.68, 0.70, 0.72, 1.0))
    dark_steel = Material("dark_bearing_steel", color=(0.08, 0.09, 0.10, 1.0))
    blue = Material("anodized_blue_carriage", color=(0.05, 0.19, 0.45, 1.0))
    black = Material("black_oxide", color=(0.01, 0.01, 0.012, 1.0))
    brass = Material("oiled_bronze_bushing", color=(0.75, 0.50, 0.18, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((0.56, 0.17, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    for i, y in enumerate((-0.045, 0.045)):
        base.visual(
            Box((0.50, 0.024, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.0375)),
            material=steel,
            name=("guide_rail_0", "guide_rail_1")[i],
        )
    for i, x in enumerate((-0.265, 0.265)):
        base.visual(
            Box((0.030, 0.17, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0525)),
            material=black,
            name=f"end_stop_{i}",
        )

    carriage = model.part("carriage")
    for i, (x, y) in enumerate(
        ((-0.035, -0.045), (0.035, -0.045), (-0.035, 0.045), (0.035, 0.045))
    ):
        carriage.visual(
            Box((0.050, 0.026, 0.008)),
            origin=Origin(xyz=(x, y, 0.054)),
            material=brass,
            name=("bearing_pad_0", "bearing_pad_1", "bearing_pad_2", "bearing_pad_3")[i],
        )
    carriage.visual(
        Box((0.135, 0.145, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0755)),
        material=blue,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.095, 0.070, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=blue,
        name="housing_pedestal",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.133), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="bearing_housing",
    )
    for i, x in enumerate((-0.050, 0.050)):
        carriage.visual(
            Cylinder(radius=0.044, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.133), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"bearing_cap_{i}",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.011, length=0.165),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_chuck",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.146, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tool_bit",
    )
    spindle.visual(
        Box((0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.100, 0.0, 0.020)),
        material=black,
        name="chuck_set_screw",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.4, lower=-0.100, upper=0.100),
    )
    model.articulation(
        "spindle_rotation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=6.0, lower=-math.pi, upper=math.pi
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    base = object_model.get_part("base_rail")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("spindle_rotation")

    ctx.allow_overlap(
        carriage,
        spindle,
        elem_a="bearing_housing",
        elem_b="spindle_shaft",
        reason="The rotating spindle shaft is intentionally captured through the fixed bearing housing.",
    )
    for cap_name in ("bearing_cap_0", "bearing_cap_1"):
        ctx.allow_overlap(
            carriage,
            spindle,
            elem_a=cap_name,
            elem_b="spindle_shaft",
            reason="The spindle shaft intentionally passes through the bearing end cap bore.",
        )
    ctx.expect_within(
        spindle,
        carriage,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="bearing_housing",
        margin=0.001,
        name="spindle shaft is centered inside the bearing bore",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="x",
        elem_a="spindle_shaft",
        elem_b="bearing_housing",
        min_overlap=0.085,
        name="spindle shaft remains captured through the housing",
    )
    for cap_name in ("bearing_cap_0", "bearing_cap_1"):
        ctx.expect_within(
            spindle,
            carriage,
            axes="yz",
            inner_elem="spindle_shaft",
            outer_elem=cap_name,
            margin=0.001,
            name=f"spindle shaft is centered in {cap_name}",
        )
        ctx.expect_overlap(
            spindle,
            carriage,
            axes="x",
            elem_a="spindle_shaft",
            elem_b=cap_name,
            min_overlap=0.009,
            name=f"spindle shaft passes through {cap_name}",
        )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="bearing_pad_0",
        negative_elem="guide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage bearing pad sits on rail",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.100}):
        end_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="bearing_pad_0",
            negative_elem="guide_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage stays seated at end of travel",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_pad_0",
            elem_b="guide_rail_0",
            min_overlap=0.040,
            name="bearing pad remains on the guide rail at end travel",
        )
    ctx.check(
        "carriage has 200 mm slide travel",
        rest_pos is not None
        and end_pos is not None
        and abs((end_pos[0] - rest_pos[0]) - 0.100) < 1e-6
        and abs((slide.motion_limits.upper - slide.motion_limits.lower) - 0.200) < 1e-6,
        details=f"rest={rest_pos}, end={end_pos}, limits={slide.motion_limits}",
    )

    with ctx.pose({spin: 0.0}):
        screw_aabb = ctx.part_element_world_aabb(spindle, elem="chuck_set_screw")
        screw_z_0 = (screw_aabb[0][2] + screw_aabb[1][2]) / 2.0 if screw_aabb else None
    with ctx.pose({spin: math.pi}):
        screw_aabb = ctx.part_element_world_aabb(spindle, elem="chuck_set_screw")
        screw_z_pi = (screw_aabb[0][2] + screw_aabb[1][2]) / 2.0 if screw_aabb else None
    ctx.check(
        "spindle rotates about its own local axis",
        screw_z_0 is not None
        and screw_z_pi is not None
        and screw_z_0 > 0.145
        and screw_z_pi < 0.125
        and spin.motion_limits.lower <= -math.pi
        and spin.motion_limits.upper >= math.pi,
        details=f"set screw z at 0={screw_z_0}, at pi={screw_z_pi}, limits={spin.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
