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
    model = ArticulatedObject(name="orthogonal_xz_stage")

    anodized = model.material("black_anodized_aluminum", rgba=(0.04, 0.045, 0.05, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.18, 0.45, 1.0))
    pad = model.material("matte_mounting_pad", rgba=(0.015, 0.016, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.34, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=anodized,
        name="ground_plate",
    )
    base.visual(
        Box((0.69, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.095, 0.052)),
        material=steel,
        name="x_rail_0",
    )
    base.visual(
        Box((0.69, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.095, 0.052)),
        material=steel,
        name="x_rail_1",
    )
    base.visual(
        Box((0.035, 0.26, 0.055)),
        origin=Origin(xyz=(-0.35, 0.0, 0.0615)),
        material=dark_steel,
        name="end_stop_0",
    )
    base.visual(
        Box((0.035, 0.26, 0.055)),
        origin=Origin(xyz=(0.35, 0.0, 0.0615)),
        material=dark_steel,
        name="end_stop_1",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.672),
        origin=Origin(xyz=(0.0, 0.0, 0.066), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="x_leadscrew",
    )

    carriage = model.part("x_carriage")
    carriage.visual(
        Box((0.17, 0.255, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=carriage_blue,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.062, 0.052, 0.025)),
        origin=Origin(xyz=(-0.045, -0.095, 0.0125)),
        material=dark_steel,
        name="x_shoe_0",
    )
    carriage.visual(
        Box((0.062, 0.052, 0.025)),
        origin=Origin(xyz=(-0.045, 0.095, 0.0125)),
        material=dark_steel,
        name="x_shoe_1",
    )
    carriage.visual(
        Box((0.062, 0.052, 0.025)),
        origin=Origin(xyz=(0.045, -0.095, 0.0125)),
        material=dark_steel,
        name="x_shoe_2",
    )
    carriage.visual(
        Box((0.062, 0.052, 0.025)),
        origin=Origin(xyz=(0.045, 0.095, 0.0125)),
        material=dark_steel,
        name="x_shoe_3",
    )
    carriage.visual(
        Box((0.15, 0.025, 0.34)),
        origin=Origin(xyz=(0.0, 0.080, 0.2055)),
        material=carriage_blue,
        name="z_backplate",
    )
    carriage.visual(
        Box((0.020, 0.016, 0.300)),
        origin=Origin(xyz=(-0.065, 0.061, 0.225)),
        material=steel,
        name="z_guide_0",
    )
    carriage.visual(
        Box((0.020, 0.016, 0.300)),
        origin=Origin(xyz=(0.065, 0.061, 0.225)),
        material=steel,
        name="z_guide_1",
    )
    carriage.visual(
        Box((0.12, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.058, 0.060)),
        material=dark_steel,
        name="z_lower_bearing",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.050, 0.034, 0.270)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark_steel,
        name="vertical_ram",
    )
    z_slide.visual(
        Box((0.150, 0.016, 0.045)),
        origin=Origin(xyz=(0.0, 0.013, 0.120)),
        material=dark_steel,
        name="z_saddle_block",
    )
    z_slide.visual(
        Box((0.130, 0.105, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.283)),
        material=pad,
        name="top_pad",
    )
    z_slide.visual(
        Box((0.090, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.261)),
        material=carriage_blue,
        name="pad_clamp",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.16, 0.0, 0.0695)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.32),
    )
    model.articulation(
        "z_slide_joint",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.032, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("x_carriage")
    z_slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("x_slide")
    z_axis = object_model.get_articulation("z_slide_joint")

    ctx.check(
        "lower joint is horizontal prismatic X",
        x_axis.articulation_type == ArticulationType.PRISMATIC and tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={x_axis.articulation_type}, axis={x_axis.axis}",
    )
    ctx.check(
        "upper joint is vertical prismatic Z",
        z_axis.articulation_type == ArticulationType.PRISMATIC and tuple(z_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={z_axis.articulation_type}, axis={z_axis.axis}",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="x_shoe_0",
        negative_elem="x_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage shoe seats on X rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="x_shoe_0",
        elem_b="x_rail_0",
        min_overlap=0.020,
        name="carriage shoe is aligned over X rail",
    )
    ctx.expect_within(
        z_slide,
        carriage,
        axes="x",
        inner_elem="vertical_ram",
        outer_elem="z_backplate",
        margin=0.0,
        name="vertical ram stays between guide sides at rest",
    )

    rest_x = ctx.part_world_position(carriage)
    rest_z = ctx.part_world_position(z_slide)
    with ctx.pose({x_axis: 0.32, z_axis: 0.14}):
        extended_x = ctx.part_world_position(carriage)
        extended_z = ctx.part_world_position(z_slide)
        ctx.expect_overlap(
            z_slide,
            carriage,
            axes="z",
            elem_a="vertical_ram",
            elem_b="z_guide_0",
            min_overlap=0.08,
            name="raised vertical ram remains engaged with guide",
        )
    ctx.check(
        "X stage translates in positive X",
        rest_x is not None and extended_x is not None and extended_x[0] > rest_x[0] + 0.30,
        details=f"rest={rest_x}, extended={extended_x}",
    )
    ctx.check(
        "Z stage translates upward",
        rest_z is not None and extended_z is not None and extended_z[2] > rest_z[2] + 0.13,
        details=f"rest={rest_z}, extended={extended_z}",
    )

    return ctx.report()


object_model = build_object_model()
