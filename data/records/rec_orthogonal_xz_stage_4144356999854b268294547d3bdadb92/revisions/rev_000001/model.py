from __future__ import annotations

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
    model = ArticulatedObject(name="stacked_xz_positioning_stage")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_anodized = model.material("black_anodized", rgba=(0.05, 0.055, 0.06, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    screw_steel = model.material("lead_screw_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    brass = model.material("brass_bearing", rgba=(0.80, 0.58, 0.24, 1.0))
    top_blue = model.material("blue_fixture_plate", rgba=(0.05, 0.22, 0.55, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.70, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.62, 0.026, 0.027)),
        origin=Origin(xyz=(0.0, -0.075, 0.0475)),
        material=rail_steel,
        name="x_rail_0",
    )
    base.visual(
        Box((0.62, 0.026, 0.027)),
        origin=Origin(xyz=(0.0, 0.075, 0.0475)),
        material=rail_steel,
        name="x_rail_1",
    )
    base.visual(
        Box((0.035, 0.22, 0.080)),
        origin=Origin(xyz=(-0.335, 0.0, 0.040)),
        material=aluminum,
        name="end_stop_0",
    )
    base.visual(
        Box((0.035, 0.22, 0.080)),
        origin=Origin(xyz=(0.335, 0.0, 0.040)),
        material=aluminum,
        name="end_stop_1",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.58),
        origin=Origin(xyz=(0.0, -0.105, 0.065), rpy=(0.0, 1.57079632679, 0.0)),
        material=screw_steel,
        name="x_lead_screw",
    )
    base.visual(
        Box((0.026, 0.030, 0.045)),
        origin=Origin(xyz=(-0.295, -0.105, 0.0575)),
        material=aluminum,
        name="x_screw_block_0",
    )
    base.visual(
        Box((0.026, 0.030, 0.045)),
        origin=Origin(xyz=(0.295, -0.105, 0.0575)),
        material=aluminum,
        name="x_screw_block_1",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.18, 0.17, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="saddle_plate",
    )
    x_carriage.visual(
        Box((0.16, 0.038, 0.018)),
        origin=Origin(xyz=(0.0, -0.075, 0.009)),
        material=brass,
        name="x_bearing_0",
    )
    x_carriage.visual(
        Box((0.16, 0.038, 0.018)),
        origin=Origin(xyz=(0.0, 0.075, 0.009)),
        material=brass,
        name="x_bearing_1",
    )
    x_carriage.visual(
        Box((0.10, 0.035, 0.320)),
        origin=Origin(xyz=(0.0, 0.045, 0.200)),
        material=dark_anodized,
        name="upright_back",
    )
    x_carriage.visual(
        Box((0.016, 0.018, 0.270)),
        origin=Origin(xyz=(-0.032, 0.019, 0.205)),
        material=rail_steel,
        name="z_rail_0",
    )
    x_carriage.visual(
        Box((0.016, 0.018, 0.270)),
        origin=Origin(xyz=(0.032, 0.019, 0.205)),
        material=rail_steel,
        name="z_rail_1",
    )
    x_carriage.visual(
        Box((0.13, 0.055, 0.025)),
        origin=Origin(xyz=(0.0, 0.045, 0.365)),
        material=aluminum,
        name="upright_cap",
    )
    x_carriage.visual(
        Cylinder(radius=0.005, length=0.250),
        origin=Origin(xyz=(0.065, -0.005, 0.180)),
        material=screw_steel,
        name="z_lead_screw",
    )
    x_carriage.visual(
        Box((0.025, 0.025, 0.025)),
        origin=Origin(xyz=(0.065, -0.005, 0.0525)),
        material=aluminum,
        name="z_screw_lower_block",
    )
    x_carriage.visual(
        Box((0.025, 0.025, 0.025)),
        origin=Origin(xyz=(0.065, -0.005, 0.3100)),
        material=aluminum,
        name="z_screw_upper_block",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.11, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=aluminum,
        name="z_slider_block",
    )
    z_carriage.visual(
        Box((0.085, 0.040, 0.150)),
        origin=Origin(xyz=(0.0, -0.030, 0.160)),
        material=dark_anodized,
        name="top_riser",
    )
    z_carriage.visual(
        Box((0.16, 0.14, 0.025)),
        origin=Origin(xyz=(0.0, -0.060, 0.2475)),
        material=top_blue,
        name="top_plate",
    )
    z_carriage.visual(
        Box((0.10, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.002, 0.030)),
        material=brass,
        name="z_gib_pad",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, -0.0075, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.16),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_carriage = object_model.get_part("z_carriage")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

    ctx.check(
        "lower joint is horizontal prismatic",
        x_axis.articulation_type == ArticulationType.PRISMATIC and tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={x_axis.articulation_type}, axis={x_axis.axis}",
    )
    ctx.check(
        "upper joint is vertical prismatic",
        z_axis.articulation_type == ArticulationType.PRISMATIC and tuple(z_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={z_axis.articulation_type}, axis={z_axis.axis}",
    )

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_bearing_0",
        negative_elem="x_rail_0",
        max_gap=0.001,
        max_penetration=0.000001,
        name="lower carriage rides on first rail",
    )
    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_bearing_1",
        negative_elem="x_rail_1",
        max_gap=0.001,
        max_penetration=0.000001,
        name="lower carriage rides on second rail",
    )
    ctx.expect_gap(
        x_carriage,
        z_carriage,
        axis="y",
        positive_elem="z_rail_0",
        negative_elem="z_slider_block",
        max_gap=0.001,
        max_penetration=0.000001,
        name="vertical slide block bears against guide rail",
    )

    with ctx.pose({x_axis: 0.18}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_bearing_0",
            elem_b="x_rail_0",
            min_overlap=0.12,
            name="positive x travel remains supported",
        )
    with ctx.pose({x_axis: -0.18}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_bearing_1",
            elem_b="x_rail_1",
            min_overlap=0.12,
            name="negative x travel remains supported",
        )

    rest_x = ctx.part_world_position(x_carriage)
    with ctx.pose({x_axis: 0.18}):
        extended_x = ctx.part_world_position(x_carriage)
    ctx.check(
        "x carriage translates along lower axis",
        rest_x is not None and extended_x is not None and extended_x[0] > rest_x[0] + 0.15,
        details=f"rest={rest_x}, extended={extended_x}",
    )

    rest_z = ctx.part_world_position(z_carriage)
    with ctx.pose({z_axis: 0.16}):
        extended_z = ctx.part_world_position(z_carriage)
        ctx.expect_overlap(
            z_carriage,
            x_carriage,
            axes="z",
            elem_a="z_slider_block",
            elem_b="z_rail_0",
            min_overlap=0.05,
            name="raised vertical stage remains on guide",
        )
    ctx.check(
        "top stage translates upward",
        rest_z is not None and extended_z is not None and extended_z[2] > rest_z[2] + 0.14,
        details=f"rest={rest_z}, extended={extended_z}",
    )

    return ctx.report()


object_model = build_object_model()
