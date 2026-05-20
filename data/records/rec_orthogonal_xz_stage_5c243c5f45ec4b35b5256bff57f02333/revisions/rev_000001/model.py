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
    model = ArticulatedObject(name="bench_positioning_axis")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.06, 0.065, 0.07, 1.0))
    carriage_blue = model.material("blue_machined_carriage", rgba=(0.05, 0.22, 0.42, 1.0))
    column_gray = model.material("ground_cast_column", rgba=(0.30, 0.32, 0.34, 1.0))
    rail_steel = model.material("polished_linear_rail", rgba=(0.72, 0.74, 0.76, 1.0))
    black = model.material("black_hardware", rgba=(0.01, 0.012, 0.014, 1.0))
    stop_red = model.material("red_stop_blocks", rgba=(0.65, 0.05, 0.025, 1.0))
    rubber = model.material("matte_rubber_pad", rgba=(0.015, 0.015, 0.013, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.34, 0.36, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=anodized,
        name="base_bed",
    )
    for idx, y in enumerate((-0.090, 0.090)):
        rail_name = ("x_rail_0", "x_rail_1")[idx]
        base.visual(
            Box((1.16, 0.040, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.074)),
            material=black,
            name=f"rail_riser_{idx}",
        )
        base.visual(
            Cylinder(radius=0.012, length=1.16),
            origin=Origin(xyz=(0.0, y, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_steel,
            name=rail_name,
        )
    for idx, x in enumerate((-0.620, 0.620)):
        base.visual(
            Box((0.060, 0.330, 0.115)),
            origin=Origin(xyz=(x, 0.0, 0.116)),
            material=column_gray,
            name=f"x_end_stop_{idx}",
        )
        base.visual(
            Box((0.012, 0.180, 0.040)),
            origin=Origin(xyz=(x * 0.965, 0.0, 0.126)),
            material=stop_red,
            name=f"x_bumper_{idx}",
        )

    x_carriage = model.part("x_carriage")
    for idx, y in enumerate((-0.090, 0.090)):
        bearing_name = ("x_bearing_0", "x_bearing_1")[idx]
        x_carriage.visual(
            Box((0.235, 0.046, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.128)),
            material=rail_steel,
            name=bearing_name,
        )
    x_carriage.visual(
        Box((0.300, 0.270, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.169)),
        material=carriage_blue,
        name="saddle_deck",
    )
    x_carriage.visual(
        Box((0.095, 0.112, 0.758)),
        origin=Origin(xyz=(-0.025, 0.0, 0.574)),
        material=column_gray,
        name="column_spine",
    )
    for idx, y in enumerate((-0.064, 0.064)):
        x_carriage.visual(
            Box((0.128, 0.020, 0.754)),
            origin=Origin(xyz=(-0.006, y, 0.574)),
            material=column_gray,
            name=f"guide_plate_{idx}",
        )
    x_carriage.visual(
        Box((0.138, 0.170, 0.034)),
        origin=Origin(xyz=(-0.006, 0.0, 0.226)),
        material=black,
        name="lower_column_tie",
    )
    x_carriage.visual(
        Box((0.138, 0.170, 0.036)),
        origin=Origin(xyz=(-0.006, 0.0, 0.936)),
        material=black,
        name="upper_column_tie",
    )
    for idx, y in enumerate((-0.043, 0.043)):
        z_rail_name = ("z_guide_rail_0", "z_guide_rail_1")[idx]
        x_carriage.visual(
            Box((0.014, 0.020, 0.630)),
            origin=Origin(xyz=(0.023, y, 0.570)),
            material=rail_steel,
            name=z_rail_name,
        )
    for idx, z in enumerate((0.300, 0.690)):
        z_stop_name = ("z_stop_0", "z_stop_1")[idx]
        x_carriage.visual(
            Box((0.040, 0.018, 0.050)),
            origin=Origin(xyz=(0.028, 0.083, z)),
            material=stop_red,
            name=z_stop_name,
        )
    x_carriage.visual(
        Box((0.026, 0.018, 0.620)),
        origin=Origin(xyz=(0.021, -0.083, 0.570)),
        material=black,
        name="scale_strip",
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(-0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.560),
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.042, 0.132, 0.124)),
        origin=Origin(xyz=(0.006, 0.0, 0.062)),
        material=carriage_blue,
        name="z_saddle",
    )
    for idx, y in enumerate((-0.043, 0.043)):
        z_bearing_name = ("z_bearing_0", "z_bearing_1")[idx]
        z_carriage.visual(
            Box((0.034, 0.032, 0.112)),
            origin=Origin(xyz=(-0.032, y, 0.062)),
            material=rail_steel,
            name=z_bearing_name,
        )
    z_carriage.visual(
        Box((0.044, 0.076, 0.420)),
        origin=Origin(xyz=(0.020, 0.0, 0.292)),
        material=column_gray,
        name="moving_column",
    )
    z_carriage.visual(
        Box((0.070, 0.096, 0.028)),
        origin=Origin(xyz=(0.020, 0.0, 0.516)),
        material=black,
        name="head_plate",
    )
    z_carriage.visual(
        Box((0.118, 0.106, 0.026)),
        origin=Origin(xyz=(0.045, 0.0, 0.543)),
        material=rubber,
        name="head_pad",
    )

    model.articulation(
        "x_carriage_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.079, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.12, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_carriage = object_model.get_part("z_carriage")
    x_axis = object_model.get_articulation("base_to_x_carriage")
    z_axis = object_model.get_articulation("x_carriage_to_z_carriage")

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.000001,
        positive_elem="x_bearing_0",
        negative_elem="x_rail_0",
        name="x bearing rides on rail without clipping",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="x",
        min_overlap=0.20,
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        name="x carriage remains carried by the rail at rest",
    )
    ctx.expect_gap(
        z_carriage,
        x_carriage,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.000001,
        positive_elem="z_bearing_0",
        negative_elem="z_guide_rail_0",
        name="z bearing rides on guide rail without clipping",
    )
    ctx.expect_gap(
        x_carriage,
        z_carriage,
        axis="y",
        min_gap=0.004,
        positive_elem="z_stop_1",
        negative_elem="z_saddle",
        name="z carriage clears side stop blocks",
    )

    rest_x = ctx.part_world_position(x_carriage)
    rest_z = ctx.part_world_position(z_carriage)
    with ctx.pose({x_axis: 0.560, z_axis: 0.280}):
        ctx.expect_gap(
            x_carriage,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.000001,
            positive_elem="x_bearing_0",
            negative_elem="x_rail_0",
            name="extended x bearing still rides on rail",
        )
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            min_overlap=0.20,
            elem_a="x_bearing_0",
            elem_b="x_rail_0",
            name="extended x carriage stays on rail",
        )
        ctx.expect_gap(
            z_carriage,
            x_carriage,
            axis="x",
            max_gap=0.0005,
            max_penetration=0.000001,
            positive_elem="z_bearing_0",
            negative_elem="z_guide_rail_0",
            name="raised z bearing still rides on guide rail",
        )
        ctx.expect_gap(
            x_carriage,
            z_carriage,
            axis="y",
            min_gap=0.004,
            positive_elem="z_stop_1",
            negative_elem="z_saddle",
            name="raised z saddle clears upper stop",
        )
        moved_x = ctx.part_world_position(x_carriage)
        moved_z = ctx.part_world_position(z_carriage)

    ctx.check(
        "x axis travels horizontally",
        rest_x is not None and moved_x is not None and moved_x[0] > rest_x[0] + 0.50,
        details=f"rest={rest_x}, moved={moved_x}",
    )
    ctx.check(
        "z axis lifts vertically",
        rest_z is not None and moved_z is not None and moved_z[2] > rest_z[2] + 0.25,
        details=f"rest={rest_z}, moved={moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
