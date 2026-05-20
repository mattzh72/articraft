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
    model = ArticulatedObject(name="wall_backed_positioning_stage")

    cast_iron = model.material("mat_cast_iron_blue_gray", rgba=(0.18, 0.23, 0.27, 1.0))
    dark_rail = model.material("mat_ground_steel_dark", rgba=(0.06, 0.065, 0.07, 1.0))
    machined = model.material("mat_machined_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    black = model.material("mat_black_oxide", rgba=(0.015, 0.015, 0.014, 1.0))
    safety_yellow = model.material("mat_axis_yellow", rgba=(0.95, 0.68, 0.12, 1.0))
    carriage_red = model.material("mat_carriage_red", rgba=(0.58, 0.08, 0.06, 1.0))

    fixed_plate = model.part("fixed_plate")
    fixed_plate.visual(
        Box((0.060, 1.800, 1.800)),
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=cast_iron,
        name="wall_plate",
    )
    fixed_plate.visual(
        Box((0.040, 1.900, 0.090)),
        origin=Origin(xyz=(-0.005, 0.0, 0.045)),
        material=cast_iron,
        name="bottom_mounting_foot",
    )
    fixed_plate.visual(
        Box((0.045, 0.050, 1.720)),
        origin=Origin(xyz=(0.010, -0.840, 0.900)),
        material=cast_iron,
        name="side_rib_0",
    )
    fixed_plate.visual(
        Box((0.045, 0.050, 1.720)),
        origin=Origin(xyz=(0.010, 0.840, 0.900)),
        material=cast_iron,
        name="side_rib_1",
    )
    fixed_plate.visual(
        Box((0.035, 1.620, 0.040)),
        origin=Origin(xyz=(0.0475, 0.0, 1.285)),
        material=dark_rail,
        name="upper_y_rail",
    )
    fixed_plate.visual(
        Box((0.035, 1.620, 0.040)),
        origin=Origin(xyz=(0.0475, 0.0, 1.025)),
        material=dark_rail,
        name="lower_y_rail",
    )
    fixed_plate.visual(
        Box((0.022, 1.500, 0.022)),
        origin=Origin(xyz=(0.041, 0.0, 1.155)),
        material=machined,
        name="y_scale_strip",
    )
    for idx, y in enumerate((-0.72, -0.36, 0.36, 0.72)):
        fixed_plate.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(0.034, y, 1.600), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"upper_anchor_{idx}",
        )
        fixed_plate.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(0.034, y, 0.320), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"lower_anchor_{idx}",
        )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.110, 0.500, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=safety_yellow,
        name="cross_saddle_body",
    )
    saddle.visual(
        Box((0.024, 0.185, 0.070)),
        origin=Origin(xyz=(-0.055, -0.135, 0.130)),
        material=machined,
        name="upper_rail_pad_0",
    )
    saddle.visual(
        Box((0.024, 0.185, 0.070)),
        origin=Origin(xyz=(-0.055, 0.135, 0.130)),
        material=machined,
        name="upper_rail_pad_1",
    )
    saddle.visual(
        Box((0.024, 0.185, 0.070)),
        origin=Origin(xyz=(-0.055, -0.135, -0.130)),
        material=machined,
        name="lower_rail_pad_0",
    )
    saddle.visual(
        Box((0.024, 0.185, 0.070)),
        origin=Origin(xyz=(-0.055, 0.135, -0.130)),
        material=machined,
        name="lower_rail_pad_1",
    )
    saddle.visual(
        Box((0.070, 0.460, 0.050)),
        origin=Origin(xyz=(0.040, 0.0, -0.180)),
        material=black,
        name="underside_wiper",
    )
    saddle.visual(
        Box((0.175, 0.210, 0.620)),
        origin=Origin(xyz=(0.095, 0.0, -0.420)),
        material=safety_yellow,
        name="hanging_web",
    )
    saddle.visual(
        Box((0.058, 0.340, 0.760)),
        origin=Origin(xyz=(0.160, 0.0, -0.485)),
        material=safety_yellow,
        name="z_guide_backbone",
    )
    saddle.visual(
        Box((0.030, 0.040, 0.750)),
        origin=Origin(xyz=(0.189, -0.120, -0.485)),
        material=dark_rail,
        name="z_rail_0",
    )
    saddle.visual(
        Box((0.030, 0.040, 0.750)),
        origin=Origin(xyz=(0.189, 0.120, -0.485)),
        material=dark_rail,
        name="z_rail_1",
    )
    saddle.visual(
        Box((0.030, 0.320, 0.045)),
        origin=Origin(xyz=(0.185, 0.0, -0.120)),
        material=black,
        name="top_z_stop",
    )
    saddle.visual(
        Box((0.030, 0.320, 0.045)),
        origin=Origin(xyz=(0.185, 0.0, -0.850)),
        material=black,
        name="bottom_z_stop",
    )

    y_axis = model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=fixed_plate,
        child=saddle,
        origin=Origin(xyz=(0.132, 0.0, 1.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.30, lower=-0.430, upper=0.430),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.070, 0.330, 0.760)),
        origin=Origin(xyz=(0.0, 0.0, -0.390)),
        material=carriage_red,
        name="vertical_carriage_plate",
    )
    carriage.visual(
        Box((0.020, 0.088, 0.160)),
        origin=Origin(xyz=(-0.041, -0.120, -0.130)),
        material=machined,
        name="upper_z_shoe_0",
    )
    carriage.visual(
        Box((0.020, 0.088, 0.160)),
        origin=Origin(xyz=(-0.041, 0.120, -0.130)),
        material=machined,
        name="upper_z_shoe_1",
    )
    carriage.visual(
        Box((0.020, 0.088, 0.160)),
        origin=Origin(xyz=(-0.041, -0.120, -0.500)),
        material=machined,
        name="lower_z_shoe_0",
    )
    carriage.visual(
        Box((0.020, 0.088, 0.160)),
        origin=Origin(xyz=(-0.041, 0.120, -0.500)),
        material=machined,
        name="lower_z_shoe_1",
    )
    carriage.visual(
        Box((0.090, 0.390, 0.130)),
        origin=Origin(xyz=(0.020, 0.0, -0.820)),
        material=carriage_red,
        name="lower_tool_lug",
    )
    carriage.visual(
        Box((0.038, 0.180, 0.300)),
        origin=Origin(xyz=(0.067, 0.0, -0.720)),
        material=black,
        name="front_tool_mount",
    )
    for idx, z in enumerate((-0.630, -0.730, -0.830)):
        carriage.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(0.090, -0.060, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name=f"tool_bolt_a_{idx}",
        )
        carriage.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(0.090, 0.060, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name=f"tool_bolt_b_{idx}",
        )

    z_axis = model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=carriage,
        origin=Origin(xyz=(0.255, 0.0, -0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.22, lower=-0.360, upper=0.060),
    )

    # Keep handles for tests readable in probes and reports.
    model.meta["primary_axes"] = (y_axis.name, z_axis.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_plate = object_model.get_part("fixed_plate")
    saddle = object_model.get_part("saddle")
    carriage = object_model.get_part("carriage")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")

    ctx.expect_gap(
        saddle,
        fixed_plate,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="upper_rail_pad_0",
        negative_elem="upper_y_rail",
        name="saddle pads sit on the wall mounted y rail",
    )
    ctx.expect_overlap(
        saddle,
        fixed_plate,
        axes="y",
        min_overlap=0.160,
        elem_a="upper_rail_pad_0",
        elem_b="upper_y_rail",
        name="saddle pad has retained length on y rail",
    )
    ctx.expect_gap(
        carriage,
        saddle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="upper_z_shoe_0",
        negative_elem="z_rail_0",
        name="carriage shoe rides the hanging z rail",
    )
    ctx.expect_overlap(
        carriage,
        saddle,
        axes="z",
        min_overlap=0.120,
        elem_a="upper_z_shoe_0",
        elem_b="z_rail_0",
        name="z carriage shoe remains captured at rest",
    )

    rest_saddle = ctx.part_world_position(saddle)
    with ctx.pose({y_axis: 0.430}):
        plus_saddle = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            fixed_plate,
            axes="y",
            min_overlap=0.080,
            elem_a="upper_rail_pad_0",
            elem_b="upper_y_rail",
            name="right travel keeps saddle on y rail",
        )
    with ctx.pose({y_axis: -0.430}):
        minus_saddle = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            fixed_plate,
            axes="y",
            min_overlap=0.080,
            elem_a="upper_rail_pad_1",
            elem_b="upper_y_rail",
            name="left travel keeps saddle on y rail",
        )
    ctx.check(
        "y_axis moves the saddle left to right",
        rest_saddle is not None
        and plus_saddle is not None
        and minus_saddle is not None
        and plus_saddle[1] > rest_saddle[1] + 0.35
        and minus_saddle[1] < rest_saddle[1] - 0.35,
        details=f"rest={rest_saddle}, plus={plus_saddle}, minus={minus_saddle}",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({z_axis: -0.360}):
        low_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            saddle,
            axes="z",
            min_overlap=0.090,
            elem_a="upper_z_shoe_0",
            elem_b="z_rail_0",
            name="lowered z carriage stays engaged with rail",
        )
    ctx.check(
        "z_axis lowers the hanging carriage",
        rest_carriage is not None
        and low_carriage is not None
        and low_carriage[2] < rest_carriage[2] - 0.30,
        details=f"rest={rest_carriage}, lowered={low_carriage}",
    )

    return ctx.report()


object_model = build_object_model()
