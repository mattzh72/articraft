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
    model = ArticulatedObject(name="bridge_backed_three_axis_stack")

    frame_paint = Material("graphite_cast_frame", color=(0.12, 0.15, 0.17, 1.0))
    way_steel = Material("dark_ground_ways", color=(0.45, 0.47, 0.47, 1.0))
    carriage_paint = Material("warm_grey_carriage", color=(0.55, 0.58, 0.56, 1.0))
    cross_paint = Material("blue_grey_cross_slide", color=(0.28, 0.34, 0.42, 1.0))
    ram_paint = Material("oxide_red_ram", color=(0.62, 0.16, 0.09, 1.0))
    tool_steel = Material("bright_tool_steel", color=(0.78, 0.78, 0.72, 1.0))
    screw_bronze = Material("oiled_bronze_screw", color=(0.70, 0.50, 0.22, 1.0))

    base_z = 0.080
    x_rail_h = 0.038
    x_rail_top = base_z + x_rail_h
    x_rail_y = 0.180
    x_rail_w = 0.036

    lower_plate_top = 0.102
    y_rail_top = 0.128
    z_rail_front_y = 0.080

    frame = model.part("frame")
    frame.visual(
        Box((1.52, 0.82, base_z)),
        origin=Origin(xyz=(0.0, 0.0, base_z / 2.0)),
        material=frame_paint,
        name="base_casting",
    )
    for y, name in ((-x_rail_y, "x_rail_front"), (x_rail_y, "x_rail_rear")):
        frame.visual(
            Box((1.36, x_rail_w, x_rail_h)),
            origin=Origin(xyz=(0.0, y, base_z + x_rail_h / 2.0)),
            material=way_steel,
            name=name,
        )
    for x, name in ((-0.710, "x_stop_0"), (0.710, "x_stop_1")):
        frame.visual(
            Box((0.045, 0.46, 0.075)),
            origin=Origin(xyz=(x, 0.0, base_z + 0.075 / 2.0)),
            material=frame_paint,
            name=name,
        )
    for x, name in ((-0.650, "rear_post_0"), (0.650, "rear_post_1")):
        frame.visual(
            Box((0.090, 0.080, 0.820)),
            origin=Origin(xyz=(x, 0.360, base_z + 0.820 / 2.0)),
            material=frame_paint,
            name=name,
        )
    frame.visual(
        Box((1.38, 0.105, 0.105)),
        origin=Origin(xyz=(0.0, 0.360, 0.880)),
        material=frame_paint,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.085, 0.035, 0.660)),
        origin=Origin(xyz=(0.0, 0.397, base_z + 0.660 / 2.0)),
        material=frame_paint,
        name="rear_backbone",
    )

    lower = model.part("lower_carriage")
    x_shoe_specs = (
        (-x_rail_y, -0.220, "x_shoe_front_0"),
        (-x_rail_y, 0.220, "x_shoe_front_1"),
        (x_rail_y, -0.220, "x_shoe_rear_0"),
        (x_rail_y, 0.220, "x_shoe_rear_1"),
    )
    for y, x, shoe_name in x_shoe_specs:
        lower.visual(
            Box((0.180, 0.074, 0.030)),
            origin=Origin(xyz=(x, y, 0.015)),
            material=carriage_paint,
            name=shoe_name,
        )
        for side_index, y_sign in enumerate((-1.0, 1.0)):
            lower.visual(
                Box((0.180, 0.008, x_rail_h)),
                origin=Origin(
                    xyz=(
                        x,
                        y + y_sign * (x_rail_w / 2.0 + 0.008 / 2.0),
                        -x_rail_h / 2.0,
                    )
                ),
                material=carriage_paint,
                name=f"{shoe_name}_lip_{side_index}",
            )
    lower.visual(
        Box((0.760, 0.560, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=carriage_paint,
        name="lower_saddle",
    )
    for x, name in ((-0.140, "y_rail_0"), (0.140, "y_rail_1")):
        lower.visual(
            Box((0.036, 0.520, 0.028)),
            origin=Origin(xyz=(x, 0.0, lower_plate_top + 0.028 / 2.0 - 0.002)),
            material=way_steel,
            name=name,
        )
    lower.visual(
        Box((0.500, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, -0.255, lower_plate_top + 0.012)),
        material=frame_paint,
        name="front_wiper",
    )

    cross = model.part("cross_slide")
    y_shoe_specs = (
        (-0.140, -0.100, "y_shoe_0_0"),
        (-0.140, 0.100, "y_shoe_0_1"),
        (0.140, -0.100, "y_shoe_1_0"),
        (0.140, 0.100, "y_shoe_1_1"),
    )
    for x, y, shoe_name in y_shoe_specs:
        cross.visual(
            Box((0.074, 0.130, 0.028)),
            origin=Origin(xyz=(x, y, 0.014)),
            material=cross_paint,
            name=shoe_name,
        )
        for side_index, x_sign in enumerate((-1.0, 1.0)):
            cross.visual(
                Box((0.008, 0.130, 0.028)),
                origin=Origin(
                    xyz=(
                        x + x_sign * (0.036 / 2.0 + 0.008 / 2.0),
                        y,
                        -0.014,
                    )
                ),
                material=cross_paint,
                name=f"{shoe_name}_lip_{side_index}",
            )
    cross.visual(
        Box((0.440, 0.300, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=cross_paint,
        name="cross_table",
    )
    cross.visual(
        Box((0.300, 0.060, 0.560)),
        origin=Origin(xyz=(0.0, 0.130, 0.372)),
        material=cross_paint,
        name="vertical_saddle",
    )
    for x, name in ((-0.100, "z_rail_0"), (0.100, "z_rail_1")):
        cross.visual(
            Box((0.034, 0.022, 0.470)),
            origin=Origin(xyz=(x, 0.091, 0.395)),
            material=way_steel,
            name=name,
        )
    cross.visual(
        Cylinder(radius=0.010, length=0.430),
        origin=Origin(xyz=(0.0, 0.098, 0.395)),
        material=screw_bronze,
        name="z_leadscrew",
    )
    cross.visual(
        Box((0.210, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.098, 0.645)),
        material=cross_paint,
        name="top_bearing",
    )

    ram = model.part("vertical_ram")
    ram.visual(
        Box((0.240, 0.075, 0.240)),
        origin=Origin(xyz=(0.0, -0.0375, 0.0)),
        material=ram_paint,
        name="ram_block",
    )
    ram.visual(
        Box((0.160, 0.210, 0.130)),
        origin=Origin(xyz=(0.0, -0.180, -0.090)),
        material=ram_paint,
        name="front_head",
    )
    ram.visual(
        Cylinder(radius=0.026, length=0.130),
        origin=Origin(xyz=(0.0, -0.260, -0.205)),
        material=tool_steel,
        name="quill",
    )
    ram.visual(
        Cylinder(radius=0.011, length=0.102),
        origin=Origin(xyz=(0.0, -0.260, -0.318)),
        material=tool_steel,
        name="tool_pin",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, x_rail_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.35, lower=-0.260, upper=0.260),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=cross,
        origin=Origin(xyz=(0.0, 0.0, y_rail_top)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.25, lower=-0.080, upper=0.080),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=cross,
        child=ram,
        origin=Origin(xyz=(0.0, z_rail_front_y, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.18, lower=-0.060, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower = object_model.get_part("lower_carriage")
    cross = object_model.get_part("cross_slide")
    ram = object_model.get_part("vertical_ram")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")
    z_slide = object_model.get_articulation("z_slide")

    for joint, axis, lower_limit, upper_limit in (
        (x_slide, (1.0, 0.0, 0.0), -0.260, 0.260),
        (y_slide, (0.0, 1.0, 0.0), -0.080, 0.080),
        (z_slide, (0.0, 0.0, 1.0), -0.060, 0.160),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is bounded prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == axis
            and limits is not None
            and limits.lower == lower_limit
            and limits.upper == upper_limit,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    ctx.expect_gap(
        lower,
        frame,
        axis="z",
        positive_elem="x_shoe_front_0",
        negative_elem="x_rail_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower carriage bears on the X way",
    )
    ctx.expect_gap(
        cross,
        lower,
        axis="z",
        positive_elem="y_shoe_0_0",
        negative_elem="y_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="cross slide bears on the Y way",
    )
    ctx.expect_gap(
        cross,
        ram,
        axis="y",
        positive_elem="z_rail_0",
        negative_elem="ram_block",
        max_gap=0.001,
        max_penetration=0.0,
        name="vertical ram rides against the Z ways",
    )

    lower_rest = ctx.part_world_position(lower)
    with ctx.pose({x_slide: 0.180}):
        lower_x = ctx.part_world_position(lower)
    ctx.check(
        "X stage translates along the long base ways",
        lower_rest is not None and lower_x is not None and lower_x[0] > lower_rest[0] + 0.170,
        details=f"rest={lower_rest}, moved={lower_x}",
    )

    cross_rest = ctx.part_world_position(cross)
    with ctx.pose({y_slide: 0.060}):
        cross_y = ctx.part_world_position(cross)
    ctx.check(
        "Y stage translates across the lower carriage",
        cross_rest is not None and cross_y is not None and cross_y[1] > cross_rest[1] + 0.050,
        details=f"rest={cross_rest}, moved={cross_y}",
    )

    ram_rest = ctx.part_world_position(ram)
    with ctx.pose({z_slide: 0.120}):
        ram_z = ctx.part_world_position(ram)
    ctx.check(
        "Z ram translates vertically on the compact saddle",
        ram_rest is not None and ram_z is not None and ram_z[2] > ram_rest[2] + 0.110,
        details=f"rest={ram_rest}, moved={ram_z}",
    )

    return ctx.report()


object_model = build_object_model()
