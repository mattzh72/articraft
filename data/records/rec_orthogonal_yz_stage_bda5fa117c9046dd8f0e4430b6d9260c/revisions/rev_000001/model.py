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
    model = ArticulatedObject(name="orthogonal_yz_stage")

    aluminum = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("ground_steel_rails", rgba=(0.64, 0.66, 0.68, 1.0))
    carriage_blue = model.material("blue_bearing_blocks", rgba=(0.05, 0.16, 0.38, 1.0))
    plate = model.material("brushed_mount_plate", rgba=(0.34, 0.36, 0.38, 1.0))
    screw = model.material("black_socket_screws", rgba=(0.015, 0.015, 0.018, 1.0))
    stop = model.material("red_travel_stops", rgba=(0.75, 0.05, 0.04, 1.0))

    def box(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def face_screw(part, name: str, xyz: tuple[float, float, float], radius: float = 0.010):
        part.visual(
            Cylinder(radius=radius, length=0.006),
            origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
            material=screw,
            name=name,
        )

    base = model.part("base_frame")
    # A fixed rectangular Y-Z support frame.  The two polished horizontal rails
    # are the lateral guideways that carry the first prismatic stage.
    box(base, "bottom_frame_rail", (0.080, 0.860, 0.060), (0.000, 0.000, 0.030), aluminum)
    box(base, "top_frame_rail", (0.080, 0.860, 0.060), (0.000, 0.000, 0.750), aluminum)
    box(base, "side_frame_post_0", (0.080, 0.060, 0.780), (0.000, -0.400, 0.390), aluminum)
    box(base, "side_frame_post_1", (0.080, 0.060, 0.780), (0.000, 0.400, 0.390), aluminum)
    box(base, "lower_y_rail", (0.035, 0.760, 0.028), (-0.0525, 0.000, 0.270), steel)
    box(base, "upper_y_rail", (0.035, 0.760, 0.028), (-0.0525, 0.000, 0.530), steel)
    box(base, "lower_stop_0", (0.040, 0.022, 0.050), (-0.087, -0.358, 0.270), stop)
    box(base, "lower_stop_1", (0.040, 0.022, 0.050), (-0.087, 0.358, 0.270), stop)
    box(base, "upper_stop_0", (0.040, 0.022, 0.050), (-0.087, -0.358, 0.530), stop)
    box(base, "upper_stop_1", (0.040, 0.022, 0.050), (-0.087, 0.358, 0.530), stop)
    box(base, "foot_0", (0.180, 0.160, 0.030), (0.030, -0.260, 0.015), aluminum)
    box(base, "foot_1", (0.180, 0.160, 0.030), (0.030, 0.260, 0.015), aluminum)
    for z, prefix in ((0.270, "lower"), (0.530, "upper")):
        for idx, y in enumerate((-0.260, 0.000, 0.260)):
            face_screw(base, f"{prefix}_rail_screw_{idx}", (-0.072, y, z), radius=0.007)

    y_carriage = model.part("y_carriage")
    # Four bearing blocks ride on the fixed Y rails and bolt to a moving
    # rectangular support frame.  The vertical guide rails for the second slide
    # are fixed to this laterally moving frame.
    for z, prefix in ((0.270, "lower"), (0.530, "upper")):
        for idx, y in enumerate((-0.130, 0.130)):
            box(
                y_carriage,
                f"{prefix}_y_block_{idx}",
                (0.050, 0.120, 0.060),
                (-0.095, y, z),
                carriage_blue,
            )
    box(y_carriage, "lower_carriage_rail", (0.028, 0.430, 0.046), (-0.132, 0.000, 0.270), aluminum)
    box(y_carriage, "upper_carriage_rail", (0.028, 0.430, 0.046), (-0.132, 0.000, 0.530), aluminum)
    box(y_carriage, "carriage_post_0", (0.028, 0.040, 0.360), (-0.132, -0.205, 0.400), aluminum)
    box(y_carriage, "carriage_post_1", (0.028, 0.040, 0.360), (-0.132, 0.205, 0.400), aluminum)
    box(y_carriage, "vertical_z_rail_0", (0.024, 0.026, 0.480), (-0.158, -0.105, 0.400), steel)
    box(y_carriage, "vertical_z_rail_1", (0.024, 0.026, 0.480), (-0.158, 0.105, 0.400), steel)
    box(y_carriage, "z_lower_stop", (0.038, 0.260, 0.024), (-0.188, 0.000, 0.150), stop)
    box(y_carriage, "z_upper_stop", (0.038, 0.260, 0.024), (-0.188, 0.000, 0.650), stop)
    for y, prefix in ((-0.105, "z_rail_0"), (0.105, "z_rail_1")):
        for idx, z in enumerate((0.240, 0.400, 0.560)):
            face_screw(y_carriage, f"{prefix}_screw_{idx}", (-0.171, y, z), radius=0.0065)

    z_carriage = model.part("z_carriage")
    # The second slide is a compact vertical carriage with guide blocks on both
    # vertical rails and a front tool plate carried by a small rectangular frame.
    for y, prefix in ((-0.105, "rail_0"), (0.105, "rail_1")):
        for idx, z in enumerate((-0.070, 0.070)):
            box(
                z_carriage,
                f"{prefix}_z_block_{idx}",
                (0.044, 0.062, 0.065),
                (-0.192, y, z),
                carriage_blue,
            )
    box(z_carriage, "lower_tool_frame", (0.030, 0.270, 0.036), (-0.227, 0.000, -0.125), plate)
    box(z_carriage, "upper_tool_frame", (0.030, 0.270, 0.036), (-0.227, 0.000, 0.125), plate)
    box(z_carriage, "tool_frame_post_0", (0.030, 0.036, 0.250), (-0.227, -0.135, 0.000), plate)
    box(z_carriage, "tool_frame_post_1", (0.030, 0.036, 0.250), (-0.227, 0.135, 0.000), plate)
    box(z_carriage, "tool_plate_bridge_0", (0.036, 0.070, 0.050), (-0.245, -0.095, 0.000), plate)
    box(z_carriage, "tool_plate_bridge_1", (0.036, 0.070, 0.050), (-0.245, 0.095, 0.000), plate)
    box(z_carriage, "tool_plate", (0.044, 0.130, 0.110), (-0.260, 0.000, 0.000), plate)
    for iy, y in enumerate((-0.040, 0.040)):
        for iz, z in enumerate((-0.035, 0.035)):
            face_screw(z_carriage, f"tool_screw_{iy}_{iz}", (-0.284, y, z), radius=0.008)

    model.articulation(
        "base_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_carriage,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.140, upper=0.140),
    )
    model.articulation(
        "y_carriage_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.25, lower=-0.120, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    y_slide = object_model.get_articulation("base_to_y_carriage")
    z_slide = object_model.get_articulation("y_carriage_to_z_carriage")

    ctx.check(
        "stage has orthogonal Y and Z prismatic joints",
        y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"y_axis={y_slide.axis}, z_axis={z_slide.axis}",
    )

    ctx.expect_contact(
        y_carriage,
        base,
        elem_a="lower_y_block_0",
        elem_b="lower_y_rail",
        name="lateral bearing block rides on lower rail",
    )
    ctx.expect_contact(
        z_carriage,
        y_carriage,
        elem_a="rail_0_z_block_0",
        elem_b="vertical_z_rail_0",
        name="vertical bearing block rides on vertical rail",
    )

    ctx.expect_within(
        y_carriage,
        base,
        axes="y",
        inner_elem="upper_y_block_1",
        outer_elem="upper_y_rail",
        name="lateral block is retained on the upper rail at center",
    )
    ctx.expect_within(
        z_carriage,
        y_carriage,
        axes="z",
        inner_elem="rail_1_z_block_1",
        outer_elem="vertical_z_rail_1",
        name="vertical block is retained on the right rail at center",
    )

    rest_y = ctx.part_world_position(y_carriage)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        ctx.expect_within(
            y_carriage,
            base,
            axes="y",
            inner_elem="upper_y_block_1",
            outer_elem="upper_y_rail",
            name="lateral block stays on rail at positive travel",
        )
        positive_y = ctx.part_world_position(y_carriage)
    with ctx.pose({y_slide: y_slide.motion_limits.lower}):
        ctx.expect_within(
            y_carriage,
            base,
            axes="y",
            inner_elem="lower_y_block_0",
            outer_elem="lower_y_rail",
            name="lateral block stays on rail at negative travel",
        )
    ctx.check(
        "lateral slide translates along positive Y at upper limit",
        rest_y is not None and positive_y is not None and positive_y[1] > rest_y[1] + 0.10,
        details=f"rest={rest_y}, upper={positive_y}",
    )

    rest_z = ctx.part_world_position(z_carriage)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        ctx.expect_within(
            z_carriage,
            y_carriage,
            axes="z",
            inner_elem="rail_1_z_block_1",
            outer_elem="vertical_z_rail_1",
            name="vertical block stays on rail at upper travel",
        )
        upper_z = ctx.part_world_position(z_carriage)
    with ctx.pose({z_slide: z_slide.motion_limits.lower}):
        ctx.expect_within(
            z_carriage,
            y_carriage,
            axes="z",
            inner_elem="rail_0_z_block_0",
            outer_elem="vertical_z_rail_0",
            name="vertical block stays on rail at lower travel",
        )
    ctx.check(
        "vertical slide translates upward at upper limit",
        rest_z is not None and upper_z is not None and upper_z[2] > rest_z[2] + 0.08,
        details=f"rest={rest_z}, upper={upper_z}",
    )

    return ctx.report()


object_model = build_object_model()
