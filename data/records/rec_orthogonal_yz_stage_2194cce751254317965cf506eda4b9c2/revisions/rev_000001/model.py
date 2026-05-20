from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_WIDTH = 0.48
FRAME_HEIGHT = 0.62
FRAME_FOOTPRINT_X = 0.06

Y_RAIL_LENGTH = 0.34
Y_RAIL_X = 0.012
Y_JOINT_X = 0.018
Y_RAIL_TOP_Z = 0.46
Y_RAIL_BOTTOM_Z = 0.38
Y_RAIL_MID_Z = 0.5 * (Y_RAIL_TOP_Z + Y_RAIL_BOTTOM_Z)
Y_RAIL_SIZE_X = 0.012
Y_RAIL_SIZE_Z = 0.014
Y_TRAVEL = 0.24

Z_RAIL_CENTER_X_LOCAL = 0.060
Z_JOINT_X_LOCAL = 0.066
Z_RAIL_Y_OFFSET = 0.038
Z_RAIL_TOP_Z_LOCAL = 0.0
Z_RAIL_LENGTH = 0.24
Z_RAIL_SIZE = 0.012
Z_TRAVEL = 0.16


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_yz_stage")

    model.material("frame_gray", rgba=(0.31, 0.34, 0.37, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("slide_light", rgba=(0.78, 0.79, 0.82, 1.0))

    frame = model.part("back_frame")
    _add_box(frame, (0.060, 0.340, 0.050), (-0.010, 0.0, 0.025), "frame_gray", "base_block")
    _add_box(frame, (0.012, 0.430, 0.500), (-0.024, 0.0, 0.300), "frame_gray", "back_plate")
    _add_box(frame, (0.030, 0.050, 0.500), (-0.015, -0.215, 0.300), "frame_gray", "left_upright")
    _add_box(frame, (0.030, 0.050, 0.500), (-0.015, 0.215, 0.300), "frame_gray", "right_upright")
    _add_box(frame, (0.030, FRAME_WIDTH, 0.040), (-0.015, 0.0, 0.560), "frame_gray", "top_tie")
    _add_box(frame, (0.030, 0.370, 0.020), (-0.003, 0.0, Y_RAIL_TOP_Z), "frame_gray", "upper_rail_support")
    _add_box(frame, (0.030, 0.370, 0.020), (-0.003, 0.0, Y_RAIL_BOTTOM_Z), "frame_gray", "lower_rail_support")
    _add_box(frame, (Y_RAIL_SIZE_X, Y_RAIL_LENGTH, Y_RAIL_SIZE_Z), (Y_RAIL_X, 0.0, Y_RAIL_TOP_Z), "rail_steel", "upper_y_rail")
    _add_box(
        frame,
        (Y_RAIL_SIZE_X, Y_RAIL_LENGTH, Y_RAIL_SIZE_Z),
        (Y_RAIL_X, 0.0, Y_RAIL_BOTTOM_Z),
        "rail_steel",
        "lower_y_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_FOOTPRINT_X, FRAME_WIDTH, FRAME_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(-0.010, 0.0, FRAME_HEIGHT / 2.0)),
    )

    y_carriage = model.part("y_carriage")
    _add_box(y_carriage, (0.024, 0.118, 0.042), (0.012, 0.0, 0.040), "carriage_dark", "upper_bearing")
    _add_box(y_carriage, (0.024, 0.118, 0.042), (0.012, 0.0, -0.040), "carriage_dark", "lower_bearing")
    _add_box(y_carriage, (0.016, 0.104, 0.280), (0.032, 0.0, -0.070), "carriage_dark", "bridge_column")
    _add_box(y_carriage, (0.012, 0.090, 0.250), (0.044, 0.0, -0.090), "carriage_dark", "front_web")
    _add_box(y_carriage, (0.016, 0.086, 0.030), (0.052, 0.0, -0.015), "carriage_dark", "z_head_mount")
    _add_box(y_carriage, (0.012, 0.078, 0.026), (0.050, 0.0, -0.220), "carriage_dark", "z_lower_mount")
    _add_box(
        y_carriage,
        (Z_RAIL_SIZE, Z_RAIL_SIZE, Z_RAIL_LENGTH),
        (Z_RAIL_CENTER_X_LOCAL, -Z_RAIL_Y_OFFSET, -0.120),
        "rail_steel",
        "left_z_rail",
    )
    _add_box(
        y_carriage,
        (Z_RAIL_SIZE, Z_RAIL_SIZE, Z_RAIL_LENGTH),
        (Z_RAIL_CENTER_X_LOCAL, Z_RAIL_Y_OFFSET, -0.120),
        "rail_steel",
        "right_z_rail",
    )
    y_carriage.inertial = Inertial.from_geometry(
        Box((0.070, 0.120, 0.320)),
        mass=4.8,
        origin=Origin(xyz=(0.035, 0.0, -0.080)),
    )

    z_carriage = model.part("z_carriage")
    _add_box(z_carriage, (0.024, 0.026, 0.120), (0.012, -Z_RAIL_Y_OFFSET, -0.080), "slide_light", "left_guide_block")
    _add_box(z_carriage, (0.024, 0.026, 0.120), (0.012, Z_RAIL_Y_OFFSET, -0.080), "slide_light", "right_guide_block")
    _add_box(z_carriage, (0.016, 0.090, 0.150), (0.032, 0.0, -0.080), "slide_light", "tool_bridge")
    _add_box(z_carriage, (0.010, 0.070, 0.120), (0.037, 0.0, -0.200), "slide_light", "tool_plate")
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.050, 0.100, 0.280)),
        mass=2.8,
        origin=Origin(xyz=(0.025, 0.0, -0.140)),
    )

    model.articulation(
        "frame_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=y_carriage,
        origin=Origin(xyz=(Y_JOINT_X, 0.0, Y_RAIL_MID_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.5 * Y_TRAVEL,
            upper=0.5 * Y_TRAVEL,
            effort=320.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "y_carriage_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(Z_JOINT_X_LOCAL, 0.0, Z_RAIL_TOP_Z_LOCAL)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=220.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("back_frame")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    y_slide = object_model.get_articulation("frame_to_y_carriage")
    z_slide = object_model.get_articulation("y_carriage_to_z_carriage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "y_slide_axis_is_horizontal_y",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {y_slide.axis}",
    )
    ctx.check(
        "z_slide_axis_is_vertical",
        tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        details=f"expected (0, 0, -1), got {z_slide.axis}",
    )

    ctx.expect_contact(frame, y_carriage, contact_tol=2e-4, name="frame_supports_y_carriage")
    ctx.expect_contact(y_carriage, z_carriage, contact_tol=2e-4, name="y_carriage_supports_z_carriage")
    ctx.expect_gap(z_carriage, frame, axis="x", min_gap=0.01, name="z_carriage_hangs_in_front_of_back_frame")

    with ctx.pose({y_slide: -0.5 * Y_TRAVEL}):
        y_low = ctx.part_world_position(y_carriage)
    with ctx.pose({y_slide: 0.5 * Y_TRAVEL}):
        y_high = ctx.part_world_position(y_carriage)
    if y_low is None or y_high is None:
        ctx.fail("y_slide_world_positions_resolve", "could not resolve y-carriage positions")
    else:
        ctx.check(
            "y_slide_translates_along_y",
            abs((y_high[1] - y_low[1]) - Y_TRAVEL) <= 1e-4
            and abs(y_high[0] - y_low[0]) <= 1e-4
            and abs(y_high[2] - y_low[2]) <= 1e-4,
            details=f"low={y_low}, high={y_high}",
        )

    with ctx.pose({z_slide: 0.0}):
        z_home = ctx.part_world_position(z_carriage)
    with ctx.pose({z_slide: Z_TRAVEL}):
        z_down = ctx.part_world_position(z_carriage)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_z_slide_extended")
    if z_home is None or z_down is None:
        ctx.fail("z_slide_world_positions_resolve", "could not resolve z-carriage positions")
    else:
        ctx.check(
            "z_slide_translates_downward",
            abs((z_home[2] - z_down[2]) - Z_TRAVEL) <= 1e-4
            and abs(z_home[0] - z_down[0]) <= 1e-4
            and abs(z_home[1] - z_down[1]) <= 1e-4,
            details=f"home={z_home}, down={z_down}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
