from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.018
PLATE_W = 0.360
PLATE_H = 0.600

Y_GUIDE_D = 0.014
Y_GUIDE_L = 0.250
Y_GUIDE_H = 0.018
Y_GUIDE_ZS = (0.070, 0.170)
Y_GUIDE_FRONT_X = (PLATE_T / 2.0) + Y_GUIDE_D
Y_AXIS_Z = sum(Y_GUIDE_ZS) / 2.0

Z_GUIDE_D = 0.012
Z_GUIDE_W = 0.016
Z_GUIDE_H = 0.280
Z_GUIDE_Y = 0.024
Z_GUIDE_CENTER_Z = -0.145
Z_GUIDE_FRONT_X = 0.032 + Z_GUIDE_D


def _frame_body_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H)
    front_relief = (
        cq.Workplane("XY")
        .box(0.008, 0.215, 0.310)
        .translate(((PLATE_T / 2.0) - 0.004, 0.0, -0.015))
    )
    mounting_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.120, -0.230),
                (0.120, -0.230),
                (-0.120, 0.230),
                (0.120, 0.230),
            ]
        )
        .circle(0.0075)
        .extrude(0.050, both=True)
    )
    screw_cover = (
        cq.Workplane("XY")
        .box(0.010, 0.210, 0.030)
        .translate(((PLATE_T / 2.0) + 0.005, 0.0, Y_AXIS_Z))
    )
    return (
        plate.edges("|Z").fillet(0.006).cut(front_relief).cut(mounting_holes).union(screw_cover)
    )


def _frame_y_guides_shape() -> cq.Workplane:
    guides = cq.Workplane("XY")
    for zc in Y_GUIDE_ZS:
        guide = (
            cq.Workplane("XY")
            .box(Y_GUIDE_D, Y_GUIDE_L, Y_GUIDE_H)
            .translate(((PLATE_T / 2.0) + (Y_GUIDE_D / 2.0), 0.0, zc))
        )
        guides = guides.union(guide)
    return guides


def _saddle_body_shape() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(0.048, 0.240, 0.052).translate((0.030, 0.0, 0.0))
    upper_bearing = cq.Workplane("XY").box(0.026, 0.072, 0.034).translate((0.013, 0.0, 0.050))
    lower_bearing = cq.Workplane("XY").box(0.026, 0.072, 0.034).translate((0.013, 0.0, -0.050))
    center_web = cq.Workplane("XY").box(0.028, 0.090, 0.170).translate((0.018, 0.0, 0.0))
    hanging_column = cq.Workplane("XY").box(0.020, 0.082, 0.255).translate((0.022, 0.0, -0.148))
    drive_housing = cq.Workplane("XY").box(0.034, 0.048, 0.034).translate((0.030, 0.102, 0.0))
    return bridge.union(upper_bearing).union(lower_bearing).union(center_web).union(hanging_column).union(
        drive_housing
    )


def _saddle_z_guides_shape() -> cq.Workplane:
    left_rail = (
        cq.Workplane("XY")
        .box(Z_GUIDE_D, Z_GUIDE_W, Z_GUIDE_H)
        .translate((0.032 + (Z_GUIDE_D / 2.0), -Z_GUIDE_Y, Z_GUIDE_CENTER_Z))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(Z_GUIDE_D, Z_GUIDE_W, Z_GUIDE_H)
        .translate((0.032 + (Z_GUIDE_D / 2.0), Z_GUIDE_Y, Z_GUIDE_CENTER_Z))
    )
    center_drive = (
        cq.Workplane("XY")
        .box(0.010, 0.020, 0.290)
        .translate((0.032 + 0.005, 0.0, Z_GUIDE_CENTER_Z))
    )
    return left_rail.union(right_rail).union(center_drive)


def _carriage_shape() -> cq.Workplane:
    left_shoe = cq.Workplane("XY").box(0.022, 0.030, 0.100).translate((0.011, -Z_GUIDE_Y, 0.0))
    right_shoe = cq.Workplane("XY").box(0.022, 0.030, 0.100).translate((0.011, Z_GUIDE_Y, 0.0))
    main_plate = cq.Workplane("XY").box(0.028, 0.092, 0.175).translate((0.036, 0.0, -0.060))
    lower_plate = cq.Workplane("XY").box(0.034, 0.060, 0.030).translate((0.040, 0.0, -0.160))
    tool_nose = cq.Workplane("XY").box(0.050, 0.024, 0.042).translate((0.048, 0.0, -0.180))
    carriage = left_shoe.union(right_shoe).union(main_plate).union(lower_plate).union(tool_nose)
    front_relief = cq.Workplane("XY").box(0.010, 0.050, 0.095).translate((0.045, 0.0, -0.050))
    return carriage.cut(front_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_positioning_stage")

    frame_color = model.material("frame_color", rgba=(0.20, 0.22, 0.24, 1.0))
    guide_color = model.material("guide_color", rgba=(0.47, 0.50, 0.53, 1.0))
    saddle_color = model.material("saddle_color", rgba=(0.70, 0.72, 0.74, 1.0))
    carriage_color = model.material("carriage_color", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_body_shape(), "frame_body"),
        material=frame_color,
        name="frame_body",
    )
    frame.visual(
        mesh_from_cadquery(_frame_y_guides_shape(), "frame_y_guides"),
        material=guide_color,
        name="frame_y_guides",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.050, PLATE_W, PLATE_H)),
        mass=12.0,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_saddle_body_shape(), "saddle_body"),
        material=saddle_color,
        name="saddle_body",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_z_guides_shape(), "saddle_z_guides"),
        material=guide_color,
        name="saddle_z_guides",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.060, 0.240, 0.330)),
        mass=4.8,
        origin=Origin(xyz=(0.026, 0.0, -0.105)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material=carriage_color,
        name="carriage",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.060, 0.095, 0.215)),
        mass=2.6,
        origin=Origin(xyz=(0.034, 0.0, -0.075)),
    )

    model.articulation(
        "frame_to_saddle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(Y_GUIDE_FRONT_X, 0.0, Y_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=-0.120, upper=0.120),
    )
    model.articulation(
        "saddle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=carriage,
        origin=Origin(xyz=(Z_GUIDE_FRONT_X, 0.0, Z_GUIDE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.30, lower=-0.140, upper=0.020),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    saddle = object_model.get_part("saddle")
    carriage = object_model.get_part("carriage")
    y_axis = object_model.get_articulation("frame_to_saddle")
    z_axis = object_model.get_articulation("saddle_to_carriage")

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
        "saddle_axis_runs_left_right_on_world_y",
        tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        f"expected (0, 1, 0), got {y_axis.axis}",
    )
    ctx.check(
        "carriage_axis_runs_vertical_on_world_z",
        tuple(z_axis.axis) == (0.0, 0.0, 1.0),
        f"expected (0, 0, 1), got {z_axis.axis}",
    )

    ctx.expect_contact(saddle, frame, name="saddle_is_supported_by_fixed_plate")
    ctx.expect_overlap(saddle, frame, axes="yz", min_overlap=0.10, name="saddle_reads_as_wall_mounted_cross_axis")
    ctx.expect_gap(
        saddle,
        frame,
        axis="x",
        min_gap=0.0,
        max_gap=0.0,
        negative_elem="frame_y_guides",
        name="saddle_bearings_seat_on_horizontal_guides",
    )

    ctx.expect_contact(carriage, saddle, name="carriage_is_supported_by_hanging_z_axis")
    ctx.expect_gap(
        carriage,
        saddle,
        axis="x",
        min_gap=0.0,
        max_gap=0.0,
        negative_elem="saddle_z_guides",
        name="carriage_shoes_seat_on_vertical_guides",
    )
    ctx.expect_origin_gap(
        saddle,
        carriage,
        axis="z",
        min_gap=0.12,
        max_gap=0.16,
        name="carriage_hangs_below_saddle",
    )

    with ctx.pose({y_axis: y_axis.motion_limits.lower}):
        ctx.expect_contact(saddle, frame, name="saddle_remains_supported_at_left_limit")
        ctx.expect_origin_gap(
            frame,
            saddle,
            axis="y",
            min_gap=0.11,
            max_gap=0.13,
            name="saddle_reaches_negative_y_travel",
        )

    with ctx.pose({y_axis: y_axis.motion_limits.upper}):
        ctx.expect_contact(saddle, frame, name="saddle_remains_supported_at_right_limit")
        ctx.expect_origin_gap(
            saddle,
            frame,
            axis="y",
            min_gap=0.11,
            max_gap=0.13,
            name="saddle_reaches_positive_y_travel",
        )

    with ctx.pose({z_axis: z_axis.motion_limits.lower}):
        ctx.expect_contact(carriage, saddle, name="carriage_remains_supported_at_low_limit")
        ctx.expect_origin_gap(
            saddle,
            carriage,
            axis="z",
            min_gap=0.26,
            max_gap=0.30,
            name="carriage_drops_down_on_z_axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
