from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    section_loft,
    tube_from_spline_points,
)


def _add_tube_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    if abs(dy) > 1e-9:
        raise ValueError("This helper is intended for x-z plane tube placement.")
    length = math.sqrt(dx * dx + dz * dz)
    pitch = math.atan2(dx, dz)
    center = ((start[0] + end[0]) * 0.5, start[1], (start[2] + end[2]) * 0.5)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def _loop3d(points_2d: list[tuple[float, float]], y: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in points_2d]


def _yz_rounded_section(
    *,
    x: float,
    width: float,
    thickness: float,
    z_center: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, thickness, corner_radius, corner_segments=5)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_coat = model.material("frame_coat", rgba=(0.13, 0.14, 0.15, 1.0))
    housing_plastic = model.material("housing_plastic", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.09, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.54, 0.56, 0.59, 1.0))
    knob_accent = model.material("knob_accent", rgba=(0.75, 0.15, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.10, 0.58, 1.10)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )
    frame.visual(
        Box((0.54, 0.22, 0.05)),
        origin=Origin(xyz=(-0.35, 0.0, 0.025)),
        material=frame_coat,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.46, 0.22, 0.05)),
        origin=Origin(xyz=(0.39, 0.0, 0.025)),
        material=frame_coat,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.52, 0.036, 0.06)),
        origin=Origin(xyz=(-0.03, -0.09, 0.11)),
        material=frame_coat,
        name="left_lower_rail",
    )
    frame.visual(
        Box((0.52, 0.036, 0.06)),
        origin=Origin(xyz=(-0.03, 0.09, 0.11)),
        material=frame_coat,
        name="right_lower_rail",
    )
    frame.visual(
        Box((0.24, 0.18, 0.02)),
        origin=Origin(xyz=(0.02, 0.0, 0.12)),
        material=frame_coat,
        name="housing_cradle",
    )
    _add_tube_between(
        frame,
        (-0.28, -0.09, 0.05),
        (-0.22, -0.09, 0.67),
        radius=0.022,
        material=frame_coat,
        name="rear_mast_left",
    )
    _add_tube_between(
        frame,
        (-0.28, 0.09, 0.05),
        (-0.22, 0.09, 0.67),
        radius=0.022,
        material=frame_coat,
        name="rear_mast_right",
    )
    _add_tube_between(
        frame,
        (0.34, -0.09, 0.05),
        (0.24, -0.09, 0.87),
        radius=0.022,
        material=frame_coat,
        name="front_mast_left",
    )
    _add_tube_between(
        frame,
        (0.34, 0.09, 0.05),
        (0.24, 0.09, 0.87),
        radius=0.022,
        material=frame_coat,
        name="front_mast_right",
    )
    _add_tube_between(
        frame,
        (-0.22, -0.09, 0.67),
        (0.24, -0.09, 0.87),
        radius=0.018,
        material=frame_coat,
        name="top_bridge_left",
    )
    _add_tube_between(
        frame,
        (-0.22, 0.09, 0.67),
        (0.24, 0.09, 0.87),
        radius=0.018,
        material=frame_coat,
        name="top_bridge_right",
    )
    frame.visual(
        Box((0.10, 0.026, 0.20)),
        origin=Origin(xyz=(0.02, -0.135, 0.22)),
        material=frame_coat,
        name="bb_lower_web",
    )
    frame.visual(
        Box((0.10, 0.026, 0.20)),
        origin=Origin(xyz=(0.02, 0.135, 0.22)),
        material=frame_coat,
        name="bb_upper_web",
    )
    for y_side, side_name in ((-0.145, "left"), (0.145, "right")):
        frame.visual(
            Box((0.06, 0.03, 0.03)),
            origin=Origin(xyz=(0.02, y_side, 0.375)),
            material=frame_coat,
            name=f"bb_{side_name}_upper",
        )
        frame.visual(
            Box((0.06, 0.03, 0.03)),
            origin=Origin(xyz=(0.02, y_side, 0.305)),
            material=frame_coat,
            name=f"bb_{side_name}_lower",
        )
        frame.visual(
            Box((0.04, 0.03, 0.08)),
            origin=Origin(xyz=(0.02, y_side, 0.34)),
            material=frame_coat,
            name=f"bb_{side_name}_web",
        )
    frame.visual(
        Box((0.08, 0.03, 0.16)),
        origin=Origin(xyz=(0.02, -0.145, 0.22)),
        material=frame_coat,
        name="left_bb_bridge",
    )
    frame.visual(
        Box((0.08, 0.03, 0.16)),
        origin=Origin(xyz=(0.02, 0.145, 0.22)),
        material=frame_coat,
        name="right_bb_bridge",
    )
    frame.visual(
        Box((0.04, 0.036, 0.16)),
        origin=Origin(xyz=(-0.02, -0.126, 0.19)),
        material=frame_coat,
        name="left_bb_link",
    )
    frame.visual(
        Box((0.04, 0.036, 0.16)),
        origin=Origin(xyz=(-0.02, 0.126, 0.19)),
        material=frame_coat,
        name="right_bb_link",
    )
    frame.visual(
        Box((0.08, 0.18, 0.08)),
        origin=Origin(xyz=(0.24, 0.0, 0.87)),
        material=frame_coat,
        name="bar_node",
    )
    frame.visual(
        Box((0.012, 0.18, 0.12)),
        origin=Origin(xyz=(-0.248, 0.0, 0.72)),
        material=frame_coat,
        name="seat_clamp_back",
    )
    frame.visual(
        Box((0.06, 0.04, 0.11)),
        origin=Origin(xyz=(-0.22, -0.04, 0.72)),
        material=frame_coat,
        name="seat_clamp_left",
    )
    frame.visual(
        Box((0.06, 0.04, 0.11)),
        origin=Origin(xyz=(-0.22, 0.04, 0.72)),
        material=frame_coat,
        name="seat_clamp_right",
    )
    frame.visual(
        Box((0.03, 0.06, 0.02)),
        origin=Origin(xyz=(-0.175, 0.0, 0.75)),
        material=steel,
        name="seat_clamp_bolt_block",
    )
    frame.visual(
        Box((0.012, 0.18, 0.12)),
        origin=Origin(xyz=(0.205, 0.0, 0.92)),
        material=frame_coat,
        name="bar_clamp_back",
    )
    frame.visual(
        Box((0.06, 0.04, 0.11)),
        origin=Origin(xyz=(0.24, -0.04, 0.92)),
        material=frame_coat,
        name="bar_clamp_left",
    )
    frame.visual(
        Box((0.06, 0.04, 0.11)),
        origin=Origin(xyz=(0.24, 0.04, 0.92)),
        material=frame_coat,
        name="bar_clamp_right",
    )
    frame.visual(
        Box((0.03, 0.06, 0.02)),
        origin=Origin(xyz=(0.285, 0.0, 0.95)),
        material=steel,
        name="bar_clamp_bolt_block",
    )

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((0.50, 0.14, 0.54)),
        mass=5.5,
        origin=Origin(xyz=(0.03, 0.0, 0.34)),
    )
    housing_outline = sample_catmull_rom_spline_2d(
        [
            (-0.16, 0.12),
            (-0.19, 0.25),
            (-0.17, 0.45),
            (-0.08, 0.57),
            (0.08, 0.60),
            (0.22, 0.53),
            (0.28, 0.39),
            (0.25, 0.23),
            (0.15, 0.11),
            (0.00, 0.08),
            (-0.11, 0.09),
        ],
        samples_per_segment=5,
        closed=True,
    )
    left_panel = section_loft([_loop3d(housing_outline, -0.060), _loop3d(housing_outline, -0.056)])
    right_panel = section_loft([_loop3d(housing_outline, 0.056), _loop3d(housing_outline, 0.060)])
    housing.visual(mesh_from_geometry(left_panel, "housing_left_panel"), material=housing_plastic, name="left_shell")
    housing.visual(mesh_from_geometry(right_panel, "housing_right_panel"), material=housing_plastic, name="right_shell")
    housing.visual(
        Box((0.20, 0.018, 0.030)),
        origin=Origin(xyz=(0.02, -0.053, 0.145)),
        material=housing_plastic,
        name="mount_flange_left",
    )
    housing.visual(
        Box((0.20, 0.018, 0.030)),
        origin=Origin(xyz=(0.02, 0.053, 0.145)),
        material=housing_plastic,
        name="mount_flange_right",
    )
    housing.visual(
        Box((0.14, 0.116, 0.014)),
        origin=Origin(xyz=(0.10, 0.0, 0.558)),
        material=housing_plastic,
        name="knob_pad",
    )
    housing.visual(
        Box((0.08, 0.116, 0.020)),
        origin=Origin(xyz=(-0.13, 0.0, 0.56)),
        material=housing_plastic,
        name="rear_pad",
    )
    housing.visual(
        Box((0.09, 0.012, 0.020)),
        origin=Origin(xyz=(0.17, 0.054, 0.32)),
        material=dark_trim,
        name="service_cover",
    )
    drive = model.part("drive_assembly")
    drive.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.22),
        mass=11.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    drive.visual(
        Cylinder(radius=0.015, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    drive.visual(
        Cylinder(radius=0.20, length=0.045),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="flywheel",
    )
    drive.visual(
        Cylinder(radius=0.075, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_spider",
    )
    drive.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.0, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_hub",
    )
    drive.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.0, 0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_hub",
    )
    _add_tube_between(
        drive,
        (0.0, -0.097, 0.0),
        (-0.09, -0.097, 0.14),
        radius=0.012,
        material=steel,
        name="left_arm",
    )
    _add_tube_between(
        drive,
        (0.0, 0.097, 0.0),
        (0.09, 0.097, -0.14),
        radius=0.012,
        material=steel,
        name="right_arm",
    )
    drive.visual(
        Cylinder(radius=0.006, length=0.07),
        origin=Origin(xyz=(-0.09, -0.097, 0.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_spindle",
    )
    drive.visual(
        Cylinder(radius=0.006, length=0.07),
        origin=Origin(xyz=(0.09, 0.097, -0.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_spindle",
    )
    drive.visual(
        Box((0.10, 0.035, 0.016)),
        origin=Origin(xyz=(-0.07, -0.097, 0.14)),
        material=dark_trim,
        name="left_pedal",
    )
    drive.visual(
        Box((0.10, 0.035, 0.016)),
        origin=Origin(xyz=(0.11, 0.097, -0.14)),
        material=dark_trim,
        name="right_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.28, 0.18, 0.58)),
        mass=3.4,
        origin=Origin(xyz=(0.04, 0.0, 0.26)),
    )
    seat_post.visual(
        Box((0.032, 0.024, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=steel,
        name="post",
    )
    seat_post.visual(
        Box((0.10, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=steel,
        name="saddle_head",
    )
    _add_tube_between(
        seat_post,
        (-0.03, -0.030, 0.39),
        (0.09, -0.030, 0.41),
        radius=0.005,
        material=steel,
        name="left_rail",
    )
    _add_tube_between(
        seat_post,
        (-0.03, 0.030, 0.39),
        (0.09, 0.030, 0.41),
        radius=0.005,
        material=steel,
        name="right_rail",
    )
    saddle_geom = section_loft(
        [
            _yz_rounded_section(x=-0.07, width=0.17, thickness=0.038, z_center=0.43, corner_radius=0.015),
            _yz_rounded_section(x=0.00, width=0.15, thickness=0.052, z_center=0.445, corner_radius=0.018),
            _yz_rounded_section(x=0.11, width=0.07, thickness=0.028, z_center=0.435, corner_radius=0.012),
            _yz_rounded_section(x=0.18, width=0.04, thickness=0.016, z_center=0.425, corner_radius=0.007),
        ]
    )
    seat_post.visual(
        mesh_from_geometry(saddle_geom, "exercise_bike_saddle"),
        material=saddle_vinyl,
        name="saddle",
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.34, 0.48, 0.34)),
        mass=3.0,
        origin=Origin(xyz=(-0.04, 0.0, 0.16)),
    )
    handlebar.visual(
        Box((0.034, 0.024, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=steel,
        name="stem",
    )
    handlebar.visual(
        Box((0.08, 0.05, 0.020)),
        origin=Origin(xyz=(-0.01, 0.0, 0.24)),
        material=steel,
        name="bar_bridge",
    )
    bar_geom = tube_from_spline_points(
        [
            (-0.12, -0.21, 0.15),
            (-0.10, -0.16, 0.22),
            (-0.06, -0.08, 0.27),
            (0.0, 0.0, 0.27),
            (-0.06, 0.08, 0.27),
            (-0.10, 0.16, 0.22),
            (-0.12, 0.21, 0.15),
        ],
        radius=0.015,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    handlebar.visual(mesh_from_geometry(bar_geom, "exercise_bike_bar"), material=steel, name="bar_loop")
    handlebar.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.115, -0.205, 0.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.115, 0.205, 0.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="right_grip",
    )

    resistance_knob = model.part("resistance_knob")
    resistance_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.060),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    resistance_knob.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="shaft",
    )
    resistance_knob.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=knob_accent,
        name="knob_body",
    )
    resistance_knob.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_accent,
        name="knob_handle",
    )

    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_drive",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=drive,
        origin=Origin(xyz=(0.02, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.226, 0.0, 0.61)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.20),
    )
    model.articulation(
        "frame_to_handlebar",
        ArticulationType.FIXED,
        parent=frame,
        child=handlebar,
        origin=Origin(xyz=(0.182, 0.0, 0.84)),
    )
    model.articulation(
        "housing_to_resistance_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=resistance_knob,
        origin=Origin(xyz=(0.10, 0.0, 0.565)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    housing = object_model.get_part("housing")
    drive = object_model.get_part("drive_assembly")
    seat_post = object_model.get_part("seat_post")
    handlebar = object_model.get_part("handlebar")
    resistance_knob = object_model.get_part("resistance_knob")
    crank_spin = object_model.get_articulation("frame_to_drive")
    seat_adjust = object_model.get_articulation("frame_to_seat_post")
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
    ctx.allow_overlap(
        housing,
        resistance_knob,
        elem_a="knob_pad",
        elem_b="shaft",
        reason="adjuster shaft intentionally passes through the molded support boss opening",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        housing,
        frame,
        elem_a="mount_flange_left",
        elem_b="housing_cradle",
        name="left_housing_mount_flange_bolts_to_cradle",
    )
    ctx.expect_contact(
        housing,
        frame,
        elem_a="mount_flange_right",
        elem_b="housing_cradle",
        name="right_housing_mount_flange_bolts_to_cradle",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="yz",
        min_overlap=0.02,
        elem_a="post",
        elem_b="seat_clamp_back",
        name="seat_post_runs_in_open_clamp_channel",
    )
    ctx.expect_overlap(
        handlebar,
        frame,
        axes="yz",
        min_overlap=0.02,
        elem_a="stem",
        elem_b="bar_clamp_back",
        name="bar_stem_runs_in_open_clamp_channel",
    )
    ctx.expect_contact(
        resistance_knob,
        housing,
        elem_a="shaft",
        elem_b="knob_pad",
        name="resistance_knob_sits_on_knob_pad",
    )

    with ctx.pose({seat_adjust: 0.18}):
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="yz",
            min_overlap=0.02,
            elem_a="post",
            elem_b="seat_clamp_back",
            name="seat_post_stays_guided_when_raised",
        )

    with ctx.pose({crank_spin: 0.0}):
        pedal_rest = _aabb_center(ctx.part_element_world_aabb(drive, elem="right_pedal"))
    with ctx.pose({crank_spin: math.pi / 2.0}):
        pedal_quarter = _aabb_center(ctx.part_element_world_aabb(drive, elem="right_pedal"))
    ctx.check(
        "crank_rotation_lifts_right_pedal",
        pedal_rest is not None and pedal_quarter is not None and pedal_quarter[2] > pedal_rest[2] + 0.18,
        details=f"rest={pedal_rest}, quarter={pedal_quarter}",
    )

    with ctx.pose({seat_adjust: 0.0}):
        saddle_low = _aabb_center(ctx.part_element_world_aabb(seat_post, elem="saddle"))
    with ctx.pose({seat_adjust: 0.20}):
        saddle_high = _aabb_center(ctx.part_element_world_aabb(seat_post, elem="saddle"))
    ctx.check(
        "seat_adjuster_raises_saddle",
        saddle_low is not None and saddle_high is not None and saddle_high[2] > saddle_low[2] + 0.18,
        details=f"low={saddle_low}, high={saddle_high}",
    )

    axle_aabb = ctx.part_element_world_aabb(drive, elem="axle")
    left_upper = ctx.part_element_world_aabb(frame, elem="bb_left_upper")
    left_lower = ctx.part_element_world_aabb(frame, elem="bb_left_lower")
    axle_center = _aabb_center(axle_aabb)
    upper_center = _aabb_center(left_upper)
    lower_center = _aabb_center(left_lower)
    bearing_ok = (
        axle_center is not None
        and upper_center is not None
        and lower_center is not None
        and left_upper is not None
        and left_lower is not None
        and abs(axle_center[0] - upper_center[0]) < 0.03
        and left_lower[1][2] < axle_center[2] - 0.01
        and left_upper[0][2] > axle_center[2] + 0.01
    )
    ctx.check(
        "axle_runs_in_explicit_bearing_yoke_gap",
        bearing_ok,
        details=f"axle={axle_aabb}, upper={left_upper}, lower={left_lower}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
