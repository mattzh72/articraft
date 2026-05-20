from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, radians, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_drive_wheel(
    part,
    *,
    side_sign: float,
    tire_mesh,
    rim_mesh,
    pushrim_mesh,
    rubber,
    aluminum,
    dark_metal,
) -> None:
    rim_x = 0.047 * side_sign
    pushrim_x = 0.079 * side_sign
    hub_x = 0.012 * side_sign

    part.visual(tire_mesh, origin=Origin(xyz=(rim_x, 0.0, 0.0)), material=rubber, name="tire")
    part.visual(rim_mesh, origin=Origin(xyz=(rim_x, 0.0, 0.0)), material=aluminum, name="rim")
    part.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(hub_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_sleeve",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.024 * side_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="hub_cap",
    )

    for angle_deg in (15.0, 75.0, 135.0, 195.0, 255.0, 315.0):
        angle = radians(angle_deg)
        outer_angle = radians(angle_deg + 10.0)
        inner = (
            0.016 * side_sign,
            cos(angle) * 0.028,
            sin(angle) * 0.028,
        )
        outer = (
            rim_x,
            cos(outer_angle) * 0.243,
            sin(outer_angle) * 0.243,
        )
        _add_member(part, inner, outer, 0.0045, aluminum)

    part.visual(
        pushrim_mesh,
        origin=Origin(xyz=(pushrim_x, 0.0, 0.0)),
        material=aluminum,
        name="pushrim",
    )
    for angle_deg in (40.0, 160.0, 280.0):
        angle = radians(angle_deg)
        _add_member(
            part,
            (0.052 * side_sign, cos(angle) * 0.250, sin(angle) * 0.250),
            (pushrim_x, cos(angle) * 0.238, sin(angle) * 0.238),
            0.0032,
            aluminum,
        )


def _add_caster_wheel(part, *, tire_mesh, rim_mesh, rubber, aluminum, dark_metal) -> None:
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(rim_mesh, material=aluminum, name="rim")
    part.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = radians(angle_deg)
        inner = (0.0, cos(angle) * 0.015, sin(angle) * 0.015)
        outer = (0.0, cos(angle) * 0.048, sin(angle) * 0.048)
        _add_member(part, inner, outer, 0.0035, aluminum)


def _add_caster_fork(part, *, frame_paint, polymer_dark) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=frame_paint,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=polymer_dark,
        name="crown",
    )
    _add_member(part, (-0.018, -0.004, -0.072), (0.018, -0.004, -0.072), 0.0070, frame_paint)
    _add_member(part, (0.018, -0.008, -0.072), (0.018, -0.034, -0.150), 0.0065, frame_paint)
    _add_member(part, (-0.018, -0.008, -0.072), (-0.018, -0.034, -0.150), 0.0065, frame_paint)
    part.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.012, -0.034, -0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="fork_boss_outer",
    )
    part.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(-0.012, -0.034, -0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="fork_boss_inner",
    )


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-9 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.42, 0.45, 0.49, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    upholstery = model.material("upholstery", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    drive_tire_mesh = _save_mesh(
        "wheelchair_drive_tire",
        TorusGeometry(radius=0.285, tube=0.025, radial_segments=18, tubular_segments=52).rotate_y(pi / 2.0),
    )
    drive_rim_mesh = _save_mesh(
        "wheelchair_drive_rim",
        TorusGeometry(radius=0.255, tube=0.012, radial_segments=14, tubular_segments=44).rotate_y(pi / 2.0),
    )
    pushrim_mesh = _save_mesh(
        "wheelchair_pushrim",
        TorusGeometry(radius=0.238, tube=0.0065, radial_segments=12, tubular_segments=40).rotate_y(pi / 2.0),
    )
    caster_tire_mesh = _save_mesh(
        "wheelchair_caster_tire",
        TorusGeometry(radius=0.056, tube=0.014, radial_segments=14, tubular_segments=36).rotate_y(pi / 2.0),
    )
    caster_rim_mesh = _save_mesh(
        "wheelchair_caster_rim",
        TorusGeometry(radius=0.049, tube=0.008, radial_segments=12, tubular_segments=28).rotate_y(pi / 2.0),
    )
    seat_mesh = _save_mesh(
        "wheelchair_seat_panel",
        ExtrudeGeometry(
            rounded_rect_profile(0.43, 0.38, 0.035, corner_segments=8),
            0.020,
            center=True,
        ),
    )
    back_mesh = _save_mesh(
        "wheelchair_back_panel",
        ExtrudeGeometry(
            rounded_rect_profile(0.42, 0.30, 0.032, corner_segments=8),
            0.014,
            center=True,
        ).rotate_x(pi / 2.0),
    )
    footplate_mesh = _save_mesh(
        "wheelchair_footplate",
        ExtrudeGeometry(
            rounded_rect_profile(0.12, 0.08, 0.014, corner_segments=6),
            0.008,
            center=True,
        ),
    )

    frame = model.part("frame")

    left_upper_rail = tube_from_spline_points(
        [
            (0.215, -0.215, 0.785),
            (0.215, -0.145, 0.610),
            (0.215, -0.070, 0.445),
            (0.210, 0.185, 0.445),
            (0.215, 0.305, 0.190),
        ],
        radius=0.0135,
        samples_per_segment=16,
        radial_segments=20,
    )
    right_upper_rail = tube_from_spline_points(
        _mirror_x(
            [
                (0.215, -0.215, 0.785),
                (0.215, -0.145, 0.610),
                (0.215, -0.070, 0.445),
                (0.210, 0.185, 0.445),
                (0.215, 0.305, 0.190),
            ]
        ),
        radius=0.0135,
        samples_per_segment=16,
        radial_segments=20,
    )
    left_lower_rail = tube_from_spline_points(
        [
            (0.185, 0.160, 0.400),
            (0.195, 0.065, 0.360),
            (0.215, -0.010, 0.330),
            (0.240, -0.055, 0.315),
        ],
        radius=0.0115,
        samples_per_segment=14,
        radial_segments=18,
    )
    right_lower_rail = tube_from_spline_points(
        _mirror_x(
            [
                (0.185, 0.160, 0.400),
                (0.195, 0.065, 0.360),
                (0.215, -0.010, 0.330),
                (0.240, -0.055, 0.315),
            ]
        ),
        radius=0.0115,
        samples_per_segment=14,
        radial_segments=18,
    )
    left_rear_brace = tube_from_spline_points(
        [
            (0.215, -0.070, 0.445),
            (0.228, -0.064, 0.380),
            (0.240, -0.055, 0.315),
        ],
        radius=0.0115,
        samples_per_segment=10,
        radial_segments=16,
    )
    right_rear_brace = tube_from_spline_points(
        _mirror_x(
            [
                (0.215, -0.070, 0.445),
                (0.228, -0.064, 0.380),
                (0.240, -0.055, 0.315),
            ]
        ),
        radius=0.0115,
        samples_per_segment=10,
        radial_segments=16,
    )
    left_footrest_hanger = tube_from_spline_points(
        [
            (0.185, 0.160, 0.400),
            (0.142, 0.230, 0.250),
            (0.105, 0.285, 0.160),
        ],
        radius=0.0105,
        samples_per_segment=12,
        radial_segments=16,
    )
    right_footrest_hanger = tube_from_spline_points(
        _mirror_x(
            [
                (0.185, 0.160, 0.400),
                (0.142, 0.230, 0.250),
                (0.105, 0.285, 0.160),
            ]
        ),
        radius=0.0105,
        samples_per_segment=12,
        radial_segments=16,
    )

    frame.visual(_save_mesh("wheelchair_left_upper_rail", left_upper_rail), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_right_upper_rail", right_upper_rail), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_left_lower_rail", left_lower_rail), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_right_lower_rail", right_lower_rail), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_left_rear_brace", left_rear_brace), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_right_rear_brace", right_rear_brace), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_left_footrest_hanger", left_footrest_hanger), material=frame_paint)
    frame.visual(_save_mesh("wheelchair_right_footrest_hanger", right_footrest_hanger), material=frame_paint)

    _add_member(frame, (-0.205, 0.155, 0.415), (0.205, 0.155, 0.415), 0.012, frame_paint)
    _add_member(frame, (-0.205, -0.030, 0.430), (0.205, -0.030, 0.430), 0.012, frame_paint)
    _add_member(frame, (-0.205, -0.168, 0.655), (0.205, -0.168, 0.655), 0.011, frame_paint)
    _add_member(frame, (-0.105, 0.285, 0.160), (0.105, 0.285, 0.160), 0.010, frame_paint)
    _add_member(frame, (0.105, 0.285, 0.160), (0.080, 0.302, 0.100), 0.0085, frame_paint)
    _add_member(frame, (-0.105, 0.285, 0.160), (-0.080, 0.302, 0.100), 0.0085, frame_paint)

    frame.visual(
        Box((0.032, 0.060, 0.050)),
        origin=Origin(xyz=(0.227, -0.055, 0.315)),
        material=frame_paint,
    )
    frame.visual(
        Box((0.032, 0.060, 0.050)),
        origin=Origin(xyz=(-0.227, -0.055, 0.315)),
        material=frame_paint,
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.046),
        origin=Origin(xyz=(0.262, -0.055, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_axle_boss",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.046),
        origin=Origin(xyz=(-0.262, -0.055, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_axle_boss",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.215, 0.305, 0.196)),
        material=dark_metal,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(-0.215, 0.305, 0.196)),
        material=dark_metal,
        name="right_caster_socket",
    )

    frame.visual(
        seat_mesh,
        origin=Origin(xyz=(0.0, 0.075, 0.468)),
        material=upholstery,
        name="seat_panel",
    )
    frame.visual(
        back_mesh,
        origin=Origin(xyz=(0.0, -0.172, 0.620)),
        material=upholstery,
        name="backrest_panel",
    )
    frame.visual(
        Box((0.004, 0.320, 0.175)),
        origin=Origin(xyz=(0.164, 0.010, 0.540)),
        material=polymer_dark,
        name="left_sideguard",
    )
    frame.visual(
        Box((0.004, 0.320, 0.175)),
        origin=Origin(xyz=(-0.164, 0.010, 0.540)),
        material=polymer_dark,
        name="right_sideguard",
    )
    frame.visual(
        Box((0.054, 0.032, 0.020)),
        origin=Origin(xyz=(0.188, 0.075, 0.520)),
        material=frame_paint,
    )
    frame.visual(
        Box((0.054, 0.032, 0.020)),
        origin=Origin(xyz=(0.188, -0.060, 0.575)),
        material=frame_paint,
    )
    frame.visual(
        Box((0.054, 0.032, 0.020)),
        origin=Origin(xyz=(-0.188, 0.075, 0.520)),
        material=frame_paint,
    )
    frame.visual(
        Box((0.054, 0.032, 0.020)),
        origin=Origin(xyz=(-0.188, -0.060, 0.575)),
        material=frame_paint,
    )
    frame.visual(
        footplate_mesh,
        origin=Origin(xyz=(0.080, 0.312, 0.096)),
        material=polymer_dark,
        name="left_footplate",
    )
    frame.visual(
        footplate_mesh,
        origin=Origin(xyz=(-0.080, 0.312, 0.096)),
        material=polymer_dark,
        name="right_footplate",
    )

    left_drive_wheel = model.part("left_drive_wheel")
    right_drive_wheel = model.part("right_drive_wheel")
    left_caster_fork = model.part("left_caster_fork")
    right_caster_fork = model.part("right_caster_fork")
    left_caster_wheel = model.part("left_caster_wheel")
    right_caster_wheel = model.part("right_caster_wheel")

    _add_drive_wheel(
        left_drive_wheel,
        side_sign=1.0,
        tire_mesh=drive_tire_mesh,
        rim_mesh=drive_rim_mesh,
        pushrim_mesh=pushrim_mesh,
        rubber=rubber,
        aluminum=aluminum,
        dark_metal=dark_metal,
    )
    _add_drive_wheel(
        right_drive_wheel,
        side_sign=-1.0,
        tire_mesh=drive_tire_mesh,
        rim_mesh=drive_rim_mesh,
        pushrim_mesh=pushrim_mesh,
        rubber=rubber,
        aluminum=aluminum,
        dark_metal=dark_metal,
    )
    _add_caster_fork(left_caster_fork, frame_paint=frame_paint, polymer_dark=polymer_dark)
    _add_caster_fork(right_caster_fork, frame_paint=frame_paint, polymer_dark=polymer_dark)
    _add_caster_wheel(
        left_caster_wheel,
        tire_mesh=caster_tire_mesh,
        rim_mesh=caster_rim_mesh,
        rubber=rubber,
        aluminum=aluminum,
        dark_metal=dark_metal,
    )
    _add_caster_wheel(
        right_caster_wheel,
        tire_mesh=caster_tire_mesh,
        rim_mesh=caster_rim_mesh,
        rubber=rubber,
        aluminum=aluminum,
        dark_metal=dark_metal,
    )

    model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(0.285, -0.055, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.285, -0.055, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.215, 0.305, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.215, 0.305, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, -0.034, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=16.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, -0.034, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_drive_spin = object_model.get_articulation("left_drive_spin")
    right_drive_spin = object_model.get_articulation("right_drive_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_roll = object_model.get_articulation("left_caster_roll")
    right_caster_roll = object_model.get_articulation("right_caster_roll")

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

    ctx.expect_contact(
        left_drive_wheel,
        frame,
        elem_a="hub_sleeve",
        elem_b="left_axle_boss",
        name="left_drive_wheel_mounted_to_axle_boss",
    )
    ctx.expect_contact(
        right_drive_wheel,
        frame,
        elem_a="hub_sleeve",
        elem_b="right_axle_boss",
        name="right_drive_wheel_mounted_to_axle_boss",
    )
    ctx.expect_contact(
        left_caster_fork,
        frame,
        elem_a="stem",
        elem_b="left_caster_socket",
        name="left_caster_stem_supported_by_socket",
    )
    ctx.expect_contact(
        right_caster_fork,
        frame,
        elem_a="stem",
        elem_b="right_caster_socket",
        name="right_caster_stem_supported_by_socket",
    )
    ctx.expect_contact(
        left_caster_wheel,
        left_caster_fork,
        elem_a="hub",
        elem_b="fork_boss_outer",
        name="left_caster_wheel_supported_by_fork",
    )
    ctx.expect_contact(
        right_caster_wheel,
        right_caster_fork,
        elem_a="hub",
        elem_b="fork_boss_outer",
        name="right_caster_wheel_supported_by_fork",
    )
    ctx.expect_origin_distance(
        left_drive_wheel,
        right_drive_wheel,
        axes="x",
        min_dist=0.55,
        max_dist=0.59,
        name="drive_wheels_have_realistic_track_width",
    )

    ctx.check(
        "drive_wheel_spin_axes_are_explicit",
        _axis_matches(left_drive_spin.axis, (1.0, 0.0, 0.0))
        and _axis_matches(right_drive_spin.axis, (1.0, 0.0, 0.0)),
        details=f"left={left_drive_spin.axis}, right={right_drive_spin.axis}",
    )
    ctx.check(
        "caster_axes_are_explicit",
        _axis_matches(left_caster_swivel.axis, (0.0, 0.0, 1.0))
        and _axis_matches(right_caster_swivel.axis, (0.0, 0.0, 1.0))
        and _axis_matches(left_caster_roll.axis, (1.0, 0.0, 0.0))
        and _axis_matches(right_caster_roll.axis, (1.0, 0.0, 0.0)),
        details=(
            f"swivels=({left_caster_swivel.axis}, {right_caster_swivel.axis}), "
            f"rolls=({left_caster_roll.axis}, {right_caster_roll.axis})"
        ),
    )

    seat_aabb = ctx.part_element_world_aabb(frame, elem="seat_panel")
    back_aabb = ctx.part_element_world_aabb(frame, elem="backrest_panel")
    left_footplate_aabb = ctx.part_element_world_aabb(frame, elem="left_footplate")
    right_footplate_aabb = ctx.part_element_world_aabb(frame, elem="right_footplate")
    ctx.check(
        "core_frame_features_exist",
        all(aabb is not None for aabb in (seat_aabb, back_aabb, left_footplate_aabb, right_footplate_aabb)),
        details="seat, backrest, and both footplates must be authored as mounted visuals",
    )
    if (
        seat_aabb is not None
        and back_aabb is not None
        and left_footplate_aabb is not None
        and right_footplate_aabb is not None
    ):
        ctx.check(
            "backrest_sits_above_and_behind_seat",
            back_aabb[0][2] > seat_aabb[0][2]
            and back_aabb[1][1] < seat_aabb[0][1],
            details=f"seat={seat_aabb}, back={back_aabb}",
        )
        ctx.check(
            "footplates_project_forward_of_seat",
            left_footplate_aabb[0][1] > seat_aabb[1][1]
            and right_footplate_aabb[0][1] > seat_aabb[1][1],
            details=(
                f"seat={seat_aabb}, left_footplate={left_footplate_aabb}, "
                f"right_footplate={right_footplate_aabb}"
            ),
        )

    left_caster_rest = ctx.part_world_position(left_caster_wheel)
    right_caster_rest = ctx.part_world_position(right_caster_wheel)
    with ctx.pose(left_caster_swivel=0.65, right_caster_swivel=-0.65):
        left_caster_turned = ctx.part_world_position(left_caster_wheel)
        right_caster_turned = ctx.part_world_position(right_caster_wheel)
    ctx.check(
        "caster_swivel_changes_wheel_lateral_position",
        left_caster_rest is not None
        and right_caster_rest is not None
        and left_caster_turned is not None
        and right_caster_turned is not None
        and left_caster_turned[0] > left_caster_rest[0] + 0.01
        and right_caster_turned[0] < right_caster_rest[0] - 0.01,
        details=(
            f"left_rest={left_caster_rest}, left_turned={left_caster_turned}, "
            f"right_rest={right_caster_rest}, right_turned={right_caster_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
