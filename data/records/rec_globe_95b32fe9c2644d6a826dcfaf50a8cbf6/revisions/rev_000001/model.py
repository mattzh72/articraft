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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wrap_profile_onto_surface,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _offset_point(
    origin: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (
        origin[0] + direction[0] * distance,
        origin[1] + direction[1] * distance,
        origin[2] + direction[2] * distance,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_globe")

    base_black = model.material("base_black", rgba=(0.14, 0.12, 0.11, 1.0))
    antique_brass = model.material("antique_brass", rgba=(0.71, 0.58, 0.31, 1.0))
    warm_ivory = model.material("warm_ivory", rgba=(0.90, 0.86, 0.74, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.21, 0.45, 0.70, 1.0))
    land_tan = model.material("land_tan", rgba=(0.79, 0.73, 0.56, 1.0))

    globe_radius = 0.11
    globe_center = (0.0, 0.0, 0.205)
    tilt = math.radians(23.5)
    polar_axis = (math.sin(tilt), 0.0, math.cos(tilt))

    date_ring_radius = 0.123
    date_ring_tube = 0.0035
    pivot_pin_radius = 0.0042
    pivot_pin_length = 0.008
    pivot_stop_radius = 0.0048
    trunnion_radius = 0.0030
    trunnion_length = 0.008
    trunnion_stop_radius = 0.0034
    meridian_radius = 0.128

    north_pin_tip = _offset_point(globe_center, polar_axis, globe_radius + pivot_pin_length)
    south_pin_tip = _offset_point(globe_center, polar_axis, -(globe_radius + pivot_pin_length))
    north_stop_center = _offset_point(north_pin_tip, polar_axis, 0.0005)
    south_stop_center = _offset_point(south_pin_tip, polar_axis, -0.0005)

    left_trunnion_tip_y = date_ring_radius + date_ring_tube + trunnion_length
    right_trunnion_tip_y = -left_trunnion_tip_y
    left_stop_center = (0.0, left_trunnion_tip_y + 0.0005, globe_center[2])
    right_stop_center = (0.0, right_trunnion_tip_y - 0.0005, globe_center[2])

    stand = model.part("stand")
    pedestal_profile = [
        (0.0, 0.0),
        (0.085, 0.0),
        (0.090, 0.006),
        (0.086, 0.012),
        (0.072, 0.018),
        (0.050, 0.026),
        (0.040, 0.068),
        (0.035, 0.086),
        (0.030, 0.090),
    ]
    stand.visual(
        _save_mesh("pedestal_body", LatheGeometry(pedestal_profile, segments=72)),
        material=base_black,
        name="pedestal_body",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=antique_brass,
        name="pedestal_cap",
    )
    meridian_stand_mount = (-0.120, -0.145, 0.145)
    support_stand_mount = (-0.145, 0.075, 0.165)
    _add_member(
        stand,
        (-0.026, -0.026, 0.088),
        meridian_stand_mount,
        radius=0.0038,
        material=antique_brass,
        name="meridian_mount_strut",
    )
    _add_member(
        stand,
        (-0.026, 0.026, 0.088),
        support_stand_mount,
        radius=0.0038,
        material=antique_brass,
        name="support_mount_strut",
    )
    stand.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=meridian_stand_mount),
        material=warm_ivory,
        name="meridian_mount",
    )
    stand.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=support_stand_mount),
        material=warm_ivory,
        name="support_mount",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.20, 0.18, 0.12)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    meridian = model.part("meridian")
    meridian_mount_pad = (-0.112, -0.145, 0.145)
    meridian.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=meridian_mount_pad),
        material=warm_ivory,
        name="meridian_mount_pad",
    )
    meridian_points = [
        meridian_mount_pad,
        (-0.090, -0.145, 0.105),
        (-0.045, -0.145, 0.078),
        (0.010, -0.145, 0.090),
        (0.042, -0.145, 0.120),
        (0.054, -0.145, 0.170),
        (0.048, -0.145, 0.235),
        (0.028, -0.145, 0.292),
        (0.010, -0.145, 0.330),
    ]
    for index, (a, b) in enumerate(zip(meridian_points[:-1], meridian_points[1:])):
        _add_member(
            meridian,
            a,
            b,
            radius=0.005,
            material=antique_brass,
            name=f"meridian_segment_{index}",
        )
    _add_member(
        meridian,
        meridian_points[1],
        south_stop_center,
        radius=0.0038,
        material=antique_brass,
        name="south_pivot_link",
    )
    _add_member(
        meridian,
        meridian_points[-1],
        north_stop_center,
        radius=0.0038,
        material=antique_brass,
        name="north_pivot_link",
    )
    meridian.visual(
        Cylinder(radius=0.0042, length=0.001),
        origin=Origin(xyz=north_stop_center, rpy=(0.0, tilt, 0.0)),
        material=warm_ivory,
        name="north_pivot_stop",
    )
    meridian.visual(
        Cylinder(radius=0.0042, length=0.001),
        origin=Origin(xyz=south_stop_center, rpy=(0.0, tilt, 0.0)),
        material=warm_ivory,
        name="south_pivot_stop",
    )
    meridian.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.28)),
        mass=0.5,
        origin=Origin(xyz=(-0.02, -0.145, 0.20)),
    )

    date_supports = model.part("date_supports")
    support_mount_pad = (-0.137, 0.075, 0.165)
    date_supports.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=support_mount_pad),
        material=warm_ivory,
        name="support_mount_pad",
    )
    left_support_points = [
        support_mount_pad,
        (-0.112, 0.090, 0.175),
        (-0.070, 0.120, 0.188),
        (-0.030, 0.136, 0.198),
        left_stop_center,
    ]
    right_support_points = [
        support_mount_pad,
        (-0.112, -0.090, 0.175),
        (-0.070, -0.120, 0.188),
        (-0.030, -0.136, 0.198),
        right_stop_center,
    ]
    for index, (a, b) in enumerate(zip(left_support_points[:-1], left_support_points[1:])):
        _add_member(
            date_supports,
            a,
            b,
            radius=0.0042,
            material=antique_brass,
            name=f"left_support_segment_{index}",
        )
    for index, (a, b) in enumerate(zip(right_support_points[:-1], right_support_points[1:])):
        _add_member(
            date_supports,
            a,
            b,
            radius=0.0042,
            material=antique_brass,
            name=f"right_support_segment_{index}",
        )
    date_supports.visual(
        Cylinder(radius=0.0030, length=0.001),
        origin=Origin(xyz=left_stop_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_ivory,
        name="left_trunnion_stop",
    )
    date_supports.visual(
        Cylinder(radius=0.0030, length=0.001),
        origin=Origin(xyz=right_stop_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_ivory,
        name="right_trunnion_stop",
    )
    date_supports.inertial = Inertial.from_geometry(
        Box((0.18, 0.30, 0.06)),
        mass=0.35,
        origin=Origin(xyz=(-0.04, 0.0, 0.18)),
    )

    globe = model.part("globe")
    globe.visual(Sphere(radius=globe_radius), material=ocean_blue, name="globe_shell")
    globe.visual(
        Cylinder(radius=pivot_pin_radius, length=pivot_pin_length),
        origin=Origin(
            xyz=tuple(
                component * (globe_radius + pivot_pin_length * 0.5)
                for component in polar_axis
            ),
            rpy=(0.0, tilt, 0.0),
        ),
        material=antique_brass,
        name="north_pivot_pin",
    )
    globe.visual(
        Cylinder(radius=pivot_pin_radius, length=pivot_pin_length),
        origin=Origin(
            xyz=tuple(
                -component * (globe_radius + pivot_pin_length * 0.5)
                for component in polar_axis
            ),
            rpy=(0.0, tilt, 0.0),
        ),
        material=antique_brass,
        name="south_pivot_pin",
    )
    globe.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(
            xyz=tuple(component * (globe_radius - 0.0005) for component in polar_axis),
            rpy=(0.0, tilt, 0.0),
        ),
        material=antique_brass,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(
            xyz=tuple(
                -component * (globe_radius - 0.0005) for component in polar_axis
            ),
            rpy=(0.0, tilt, 0.0),
        ),
        material=antique_brass,
        name="south_cap",
    )

    sphere_target = Sphere(radius=globe_radius)
    americas_profile = [
        (-0.017, 0.036),
        (-0.022, 0.016),
        (-0.016, -0.002),
        (-0.018, -0.018),
        (-0.008, -0.035),
        (0.004, -0.042),
        (0.016, -0.029),
        (0.018, -0.010),
        (0.012, 0.010),
        (0.017, 0.036),
    ]
    eurafrica_profile = [
        (-0.034, 0.028),
        (-0.022, 0.044),
        (0.004, 0.048),
        (0.028, 0.034),
        (0.038, 0.012),
        (0.030, -0.004),
        (0.012, -0.014),
        (0.006, -0.040),
        (-0.010, -0.048),
        (-0.026, -0.030),
        (-0.034, 0.028),
    ]
    australia_profile = [
        (-0.014, 0.012),
        (0.002, 0.020),
        (0.020, 0.012),
        (0.024, -0.002),
        (0.012, -0.016),
        (-0.006, -0.018),
        (-0.018, -0.006),
        (-0.014, 0.012),
    ]
    for mesh_name, profile, direction, visual_name in (
        ("americas_relief", americas_profile, (0.82, -0.10, 0.18), "americas_relief"),
        ("eurafrica_relief", eurafrica_profile, (-0.78, 0.18, 0.24), "eurafrica_relief"),
        ("australia_relief", australia_profile, (-0.48, 0.82, -0.26), "australia_relief"),
    ):
        globe.visual(
            _save_mesh(
                mesh_name,
                wrap_profile_onto_surface(
                    profile,
                    sphere_target,
                    thickness=0.0015,
                    direction=direction,
                    mapping="intrinsic",
                    visible_relief=0.0002,
                ),
            ),
            material=land_tan,
            name=visual_name,
        )
    globe.inertial = Inertial.from_geometry(Sphere(radius=globe_radius), mass=0.9)

    date_ring = model.part("date_ring")
    date_ring.visual(
        _save_mesh(
            "date_ring_band",
            TorusGeometry(
                radius=date_ring_radius,
                tube=date_ring_tube,
                radial_segments=20,
                tubular_segments=96,
            ).rotate_y(tilt),
        ),
        material=warm_ivory,
        name="date_ring_band",
    )
    date_ring.visual(
        Cylinder(radius=trunnion_radius, length=trunnion_length),
        origin=Origin(
            xyz=(0.0, date_ring_radius + date_ring_tube + trunnion_length * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=antique_brass,
        name="left_trunnion_pin",
    )
    date_ring.visual(
        Cylinder(radius=trunnion_radius, length=trunnion_length),
        origin=Origin(
            xyz=(0.0, -(date_ring_radius + date_ring_tube + trunnion_length * 0.5), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=antique_brass,
        name="right_trunnion_pin",
    )
    ring_marker_radius = date_ring_radius + 0.007
    date_ring.visual(
        Box((0.016, 0.007, 0.004)),
        origin=Origin(
            xyz=(
                math.cos(tilt) * ring_marker_radius,
                0.0,
                -math.sin(tilt) * ring_marker_radius,
            ),
            rpy=(0.0, tilt, 0.0),
        ),
        material=antique_brass,
        name="date_marker",
    )
    date_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=date_ring_radius + date_ring_tube, length=0.010),
        mass=0.11,
    )

    model.articulation(
        "stand_to_meridian",
        ArticulationType.FIXED,
        parent=stand,
        child=meridian,
        origin=Origin(),
    )
    model.articulation(
        "stand_to_date_supports",
        ArticulationType.FIXED,
        parent=stand,
        child=date_supports,
        origin=Origin(),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=globe,
        origin=Origin(xyz=globe_center),
        axis=polar_axis,
        motion_limits=MotionLimits(effort=3.0, velocity=3.5),
    )
    model.articulation(
        "date_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=date_ring,
        origin=Origin(xyz=globe_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    date_supports = object_model.get_part("date_supports")
    globe = object_model.get_part("globe")
    date_ring = object_model.get_part("date_ring")
    globe_spin = object_model.get_articulation("globe_spin")
    date_ring_spin = object_model.get_articulation("date_ring_spin")

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
        date_supports,
        stand,
        elem_a="right_support_segment_0",
        elem_b="support_mount_strut",
        reason="Date-support frame is welded onto the stand mount strut at a shared junction.",
    )
    ctx.allow_overlap(
        date_supports,
        stand,
        elem_a="left_support_segment_0",
        elem_b="support_mount_strut",
        reason="Date-support frame is welded onto the stand mount strut at the same shared junction on the opposite side.",
    )
    ctx.allow_overlap(
        meridian,
        stand,
        elem_a="meridian_segment_0",
        elem_b="meridian_mount_strut",
        reason="Meridian frame is rigidly brazed to the stand at the rear mount node.",
    )
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="south_pivot_pin",
        elem_b="south_pivot_link",
        reason="Polar pivot pin seats slightly into the meridian-side bearing link.",
    )
    ctx.allow_overlap(
        date_supports,
        stand,
        elem_a="support_mount_pad",
        elem_b="support_mount_strut",
        reason="Ivory mount pad is a decorative cap blended into the stand's support strut.",
    )
    ctx.allow_overlap(
        meridian,
        stand,
        elem_a="meridian_mount_pad",
        elem_b="meridian_mount_strut",
        reason="Rear meridian mount pad is integrated into the stand mounting strut.",
    )
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="north_pivot_pin",
        elem_b="north_pivot_link",
        reason="North polar pivot pin seats slightly into the meridian-side bearing link.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(meridian, stand, name="meridian_attaches_to_stand")
    ctx.expect_contact(date_supports, stand, name="date_supports_attach_to_stand")
    ctx.expect_contact(globe, meridian, name="globe_contacts_polar_bearings")
    ctx.expect_contact(date_ring, date_supports, name="date_ring_contacts_support_clips")
    ctx.expect_origin_distance(
        globe,
        date_ring,
        axes="xyz",
        max_dist=0.0005,
        name="date_ring_stays_concentric_with_globe",
    )

    ctx.check(
        "globe_joint_uses_tilted_local_polar_axis",
        all(abs(a - b) < 1e-9 for a, b in zip(globe_spin.axis, (math.sin(math.radians(23.5)), 0.0, math.cos(math.radians(23.5))))),
        details=f"axis={globe_spin.axis}",
    )
    ctx.check(
        "date_ring_joint_matches_equatorial_plane",
        date_ring_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={date_ring_spin.axis}",
    )

    americas_rest_center = _aabb_center(
        ctx.part_element_world_aabb(globe, elem="americas_relief")
    )
    with ctx.pose({globe_spin: 1.1}):
        americas_rotated_center = _aabb_center(
            ctx.part_element_world_aabb(globe, elem="americas_relief")
        )
        moved = (
            americas_rest_center is not None
            and americas_rotated_center is not None
            and math.dist(americas_rest_center, americas_rotated_center) > 0.015
        )
        ctx.check(
            "globe_rotation_moves_surface_detail",
            moved,
            details=(
                f"rest_center={americas_rest_center}, "
                f"rotated_center={americas_rotated_center}"
            ),
        )
        ctx.expect_contact(globe, meridian, name="globe_remains_supported_when_spun")

    with ctx.pose({date_ring_spin: math.pi / 2.0}):
        ctx.expect_contact(
            date_ring,
            date_supports,
            name="date_ring_remains_clipped_at_quarter_turn",
        )
        ctx.expect_origin_distance(
            globe,
            date_ring,
            axes="xyz",
            max_dist=0.0005,
            name="date_ring_quarter_turn_stays_concentric",
        )

    with ctx.pose({globe_spin: 2.1, date_ring_spin: math.pi}):
        ctx.expect_contact(globe, meridian, name="globe_supported_in_combined_pose")
        ctx.expect_contact(date_ring, date_supports, name="date_ring_supported_in_combined_pose")
        ctx.expect_origin_distance(
            globe,
            date_ring,
            axes="xyz",
            max_dist=0.0005,
            name="combined_pose_keeps_ring_centered",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_combined_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
