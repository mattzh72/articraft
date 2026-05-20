from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


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
    length_xy = sqrt(dx * dx + dy * dy)
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


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)


def _build_drive_wheel(part, mesh_prefix: str, *, side_sign: float, tire, rim, steel) -> None:
    tire_center_x = side_sign * 0.024
    handrim_center_x = side_sign * 0.052
    hub_center_x = side_sign * 0.024
    spoke_plane_x = side_sign * 0.024

    tire_geom = TorusGeometry(
        radius=0.292,
        tube=0.014,
        radial_segments=18,
        tubular_segments=52,
    ).rotate_y(pi / 2.0)
    part.visual(
        mesh_from_geometry(tire_geom, f"{mesh_prefix}_tire"),
        origin=Origin(xyz=(tire_center_x, 0.0, 0.0)),
        material=tire,
        name="tire",
    )

    rim_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.280),
        [_circle_profile(0.240)],
        height=0.008,
        center=True,
    ).rotate_y(pi / 2.0)
    part.visual(
        mesh_from_geometry(rim_geom, f"{mesh_prefix}_rim"),
        origin=Origin(xyz=(spoke_plane_x, 0.0, 0.0)),
        material=rim,
        name="rim_ring",
    )

    part.visual(
        Cylinder(radius=0.042, length=0.048),
        origin=Origin(xyz=(hub_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(side_sign * 0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim,
        name="outer_cap",
    )

    handrim_geom = TorusGeometry(
        radius=0.268,
        tube=0.006,
        radial_segments=14,
        tubular_segments=44,
    ).rotate_y(pi / 2.0)
    part.visual(
        mesh_from_geometry(handrim_geom, f"{mesh_prefix}_handrim"),
        origin=Origin(xyz=(handrim_center_x, 0.0, 0.0)),
        material=steel,
        name="handrim",
    )

    for spoke_index in range(12):
        angle = (2.0 * pi * spoke_index) / 12.0 + (0.12 if spoke_index % 2 else 0.0)
        inner = (
            spoke_plane_x,
            cos(angle) * 0.036,
            sin(angle) * 0.036,
        )
        outer = (
            spoke_plane_x,
            cos(angle) * 0.248,
            sin(angle) * 0.248,
        )
        _add_member(part, inner, outer, 0.0032, steel)

    for standoff_angle in (0.35, 1.95, 4.10, 5.35):
        radial = 0.268
        part.visual(
            Cylinder(radius=0.0045, length=0.034),
            origin=Origin(
                xyz=(side_sign * 0.037, cos(standoff_angle) * radial, sin(standoff_angle) * radial),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
        )

    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.306, length=0.055),
        mass=2.6,
        origin=Origin(xyz=(side_sign * 0.028, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )


def _build_caster_fork(part, *, metal, dark_steel) -> None:
    trail = -0.024
    part.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_steel,
        name="stem",
    )
    part.visual(
        Box((0.048, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, trail * 0.35, -0.058)),
        material=metal,
        name="yoke_bridge",
    )
    part.visual(
        Box((0.010, 0.018, 0.124)),
        origin=Origin(xyz=(0.024, trail, -0.129)),
        material=metal,
        name="left_arm",
    )
    part.visual(
        Box((0.010, 0.018, 0.124)),
        origin=Origin(xyz=(-0.024, trail, -0.129)),
        material=metal,
        name="right_arm",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.021, trail, -0.191), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_cap",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(-0.021, trail, -0.191), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.220)),
        mass=0.7,
        origin=Origin(xyz=(0.0, trail * 0.5, -0.100)),
    )


def _build_caster_wheel(part, *, tire, rim, steel) -> None:
    part.visual(
        Cylinder(radius=0.068, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire,
        name="tread",
    )
    part.visual(
        Cylinder(radius=0.054, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rim,
        name="inner_wheel",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.068, length=0.024),
        mass=0.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wheelchair")

    frame_green = model.material("frame_green", rgba=(0.34, 0.39, 0.34, 1.0))
    dark_canvas = model.material("dark_canvas", rgba=(0.11, 0.12, 0.13, 1.0))
    arm_pad = model.material("arm_pad", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.70, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    hatch_grey = model.material("hatch_grey", rgba=(0.74, 0.75, 0.77, 1.0))
    service_olive = model.material("service_olive", rgba=(0.42, 0.44, 0.39, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_grey = model.material("wheel_grey", rgba=(0.78, 0.79, 0.81, 1.0))
    footplate_black = model.material("footplate_black", rgba=(0.12, 0.12, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.66, 0.92, 0.94)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.03, 0.47)),
    )

    frame.visual(
        Box((0.410, 0.380, 0.008)),
        origin=Origin(xyz=(0.0, 0.03, 0.525)),
        material=dark_canvas,
        name="seat_sling",
    )
    frame.visual(
        Box((0.420, 0.020, 0.240)),
        origin=Origin(xyz=(0.0, -0.190, 0.720)),
        material=dark_canvas,
        name="back_sling",
    )

    for side_sign in (1.0, -1.0):
        side_name = "left" if side_sign > 0.0 else "right"
        back_top = (side_sign * 0.225, -0.240, 0.900)
        back_joint = (side_sign * 0.225, -0.180, 0.660)
        seat_rear = (side_sign * 0.210, -0.160, 0.520)
        arm_front = (side_sign * 0.223, 0.100, 0.660)
        front_upper = (side_sign * 0.216, 0.180, 0.620)
        seat_front = (side_sign * 0.205, 0.240, 0.520)
        lower_rear = (side_sign * 0.195, -0.140, 0.430)
        lower_front = (side_sign * 0.205, 0.290, 0.430)

        _add_member(frame, back_top, back_joint, 0.014, frame_green)
        _add_member(frame, back_joint, seat_rear, 0.014, frame_green)
        _add_member(frame, back_joint, arm_front, 0.014, frame_green)
        _add_member(frame, arm_front, front_upper, 0.014, frame_green)
        _add_member(frame, front_upper, seat_front, 0.014, frame_green)
        _add_member(frame, seat_rear, seat_front, 0.014, frame_green)
        _add_member(frame, seat_rear, lower_rear, 0.014, frame_green)
        _add_member(frame, seat_front, lower_front, 0.013, frame_green)
        _add_member(frame, lower_rear, lower_front, 0.014, frame_green)
        _add_member(
            frame,
            (side_sign * 0.225, -0.285, 0.895),
            (side_sign * 0.225, -0.235, 0.895),
            0.017,
            dark_canvas,
        )

        frame.visual(
            Box((0.050, 0.220, 0.024)),
            origin=Origin(xyz=(side_sign * 0.223, -0.010, 0.686)),
            material=arm_pad,
            name=f"{side_name}_arm_pad",
        )

        frame.visual(
            Box((0.124, 0.180, 0.110)),
            origin=Origin(xyz=(side_sign * 0.166, -0.005, 0.360)),
            material=service_olive,
            name=f"{side_name}_service_box",
        )
        frame.visual(
            Box((0.024, 0.040, 0.050)),
            origin=Origin(xyz=(side_sign * 0.196, -0.060, 0.380)),
            material=dark_steel,
        )
        frame.visual(
            Box((0.024, 0.040, 0.050)),
            origin=Origin(xyz=(side_sign * 0.196, 0.050, 0.380)),
            material=dark_steel,
        )
        frame.visual(
            Box((0.006, 0.150, 0.090)),
            origin=Origin(xyz=(side_sign * 0.191, -0.005, 0.360)),
            material=hatch_grey,
            name=f"{side_name}_service_hatch",
        )
        for bolt_y in (-0.060, 0.050):
            for bolt_z in (0.330, 0.390):
                frame.visual(
                    Cylinder(radius=0.006, length=0.010),
                    origin=Origin(
                        xyz=(side_sign * 0.194, bolt_y, bolt_z),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material=steel,
                )

        frame.visual(
            Box((0.016, 0.140, 0.220)),
            origin=Origin(xyz=(side_sign * 0.218, -0.080, 0.320)),
            material=dark_steel,
            name=f"{side_name}_axle_plate",
        )
        frame.visual(
            Box((0.040, 0.040, 0.040)),
            origin=Origin(xyz=(side_sign * 0.205, -0.115, 0.450)),
            material=dark_steel,
        )
        frame.visual(
            Box((0.040, 0.040, 0.040)),
            origin=Origin(xyz=(side_sign * 0.205, -0.015, 0.450)),
            material=dark_steel,
        )
        frame.visual(
            Cylinder(radius=0.028, length=0.010),
            origin=Origin(
                xyz=(side_sign * 0.230, -0.030, 0.310),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(
                xyz=(side_sign * 0.238, -0.030, 0.310),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"{side_name}_axle_sleeve",
        )
        for bolt_y in (-0.130, -0.030):
            for bolt_z in (0.270, 0.390):
                frame.visual(
                    Cylinder(radius=0.006, length=0.012),
                    origin=Origin(
                        xyz=(side_sign * 0.224, bolt_y, bolt_z),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material=steel,
                )

        frame.visual(
            Box((0.018, 0.080, 0.170)),
            origin=Origin(xyz=(side_sign * 0.214, 0.310, 0.345)),
            material=dark_steel,
            name=f"{side_name}_caster_plate",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.044),
            origin=Origin(xyz=(side_sign * 0.214, 0.330, 0.282)),
            material=steel,
            name=f"{side_name}_caster_sleeve",
        )
        for bolt_y in (0.280, 0.340):
            for bolt_z in (0.300, 0.380):
                frame.visual(
                    Cylinder(radius=0.0055, length=0.012),
                    origin=Origin(
                        xyz=(side_sign * 0.219, bolt_y, bolt_z),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material=steel,
                )

        frame.visual(
            Box((0.060, 0.030, 0.040)),
            origin=Origin(xyz=(side_sign * 0.168, 0.245, 0.425)),
            material=dark_steel,
            name=f"{side_name}_footrest_clamp",
        )
        _add_member(
            frame,
            (side_sign * 0.145, 0.245, 0.405),
            (side_sign * 0.145, 0.245, 0.086),
            0.012,
            steel,
        )
        frame.visual(
            Box((0.110, 0.075, 0.012)),
            origin=Origin(xyz=(side_sign * 0.145, 0.245, 0.080)),
            material=footplate_black,
            name=f"{side_name}_footplate",
        )
        frame.visual(
            Box((0.090, 0.010, 0.030)),
            origin=Origin(xyz=(side_sign * 0.145, 0.278, 0.095)),
            material=footplate_black,
        )

    _add_member(frame, (-0.225, -0.180, 0.660), (0.225, -0.180, 0.660), 0.012, frame_green)
    _add_member(frame, (-0.210, -0.160, 0.520), (0.210, -0.160, 0.520), 0.013, frame_green)
    _add_member(frame, (-0.205, 0.240, 0.520), (0.205, 0.240, 0.520), 0.013, frame_green)
    _add_member(frame, (-0.205, 0.290, 0.430), (0.205, 0.290, 0.430), 0.012, frame_green)
    _add_member(frame, (-0.195, -0.080, 0.430), (0.195, -0.080, 0.430), 0.012, frame_green)
    _add_member(frame, (0.195, -0.100, 0.430), (-0.195, 0.100, 0.430), 0.009, steel)
    _add_member(frame, (-0.195, -0.100, 0.430), (0.195, 0.100, 0.430), 0.009, steel)
    _add_member(frame, (-0.018, 0.000, 0.430), (0.018, 0.000, 0.430), 0.012, dark_steel)
    _add_member(frame, (0.0, 0.000, 0.408), (0.0, 0.000, 0.452), 0.006, steel)

    left_drive_wheel = model.part("left_drive_wheel")
    _build_drive_wheel(
        left_drive_wheel,
        "left_drive_wheel",
        side_sign=1.0,
        tire=tire_black,
        rim=wheel_grey,
        steel=steel,
    )

    right_drive_wheel = model.part("right_drive_wheel")
    _build_drive_wheel(
        right_drive_wheel,
        "right_drive_wheel",
        side_sign=-1.0,
        tire=tire_black,
        rim=wheel_grey,
        steel=steel,
    )

    left_caster_fork = model.part("left_caster_fork")
    _build_caster_fork(left_caster_fork, metal=wheel_grey, dark_steel=dark_steel)

    right_caster_fork = model.part("right_caster_fork")
    _build_caster_fork(right_caster_fork, metal=wheel_grey, dark_steel=dark_steel)

    left_caster_wheel = model.part("left_caster_wheel")
    _build_caster_wheel(left_caster_wheel, tire=tire_black, rim=wheel_grey, steel=steel)

    right_caster_wheel = model.part("right_caster_wheel")
    _build_caster_wheel(right_caster_wheel, tire=tire_black, rim=wheel_grey, steel=steel)

    model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(0.250, -0.030, 0.310)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=18.0),
    )
    model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.250, -0.030, 0.310)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.214, 0.330, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=7.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.214, 0.330, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=7.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, -0.024, -0.191)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, -0.024, -0.191)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
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
    left_caster_wheel_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_caster_wheel_spin = object_model.get_articulation("right_caster_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        left_caster_fork,
        left_caster_wheel,
        elem_a="left_axle_cap",
        elem_b="hub",
        reason="Caster axle cap intentionally occupies the wheel hub bore as the visible axle stub.",
    )
    ctx.allow_overlap(
        left_caster_fork,
        left_caster_wheel,
        elem_a="right_axle_cap",
        elem_b="hub",
        reason="Caster axle cap intentionally occupies the wheel hub bore as the visible axle stub.",
    )
    ctx.allow_overlap(
        right_caster_fork,
        right_caster_wheel,
        elem_a="left_axle_cap",
        elem_b="hub",
        reason="Caster axle cap intentionally occupies the wheel hub bore as the visible axle stub.",
    )
    ctx.allow_overlap(
        right_caster_fork,
        right_caster_wheel,
        elem_a="right_axle_cap",
        elem_b="hub",
        reason="Caster axle cap intentionally occupies the wheel hub bore as the visible axle stub.",
    )

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
        frame,
        left_drive_wheel,
        elem_a="left_axle_sleeve",
        elem_b="hub_shell",
        name="left_drive_hub_supported",
    )
    ctx.expect_contact(
        frame,
        right_drive_wheel,
        elem_a="right_axle_sleeve",
        elem_b="hub_shell",
        name="right_drive_hub_supported",
    )
    ctx.expect_contact(
        frame,
        left_caster_fork,
        elem_a="left_caster_sleeve",
        elem_b="stem",
        name="left_caster_swivel_supported",
    )
    ctx.expect_contact(
        frame,
        right_caster_fork,
        elem_a="right_caster_sleeve",
        elem_b="stem",
        name="right_caster_swivel_supported",
    )
    ctx.expect_contact(left_caster_fork, left_caster_wheel, name="left_caster_wheel_supported")
    ctx.expect_contact(right_caster_fork, right_caster_wheel, name="right_caster_wheel_supported")
    ctx.expect_origin_distance(
        left_drive_wheel,
        right_drive_wheel,
        axes="x",
        min_dist=0.48,
        max_dist=0.53,
        name="rear_track_width_realistic",
    )
    ctx.expect_origin_gap(
        left_caster_fork,
        left_drive_wheel,
        axis="y",
        min_gap=0.30,
        max_gap=0.40,
        name="front_caster_leads_left_drive",
    )
    ctx.expect_origin_gap(
        right_caster_fork,
        right_drive_wheel,
        axis="y",
        min_gap=0.30,
        max_gap=0.40,
        name="front_caster_leads_right_drive",
    )

    ctx.check(
        "drive_wheels_are_explicit_spin_joints",
        (
            left_drive_spin.articulation_type == ArticulationType.CONTINUOUS
            and right_drive_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(left_drive_spin.axis) == (-1.0, 0.0, 0.0)
            and tuple(right_drive_spin.axis) == (-1.0, 0.0, 0.0)
        ),
        details="Rear drive wheels should spin about the exposed axle line.",
    )
    ctx.check(
        "caster_swivels_are_vertical",
        (
            left_caster_swivel.articulation_type == ArticulationType.CONTINUOUS
            and right_caster_swivel.articulation_type == ArticulationType.CONTINUOUS
            and tuple(left_caster_swivel.axis) == (0.0, 0.0, 1.0)
            and tuple(right_caster_swivel.axis) == (0.0, 0.0, 1.0)
        ),
        details="Caster forks should swivel around explicit vertical stems.",
    )
    ctx.check(
        "caster_wheels_spin_on_their_axles",
        (
            left_caster_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and right_caster_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(left_caster_wheel_spin.axis) == (-1.0, 0.0, 0.0)
            and tuple(right_caster_wheel_spin.axis) == (-1.0, 0.0, 0.0)
        ),
        details="Each caster wheel should rotate on a lateral axle inside its fork.",
    )

    with ctx.pose(left_caster_swivel=1.0, right_caster_swivel=-0.9):
        ctx.expect_contact(left_caster_fork, left_caster_wheel, name="left_caster_contact_while_swiveled")
        ctx.expect_contact(right_caster_fork, right_caster_wheel, name="right_caster_contact_while_swiveled")
        ctx.fail_if_parts_overlap_in_current_pose(name="swiveled_caster_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
