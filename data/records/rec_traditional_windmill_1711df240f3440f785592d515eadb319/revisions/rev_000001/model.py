from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rotate_x_point(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, (c * y) - (s * z), (s * y) + (c * z))


def _lerp_point(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _octagon_section(rx: float, ry: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (rx, 0.0, z),
        (0.70 * rx, 0.70 * ry, z),
        (0.0, ry, z),
        (-0.70 * rx, 0.70 * ry, z),
        (-rx, 0.0, z),
        (-0.70 * rx, -0.70 * ry, z),
        (0.0, -ry, z),
        (0.70 * rx, -0.70 * ry, z),
    ]


def _yz_rounded_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def _build_tower_shell_mesh():
    return section_loft(
        [
            _octagon_section(0.205, 0.182, 0.10),
            _octagon_section(0.166, 0.148, 0.66),
            _octagon_section(0.122, 0.108, 1.14),
        ]
    )


def _build_cap_shell_mesh():
    return section_loft(
        [
            _yz_rounded_section(-0.19, 0.14, 0.09, 0.022, 0.105),
            _yz_rounded_section(-0.08, 0.22, 0.15, 0.035, 0.125),
            _yz_rounded_section(0.04, 0.27, 0.19, 0.045, 0.145),
            _yz_rounded_section(0.11, 0.20, 0.14, 0.032, 0.150),
            _yz_rounded_section(0.145, 0.12, 0.08, 0.020, 0.146),
        ]
    )


def _build_nose_housing_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.046, -0.080),
            (0.052, -0.052),
            (0.052, -0.012),
            (0.048, 0.000),
        ],
        [
            (0.024, -0.079),
            (0.028, -0.010),
            (0.028, -0.002),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _build_bearing_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.008),
            (0.034, 0.000),
        ],
        [
            (0.018, -0.008),
            (0.018, 0.000),
        ],
        segments=32,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _build_spinner_mesh():
    return ConeGeometry(radius=0.050, height=0.068, radial_segments=36).rotate_y(math.pi / 2.0)


def _add_blade_lattice(rotor, angle: float, hub_material, blade_material) -> None:
    def rp(point: tuple[float, float, float]) -> tuple[float, float, float]:
        return _rotate_x_point(point, angle)

    left_root = (0.024, -0.030, 0.120)
    right_root = (0.024, 0.030, 0.120)
    left_tip = (0.034, -0.061, 0.585)
    right_tip = (0.034, 0.061, 0.585)
    center_root = (0.024, 0.000, 0.120)
    center_tip = (0.034, 0.000, 0.585)
    hub_left = (0.010, -0.016, 0.056)
    hub_right = (0.010, 0.016, 0.056)
    hub_center = (0.006, 0.000, 0.050)

    _add_member(rotor, rp(hub_left), rp(left_root), 0.0095, hub_material)
    _add_member(rotor, rp(hub_right), rp(right_root), 0.0095, hub_material)
    _add_member(rotor, rp(hub_center), rp(center_root), 0.0080, hub_material)
    _add_member(rotor, rp(left_root), rp(left_tip), 0.0060, blade_material)
    _add_member(rotor, rp(right_root), rp(right_tip), 0.0060, blade_material)
    _add_member(rotor, rp(center_root), rp(center_tip), 0.0048, blade_material)
    _add_member(rotor, rp(left_root), rp(right_root), 0.0062, blade_material)
    _add_member(rotor, rp(left_tip), rp(right_tip), 0.0058, blade_material)

    for t in (0.18, 0.34, 0.50, 0.66, 0.82):
        _add_member(
            rotor,
            rp(_lerp_point(left_root, left_tip, t)),
            rp(_lerp_point(right_root, right_tip, t)),
            0.0046 if t < 0.70 else 0.0042,
            blade_material,
        )

    _add_member(
        rotor,
        rp(left_root),
        rp(_lerp_point(right_root, right_tip, 0.50)),
        0.0040,
        blade_material,
    )
    _add_member(
        rotor,
        rp(right_root),
        rp(_lerp_point(left_root, left_tip, 0.78)),
        0.0040,
        blade_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_traditional_windmill")

    tower_paint = model.material("tower_paint", rgba=(0.90, 0.91, 0.88, 1.0))
    graphite_metal = model.material("graphite_metal", rgba=(0.23, 0.25, 0.28, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.71, 0.74, 0.77, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.17, 0.18, 0.20, 1.0))
    smoked_polymer = model.material("smoked_polymer", rgba=(0.18, 0.22, 0.24, 0.55))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    tower = model.part("tower")
    tower.visual(
        _save_mesh("tower_shell", _build_tower_shell_mesh()),
        material=tower_paint,
        name="tower_shell",
    )
    tower.visual(
        Box((0.56, 0.50, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=graphite_metal,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=0.185, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        material=graphite_metal,
        name="crown_plate",
    )
    tower.visual(
        Box((0.110, 0.020, 0.235)),
        origin=Origin(xyz=(0.204, 0.0, 0.150)),
        material=dark_polymer,
        name="service_door",
    )
    tower.visual(
        Box((0.076, 0.016, 0.104)),
        origin=Origin(xyz=(0.170, 0.0, 0.560)),
        material=smoked_polymer,
        name="front_window",
    )
    tower.visual(
        Box((0.062, 0.016, 0.086)),
        origin=Origin(xyz=(0.138, 0.0, 0.875)),
        material=smoked_polymer,
        name="upper_window",
    )
    for sx in (-0.185, 0.185):
        for sy in (-0.165, 0.165):
            tower.visual(
                Box((0.090, 0.080, 0.018)),
                origin=Origin(xyz=(sx, sy, 0.009)),
                material=rubber_black,
            )
    tower.inertial = Inertial.from_geometry(
        Box((0.56, 0.50, 1.18)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.190, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=graphite_metal,
        name="yaw_plate",
    )
    cap.visual(
        _save_mesh("cap_shell", _build_cap_shell_mesh()),
        material=dark_polymer,
        name="cap_shell",
    )
    cap.visual(
        Box((0.160, 0.160, 0.022)),
        origin=Origin(xyz=(0.030, 0.0, 0.050)),
        material=graphite_metal,
        name="nacelle_deck",
    )
    cap.visual(
        Box((0.185, 0.014, 0.120)),
        origin=Origin(xyz=(0.120, 0.056, 0.100)),
        material=graphite_metal,
    )
    cap.visual(
        Box((0.185, 0.014, 0.120)),
        origin=Origin(xyz=(0.120, -0.056, 0.100)),
        material=graphite_metal,
    )
    cap.visual(
        Box((0.105, 0.118, 0.022)),
        origin=Origin(xyz=(0.145, 0.0, 0.176)),
        material=graphite_metal,
    )
    cap.visual(
        _save_mesh("nose_housing", _build_nose_housing_mesh()),
        origin=Origin(xyz=(0.245, 0.0, 0.148)),
        material=graphite_metal,
        name="nose_housing",
    )
    cap.visual(
        _save_mesh("bearing_seat", _build_bearing_ring_mesh()),
        origin=Origin(xyz=(0.245, 0.0, 0.148)),
        material=graphite_metal,
        name="bearing_seat",
    )
    cap.visual(
        Box((0.240, 0.025, 0.025)),
        origin=Origin(xyz=(-0.290, 0.0, 0.130)),
        material=graphite_metal,
    )
    cap.visual(
        Box((0.110, 0.006, 0.180)),
        origin=Origin(xyz=(-0.440, 0.0, 0.188)),
        material=dark_polymer,
        name="tail_vane",
    )
    cap.visual(
        Box((0.082, 0.028, 0.026)),
        origin=Origin(xyz=(-0.426, 0.0, 0.120)),
        material=dark_polymer,
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.66, 0.40, 0.28)),
        mass=4.6,
        origin=Origin(xyz=(-0.02, 0.0, 0.14)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.016, length=0.082),
        origin=Origin(xyz=(-0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_metal,
        name="drive_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_metal,
        name="thrust_collar",
    )
    rotor.visual(
        Cylinder(radius=0.058, length=0.090),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_metal,
        name="hub_shell",
    )
    rotor.visual(
        _save_mesh("rotor_spinner", _build_spinner_mesh()),
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        material=graphite_metal,
        name="spinner",
    )
    for blade_angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        _add_blade_lattice(rotor, blade_angle, graphite_metal, blade_metal)
    rotor.inertial = Inertial.from_geometry(
        Box((1.22, 1.22, 0.18)),
        mass=3.2,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    cap_yaw = model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 1.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.9),
    )
    rotor_spin = model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.245, 0.0, 0.148)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=14.0),
    )

    model.meta["primary_articulations"] = (cap_yaw.name, rotor_spin.name)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    cap_yaw = object_model.get_articulation("tower_to_cap")
    rotor_spin = object_model.get_articulation("cap_to_rotor")

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
        "cap_yaw_axis_is_vertical",
        cap_yaw.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {cap_yaw.axis}",
    )
    ctx.check(
        "rotor_spin_axis_is_fore_aft",
        rotor_spin.axis == (1.0, 0.0, 0.0),
        details=f"expected fore-aft rotor axis, got {rotor_spin.axis}",
    )
    ctx.expect_contact(
        cap,
        tower,
        elem_a="yaw_plate",
        elem_b="crown_plate",
        contact_tol=1e-4,
        name="cap_bearing_plate_contacts_tower_crown",
    )
    ctx.expect_contact(
        rotor,
        cap,
        elem_a="thrust_collar",
        elem_b="bearing_seat",
        contact_tol=1e-5,
        name="rotor_thrust_collar_seats_on_bearing_face",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="yz",
        inner_elem="drive_shaft",
        outer_elem="nose_housing",
        margin=0.0,
        name="drive_shaft_runs_within_nose_bearing_housing",
    )
    ctx.expect_gap(
        rotor,
        cap,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="nose_housing",
        min_gap=0.003,
        max_gap=0.012,
        name="hub_sits_cleanly_ahead_of_cap_nose",
    )

    with ctx.pose({cap_yaw: math.radians(38.0), rotor_spin: 1.15}):
        ctx.expect_contact(
            cap,
            tower,
            elem_a="yaw_plate",
            elem_b="crown_plate",
            contact_tol=1e-4,
            name="cap_bearing_contact_persists_in_yawed_pose",
        )
        ctx.expect_contact(
            rotor,
            cap,
            elem_a="thrust_collar",
            elem_b="bearing_seat",
            contact_tol=1e-5,
            name="rotor_bearing_contact_persists_in_spun_pose",
        )
        ctx.expect_within(
            rotor,
            cap,
            axes="yz",
            inner_elem="drive_shaft",
            outer_elem="nose_housing",
            margin=0.0,
            name="drive_shaft_stays_within_bearing_housing_in_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
