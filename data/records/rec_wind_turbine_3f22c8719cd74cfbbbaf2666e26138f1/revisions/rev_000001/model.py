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
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


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


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if mag < 1e-9:
        return (0.0, 0.0, 0.0)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _rpy_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    r00, r10, r20 = x_axis
    r01, r11, r21 = y_axis
    r02, r12, r22 = z_axis
    pitch = math.asin(_clamp(-r20, -1.0, 1.0))
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) > 1e-8:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    return (roll, pitch, yaw)


def _blade_mount_rpy(azimuth: float) -> tuple[float, float, float]:
    radial = (0.0, math.cos(azimuth), math.sin(azimuth))
    z_axis = (1.0, 0.0, 0.0)
    y_axis = _normalize(_cross(z_axis, radial))
    return _rpy_from_axes(radial, y_axis, z_axis)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _build_tower_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (1.55, 1.80),
            (1.52, 4.40),
            (1.40, 11.50),
            (1.20, 20.50),
            (0.98, 28.80),
        ],
        [
            (1.42, 1.80),
            (1.40, 4.40),
            (1.28, 11.50),
            (1.08, 20.50),
            (0.86, 28.80),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _blade_section_loop(
    x_pos: float,
    *,
    chord: float,
    thickness: float,
    twist_deg: float,
    z_bias: float,
) -> list[tuple[float, float, float]]:
    profile = [
        (-0.46, 0.02),
        (-0.26, 0.48),
        (0.00, 0.96),
        (0.30, 0.62),
        (0.48, 0.14),
        (0.50, -0.06),
        (0.32, -0.26),
        (0.04, -0.40),
        (-0.26, -0.28),
        (-0.48, -0.08),
    ]
    twist = math.radians(twist_deg)
    cos_t = math.cos(twist)
    sin_t = math.sin(twist)
    pts: list[tuple[float, float, float]] = []
    for chord_n, thick_n in profile:
        local_y = 0.5 * thickness * thick_n
        local_z = 0.5 * chord * chord_n + z_bias
        y_rot = (cos_t * local_y) - (sin_t * local_z)
        z_rot = (sin_t * local_y) + (cos_t * local_z)
        pts.append((x_pos, y_rot, z_rot))
    return pts


def _build_blade_shell_mesh(name: str):
    sections = [
        _blade_section_loop(0.30, chord=1.08, thickness=0.84, twist_deg=18.0, z_bias=0.00),
        _blade_section_loop(1.00, chord=1.54, thickness=0.58, twist_deg=15.0, z_bias=0.06),
        _blade_section_loop(3.00, chord=1.44, thickness=0.40, twist_deg=11.5, z_bias=0.14),
        _blade_section_loop(5.80, chord=1.08, thickness=0.24, twist_deg=7.5, z_bias=0.28),
        _blade_section_loop(8.60, chord=0.70, thickness=0.13, twist_deg=4.0, z_bias=0.48),
        _blade_section_loop(10.80, chord=0.34, thickness=0.055, twist_deg=1.5, z_bias=0.67),
        _blade_section_loop(11.60, chord=0.09, thickness=0.020, twist_deg=0.0, z_bias=0.73),
    ]
    return _save_mesh(repair_loft(section_loft(sections)), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.83, 0.85, 0.86, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.75, 0.77, 0.79, 1.0))
    blade_paint = model.material("blade_paint", rgba=(0.90, 0.91, 0.92, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.43, 0.46, 0.50, 1.0))
    machinery = model.material("machinery", rgba=(0.52, 0.55, 0.58, 1.0))
    service_orange = model.material("service_orange", rgba=(0.80, 0.40, 0.12, 1.0))
    concrete = model.material("concrete", rgba=(0.60, 0.60, 0.58, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((7.20, 7.20, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        Box((4.80, 4.80, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        material=concrete,
    )
    tower.visual(
        _save_mesh(_build_tower_shell_mesh(), "tower_shell"),
        material=tower_paint,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=1.24, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 28.91)),
        material=steel_dark,
        name="tower_top_flange",
    )
    tower.visual(
        Cylinder(radius=1.72, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.86)),
        material=steel_dark,
    )
    tower.visual(
        Box((1.05, 0.14, 2.10)),
        origin=Origin(xyz=(1.49, 0.0, 2.95)),
        material=steel_dark,
    )
    tower.visual(
        Box((0.96, 0.05, 1.92)),
        origin=Origin(xyz=(1.55, 0.0, 2.95)),
        material=service_orange,
    )
    for bolt_x in (-0.95, -0.45, 0.45, 0.95):
        for bolt_y in (-0.95, -0.45, 0.45, 0.95):
            tower.visual(
                Cylinder(radius=0.06, length=0.22),
                origin=Origin(xyz=(bolt_x, bolt_y, 1.91)),
                material=steel_mid,
            )
    tower.inertial = Inertial.from_geometry(
        Box((7.20, 7.20, 29.20)),
        mass=82000.0,
        origin=Origin(xyz=(0.0, 0.0, 14.60)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.16, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=steel_dark,
        name="yaw_ring",
    )
    nacelle.visual(
        Box((5.90, 2.90, 0.12)),
        origin=Origin(xyz=(0.15, 0.0, 0.24)),
        material=steel_dark,
        name="bed_deck",
    )
    nacelle.visual(
        Box((5.10, 2.20, 0.42)),
        origin=Origin(xyz=(0.18, 0.0, 0.51)),
        material=steel_mid,
    )
    nacelle.visual(
        Box((4.90, 0.10, 1.62)),
        origin=Origin(xyz=(0.18, -1.40, 1.15)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((2.90, 0.10, 1.62)),
        origin=Origin(xyz=(1.65, 1.40, 1.15)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((0.80, 0.10, 1.62)),
        origin=Origin(xyz=(-1.90, 1.40, 1.15)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((1.66, 0.10, 0.18)),
        origin=Origin(xyz=(-0.65, 1.40, 0.51)),
        material=nacelle_paint,
        name="service_sill",
    )
    nacelle.visual(
        Box((1.66, 0.10, 0.16)),
        origin=Origin(xyz=(-0.65, 1.40, 1.88)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((0.28, 2.60, 1.58)),
        origin=Origin(xyz=(-2.34, 0.0, 1.13)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((5.20, 2.80, 0.10)),
        origin=Origin(xyz=(0.00, 0.0, 2.01)),
        material=nacelle_paint,
        name="roof_shell",
    )
    nacelle.visual(
        Box((5.55, 2.70, 1.55)),
        origin=Origin(xyz=(0.12, 0.0, 1.05)),
        material=steel_dark,
        name="mainframe",
    )
    nacelle.visual(
        Box((4.20, 2.40, 0.18)),
        origin=Origin(xyz=(0.20, 0.0, 1.89)),
        material=steel_dark,
    )
    nacelle.visual(
        Box((1.65, 1.70, 1.16)),
        origin=Origin(xyz=(0.76, 0.0, 1.03)),
        material=machinery,
    )
    nacelle.visual(
        Box((1.55, 1.10, 0.15)),
        origin=Origin(xyz=(0.76, 0.0, 0.375)),
        material=steel_dark,
    )
    nacelle.visual(
        Cylinder(radius=0.62, length=1.70),
        origin=Origin(xyz=(-0.95, 0.0, 1.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery,
    )
    nacelle.visual(
        Box((1.50, 1.00, 0.20)),
        origin=Origin(xyz=(-0.95, 0.0, 0.40)),
        material=steel_dark,
    )
    nacelle.visual(
        Box((0.95, 1.90, 1.06)),
        origin=Origin(xyz=(-1.98, 0.0, 1.00)),
        material=steel_mid,
    )
    nacelle.visual(
        Box((0.95, 1.90, 0.17)),
        origin=Origin(xyz=(-1.98, 0.0, 0.385)),
        material=steel_dark,
    )
    nacelle.visual(
        Box((0.95, 2.24, 1.05)),
        origin=Origin(xyz=(2.92, 0.0, 1.02)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((0.95, 2.24, 0.195)),
        origin=Origin(xyz=(2.92, 0.0, 0.3975)),
        material=steel_dark,
    )
    nacelle.visual(
        Box((0.68, 1.80, 0.54)),
        origin=Origin(xyz=(2.72, 0.0, 1.62)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Cylinder(radius=0.78, length=0.70),
        origin=Origin(xyz=(3.24, 0.0, 1.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="nose_housing",
    )
    _add_member(
        nacelle,
        (2.20, -0.68, 0.58),
        (3.04, -0.52, 1.14),
        radius=0.12,
        material=steel_dark,
    )
    _add_member(
        nacelle,
        (2.20, 0.68, 0.58),
        (3.04, 0.52, 1.14),
        radius=0.12,
        material=steel_dark,
    )
    _add_member(
        nacelle,
        (2.30, -0.58, 0.92),
        (3.08, -0.42, 1.70),
        radius=0.10,
        material=steel_dark,
    )
    _add_member(
        nacelle,
        (2.30, 0.58, 0.92),
        (3.08, 0.42, 1.70),
        radius=0.10,
        material=steel_dark,
    )
    rail_left_x = (-1.80, 1.10)
    rail_y = 1.23
    rail_z = 2.22
    for side in (-1.0, 1.0):
        _add_member(
            nacelle,
            (rail_left_x[0], side * rail_y, rail_z),
            (rail_left_x[1], side * rail_y, rail_z),
            radius=0.028,
            material=steel_mid,
        )
        for x_pos in (-1.60, -0.90, -0.20, 0.50, 1.00):
            _add_member(
                nacelle,
                (x_pos, side * rail_y, 2.01),
                (x_pos, side * rail_y, rail_z),
                radius=0.020,
                material=steel_mid,
            )
    _add_member(
        nacelle,
        (-1.80, -rail_y, rail_z),
        (-1.80, rail_y, rail_z),
        radius=0.028,
        material=steel_mid,
    )
    _add_member(
        nacelle,
        (1.10, -rail_y, rail_z),
        (1.10, rail_y, rail_z),
        radius=0.028,
        material=steel_mid,
    )
    nacelle.visual(
        Box((0.12, 0.08, 1.34)),
        origin=Origin(xyz=(-1.54, 1.39, 1.15)),
        material=steel_mid,
        name="jamb_barrel",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((6.30, 3.00, 2.40)),
        mass=18000.0,
        origin=Origin(xyz=(0.20, 0.0, 1.10)),
    )

    access_door = model.part("access_door")
    access_door.visual(
        Box((1.48, 0.05, 1.18)),
        origin=Origin(xyz=(0.74, -0.035, -0.01)),
        material=service_orange,
        name="door_panel",
    )
    access_door.visual(
        Box((1.46, 0.025, 0.11)),
        origin=Origin(xyz=(0.74, -0.0725, 0.47)),
        material=steel_dark,
    )
    access_door.visual(
        Box((1.46, 0.025, 0.11)),
        origin=Origin(xyz=(0.74, -0.0725, -0.47)),
        material=steel_dark,
    )
    access_door.visual(
        Box((0.10, 0.04, 1.18)),
        origin=Origin(xyz=(0.05, -0.06, -0.01)),
        material=steel_mid,
        name="hinge_barrels",
    )
    access_door.visual(
        Box((0.12, 0.04, 1.18)),
        origin=Origin(xyz=(1.43, -0.06, -0.01)),
        material=steel_dark,
    )
    access_door.visual(
        Box((0.34, 0.045, 0.10)),
        origin=Origin(xyz=(1.15, -0.0825, 0.0)),
        material=steel_mid,
        name="latch_paddle",
    )
    access_door.visual(
        Box((0.08, 0.045, 0.36)),
        origin=Origin(xyz=(1.15, -0.0825, 0.0)),
        material=steel_mid,
        name="latch_backplate",
    )
    access_door.inertial = Inertial.from_geometry(
        Box((1.70, 0.10, 1.24)),
        mass=120.0,
        origin=Origin(xyz=(0.74, -0.05, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.50, length=0.16),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="rear_flange",
    )
    rotor.visual(
        Cylinder(radius=0.34, length=1.05),
        origin=Origin(xyz=(0.60, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mid,
    )
    rotor.visual(
        Sphere(radius=1.05),
        origin=Origin(xyz=(1.55, 0.0, 0.0)),
        material=steel_dark,
    )
    rotor.visual(
        Cylinder(radius=0.58, length=0.56),
        origin=Origin(xyz=(2.38, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mid,
    )
    blade_mounts = {
        "a": math.pi / 2.0,
        "b": -math.pi / 6.0,
        "c": 7.0 * math.pi / 6.0,
    }
    for label, azimuth in blade_mounts.items():
        radial_y = 1.50 * math.cos(azimuth)
        radial_z = 1.50 * math.sin(azimuth)
        _add_member(
            rotor,
            (1.18, 0.0, 0.0),
            (1.70, radial_y * (1.28 / 1.50), radial_z * (1.28 / 1.50)),
            radius=0.32,
            material=steel_dark,
        )
        _add_member(
            rotor,
            (1.70, radial_y * (1.28 / 1.50), radial_z * (1.28 / 1.50)),
            (1.70, radial_y, radial_z),
            radius=0.52,
            material=steel_mid,
            name=f"pitch_socket_{label}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((3.20, 4.20, 4.20)),
        mass=7000.0,
        origin=Origin(xyz=(1.55, 0.0, 0.0)),
    )

    blade_meshes = {
        "blade_a": _build_blade_shell_mesh("blade_a_shell"),
        "blade_b": _build_blade_shell_mesh("blade_b_shell"),
        "blade_c": _build_blade_shell_mesh("blade_c_shell"),
    }
    blades = {}
    for blade_name, mesh in blade_meshes.items():
        blade = model.part(blade_name)
        blade.visual(
            Cylinder(radius=0.54, length=0.18),
            origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel_mid,
            name="root_flange",
        )
        blade.visual(
            Cylinder(radius=0.48, length=0.92),
            origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=blade_paint,
        )
        blade.visual(
            mesh,
            material=blade_paint,
            name="blade_shell",
        )
        blade.inertial = Inertial.from_geometry(
            Box((11.80, 1.60, 1.00)),
            mass=1800.0,
            origin=Origin(xyz=(5.90, 0.0, 0.22)),
        )
        blades[blade_name] = blade

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 29.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.09),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(3.59, 0.0, 1.42)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160000.0, velocity=1.2),
    )
    model.articulation(
        "nacelle_to_access_door",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child=access_door,
        origin=Origin(xyz=(-1.48, 1.51, 1.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.0, lower=0.0, upper=1.45),
    )
    for label, azimuth in blade_mounts.items():
        model.articulation(
            f"rotor_to_blade_{label}_pitch",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=blades[f"blade_{label}"],
            origin=Origin(
                xyz=(1.70, 1.50 * math.cos(azimuth), 1.50 * math.sin(azimuth)),
                rpy=_blade_mount_rpy(azimuth),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=20000.0,
                velocity=0.35,
                lower=-0.25,
                upper=1.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    access_door = object_model.get_part("access_door")
    blade_a = object_model.get_part("blade_a")
    blade_b = object_model.get_part("blade_b")
    blade_c = object_model.get_part("blade_c")

    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    rotor_spin = object_model.get_articulation("nacelle_to_rotor_spin")
    door_hinge = object_model.get_articulation("nacelle_to_access_door")
    blade_a_pitch = object_model.get_articulation("rotor_to_blade_a_pitch")
    blade_b_pitch = object_model.get_articulation("rotor_to_blade_b_pitch")
    blade_c_pitch = object_model.get_articulation("rotor_to_blade_c_pitch")

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
        "articulation_axes_match_turbine_mechanics",
        yaw.axis == (0.0, 0.0, 1.0)
        and rotor_spin.axis == (1.0, 0.0, 0.0)
        and blade_a_pitch.axis == (1.0, 0.0, 0.0)
        and blade_b_pitch.axis == (1.0, 0.0, 0.0)
        and blade_c_pitch.axis == (1.0, 0.0, 0.0),
        "yaw should be vertical, rotor spin along shaft, and blade pitch along each blade span.",
    )

    with ctx.pose(
        {
            yaw: 0.0,
            rotor_spin: 0.0,
            door_hinge: 0.0,
            blade_a_pitch: 0.0,
            blade_b_pitch: 0.0,
            blade_c_pitch: 0.0,
        }
    ):
        ctx.expect_contact(
            nacelle,
            tower,
            elem_a="yaw_ring",
            elem_b="tower_top_flange",
            contact_tol=0.01,
            name="nacelle_seated_on_tower_flange",
        )
        ctx.expect_contact(
            rotor,
            nacelle,
            elem_a="rear_flange",
            elem_b="nose_housing",
            contact_tol=0.02,
            name="rotor_load_path_seated_in_nose_housing",
        )
        ctx.expect_contact(
            blade_a,
            rotor,
            elem_a="root_flange",
            elem_b="pitch_socket_a",
            contact_tol=0.01,
            name="blade_a_root_carried_by_pitch_socket",
        )
        ctx.expect_contact(
            blade_b,
            rotor,
            elem_a="root_flange",
            elem_b="pitch_socket_b",
            contact_tol=0.01,
            name="blade_b_root_carried_by_pitch_socket",
        )
        ctx.expect_contact(
            blade_c,
            rotor,
            elem_a="root_flange",
            elem_b="pitch_socket_c",
            contact_tol=0.01,
            name="blade_c_root_carried_by_pitch_socket",
        )
        ctx.expect_contact(
            access_door,
            nacelle,
            elem_a="door_panel",
            elem_b="service_sill",
            contact_tol=0.03,
            name="access_door_seats_on_service_sill",
        )

    closed_y = ctx.part_world_aabb(access_door)[1][1]
    with ctx.pose({door_hinge: 1.10}):
        open_y = ctx.part_world_aabb(access_door)[1][1]
    ctx.check(
        "access_door_opens_outboard",
        open_y > closed_y + 0.45,
        f"expected opened service door to swing outward; closed max y={closed_y:.3f}, open max y={open_y:.3f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
