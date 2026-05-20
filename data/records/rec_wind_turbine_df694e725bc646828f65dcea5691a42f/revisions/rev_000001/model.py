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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry: MeshGeometry):
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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


_TOWER_RADIUS_PROFILE: tuple[tuple[float, float], ...] = (
    (0.30, 1.35),
    (8.00, 1.22),
    (16.00, 1.02),
    (23.84, 0.82),
)


def _tower_radius(z: float) -> float:
    if z <= _TOWER_RADIUS_PROFILE[0][0]:
        return _TOWER_RADIUS_PROFILE[0][1]
    for (z0, r0), (z1, r1) in zip(_TOWER_RADIUS_PROFILE, _TOWER_RADIUS_PROFILE[1:]):
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return _TOWER_RADIUS_PROFILE[-1][1]


def _ellipse_section(
    z_pos: float, half_x: float, half_y: float, twist_deg: float = 0.0
) -> list[tuple[float, float, float]]:
    twist = math.radians(twist_deg)
    c = math.cos(twist)
    s = math.sin(twist)
    points = []
    for index in range(8):
        angle = math.tau * index / 8.0
        x = half_x * math.cos(angle)
        y = half_y * math.sin(angle)
        points.append((c * x - s * y, s * x + c * y, z_pos))
    return points


def _blade_section(
    z_pos: float, chord: float, thickness: float, twist_deg: float
) -> list[tuple[float, float, float]]:
    twist = math.radians(twist_deg)
    c = math.cos(twist)
    s = math.sin(twist)
    profile = [
        (-0.42 * chord, 0.00 * thickness),
        (-0.16 * chord, 0.56 * thickness),
        (0.18 * chord, 0.46 * thickness),
        (0.50 * chord, 0.12 * thickness),
        (0.46 * chord, -0.10 * thickness),
        (0.18 * chord, -0.34 * thickness),
        (-0.16 * chord, -0.30 * thickness),
        (-0.40 * chord, -0.08 * thickness),
    ]
    return [(c * x - s * y, s * x + c * y, z_pos) for x, y in profile]


def _build_blade_mesh(name: str):
    blade_geom = MeshGeometry()
    blade_geom.merge(
        CylinderGeometry(radius=0.18, height=0.86, radial_segments=30).translate(0.0, 0.0, 0.43)
    )
    blade_geom.merge(
        CylinderGeometry(radius=0.25, height=0.16, radial_segments=30).translate(0.0, 0.0, 0.24)
    )
    blade_geom.merge(
        CylinderGeometry(radius=0.22, height=0.26, radial_segments=30).translate(0.0, 0.0, 0.64)
    )
    for index in range(8):
        angle = math.tau * index / 8.0
        blade_geom.merge(
            CylinderGeometry(radius=0.017, height=0.05, radial_segments=14).translate(
                0.23 * math.cos(angle),
                0.23 * math.sin(angle),
                0.24,
            )
        )
    blade_geom.merge(
        section_loft(
            [
                _ellipse_section(0.72, 0.24, 0.21, twist_deg=8.0),
                _blade_section(1.35, 0.80, 0.20, twist_deg=7.0),
                _blade_section(3.00, 0.56, 0.12, twist_deg=2.0),
                _blade_section(5.30, 0.31, 0.055, twist_deg=-3.0),
                _blade_section(6.80, 0.14, 0.022, twist_deg=-6.0),
            ]
        )
    )
    return _save_mesh(name, blade_geom)


def _build_spinner_mesh(name: str):
    spinner = LatheGeometry(
        [
            (0.00, -0.42),
            (0.08, -0.37),
            (0.20, -0.20),
            (0.30, 0.02),
            (0.34, 0.18),
            (0.28, 0.36),
            (0.00, 0.50),
        ],
        segments=56,
    )
    spinner.rotate_y(math.pi / 2.0).translate(1.82, 0.0, 0.0)
    return _save_mesh(name, spinner)


def _build_tower_shell_mesh(name: str):
    outer_profile = [
        (1.48, 0.00),
        (1.40, 0.18),
        (1.35, 0.30),
        (1.22, 8.00),
        (1.02, 16.00),
        (0.82, 23.84),
    ]
    inner_profile = [
        (1.40, 0.00),
        (1.33, 0.18),
        (1.29, 0.30),
        (1.17, 8.00),
        (0.97, 16.00),
        (0.77, 23.84),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _build_ring_mesh(name: str, radius: float, tube: float):
    return _save_mesh(
        name,
        TorusGeometry(
            radius=radius,
            tube=tube,
            radial_segments=18,
            tubular_segments=72,
        ),
    )


def _add_bolt_circle(
    part,
    *,
    z: float,
    radius: float,
    count: int,
    bolt_radius: float,
    bolt_length: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(count):
        angle = math.tau * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(
                xyz=(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    z,
                )
            ),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def _add_x_face_bolt_circle(
    part,
    *,
    x: float,
    radius: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
    count: int,
    bolt_radius: float,
    bolt_length: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(count):
        angle = math.tau * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(
                xyz=(
                    x,
                    center_y + radius * math.cos(angle),
                    center_z + radius * math.sin(angle),
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wind_turbine")

    tower_white = model.material("tower_white", rgba=(0.84, 0.86, 0.86, 1.0))
    nacelle_cream = model.material("nacelle_cream", rgba=(0.80, 0.81, 0.76, 1.0))
    weathered_grey = model.material("weathered_grey", rgba=(0.36, 0.38, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    galvanized = model.material("galvanized", rgba=(0.69, 0.72, 0.74, 1.0))
    service_grey = model.material("service_grey", rgba=(0.60, 0.63, 0.65, 1.0))

    tower_shell_mesh = _build_tower_shell_mesh("tower_shell")
    mid_ring_mesh = _build_ring_mesh("tower_mid_ring", radius=1.274, tube=0.070)
    upper_ring_mesh = _build_ring_mesh("tower_upper_ring", radius=1.050, tube=0.060)
    blade_mesh = _build_blade_mesh("turbine_blade")
    spinner_mesh = _build_spinner_mesh("hub_spinner")

    tower = model.part("tower")
    tower.visual(tower_shell_mesh, material=tower_white, name="tower_shell")
    tower.visual(
        Cylinder(radius=1.62, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=weathered_grey,
        name="foundation_plinth",
    )
    tower.visual(
        Cylinder(radius=0.92, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 23.91)),
        material=dark_steel,
        name="top_adapter_cap",
    )
    tower.visual(
        Cylinder(radius=1.34, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 8.00)),
        material=dark_steel,
        name="segment_ring_lower",
    )
    tower.visual(
        Cylinder(radius=1.11, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 16.00)),
        material=dark_steel,
        name="segment_ring_upper",
    )
    tower.visual(
        Box((0.08, 0.84, 1.95)),
        origin=Origin(xyz=(_tower_radius(1.45) + 0.01, 0.0, 1.45)),
        material=service_grey,
        name="tower_service_hatch",
    )
    tower.visual(
        Box((0.10, 0.96, 0.10)),
        origin=Origin(xyz=(_tower_radius(2.45) + 0.015, 0.0, 2.40)),
        material=dark_steel,
        name="tower_hatch_header",
    )
    tower.visual(
        Box((0.10, 0.96, 0.10)),
        origin=Origin(xyz=(_tower_radius(0.50) + 0.015, 0.0, 0.50)),
        material=dark_steel,
        name="tower_hatch_sill",
    )
    tower.visual(
        Box((0.10, 0.08, 1.96)),
        origin=Origin(xyz=(_tower_radius(1.45) + 0.015, -0.44, 1.45)),
        material=dark_steel,
        name="tower_hatch_left_jamb",
    )
    tower.visual(
        Box((0.10, 0.08, 1.96)),
        origin=Origin(xyz=(_tower_radius(1.45) + 0.015, 0.44, 1.45)),
        material=dark_steel,
        name="tower_hatch_right_jamb",
    )
    tower.visual(
        Cylinder(radius=0.025, length=1.70),
        origin=Origin(
            xyz=(_tower_radius(1.45) + 0.07, -0.47, 1.55),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=galvanized,
        name="tower_hatch_hinge_bar",
    )
    _add_bolt_circle(
        tower,
        z=0.22,
        radius=1.44,
        count=20,
        bolt_radius=0.022,
        bolt_length=0.08,
        material=galvanized,
        name_prefix="base_bolt",
    )
    _add_bolt_circle(
        tower,
        z=8.00,
        radius=1.29,
        count=18,
        bolt_radius=0.016,
        bolt_length=0.09,
        material=galvanized,
        name_prefix="mid_bolt",
    )
    _add_bolt_circle(
        tower,
        z=16.00,
        radius=1.08,
        count=16,
        bolt_radius=0.015,
        bolt_length=0.09,
        material=galvanized,
        name_prefix="upper_bolt",
    )
    _add_bolt_circle(
        tower,
        z=23.95,
        radius=0.82,
        count=16,
        bolt_radius=0.015,
        bolt_length=0.08,
        material=galvanized,
        name_prefix="top_bolt",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=1.50, length=24.0),
        mass=9600.0,
        origin=Origin(xyz=(0.0, 0.0, 12.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.95, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="yaw_deck",
    )
    nacelle.visual(
        Box((1.70, 1.30, 0.14)),
        origin=Origin(xyz=(0.05, 0.0, 0.22)),
        material=weathered_grey,
        name="yaw_adapter_plate",
    )
    for index, y_pos in enumerate((-0.56, 0.56)):
        nacelle.visual(
            Box((3.60, 0.22, 0.24)),
            origin=Origin(xyz=(0.24, y_pos, 0.42)),
            material=dark_steel,
            name=f"bed_rail_{index}",
        )
    for index, x_pos in enumerate((-1.04, -0.14, 0.98, 1.86)):
        nacelle.visual(
            Box((0.18, 1.30, 0.22)),
            origin=Origin(xyz=(x_pos, 0.0, 0.42)),
            material=dark_steel,
            name=f"bed_crossmember_{index}",
        )
    _add_member(nacelle, (-0.58, -0.55, 0.18), (-0.16, -0.55, 0.66), 0.065, dark_steel)
    _add_member(nacelle, (-0.58, 0.55, 0.18), (-0.16, 0.55, 0.66), 0.065, dark_steel)
    _add_member(nacelle, (0.08, -0.55, 0.18), (0.42, -0.55, 0.70), 0.060, dark_steel)
    _add_member(nacelle, (0.08, 0.55, 0.18), (0.42, 0.55, 0.70), 0.060, dark_steel)
    nacelle.visual(
        Box((3.12, 0.05, 1.66)),
        origin=Origin(xyz=(-0.02, -0.88, 1.18)),
        material=nacelle_cream,
        name="starboard_shell",
    )
    nacelle.visual(
        Box((3.12, 0.05, 1.66)),
        origin=Origin(xyz=(-0.02, 0.88, 1.18)),
        material=nacelle_cream,
        name="port_shell",
    )
    nacelle.visual(
        Box((2.94, 1.74, 0.06)),
        origin=Origin(xyz=(-0.10, 0.0, 2.03)),
        material=nacelle_cream,
        name="roof_panel",
    )
    nacelle.visual(
        Box((0.44, 1.72, 1.56)),
        origin=Origin(xyz=(-1.46, 0.0, 1.18)),
        material=nacelle_cream,
        name="rear_bulkhead",
    )
    nacelle.visual(
        Box((0.92, 1.66, 0.06)),
        origin=Origin(xyz=(1.72, 0.0, 2.00), rpy=(0.0, -0.24, 0.0)),
        material=nacelle_cream,
        name="front_roof_break",
    )
    nacelle.visual(
        Box((0.92, 1.55, 0.05)),
        origin=Origin(xyz=(1.78, 0.0, 0.76), rpy=(0.0, 0.16, 0.0)),
        material=nacelle_cream,
        name="nose_belly_panel",
    )
    nacelle.visual(
        Box((0.72, 1.06, 1.06)),
        origin=Origin(xyz=(1.90, 0.0, 1.45)),
        material=weathered_grey,
        name="bearing_pedestal",
    )
    nacelle.visual(
        Box((0.92, 1.04, 0.34)),
        origin=Origin(xyz=(1.58, 0.0, 0.66)),
        material=dark_steel,
        name="bearing_saddle_frame",
    )
    nacelle.visual(
        Box((0.88, 0.18, 0.90)),
        origin=Origin(xyz=(1.62, -0.44, 0.97)),
        material=dark_steel,
        name="bearing_cheek_starboard",
    )
    nacelle.visual(
        Box((0.88, 0.18, 0.90)),
        origin=Origin(xyz=(1.62, 0.44, 0.97)),
        material=dark_steel,
        name="bearing_cheek_port",
    )
    nacelle.visual(
        Box((0.90, 1.02, 0.24)),
        origin=Origin(xyz=(1.58, 0.0, 1.84)),
        material=dark_steel,
        name="bearing_crown_frame",
    )
    nacelle.visual(
        Cylinder(radius=0.68, length=0.08),
        origin=Origin(xyz=(2.09, 0.0, 1.45), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="bearing_adapter_flange",
    )
    nacelle.visual(
        Cylinder(radius=0.55, length=0.22),
        origin=Origin(xyz=(2.24, 0.0, 1.45), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="main_bearing_housing",
    )
    nacelle.visual(
        Box((0.96, 0.06, 1.12)),
        origin=Origin(xyz=(-0.12, -0.87, 1.20)),
        material=service_grey,
        name="nacelle_service_hatch",
    )
    nacelle.visual(
        Box((1.04, 0.10, 0.08)),
        origin=Origin(xyz=(-0.10, -0.88, 1.78)),
        material=dark_steel,
        name="nacelle_hatch_header",
    )
    nacelle.visual(
        Box((1.04, 0.10, 0.08)),
        origin=Origin(xyz=(-0.10, -0.88, 0.64)),
        material=dark_steel,
        name="nacelle_hatch_sill",
    )
    nacelle.visual(
        Box((0.10, 0.10, 1.16)),
        origin=Origin(xyz=(-0.58, -0.88, 1.20)),
        material=dark_steel,
        name="nacelle_hatch_front_jamb",
    )
    nacelle.visual(
        Box((0.10, 0.10, 1.16)),
        origin=Origin(xyz=(0.36, -0.88, 1.20)),
        material=dark_steel,
        name="nacelle_hatch_rear_jamb",
    )
    nacelle.visual(
        Cylinder(radius=0.020, length=1.02),
        origin=Origin(
            xyz=(-0.58, -0.93, 1.22),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=galvanized,
        name="nacelle_hatch_hinge",
    )
    nacelle.visual(
        Box((0.44, 0.52, 0.46)),
        origin=Origin(xyz=(-1.00, 0.0, 1.98)),
        material=weathered_grey,
        name="rear_cooling_box",
    )
    nacelle.visual(
        Box((0.40, 1.20, 0.08)),
        origin=Origin(xyz=(-1.38, 0.0, 1.36)),
        material=dark_steel,
        name="rear_service_frame",
    )
    _add_bolt_circle(
        nacelle,
        z=0.12,
        radius=0.88,
        count=18,
        bolt_radius=0.014,
        bolt_length=0.07,
        material=galvanized,
        name_prefix="yaw_bolt",
    )
    _add_x_face_bolt_circle(
        nacelle,
        x=2.09,
        radius=0.58,
        center_z=1.45,
        count=14,
        bolt_radius=0.014,
        bolt_length=0.07,
        material=galvanized,
        name_prefix="bearing_bolt",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((4.00, 2.20, 2.20)),
        mass=2600.0,
        origin=Origin(xyz=(0.30, 0.0, 1.10)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.24, length=0.26),
        origin=Origin(xyz=(0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="main_shaft_stub",
    )
    rotor.visual(
        Cylinder(radius=0.38, length=0.44),
        origin=Origin(xyz=(0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_grey,
        name="shaft_adapter",
    )
    rotor.visual(
        Cylinder(radius=0.50, length=1.30),
        origin=Origin(xyz=(0.85, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_grey,
        name="hub_barrel",
    )
    rotor.visual(spinner_mesh, material=tower_white, name="spinner")
    for blade_index, angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        rotor.visual(
            Box((0.44, 0.18, 0.76)),
            origin=Origin(xyz=(0.82, 0.0, 0.34), rpy=(angle, 0.0, 0.0)),
            material=weathered_grey,
            name=f"root_gusset_{blade_index}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.86, 0.0, 0.0), rpy=(angle, 0.0, 0.0)),
            material=tower_white,
            name=f"blade_{blade_index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=7.0, length=2.3),
        mass=1800.0,
        origin=Origin(xyz=(0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 24.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.20),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(2.35, 0.0, 1.45)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_rotor_spin")

    tower.get_visual("tower_service_hatch")
    nacelle.get_visual("nacelle_service_hatch")
    rotor.get_visual("blade_0")
    rotor.get_visual("blade_1")
    rotor.get_visual("blade_2")

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
        "yaw articulation is vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical yaw axis, got {yaw.axis!r}",
    )
    ctx.check(
        "rotor articulation is fore-aft",
        spin.axis == (1.0, 0.0, 0.0),
        details=f"Expected fore-aft rotor axis, got {spin.axis!r}",
    )

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="yaw_deck",
        negative_elem="top_adapter_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="nacelle seats on tower adapter",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        elem_a="yaw_deck",
        elem_b="top_adapter_cap",
        min_overlap=1.60,
        name="yaw deck overlaps tower adapter footprint",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="main_shaft_stub",
        negative_elem="main_bearing_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotor shaft seats on bearing housing",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="yz",
        elem_a="main_shaft_stub",
        elem_b="main_bearing_housing",
        min_overlap=0.45,
        name="rotor shaft aligns with bearing housing",
    )

    with ctx.pose({yaw: 0.70, spin: math.pi / 3.0}):
        ctx.expect_origin_distance(
            rotor,
            tower,
            axes="xy",
            min_dist=2.20,
            name="rotor remains well upwind of tower centerline",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
