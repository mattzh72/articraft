from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


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
):
    return part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _scale(
    v: tuple[float, float, float], s: float
) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _add_vec(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub_vec(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _basis_from_normal(
    normal: tuple[float, float, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    n = _normalize(normal)
    ref = (0.0, 0.0, 1.0) if abs(n[2]) < 0.9 else (0.0, 1.0, 0.0)
    u = _normalize(_cross(ref, n))
    v = _normalize(_cross(n, u))
    return n, u, v


def _add_bolt_ring(
    part,
    *,
    center: tuple[float, float, float],
    normal: tuple[float, float, float],
    bolt_circle_radius: float,
    bolt_radius: float,
    bolt_length: float,
    count: int,
    material,
    name_prefix: str | None = None,
) -> None:
    n, u, v = _basis_from_normal(normal)
    for idx in range(count):
        angle = math.tau * idx / count
        radial = _add_vec(_scale(u, math.cos(angle)), _scale(v, math.sin(angle)))
        bolt_center = _add_vec(center, _scale(radial, bolt_circle_radius))
        a = _sub_vec(bolt_center, _scale(n, bolt_length * 0.5))
        b = _add_vec(bolt_center, _scale(n, bolt_length * 0.5))
        _add_member(
            part,
            a,
            b,
            radius=bolt_radius,
            material=material,
            name=None if name_prefix is None else f"{name_prefix}_{idx}",
        )


def _circle_profile(
    radius: float, *, segments: int = 28, offset: tuple[float, float] = (0.0, 0.0)
) -> list[tuple[float, float]]:
    return [
        (
            offset[0] + radius * math.cos(math.tau * i / segments),
            offset[1] + radius * math.sin(math.tau * i / segments),
        )
        for i in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_tower_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.72, 0.60),
            (0.70, 3.40),
            (0.64, 7.80),
            (0.57, 11.30),
            (0.51, 14.02),
        ],
        [
            (0.68, 0.66),
            (0.665, 3.40),
            (0.615, 7.80),
            (0.548, 11.30),
            (0.488, 13.98),
        ],
        segments=84,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_bulkhead_mesh(
    *,
    z_height: float,
    y_width: float,
    thickness: float,
    central_hole_radius: float | None = None,
    vent_slots: bool = False,
) -> MeshGeometry:
    outer = rounded_rect_profile(z_height, y_width, radius=0.10, corner_segments=8)
    hole_profiles: list[list[tuple[float, float]]] = []
    if central_hole_radius is not None:
        hole_profiles.append(_circle_profile(central_hole_radius, segments=30))
    if vent_slots:
        slot = rounded_rect_profile(0.18, 0.28, radius=0.03, corner_segments=6)
        hole_profiles.extend(
            [
                _translate_profile(slot, 0.12, -0.38),
                _translate_profile(slot, 0.12, 0.0),
                _translate_profile(slot, 0.12, 0.38),
            ]
        )
    geometry = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_y(math.pi / 2.0)
    return geometry


def _build_hub_body_mesh() -> MeshGeometry:
    rear_drum = CylinderGeometry(radius=0.40, height=0.28, radial_segments=56).translate(
        0.0, 0.0, 0.28
    )
    center_drum = CylinderGeometry(
        radius=0.50, height=0.56, radial_segments=56
    ).translate(0.0, 0.0, 0.58)
    nose_cone = ConeGeometry(radius=0.50, height=0.86, radial_segments=56).translate(
        0.0, 0.0, 1.22
    )
    tip_stub = CylinderGeometry(radius=0.14, height=0.10, radial_segments=40).translate(
        0.0, 0.0, 1.70
    )
    return _merge_geometries([rear_drum, center_drum, nose_cone, tip_stub]).rotate_y(
        math.pi / 2.0
    )


def _blade_section_loop(
    span_y: float, chord: float, thickness: float, twist_deg: float
) -> list[tuple[float, float, float]]:
    twist = math.radians(twist_deg)
    airfoil = [
        (0.64 * chord, 0.00 * thickness),
        (0.52 * chord, 0.28 * thickness),
        (0.26 * chord, 0.56 * thickness),
        (-0.04 * chord, 0.64 * thickness),
        (-0.32 * chord, 0.46 * thickness),
        (-0.54 * chord, 0.16 * thickness),
        (-0.58 * chord, 0.00 * thickness),
        (-0.50 * chord, -0.08 * thickness),
        (-0.24 * chord, -0.22 * thickness),
        (0.08 * chord, -0.20 * thickness),
        (0.42 * chord, -0.10 * thickness),
        (0.60 * chord, -0.03 * thickness),
    ]
    points: list[tuple[float, float, float]] = []
    for x, z in airfoil:
        xr = x * math.cos(twist) + z * math.sin(twist)
        zr = -x * math.sin(twist) + z * math.cos(twist)
        points.append((xr, span_y, zr))
    return points


def _build_blade_mesh() -> MeshGeometry:
    sections = [
        _blade_section_loop(0.16, 0.54, 0.42, 18.0),
        _blade_section_loop(0.78, 1.08, 0.37, 15.0),
        _blade_section_loop(2.20, 0.86, 0.20, 9.0),
        _blade_section_loop(3.90, 0.52, 0.09, 4.0),
        _blade_section_loop(5.05, 0.18, 0.024, 1.0),
    ]
    return repair_loft(section_loft(sections))


def _blade_direction(phase: float) -> tuple[float, float, float]:
    return (0.0, math.cos(phase), math.sin(phase))


def _add_blade_part(
    model: ArticulatedObject,
    *,
    name: str,
    phase: float,
    blade_mesh,
    blade_material,
    root_material,
) -> None:
    direction = _blade_direction(phase)
    blade = model.part(name)
    root_flange_end = _scale(direction, 0.12)
    cuff_end = _scale(direction, 0.46)
    _add_member(
        blade,
        (0.0, 0.0, 0.0),
        root_flange_end,
        radius=0.19,
        material=root_material,
        name="blade_root_flange",
    )
    _add_member(
        blade,
        root_flange_end,
        cuff_end,
        radius=0.17,
        material=root_material,
        name="blade_root_cuff",
    )
    blade.visual(
        blade_mesh,
        origin=Origin(rpy=(phase, 0.0, 0.0)),
        material=blade_material,
        name="blade_shell",
    )
    blade.inertial = Inertial.from_geometry(
        Box((1.20, 5.15, 0.36)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 2.55 * math.cos(phase), 2.55 * math.sin(phase))),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wind_turbine", assets=ASSETS)

    foundation_concrete = model.material(
        "foundation_concrete", rgba=(0.60, 0.60, 0.58, 1.0)
    )
    tower_paint = model.material("tower_paint", rgba=(0.81, 0.84, 0.85, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.44, 0.52, 0.46, 1.0))
    blade_composite = model.material(
        "blade_composite", rgba=(0.90, 0.90, 0.87, 1.0)
    )
    root_guard = model.material("root_guard", rgba=(0.28, 0.30, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.64, 0.67, 1.0))
    service_orange = model.material("service_orange", rgba=(0.84, 0.35, 0.08, 1.0))

    tower_shell_mesh = _save_mesh(_build_tower_shell_mesh(), "tower_shell.obj")
    front_bulkhead_mesh = _save_mesh(
        _build_bulkhead_mesh(
            z_height=1.18,
            y_width=1.48,
            thickness=0.06,
            central_hole_radius=0.30,
        ),
        "front_bulkhead.obj",
    )
    rear_bulkhead_mesh = _save_mesh(
        _build_bulkhead_mesh(
            z_height=1.08,
            y_width=1.46,
            thickness=0.05,
            vent_slots=True,
        ),
        "rear_bulkhead.obj",
    )
    hub_body_mesh = _save_mesh(_build_hub_body_mesh(), "hub_body.obj")
    blade_mesh = _save_mesh(_build_blade_mesh(), "utility_blade.obj")

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=1.95, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=foundation_concrete,
        name="foundation_pad",
    )
    tower.visual(
        Cylinder(radius=1.18, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=foundation_concrete,
        name="foundation_plinth",
    )
    tower.visual(tower_shell_mesh, material=tower_paint, name="tower_shell")
    tower.visual(
        Cylinder(radius=0.88, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=dark_steel,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=0.54, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 14.06)),
        material=dark_steel,
        name="tower_top_adapter",
    )
    tower.visual(
        Cylinder(radius=0.62, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 14.21)),
        material=dark_steel,
        name="yaw_bearing",
    )
    tower.visual(
        Box((0.34, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, 0.70, 0.70)),
        material=service_orange,
        name="service_marking_bar",
    )
    _add_bolt_ring(
        tower,
        center=(0.0, 0.0, 0.71),
        normal=(0.0, 0.0, 1.0),
        bolt_circle_radius=0.74,
        bolt_radius=0.022,
        bolt_length=0.12,
        count=24,
        material=galvanized,
        name_prefix="anchor_bolt",
    )
    _add_bolt_ring(
        tower,
        center=(0.0, 0.0, 14.25),
        normal=(0.0, 0.0, 1.0),
        bolt_circle_radius=0.61,
        bolt_radius=0.015,
        bolt_length=0.08,
        count=20,
        material=galvanized,
        name_prefix="yaw_bolt",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.90, length=14.40),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 7.20)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.58, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="yaw_collar",
    )
    nacelle.visual(
        Box((2.92, 1.56, 0.20)),
        origin=Origin(xyz=(-0.40, 0.0, 0.18)),
        material=dark_steel,
        name="bedplate",
    )
    nacelle.visual(
        Box((2.72, 0.06, 1.02)),
        origin=Origin(xyz=(-0.50, -0.75, 0.75)),
        material=nacelle_paint,
        name="left_side_panel",
    )
    nacelle.visual(
        Box((2.72, 0.06, 1.02)),
        origin=Origin(xyz=(-0.50, 0.75, 0.75)),
        material=nacelle_paint,
        name="right_side_panel",
    )
    nacelle.visual(
        Box((2.54, 0.10, 0.30)),
        origin=Origin(xyz=(-0.44, -0.67, 0.34)),
        material=nacelle_paint,
        name="left_lower_skirt",
    )
    nacelle.visual(
        Box((2.54, 0.10, 0.30)),
        origin=Origin(xyz=(-0.44, 0.67, 0.34)),
        material=nacelle_paint,
        name="right_lower_skirt",
    )
    nacelle.visual(
        Box((2.10, 1.20, 0.08)),
        origin=Origin(xyz=(-0.62, 0.0, 1.29)),
        material=nacelle_paint,
        name="roof_panel",
    )
    nacelle.visual(
        Box((0.92, 1.10, 0.08)),
        origin=Origin(xyz=(0.42, 0.0, 1.12), rpy=(0.0, 0.26, 0.0)),
        material=nacelle_paint,
        name="front_hood",
    )
    nacelle.visual(
        front_bulkhead_mesh,
        origin=Origin(xyz=(0.86, 0.0, 0.83)),
        material=nacelle_paint,
        name="front_bulkhead",
    )
    nacelle.visual(
        rear_bulkhead_mesh,
        origin=Origin(xyz=(-1.86, 0.0, 0.78)),
        material=nacelle_paint,
        name="rear_bulkhead",
    )
    nacelle.visual(
        Cylinder(radius=0.29, length=0.52),
        origin=Origin(xyz=(1.12, 0.0, 0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="main_bearing_front",
    )
    nacelle.visual(
        Cylinder(radius=0.44, length=0.06),
        origin=Origin(xyz=(1.12, 0.0, 0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="main_bearing_flange",
    )
    nacelle.visual(
        Cylinder(radius=0.38, length=0.22),
        origin=Origin(xyz=(0.91, 0.0, 0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="main_bearing_base",
    )
    nacelle.visual(
        Box((1.96, 0.04, 0.12)),
        origin=Origin(xyz=(-0.42, -0.71, 0.96)),
        material=galvanized,
        name="left_reinforcement_rib",
    )
    nacelle.visual(
        Box((1.96, 0.04, 0.12)),
        origin=Origin(xyz=(-0.42, 0.71, 0.96)),
        material=galvanized,
        name="right_reinforcement_rib",
    )
    nacelle.visual(
        Box((0.42, 0.36, 0.16)),
        origin=Origin(xyz=(-1.02, 0.0, 1.41)),
        material=dark_steel,
        name="roof_service_box",
    )
    nacelle.visual(
        Box((0.24, 0.02, 0.10)),
        origin=Origin(xyz=(-0.86, 0.77, 1.23)),
        material=service_orange,
        name="inspection_marking",
    )
    _add_bolt_ring(
        nacelle,
        center=(0.0, 0.0, 0.14),
        normal=(0.0, 0.0, 1.0),
        bolt_circle_radius=0.66,
        bolt_radius=0.017,
        bolt_length=0.07,
        count=20,
        material=galvanized,
        name_prefix="yaw_collar_bolt",
    )
    _add_bolt_ring(
        nacelle,
        center=(1.12, 0.0, 0.92),
        normal=(1.0, 0.0, 0.0),
        bolt_circle_radius=0.38,
        bolt_radius=0.014,
        bolt_length=0.08,
        count=16,
        material=galvanized,
        name_prefix="front_bearing_bolt",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((3.10, 1.70, 1.60)),
        mass=980.0,
        origin=Origin(xyz=(-0.35, 0.0, 0.82)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.36, length=0.14),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_rear_flange",
    )
    rotor.visual(hub_body_mesh, material=root_guard, name="hub_shell")
    rotor.visual(
        Cylinder(radius=0.22, length=0.22),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft_collar",
    )

    blade_specs = [
        ("blade_a", math.pi / 2.0, "pitch_flange_a"),
        ("blade_b", math.pi / 2.0 + (2.0 * math.pi / 3.0), "pitch_flange_b"),
        ("blade_c", math.pi / 2.0 - (2.0 * math.pi / 3.0), "pitch_flange_c"),
    ]
    blade_root_points: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    for _, phase, flange_name in blade_specs:
        direction = _blade_direction(phase)
        collar_inner = (0.42, 0.48 * direction[1], 0.48 * direction[2])
        root = (0.42, 0.72 * direction[1], 0.72 * direction[2])
        blade_root_points[flange_name] = (collar_inner, root)
        _add_member(
            rotor,
            (0.18, 0.0, 0.0),
            collar_inner,
            radius=0.11,
            material=dark_steel,
            name=f"{flange_name}_support_arm",
        )
        _add_member(
            rotor,
            collar_inner,
            root,
            radius=0.21,
            material=dark_steel,
            name=flange_name,
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.98, length=1.48),
        mass=460.0,
        origin=Origin(xyz=(0.54, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    for blade_name, phase, _ in blade_specs:
        _add_blade_part(
            model,
            name=blade_name,
            phase=phase,
            blade_mesh=blade_mesh,
            blade_material=blade_composite,
            root_material=root_guard,
        )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 14.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6500.0, velocity=0.20),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(1.38, 0.0, 0.92)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=1.2),
    )
    for blade_name, phase, flange_name in blade_specs:
        direction = _blade_direction(phase)
        _, root = blade_root_points[flange_name]
        model.articulation(
            f"rotor_to_{blade_name}_pitch",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=blade_name,
            origin=Origin(xyz=root),
            axis=direction,
            motion_limits=MotionLimits(
                effort=380.0,
                velocity=0.55,
                lower=math.radians(-8.0),
                upper=math.radians(88.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    blade_a = object_model.get_part("blade_a")
    blade_b = object_model.get_part("blade_b")
    blade_c = object_model.get_part("blade_c")

    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    rotor_spin = object_model.get_articulation("nacelle_to_rotor_spin")
    pitch_a = object_model.get_articulation("rotor_to_blade_a_pitch")
    pitch_b = object_model.get_articulation("rotor_to_blade_b_pitch")
    pitch_c = object_model.get_articulation("rotor_to_blade_c_pitch")

    yaw_bearing = tower.get_visual("yaw_bearing")
    yaw_collar = nacelle.get_visual("yaw_collar")
    main_bearing_front = nacelle.get_visual("main_bearing_front")
    hub_rear_flange = rotor.get_visual("hub_rear_flange")
    pitch_flange_a = rotor.get_visual("pitch_flange_a")
    pitch_flange_b = rotor.get_visual("pitch_flange_b")
    pitch_flange_c = rotor.get_visual("pitch_flange_c")
    blade_root_flange_a = blade_a.get_visual("blade_root_flange")
    blade_root_flange_b = blade_b.get_visual("blade_root_flange")
    blade_root_flange_c = blade_c.get_visual("blade_root_flange")
    blade_shell_a = blade_a.get_visual("blade_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        blade_a,
        rotor,
        elem_a=blade_root_flange_a,
        elem_b=pitch_flange_a,
        reason="Simplified pitch-bearing race modeled as nested reinforced collars at blade A root.",
    )
    ctx.allow_overlap(
        blade_b,
        rotor,
        elem_a=blade_root_flange_b,
        elem_b=pitch_flange_b,
        reason="Simplified pitch-bearing race modeled as nested reinforced collars at blade B root.",
    )
    ctx.allow_overlap(
        blade_c,
        rotor,
        elem_a=blade_root_flange_c,
        elem_b=pitch_flange_c,
        reason="Simplified pitch-bearing race modeled as nested reinforced collars at blade C root.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=28,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem=yaw_collar,
        negative_elem=yaw_bearing,
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw_ring_seats_on_tower_bearing",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        elem_a=yaw_collar,
        elem_b=yaw_bearing,
        min_overlap=1.00,
        name="yaw_ring_has_broad_bearing_overlap",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem=hub_rear_flange,
        negative_elem=main_bearing_front,
        max_gap=0.001,
        max_penetration=1e-5,
        name="rotor_hub_seats_on_main_bearing_face",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="yz",
        elem_a=hub_rear_flange,
        elem_b=main_bearing_front,
        min_overlap=0.55,
        name="rotor_hub_has_broad_main_bearing_overlap",
    )
    ctx.expect_contact(
        blade_a,
        rotor,
        elem_a=blade_root_flange_a,
        elem_b=pitch_flange_a,
        name="blade_a_root_contacts_pitch_flange",
    )
    ctx.expect_contact(
        blade_b,
        rotor,
        elem_a=blade_root_flange_b,
        elem_b=pitch_flange_b,
        name="blade_b_root_contacts_pitch_flange",
    )
    ctx.expect_contact(
        blade_c,
        rotor,
        elem_a=blade_root_flange_c,
        elem_b=pitch_flange_c,
        name="blade_c_root_contacts_pitch_flange",
    )
    ctx.expect_origin_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=1.20,
        name="rotor_projects_forward_of_nacelle",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    if tower_aabb is None:
        ctx.fail("tower_aabb_present", "Tower AABB could not be resolved.")
    else:
        tower_height = tower_aabb[1][2] - tower_aabb[0][2]
        ctx.check(
            "tower_height_realistic",
            tower_height > 14.0,
            f"Expected a utility-scale tower taller than 14 m, got {tower_height:.3f} m.",
        )

    yaw_bearing_aabb = ctx.part_element_world_aabb(tower, elem=yaw_bearing)
    if yaw_bearing_aabb is None:
        ctx.fail("yaw_bearing_present", "Yaw bearing visual AABB could not be resolved.")
    else:
        yaw_diameter = max(
            yaw_bearing_aabb[1][0] - yaw_bearing_aabb[0][0],
            yaw_bearing_aabb[1][1] - yaw_bearing_aabb[0][1],
        )
        ctx.check(
            "yaw_bearing_is_heavy_duty",
            yaw_diameter > 1.20,
            f"Yaw bearing should read as a reinforced ring; got diameter {yaw_diameter:.3f} m.",
        )

    pitch_flange_aabb = ctx.part_element_world_aabb(rotor, elem=pitch_flange_a)
    if pitch_flange_aabb is None:
        ctx.fail("pitch_flange_a_present", "Pitch flange AABB could not be resolved.")
    else:
        pitch_flange_diameter = max(
            pitch_flange_aabb[1][1] - pitch_flange_aabb[0][1],
            pitch_flange_aabb[1][2] - pitch_flange_aabb[0][2],
        )
        ctx.check(
            "pitch_flange_reads_as_reinforced_interface",
            pitch_flange_diameter >= 0.40,
            f"Pitch flange should be thick and serviceable; got {pitch_flange_diameter:.3f} m.",
        )

    blade_aabb = ctx.part_world_aabb(blade_a)
    if blade_aabb is None:
        ctx.fail("blade_aabb_present", "Blade AABB could not be resolved.")
    else:
        blade_span = blade_aabb[1][2] - blade_aabb[0][2]
        ctx.check(
            "blade_span_realistic",
            blade_span > 4.8,
            f"Expected a multi-meter utility blade span, got {blade_span:.3f} m.",
        )

    rotor_rest = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        rotor_yawed = ctx.part_world_position(rotor)
        if rotor_rest is None or rotor_yawed is None:
            ctx.fail("yaw_motion_resolves_positions", "Rotor positions could not be measured across yaw.")
        else:
            ctx.check(
                "yaw_axis_rotates_rotor_around_tower",
                rotor_rest[0] > 1.0
                and abs(rotor_yawed[0]) < 0.05
                and rotor_yawed[1] > 1.0,
                f"Unexpected rotor positions across yaw: rest={rotor_rest}, yawed={rotor_yawed}.",
            )
        ctx.expect_contact(
            nacelle,
            tower,
            elem_a=yaw_collar,
            elem_b=yaw_bearing,
            name="yaw_pose_keeps_bearing_contact",
        )
        ctx.fail_if_isolated_parts(name="yaw_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="yaw_pose_no_overlap")

    blade_a_origin_rest = ctx.part_world_position(blade_a)
    with ctx.pose({rotor_spin: math.tau / 3.0}):
        blade_a_origin_spun = ctx.part_world_position(blade_a)
        if blade_a_origin_rest is None or blade_a_origin_spun is None:
            ctx.fail("spin_motion_resolves_positions", "Blade A origin positions could not be measured.")
        else:
            ctx.check(
                "rotor_spin_moves_blade_root",
                abs(blade_a_origin_spun[1] - blade_a_origin_rest[1]) > 0.40
                and abs(blade_a_origin_spun[2] - blade_a_origin_rest[2]) > 0.40,
                f"Blade A root should orbit with hub spin: rest={blade_a_origin_rest}, spun={blade_a_origin_spun}.",
            )
        ctx.expect_contact(
            blade_a,
            rotor,
            elem_a=blade_root_flange_a,
            elem_b=pitch_flange_a,
            name="spin_pose_keeps_blade_a_seated",
        )
        ctx.fail_if_isolated_parts(name="spin_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="spin_pose_no_overlap")

    blade_shell_rest = ctx.part_element_world_aabb(blade_a, elem=blade_shell_a)
    with ctx.pose({pitch_a: math.radians(70.0)}):
        blade_shell_pitched = ctx.part_element_world_aabb(blade_a, elem=blade_shell_a)
        if blade_shell_rest is None or blade_shell_pitched is None:
            ctx.fail("pitch_motion_resolves_blade_shell", "Blade shell AABBs could not be measured.")
        else:
            rest_y_span = blade_shell_rest[1][1] - blade_shell_rest[0][1]
            pitch_y_span = blade_shell_pitched[1][1] - blade_shell_pitched[0][1]
            ctx.check(
                "pitch_axis_rolls_airfoil_section",
                pitch_y_span > rest_y_span + 0.18,
                f"Blade pitch should roll the airfoil about its span; rest y-span={rest_y_span:.3f}, pitched y-span={pitch_y_span:.3f}.",
            )
        ctx.expect_contact(
            blade_a,
            rotor,
            elem_a=blade_root_flange_a,
            elem_b=pitch_flange_a,
            name="pitched_blade_a_keeps_root_contact",
        )
        ctx.fail_if_isolated_parts(name="pitched_blade_a_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="pitched_blade_a_no_overlap")

    for blade, pitch_joint, root_flange, pitch_flange, label in [
        (blade_a, pitch_a, blade_root_flange_a, pitch_flange_a, "a"),
        (blade_b, pitch_b, blade_root_flange_b, pitch_flange_b, "b"),
        (blade_c, pitch_c, blade_root_flange_c, pitch_flange_c, "c"),
    ]:
        limits = pitch_joint.motion_limits
        if (
            limits is None
            or limits.lower is None
            or limits.upper is None
        ):
            ctx.fail(f"blade_{label}_limits_present", "Pitch limits must be finite.")
            continue
        with ctx.pose({pitch_joint: limits.lower}):
            ctx.expect_contact(
                blade,
                rotor,
                elem_a=root_flange,
                elem_b=pitch_flange,
                name=f"blade_{label}_lower_limit_keeps_contact",
            )
            ctx.fail_if_isolated_parts(name=f"blade_{label}_lower_limit_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"blade_{label}_lower_limit_no_overlap"
            )
        with ctx.pose({pitch_joint: limits.upper}):
            ctx.expect_contact(
                blade,
                rotor,
                elem_a=root_flange,
                elem_b=pitch_flange,
                name=f"blade_{label}_upper_limit_keeps_contact",
            )
            ctx.fail_if_isolated_parts(name=f"blade_{label}_upper_limit_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"blade_{label}_upper_limit_no_overlap"
            )

    with ctx.pose(
        {
            yaw: 1.15,
            rotor_spin: 1.45,
            pitch_a: math.radians(82.0),
            pitch_b: math.radians(82.0),
            pitch_c: math.radians(82.0),
        }
    ):
        ctx.fail_if_isolated_parts(name="parked_feathered_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="parked_feathered_pose_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
