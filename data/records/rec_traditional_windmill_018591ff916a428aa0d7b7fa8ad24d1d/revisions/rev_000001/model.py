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


def _octagon_loop(rx: float, ry: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (rx * math.cos(angle), ry * math.sin(angle), z)
        for angle in [index * math.tau / 8.0 for index in range(8)]
    ]


def _ring_shell_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float, axis: str):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -0.5 * length), (outer_radius, 0.5 * length)],
        [(inner_radius, -0.5 * length), (inner_radius, 0.5 * length)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        shell.rotate_y(math.pi / 2.0)
    return _save_mesh(name, shell)


def _bolt_circle_x(
    part,
    *,
    x: float,
    radius: float,
    bolt_radius: float,
    length: float,
    count: int,
    material,
    phase: float = 0.0,
) -> None:
    for index in range(count):
        angle = phase + (index * math.tau / count)
        part.visual(
            Cylinder(radius=bolt_radius, length=length),
            origin=Origin(
                xyz=(x, radius * math.cos(angle), radius * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
        )


def _bolt_circle_z(
    part,
    *,
    z: float,
    radius: float,
    bolt_radius: float,
    length: float,
    count: int,
    material,
    phase: float = 0.0,
) -> None:
    for index in range(count):
        angle = phase + (index * math.tau / count)
        part.visual(
            Cylinder(radius=bolt_radius, length=length),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
            ),
            material=material,
        )


def _rotate_about_x(y: float, z: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (y * c - z * s, y * s + z * c)


def _blade_box_origin(
    *,
    x: float,
    local_y: float,
    local_z: float,
    roll: float,
) -> Origin:
    y, z = _rotate_about_x(local_y, local_z, roll)
    return Origin(xyz=(x, y, z), rpy=(roll, 0.0, 0.0))


def _rotated_point(x: float, local_y: float, local_z: float, roll: float) -> tuple[float, float, float]:
    y, z = _rotate_about_x(local_y, local_z, roll)
    return (x, y, z)


def _add_lattice_blade(part, *, roll: float, wood_material, steel_material) -> None:
    part.visual(
        Box((0.18, 0.18, 0.58)),
        origin=_blade_box_origin(x=0.30, local_y=0.0, local_z=0.23, roll=roll),
        material=steel_material,
    )
    part.visual(
        Box((0.16, 0.18, 1.02)),
        origin=_blade_box_origin(x=0.46, local_y=0.0, local_z=0.96, roll=roll),
        material=steel_material,
    )
    for side in (-0.34, 0.34):
        part.visual(
            Box((0.06, 0.06, 2.52)),
            origin=_blade_box_origin(x=0.53, local_y=side, local_z=2.23, roll=roll),
            material=wood_material,
        )
    for radial, chord in [
        (0.92, 0.82),
        (1.34, 0.80),
        (1.76, 0.78),
        (2.18, 0.74),
        (2.60, 0.68),
        (3.02, 0.62),
        (3.38, 0.70),
    ]:
        part.visual(
            Box((0.042, chord, 0.052)),
            origin=_blade_box_origin(x=0.52, local_y=0.0, local_z=radial, roll=roll),
            material=wood_material,
        )
    part.visual(
        Box((0.05, 0.76, 0.07)),
        origin=_blade_box_origin(x=0.50, local_y=0.0, local_z=0.86, roll=roll),
        material=steel_material,
    )
    part.visual(
        Box((0.05, 0.72, 0.07)),
        origin=_blade_box_origin(x=0.56, local_y=0.0, local_z=3.52, roll=roll),
        material=steel_material,
    )
    part.visual(
        Box((0.20, 0.30, 0.34)),
        origin=_blade_box_origin(x=0.28, local_y=0.0, local_z=0.47, roll=roll),
        material=steel_material,
    )
    _add_member(
        part,
        _rotated_point(0.50, -0.34, 1.00, roll),
        _rotated_point(0.56, 0.34, 2.95, roll),
        0.020,
        steel_material,
    )
    _add_member(
        part,
        _rotated_point(0.50, 0.34, 1.00, roll),
        _rotated_point(0.56, -0.34, 2.95, roll),
        0.020,
        steel_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_first_traditional_windmill")

    painted_white = model.material("painted_white", rgba=(0.86, 0.86, 0.82, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.61, 0.56, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    medium_steel = model.material("medium_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    light_steel = model.material("light_steel", rgba=(0.68, 0.70, 0.74, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.80, 0.69, 0.14, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.88, 0.42, 0.12, 1.0))
    brake_red = model.material("brake_red", rgba=(0.70, 0.14, 0.11, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((5.40, 5.40, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=dark_steel,
        name="foundation_plinth",
    )
    tower.visual(
        Box((4.30, 4.30, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        material=medium_steel,
        name="base_transition_plate",
    )
    tower.visual(
        _save_mesh(
            "windmill_tower_shell",
            section_loft(
                [
                    _octagon_loop(2.20, 2.05, 0.60),
                    _octagon_loop(2.02, 1.88, 3.20),
                    _octagon_loop(1.82, 1.70, 6.30),
                    _octagon_loop(1.58, 1.48, 9.20),
                    _octagon_loop(1.46, 1.36, 10.55),
                ]
            ),
        ),
        material=painted_white,
        name="tower_shell",
    )
    tower.visual(
        _ring_shell_mesh(
            "tower_band_low",
            outer_radius=2.14,
            inner_radius=2.00,
            length=0.16,
            axis="z",
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.20)),
        material=medium_steel,
        name="band_low",
    )
    tower.visual(
        _ring_shell_mesh(
            "tower_band_mid",
            outer_radius=1.90,
            inner_radius=1.76,
            length=0.16,
            axis="z",
        ),
        origin=Origin(xyz=(0.0, 0.0, 6.85)),
        material=medium_steel,
        name="band_mid",
    )
    tower.visual(
        Cylinder(radius=1.55, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 10.66)),
        material=dark_steel,
        name="yaw_curb",
    )
    tower.visual(
        _ring_shell_mesh(
            "service_platform_ring",
            outer_radius=2.18,
            inner_radius=1.70,
            length=0.10,
            axis="z",
        ),
        origin=Origin(xyz=(0.0, 0.0, 10.30)),
        material=medium_steel,
        name="service_platform_ring",
    )
    for angle in [index * math.tau / 8.0 for index in range(8)]:
        shell_x = 1.56 * math.cos(angle)
        shell_y = 1.46 * math.sin(angle)
        ring_x = 1.96 * math.cos(angle)
        ring_y = 1.96 * math.sin(angle)
        _add_member(
            tower,
            (shell_x, shell_y, 10.30),
            (ring_x, ring_y, 10.30),
            0.040,
            medium_steel,
        )
        _add_member(
            tower,
            (1.34 * math.cos(angle), 1.24 * math.sin(angle), 10.30),
            (ring_x, ring_y, 10.30),
            0.085,
            medium_steel,
        )
        tower.visual(
            Cylinder(radius=0.025, length=0.66),
            origin=Origin(xyz=(2.18 * math.cos(angle), 2.18 * math.sin(angle), 10.63)),
            material=safety_yellow,
        )
    tower.visual(
        Box((2.40, 2.20, 9.95)),
        origin=Origin(xyz=(0.0, 0.0, 5.575)),
        material=medium_steel,
        name="internal_tower_spine",
    )
    tower.visual(
        _save_mesh(
            "service_top_rail",
            TorusGeometry(radius=2.18, tube=0.040, radial_segments=16, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 10.96)),
        material=safety_yellow,
        name="service_top_rail",
    )
    tower.visual(
        _save_mesh(
            "service_mid_rail",
            TorusGeometry(radius=2.18, tube=0.028, radial_segments=14, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 10.63)),
        material=safety_yellow,
        name="service_mid_rail",
    )
    tower.visual(
        Box((0.16, 0.18, 0.28)),
        origin=Origin(xyz=(1.96, 0.56, 10.96)),
        material=brake_red,
        name="yaw_stop_port",
    )
    tower.visual(
        Box((0.16, 0.18, 0.28)),
        origin=Origin(xyz=(1.96, -0.56, 10.96)),
        material=brake_red,
        name="yaw_stop_starboard",
    )
    _bolt_circle_z(
        tower,
        z=0.67,
        radius=1.84,
        bolt_radius=0.030,
        length=0.32,
        count=8,
        material=light_steel,
        phase=math.pi / 8.0,
    )
    tower.inertial = Inertial.from_geometry(
        Box((5.40, 5.40, 10.98)),
        mass=22000.0,
        origin=Origin(xyz=(0.0, 0.0, 5.49)),
    )

    cap = model.part("cap")
    cap.visual(
        _ring_shell_mesh(
            "cap_yaw_ring",
            outer_radius=1.62,
            inner_radius=1.18,
            length=0.18,
            axis="z",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="yaw_ring",
    )
    cap.visual(
        Box((3.00, 2.30, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=medium_steel,
        name="deck_plate",
    )
    cap.visual(
        Box((1.90, 0.10, 0.86)),
        origin=Origin(xyz=(-0.20, 1.00, 0.57)),
        material=dark_steel,
        name="port_side_plate",
    )
    cap.visual(
        Box((1.90, 0.10, 0.86)),
        origin=Origin(xyz=(-0.20, -1.00, 0.57)),
        material=dark_steel,
        name="starboard_side_plate",
    )
    cap.visual(
        Box((0.12, 1.92, 0.76)),
        origin=Origin(xyz=(-1.10, 0.0, 0.56)),
        material=dark_steel,
        name="rear_bulkhead",
    )
    cap.visual(
        Box((2.25, 1.10, 0.08)),
        origin=Origin(xyz=(-0.05, 0.54, 1.34), rpy=(0.50, 0.0, 0.0)),
        material=medium_steel,
        name="port_roof_panel",
    )
    cap.visual(
        Box((2.25, 1.10, 0.08)),
        origin=Origin(xyz=(-0.05, -0.54, 1.34), rpy=(-0.50, 0.0, 0.0)),
        material=medium_steel,
        name="starboard_roof_panel",
    )
    cap.visual(
        Box((2.10, 0.28, 0.22)),
        origin=Origin(xyz=(-0.08, 0.0, 1.47)),
        material=dark_steel,
        name="ridge_beam",
    )
    cap.visual(
        Box((0.56, 0.22, 1.02)),
        origin=Origin(xyz=(2.34, 0.46, 0.71)),
        material=medium_steel,
        name="front_cheek_port",
    )
    cap.visual(
        Box((0.56, 0.22, 1.02)),
        origin=Origin(xyz=(2.34, -0.46, 0.71)),
        material=medium_steel,
        name="front_cheek_starboard",
    )
    cap.visual(
        Box((0.28, 0.92, 0.22)),
        origin=Origin(xyz=(2.34, 0.0, 0.60)),
        material=dark_steel,
        name="front_cross_tie",
    )
    cap.visual(
        Box((2.72, 0.24, 0.22)),
        origin=Origin(xyz=(1.66, 0.36, 0.70)),
        material=dark_steel,
        name="bed_rail_port",
    )
    cap.visual(
        Box((2.72, 0.24, 0.22)),
        origin=Origin(xyz=(1.66, -0.36, 0.70)),
        material=dark_steel,
        name="bed_rail_starboard",
    )
    cap.visual(
        Box((0.18, 0.18, 0.60)),
        origin=Origin(xyz=(1.22, 0.28, 0.48)),
        material=medium_steel,
        name="rear_post_port",
    )
    cap.visual(
        Box((0.18, 0.18, 0.60)),
        origin=Origin(xyz=(1.22, -0.28, 0.48)),
        material=medium_steel,
        name="rear_post_starboard",
    )
    cap.visual(
        Box((0.34, 0.18, 0.24)),
        origin=Origin(xyz=(2.84, 0.36, 1.05)),
        material=medium_steel,
        name="front_bearing_saddle_port",
    )
    cap.visual(
        Box((0.34, 0.18, 0.24)),
        origin=Origin(xyz=(2.84, -0.36, 1.05)),
        material=medium_steel,
        name="front_bearing_saddle_starboard",
    )
    cap.visual(
        Box((0.28, 0.16, 0.20)),
        origin=Origin(xyz=(2.08, 0.28, 1.03)),
        material=medium_steel,
        name="rear_bearing_saddle_port",
    )
    cap.visual(
        Box((0.28, 0.16, 0.20)),
        origin=Origin(xyz=(2.08, -0.28, 1.03)),
        material=medium_steel,
        name="rear_bearing_saddle_starboard",
    )
    _add_member(cap, (0.74, 0.40, 0.18), (1.56, 0.22, 0.86), 0.045, medium_steel)
    _add_member(cap, (0.74, -0.40, 0.18), (1.56, -0.22, 0.86), 0.045, medium_steel)
    _add_member(cap, (1.56, 0.48, 0.18), (2.34, 0.18, 0.86), 0.050, medium_steel)
    _add_member(cap, (1.56, -0.48, 0.18), (2.34, -0.18, 0.86), 0.050, medium_steel)
    _add_member(cap, (1.66, 0.36, 0.80), (2.84, 0.20, 0.98), 0.040, medium_steel)
    _add_member(cap, (1.66, -0.36, 0.80), (2.84, -0.20, 0.98), 0.040, medium_steel)
    cap.visual(
        _ring_shell_mesh(
            "front_bearing_seat_mesh",
            outer_radius=0.38,
            inner_radius=0.16,
            length=0.16,
            axis="x",
        ),
        origin=Origin(xyz=(2.96, 0.0, 1.05)),
        material=light_steel,
        name="front_bearing_seat",
    )
    cap.visual(
        _ring_shell_mesh(
            "rear_bearing_seat_mesh",
            outer_radius=0.32,
            inner_radius=0.16,
            length=0.16,
            axis="x",
        ),
        origin=Origin(xyz=(2.08, 0.0, 1.03)),
        material=light_steel,
        name="rear_bearing_seat",
    )
    _add_member(cap, (0.58, 0.98, 0.98), (0.58, 0.00, 1.46), 0.060, medium_steel)
    _add_member(cap, (-0.72, 0.98, 0.98), (-0.72, 0.00, 1.46), 0.060, medium_steel)
    _add_member(cap, (0.58, -0.98, 0.98), (0.58, 0.00, 1.46), 0.060, medium_steel)
    _add_member(cap, (-0.72, -0.98, 0.98), (-0.72, 0.00, 1.46), 0.060, medium_steel)
    cap.visual(
        Box((0.86, 0.08, 0.88)),
        origin=Origin(xyz=(2.52, 0.44, 1.05)),
        material=safety_yellow,
        name="front_guard_side_frame_port",
    )
    cap.visual(
        Box((0.86, 0.08, 0.88)),
        origin=Origin(xyz=(2.52, -0.44, 1.05)),
        material=safety_yellow,
        name="front_guard_side_frame_starboard",
    )
    for x_plane in (2.24, 2.80):
        cap.visual(
            Box((0.08, 0.08, 0.70)),
            origin=Origin(xyz=(x_plane, 0.44, 1.05)),
            material=safety_yellow,
        )
        cap.visual(
            Box((0.08, 0.08, 0.70)),
            origin=Origin(xyz=(x_plane, -0.44, 1.05)),
            material=safety_yellow,
        )
        cap.visual(
            Cylinder(radius=0.030, length=0.88),
            origin=Origin(xyz=(x_plane, 0.0, 1.38), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
        )
        cap.visual(
            Cylinder(radius=0.030, length=0.88),
            origin=Origin(xyz=(x_plane, 0.0, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
        )
    for y_pos in (-0.44, 0.44):
        cap.visual(
            Cylinder(radius=0.026, length=0.36),
            origin=Origin(xyz=(2.52, y_pos, 1.38), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=safety_yellow,
        )
        cap.visual(
            Cylinder(radius=0.026, length=0.36),
            origin=Origin(xyz=(2.52, y_pos, 0.72), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=safety_yellow,
        )
    cap.visual(
        Box((0.28, 0.10, 0.18)),
        origin=Origin(xyz=(-0.12, 1.12, 0.26)),
        material=safety_orange,
        name="yaw_lockout_body",
    )
    cap.visual(
        Cylinder(radius=0.030, length=0.18),
        origin=Origin(xyz=(-0.02, 1.14, 0.32), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_steel,
        name="yaw_lockout_pin",
    )
    cap.visual(
        Box((0.12, 0.14, 0.24)),
        origin=Origin(xyz=(1.66, 0.70, 0.18)),
        material=brake_red,
        name="cap_stop_ear_port",
    )
    cap.visual(
        Box((0.12, 0.14, 0.24)),
        origin=Origin(xyz=(1.66, -0.70, 0.18)),
        material=brake_red,
        name="cap_stop_ear_starboard",
    )
    _add_member(cap, (1.66, 0.70, 0.18), (1.56, 0.46, 0.32), 0.050, dark_steel)
    _add_member(cap, (1.66, -0.70, 0.18), (1.56, -0.46, 0.32), 0.050, dark_steel)
    cap.inertial = Inertial.from_geometry(
        Box((5.20, 2.30, 1.70)),
        mass=4200.0,
        origin=Origin(xyz=(0.75, 0.0, 0.85)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.11, length=2.20),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_steel,
        name="main_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.18),
        origin=Origin(xyz=(0.28, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_root_collar",
    )
    rotor.visual(
        Cylinder(radius=0.20, length=0.14),
        origin=Origin(xyz=(0.15, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=medium_steel,
        name="front_thrust_collar",
    )
    rotor.visual(
        Cylinder(radius=0.44, length=0.12),
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.34, length=0.78),
        origin=Origin(xyz=(0.86, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=medium_steel,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.24, length=0.32),
        origin=Origin(xyz=(1.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=medium_steel,
        name="nose_spigot",
    )
    rotor.visual(
        Sphere(radius=0.22),
        origin=Origin(xyz=(1.60, 0.0, 0.0)),
        material=medium_steel,
        name="nose_cap",
    )
    rotor.visual(
        Cylinder(radius=0.30, length=0.045),
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brake_red,
        name="lock_disc",
    )
    rotor.visual(
        Cylinder(radius=0.24, length=0.06),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="brake_spider",
    )
    _bolt_circle_x(
        rotor,
        x=0.18,
        radius=0.37,
        bolt_radius=0.020,
        length=0.14,
        count=8,
        material=light_steel,
        phase=math.pi / 8.0,
    )
    for blade_roll in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        _add_lattice_blade(
            rotor,
            roll=blade_roll,
            wood_material=weathered_wood,
            steel_material=dark_steel,
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=4.05, length=2.70),
        mass=1800.0,
        origin=Origin(xyz=(0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 10.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28000.0,
            velocity=0.12,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(2.96, 0.0, 1.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14000.0,
            velocity=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cap, tower, elem_a="yaw_ring", elem_b="yaw_curb", name="cap_seats_on_yaw_curb")
    ctx.expect_overlap(
        rotor,
        cap,
        elem_a="main_shaft",
        elem_b="front_bearing_seat",
        axes="yz",
        min_overlap=0.20,
        name="front_bearing_seat_wraps_shaft",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        elem_a="main_shaft",
        elem_b="rear_bearing_seat",
        axes="yz",
        min_overlap=0.20,
        name="rear_bearing_seat_wraps_shaft",
    )
    ctx.expect_contact(
        rotor,
        cap,
        elem_a="front_thrust_collar",
        elem_b="front_bearing_seat",
        name="front_thrust_collar_contacts_front_bearing",
    )
    ctx.expect_overlap(rotor, cap, axes="yz", min_overlap=0.24, name="rotor_axis_aligned_with_cap")
    ctx.expect_origin_gap(rotor, tower, axis="x", min_gap=1.20, name="rotor_projects_forward")
    ctx.check("yaw_axis_is_vertical", yaw.axis == (0.0, 0.0, 1.0), f"axis={yaw.axis}")
    ctx.check("rotor_axis_is_longitudinal", spin.axis == (1.0, 0.0, 0.0), f"axis={spin.axis}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
