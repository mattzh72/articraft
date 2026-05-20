from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_tower_shell() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.0),
            (2.55, 0.0),
            (2.35, 4.0),
            (2.10, 18.0),
            (1.82, 42.0),
            (1.55, 63.0),
            (1.34, 78.2),
            (0.0, 78.2),
        ],
        segments=88,
    )


def _superellipse_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    center_z: float,
    exponent: float = 2.6,
    samples: int = 28,
) -> list[tuple[float, float, float]]:
    section: list[tuple[float, float, float]] = []
    width_radius = width * 0.5
    height_radius = height * 0.5

    for index in range(samples):
        angle = math.tau * index / samples
        c = math.cos(angle)
        s = math.sin(angle)
        y = width_radius * math.copysign(abs(c) ** (2.0 / exponent), c)
        z = center_z + height_radius * math.copysign(abs(s) ** (2.0 / exponent), s)
        section.append((x_pos, y, z))

    return section


def _build_nacelle_body() -> MeshGeometry:
    sections = [
        _superellipse_section(-4.9, width=2.0, height=2.1, center_z=2.55, exponent=2.2),
        _superellipse_section(-3.3, width=2.8, height=3.1, center_z=2.80, exponent=2.4),
        _superellipse_section(-0.8, width=3.8, height=4.2, center_z=3.15, exponent=2.7),
        _superellipse_section(2.2, width=4.2, height=4.8, center_z=3.40, exponent=2.9),
        _superellipse_section(4.8, width=3.4, height=4.0, center_z=3.42, exponent=2.8),
        _superellipse_section(5.8, width=2.2, height=2.5, center_z=3.35, exponent=2.5),
        _superellipse_section(6.4, width=1.1, height=1.2, center_z=3.35, exponent=2.2),
    ]
    return repair_loft(section_loft(sections))


def _blade_section(
    span_y: float,
    *,
    chord: float,
    thickness: float,
    sweep_x: float,
    pitch_deg: float,
) -> list[tuple[float, float, float]]:
    airfoil = (
        (0.52, 0.00),
        (0.28, 0.48),
        (-0.02, 0.42),
        (-0.32, 0.22),
        (-0.58, 0.02),
        (-0.48, -0.08),
        (-0.10, -0.22),
        (0.24, -0.26),
    )
    pitch = math.radians(pitch_deg)
    pivot_x = sweep_x - 0.08 * chord
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)

    section: list[tuple[float, float, float]] = []
    for x_norm, z_norm in airfoil:
        x_val = sweep_x + x_norm * chord
        z_val = z_norm * thickness
        dx = x_val - pivot_x
        x_rot = pivot_x + dx * cos_pitch - z_val * sin_pitch
        z_rot = dx * sin_pitch + z_val * cos_pitch
        section.append((x_rot, span_y, z_rot))

    return section


def _build_blade() -> MeshGeometry:
    sections = [
        _blade_section(1.45, chord=3.45, thickness=0.72, sweep_x=2.15, pitch_deg=18.0),
        _blade_section(5.2, chord=2.85, thickness=0.50, sweep_x=2.45, pitch_deg=12.0),
        _blade_section(12.8, chord=1.90, thickness=0.27, sweep_x=2.95, pitch_deg=6.0),
        _blade_section(20.2, chord=0.98, thickness=0.12, sweep_x=3.25, pitch_deg=3.0),
        _blade_section(24.7, chord=0.58, thickness=0.060, sweep_x=3.38, pitch_deg=1.0),
    ]
    return repair_loft(section_loft(sections))


def _build_hub_shell() -> MeshGeometry:
    spinner = LatheGeometry(
        [
            (0.0, 0.0),
            (1.58, 0.0),
            (1.48, 0.34),
            (1.20, 1.30),
            (0.72, 2.55),
            (0.18, 3.65),
            (0.0, 4.10),
        ],
        segments=72,
    ).rotate_y(math.pi / 2.0)

    barrel = LatheGeometry(
        [
            (0.0, 0.0),
            (1.60, 0.0),
            (1.60, 3.20),
            (1.40, 3.90),
            (0.0, 3.90),
        ],
        segments=72,
    ).rotate_y(math.pi / 2.0)

    return _merge_geometries(barrel, spinner)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.90, 0.92, 0.94, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.90, 0.92, 0.94, 1.0))
    blade_paint = model.material("blade_paint", rgba=(0.94, 0.95, 0.95, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.29, 0.32, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.42, 0.45, 0.48, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_build_tower_shell(), "tower_shell"),
        material=tower_paint,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.80, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=dark_steel,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=1.55, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 78.05)),
        material=tower_paint,
        name="tower_cap",
    )
    tower.visual(
        Cylinder(radius=1.95, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 78.725)),
        material=bearing_steel,
        name="yaw_bearing",
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.72, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=bearing_steel,
        name="yaw_collar",
    )
    nacelle.visual(
        Box((7.2, 3.0, 0.78)),
        origin=Origin(xyz=(0.85, 0.0, 0.99)),
        material=dark_steel,
        name="bedplate",
    )
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_body(), "nacelle_body"),
        material=nacelle_paint,
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=0.78, length=1.18),
        origin=Origin(
            xyz=(5.81, 0.0, 3.35),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="shaft_housing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=1.78, length=0.38),
        origin=Origin(
            xyz=(0.19, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bearing_steel,
        name="hub_flange",
    )
    rotor.visual(
        mesh_from_geometry(_build_hub_shell(), "hub_shell"),
        material=nacelle_paint,
        name="hub_shell",
    )

    blade_mesh = _build_blade()
    rotor.visual(
        mesh_from_geometry(blade_mesh.copy(), "blade_0"),
        material=blade_paint,
        name="blade_0",
    )
    rotor.visual(
        mesh_from_geometry(blade_mesh.copy().rotate_x(math.tau / 3.0), "blade_1"),
        material=blade_paint,
        name="blade_1",
    )
    rotor.visual(
        mesh_from_geometry(blade_mesh.copy().rotate_x(2.0 * math.tau / 3.0), "blade_2"),
        material=blade_paint,
        name="blade_2",
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 78.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500000.0, velocity=0.20),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(6.40, 0.0, 3.35)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500000.0, velocity=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.expect_contact(
        nacelle,
        tower,
        elem_a="yaw_collar",
        elem_b="yaw_bearing",
        name="nacelle sits on yaw bearing",
    )
    ctx.expect_within(
        nacelle,
        tower,
        axes="xy",
        inner_elem="yaw_collar",
        outer_elem="yaw_bearing",
        margin=0.0,
        name="yaw collar stays centered on bearing ring",
    )
    ctx.expect_contact(
        rotor,
        nacelle,
        elem_a="hub_flange",
        elem_b="shaft_housing",
        name="rotor mounts to nacelle shaft housing",
    )
    ctx.expect_origin_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=6.2,
        max_gap=6.6,
        name="rotor axis stays forward of the yaw axis",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw rotates nacelle around tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and abs(rest_rotor_pos[0] - 6.40) < 0.05
        and abs(rest_rotor_pos[1]) < 0.05
        and abs(yawed_rotor_pos[0]) < 0.05
        and yawed_rotor_pos[1] > 6.30,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    blade_rest_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="blade_0"))
    with ctx.pose({spin: math.pi / 3.0}):
        blade_spun_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="blade_0"))
    ctx.check(
        "rotor spin reorients a blade about the main shaft",
        blade_rest_center is not None
        and blade_spun_center is not None
        and abs(blade_rest_center[0] - blade_spun_center[0]) < 0.75
        and (
            abs(blade_rest_center[1] - blade_spun_center[1]) > 3.0
            or abs(blade_rest_center[2] - blade_spun_center[2]) > 3.0
        ),
        details=f"rest={blade_rest_center}, spun={blade_spun_center}",
    )

    return ctx.report()


object_model = build_object_model()
