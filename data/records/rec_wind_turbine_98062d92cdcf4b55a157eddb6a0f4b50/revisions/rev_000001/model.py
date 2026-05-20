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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _circle_section(radius: float, z: float, segments: int = 40) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
            z,
        )
        for i in range(segments)
    ]


def _yz_section(
    x: float,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, corner_radius, corner_segments=10)]


def _blade_section(
    span_y: float,
    chord: float,
    thickness: float,
    twist: float,
    *,
    x_offset: float = 0.0,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    airfoil = [
        (0.55 * chord, 0.00 * thickness),
        (0.32 * chord, 0.16 * thickness),
        (0.05 * chord, 0.28 * thickness),
        (-0.22 * chord, 0.22 * thickness),
        (-0.44 * chord, 0.08 * thickness),
        (-0.50 * chord, 0.00 * thickness),
        (-0.44 * chord, -0.07 * thickness),
        (-0.14 * chord, -0.12 * thickness),
        (0.18 * chord, -0.08 * thickness),
        (0.44 * chord, -0.03 * thickness),
    ]
    c = math.cos(twist)
    s = math.sin(twist)
    return [
        (x_offset + x * c + z * s, span_y, z_offset - x * s + z * c)
        for x, z in airfoil
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_white = model.material("tower_white", rgba=(0.92, 0.94, 0.95, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.90, 0.92, 0.93, 1.0))
    light_gray = model.material("light_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.69, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=9.0, length=1.2),
        origin=Origin(xyz=(0.0, 0.0, -0.6)),
        material=concrete,
        name="foundation_pad",
    )
    tower_shell = section_loft(
        [
            _circle_section(4.3, 0.0, segments=44),
            _circle_section(3.6, 28.0, segments=44),
            _circle_section(2.8, 56.0, segments=44),
            _circle_section(2.0, 78.0, segments=44),
        ]
    )
    tower.visual(mesh_from_geometry(tower_shell, "tower_shell"), material=tower_white, name="tower_shell")
    tower.visual(
        Cylinder(radius=2.2, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 78.5)),
        material=light_gray,
        name="tower_head_flange",
    )
    tower.inertial = Inertial.from_geometry(
        Box((18.0, 18.0, 80.0)),
        mass=175000.0,
        origin=Origin(xyz=(0.0, 0.0, 39.0)),
    )

    nacelle = model.part("nacelle")
    nacelle_shell = section_loft(
        [
            _yz_section(-3.4, 3.2, 3.0, 0.55),
            _yz_section(-1.0, 4.4, 4.3, 0.85),
            _yz_section(3.0, 4.7, 4.6, 0.95),
            _yz_section(7.2, 3.6, 3.7, 0.75),
            _yz_section(9.6, 2.1, 2.2, 0.45),
        ]
    )
    nacelle.visual(
        mesh_from_geometry(nacelle_shell, "nacelle_shell"),
        origin=Origin(xyz=(1.2, 0.0, 2.6)),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=2.2, length=0.9),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=dark_gray,
        name="yaw_bearing_ring",
    )
    nacelle.visual(
        Box((4.6, 3.2, 0.9)),
        origin=Origin(xyz=(1.2, 0.0, 0.95)),
        material=dark_gray,
        name="bedplate",
    )
    nacelle.visual(
        Box((2.8, 2.6, 1.9)),
        origin=Origin(xyz=(4.6, 0.0, 2.2)),
        material=dark_gray,
        name="front_bearing_block",
    )
    nacelle.visual(
        Cylinder(radius=1.18, length=7.0),
        origin=Origin(xyz=(7.5, 0.0, 2.8), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="main_shaft_housing",
    )
    nacelle.visual(
        Box((2.2, 3.4, 2.4)),
        origin=Origin(xyz=(-2.0, 0.0, 2.5)),
        material=dark_gray,
        name="rear_equipment_bay",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((14.0, 6.0, 6.0)),
        mass=68000.0,
        origin=Origin(xyz=(1.8, 0.0, 2.8)),
    )

    model.articulation(
        "nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 79.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280000.0, velocity=0.18),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=1.45, length=5.0),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="hub_barrel",
    )
    rotor.visual(
        mesh_from_geometry(
            ConeGeometry(radius=2.25, height=4.2, radial_segments=44, closed=True).rotate_y(math.pi / 2.0).translate(2.1, 0.0, 0.0),
            "hub_spinner",
        ),
        material=tower_white,
        name="hub_spinner",
    )
    blade_mesh = mesh_from_geometry(
        section_loft(
            [
                _blade_section(0.0, 4.8, 1.05, math.radians(14.0), x_offset=0.1),
                _blade_section(5.5, 4.6, 1.00, math.radians(13.0), x_offset=0.16),
                _blade_section(14.0, 4.0, 0.82, math.radians(9.0), x_offset=0.35),
                _blade_section(28.0, 2.9, 0.50, math.radians(5.0), x_offset=0.85),
                _blade_section(42.0, 1.9, 0.28, math.radians(2.0), x_offset=1.40),
                _blade_section(54.0, 0.82, 0.10, math.radians(-1.5), x_offset=1.95),
            ]
        ),
        "blade_shell",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=tower_white,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((8.0, 112.0, 112.0)),
        mass=52000.0,
        origin=Origin(xyz=(0.8, 0.0, 0.0)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(13.5, 0.0, 2.8)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420000.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("nacelle_yaw")
    spin = object_model.get_articulation("rotor_spin")

    ctx.expect_origin_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=8.0,
        name="rotor hub sits forward of nacelle body",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    tower_aabb = ctx.part_world_aabb(tower)
    if tower_aabb is not None and rotor_aabb is not None:
        tower_height = tower_aabb[1][2] - tower_aabb[0][2]
        ctx.check(
            "lowest blade tip clears the ground",
            rotor_aabb[0][2] > 20.0,
            details=f"rotor_min_z={rotor_aabb[0][2]:.3f}",
        )

    blade_rest = ctx.part_element_world_aabb(rotor, elem="blade_0")
    rotor_pos_rest = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 2.0}):
        blade_quarter_turn = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({yaw: math.pi / 2.0}):
        rotor_pos_yawed = ctx.part_world_position(rotor)

    if blade_rest is not None and blade_quarter_turn is not None:
        rest_center = tuple((a + b) * 0.5 for a, b in zip(blade_rest[0], blade_rest[1]))
        turned_center = tuple((a + b) * 0.5 for a, b in zip(blade_quarter_turn[0], blade_quarter_turn[1]))
        blade_length_est = blade_rest[1][1] - blade_rest[0][1]
        if tower_aabb is not None:
            tower_height = tower_aabb[1][2] - tower_aabb[0][2]
            ctx.check(
                "rotor reads large relative to support",
                2.0 * blade_length_est > tower_height * 1.2,
                details=f"estimated_diameter={2.0 * blade_length_est:.3f}, tower_height={tower_height:.3f}",
            )
        ctx.check(
            "rotor spin reorients a blade about the main shaft",
            abs(turned_center[2] - rest_center[2]) > 20.0 and abs(turned_center[1]) < abs(rest_center[1]) * 0.35,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    ctx.check(
        "nacelle yaw swings the rotor around the tower axis",
        rotor_pos_rest is not None
        and rotor_pos_yawed is not None
        and abs(rotor_pos_yawed[1]) > 9.0
        and abs(rotor_pos_yawed[0]) < abs(rotor_pos_rest[0]) * 0.25,
        details=f"rest={rotor_pos_rest}, yawed={rotor_pos_yawed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
