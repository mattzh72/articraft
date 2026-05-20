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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.86, 0.87, 0.88, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.92, 0.93, 0.94, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.34, 0.37, 0.40, 1.0))
    blade_paint = model.material("blade_paint", rgba=(0.95, 0.95, 0.96, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=2.2, length=78.0),
        origin=Origin(xyz=(0.0, 0.0, 39.0)),
        material=tower_paint,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.6, length=1.2),
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        material=steel_dark,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=1.8, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 77.5)),
        material=steel_dark,
        name="tower_collar",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.2, length=78.0),
        mass=180000.0,
        origin=Origin(xyz=(0.0, 0.0, 39.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Box((8.0, 3.2, 3.2)),
        origin=Origin(xyz=(1.6, 0.0, 2.2)),
        material=nacelle_paint,
        name="housing",
    )
    nacelle.visual(
        Cylinder(radius=1.6, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        material=steel_dark,
        name="yaw_deck",
    )
    nacelle.visual(
        Cylinder(radius=0.55, length=0.9),
        origin=Origin(xyz=(5.55, 0.0, 2.2), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="shaft_sleeve",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((8.0, 3.2, 3.2)),
        mass=52000.0,
        origin=Origin(xyz=(1.6, 0.0, 2.2)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=1.35, length=1.8),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub",
    )

    def blade_section(
        chord: float,
        thickness: float,
        z_pos: float,
        twist_deg: float,
    ) -> list[tuple[float, float, float]]:
        half_chord = chord / 2.0
        half_thickness = thickness / 2.0
        twist = math.radians(twist_deg)
        cos_t = math.cos(twist)
        sin_t = math.sin(twist)
        section_xy = [
            (0.0, -0.52 * half_chord * 2.0),
            (0.92 * half_thickness, -0.18 * half_chord * 2.0),
            (half_thickness, 0.18 * half_chord * 2.0),
            (0.0, 0.50 * half_chord * 2.0),
            (-half_thickness, 0.18 * half_chord * 2.0),
            (-0.92 * half_thickness, -0.18 * half_chord * 2.0),
        ]
        return [
            (
                x_val * cos_t - y_val * sin_t,
                x_val * sin_t + y_val * cos_t,
                z_pos,
            )
            for x_val, y_val in section_xy
        ]

    blade_geom = section_loft(
        [
            blade_section(chord=1.8, thickness=0.28, z_pos=0.8, twist_deg=22.0),
            blade_section(chord=1.45, thickness=0.22, z_pos=4.5, twist_deg=14.0),
            blade_section(chord=0.95, thickness=0.14, z_pos=11.5, twist_deg=8.0),
            blade_section(chord=0.52, thickness=0.08, z_pos=21.8, twist_deg=4.0),
        ]
    )
    blade_mesh = mesh_from_geometry(blade_geom, "turbine_blade")
    rotor.visual(
        blade_mesh,
        origin=Origin(),
        material=blade_paint,
        name="blade_a",
    )
    rotor.visual(
        blade_mesh,
        origin=Origin(
            rpy=(2.0 * math.pi / 3.0, 0.0, 0.0),
        ),
        material=blade_paint,
        name="blade_b",
    )
    rotor.visual(
        blade_mesh,
        origin=Origin(
            rpy=(4.0 * math.pi / 3.0, 0.0, 0.0),
        ),
        material=blade_paint,
        name="blade_c",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.35, length=1.8),
        mass=18000.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 78.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500000.0, velocity=0.3),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(6.9, 0.0, 2.2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800000.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_rotor_spin")

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="yaw_deck",
        negative_elem="tower_collar",
        max_gap=0.02,
        max_penetration=0.0,
        name="nacelle sits on tower collar",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="yz",
        elem_a="hub",
        elem_b="shaft_sleeve",
        min_overlap=0.6,
        name="hub stays aligned with nacelle shaft sleeve",
    )

    rest_pos = ctx.part_world_position(rotor)
    blade_rest = ctx.part_element_world_aabb(rotor, elem="blade_a")
    with ctx.pose({yaw: math.pi / 2.0}):
        yaw_pos = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 2.0}):
        blade_spun = ctx.part_element_world_aabb(rotor, elem="blade_a")

    ctx.check(
        "yaw moves rotor around tower axis",
        rest_pos is not None
        and yaw_pos is not None
        and abs(rest_pos[0]) > 5.0
        and abs(yaw_pos[1]) > 5.0,
        details=f"rest={rest_pos}, yawed={yaw_pos}",
    )
    ctx.check(
        "rotor spin changes blade_a orientation",
        blade_rest is not None
        and blade_spun is not None
        and abs(blade_rest[1][2] - blade_rest[0][2]) > 15.0
        and abs(blade_spun[1][1] - blade_spun[0][1]) > 15.0,
        details=f"rest={blade_rest}, spun={blade_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
