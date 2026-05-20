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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    galvanized = model.material("galvanized", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    white_paint = model.material("white_paint", rgba=(0.90, 0.92, 0.94, 1.0))
    blade_white = model.material("blade_white", rgba=(0.94, 0.95, 0.96, 1.0))

    support = model.part("support")
    support.visual(
        Box((1.70, 1.40, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=concrete,
        name="foundation_pad",
    )
    support.visual(
        Box((0.86, 0.40, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=dark_steel,
        name="mount_plinth",
    )
    support.visual(
        Cylinder(radius=0.23, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=dark_steel,
        name="yaw_bearing_drum",
    )
    support.visual(
        Cylinder(radius=0.15, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        material=white_paint,
        name="tower_column",
    )
    support.inertial = Inertial.from_geometry(
        Box((1.70, 1.40, 0.98)),
        mass=380.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Box((0.46, 0.28, 0.28)),
        origin=Origin(xyz=(0.12, 0.0, 0.14)),
        material=white_paint,
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=0.07, length=0.12),
        origin=Origin(xyz=(0.35, 0.0, 0.14), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bearing_housing",
    )
    nacelle.visual(
        Box((0.34, 0.04, 0.04)),
        origin=Origin(xyz=(-0.23, 0.0, 0.14)),
        material=galvanized,
        name="tail_boom",
    )
    nacelle.visual(
        Box((0.18, 0.012, 0.24)),
        origin=Origin(xyz=(-0.49, 0.0, 0.22)),
        material=galvanized,
        name="tail_fin",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((0.74, 0.28, 0.36)),
        mass=24.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.04)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="spinner_stub",
    )
    blade_span = 0.54
    blade_chord = 0.11
    blade_thickness = 0.024
    blade_center_radius = 0.35
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Box((blade_chord, blade_thickness, blade_span)),
            origin=Origin(
                xyz=(
                    0.08,
                    -math.sin(angle) * blade_center_radius,
                    math.cos(angle) * blade_center_radius,
                ),
                rpy=(angle, math.radians(9.0), 0.0),
            ),
            material=blade_white,
            name=f"blade_{index + 1}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.68, length=0.18),
        mass=18.0,
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "support_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=1.0),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(0.41, 0.0, 0.14)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("support_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_rotor_spin")

    ctx.expect_gap(
        nacelle,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="nacelle_body",
        negative_elem="tower_column",
        name="nacelle sits directly on tower top",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="hub_shell",
        negative_elem="front_bearing_housing",
        name="rotor hub starts just ahead of nacelle bearing housing",
    )

    with ctx.pose({spin: math.pi}):
        ctx.expect_gap(
            rotor,
            support,
            axis="z",
            min_gap=0.10,
            positive_elem="blade_1",
            negative_elem="foundation_pad",
            name="lowest blade clears the wide support pad",
        )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw rotates rotor around the vertical tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and rest_rotor_pos[0] > 0.40
        and abs(yawed_rotor_pos[0]) < 0.08
        and yawed_rotor_pos[1] > 0.40
        and abs(yawed_rotor_pos[2] - rest_rotor_pos[2]) < 1e-6,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
