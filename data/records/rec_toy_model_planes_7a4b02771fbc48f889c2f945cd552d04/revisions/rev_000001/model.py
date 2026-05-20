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
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        min(corner_radius, width * 0.45, height * 0.45),
        corner_segments=8,
    )
    return [(x_pos, y_val, z_val) for y_val, z_val in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    body_red = model.material("body_red", rgba=(0.80, 0.18, 0.14, 1.0))
    wing_cream = model.material("wing_cream", rgba=(0.95, 0.94, 0.88, 1.0))
    canopy_glass = model.material("canopy_glass", rgba=(0.35, 0.54, 0.66, 0.45))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.18, 0.20, 1.0))
    propeller_black = model.material("propeller_black", rgba=(0.12, 0.12, 0.13, 1.0))
    metal = model.material("metal", rgba=(0.76, 0.78, 0.82, 1.0))

    airframe = model.part("airframe")

    fuselage_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(-0.225, 0.008, 0.010, 0.003),
                _yz_section(-0.190, 0.030, 0.040, 0.010),
                _yz_section(-0.100, 0.052, 0.072, 0.018),
                _yz_section(0.000, 0.080, 0.118, 0.030),
                _yz_section(0.100, 0.068, 0.094, 0.024),
                _yz_section(0.180, 0.040, 0.058, 0.016),
                _yz_section(0.232, 0.010, 0.012, 0.004),
            ]
        ),
        "fuselage_shell",
    )
    airframe.visual(fuselage_mesh, material=body_red, name="fuselage_shell")

    airframe.visual(
        Box((0.110, 0.160, 0.014)),
        origin=Origin(xyz=(0.005, 0.000, 0.012)),
        material=wing_cream,
        name="center_wing",
    )
    airframe.visual(
        Box((0.150, 0.280, 0.012)),
        origin=Origin(xyz=(0.015, 0.218, 0.021), rpy=(math.radians(7.0), 0.0, 0.0)),
        material=wing_cream,
        name="left_wing",
    )
    airframe.visual(
        Box((0.150, 0.280, 0.012)),
        origin=Origin(xyz=(0.015, -0.218, 0.021), rpy=(math.radians(-7.0), 0.0, 0.0)),
        material=wing_cream,
        name="right_wing",
    )

    airframe.visual(
        Box((0.090, 0.210, 0.010)),
        origin=Origin(xyz=(-0.165, 0.000, 0.040)),
        material=wing_cream,
        name="horizontal_tail",
    )
    airframe.visual(
        Box((0.100, 0.012, 0.085)),
        origin=Origin(xyz=(-0.185, 0.000, 0.068)),
        material=body_red,
        name="vertical_tail",
    )

    airframe.visual(
        Box((0.090, 0.055, 0.032)),
        origin=Origin(xyz=(0.020, 0.000, 0.056)),
        material=canopy_glass,
        name="canopy",
    )
    airframe.visual(
        Cylinder(radius=0.008, length=0.120),
        origin=Origin(xyz=(0.012, 0.000, -0.080)),
        material=dark_trim,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.012, 0.000, -0.145)),
        material=dark_trim,
        name="display_base",
    )
    airframe.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.205, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="nose_cowl",
    )
    airframe.visual(
        Cylinder(radius=0.0085, length=0.022),
        origin=Origin(xyz=(0.232, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="nose_bearing",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.620, 0.740, 0.300)),
        mass=1.2,
        origin=Origin(xyz=(0.000, 0.000, -0.015)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.014, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="propeller_hub",
    )
    propeller.visual(
        Box((0.006, 0.018, 0.075)),
        origin=Origin(xyz=(0.008, 0.000, 0.047), rpy=(0.0, 0.0, math.radians(16.0))),
        material=propeller_black,
        name="upper_blade",
    )
    propeller.visual(
        Box((0.006, 0.018, 0.075)),
        origin=Origin(xyz=(0.008, 0.000, -0.047), rpy=(0.0, 0.0, math.radians(16.0))),
        material=propeller_black,
        name="lower_blade",
    )
    propeller.visual(
        Box((0.020, 0.018, 0.018)),
        origin=Origin(xyz=(0.026, 0.000, 0.000)),
        material=metal,
        name="spinner_tip",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.180)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.000, 0.000)),
    )

    model.articulation(
        "nose_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.238, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    spin = object_model.get_articulation("nose_to_propeller")

    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="propeller_hub",
        negative_elem="nose_bearing",
        name="propeller hub seats against the nose bearing",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        min_overlap=0.010,
        elem_a="propeller_hub",
        elem_b="nose_bearing",
        name="propeller is centered on the nose bearing",
    )
    ctx.check(
        "propeller uses continuous nose-axis rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    rest_aabb = ctx.part_world_aabb(propeller)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(propeller)

    rest_y = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else None
    rest_z = rest_aabb[1][2] - rest_aabb[0][2] if rest_aabb is not None else None
    turned_y = turned_aabb[1][1] - turned_aabb[0][1] if turned_aabb is not None else None
    turned_z = turned_aabb[1][2] - turned_aabb[0][2] if turned_aabb is not None else None
    ctx.check(
        "quarter turn changes blade orientation",
        rest_y is not None
        and rest_z is not None
        and turned_y is not None
        and turned_z is not None
        and rest_z > rest_y + 0.050
        and turned_y > turned_z + 0.050,
        details=(
            f"rest_y={rest_y}, rest_z={rest_z}, "
            f"turned_y={turned_y}, turned_z={turned_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
