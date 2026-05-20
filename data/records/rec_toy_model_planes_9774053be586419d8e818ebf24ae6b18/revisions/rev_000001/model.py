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


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    corner: float,
    *,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z + z_offset)
        for y, z in rounded_rect_profile(width, height, min(corner, width * 0.45, height * 0.45))
    ]


def _airfoil_section(
    y_pos: float,
    chord: float,
    thickness: float,
    *,
    x_offset: float = 0.0,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_t = thickness * 0.5
    trailing = x_offset - 0.55 * chord
    leading = x_offset + 0.45 * chord
    return [
        (trailing, y_pos, z_offset + 0.04 * half_t),
        (x_offset - 0.30 * chord, y_pos, z_offset + 0.82 * half_t),
        (x_offset - 0.02 * chord, y_pos, z_offset + 1.00 * half_t),
        (leading, y_pos, z_offset + 0.12 * half_t),
        (x_offset + 0.10 * chord, y_pos, z_offset - 0.26 * half_t),
        (x_offset - 0.14 * chord, y_pos, z_offset - 0.74 * half_t),
        (x_offset - 0.40 * chord, y_pos, z_offset - 0.46 * half_t),
        (trailing, y_pos, z_offset - 0.08 * half_t),
    ]


def _fin_section(
    y_pos: float,
    chord: float,
    height: float,
    *,
    x_offset: float = 0.0,
    z_base: float = 0.0,
) -> list[tuple[float, float, float]]:
    trailing = x_offset - 0.52 * chord
    leading = x_offset + 0.48 * chord
    return [
        (trailing, y_pos, z_base),
        (x_offset - 0.24 * chord, y_pos, z_base + 0.14 * height),
        (x_offset - 0.12 * chord, y_pos, z_base + 0.62 * height),
        (x_offset + 0.04 * chord, y_pos, z_base + height),
        (leading, y_pos, z_base + 0.66 * height),
        (x_offset + 0.18 * chord, y_pos, z_base + 0.12 * height),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    body_paint = model.material("body_paint", rgba=(0.84, 0.16, 0.14, 1.0))
    trim_paint = model.material("trim_paint", rgba=(0.96, 0.96, 0.94, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.28, 0.42, 0.58, 0.65))
    prop_black = model.material("prop_black", rgba=(0.16, 0.14, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.76, 1.0))

    fuselage = model.part("fuselage")

    fuselage_sections = [
        _yz_section(-0.220, 0.010, 0.014, 0.004),
        _yz_section(-0.160, 0.020, 0.026, 0.007),
        _yz_section(-0.075, 0.034, 0.044, 0.011, z_offset=0.004),
        _yz_section(0.015, 0.058, 0.074, 0.018, z_offset=0.010),
        _yz_section(0.085, 0.054, 0.066, 0.017, z_offset=0.008),
        _yz_section(0.145, 0.038, 0.050, 0.013, z_offset=0.004),
        _yz_section(0.176, 0.018, 0.026, 0.006, z_offset=0.002),
    ]
    fuselage.visual(
        mesh_from_geometry(section_loft(fuselage_sections), "fuselage_shell"),
        material=body_paint,
        name="fuselage_shell",
    )
    fuselage.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.181, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="nose_axle",
    )
    fuselage.visual(
        Cylinder(radius=0.017, length=0.088),
        origin=Origin(xyz=(0.030, 0.0, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=canopy_tint,
        name="canopy",
    )
    fuselage.visual(
        Box((0.090, 0.070, 0.004)),
        origin=Origin(xyz=(0.045, 0.0, -0.004)),
        material=trim_paint,
        name="belly_trim",
    )
    fuselage.visual(
        Box((0.054, 0.032, 0.0042)),
        origin=Origin(xyz=(0.020, 0.0, 0.0499)),
        material=trim_paint,
        name="wing_mount_pad",
    )
    fuselage.visual(
        Box((0.020, 0.018, 0.008)),
        origin=Origin(xyz=(-0.176, 0.0, 0.016)),
        material=body_paint,
        name="tail_mount_pad",
    )
    fuselage.visual(
        Box((0.048, 0.020, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, -0.02549)),
        material=body_paint,
        name="gear_mount_pad",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.400, 0.080, 0.090)),
        mass=0.65,
        origin=Origin(xyz=(-0.010, 0.0, 0.004)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )
    propeller.visual(
        mesh_from_geometry(
            ConeGeometry(radius=0.018, height=0.030).rotate_y(math.pi / 2.0).translate(0.015, 0.0, 0.0),
            "spinner",
        ),
        material=body_paint,
        name="spinner",
    )
    propeller.visual(
        Box((0.014, 0.190, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.30, 0.0, 0.0)),
        material=prop_black,
        name="blade_pair",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.032, 0.190, 0.030)),
        mass=0.06,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.188, 0.0, 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
    )

    wing_assembly = model.part("wing_assembly")
    wing_sections = [
        _airfoil_section(-0.245, 0.078, 0.010, x_offset=0.010, z_offset=0.022),
        _airfoil_section(-0.120, 0.108, 0.013, x_offset=0.006, z_offset=0.010),
        _airfoil_section(0.000, 0.132, 0.016, x_offset=0.000, z_offset=0.000),
        _airfoil_section(0.120, 0.108, 0.013, x_offset=0.006, z_offset=0.010),
        _airfoil_section(0.245, 0.078, 0.010, x_offset=0.010, z_offset=0.022),
    ]
    wing_assembly.visual(
        mesh_from_geometry(section_loft(wing_sections), "wing_assembly"),
        material=trim_paint,
        name="wing_shell",
    )
    wing_assembly.visual(
        Box((0.050, 0.028, 0.006)),
        origin=Origin(xyz=(0.000, 0.0, 0.003)),
        material=trim_paint,
        name="center_mount",
    )
    wing_assembly.inertial = Inertial.from_geometry(
        Box((0.145, 0.500, 0.030)),
        mass=0.22,
        origin=Origin(),
    )

    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing_assembly,
        origin=Origin(xyz=(0.020, 0.0, 0.052)),
    )

    tail_assembly = model.part("tail_assembly")
    tailplane_sections = [
        _airfoil_section(-0.105, 0.040, 0.006, x_offset=0.002, z_offset=0.024),
        _airfoil_section(-0.050, 0.052, 0.008, x_offset=0.001, z_offset=0.020),
        _airfoil_section(0.000, 0.066, 0.009, x_offset=0.000, z_offset=0.018),
        _airfoil_section(0.050, 0.052, 0.008, x_offset=0.001, z_offset=0.020),
        _airfoil_section(0.105, 0.040, 0.006, x_offset=0.002, z_offset=0.024),
    ]
    tail_assembly.visual(
        mesh_from_geometry(section_loft(tailplane_sections), "tailplane"),
        material=trim_paint,
        name="tailplane",
    )
    tail_assembly.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _fin_section(-0.006, 0.070, 0.105, x_offset=0.010, z_base=0.000),
                    _fin_section(0.006, 0.070, 0.105, x_offset=0.010, z_base=0.000),
                ]
            ),
            "vertical_fin",
        ),
        material=body_paint,
        name="vertical_fin",
    )
    tail_assembly.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.007)),
        material=body_paint,
        name="fin_root",
    )
    tail_assembly.inertial = Inertial.from_geometry(
        Box((0.090, 0.220, 0.120)),
        mass=0.08,
        origin=Origin(xyz=(0.004, 0.0, 0.040)),
    )

    model.articulation(
        "fuselage_to_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail_assembly,
        origin=Origin(xyz=(-0.176, 0.0, 0.020)),
    )

    landing_gear = model.part("landing_gear")
    landing_gear.visual(
        Box((0.060, 0.022, 0.012)),
        origin=Origin(),
        material=body_paint,
        name="gear_fairing",
    )
    strut_angle = math.atan2(0.046, -0.038)
    landing_gear.visual(
        Cylinder(radius=0.0045, length=0.060),
        origin=Origin(xyz=(0.0, -0.033, -0.022), rpy=(strut_angle, 0.0, 0.0)),
        material=metal,
        name="left_strut",
    )
    landing_gear.visual(
        Cylinder(radius=0.0045, length=0.060),
        origin=Origin(xyz=(0.0, 0.033, -0.022), rpy=(-strut_angle, 0.0, 0.0)),
        material=metal,
        name="right_strut",
    )
    landing_gear.visual(
        Cylinder(radius=0.004, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, -0.041), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    landing_gear.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.060, -0.041), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=prop_black,
        name="left_wheel",
    )
    landing_gear.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.060, -0.041), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=prop_black,
        name="right_wheel",
    )
    landing_gear.inertial = Inertial.from_geometry(
        Box((0.080, 0.150, 0.090)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )

    model.articulation(
        "fuselage_to_landing_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=landing_gear,
        origin=Origin(xyz=(0.020, 0.0, -0.03349)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    propeller = object_model.get_part("propeller")
    wing_assembly = object_model.get_part("wing_assembly")
    tail_assembly = object_model.get_part("tail_assembly")
    landing_gear = object_model.get_part("landing_gear")
    spin = object_model.get_articulation("fuselage_to_propeller")

    ctx.expect_overlap(
        propeller,
        fuselage,
        axes="yz",
        min_overlap=0.010,
        name="propeller stays centered on the nose axis",
    )
    ctx.expect_gap(
        propeller,
        fuselage,
        axis="x",
        min_gap=0.0,
        max_gap=0.025,
        name="propeller sits just ahead of the nose",
    )
    ctx.expect_contact(
        wing_assembly,
        fuselage,
        elem_a="center_mount",
        elem_b="wing_mount_pad",
        name="wing center section mounts to the cabin roof pad",
    )
    ctx.expect_contact(
        landing_gear,
        fuselage,
        elem_a="gear_fairing",
        elem_b="gear_mount_pad",
        name="landing gear mounts directly under the belly",
    )
    ctx.expect_contact(
        tail_assembly,
        fuselage,
        elem_a="fin_root",
        elem_b="tail_mount_pad",
        name="tail fin root mounts to the fuselage tail pad",
    )

    rest_position = ctx.part_world_position(propeller)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_position = ctx.part_world_position(propeller)
    ctx.check(
        "continuous propeller spin preserves hub location",
        rest_position is not None
        and spun_position is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, spun_position)),
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
