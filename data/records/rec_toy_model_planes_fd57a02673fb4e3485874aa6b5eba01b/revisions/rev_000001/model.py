from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _superellipse_section(
    x_pos: float,
    width: float,
    height: float,
    *,
    z_offset: float = 0.0,
    exponent: float = 2.5,
    segments: int = 32,
):
    return tuple(
        (x_pos, y_val, z_val + z_offset)
        for y_val, z_val in superellipse_profile(width, height, exponent=exponent, segments=segments)
    )


def _propeller_section(
    z_pos: float,
    thickness_x: float,
    chord_y: float,
    *,
    skew_x: float = 0.0,
    exponent: float = 2.2,
    segments: int = 24,
):
    return tuple(
        (x_val + skew_x, y_val, z_pos)
        for x_val, y_val in superellipse_profile(thickness_x, chord_y, exponent=exponent, segments=segments)
    )


def _wing_section(
    y_pos: float,
    chord: float,
    thickness: float,
    *,
    x_offset: float = 0.0,
    z_offset: float = 0.0,
):
    lead = x_offset - 0.48 * chord
    trail = x_offset + 0.52 * chord
    return (
        (lead, y_pos, z_offset),
        (lead + 0.10 * chord, y_pos, z_offset + 0.48 * thickness),
        (x_offset + 0.04 * chord, y_pos, z_offset + 0.34 * thickness),
        (trail - 0.18 * chord, y_pos, z_offset + 0.11 * thickness),
        (trail, y_pos, z_offset + 0.02 * thickness),
        (trail, y_pos, z_offset - 0.02 * thickness),
        (trail - 0.22 * chord, y_pos, z_offset - 0.08 * thickness),
        (x_offset - 0.02 * chord, y_pos, z_offset - 0.20 * thickness),
        (lead + 0.12 * chord, y_pos, z_offset - 0.30 * thickness),
    )


def _fin_section(
    z_pos: float,
    chord: float,
    thickness: float,
    *,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
):
    lead = x_offset - 0.44 * chord
    trail = x_offset + 0.56 * chord
    return (
        (lead, y_offset, z_pos),
        (lead + 0.12 * chord, y_offset + 0.50 * thickness, z_pos),
        (x_offset + 0.05 * chord, y_offset + 0.34 * thickness, z_pos),
        (trail - 0.16 * chord, y_offset + 0.11 * thickness, z_pos),
        (trail, y_offset + 0.02 * thickness, z_pos),
        (trail, y_offset - 0.02 * thickness, z_pos),
        (trail - 0.20 * chord, y_offset - 0.08 * thickness, z_pos),
        (x_offset + 0.01 * chord, y_offset - 0.18 * thickness, z_pos),
        (lead + 0.12 * chord, y_offset - 0.28 * thickness, z_pos),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    cream = model.material("cream", rgba=(0.94, 0.93, 0.86, 1.0))
    navy = model.material("navy", rgba=(0.16, 0.24, 0.42, 1.0))
    red = model.material("red", rgba=(0.71, 0.16, 0.14, 1.0))
    walnut = model.material("walnut", rgba=(0.47, 0.29, 0.16, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))

    fuselage = model.part("fuselage")
    fuselage.inertial = Inertial.from_geometry(
        Box((0.28, 0.08, 0.09)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    fuselage.visual(
        _save_mesh(
            "fuselage_shell",
            section_loft(
                [
                    _superellipse_section(-0.140, 0.008, 0.012, exponent=2.0),
                    _superellipse_section(-0.112, 0.020, 0.026, z_offset=0.003, exponent=2.1),
                    _superellipse_section(-0.062, 0.040, 0.046, z_offset=0.002, exponent=2.4),
                    _superellipse_section(0.000, 0.056, 0.060, exponent=2.8),
                    _superellipse_section(0.060, 0.052, 0.056, exponent=2.8),
                    _superellipse_section(0.108, 0.036, 0.040, exponent=2.5),
                    _superellipse_section(0.136, 0.014, 0.018, exponent=2.1),
                ]
            ),
        ),
        material=cream,
        name="fuselage_shell",
    )
    fuselage.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(-0.006, 0.0, -0.031)),
        material=navy,
        name="belly_socket",
    )
    fuselage.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=navy,
        name="nose_cowl",
    )
    fuselage.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.143, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="nose_bearing",
    )
    fuselage.visual(
        _save_mesh(
            "main_wing",
            section_loft(
                [
                    _wing_section(-0.170, 0.058, 0.004, x_offset=0.012, z_offset=0.012),
                    _wing_section(-0.090, 0.094, 0.008, x_offset=0.012, z_offset=0.007),
                    _wing_section(0.000, 0.118, 0.012, x_offset=0.012, z_offset=0.003),
                    _wing_section(0.090, 0.094, 0.008, x_offset=0.012, z_offset=0.007),
                    _wing_section(0.170, 0.058, 0.004, x_offset=0.012, z_offset=0.012),
                ]
            ),
        ),
        material=red,
        name="main_wing",
    )
    fuselage.visual(
        _save_mesh(
            "horizontal_tail",
            section_loft(
                [
                    _wing_section(-0.072, 0.040, 0.003, x_offset=-0.103, z_offset=0.027),
                    _wing_section(0.000, 0.056, 0.005, x_offset=-0.103, z_offset=0.013),
                    _wing_section(0.072, 0.040, 0.003, x_offset=-0.103, z_offset=0.027),
                ]
            ),
        ),
        material=red,
        name="horizontal_tail",
    )
    fuselage.visual(
        _save_mesh(
            "vertical_tail",
            section_loft(
                [
                    _fin_section(0.012, 0.050, 0.006, x_offset=-0.106),
                    _fin_section(0.048, 0.034, 0.0045, x_offset=-0.116),
                    _fin_section(0.074, 0.014, 0.0025, x_offset=-0.128),
                ]
            ),
        ),
        material=navy,
        name="vertical_tail",
    )

    propeller = model.part("propeller")
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.018),
        mass=0.04,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    propeller.visual(
        _save_mesh(
            "propeller_blade",
            section_loft(
                [
                    _propeller_section(-0.057, 0.003, 0.012, skew_x=0.0005),
                    _propeller_section(-0.038, 0.004, 0.018, skew_x=0.0010),
                    _propeller_section(-0.014, 0.006, 0.024, skew_x=0.0012),
                    _propeller_section(0.000, 0.010, 0.022),
                    _propeller_section(0.014, 0.006, 0.024, skew_x=-0.0012),
                    _propeller_section(0.038, 0.004, 0.018, skew_x=-0.0010),
                    _propeller_section(0.057, 0.003, 0.012, skew_x=-0.0005),
                ]
            ),
        ),
        material=walnut,
        name="blade",
    )
    propeller.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub",
    )

    display_peg = model.part("display_peg")
    display_peg.inertial = Inertial.from_geometry(
        Box((0.035, 0.035, 0.100)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )
    display_peg.visual(
        Cylinder(radius=0.0045, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_metal,
        name="peg_post",
    )
    display_peg.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_metal,
        name="peg_base",
    )

    model.articulation(
        "nose_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=30.0),
    )
    model.articulation(
        "peg_mount",
        ArticulationType.FIXED,
        parent=fuselage,
        child=display_peg,
        origin=Origin(xyz=(-0.006, 0.0, -0.133)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    propeller = object_model.get_part("propeller")
    display_peg = object_model.get_part("display_peg")
    spin = object_model.get_articulation("nose_propeller_spin")

    ctx.expect_gap(
        propeller,
        fuselage,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="hub",
        negative_elem="nose_bearing",
        name="propeller hub seats against the nose bearing",
    )
    ctx.expect_overlap(
        propeller,
        fuselage,
        axes="yz",
        min_overlap=0.015,
        elem_a="hub",
        elem_b="nose_cowl",
        name="propeller stays centered on the nose axis",
    )
    ctx.expect_gap(
        fuselage,
        display_peg,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="belly_socket",
        negative_elem="peg_post",
        name="display peg seats against the belly socket",
    )

    rest_pos = ctx.part_world_position(propeller)
    with ctx.pose({spin: pi / 2.0}):
        turned_pos = ctx.part_world_position(propeller)
    ctx.check(
        "propeller spins in place",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
