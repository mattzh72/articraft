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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    body_blue = model.material("body_blue", rgba=(0.18, 0.34, 0.78, 1.0))
    trim_white = model.material("trim_white", rgba=(0.93, 0.94, 0.96, 1.0))
    canopy_smoke = model.material("canopy_smoke", rgba=(0.22, 0.28, 0.34, 0.72))
    prop_black = model.material("prop_black", rgba=(0.1, 0.1, 0.1, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.34, 0.36, 0.40, 1.0))

    def yz_superellipse_section(
        x_pos: float,
        width: float,
        height: float,
        *,
        exponent: float = 2.4,
        segments: int = 28,
    ) -> list[tuple[float, float, float]]:
        return [(x_pos, y, z) for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)]

    def airfoil_section(
        y_pos: float,
        chord: float,
        thickness: float,
        x_le: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_le, y_pos, 0.0),
            (x_le - 0.10 * chord, y_pos, 0.50 * thickness),
            (x_le - 0.32 * chord, y_pos, 0.64 * thickness),
            (x_le - 0.68 * chord, y_pos, 0.34 * thickness),
            (x_le - 1.00 * chord, y_pos, 0.0),
            (x_le - 0.72 * chord, y_pos, -0.12 * thickness),
            (x_le - 0.30 * chord, y_pos, -0.22 * thickness),
            (x_le - 0.08 * chord, y_pos, -0.08 * thickness),
        ]

    model_airframe = model.part("airframe")

    fuselage_geom = section_loft(
        [
            yz_superellipse_section(-0.180, 0.006, 0.006, exponent=2.0),
            yz_superellipse_section(-0.148, 0.024, 0.024),
            yz_superellipse_section(-0.095, 0.042, 0.042),
            yz_superellipse_section(-0.010, 0.062, 0.074, exponent=2.8),
            yz_superellipse_section(0.070, 0.056, 0.062, exponent=2.6),
            yz_superellipse_section(0.132, 0.041, 0.046, exponent=2.4),
            yz_superellipse_section(0.172, 0.022, 0.022, exponent=2.0),
        ]
    )
    model_airframe.visual(
        mesh_from_geometry(fuselage_geom, "fuselage_shell"),
        material=body_blue,
        name="fuselage_shell",
    )

    wing_geom = section_loft(
        [
            airfoil_section(-0.240, 0.075, 0.004, 0.020),
            airfoil_section(-0.135, 0.108, 0.008, 0.036),
            airfoil_section(0.000, 0.142, 0.011, 0.052),
            airfoil_section(0.135, 0.108, 0.008, 0.036),
            airfoil_section(0.240, 0.075, 0.004, 0.020),
        ]
    )
    model_airframe.visual(
        mesh_from_geometry(wing_geom, "main_wing"),
        origin=Origin(xyz=(-0.008, 0.0, 0.004)),
        material=trim_white,
        name="main_wing",
    )

    tailplane_geom = section_loft(
        [
            airfoil_section(-0.105, 0.050, 0.003, -0.100),
            airfoil_section(0.000, 0.080, 0.005, -0.090),
            airfoil_section(0.105, 0.050, 0.003, -0.100),
        ]
    )
    model_airframe.visual(
        mesh_from_geometry(tailplane_geom, "tailplane"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=trim_white,
        name="tailplane",
    )

    model_airframe.visual(
        Box((0.068, 0.008, 0.070)),
        origin=Origin(xyz=(-0.117, 0.0, 0.055)),
        material=trim_white,
        name="vertical_fin",
    )
    model_airframe.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.030)),
        material=canopy_smoke,
        name="canopy",
    )
    model_airframe.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(-0.026, 0.0, -0.070)),
        material=stand_gray,
        name="display_post",
    )
    model_airframe.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(-0.026, 0.0, -0.113)),
        material=stand_gray,
        name="display_base",
    )
    model_airframe.inertial = Inertial.from_geometry(
        Box((0.38, 0.50, 0.13)),
        mass=0.45,
        origin=Origin(xyz=(-0.010, 0.0, -0.030)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(ConeGeometry(radius=0.011, height=0.022, radial_segments=28), "prop_spinner"),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="spinner",
    )
    propeller.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="hub",
    )
    propeller.visual(
        Box((0.005, 0.086, 0.015)),
        origin=Origin(xyz=(0.007, 0.049, 0.0), rpy=(0.20, 0.0, 0.0)),
        material=prop_black,
        name="blade_right",
    )
    propeller.visual(
        Box((0.005, 0.086, 0.015)),
        origin=Origin(xyz=(0.007, -0.049, 0.0), rpy=(-0.20, 0.0, 0.0)),
        material=prop_black,
        name="blade_left",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.024, 0.196, 0.040)),
        mass=0.03,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    model.articulation(
        "nose_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=model_airframe,
        child=propeller,
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    spin_joint = object_model.get_articulation("nose_propeller_spin")

    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        min_gap=0.0,
        max_gap=0.003,
        name="propeller hub mounts tightly to the fuselage nose",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        min_overlap=0.015,
        name="propeller hub stays centered on the nose axis",
    )

    with ctx.pose({spin_joint: math.pi / 2.0}):
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            min_gap=0.0,
            max_gap=0.003,
            name="rotated propeller remains mounted tightly to the nose in quarter-turn pose",
        )

    blade_box = ctx.part_element_world_aabb(propeller, elem="blade_right")
    if blade_box is None:
        ctx.fail("named propeller blade exists", "Missing blade_right AABB")
    else:
        extents = [blade_box[1][i] - blade_box[0][i] for i in range(3)]
        major = max(extents)
        minor = sorted(extents)[1]
        ctx.check(
            "propeller blade is narrow and long",
            major > 0.07 and extents[0] < 0.01 and minor < 0.04,
            details=f"extents={extents}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
