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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="model_propeller_airplane")

    painted_red = model.material("painted_red", rgba=(0.84, 0.16, 0.14, 1.0))
    cream = model.material("cream", rgba=(0.96, 0.94, 0.84, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.22, 0.36, 0.44, 0.55))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    prop_black = model.material("prop_black", rgba=(0.10, 0.10, 0.11, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))

    def yz_section(
        *,
        x: float,
        width: float,
        height: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)]

    airframe = model.part("airframe")
    fuselage_geom = section_loft(
        [
            yz_section(x=-0.150, width=0.018, height=0.020, radius=0.006),
            yz_section(x=-0.095, width=0.036, height=0.034, radius=0.012),
            yz_section(x=-0.015, width=0.054, height=0.050, radius=0.018),
            yz_section(x=0.065, width=0.050, height=0.046, radius=0.016),
            yz_section(x=0.135, width=0.022, height=0.022, radius=0.008),
        ]
    )
    airframe.visual(
        mesh_from_geometry(fuselage_geom, "airplane_fuselage"),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=painted_red,
        name="fuselage_shell",
    )
    airframe.visual(
        Box((0.110, 0.120, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.104)),
        material=cream,
        name="wing_center",
    )
    airframe.visual(
        Box((0.085, 0.180, 0.010)),
        origin=Origin(xyz=(0.006, 0.138, 0.111), rpy=(0.11, 0.00, -0.03)),
        material=cream,
        name="wing_right",
    )
    airframe.visual(
        Box((0.085, 0.180, 0.010)),
        origin=Origin(xyz=(0.006, -0.138, 0.111), rpy=(-0.11, 0.00, 0.03)),
        material=cream,
        name="wing_left",
    )
    airframe.visual(
        Box((0.048, 0.142, 0.008)),
        origin=Origin(xyz=(-0.118, 0.000, 0.116)),
        material=cream,
        name="tailplane",
    )
    airframe.visual(
        Box((0.060, 0.012, 0.062)),
        origin=Origin(xyz=(-0.132, 0.000, 0.143), rpy=(0.00, -0.10, 0.00)),
        material=painted_red,
        name="vertical_tail",
    )
    airframe.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.010, 0.000, 0.128)),
        material=canopy_tint,
        name="canopy",
    )
    airframe.visual(
        Cylinder(radius=0.005, length=0.084),
        origin=Origin(xyz=(-0.018, 0.000, 0.045)),
        material=dark_gray,
        name="display_post",
    )
    airframe.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(-0.018, 0.000, 0.003)),
        material=dark_gray,
        name="display_base",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.290, 0.460, 0.150)),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.000, 0.095)),
    )

    nose_sleeve = model.part("nose_sleeve")
    nose_sleeve.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.010, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=aluminum,
        name="sleeve_body",
    )
    nose_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.020),
        mass=0.02,
        origin=Origin(xyz=(0.010, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
    )
    model.articulation(
        "airframe_to_nose_sleeve",
        ArticulationType.FIXED,
        parent=airframe,
        child=nose_sleeve,
        origin=Origin(xyz=(0.135, 0.000, 0.100)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.006, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=aluminum,
        name="prop_hub",
    )
    propeller.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.014, 0.000, 0.000)),
        material=aluminum,
        name="spinner_tip",
    )
    propeller.visual(
        Box((0.008, 0.076, 0.020)),
        origin=Origin(xyz=(0.012, 0.036, 0.000), rpy=(0.000, 0.12, 0.000)),
        material=prop_black,
        name="upper_blade",
    )
    propeller.visual(
        Box((0.008, 0.076, 0.020)),
        origin=Origin(xyz=(0.012, -0.036, 0.000), rpy=(0.000, 0.12, 0.000)),
        material=prop_black,
        name="lower_blade",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.032, 0.160, 0.030)),
        mass=0.03,
        origin=Origin(xyz=(0.012, 0.000, 0.000)),
    )
    model.articulation(
        "nose_sleeve_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=nose_sleeve,
        child=propeller,
        origin=Origin(xyz=(0.020, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    nose_sleeve = object_model.get_part("nose_sleeve")
    propeller = object_model.get_part("propeller")
    prop_joint = object_model.get_articulation("nose_sleeve_to_propeller")

    ctx.expect_gap(
        nose_sleeve,
        airframe,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="nose sleeve seats against fuselage nose",
    )
    ctx.expect_overlap(
        nose_sleeve,
        airframe,
        axes="yz",
        min_overlap=0.018,
        name="nose sleeve stays centered on the fuselage nose",
    )
    ctx.expect_gap(
        propeller,
        nose_sleeve,
        axis="x",
        max_penetration=1e-6,
        max_gap=0.001,
        name="propeller hub sits directly ahead of the sleeve support",
    )
    ctx.expect_overlap(
        propeller,
        nose_sleeve,
        axes="yz",
        min_overlap=0.018,
        name="propeller remains coaxial with the sleeve support",
    )

    with ctx.pose({prop_joint: math.pi / 2.0}):
        ctx.expect_gap(
            propeller,
            nose_sleeve,
            axis="x",
            max_penetration=1e-6,
            max_gap=0.001,
            name="spun propeller keeps the same axial seating gap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
