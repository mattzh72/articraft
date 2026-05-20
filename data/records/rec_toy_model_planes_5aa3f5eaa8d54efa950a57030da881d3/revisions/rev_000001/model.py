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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    z_center: float,
    corner_ratio: float = 0.28,
) -> list[tuple[float, float, float]]:
    radius = max(0.001, min(width, height) * corner_ratio)
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _wing_section(
    y: float,
    *,
    leading_x: float,
    trailing_x: float,
    z_center: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    chord = trailing_x - leading_x
    return [
        (leading_x, y, z_center),
        (leading_x + 0.18 * chord, y, z_center + 0.50 * thickness),
        (trailing_x - 0.18 * chord, y, z_center + 0.14 * thickness),
        (trailing_x, y, z_center),
        (trailing_x - 0.12 * chord, y, z_center - 0.10 * thickness),
        (leading_x + 0.22 * chord, y, z_center - 0.28 * thickness),
    ]


def _fin_slice(y: float) -> list[tuple[float, float, float]]:
    return [
        (-0.194, y, 0.050),
        (-0.168, y, 0.110),
        (-0.128, y, 0.160),
        (-0.102, y, 0.151),
        (-0.132, y, 0.074),
        (-0.152, y, 0.050),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    body_red = model.material("body_red", rgba=(0.78, 0.14, 0.16, 1.0))
    wing_cream = model.material("wing_cream", rgba=(0.95, 0.93, 0.86, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.42, 0.58, 0.70, 0.72))
    prop_black = model.material("prop_black", rgba=(0.09, 0.09, 0.10, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.64, 0.65, 0.68, 1.0))

    airframe = model.part("airframe")

    fuselage_geom = section_loft(
        [
            _yz_section(-0.225, width=0.010, height=0.016, z_center=0.010, corner_ratio=0.35),
            _yz_section(-0.170, width=0.036, height=0.045, z_center=0.014),
            _yz_section(-0.060, width=0.062, height=0.078, z_center=0.018),
            _yz_section(0.060, width=0.074, height=0.092, z_center=0.020),
            _yz_section(0.165, width=0.060, height=0.076, z_center=0.017),
            _yz_section(0.218, width=0.018, height=0.024, z_center=0.010, corner_ratio=0.34),
        ]
    )
    airframe.visual(
        _save_mesh("fuselage_shell", fuselage_geom),
        material=body_red,
        name="fuselage_shell",
    )

    wing_geom = section_loft(
        [
            _wing_section(-0.310, leading_x=-0.010, trailing_x=0.070, z_center=0.055, thickness=0.010),
            _wing_section(0.000, leading_x=-0.055, trailing_x=0.108, z_center=0.032, thickness=0.016),
            _wing_section(0.310, leading_x=-0.010, trailing_x=0.070, z_center=0.055, thickness=0.010),
        ]
    )
    airframe.visual(
        _save_mesh("main_wing", wing_geom),
        material=wing_cream,
        name="main_wing",
    )

    tailplane_geom = section_loft(
        [
            _wing_section(-0.140, leading_x=-0.195, trailing_x=-0.108, z_center=0.053, thickness=0.007),
            _wing_section(0.000, leading_x=-0.208, trailing_x=-0.092, z_center=0.050, thickness=0.010),
            _wing_section(0.140, leading_x=-0.195, trailing_x=-0.108, z_center=0.053, thickness=0.007),
        ]
    )
    airframe.visual(
        _save_mesh("tailplane", tailplane_geom),
        material=wing_cream,
        name="tailplane",
    )

    fin_geom = section_loft([_fin_slice(-0.004), _fin_slice(0.004)])
    airframe.visual(
        _save_mesh("vertical_fin", fin_geom),
        material=wing_cream,
        name="vertical_fin",
    )

    canopy_geom = section_loft(
        [
            _yz_section(-0.010, width=0.012, height=0.008, z_center=0.058, corner_ratio=0.40),
            _yz_section(0.028, width=0.044, height=0.032, z_center=0.066, corner_ratio=0.40),
            _yz_section(0.072, width=0.024, height=0.018, z_center=0.060, corner_ratio=0.40),
        ]
    )
    airframe.visual(
        _save_mesh("canopy", canopy_geom),
        material=canopy_tint,
        name="canopy",
    )

    airframe.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(-0.015, 0.0, -0.065)),
        material=metal_gray,
        name="display_peg",
    )
    airframe.visual(
        Box((0.075, 0.020, 0.012)),
        origin=Origin(xyz=(-0.015, 0.0, -0.116)),
        material=metal_gray,
        name="display_foot",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.64, 0.64, 0.23)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    propeller = model.part("propeller")
    spinner_mesh = _save_mesh("spinner", ConeGeometry(radius=0.018, height=0.024).rotate_y(math.pi / 2.0))
    propeller.visual(
        spinner_mesh,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=metal_gray,
        name="spinner",
    )
    propeller.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="hub_barrel",
    )
    propeller.visual(
        Box((0.005, 0.168, 0.028)),
        origin=Origin(xyz=(0.000, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(3.0))),
        material=prop_black,
        name="prop_blade",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.018),
        mass=0.05,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "nose_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.226, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    prop_joint = object_model.get_articulation("nose_propeller")

    ctx.check(
        "propeller uses continuous spin articulation",
        prop_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={prop_joint.articulation_type}",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        elem_a="hub_barrel",
        elem_b="fuselage_shell",
        min_overlap=0.016,
        name="propeller hub stays aligned with the nose profile",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        positive_elem="hub_barrel",
        negative_elem="fuselage_shell",
        min_gap=0.0,
        max_gap=0.012,
        name="propeller hub sits just ahead of the nose",
    )
    with ctx.pose({prop_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            propeller,
            airframe,
            axes="yz",
            elem_a="prop_blade",
            elem_b="fuselage_shell",
            min_overlap=0.014,
            name="rotated propeller still spins about the nose axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
