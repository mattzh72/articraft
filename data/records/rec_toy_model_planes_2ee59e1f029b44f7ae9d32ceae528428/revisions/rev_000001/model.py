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
    DomeGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="model_propeller_airplane")

    painted_red = model.material("painted_red", rgba=(0.78, 0.15, 0.12, 1.0))
    cream = model.material("cream", rgba=(0.95, 0.93, 0.84, 1.0))
    canopy_glass = model.material("canopy_glass", rgba=(0.50, 0.68, 0.86, 0.45))
    dark_prop = model.material("dark_prop", rgba=(0.16, 0.13, 0.10, 1.0))
    aluminum = model.material("aluminum", rgba=(0.82, 0.84, 0.87, 1.0))
    stand_metal = model.material("stand_metal", rgba=(0.43, 0.45, 0.50, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.08, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def yz_section(
        x_pos: float,
        width: float,
        height: float,
        *,
        z_center: float,
        exponent: float = 2.3,
        segments: int = 28,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_val + z_center)
            for y_val, z_val in superellipse_profile(width, height, exponent=exponent, segments=segments)
        ]

    def airfoil_section(
        span_y: float,
        *,
        leading_x: float,
        trailing_x: float,
        z_center: float,
        thickness: float,
    ) -> list[tuple[float, float, float]]:
        chord = leading_x - trailing_x
        return [
            (leading_x, span_y, z_center),
            (leading_x - 0.10 * chord, span_y, z_center + 0.42 * thickness),
            (leading_x - 0.34 * chord, span_y, z_center + 0.82 * thickness),
            (leading_x - 0.70 * chord, span_y, z_center + 0.48 * thickness),
            (trailing_x, span_y, z_center + 0.05 * thickness),
            (trailing_x, span_y, z_center - 0.03 * thickness),
            (leading_x - 0.68 * chord, span_y, z_center - 0.20 * thickness),
            (leading_x - 0.34 * chord, span_y, z_center - 0.28 * thickness),
            (leading_x - 0.10 * chord, span_y, z_center - 0.12 * thickness),
        ]

    def fin_section(
        span_y: float,
        *,
        thickness_scale: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (-0.214, span_y, 0.040),
            (-0.198, span_y, 0.046),
            (-0.174, span_y, 0.082 * thickness_scale + 0.025),
            (-0.147, span_y, 0.110),
            (-0.132, span_y, 0.104),
            (-0.152, span_y, 0.070),
            (-0.176, span_y, 0.040),
        ]

    airframe = model.part("airframe")

    fuselage_geom = section_loft(
        [
            yz_section(-0.210, 0.010, 0.014, z_center=0.008, exponent=2.1),
            yz_section(-0.165, 0.026, 0.026, z_center=0.010, exponent=2.2),
            yz_section(-0.095, 0.042, 0.044, z_center=0.012, exponent=2.3),
            yz_section(-0.020, 0.058, 0.066, z_center=0.014, exponent=2.4),
            yz_section(0.055, 0.050, 0.058, z_center=0.015, exponent=2.35),
            yz_section(0.138, 0.034, 0.040, z_center=0.015, exponent=2.2),
            yz_section(0.205, 0.018, 0.024, z_center=0.013, exponent=2.0),
        ]
    )
    airframe.visual(save_mesh("airplane_fuselage", fuselage_geom), material=painted_red, name="fuselage_shell")

    wing_half_geom = ExtrudeGeometry(
        [
            (0.040, 0.000),
            (-0.118, 0.000),
            (-0.072, 0.282),
            (0.012, 0.282),
        ],
        0.012,
        center=True,
    )
    airframe.visual(
        save_mesh("airplane_right_wing", wing_half_geom.copy()),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.11, 0.0, 0.0)),
        material=cream,
        name="right_wing",
    )
    airframe.visual(
        save_mesh("airplane_left_wing", wing_half_geom.copy().scale(1.0, -1.0, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(-0.11, 0.0, 0.0)),
        material=cream,
        name="left_wing",
    )

    tail_half_geom = ExtrudeGeometry(
        [
            (-0.146, 0.000),
            (-0.222, 0.000),
            (-0.206, 0.122),
            (-0.164, 0.122),
        ],
        0.008,
        center=True,
    )
    airframe.visual(
        save_mesh("airplane_right_tailplane", tail_half_geom.copy()),
        origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(0.05, 0.0, 0.0)),
        material=cream,
        name="right_tailplane",
    )
    airframe.visual(
        save_mesh("airplane_left_tailplane", tail_half_geom.copy().scale(1.0, -1.0, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(-0.05, 0.0, 0.0)),
        material=cream,
        name="left_tailplane",
    )
    airframe.visual(
        Box((0.066, 0.018, 0.012)),
        origin=Origin(xyz=(-0.184, 0.020, 0.036)),
        material=painted_red,
        name="right_tail_fairing",
    )
    airframe.visual(
        Box((0.066, 0.018, 0.012)),
        origin=Origin(xyz=(-0.184, -0.020, 0.036)),
        material=painted_red,
        name="left_tail_fairing",
    )

    fin_geom = ExtrudeGeometry(
        [
            (-0.214, 0.028),
            (-0.194, 0.038),
            (-0.166, 0.082),
            (-0.146, 0.112),
            (-0.130, 0.106),
            (-0.154, 0.064),
            (-0.178, 0.028),
        ],
        0.010,
        center=True,
    ).rotate_x(math.pi / 2.0)
    airframe.visual(save_mesh("airplane_fin", fin_geom), material=painted_red, name="vertical_fin")
    airframe.visual(
        Box((0.046, 0.010, 0.022)),
        origin=Origin(xyz=(-0.176, 0.0, 0.030)),
        material=painted_red,
        name="fin_root_fairing",
    )

    canopy_geom = DomeGeometry(radius=0.024, radial_segments=28, height_segments=14, closed=True).scale(1.55, 0.95, 0.78)
    airframe.visual(
        save_mesh("airplane_canopy", canopy_geom),
        origin=Origin(xyz=(-0.006, 0.0, 0.023)),
        material=canopy_glass,
        name="canopy",
    )
    airframe.visual(
        Box((0.076, 0.034, 0.014)),
        origin=Origin(xyz=(-0.004, 0.0, 0.018)),
        material=painted_red,
        name="canopy_sill",
    )

    airframe.visual(
        Cylinder(radius=0.0175, length=0.028),
        origin=Origin(xyz=(0.191, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="nose_cowl",
    )
    airframe.visual(
        Box((0.030, 0.020, 0.016)),
        origin=Origin(xyz=(-0.028, 0.0, -0.022)),
        material=stand_metal,
        name="belly_mount",
    )
    airframe.visual(
        Cylinder(radius=0.0045, length=0.090),
        origin=Origin(xyz=(-0.028, 0.0, -0.075)),
        material=stand_metal,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(-0.028, 0.0, -0.125)),
        material=stand_metal,
        name="display_base",
    )

    airframe.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(-0.148, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tire_black,
        name="tail_skid",
    )

    airframe.inertial = Inertial.from_geometry(
        Box((0.430, 0.560, 0.260)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_prop,
        name="propeller_collar",
    )
    propeller.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="spinner_backplate",
    )
    spinner_geom = ConeGeometry(radius=0.023, height=0.050, radial_segments=28, closed=True).rotate_y(math.pi / 2.0)
    propeller.visual(
        save_mesh("airplane_spinner", spinner_geom),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=aluminum,
        name="spinner",
    )
    propeller.visual(
        Box((0.010, 0.250, 0.026)),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, 0.34, 0.0)),
        material=dark_prop,
        name="propeller_blades",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.080, 0.250, 0.040)),
        mass=0.04,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "nose_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.205, 0.0, 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    spin = object_model.get_articulation("nose_propeller_spin")

    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        positive_elem="propeller_collar",
        negative_elem="nose_cowl",
        max_gap=0.001,
        max_penetration=1e-6,
        name="propeller collar seats against the nose cowl",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        elem_a="propeller_collar",
        elem_b="nose_cowl",
        min_overlap=0.030,
        name="propeller collar stays centered on the nose axis",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            positive_elem="propeller_collar",
            negative_elem="nose_cowl",
            max_gap=0.001,
            max_penetration=1e-6,
            name="propeller remains seated when spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
