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
    model = ArticulatedObject(name="small_propeller_model_airplane")

    paint_red = model.material("paint_red", rgba=(0.78, 0.13, 0.12, 1.0))
    cream = model.material("cream", rgba=(0.93, 0.92, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.15, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.32, 0.42, 0.54, 0.92))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.81, 1.0))

    def yz_section(x: float, width: float, height: float, corner: float) -> list[tuple[float, float, float]]:
        profile = rounded_rect_profile(width, height, corner, corner_segments=7)
        return [(x, y, z) for y, z in profile]

    fuselage = model.part("fuselage")
    fuselage_sections = [
        yz_section(-0.225, 0.012, 0.016, 0.003),
        yz_section(-0.190, 0.036, 0.034, 0.007),
        yz_section(-0.115, 0.058, 0.060, 0.012),
        yz_section(-0.020, 0.082, 0.094, 0.020),
        yz_section(0.060, 0.086, 0.106, 0.022),
        yz_section(0.140, 0.067, 0.080, 0.018),
        yz_section(0.195, 0.043, 0.054, 0.012),
        yz_section(0.220, 0.022, 0.030, 0.008),
    ]
    fuselage_mesh = mesh_from_geometry(section_loft(fuselage_sections), "fuselage_shell")
    fuselage.visual(fuselage_mesh, material=paint_red, name="fuselage_shell")
    fuselage.visual(
        Box((0.100, 0.080, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.058)),
        material=cream,
        name="wing_saddle",
    )
    fuselage.visual(
        Box((0.060, 0.040, 0.034)),
        origin=Origin(xyz=(-0.015, 0.0, 0.073)),
        material=canopy_tint,
        name="canopy",
    )
    fuselage.visual(
        Box((0.055, 0.028, 0.018)),
        origin=Origin(xyz=(-0.160, 0.0, 0.032)),
        material=paint_red,
        name="tail_mount",
    )
    fuselage.visual(
        Box((0.052, 0.024, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, -0.051)),
        material=cream,
        name="belly_skid",
    )
    fuselage.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.192, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="engine_cowl",
    )
    fuselage.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.217, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="nose_spindle",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.470, 0.090, 0.130)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    wing = model.part("wing")
    wing.visual(
        Box((0.028, 0.020, 0.030)),
        origin=Origin(xyz=(0.006, 0.030, 0.015)),
        material=paint_red,
        name="left_cabane",
    )
    wing.visual(
        Box((0.028, 0.020, 0.030)),
        origin=Origin(xyz=(0.006, -0.030, 0.015)),
        material=paint_red,
        name="right_cabane",
    )
    wing.visual(
        Box((0.170, 0.220, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, 0.038)),
        material=cream,
        name="center_section",
    )
    wing.visual(
        Box((0.225, 0.220, 0.012)),
        origin=Origin(xyz=(-0.018, 0.170, 0.044), rpy=(0.09, 0.0, 0.09)),
        material=cream,
        name="left_panel",
    )
    wing.visual(
        Box((0.225, 0.220, 0.012)),
        origin=Origin(xyz=(-0.018, -0.170, 0.044), rpy=(-0.09, 0.0, -0.09)),
        material=cream,
        name="right_panel",
    )
    wing.visual(
        Box((0.080, 0.080, 0.014)),
        origin=Origin(xyz=(-0.045, 0.090, 0.040)),
        material=paint_red,
        name="left_root_fairing",
    )
    wing.visual(
        Box((0.080, 0.080, 0.014)),
        origin=Origin(xyz=(-0.045, -0.090, 0.040)),
        material=paint_red,
        name="right_root_fairing",
    )
    wing.inertial = Inertial.from_geometry(
        Box((0.260, 0.640, 0.080)),
        mass=0.25,
        origin=Origin(xyz=(-0.010, 0.0, 0.040)),
    )

    tail_assembly = model.part("tail_assembly")
    tail_assembly.visual(
        Box((0.050, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=paint_red,
        name="tail_root_block",
    )
    tail_assembly.visual(
        Box((0.100, 0.120, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.020)),
        material=cream,
        name="tail_centerplane",
    )
    tail_assembly.visual(
        Box((0.120, 0.120, 0.008)),
        origin=Origin(xyz=(-0.020, 0.110, 0.023), rpy=(0.05, 0.0, 0.05)),
        material=cream,
        name="left_stabilizer",
    )
    tail_assembly.visual(
        Box((0.120, 0.120, 0.008)),
        origin=Origin(xyz=(-0.020, -0.110, 0.023), rpy=(-0.05, 0.0, -0.05)),
        material=cream,
        name="right_stabilizer",
    )
    tail_assembly.visual(
        Box((0.095, 0.014, 0.092)),
        origin=Origin(xyz=(-0.020, 0.0, 0.054), rpy=(0.0, 0.22, 0.0)),
        material=paint_red,
        name="vertical_fin",
    )
    tail_assembly.inertial = Inertial.from_geometry(
        Box((0.150, 0.300, 0.110)),
        mass=0.12,
        origin=Origin(xyz=(-0.020, 0.0, 0.048)),
    )

    display_stand = model.part("display_stand")
    display_stand.visual(
        Cylinder(radius=0.0065, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=aluminum,
        name="stand_post",
    )
    display_stand.visual(
        Box((0.070, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=dark_trim,
        name="stand_foot_x",
    )
    display_stand.visual(
        Box((0.016, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=dark_trim,
        name="stand_foot_y",
    )
    display_stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.110),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hub_barrel",
    )
    propeller.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=paint_red,
        name="spinner",
    )
    propeller.visual(
        Box((0.008, 0.268, 0.024)),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, 0.28, 0.0)),
        material=tire_black,
        name="blade_bar",
    )
    propeller.visual(
        Box((0.010, 0.070, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=dark_trim,
        name="blade_root",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.040),
        mass=0.05,
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing,
        origin=Origin(xyz=(0.018, 0.0, 0.068)),
    )
    model.articulation(
        "fuselage_to_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail_assembly,
        origin=Origin(xyz=(-0.160, 0.0, 0.041)),
    )
    model.articulation(
        "fuselage_to_display_stand",
        ArticulationType.FIXED,
        parent=fuselage,
        child=display_stand,
        origin=Origin(xyz=(0.012, 0.0, -0.060)),
    )
    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.224, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")
    tail_assembly = object_model.get_part("tail_assembly")
    display_stand = object_model.get_part("display_stand")
    propeller = object_model.get_part("propeller")
    propeller_spin = object_model.get_articulation("propeller_spin")

    ctx.expect_contact(wing, fuselage, name="wing is mounted to fuselage")
    ctx.expect_contact(tail_assembly, fuselage, name="tail is mounted to fuselage")
    ctx.expect_contact(display_stand, fuselage, name="display stand supports fuselage belly")
    ctx.expect_gap(
        propeller,
        fuselage,
        axis="x",
        positive_elem="spinner",
        min_gap=0.0005,
        max_gap=0.004,
        name="spinner sits just ahead of the nose",
    )
    ctx.expect_overlap(
        propeller,
        fuselage,
        axes="yz",
        min_overlap=0.028,
        name="propeller stays closely wrapped by the nose profile",
    )

    with ctx.pose({propeller_spin: math.pi / 2.0}):
        ctx.expect_gap(
            propeller,
            fuselage,
            axis="x",
            positive_elem="spinner",
            min_gap=0.0005,
            max_gap=0.004,
            name="spun propeller keeps nose clearance",
        )
        ctx.expect_overlap(
            propeller,
            fuselage,
            axes="yz",
            min_overlap=0.020,
            name="spun propeller remains centered on the nose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
