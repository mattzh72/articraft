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
    ExtrudeGeometry,
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
    *,
    center_z: float,
    radius: float | None = None,
) -> list[tuple[float, float, float]]:
    corner = radius
    if corner is None:
        corner = 0.24 * min(width, height)
    corner = min(corner, 0.48 * min(width, height))
    profile = rounded_rect_profile(width, height, corner)
    return [(x_pos, y, z + center_z) for y, z in profile]


def _wing_section(
    y_pos: float,
    chord: float,
    thickness: float,
    *,
    sweep: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    x_le = sweep - 0.25 * chord
    x_te = sweep + 0.75 * chord
    return [
        (x_le, y_pos, z_center),
        (x_le + 0.10 * chord, y_pos, z_center + 0.42 * thickness),
        (x_le + 0.52 * chord, y_pos, z_center + 0.50 * thickness),
        (x_te - 0.06 * chord, y_pos, z_center + 0.08 * thickness),
        (x_te, y_pos, z_center),
        (x_te - 0.06 * chord, y_pos, z_center - 0.06 * thickness),
        (x_le + 0.50 * chord, y_pos, z_center - 0.22 * thickness),
        (x_le + 0.08 * chord, y_pos, z_center - 0.16 * thickness),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="model_airplane")

    paint_red = model.material("paint_red", rgba=(0.78, 0.18, 0.14, 1.0))
    cream = model.material("cream", rgba=(0.94, 0.91, 0.82, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    fuselage = model.part("fuselage")
    fuselage_shell = section_loft(
        [
            _yz_section(0.000, 0.006, 0.006, center_z=0.048, radius=0.0015),
            _yz_section(-0.040, 0.050, 0.056, center_z=0.047),
            _yz_section(-0.120, 0.074, 0.094, center_z=0.052),
            _yz_section(-0.235, 0.070, 0.086, center_z=0.050),
            _yz_section(-0.345, 0.038, 0.046, center_z=0.044),
            _yz_section(-0.440, 0.012, 0.016, center_z=0.041, radius=0.003),
        ]
    )
    fuselage.visual(
        mesh_from_geometry(fuselage_shell, "fuselage_shell"),
        material=paint_red,
        name="fuselage_shell",
    )
    fuselage.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _wing_section(-0.260, 0.110, 0.008, sweep=-0.018, z_center=0.030),
                    _wing_section(-0.090, 0.150, 0.010, sweep=-0.008, z_center=0.008),
                    _wing_section(0.000, 0.180, 0.012, sweep=0.000, z_center=0.000),
                    _wing_section(0.090, 0.150, 0.010, sweep=-0.008, z_center=0.008),
                    _wing_section(0.260, 0.110, 0.008, sweep=-0.018, z_center=0.030),
                ]
            ),
            "wing_panel",
        ),
        origin=Origin(xyz=(-0.170, 0.0, 0.088)),
        material=cream,
        name="wing_panel",
    )
    fuselage.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _wing_section(-0.100, 0.055, 0.006, sweep=-0.010, z_center=0.010),
                    _wing_section(-0.040, 0.078, 0.007, sweep=-0.004, z_center=0.003),
                    _wing_section(0.000, 0.090, 0.008, sweep=0.000, z_center=0.000),
                    _wing_section(0.040, 0.078, 0.007, sweep=-0.004, z_center=0.003),
                    _wing_section(0.100, 0.055, 0.006, sweep=-0.010, z_center=0.010),
                ]
            ),
            "tailplane_panel",
        ),
        origin=Origin(xyz=(-0.360, 0.0, 0.070)),
        material=cream,
        name="tailplane_panel",
    )
    fin_profile = [
        (-0.050, 0.000),
        (0.004, 0.000),
        (0.000, 0.018),
        (-0.012, 0.048),
        (-0.034, 0.106),
        (-0.058, 0.080),
    ]
    fin_geom = ExtrudeGeometry(fin_profile, 0.010).rotate_x(math.pi / 2.0)
    fuselage.visual(
        mesh_from_geometry(fin_geom, "vertical_tail"),
        origin=Origin(xyz=(-0.366, 0.0, 0.075)),
        material=cream,
        name="fin_surface",
    )
    fuselage.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(-0.374, 0.0, 0.081)),
        material=cream,
        name="fin_root_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(-0.185, 0.0, -0.026)),
        material=aluminum,
        name="peg_post",
    )
    fuselage.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(-0.185, 0.0, -0.064)),
        material=black,
        name="peg_foot",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.445, 0.090, 0.110)),
        mass=0.8,
        origin=Origin(xyz=(-0.220, 0.0, 0.050)),
    )

    propeller = model.part("propeller")
    spinner = ConeGeometry(radius=0.018, height=0.038).rotate_y(math.pi / 2.0).translate(
        0.019,
        0.0,
        0.0,
    )
    propeller.visual(
        mesh_from_geometry(spinner, "spinner"),
        material=paint_red,
        name="spinner",
    )
    propeller.visual(
        Box((0.006, 0.180, 0.024)),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(math.radians(9.0), 0.0, 0.0)),
        material=black,
        name="blade_bar",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.010),
        mass=0.08,
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    propeller = object_model.get_part("propeller")
    prop_joint = object_model.get_articulation("fuselage_to_propeller")

    ctx.expect_gap(
        propeller,
        fuselage,
        axis="x",
        positive_elem="spinner",
        negative_elem="fuselage_shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="spinner seats at the nose",
    )
    limits = prop_joint.motion_limits
    ctx.check(
        "propeller joint stays continuous around fuselage axis",
        prop_joint.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"axis={prop_joint.axis}, limits={limits}",
    )
    ctx.expect_gap(
        fuselage,
        fuselage,
        axis="z",
        positive_elem="wing_panel",
        negative_elem="fuselage_shell",
        max_gap=0.03,
        max_penetration=0.02,
        name="wing roots blend into the fuselage body",
    )
    ctx.expect_gap(
        fuselage,
        fuselage,
        axis="z",
        positive_elem="tailplane_panel",
        negative_elem="fuselage_shell",
        max_gap=0.03,
        max_penetration=0.04,
        name="tailplane blends into the aft fuselage",
    )
    ctx.expect_gap(
        fuselage,
        fuselage,
        axis="z",
        positive_elem="fuselage_shell",
        negative_elem="peg_post",
        max_gap=0.002,
        max_penetration=0.008,
        name="display peg reaches the belly without reading as detached",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
