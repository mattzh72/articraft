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
    LatheGeometry,
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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    z_center: float = 0.0,
    y_center: float = 0.0,
    radius_ratio: float = 0.28,
) -> list[tuple[float, float, float]]:
    radius = min(width, height) * radius_ratio
    profile = rounded_rect_profile(height, width, radius, corner_segments=6)
    return [(x, y_center + py, z_center + pz) for pz, py in profile]


def _wing_section(
    *,
    y: float,
    x_le: float,
    chord: float,
    thickness: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_le + 0.00 * chord, y, z_center + 0.00 * thickness),
        (x_le + 0.08 * chord, y, z_center + 0.48 * thickness),
        (x_le + 0.34 * chord, y, z_center + 1.00 * thickness),
        (x_le + 0.76 * chord, y, z_center + 0.46 * thickness),
        (x_le + 1.00 * chord, y, z_center + 0.06 * thickness),
        (x_le + 0.86 * chord, y, z_center - 0.12 * thickness),
        (x_le + 0.42 * chord, y, z_center - 0.36 * thickness),
        (x_le + 0.10 * chord, y, z_center - 0.18 * thickness),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    painted_red = model.material("painted_red", rgba=(0.82, 0.14, 0.16, 1.0))
    cream = model.material("cream", rgba=(0.95, 0.93, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.36, 0.50, 0.60, 0.55))
    walnut = model.material("walnut", rgba=(0.36, 0.24, 0.16, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.18, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=walnut,
        name="base_foot",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=steel,
        name="mounting_flange",
    )
    _add_member(
        stand,
        (0.0, 0.0, 0.022),
        (0.0, 0.0, 0.150),
        0.006,
        steel,
        name="display_peg",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=dark_metal,
        name="peg_cap",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.17)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    fuselage_sections = [
        _yz_section(x=0.000, width=0.012, height=0.012, z_center=0.000, radius_ratio=0.40),
        _yz_section(x=0.024, width=0.050, height=0.056, z_center=0.000),
        _yz_section(x=0.085, width=0.070, height=0.078, z_center=0.004),
        _yz_section(x=0.155, width=0.066, height=0.076, z_center=0.004),
        _yz_section(x=0.225, width=0.056, height=0.064, z_center=0.000),
        _yz_section(x=0.305, width=0.036, height=0.046, z_center=0.008),
        _yz_section(x=0.395, width=0.008, height=0.014, z_center=0.012, radius_ratio=0.35),
    ]
    wing_sections = [
        _wing_section(y=-0.225, x_le=0.118, chord=0.108, thickness=0.010, z_center=0.026),
        _wing_section(y=-0.120, x_le=0.100, chord=0.150, thickness=0.014, z_center=0.018),
        _wing_section(y=0.000, x_le=0.082, chord=0.186, thickness=0.018, z_center=0.012),
        _wing_section(y=0.120, x_le=0.100, chord=0.150, thickness=0.014, z_center=0.018),
        _wing_section(y=0.225, x_le=0.118, chord=0.108, thickness=0.010, z_center=0.026),
    ]
    tailplane_sections = [
        _wing_section(y=-0.110, x_le=0.286, chord=0.058, thickness=0.007, z_center=0.044),
        _wing_section(y=0.000, x_le=0.272, chord=0.088, thickness=0.009, z_center=0.042),
        _wing_section(y=0.110, x_le=0.286, chord=0.058, thickness=0.007, z_center=0.044),
    ]

    airframe = model.part("airframe")
    airframe.visual(
        _save_mesh("airframe_fuselage", section_loft(fuselage_sections)),
        material=painted_red,
        name="fuselage_shell",
    )
    airframe.visual(
        _save_mesh("main_wing", section_loft(wing_sections)),
        material=cream,
        name="main_wing",
    )
    airframe.visual(
        _save_mesh("tailplane", section_loft(tailplane_sections)),
        material=cream,
        name="tailplane",
    )
    airframe.visual(
        Box((0.070, 0.028, 0.028)),
        origin=Origin(xyz=(0.302, 0.0, 0.036)),
        material=painted_red,
        name="tail_fillet",
    )
    airframe.visual(
        Box((0.110, 0.010, 0.090)),
        origin=Origin(xyz=(0.320, 0.0, 0.082), rpy=(0.0, -0.08, 0.0)),
        material=cream,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.090, 0.040, 0.032)),
        origin=Origin(xyz=(0.132, 0.0, 0.042)),
        material=smoked_glass,
        name="canopy",
    )
    airframe.visual(
        Box((0.032, 0.020, 0.016)),
        origin=Origin(xyz=(0.170, 0.0, -0.026)),
        material=dark_metal,
        name="mount_block",
    )
    airframe.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.040, -0.026, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="exhaust_stub",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.40, 0.46, 0.16)),
        mass=0.35,
        origin=Origin(xyz=(0.200, 0.0, 0.030)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        _save_mesh(
            "spinner",
            LatheGeometry(
                [
                    (0.000, -0.030),
                    (0.006, -0.026),
                    (0.012, -0.016),
                    (0.016, -0.004),
                    (0.016, 0.000),
                    (0.000, 0.000),
                ],
                segments=48,
            ),
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spinner",
    )
    propeller.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_collar",
    )
    propeller.visual(
        Box((0.006, 0.086, 0.018)),
        origin=Origin(xyz=(-0.004, 0.042, 0.0), rpy=(0.18, 0.0, 0.10)),
        material=dark_metal,
        name="blade_upper",
    )
    propeller.visual(
        Box((0.006, 0.086, 0.018)),
        origin=Origin(xyz=(-0.004, -0.042, 0.0), rpy=(-0.18, 0.0, 0.10 + math.pi)),
        material=dark_metal,
        name="blade_lower",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.036, 0.176, 0.020)),
        mass=0.05,
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_airframe",
        ArticulationType.FIXED,
        parent=stand,
        child=airframe,
        origin=Origin(xyz=(-0.170, 0.0, 0.200)),
    )
    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    propeller_spin = object_model.get_articulation("propeller_spin")

    ctx.expect_contact(
        airframe,
        stand,
        elem_a="mount_block",
        elem_b="peg_cap",
        contact_tol=0.0005,
        name="display peg meets belly mount",
    )
    ctx.expect_overlap(
        airframe,
        stand,
        axes="xy",
        elem_a="mount_block",
        elem_b="peg_cap",
        min_overlap=0.015,
        name="peg cap sits within belly mount footprint",
    )
    ctx.expect_origin_gap(
        airframe,
        stand,
        axis="z",
        min_gap=0.18,
        max_gap=0.23,
        name="airframe rides above the support base",
    )

    rest_position = ctx.part_world_position(propeller)
    with ctx.pose({propeller_spin: math.pi / 2.0}):
        spun_position = ctx.part_world_position(propeller)
        ctx.expect_overlap(
            propeller,
            airframe,
            axes="yz",
            elem_a="hub_collar",
            elem_b="fuselage_shell",
            min_overlap=0.010,
            name="propeller hub remains centered on the nose axis",
        )
    ctx.check(
        "propeller spins in place",
        rest_position is not None
        and spun_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, spun_position)) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
