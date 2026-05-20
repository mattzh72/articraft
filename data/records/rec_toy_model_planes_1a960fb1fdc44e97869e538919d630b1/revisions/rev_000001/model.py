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
    repair_loft,
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


def _rpy_for_cylinder(
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
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(x: float, width: float, height: float, radius: float):
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius)]


def _wing_section(
    *,
    y: float,
    leading_x: float,
    chord: float,
    thickness: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (leading_x, y, z),
        (leading_x + 0.10 * chord, y, z + 0.48 * thickness),
        (leading_x + 0.55 * chord, y, z + 0.28 * thickness),
        (leading_x + chord, y, z + 0.04 * thickness),
        (leading_x + 0.70 * chord, y, z - 0.16 * thickness),
        (leading_x + 0.12 * chord, y, z - 0.32 * thickness),
    ]


def _fin_section(y: float) -> list[tuple[float, float, float]]:
    return [
        (-0.288, y, 0.018),
        (-0.254, y, 0.026),
        (-0.236, y, 0.074),
        (-0.190, y, 0.148),
        (-0.210, y, 0.176),
        (-0.246, y, 0.122),
        (-0.294, y, 0.062),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_propeller_model_airplane")

    cream = model.material("cream", rgba=(0.92, 0.93, 0.88, 1.0))
    red = model.material("red", rgba=(0.73, 0.14, 0.12, 1.0))
    canopy = model.material("canopy", rgba=(0.23, 0.35, 0.43, 0.55))
    metal = model.material("metal", rgba=(0.67, 0.69, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    prop_black = model.material("prop_black", rgba=(0.10, 0.10, 0.11, 1.0))
    prop_tip = model.material("prop_tip", rgba=(0.90, 0.76, 0.14, 1.0))

    airframe = model.part("airframe")

    fuselage_geom = repair_loft(
        section_loft(
        [
            _yz_section(-0.320, 0.020, 0.024, 0.008),
            _yz_section(-0.245, 0.050, 0.062, 0.016),
            _yz_section(-0.105, 0.090, 0.108, 0.026),
            _yz_section(0.050, 0.112, 0.138, 0.032),
            _yz_section(0.205, 0.095, 0.125, 0.028),
            _yz_section(0.340, 0.056, 0.075, 0.020),
            _yz_section(0.392, 0.022, 0.030, 0.009),
        ]
        )
    )
    airframe.visual(
        _save_mesh("fuselage_shell", fuselage_geom),
        material=cream,
        name="fuselage_shell",
    )

    wing_geom = section_loft(
        [
            _wing_section(y=-0.470, leading_x=0.016, chord=0.082, thickness=0.010, z=0.064),
            _wing_section(y=-0.210, leading_x=-0.002, chord=0.142, thickness=0.018, z=0.038),
            _wing_section(y=0.000, leading_x=-0.020, chord=0.176, thickness=0.022, z=0.026),
            _wing_section(y=0.210, leading_x=-0.002, chord=0.142, thickness=0.018, z=0.038),
            _wing_section(y=0.470, leading_x=0.016, chord=0.082, thickness=0.010, z=0.064),
        ]
    )
    airframe.visual(
        _save_mesh("main_wing", wing_geom),
        material=red,
        name="main_wing",
    )

    tailplane_geom = section_loft(
        [
            _wing_section(y=-0.165, leading_x=-0.250, chord=0.058, thickness=0.006, z=0.096),
            _wing_section(y=0.000, leading_x=-0.282, chord=0.102, thickness=0.010, z=0.060),
            _wing_section(y=0.165, leading_x=-0.250, chord=0.058, thickness=0.006, z=0.096),
        ]
    )
    airframe.visual(
        _save_mesh("tailplane", tailplane_geom),
        material=red,
        name="tailplane",
    )

    fin_geom = section_loft([_fin_section(-0.007), _fin_section(0.007)])
    airframe.visual(
        _save_mesh("vertical_fin", fin_geom),
        material=red,
        name="vertical_fin",
    )

    airframe.visual(
        Box((0.146, 0.070, 0.042)),
        origin=Origin(xyz=(0.062, 0.0, 0.100), rpy=(0.0, -0.10, 0.0)),
        material=canopy,
        name="canopy_glass",
    )
    airframe.visual(
        Box((0.115, 0.066, 0.028)),
        origin=Origin(xyz=(0.040, 0.0, 0.075), rpy=(0.0, -0.08, 0.0)),
        material=cream,
        name="canopy_base",
    )
    airframe.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.384, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="nose_mount",
    )

    gear_left_mount = (-0.010, -0.032, -0.020)
    gear_right_mount = (-0.010, 0.032, -0.020)
    left_wheel_center = (0.020, -0.118, -0.092)
    right_wheel_center = (0.020, 0.118, -0.092)

    _add_member(airframe, gear_left_mount, left_wheel_center, 0.006, metal, name="left_gear_strut")
    _add_member(airframe, gear_right_mount, right_wheel_center, 0.006, metal, name="right_gear_strut")
    airframe.visual(
        Cylinder(radius=0.0055, length=0.236),
        origin=Origin(xyz=(0.020, 0.0, -0.092), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="gear_axle",
    )
    airframe.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.020, -0.118, -0.092), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_wheel",
    )
    airframe.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.020, 0.118, -0.092), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_wheel",
    )
    _add_member(
        airframe,
        (-0.282, 0.0, -0.004),
        (-0.264, 0.0, -0.058),
        0.004,
        metal,
        name="tail_skid",
    )

    airframe.inertial = Inertial.from_geometry(
        Box((0.790, 0.960, 0.280)),
        mass=2.4,
        origin=Origin(xyz=(0.035, 0.0, 0.026)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rear_collar",
    )
    propeller.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="hub_barrel",
    )
    propeller.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="front_cap",
    )
    propeller.visual(
        _save_mesh("spinner", ConeGeometry(radius=0.024, height=0.064).rotate_y(math.pi / 2.0)),
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
        material=cream,
        name="spinner",
    )
    propeller.visual(
        Box((0.012, 0.226, 0.032)),
        origin=Origin(xyz=(0.074, 0.120, 0.0), rpy=(0.0, 0.32, 0.0)),
        material=prop_black,
        name="blade_upper",
    )
    propeller.visual(
        Box((0.012, 0.226, 0.032)),
        origin=Origin(xyz=(0.074, -0.120, 0.0), rpy=(0.0, -0.32, 0.0)),
        material=prop_black,
        name="blade_lower",
    )
    propeller.visual(
        Box((0.013, 0.036, 0.034)),
        origin=Origin(xyz=(0.074, 0.223, 0.0), rpy=(0.0, 0.32, 0.0)),
        material=prop_tip,
        name="blade_upper_tip",
    )
    propeller.visual(
        Box((0.013, 0.036, 0.034)),
        origin=Origin(xyz=(0.074, -0.223, 0.0), rpy=(0.0, -0.32, 0.0)),
        material=prop_tip,
        name="blade_lower_tip",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.120),
        mass=0.16,
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.392, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    prop_spin = object_model.get_articulation("propeller_spin")

    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rear_collar",
        negative_elem="nose_mount",
        name="propeller collar seats against nose mount",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        min_overlap=0.025,
        elem_a="rear_collar",
        elem_b="nose_mount",
        name="propeller stays centered on the nose axis",
    )

    rest_pos = ctx.part_world_position(propeller)
    with ctx.pose({prop_spin: 1.8}):
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="rear_collar",
            negative_elem="nose_mount",
            name="spun propeller collar stays seated",
        )
        ctx.expect_overlap(
            propeller,
            airframe,
            axes="yz",
            min_overlap=0.025,
            elem_a="rear_collar",
            elem_b="nose_mount",
            name="spun propeller remains centered on the nose axis",
        )
        spun_pos = ctx.part_world_position(propeller)

    ctx.check(
        "propeller spins in place",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-9,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
