from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _blade_section(
    x_pos: float,
    chord: float,
    thickness: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
    pitch_deg: float = 0.0,
) -> list[tuple[float, float, float]]:
    pitch = math.radians(pitch_deg)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    profile = [
        (-0.50 * chord, 0.00 * thickness),
        (-0.18 * chord, 0.54 * thickness),
        (0.18 * chord, 0.62 * thickness),
        (0.50 * chord, 0.08 * thickness),
        (0.24 * chord, -0.44 * thickness),
        (-0.22 * chord, -0.38 * thickness),
    ]

    section: list[tuple[float, float, float]] = []
    for y_pos, z_pos in profile:
        rotated_y = (y_pos * cos_p) - (z_pos * sin_p)
        rotated_z = (y_pos * sin_p) + (z_pos * cos_p)
        section.append((x_pos, y_center + rotated_y, z_center + rotated_z))
    return section


def _build_blade_mesh():
    return repair_loft(
        section_loft(
            [
                _blade_section(
                    0.176,
                    0.126,
                    0.0105,
                    y_center=-0.001,
                    z_center=-0.027,
                    pitch_deg=15.0,
                ),
                _blade_section(
                    0.300,
                    0.114,
                    0.0090,
                    y_center=0.003,
                    z_center=-0.029,
                    pitch_deg=12.0,
                ),
                _blade_section(
                    0.430,
                    0.100,
                    0.0075,
                    y_center=0.008,
                    z_center=-0.032,
                    pitch_deg=8.0,
                ),
                _blade_section(
                    0.555,
                    0.088,
                    0.0060,
                    y_center=0.010,
                    z_center=-0.035,
                    pitch_deg=4.0,
                ),
                _blade_section(
                    0.624,
                    0.076,
                    0.0045,
                    y_center=0.009,
                    z_center=-0.037,
                    pitch_deg=2.0,
                ),
            ]
        )
    )


def _build_motor_shell_mesh():
    return LatheGeometry(
        [
            (0.0, -0.006),
            (0.088, -0.006),
            (0.118, -0.018),
            (0.150, -0.056),
            (0.160, -0.096),
            (0.148, -0.124),
            (0.118, -0.142),
            (0.072, -0.148),
            (0.0, -0.142),
        ],
        segments=72,
    )


def _iron_section(
    x_pos: float,
    width: float,
    thickness: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, -0.50 * width, z_center - 0.22 * thickness),
        (x_pos, -0.28 * width, z_center + 0.36 * thickness),
        (x_pos, 0.28 * width, z_center + 0.36 * thickness),
        (x_pos, 0.50 * width, z_center - 0.22 * thickness),
        (x_pos, 0.16 * width, z_center - 0.50 * thickness),
        (x_pos, -0.16 * width, z_center - 0.50 * thickness),
    ]


def _build_blade_iron_mesh():
    return repair_loft(
        section_loft(
            [
                _iron_section(0.050, 0.030, 0.010, z_center=-0.018),
                _iron_section(0.114, 0.024, 0.009, z_center=-0.020),
                _iron_section(0.170, 0.036, 0.0085, z_center=-0.023),
                _iron_section(0.208, 0.058, 0.008, z_center=-0.026),
            ]
        )
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][idx] + aabb[1][idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hugger_ceiling_fan")

    ceiling_white = model.material("ceiling_white", rgba=(0.95, 0.95, 0.93, 1.0))
    housing_white = model.material("housing_white", rgba=(0.93, 0.93, 0.91, 1.0))
    bronze = model.material("bronze", rgba=(0.28, 0.24, 0.20, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.20, 0.17, 0.14, 1.0))
    walnut = model.material("walnut", rgba=(0.46, 0.31, 0.20, 1.0))

    motor_shell_mesh = mesh_from_geometry(_build_motor_shell_mesh(), "motor_shell")
    iron_mesh = mesh_from_geometry(_build_blade_iron_mesh(), "blade_iron")
    blade_mesh = mesh_from_geometry(_build_blade_mesh(), "fan_blade")

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Cylinder(radius=0.095, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=ceiling_white,
        name="plate_disk",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.064, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=ceiling_white,
        name="mount_collar",
    )
    ceiling_plate.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.028),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=housing_white,
        name="upper_neck",
    )
    motor_housing.visual(
        motor_shell_mesh,
        material=housing_white,
        name="motor_shell",
    )
    motor_housing.visual(
        Cylinder(radius=0.162, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=housing_white,
        name="vent_band",
    )
    motor_housing.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.143)),
        material=bronze,
        name="lower_cap",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.148),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.073, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark_bronze,
        name="hub_shell",
    )
    blade_assembly.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_bronze,
        name="rotor_spindle",
    )
    blade_assembly.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=bronze,
        name="hub_cap",
    )

    blade_count = 5
    for blade_index in range(blade_count):
        angle = blade_index * math.tau / blade_count
        blade_assembly.visual(
            iron_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"iron_{blade_index}",
        )
        blade_assembly.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=walnut,
            name=f"blade_{blade_index}",
        )

    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.630, length=0.060),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    model.articulation(
        "plate_to_housing",
        ArticulationType.FIXED,
        parent=ceiling_plate,
        child=motor_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
    )
    model.articulation(
        "housing_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.148)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_plate = object_model.get_part("ceiling_plate")
    motor_housing = object_model.get_part("motor_housing")
    blade_assembly = object_model.get_part("blade_assembly")
    spin_joint = object_model.get_articulation("housing_to_blade_assembly")

    ctx.check(
        "spin joint is continuous around the vertical axis",
        spin_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in spin_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin_joint.joint_type}, axis={spin_joint.axis}",
    )

    ctx.expect_contact(
        ceiling_plate,
        motor_housing,
        elem_a="mount_collar",
        elem_b="upper_neck",
        name="motor housing mounts directly to the ceiling plate",
    )

    with ctx.pose({spin_joint: 0.0}):
        ctx.expect_contact(
            motor_housing,
            blade_assembly,
            elem_a="lower_cap",
            elem_b="rotor_spindle",
            name="blade assembly is supported by the motor bearing face",
        )
        ctx.expect_gap(
            motor_housing,
            blade_assembly,
            axis="z",
            positive_elem="lower_cap",
            negative_elem="hub_shell",
            min_gap=0.002,
            max_gap=0.006,
            name="rotating hub hangs just below the housing",
        )
        ctx.expect_within(
            blade_assembly,
            motor_housing,
            axes="xy",
            inner_elem="hub_shell",
            outer_elem="motor_shell",
            margin=0.008,
            name="rotor hub stays centered under the motor shell",
        )

    rest_center = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_0"))
    with ctx.pose({spin_joint: math.pi / 2.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_0"))
        ctx.expect_within(
            blade_assembly,
            motor_housing,
            axes="xy",
            inner_elem="hub_shell",
            outer_elem="motor_shell",
            margin=0.008,
            name="rotor hub remains centered while spinning",
        )

    rest_radius = None if rest_center is None else math.hypot(rest_center[0], rest_center[1])
    turned_radius = (
        None if turned_center is None else math.hypot(turned_center[0], turned_center[1])
    )
    ctx.check(
        "blade assembly visibly rotates",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.18
        and abs(rest_center[1] - turned_center[1]) > 0.18
        and rest_radius is not None
        and turned_radius is not None
        and abs(rest_radius - turned_radius) < 0.02,
        details=(
            f"rest_center={rest_center}, turned_center={turned_center}, "
            f"rest_radius={rest_radius}, turned_radius={turned_radius}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
