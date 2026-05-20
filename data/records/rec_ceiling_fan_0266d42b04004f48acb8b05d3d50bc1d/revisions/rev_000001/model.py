from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _blade_section(
    radius: float,
    *,
    chord: float,
    thickness: float,
    sweep_y: float,
    z_offset: float,
    pitch: float,
) -> list[tuple[float, float, float]]:
    airfoil = [
        (-0.50 * chord, 0.00 * thickness),
        (-0.28 * chord, 0.60 * thickness),
        (0.02 * chord, 0.82 * thickness),
        (0.28 * chord, 0.46 * thickness),
        (0.50 * chord, 0.05 * thickness),
        (0.42 * chord, -0.10 * thickness),
        (0.08 * chord, -0.24 * thickness),
        (-0.24 * chord, -0.16 * thickness),
    ]
    c = math.cos(pitch)
    s = math.sin(pitch)
    return [
        (
            radius,
            y * c - z * s + sweep_y,
            y * s + z * c + z_offset,
        )
        for y, z in airfoil
    ]


def _build_canopy_shell() -> MeshGeometry:
    outer_profile = [
        (0.016, 0.000),
        (0.044, -0.006),
        (0.073, -0.016),
        (0.080, -0.026),
        (0.078, -0.036),
        (0.070, -0.048),
        (0.056, -0.060),
        (0.041, -0.068),
    ]
    inner_profile = [
        (0.013, -0.003),
        (0.030, -0.010),
        (0.052, -0.022),
        (0.059, -0.034),
        (0.056, -0.046),
        (0.048, -0.058),
        (0.036, -0.068),
    ]

    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    shell.merge(
        CylinderGeometry(radius=0.080, height=0.004, radial_segments=56).translate(
            0.0, 0.0, -0.003
        )
    )
    return shell


def _build_motor_shell() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.021, 0.000),
            (0.043, -0.006),
            (0.076, -0.030),
            (0.091, -0.078),
            (0.086, -0.120),
            (0.060, -0.148),
            (0.032, -0.160),
        ],
        [
            (0.014, -0.003),
            (0.033, -0.010),
            (0.063, -0.032),
            (0.077, -0.078),
            (0.072, -0.118),
            (0.049, -0.145),
            (0.026, -0.156),
        ],
        segments=84,
        start_cap="flat",
        end_cap="flat",
    )
    shell.merge(
        CylinderGeometry(radius=0.030, height=0.016, radial_segments=56).translate(
            0.0, 0.0, -0.160
        )
    )
    return shell


def _build_blade_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _blade_section(
                    0.158,
                    chord=0.070,
                    thickness=0.0032,
                    sweep_y=0.000,
                    z_offset=-0.011,
                    pitch=math.radians(14.0),
                ),
                _blade_section(
                    0.285,
                    chord=0.092,
                    thickness=0.0042,
                    sweep_y=-0.008,
                    z_offset=-0.010,
                    pitch=math.radians(11.0),
                ),
                _blade_section(
                    0.412,
                    chord=0.095,
                    thickness=0.0034,
                    sweep_y=-0.021,
                    z_offset=-0.009,
                    pitch=math.radians(8.0),
                ),
                _blade_section(
                    0.535,
                    chord=0.080,
                    thickness=0.0026,
                    sweep_y=-0.037,
                    z_offset=-0.008,
                    pitch=math.radians(6.0),
                ),
                _blade_section(
                    0.622,
                    chord=0.044,
                    thickness=0.0015,
                    sweep_y=-0.052,
                    z_offset=-0.007,
                    pitch=math.radians(4.0),
                ),
            ]
        )
    )


def _plate_section(
    x: float,
    *,
    width: float,
    thickness: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = 0.5 * width
    half_thickness = 0.5 * thickness
    return [
        (x, center_y - half_width, center_z + half_thickness),
        (x, center_y + half_width, center_z + half_thickness),
        (x, center_y + half_width, center_z - half_thickness),
        (x, center_y - half_width, center_z - half_thickness),
    ]


def _build_blade_arm_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _plate_section(0.060, width=0.024, thickness=0.010, center_z=-0.038),
                _plate_section(0.132, width=0.029, thickness=0.009, center_z=-0.039),
                _plate_section(
                    0.204,
                    width=0.040,
                    thickness=0.007,
                    center_y=-0.003,
                    center_z=-0.041,
                ),
            ]
        )
    )


def _build_blade_bracket_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _plate_section(
                    0.146,
                    width=0.042,
                    thickness=0.0045,
                    center_y=-0.001,
                    center_z=-0.040,
                ),
                _plate_section(
                    0.176,
                    width=0.063,
                    thickness=0.0040,
                    center_y=-0.004,
                    center_z=-0.040,
                ),
                _plate_section(
                    0.206,
                    width=0.074,
                    thickness=0.0032,
                    center_y=-0.008,
                    center_z=-0.040,
                ),
            ]
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contemporary_ceiling_fan")

    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.82, 0.83, 0.85, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.28, 0.31, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        _mesh("canopy_shell", _build_canopy_shell()),
        material=brushed_metal,
        name="canopy_shell",
    )
    for suffix, y in (("pos", 0.031), ("neg", -0.031)):
        canopy.visual(
            Box((0.050, 0.006, 0.026)),
            origin=Origin(xyz=(0.0, y, -0.046)),
            material=graphite,
            name=f"socket_cheek_{suffix}",
        )
        canopy.visual(
            Box((0.014, 0.006, 0.028)),
            origin=Origin(xyz=(0.0, y, -0.019)),
            material=graphite,
            name=f"socket_brace_{suffix}",
        )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.110),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    downrod = model.part("downrod")
    downrod.visual(
        Sphere(radius=0.028),
        material=satin_aluminum,
        name="swivel_ball",
    )
    downrod.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=graphite,
        name="ball_stem",
    )
    downrod.visual(
        Cylinder(radius=0.011, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, -0.142)),
        material=brushed_metal,
        name="downrod_tube",
    )
    downrod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.320),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
    )

    motor = model.part("motor_housing")
    motor.visual(
        _mesh("motor_shell", _build_motor_shell()),
        material=brushed_metal,
        name="motor_shell",
    )
    motor.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=graphite,
        name="motor_top_collar",
    )
    motor.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        material=graphite,
        name="motor_lower_stub",
    )
    motor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.190),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    rotor = model.part("rotor")
    blade_mesh = _mesh("curved_blade", _build_blade_mesh().translate(0.0, 0.0, -0.030))
    blade_arm_mesh = _mesh("blade_arm", _build_blade_arm_mesh())
    blade_bracket_mesh = _mesh("blade_bracket", _build_blade_bracket_mesh())
    rotor.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=satin_aluminum,
        name="rotor_spindle",
    )
    rotor.visual(
        Cylinder(radius=0.080, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=graphite,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.056, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=graphite,
        name="hub_cap",
    )
    rotor.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=satin_aluminum,
        name="hub_boss",
    )
    for index, angle in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        rotor.visual(
            blade_arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"blade_arm_{index}",
        )
        rotor.visual(
            blade_bracket_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"blade_bracket_{index}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=satin_aluminum,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((1.280, 1.280, 0.060)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    tilt_angle = math.radians(14.0)
    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.046), rpy=(0.0, -tilt_angle, 0.0)),
    )
    model.articulation(
        "downrod_to_motor",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, -0.262)),
    )
    model.articulation(
        "motor_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor = object_model.get_part("motor_housing")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("motor_to_rotor")

    ctx.check(
        "rotor articulation is continuous about the motor axis",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and rotor_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={rotor_spin.articulation_type}, axis={rotor_spin.axis}",
    )
    ctx.expect_overlap(
        rotor,
        motor,
        axes="xy",
        min_overlap=0.12,
        name="blade assembly stays centered under the motor housing",
    )
    ctx.expect_contact(
        downrod,
        motor,
        elem_a="downrod_tube",
        elem_b="motor_top_collar",
        name="downrod tube seats on the motor collar",
    )
    ctx.expect_contact(
        motor,
        rotor,
        elem_a="motor_lower_stub",
        elem_b="rotor_spindle",
        name="rotor spindle seats on the lower motor stub",
    )
    ctx.expect_overlap(
        downrod,
        canopy,
        axes="xy",
        min_overlap=0.035,
        elem_a="swivel_ball",
        elem_b="canopy_shell",
        name="swivel ball sits within the canopy footprint",
    )
    ctx.expect_contact(
        downrod,
        canopy,
        elem_a="swivel_ball",
        elem_b="socket_cheek_pos",
        name="swivel ball seats in the canopy receiver",
    )

    canopy_pos = ctx.part_world_position(canopy)
    motor_pos = ctx.part_world_position(motor)
    ctx.check(
        "downrod is visibly angled below the canopy",
        canopy_pos is not None
        and motor_pos is not None
        and motor_pos[0] > canopy_pos[0] + 0.05
        and motor_pos[2] < canopy_pos[2] - 0.28,
        details=f"canopy={canopy_pos}, motor={motor_pos}",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    rotor_span_x = None if rotor_aabb is None else rotor_aabb[1][0] - rotor_aabb[0][0]
    rotor_span_y = None if rotor_aabb is None else rotor_aabb[1][1] - rotor_aabb[0][1]
    ctx.check(
        "fan spans a realistic residential diameter",
        rotor_span_x is not None
        and rotor_span_y is not None
        and rotor_span_x > 1.20
        and rotor_span_y > 1.20,
        details=f"span_x={rotor_span_x}, span_y={rotor_span_y}",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({rotor_spin: math.pi / 4.0}):
        turned_rotor_pos = ctx.part_world_position(rotor)
        ctx.expect_contact(
            motor,
            rotor,
            elem_a="motor_lower_stub",
            elem_b="rotor_spindle",
            name="rotor remains seated under the housing when spun 45 degrees",
        )
        ctx.expect_overlap(
            rotor,
            motor,
            axes="xy",
            min_overlap=0.14,
            name="rotor stays centered when spun 45 degrees",
        )

    ctx.check(
        "continuous spin keeps the rotor origin fixed on the motor axis",
        rest_rotor_pos is not None
        and turned_rotor_pos is not None
        and abs(rest_rotor_pos[0] - turned_rotor_pos[0]) < 1e-6
        and abs(rest_rotor_pos[1] - turned_rotor_pos[1]) < 1e-6
        and abs(rest_rotor_pos[2] - turned_rotor_pos[2]) < 1e-6,
        details=f"rest={rest_rotor_pos}, turned={turned_rotor_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
