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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_member(part, a, b, radius: float, material, *, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_circle_point(x: float, radius: float, angle: float) -> tuple[float, float, float]:
    return (x, radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    warm_white = model.material("warm_white", rgba=(0.92, 0.93, 0.91, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.73, 0.76, 1.0))

    base = model.part("base")
    base_plate = _save_mesh(
        "fan_base_plate",
        ExtrudeGeometry(rounded_rect_profile(0.220, 0.168, 0.045), 0.022, center=True),
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=warm_white,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=charcoal,
        name="base_neck",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.210),
        origin=Origin(xyz=(-0.008, 0.0, 0.134)),
        material=graphite,
        name="support_stem",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(-0.006, 0.0, 0.245)),
        material=charcoal,
        name="tilt_stage",
    )

    for side_sign, arm_name in ((1.0, "left"), (-1.0, "right")):
        arm_geom = tube_from_spline_points(
            [
                (-0.006, 0.0, 0.245),
                (0.024, 0.046 * side_sign, 0.267),
                (0.034, 0.096 * side_sign, 0.291),
                (0.032, 0.126 * side_sign, 0.307),
            ],
            radius=0.0085,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            _save_mesh(f"fan_yoke_arm_{arm_name}", arm_geom),
            material=graphite,
            name=f"{arm_name}_yoke_arm",
        )
        base.visual(
            Box((0.050, 0.006, 0.076)),
            origin=Origin(xyz=(0.018, 0.133 * side_sign, 0.307)),
            material=graphite,
            name=f"{arm_name}_bracket_plate",
        )
        base.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(
                xyz=(0.004, 0.133 * side_sign, 0.307),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name=f"{arm_name}_pivot_boss",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.320)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.007, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_axle",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        head.visual(
            Cylinder(radius=0.011, length=0.028),
            origin=Origin(xyz=(0.0, 0.118 * side_sign, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"{side_name}_trunnion",
        )
        head.visual(
            Cylinder(radius=0.015, length=0.012),
            origin=Origin(xyz=(0.010, 0.095 * side_sign, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"{side_name}_axle_collar",
        )

    head.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="spider_hub",
    )
    head.visual(
        Cylinder(radius=0.029, length=0.032),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="guard_mount",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.094, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="motor_shaft",
    )

    front_ring = _save_mesh(
        "fan_front_guard_ring",
        TorusGeometry(radius=0.138, tube=0.0035, radial_segments=16, tubular_segments=56).rotate_y(
            math.pi / 2.0
        ),
    )
    mid_ring = _save_mesh(
        "fan_mid_guard_ring",
        TorusGeometry(radius=0.130, tube=0.0032, radial_segments=16, tubular_segments=52).rotate_y(
            math.pi / 2.0
        ),
    )
    rear_ring = _save_mesh(
        "fan_rear_guard_ring",
        TorusGeometry(radius=0.120, tube=0.0032, radial_segments=16, tubular_segments=48).rotate_y(
            math.pi / 2.0
        ),
    )
    nose_ring = _save_mesh(
        "fan_nose_ring",
        TorusGeometry(radius=0.030, tube=0.0030, radial_segments=14, tubular_segments=36).rotate_y(
            math.pi / 2.0
        ),
    )

    head.visual(front_ring, origin=Origin(xyz=(0.150, 0.0, 0.0)), material=steel, name="front_guard_ring")
    head.visual(mid_ring, origin=Origin(xyz=(0.114, 0.0, 0.0)), material=steel, name="mid_guard_ring")
    head.visual(rear_ring, origin=Origin(xyz=(0.074, 0.0, 0.0)), material=steel, name="rear_guard_ring")
    head.visual(nose_ring, origin=Origin(xyz=(0.154, 0.0, 0.0)), material=steel, name="nose_ring")

    spoke_angles = [i * (2.0 * math.pi / 10.0) for i in range(10)]
    for index, angle in enumerate(spoke_angles):
        _add_member(
            head,
            _yz_circle_point(0.154, 0.033, angle),
            _yz_circle_point(0.150, 0.138, angle),
            0.0022,
            steel,
            name=f"front_spoke_{index:02d}",
        )

    rib_angles = [i * (2.0 * math.pi / 8.0) for i in range(8)]
    for index, angle in enumerate(rib_angles):
        _add_member(
            head,
            _yz_circle_point(0.074, 0.120, angle),
            _yz_circle_point(0.114, 0.130, angle),
            0.0021,
            steel,
            name=f"rear_rib_{index:02d}",
        )
        _add_member(
            head,
            _yz_circle_point(0.114, 0.130, angle),
            _yz_circle_point(0.150, 0.138, angle),
            0.0021,
            steel,
            name=f"front_rib_{index:02d}",
        )

    strut_angles = (
        math.pi / 4.0,
        3.0 * math.pi / 4.0,
        5.0 * math.pi / 4.0,
        7.0 * math.pi / 4.0,
    )
    for index, angle in enumerate(strut_angles):
        _add_member(
            head,
            _yz_circle_point(0.040, 0.034, angle),
            _yz_circle_point(0.074, 0.120, angle),
            0.0034,
            graphite,
            name=f"rear_strut_{index:02d}",
        )

    head.inertial = Inertial.from_geometry(
        Box((0.320, 0.290, 0.290)),
        mass=1.1,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="spinner",
    )
    blade_radius = 0.052
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        rotor.visual(
            Box((0.013, 0.082, 0.005)),
            origin=Origin(
                xyz=(0.018, blade_radius * math.cos(angle), blade_radius * math.sin(angle)),
                rpy=(angle, 0.30, 0.0),
            ),
            material=warm_white,
            name=f"blade_{index:02d}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((0.060, 0.160, 0.160)),
        mass=0.15,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.004, 0.0, 0.307)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=math.radians(-12.0),
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=35.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.20,
        name="head pivot sits clearly above the base",
    )

    rest_ring = ctx.part_element_world_aabb(head, elem="front_guard_ring")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_ring = ctx.part_element_world_aabb(head, elem="front_guard_ring")
    ctx.check(
        "positive tilt raises the fan head",
        rest_ring is not None
        and tilted_ring is not None
        and tilted_ring[1][2] > rest_ring[1][2] + 0.05,
        details=f"rest={rest_ring}, tilted={tilted_ring}",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        outer_elem="front_guard_ring",
        margin=0.006,
        name="rotor stays inside the front guard footprint",
    )
    ctx.expect_origin_gap(
        rotor,
        head,
        axis="x",
        min_gap=0.09,
        max_gap=0.11,
        name="rotor sits forward of the tilt axle",
    )
    ctx.check(
        "rotor uses a forward spin axis",
        tuple(round(v, 4) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
