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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    def _mesh(name: str, geometry):
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

    def _add_member(
        part,
        name: str,
        a: tuple[float, float, float],
        b: tuple[float, float, float],
        radius: float,
        material,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=_distance(a, b)),
            origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    light_steel = model.material("light_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    translucent_blade = model.material("translucent_blade", rgba=(0.80, 0.86, 0.90, 0.68))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.115, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="base_foot",
    )
    support.visual(
        Cylinder(radius=0.102, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_plastic,
        name="base_body",
    )
    support.visual(
        Cylinder(radius=0.074, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=charcoal,
        name="base_neck",
    )
    support_frame = tube_from_spline_points(
        [
            (0.0, 0.0, 0.042),
            (-0.018, 0.0, 0.112),
            (-0.060, 0.0, 0.196),
            (-0.095, 0.0, 0.248),
        ],
        radius=0.013,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    support_frame.merge(
        tube_from_spline_points(
            [
                (-0.095, -0.043, 0.248),
                (-0.095, 0.043, 0.248),
            ],
            radius=0.011,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
        )
    )
    support_frame.merge(
        tube_from_spline_points(
            [
                (-0.095, 0.043, 0.248),
                (-0.078, 0.066, 0.254),
                (-0.054, 0.086, 0.254),
                (-0.032, 0.100, 0.250),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
    )
    support_frame.merge(
        tube_from_spline_points(
            [
                (-0.095, -0.043, 0.248),
                (-0.078, -0.066, 0.254),
                (-0.054, -0.086, 0.254),
                (-0.032, -0.100, 0.250),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
    )
    support.visual(
        _mesh("desk_fan_support_frame", support_frame),
        material=charcoal,
        name="support_frame",
    )
    support.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.020, 0.100, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_pivot_cap",
    )
    support.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.020, -0.100, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_pivot_cap",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.28)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    head = model.part("head")
    motor_shell = LatheGeometry(
        [
            (0.0, -0.050),
            (0.028, -0.046),
            (0.046, -0.028),
            (0.055, 0.006),
            (0.054, 0.052),
            (0.042, 0.088),
            (0.020, 0.116),
            (0.0, 0.126),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)
    head.visual(
        _mesh("desk_fan_motor_shell", motor_shell),
        material=dark_plastic,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, 0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_trunnion",
    )
    head.visual(
        Cylinder(radius=0.031, length=0.028),
        origin=Origin(xyz=(0.119, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="nose_collar",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_guard_mount",
    )
    ring_radius = 0.138
    ring_tube = 0.0055
    rear_ring = TorusGeometry(
        radius=ring_radius,
        tube=ring_tube,
        radial_segments=18,
        tubular_segments=72,
    ).rotate_y(math.pi / 2.0)
    front_ring = TorusGeometry(
        radius=ring_radius,
        tube=ring_tube,
        radial_segments=18,
        tubular_segments=72,
    ).rotate_y(math.pi / 2.0)
    head.visual(
        _mesh("desk_fan_rear_ring", rear_ring),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material=light_steel,
        name="rear_guard_ring",
    )
    head.visual(
        _mesh("desk_fan_front_ring", front_ring),
        origin=Origin(xyz=(0.176, 0.0, 0.0)),
        material=light_steel,
        name="front_guard_ring",
    )
    for side_name, yz in (
        ("top", (0.0, ring_radius)),
        ("bottom", (0.0, -ring_radius)),
        ("left", (ring_radius, 0.0)),
        ("right", (-ring_radius, 0.0)),
    ):
        _add_member(
            head,
            f"guard_spine_{side_name}",
            (0.104, yz[0], yz[1]),
            (0.182, yz[0], yz[1]),
            0.0042,
            light_steel,
        )

    head.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_steel,
        name="rear_guard_hub",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_steel,
        name="front_guard_hub",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        c = math.cos(angle)
        s = math.sin(angle)
        _add_member(
            head,
            f"rear_spoke_{index}",
            (0.110, 0.021 * c, 0.021 * s),
            (0.110, 0.137 * c, 0.137 * s),
            0.0028,
            light_steel,
        )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        c = math.cos(angle)
        s = math.sin(angle)
        _add_member(
            head,
            f"front_spoke_{index}",
            (0.176, 0.024 * c, 0.024 * s),
            (0.176, 0.137 * c, 0.137 * s),
            0.0028,
            light_steel,
        )
    head.inertial = Inertial.from_geometry(
        Box((0.25, 0.29, 0.29)),
        mass=0.9,
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.019, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    rotor.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=steel,
        name="hub_cap",
    )
    blade_size = (0.006, 0.112, 0.050)
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        rotor.visual(
            Box(blade_size),
            origin=Origin(
                xyz=(0.0, 0.058 * math.cos(angle), 0.058 * math.sin(angle)),
                rpy=(angle, 0.22, 0.0),
            ),
            material=translucent_blade,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.034),
        mass=0.18,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "support_to_head",
        ArticulationType.REVOLUTE,
        parent=support,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("support_to_head")

    ctx.expect_contact(
        support,
        head,
        elem_a="left_pivot_cap",
        elem_b="left_trunnion",
        name="left trunnion seats on the left bracket pivot cap",
    )
    ctx.expect_contact(
        support,
        head,
        elem_a="right_pivot_cap",
        elem_b="right_trunnion",
        name="right trunnion seats on the right bracket pivot cap",
    )
    ctx.expect_overlap(
        support,
        head,
        axes="xz",
        elem_a="left_pivot_cap",
        elem_b="left_trunnion",
        min_overlap=0.014,
        name="left bracket laterally captures the left trunnion",
    )
    ctx.expect_overlap(
        support,
        head,
        axes="xz",
        elem_a="right_pivot_cap",
        elem_b="right_trunnion",
        min_overlap=0.014,
        name="right bracket laterally captures the right trunnion",
    )

    with ctx.pose({tilt: 0.0}):
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            outer_elem="front_guard_ring",
            margin=0.002,
            name="rotor fits inside the front guard opening at rest",
        )

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            outer_elem="front_guard_ring",
            margin=0.002,
            name="rotor stays inside the guard opening when tilted up",
        )

    with ctx.pose({tilt: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(head, elem="front_guard_ring")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_aabb = ctx.part_element_world_aabb(head, elem="front_guard_ring")

    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    tilted_center_z = None if tilted_aabb is None else (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5
    ctx.check(
        "positive tilt raises the fan head",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z > rest_center_z + 0.04,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
