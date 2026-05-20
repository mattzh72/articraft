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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _ring_mesh(*, x: float, radius: float, tube_radius: float, samples: int = 28):
    points = []
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        points.append((x, radius * math.cos(angle), radius * math.sin(angle)))
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=6,
        radial_segments=16,
        closed_spline=True,
        cap_ends=False,
        up_hint=(1.0, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    off_white = model.material("off_white", rgba=(0.90, 0.92, 0.91, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    blade_blue = model.material("blade_blue", rgba=(0.66, 0.79, 0.87, 0.62))
    rubber_dark = model.material("rubber_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.22, 0.16, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=off_white,
        name="base_foot",
    )
    base.visual(
        Box((0.060, 0.090, 0.012)),
        origin=Origin(xyz=(-0.010, 0.0, 0.026)),
        material=warm_gray,
        name="base_riser",
    )
    base.visual(
        Box((0.050, 0.040, 0.100)),
        origin=Origin(xyz=(-0.040, 0.0, 0.074)),
        material=off_white,
        name="neck_column",
    )
    base.visual(
        Box((0.072, 0.140, 0.022)),
        origin=Origin(xyz=(-0.006, 0.0, 0.102)),
        material=off_white,
        name="yoke_bridge",
    )
    for side, name in ((0.070, "left_bracket"), (-0.070, "right_bracket")):
        base.visual(
            Box((0.046, 0.010, 0.102)),
            origin=Origin(xyz=(0.008, side, 0.153)),
            material=off_white,
            name=name,
        )
    for foot_x, foot_y, foot_name in (
        (-0.075, -0.050, "rear_right_foot"),
        (-0.075, 0.050, "rear_left_foot"),
        (0.075, -0.050, "front_right_foot"),
        (0.075, 0.050, "front_left_foot"),
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(foot_x, foot_y, 0.002)),
            material=rubber_dark,
            name=foot_name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.19)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.045, length=0.058),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="motor_body",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="nose_cap",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.000, 0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.000, -0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="right_trunnion",
    )
    head.visual(
        mesh_from_geometry(
            _ring_mesh(x=0.020, radius=0.076, tube_radius=0.0035),
            "desk_fan_rear_guard_ring",
        ),
        material=warm_gray,
        name="rear_guard_ring",
    )
    head.visual(
        mesh_from_geometry(
            _ring_mesh(x=0.075, radius=0.082, tube_radius=0.0035),
            "desk_fan_front_guard_ring",
        ),
        material=warm_gray,
        name="front_guard_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0)):
        rear_point = (
            0.019,
            0.074 * math.cos(angle),
            0.074 * math.sin(angle),
        )
        front_point = (
            0.076,
            0.080 * math.cos(angle),
            0.080 * math.sin(angle),
        )
        _add_member(
            head,
            rear_point,
            front_point,
            0.0024,
            warm_gray,
            name=f"cage_rib_{index}",
        )
        _add_member(
            head,
            (0.076, 0.0, 0.0),
            (0.076, 0.078 * math.cos(angle), 0.078 * math.sin(angle)),
            0.0010,
            warm_gray,
            name=f"grille_spoke_{index}",
        )
    for index, angle in enumerate(
        (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)
    ):
        _add_member(
            head,
            (
                0.006,
                0.032 * math.cos(angle),
                0.032 * math.sin(angle),
            ),
            (
                0.021,
                0.072 * math.cos(angle),
                0.072 * math.sin(angle),
            ),
            0.0030,
            warm_gray,
            name=f"guard_mount_{index}",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.17, 0.14, 0.14)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="blade_root_disk",
    )
    rotor.visual(
        Box((0.016, 0.084, 0.004)),
        origin=Origin(xyz=(0.044, 0.032, 0.0), rpy=(0.0, 0.26, 0.0)),
        material=blade_blue,
        name="blade_a",
    )
    rotor.visual(
        Box((0.016, 0.084, 0.004)),
        origin=Origin(xyz=(0.044, -0.016, 0.028), rpy=(2.0 * math.pi / 3.0, 0.26, 0.0)),
        material=blade_blue,
        name="blade_b",
    )
    rotor.visual(
        Box((0.016, 0.084, 0.004)),
        origin=Origin(xyz=(0.044, -0.016, -0.028), rpy=(-2.0 * math.pi / 3.0, 0.26, 0.0)),
        material=blade_blue,
        name="blade_c",
    )
    rotor.inertial = Inertial.from_geometry(
        Box((0.11, 0.18, 0.18)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.010, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=math.radians(-8.0),
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

    ctx.expect_overlap(
        rotor,
        head,
        axes="yz",
        min_overlap=0.110,
        name="rotor stays inside the guarded fan head footprint",
    )
    ctx.expect_gap(
        head,
        base,
        axis="z",
        positive_elem="motor_body",
        negative_elem="yoke_bridge",
        min_gap=0.010,
        name="motor body clears the yoke bridge",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="rotor_hub",
        negative_elem="nose_cap",
        min_gap=0.001,
        max_gap=0.030,
        name="rotor hub clears the stationary nose cap",
    )

    rest_mount = ctx.part_element_world_aabb(head, elem="nose_cap")
    with ctx.pose({tilt: tilt.motion_limits.upper, spin: 1.0}):
        tilted_mount = ctx.part_element_world_aabb(head, elem="nose_cap")
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=0.10,
            name="tilted head remains above the base",
        )

    ctx.check(
        "positive tilt raises the fan head",
        rest_mount is not None
        and tilted_mount is not None
        and tilted_mount[0][2] + tilted_mount[1][2] > rest_mount[0][2] + rest_mount[1][2] + 0.008,
        details=f"rest={rest_mount}, tilted={tilted_mount}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
