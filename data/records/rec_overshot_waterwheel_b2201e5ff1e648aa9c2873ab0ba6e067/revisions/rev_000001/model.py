from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_points(
    *,
    radius: float,
    y: float,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * cos(2.0 * pi * idx / segments),
            y,
            radius * sin(2.0 * pi * idx / segments),
        )
        for idx in range(segments)
    ]


def _add_wheel_geometry(part, *, iron) -> None:
    ring_radius = 1.42
    ring_tube_radius = 0.055
    side_ring_y = 0.31
    bucket_radius = 1.30
    bucket_count = 16
    bucket_width = 0.58
    bucket_tangent = 0.050
    bucket_radial = 0.32
    spoke_count = 12
    hub_radius = 0.17
    hub_half_length = 0.23

    left_ring = tube_from_spline_points(
        _circle_points(radius=ring_radius, y=side_ring_y, segments=28),
        radius=ring_tube_radius,
        samples_per_segment=6,
        radial_segments=18,
        closed_spline=True,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )
    right_ring = tube_from_spline_points(
        _circle_points(radius=ring_radius, y=-side_ring_y, segments=28),
        radius=ring_tube_radius,
        samples_per_segment=6,
        radial_segments=18,
        closed_spline=True,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )
    part.visual(mesh_from_geometry(left_ring, "wheel_left_ring"), material=iron, name="left_ring")
    part.visual(mesh_from_geometry(right_ring, "wheel_right_ring"), material=iron, name="right_ring")

    part.visual(
        Cylinder(radius=0.072, length=0.78),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=0.50),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub",
    )

    for side_y in (-hub_half_length, hub_half_length):
        part.visual(
            Cylinder(radius=0.21, length=0.04),
            origin=Origin(xyz=(0.0, side_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=iron,
        )

    for spoke_index in range(spoke_count):
        theta = 2.0 * pi * spoke_index / spoke_count
        inner_left = (0.20 * cos(theta), side_ring_y * 0.78, 0.20 * sin(theta))
        outer_left = (
            (ring_radius - ring_tube_radius * 0.55) * cos(theta),
            side_ring_y,
            (ring_radius - ring_tube_radius * 0.55) * sin(theta),
        )
        inner_right = (0.20 * cos(theta), -side_ring_y * 0.78, 0.20 * sin(theta))
        outer_right = (
            (ring_radius - ring_tube_radius * 0.55) * cos(theta),
            -side_ring_y,
            (ring_radius - ring_tube_radius * 0.55) * sin(theta),
        )
        _add_member(part, inner_left, outer_left, radius=0.020, material=iron)
        _add_member(part, inner_right, outer_right, radius=0.020, material=iron)

    for bucket_index in range(bucket_count):
        theta = 2.0 * pi * bucket_index / bucket_count
        bucket_x = bucket_radius * cos(theta)
        bucket_z = bucket_radius * sin(theta)
        radial_angle = pi / 2.0 - theta
        part.visual(
            Box((bucket_tangent, bucket_width, bucket_radial)),
            origin=Origin(xyz=(bucket_x, 0.0, bucket_z), rpy=(0.0, radial_angle, 0.0)),
            material=iron,
            name=f"bucket_{bucket_index}",
        )
        lip_radius = bucket_radius + bucket_radial * 0.34
        lip_x = lip_radius * cos(theta)
        lip_z = lip_radius * sin(theta)
        part.visual(
            Box((0.030, bucket_width * 0.96, 0.085)),
            origin=Origin(
                xyz=(lip_x, 0.0, lip_z),
                rpy=(0.0, radial_angle + 0.18, 0.0),
            ),
            material=iron,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_water_wheel")

    stone = model.material("stone", rgba=(0.63, 0.61, 0.57, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.49, 0.48, 0.45, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.20, 0.21, 0.22, 1.0))
    gate_iron = model.material("gate_iron", rgba=(0.26, 0.24, 0.21, 1.0))

    support = model.part("support")
    support.visual(
        Box((4.40, 2.60, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=stone_dark,
        name="foundation",
    )
    support.visual(
        Box((1.80, 0.62, 0.16)),
        origin=Origin(xyz=(-0.15, 0.77, 0.50)),
        material=stone,
        name="left_plinth",
    )
    support.visual(
        Box((1.80, 0.62, 0.16)),
        origin=Origin(xyz=(-0.15, -0.77, 0.50)),
        material=stone,
        name="right_plinth",
    )
    support.visual(
        Box((1.90, 0.62, 3.55)),
        origin=Origin(xyz=(0.0, 0.77, 2.195)),
        material=stone,
        name="left_pier",
    )
    support.visual(
        Box((1.90, 0.62, 3.55)),
        origin=Origin(xyz=(0.0, -0.77, 2.195)),
        material=stone,
        name="right_pier",
    )
    support.visual(
        Box((2.20, 0.78, 0.22)),
        origin=Origin(xyz=(-0.20, 0.77, 4.02)),
        material=stone_dark,
        name="left_cap",
    )
    support.visual(
        Box((2.20, 0.78, 0.22)),
        origin=Origin(xyz=(-0.20, -0.77, 4.02)),
        material=stone_dark,
        name="right_cap",
    )
    support.visual(
        Box((0.28, 0.07, 0.24)),
        origin=Origin(xyz=(0.0, 0.425, 1.96)),
        material=stone_dark,
        name="left_bearing",
    )
    support.visual(
        Box((0.28, 0.07, 0.24)),
        origin=Origin(xyz=(0.0, -0.425, 1.96)),
        material=stone_dark,
        name="right_bearing",
    )
    support.visual(
        Box((1.00, 0.82, 0.14)),
        origin=Origin(xyz=(-1.115, 0.0, 3.59)),
        material=stone_dark,
        name="flume_floor",
    )
    support.visual(
        Box((0.28, 0.82, 0.14)),
        origin=Origin(xyz=(-0.14, 0.0, 3.59)),
        material=stone_dark,
        name="spout_floor",
    )
    support.visual(
        Box((1.52, 0.10, 0.44)),
        origin=Origin(xyz=(-0.96, 0.46, 3.88)),
        material=stone,
        name="left_flume_wall",
    )
    support.visual(
        Box((1.52, 0.10, 0.44)),
        origin=Origin(xyz=(-0.96, -0.46, 3.88)),
        material=stone,
        name="right_flume_wall",
    )
    support.visual(
        Box((0.28, 0.10, 0.32)),
        origin=Origin(xyz=(-0.14, 0.46, 3.82)),
        material=stone,
        name="left_spout_cheek",
    )
    support.visual(
        Box((0.28, 0.10, 0.32)),
        origin=Origin(xyz=(-0.14, -0.46, 3.82)),
        material=stone,
        name="right_spout_cheek",
    )
    support.visual(
        Box((0.12, 0.82, 0.30)),
        origin=Origin(xyz=(-1.66, 0.0, 3.82)),
        material=stone,
        name="flume_backwall",
    )
    support.visual(
        Box((0.18, 0.82, 0.06)),
        origin=Origin(xyz=(-0.56, 0.0, 3.49)),
        material=stone_dark,
        name="gate_sill",
    )
    support.visual(
        Box((0.12, 0.08, 1.18)),
        origin=Origin(xyz=(-0.58, 0.39, 4.08)),
        material=stone_dark,
        name="left_gate_guide",
    )
    support.visual(
        Box((0.12, 0.08, 1.18)),
        origin=Origin(xyz=(-0.58, -0.39, 4.08)),
        material=stone_dark,
        name="right_gate_guide",
    )
    support.visual(
        Box((0.30, 0.82, 0.18)),
        origin=Origin(xyz=(-0.58, 0.0, 4.67)),
        material=stone_dark,
        name="gate_head",
    )
    support.inertial = Inertial.from_geometry(
        Box((4.40, 2.60, 4.20)),
        mass=6800.0,
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
    )

    wheel = model.part("wheel")
    _add_wheel_geometry(wheel, iron=cast_iron)
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.50, length=0.62),
        mass=420.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    gate = model.part("gate_panel")
    gate.visual(
        Box((0.045, 0.66, 0.86)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gate_iron,
        name="gate_panel",
    )
    gate.visual(
        Box((0.075, 0.62, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=gate_iron,
        name="gate_headbar",
    )
    gate.visual(
        Box((0.030, 0.58, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=cast_iron,
        name="gate_stiffener",
    )
    gate.visual(
        Cylinder(radius=0.016, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.52), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="gate_handle",
    )
    gate.inertial = Inertial.from_geometry(
        Box((0.08, 0.72, 1.02)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.96)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=support,
        child=gate,
        origin=Origin(xyz=(-0.58, 0.0, 3.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.18, lower=0.0, upper=0.42),
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

    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("gate_panel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check("support exists", support is not None)
    ctx.check("wheel exists", wheel is not None)
    ctx.check("gate exists", gate is not None)
    ctx.check(
        "wheel uses continuous rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "gate uses vertical prismatic travel",
        gate_slide.articulation_type == ArticulationType.PRISMATIC and gate_slide.axis == (0.0, 0.0, 1.0),
        details=f"type={gate_slide.articulation_type}, axis={gate_slide.axis}",
    )

    ctx.expect_gap(
        support,
        wheel,
        axis="y",
        positive_elem="left_pier",
        min_gap=0.06,
        max_gap=0.35,
        name="wheel clears left masonry pier",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="y",
        positive_elem="axle",
        negative_elem="right_pier",
        min_gap=0.06,
        max_gap=0.30,
        name="axle clears right masonry pier",
    )
    ctx.expect_gap(
        support,
        wheel,
        axis="z",
        positive_elem="spout_floor",
        min_gap=0.04,
        max_gap=0.18,
        name="flume spout sits just above wheel rim",
    )
    ctx.expect_gap(
        support,
        gate,
        axis="y",
        positive_elem="left_gate_guide",
        negative_elem="gate_panel",
        min_gap=0.02,
        max_gap=0.06,
        name="gate clears left guide slot",
    )
    ctx.expect_gap(
        gate,
        support,
        axis="y",
        positive_elem="gate_panel",
        negative_elem="right_gate_guide",
        min_gap=0.02,
        max_gap=0.06,
        name="gate clears right guide slot",
    )
    ctx.expect_gap(
        gate,
        support,
        axis="z",
        positive_elem="gate_panel",
        negative_elem="gate_sill",
        min_gap=0.0,
        max_gap=0.01,
        max_penetration=0.0,
        name="closed gate drops to the sill",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="left_bearing",
        elem_b="axle",
        contact_tol=1e-6,
        name="left bearing supports the axle",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="right_bearing",
        elem_b="axle",
        contact_tol=1e-6,
        name="right bearing supports the axle",
    )

    wheel_rest = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: pi / 3.0}):
        wheel_rotated = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins about a fixed axle center",
        wheel_rest is not None
        and wheel_rotated is not None
        and abs(wheel_rest[0] - wheel_rotated[0]) < 1e-6
        and abs(wheel_rest[1] - wheel_rotated[1]) < 1e-6
        and abs(wheel_rest[2] - wheel_rotated[2]) < 1e-6,
        details=f"rest={wheel_rest}, rotated={wheel_rotated}",
    )

    gate_rest = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 0.42}):
        gate_open = ctx.part_world_position(gate)
        ctx.expect_gap(
            gate,
            support,
            axis="z",
            positive_elem="gate_panel",
            negative_elem="gate_sill",
            min_gap=0.38,
            name="raised gate opens the flume throat",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="y",
            positive_elem="left_gate_guide",
            negative_elem="gate_panel",
            min_gap=0.02,
            max_gap=0.06,
            name="raised gate stays in left guide slot",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="y",
            positive_elem="gate_panel",
            negative_elem="right_gate_guide",
            min_gap=0.02,
            max_gap=0.06,
            name="raised gate stays in right guide slot",
        )
    ctx.check(
        "gate lifts upward when opened",
        gate_rest is not None and gate_open is not None and gate_open[2] > gate_rest[2] + 0.35,
        details=f"rest={gate_rest}, open={gate_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
