from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


HEAD_ANGLE = math.radians(-20.0)


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _head_tube_shell():
    # Local +Z is the steering axis.  The outer profile flares wider toward the
    # lower bearing, matching modern tapered head tubes for fat-bike forks.
    return LatheGeometry.from_shell_profiles(
        [
            (0.055, -0.006),
            (0.058, 0.018),
            (0.050, 0.050),
            (0.044, 0.140),
            (0.040, 0.205),
            (0.043, 0.246),
        ],
        [
            (0.031, -0.006),
            (0.030, 0.060),
            (0.025, 0.190),
            (0.024, 0.246),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _bearing_ring(z0: float, z1: float, inner: float, outer: float):
    return LatheGeometry.from_shell_profiles(
        [(outer, z0), (outer, z1)],
        [(inner, z0), (inner, z1)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _crown_mesh():
    crown_profile = rounded_rect_profile(0.185, 0.082, 0.022, corner_segments=10)
    return ExtrudeGeometry(crown_profile, 0.345, center=True, cap=True)


def _blade_mesh(y: float):
    # The two fork blades are deliberately stout and widely spaced for fat tire
    # clearance.  The path gives them a small forward rake below the crown.
    return tube_from_spline_points(
        [
            (0.032, y, -0.070),
            (0.058, y * 1.04, -0.210),
            (0.100, y * 1.08, -0.520),
            (0.138, y * 1.05, -0.790),
        ],
        radius=0.030,
        samples_per_segment=18,
        radial_segments=28,
        up_hint=(0.0, 1.0, 0.0),
    )


def _handlebar_mesh():
    return tube_from_spline_points(
        [
            (-0.012, -0.440, 0.350),
            (0.032, -0.260, 0.358),
            (0.062, 0.000, 0.360),
            (0.032, 0.260, 0.358),
            (-0.012, 0.440, 0.350),
        ],
        radius=0.0125,
        samples_per_segment=18,
        radial_segments=24,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fat_bike_fork")

    frame_paint = model.material("deep_blue_frame", rgba=(0.05, 0.11, 0.19, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    alloy = model.material("brushed_alloy", rgba=(0.72, 0.73, 0.70, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    rubber = model.material("rubber_grip", rgba=(0.02, 0.022, 0.024, 1.0))
    steel = model.material("polished_steel", rgba=(0.63, 0.65, 0.67, 1.0))

    head_tube = model.part("head_tube")
    head_axis_origin = Origin(xyz=(0.0, 0.0, 1.00), rpy=(0.0, HEAD_ANGLE, 0.0))
    head_tube.visual(
        mesh_from_geometry(_head_tube_shell(), "tapered_head_tube"),
        origin=head_axis_origin,
        material=frame_paint,
        name="tapered_head_tube",
    )
    head_tube.visual(
        mesh_from_geometry(_bearing_ring(-0.012, 0.016, 0.031, 0.059), "lower_bearing_cup"),
        origin=head_axis_origin,
        material=dark_metal,
        name="lower_bearing_cup",
    )
    head_tube.visual(
        mesh_from_geometry(_bearing_ring(0.224, 0.256, 0.024, 0.044), "upper_bearing_cup"),
        origin=head_axis_origin,
        material=dark_metal,
        name="upper_bearing_cup",
    )
    # Short welded frame stubs make the tapered tube read as part of a bike
    # front triangle without distracting from the requested fork.
    _add_tube(head_tube, (-0.124, 0.0, 1.205), (-0.560, 0.0, 1.155), 0.022, frame_paint, name="top_tube_stub")
    _add_tube(head_tube, (-0.075, 0.0, 1.030), (-0.500, 0.0, 0.720), 0.035, frame_paint, name="down_tube_stub")

    fork = model.part("steerer_fork")
    fork.visual(
        Cylinder(radius=0.014, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=steel,
        name="steerer_tube",
    )
    fork.visual(
        mesh_from_geometry(_bearing_ring(-0.020, -0.012, 0.018, 0.045), "crown_race"),
        material=steel,
        name="crown_race",
    )
    fork.visual(
        mesh_from_geometry(_crown_mesh(), "wide_alloy_crown"),
        origin=Origin(xyz=(0.040, 0.0, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="wide_alloy_crown",
    )
    fork.visual(
        Cylinder(radius=0.036, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=alloy,
        name="steerer_crown_socket",
    )
    for idx, y in enumerate((-0.135, 0.135)):
        fork.visual(
            Cylinder(radius=0.040, length=0.060),
            origin=Origin(xyz=(0.034, y, -0.068)),
            material=alloy,
            name=f"blade_root_{idx}",
        )
        fork.visual(
            mesh_from_geometry(_blade_mesh(y), f"oversized_blade_{idx}"),
            material=black,
            name=f"oversized_blade_{idx}",
        )
        fork.visual(
            Box((0.040, 0.024, 0.082)),
            origin=Origin(xyz=(0.148, y * 1.05, -0.820)),
            material=black,
            name=f"dropout_{idx}",
        )
        fork.visual(
            Cylinder(radius=0.012, length=0.046),
            origin=Origin(xyz=(0.157, y * 1.05, -0.835), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"axle_boss_{idx}",
        )

    fork.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(0.157, 0.0, -0.835), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="through_axle",
    )
    fork.visual(
        Cylinder(radius=0.017, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=dark_metal,
        name="stem_clamp_stack",
    )
    fork.visual(
        Cylinder(radius=0.018, length=0.112),
        origin=Origin(xyz=(0.035, 0.0, 0.333), rpy=(0.0, math.radians(63.0), 0.0)),
        material=dark_metal,
        name="short_stem",
    )
    fork.visual(
        Cylinder(radius=0.030, length=0.055),
        origin=Origin(xyz=(0.063, 0.0, 0.360), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bar_clamp",
    )
    fork.visual(
        mesh_from_geometry(_handlebar_mesh(), "wide_flat_handlebar"),
        material=black,
        name="wide_flat_handlebar",
    )
    for idx, y in enumerate((-0.405, 0.405)):
        fork.visual(
            Cylinder(radius=0.0165, length=0.110),
            origin=Origin(xyz=(-0.012, y, 0.350), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{idx}",
        )

    model.articulation(
        "steering_pivot",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=head_axis_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=math.radians(-80.0),
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("steerer_fork")
    steering = object_model.get_articulation("steering_pivot")

    ctx.allow_overlap(
        head_tube,
        fork,
        elem_a="lower_bearing_cup",
        elem_b="crown_race",
        reason=(
            "The fork crown race is intentionally seated into the lower headset "
            "cup as a local captured bearing interface."
        ),
    )
    ctx.expect_gap(
        head_tube,
        fork,
        axis="z",
        positive_elem="lower_bearing_cup",
        negative_elem="crown_race",
        max_gap=0.004,
        max_penetration=0.040,
        name="crown race is seated at the lower headset cup",
    )

    ctx.check(
        "steerer has realistic steering limits",
        steering.motion_limits is not None
        and steering.motion_limits.lower < -1.2
        and steering.motion_limits.upper > 1.2,
        details=f"limits={steering.motion_limits}",
    )
    ctx.expect_overlap(
        fork,
        head_tube,
        axes="z",
        min_overlap=0.18,
        elem_a="steerer_tube",
        elem_b="tapered_head_tube",
        name="steerer passes through tapered head tube",
    )

    rest_aabb = ctx.part_element_world_aabb(fork, elem="wide_flat_handlebar")
    with ctx.pose({steering: math.radians(50.0)}):
        turned_aabb = ctx.part_element_world_aabb(fork, elem="wide_flat_handlebar")
    ctx.check(
        "handlebar turns with steerer",
        rest_aabb is not None
        and turned_aabb is not None
        and (rest_aabb[1][1] - rest_aabb[0][1]) - (turned_aabb[1][1] - turned_aabb[0][1]) > 0.10,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
