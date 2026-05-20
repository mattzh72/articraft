from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


SEAT_HEIGHT = 0.66
SEAT_RADIUS = 0.20
HINGE_X = -0.18
HINGE_Z = 0.05


def _merge_geometries(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _build_base_geometry():
    base_plate = LatheGeometry(
        [
            (0.0, 0.0),
            (0.08, 0.0),
            (0.16, 0.004),
            (0.205, 0.010),
            (0.225, 0.020),
            (0.225, 0.026),
            (0.18, 0.034),
            (0.09, 0.038),
            (0.0, 0.038),
        ],
        segments=72,
    )

    lower_pedestal = CylinderGeometry(radius=0.045, height=0.48, radial_segments=48).translate(
        0.0,
        0.0,
        0.278,
    )
    upper_sleeve = CylinderGeometry(radius=0.034, height=0.11, radial_segments=42).translate(
        0.0,
        0.0,
        0.555,
    )
    lift_stem = CylinderGeometry(radius=0.024, height=0.08, radial_segments=36).translate(
        0.0,
        0.0,
        0.615,
    )
    swivel_hub = CylinderGeometry(radius=0.072, height=0.04, radial_segments=48).translate(
        0.0,
        0.0,
        SEAT_HEIGHT - 0.02,
    )

    ring_collar = CylinderGeometry(radius=0.055, height=0.06, radial_segments=36).translate(
        0.0,
        0.0,
        0.235,
    )
    foot_ring = TorusGeometry(
        radius=0.17,
        tube=0.011,
        radial_segments=18,
        tubular_segments=72,
    ).translate(0.0, 0.0, 0.235)

    support_spokes = []
    spoke_path = [
        (0.040, 0.0, 0.235),
        (0.095, 0.0, 0.230),
        (0.162, 0.0, 0.235),
    ]
    for angle in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
        support_spokes.append(
            tube_from_spline_points(
                spoke_path,
                radius=0.0075,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ).rotate_z(angle)
        )

    return _merge_geometries(
        base_plate,
        lower_pedestal,
        upper_sleeve,
        lift_stem,
        swivel_hub,
        ring_collar,
        foot_ring,
        *support_spokes,
    )


def _build_seat_shell_geometry():
    shell_body = LatheGeometry(
        [
            (0.0, 0.0),
            (0.065, 0.0),
            (0.145, 0.004),
            (0.186, 0.011),
            (0.198, 0.021),
            (0.196, 0.032),
            (0.180, 0.040),
            (0.118, 0.042),
            (0.0, 0.035),
        ],
        segments=72,
    )

    hinge_mount = BoxGeometry((0.050, 0.120, 0.022)).translate(HINGE_X + 0.007, 0.0, 0.031)

    barrel_left = (
        CylinderGeometry(radius=0.011, height=0.034, radial_segments=28)
        .rotate_x(math.pi * 0.5)
        .translate(HINGE_X, 0.045, HINGE_Z)
    )
    barrel_right = (
        CylinderGeometry(radius=0.011, height=0.034, radial_segments=28)
        .rotate_x(math.pi * 0.5)
        .translate(HINGE_X, -0.045, HINGE_Z)
    )

    return _merge_geometries(shell_body, hinge_mount, barrel_left, barrel_right)


def _build_seat_cushion_geometry():
    return LatheGeometry(
        [
            (0.0, 0.028),
            (0.050, 0.030),
            (0.112, 0.036),
            (0.155, 0.045),
            (0.167, 0.055),
            (0.165, 0.068),
            (0.150, 0.078),
            (0.082, 0.084),
            (0.0, 0.080),
        ],
        segments=72,
    )


def _build_backrest_frame_geometry():
    hinge_barrel = (
        CylinderGeometry(radius=0.0105, height=0.052, radial_segments=28)
        .rotate_x(math.pi * 0.5)
        .translate(0.0, 0.0, 0.0)
    )

    left_arm = tube_from_spline_points(
        [
            (0.0, 0.018, 0.0),
            (-0.010, 0.024, 0.050),
            (-0.026, 0.040, 0.106),
            (-0.032, 0.060, 0.145),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    right_arm = tube_from_spline_points(
        [
            (0.0, -0.018, 0.0),
            (-0.010, -0.024, 0.050),
            (-0.026, -0.040, 0.106),
            (-0.032, -0.060, 0.145),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )

    return _merge_geometries(hinge_barrel, left_arm, right_arm)


def _build_backrest_pad():
    return (
        cq.Workplane("XY")
        .box(0.045, 0.255, 0.110)
        .edges()
        .fillet(0.010)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    satin_chrome = model.material("satin_chrome", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_shell = model.material("dark_shell", rgba=(0.16, 0.17, 0.19, 1.0))
    vinyl = model.material("vinyl", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_geometry(), "pedestal_assembly"),
        material=satin_chrome,
        name="pedestal_assembly",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_shell,
        name="swivel_mount",
    )
    seat.visual(
        mesh_from_geometry(_build_seat_shell_geometry(), "seat_shell"),
        material=dark_shell,
        name="seat_shell",
    )
    seat.visual(
        mesh_from_geometry(_build_seat_cushion_geometry(), "seat_cushion"),
        material=vinyl,
        name="seat_cushion",
    )

    backrest = model.part("backrest")
    backrest.visual(
        mesh_from_geometry(_build_backrest_frame_geometry(), "backrest_frame"),
        material=satin_chrome,
        name="frame",
    )
    backrest.visual(
        mesh_from_cadquery(_build_backrest_pad(), "backrest_pad"),
        origin=Origin(xyz=(-0.034, 0.0, 0.154), rpy=(0.0, -0.18, 0.0)),
        material=vinyl,
        name="pad",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SEAT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.4),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=0.30,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    swivel = object_model.get_articulation("base_to_seat")
    tilt = object_model.get_articulation("seat_to_backrest")

    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="swivel_mount",
        negative_elem="pedestal_assembly",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat bearing rests on the pedestal hub",
    )
    ctx.expect_origin_gap(
        seat,
        base,
        axis="z",
        min_gap=0.64,
        max_gap=0.68,
        name="seat height reads as counter stool scale",
    )
    ctx.expect_origin_distance(
        backrest,
        seat,
        axes="x",
        min_dist=0.17,
        max_dist=0.19,
        name="backrest hinge sits at the rear of the seat",
    )

    with ctx.pose({swivel: 0.0, tilt: 0.0}):
        rest_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem="pad"))

    with ctx.pose({swivel: math.pi * 0.5, tilt: 0.0}):
        swiveled_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem="pad"))

    ctx.check(
        "seat swivels around the pedestal axis",
        rest_center is not None
        and swiveled_center is not None
        and rest_center[0] < -0.18
        and abs(swiveled_center[0]) < 0.05
        and swiveled_center[1] < -0.18,
        details=f"rest_center={rest_center}, swiveled_center={swiveled_center}",
    )

    with ctx.pose({tilt: 0.0}):
        upright_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem="pad"))

    with ctx.pose({tilt: 0.30}):
        tilted_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem="pad"))

    ctx.check(
        "backrest tilts slightly rearward",
        upright_center is not None
        and tilted_center is not None
        and tilted_center[0] < upright_center[0] - 0.015,
        details=f"upright_center={upright_center}, tilted_center={tilted_center}",
    )

    return ctx.report()


object_model = build_object_model()
