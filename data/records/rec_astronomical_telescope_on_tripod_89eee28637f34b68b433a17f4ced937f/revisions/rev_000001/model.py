from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _axis_rpy_from_z(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an rpy rotation that aligns a local +Z cylinder with p0 -> p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(horizontal, dz)
    return (0.0, pitch, yaw)


def _cylinder_between(part, p0, p1, *, radius: float, material, name: str) -> None:
    length = math.dist(p0, p1)
    mid = tuple((a + b) * 0.5 for a, b in zip(p0, p1))
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=_axis_rpy_from_z(p0, p1)),
        material=material,
        name=name,
    )


def _annular_x_tube(outer_radius: float, inner_radius: float, length: float):
    """Open cylindrical shell along local X for optical tubes."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length, both=True)
    )


def _annular_z_ring(outer_radius: float, inner_radius: float, height: float):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refractor_spotting_scope_tripod")

    satin_black = model.material("satin_black", rgba=(0.02, 0.023, 0.026, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    anodized = model.material("anodized_aluminum", rgba=(0.38, 0.40, 0.42, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("coated_glass", rgba=(0.13, 0.32, 0.48, 0.62))
    screw_metal = model.material("screw_metal", rgba=(0.72, 0.73, 0.70, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.030, length=0.53),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=anodized,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.075, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=graphite,
        name="leg_hub",
    )
    tripod.visual(
        Cylinder(radius=0.090, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        material=graphite,
        name="top_plate",
    )
    tripod.visual(
        Cylinder(radius=0.038, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.782)),
        material=graphite,
        name="column_collar",
    )
    for i in range(3):
        a = 2.0 * math.pi * i / 3.0 + math.radians(30)
        top = (0.075 * math.cos(a), 0.075 * math.sin(a), 0.700)
        foot = (0.50 * math.cos(a), 0.50 * math.sin(a), 0.055)
        _cylinder_between(
            tripod,
            top,
            foot,
            radius=0.018,
            material=anodized,
            name=f"leg_{i}",
        )
        brace_top = (0.032 * math.cos(a), 0.032 * math.sin(a), 0.525)
        brace_foot = (0.355 * math.cos(a), 0.355 * math.sin(a), 0.245)
        _cylinder_between(
            tripod,
            brace_top,
            brace_foot,
            radius=0.010,
            material=graphite,
            name=f"brace_{i}",
        )
        tripod.visual(
            Box((0.14, 0.055, 0.020)),
            origin=Origin(xyz=foot, rpy=(0.0, 0.0, a)),
            material=rubber,
            name=f"foot_{i}",
        )
    tripod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.55, length=0.85),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="pan_disc",
    )
    pan_head.visual(
        Cylinder(radius=0.036, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=satin_black,
        name="pedestal_stem",
    )
    pan_head.visual(
        Cylinder(radius=0.065, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=graphite,
        name="socket_flare",
    )
    pan_head.visual(
        mesh_from_cadquery(_annular_z_ring(0.078, 0.045, 0.020), "socket_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=graphite,
        name="socket_ring",
    )
    pan_head.visual(
        Cylinder(radius=0.008, length=0.105),
        origin=Origin(xyz=(0.0, 0.102, 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="pan_lock_stem",
    )
    pan_head.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.163, 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="pan_lock_knob",
    )
    pan_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.14),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(Sphere(radius=0.055), material=satin_black, name="ball")
    tilt_head.visual(
        Cylinder(radius=0.018, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=satin_black,
        name="neck_post",
    )
    tilt_head.visual(
        Box((0.190, 0.130, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=graphite,
        name="saddle_plate",
    )
    tilt_head.visual(
        Box((0.055, 0.014, 0.036)),
        origin=Origin(xyz=(-0.065, 0.062, 0.161)),
        material=graphite,
        name="clamp_lug_0",
    )
    tilt_head.visual(
        Box((0.055, 0.014, 0.036)),
        origin=Origin(xyz=(0.065, -0.062, 0.161)),
        material=graphite,
        name="clamp_lug_1",
    )
    tilt_head.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(0.0, -0.060, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="tilt_lock_stem",
    )
    tilt_head.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, -0.115, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tilt_lock_knob",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Sphere(radius=0.10),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    scope = model.part("scope")
    scope.visual(
        mesh_from_cadquery(_annular_x_tube(0.050, 0.041, 0.480), "main_tube_shell"),
        material=satin_black,
        name="main_tube",
    )
    scope.visual(
        mesh_from_cadquery(_annular_x_tube(0.064, 0.049, 0.140), "dew_shield_shell"),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=satin_black,
        name="dew_shield",
    )
    scope.visual(
        Cylinder(radius=0.049, length=0.008),
        origin=Origin(xyz=(0.315, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    scope.visual(
        Cylinder(radius=0.056, length=0.052),
        origin=Origin(xyz=(-0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="focus_ring",
    )
    scope.visual(
        Box((0.170, 0.048, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=graphite,
        name="dovetail_foot",
    )
    scope.visual(
        Cylinder(radius=0.043, length=0.065),
        origin=Origin(xyz=(-0.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_cell",
    )
    _cylinder_between(
        scope,
        (-0.300, 0.0, 0.020),
        (-0.360, 0.0, 0.090),
        radius=0.026,
        material=graphite,
        name="diagonal_body",
    )
    _cylinder_between(
        scope,
        (-0.356, 0.0, 0.085),
        (-0.418, 0.0, 0.156),
        radius=0.021,
        material=rubber,
        name="eyepiece_barrel",
    )
    scope.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(-0.420, 0.0, 0.158), rpy=_axis_rpy_from_z((-0.356, 0.0, 0.085), (-0.418, 0.0, 0.156))),
        material=glass,
        name="eyepiece_glass",
    )
    scope.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.62),
        mass=1.35,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "pan_bearing",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.55, upper=0.80),
    )
    model.articulation(
        "scope_mount",
        ArticulationType.FIXED,
        parent=tilt_head,
        child=scope,
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    pan_head = object_model.get_part("pan_head")
    tilt_head = object_model.get_part("tilt_head")
    scope = object_model.get_part("scope")
    pan = object_model.get_articulation("pan_bearing")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.check("pan joint is continuous", pan.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("tilt joint is limited revolute", tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.allow_overlap(
        pan_head,
        tilt_head,
        elem_a="socket_ring",
        elem_b="ball",
        reason="The ball is intentionally seated into the socket lip of the ball head.",
    )
    ctx.expect_gap(
        pan_head,
        tripod,
        axis="z",
        positive_elem="pan_disc",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan bearing sits on tripod top plate",
    )
    ctx.expect_gap(
        scope,
        tilt_head,
        axis="z",
        positive_elem="dovetail_foot",
        negative_elem="saddle_plate",
        max_gap=0.004,
        max_penetration=0.001,
        name="scope dovetail foot is seated on ball-head saddle",
    )
    ctx.expect_within(
        tilt_head,
        pan_head,
        axes="xy",
        inner_elem="ball",
        outer_elem="socket_ring",
        margin=0.0,
        name="ball stays centered inside socket lip",
    )
    ctx.expect_overlap(
        tilt_head,
        pan_head,
        axes="z",
        elem_a="ball",
        elem_b="socket_ring",
        min_overlap=0.002,
        name="socket lip captures the ball vertically",
    )

    def _elem_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    def _elem_center_xy(part, elem: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    rest_front_z = _elem_center_z(scope, "front_lens")
    with ctx.pose({tilt: 0.65}):
        raised_front_z = _elem_center_z(scope, "front_lens")
    ctx.check(
        "positive tilt raises objective end",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 0.08,
        details=f"rest_z={rest_front_z}, raised_z={raised_front_z}",
    )

    rest_front_xy = _elem_center_xy(scope, "front_lens")
    with ctx.pose({pan: math.pi / 2.0}):
        panned_front_xy = _elem_center_xy(scope, "front_lens")
    ctx.check(
        "pan bearing rotates objective around vertical axis",
        rest_front_xy is not None
        and panned_front_xy is not None
        and abs(rest_front_xy[0]) > 0.25
        and abs(panned_front_xy[1]) > 0.25,
        details=f"rest_xy={rest_front_xy}, panned_xy={panned_front_xy}",
    )

    return ctx.report()


object_model = build_object_model()
