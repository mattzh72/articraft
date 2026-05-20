from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SOCKET_TOP_Z = 0.063


def _helix(radius: float, radius_eps: float, pitch: float, height: float, dz: float, frac: float = 0.08):
    """Smoothly faded helix used for visible Edison-style thread crests."""

    def func(t: float) -> tuple[float, float, float]:
        if frac < t < 1.0 - frac:
            z = height * t + dz
            r = radius + radius_eps
        elif t <= frac:
            blend = sin(pi / 2.0 * t / frac)
            z = height * t + dz * blend
            r = radius + radius_eps * blend
        else:
            blend = sin(2.0 * pi - pi / 2.0 * (1.0 - t) / frac)
            z = height * t - dz * blend
            r = radius - radius_eps * blend

        turns = height / pitch
        x = r * sin(-2.0 * pi * turns * t)
        y = r * cos(2.0 * pi * turns * t)
        return x, y, z

    return func


def _thread_solid(radius: float, pitch: float, height: float, radial_height: float) -> cq.Solid:
    """Create a small helical thread ridge as a ruled-surface solid."""
    half_width = pitch / 4.0
    core = (
        cq.Workplane("XY")
        .parametricCurve(_helix(radius, 0.0, pitch, height, -half_width))
        .val()
    )
    core_top = (
        cq.Workplane("XY")
        .parametricCurve(_helix(radius, 0.0, pitch, height, half_width))
        .val()
    )
    crest = (
        cq.Workplane("XY")
        .parametricCurve(_helix(radius, radial_height, pitch, height, -half_width / 8.0))
        .val()
    )
    crest_top = (
        cq.Workplane("XY")
        .parametricCurve(_helix(radius, radial_height, pitch, height, half_width / 8.0))
        .val()
    )

    faces = [
        cq.Face.makeRuledSurface(core, core_top),
        cq.Face.makeRuledSurface(crest, crest_top),
        cq.Face.makeRuledSurface(core, crest),
        cq.Face.makeRuledSurface(core_top, crest_top),
    ]
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces))


def _tube(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _revolve_z(profile: list[tuple[float, float]]) -> cq.Workplane:
    """Revolve a (radius, z) profile around the final model Z axis."""
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _socket_body() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.038).extrude(0.010)
    cup = cq.Workplane("XY").circle(0.031).extrude(0.066)
    bore = cq.Workplane("XY").workplane(offset=0.011).circle(0.0197).extrude(0.060)
    return base.union(cup).cut(bore)


def _bulb_envelope() -> cq.Workplane:
    """Thin transparent pear-shaped A19 glass envelope, not a solid plug."""
    return (
        cq.Workplane("XY")
        .moveTo(0.0115, 0.006)
        .spline(
            [
                (0.020, 0.014),
                (0.030, 0.037),
                (0.028, 0.070),
                (0.012, 0.096),
                (0.000, 0.104),
            ]
        )
        .lineTo(0.000, 0.101)
        .spline(
            [
                (0.010, 0.092),
                (0.026, 0.069),
                (0.0278, 0.039),
                (0.0185, 0.016),
                (0.0103, 0.008),
            ]
        )
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _threaded_bulb_base() -> cq.Workplane:
    z_min = -0.0435
    z_max = -0.0060
    core_radius = 0.01115
    crest_radius = 0.01265
    pitch = 0.0042
    profile: list[tuple[float, float]] = [(0.0, z_min), (core_radius, z_min)]
    z = z_min
    while z + pitch < z_max:
        profile.extend(
            [
                (core_radius, z + 0.0006),
                (crest_radius, z + 0.0014),
                (crest_radius, z + 0.0024),
                (core_radius, z + 0.0033),
            ]
        )
        z += pitch
    profile.extend([(core_radius, z_max), (0.0, z_max)])
    return _revolve_z(profile)


def _socket_threads() -> cq.Workplane:
    z_min = 0.022
    z_max = 0.059
    outer_radius = 0.01580
    groove_radius = 0.01525
    crest_radius = 0.01420
    pitch = 0.0042
    profile: list[tuple[float, float]] = [(outer_radius, z_min), (outer_radius, z_max), (groove_radius, z_max)]
    z = z_max
    while z - pitch > z_min:
        profile.extend(
            [
                (groove_radius, z - 0.0005),
                (crest_radius, z - 0.0014),
                (crest_radius, z - 0.0024),
                (groove_radius, z - 0.0033),
            ]
        )
        z -= pitch
    profile.extend([(groove_radius, z_min)])
    return _revolve_z(profile)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_socket")

    ceramic = Material("warm_white_ceramic", rgba=(0.92, 0.89, 0.80, 1.0))
    brass = Material("brass", rgba=(0.78, 0.58, 0.25, 1.0))
    nickel = Material("brushed_nickel", rgba=(0.66, 0.67, 0.64, 1.0))
    glass = Material("clear_warm_glass", rgba=(0.74, 0.90, 1.0, 0.34))
    dark_print = Material("gray_print", rgba=(0.15, 0.15, 0.16, 0.82))
    tungsten = Material("warm_tungsten", rgba=(1.00, 0.72, 0.22, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_cadquery(_socket_body(), "socket_ceramic_body", tolerance=0.0008),
        material=ceramic,
        name="socket_body",
    )
    socket.visual(
        mesh_from_cadquery(_tube(0.0200, 0.0153, 0.050, 0.015), "socket_threaded_collar", tolerance=0.0005),
        material=brass,
        name="threaded_collar",
    )
    socket.visual(
        mesh_from_cadquery(_socket_threads(), "socket_internal_threads", tolerance=0.0005),
        material=brass,
        name="internal_threads",
    )
    socket.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=brass,
        name="socket_contact",
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_cadquery(_bulb_envelope(), "pear_glass_envelope", tolerance=0.0008),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        mesh_from_cadquery(_threaded_bulb_base(), "edison_threaded_base", tolerance=0.0005),
        material=nickel,
        name="threaded_base",
    )
    bulb.visual(
        Cylinder(radius=0.0140, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.001), rpy=(0.0, 0.0, 0.0)),
        material=nickel,
        name="crimp_ring",
    )
    bulb.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.0455)),
        material=brass,
        name="bottom_contact",
    )
    for x in (-0.0048, 0.0048):
        bulb.visual(
            Cylinder(radius=0.00045, length=0.047),
            origin=Origin(xyz=(x, 0.0, 0.0315)),
            material=tungsten,
            name=f"support_wire_{'neg' if x < 0 else 'pos'}",
        )
    bulb.visual(
        Cylinder(radius=0.00065, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=tungsten,
        name="filament",
    )
    bulb.visual(
        Box((0.015, 0.0040, 0.008)),
        origin=Origin(xyz=(0.0, -0.0285, 0.046)),
        material=dark_print,
        name="wattage_mark",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, SOCKET_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    joint = object_model.get_articulation("socket_to_bulb")

    ctx.check(
        "bulb joint is continuous spin",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.0,
        name="threaded bulb base stays centered inside collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.030,
        name="threaded base is seated through the collar depth",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="bottom_contact",
        elem_b="socket_contact",
        contact_tol=0.0005,
        name="bulb contact seats on socket contact",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({joint: pi / 2.0}):
        spun_pos = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.0,
            name="rotated bulb remains centered in collar footprint",
        )
    ctx.check(
        "continuous rotation keeps bulb on socket midline",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-7
        and abs(rest_pos[1] - spun_pos[1]) < 1e-7
        and abs(spun_pos[0]) < 1e-7
        and abs(spun_pos[1]) < 1e-7,
        details=f"rest={rest_pos}, rotated={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
