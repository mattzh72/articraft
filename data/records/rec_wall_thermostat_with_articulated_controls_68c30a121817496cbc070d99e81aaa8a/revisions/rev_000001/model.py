from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_WIDTH = 0.145
PLATE_HEIGHT = 0.170
PLATE_THICKNESS = 0.006
BODY_HEIGHT = 0.023
BODY_TOP_Z = PLATE_THICKNESS + BODY_HEIGHT
DIAL_CENTER = (-0.009, 0.006)
DIAL_OUTER_RADIUS = 0.046
DIAL_INNER_RADIUS = 0.032
DIAL_BACK_Z = BODY_TOP_Z + 0.0022
DIAL_DEPTH = 0.012


def _rounded_rect_points(width: float, height: float, radius: float, *, segments: int = 10):
    """Counter-clockwise rounded rectangle profile in the XY plane."""
    hw = width * 0.5 - radius
    hh = height * 0.5 - radius
    points = []
    corners = (
        (hw, hh, 0.0, math.pi * 0.5),
        (-hw, hh, math.pi * 0.5, math.pi),
        (-hw, -hh, math.pi, math.pi * 1.5),
        (hw, -hh, math.pi * 1.5, math.pi * 2.0),
    )
    for cx, cy, start, end in corners:
        for i in range(segments + 1):
            if points and i == 0:
                continue
            angle = start + (end - start) * i / segments
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def _asymmetric_body_points(*, segments: int = 112):
    """Soft lopsided superellipse: extra shelf on the right and lower edge."""
    points = []
    exponent = 3.4
    for i in range(segments):
        t = math.pi * 2.0 * i / segments
        c = math.cos(t)
        s = math.sin(t)
        a = 0.070 if c >= 0.0 else 0.060
        b = 0.058 if s >= 0.0 else 0.066
        x = (1.0 if c >= 0.0 else -1.0) * a * (abs(c) ** (2.0 / exponent)) + 0.004
        y = (1.0 if s >= 0.0 else -1.0) * b * (abs(s) ** (2.0 / exponent)) - 0.002
        points.append((x, y))
    return points


def _extrude_profile(points, height: float):
    return cq.Workplane("XY").polyline(points).close().extrude(height)


def _make_fixed_shell():
    plate = _extrude_profile(
        _rounded_rect_points(PLATE_WIDTH, PLATE_HEIGHT, 0.014, segments=10),
        PLATE_THICKNESS,
    )
    body = _extrude_profile(_asymmetric_body_points(), BODY_HEIGHT + 0.001).translate(
        (0.0, 0.0, PLATE_THICKNESS - 0.001)
    )
    return plate.union(body)


def _make_dial_ring():
    return (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_RADIUS)
        .circle(DIAL_INNER_RADIUS)
        .extrude(DIAL_DEPTH)
    )


def _make_bearing_lip():
    return (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_RADIUS + 0.0010)
        .circle(DIAL_INNER_RADIUS - 0.0010)
        .extrude(DIAL_BACK_Z - BODY_TOP_Z)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_large_dial")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    soft_shadow = model.material("soft_shadow_grey", rgba=(0.42, 0.43, 0.41, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.02, 0.025, 0.030, 1.0))
    brushed_metal = model.material("brushed_warm_metal", rgba=(0.74, 0.70, 0.62, 1.0))
    print_white = model.material("printed_white", rgba=(0.95, 0.96, 0.92, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_fixed_shell(), "fixed_wall_plate_body", tolerance=0.00055),
        material=warm_white,
        name="wall_plate_body",
    )
    housing.visual(
        Cylinder(radius=0.029, length=0.0018),
        origin=Origin(xyz=(DIAL_CENTER[0], DIAL_CENTER[1], BODY_TOP_Z + 0.0009)),
        material=dark_glass,
        name="display_face",
    )
    housing.visual(
        Box((0.020, 0.004, 0.0007)),
        origin=Origin(xyz=(DIAL_CENTER[0], DIAL_CENTER[1] + 0.004, BODY_TOP_Z + 0.00215)),
        material=print_white,
        name="display_mark",
    )
    housing.visual(
        Box((0.004, 0.027, 0.0010)),
        origin=Origin(xyz=(0.054, -0.026, BODY_TOP_Z + 0.0005)),
        material=soft_shadow,
        name="sensor_slot",
    )
    housing.visual(
        Box((0.017, 0.003, 0.0010)),
        origin=Origin(xyz=(0.054, -0.042, BODY_TOP_Z + 0.0005)),
        material=soft_shadow,
        name="sensor_vent",
    )
    housing.visual(
        mesh_from_cadquery(_make_bearing_lip(), "fixed_bearing_lip", tolerance=0.00045),
        origin=Origin(xyz=(DIAL_CENTER[0], DIAL_CENTER[1], BODY_TOP_Z)),
        material=soft_shadow,
        name="bearing_lip",
    )

    dial_ring = model.part("dial_ring")
    dial_ring.visual(
        mesh_from_cadquery(_make_dial_ring(), "rotating_dial_annulus", tolerance=0.00045),
        material=brushed_metal,
        name="dial_annulus",
    )
    dial_ring.visual(
        Box((0.004, 0.017, 0.0010)),
        origin=Origin(xyz=(0.0, DIAL_OUTER_RADIUS - 0.0075, DIAL_DEPTH + 0.00025)),
        material=soft_shadow,
        name="index_mark",
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial_ring,
        origin=Origin(xyz=(DIAL_CENTER[0], DIAL_CENTER[1], DIAL_BACK_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.45, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    dial_ring = object_model.get_part("dial_ring")
    dial_spin = object_model.get_articulation("dial_spin")

    ctx.check(
        "dial has continuous rotation",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {dial_spin.articulation_type}",
    )
    ctx.expect_gap(
        dial_ring,
        housing,
        axis="z",
        min_gap=0.0,
        max_gap=0.0002,
        positive_elem="dial_annulus",
        negative_elem="bearing_lip",
        name="rotating ring rides on fixed bearing lip",
    )
    ctx.expect_overlap(
        dial_ring,
        housing,
        axes="xy",
        min_overlap=0.060,
        elem_a="dial_annulus",
        elem_b="wall_plate_body",
        name="large dial remains seated on thermostat body",
    )

    rest_pos = ctx.part_world_position(dial_ring)
    with ctx.pose({dial_spin: math.pi * 1.5}):
        turned_pos = ctx.part_world_position(dial_ring)
        ctx.expect_gap(
            dial_ring,
            housing,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            positive_elem="dial_annulus",
            negative_elem="bearing_lip",
            name="turned dial stays on bearing lip",
        )
    ctx.check(
        "dial rotation stays centered",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
