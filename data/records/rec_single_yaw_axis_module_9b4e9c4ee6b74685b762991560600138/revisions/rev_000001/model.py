from __future__ import annotations

import math

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


def _hex_prism(radius: float, height: float) -> cq.Workplane:
    points = [
        (
            radius * math.cos(math.pi / 6.0 + i * math.tau / 6.0),
            radius * math.sin(math.pi / 6.0 + i * math.tau / 6.0),
        )
        for i in range(6)
    ]
    return cq.Workplane("XY").polyline(points).close().extrude(height)


def _solid_cylinder(radius: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _annulus(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate((0.0, 0.0, -0.002))
    return outer.cut(bore).translate((0.0, 0.0, z0))


def _build_static_support() -> cq.Workplane:
    pedestal = _hex_prism(0.105, 0.045).edges("|Z").chamfer(0.004)
    shoulder = _solid_cylinder(0.074, 0.014, 0.043)
    lower_race = _annulus(0.122, 0.058, 0.016, 0.055)
    outer_register = _annulus(0.128, 0.120, 0.010, 0.059)
    center_bushing = _solid_cylinder(0.034, 0.014, 0.055)
    return pedestal.union(shoulder).union(lower_race).union(outer_register).union(center_bushing)


def _build_upper_ring() -> cq.Workplane:
    bearing_race = _annulus(0.108, 0.052, 0.014, 0.0)
    adapter_boss = _solid_cylinder(0.070, 0.014, 0.012)
    return bearing_race.union(adapter_boss)


def _build_top_deck() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(0.180, 0.180, 0.022)
        .translate((0.0, 0.0, 0.037))
        .edges("|Z")
        .fillet(0.006)
    )
    hole_points = [
        (-0.060, -0.060),
        (-0.060, 0.060),
        (0.060, -0.060),
        (0.060, 0.060),
    ]
    through_holes = (
        cq.Workplane("XY")
        .pushPoints(hole_points)
        .circle(0.0065)
        .extrude(0.040)
        .translate((0.0, 0.0, 0.018))
    )
    counterbores = (
        cq.Workplane("XY")
        .pushPoints(hole_points)
        .circle(0.012)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.040))
    )
    return deck.cut(through_holes).cut(counterbores)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_base")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.20, 0.21, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    anodized_blue = model.material("anodized_blue", rgba=(0.02, 0.11, 0.24, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.74, 0.68, 1.0))
    white_mark = model.material("white_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_build_static_support(), "static_hex_pedestal"),
        material=matte_black,
        name="hex_pedestal",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        support.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(
                xyz=(0.116 * math.cos(angle), 0.116 * math.sin(angle), 0.071)
            ),
            material=stainless,
            name=f"fixed_bolt_{i}",
        )

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_build_upper_ring(), "upper_slewing_ring"),
        material=satin_steel,
        name="slewing_ring",
    )
    deck.visual(
        mesh_from_cadquery(_build_top_deck(), "square_mount_deck"),
        material=anodized_blue,
        name="top_deck",
    )
    deck.visual(
        Box((0.014, 0.048, 0.003)),
        origin=Origin(xyz=(0.0, 0.065, 0.0495)),
        material=white_mark,
        name="index_mark",
    )
    for i in range(8):
        angle = math.pi / 8.0 + i * math.tau / 8.0
        deck.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(
                xyz=(0.089 * math.cos(angle), 0.089 * math.sin(angle), 0.0155)
            ),
            material=dark_steel,
            name=f"race_screw_{i}",
        )

    model.articulation(
        "support_to_deck",
        ArticulationType.REVOLUTE,
        parent=support,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    deck = object_model.get_part("deck")
    yaw = object_model.get_articulation("support_to_deck")

    ctx.check("single vertical yaw joint", yaw.axis == (0.0, 0.0, 1.0))
    ctx.expect_contact(
        deck,
        support,
        elem_a="slewing_ring",
        elem_b="hex_pedestal",
        contact_tol=0.001,
        name="upper ring sits on fixed lower race",
    )
    ctx.expect_overlap(
        deck,
        support,
        axes="xy",
        elem_a="slewing_ring",
        elem_b="hex_pedestal",
        min_overlap=0.08,
        name="turntable races share the centerline",
    )

    rest_mark = ctx.part_element_world_aabb(deck, elem="index_mark")
    with ctx.pose({yaw: math.pi / 2.0}):
        turned_mark = ctx.part_element_world_aabb(deck, elem="index_mark")
    if rest_mark is None or turned_mark is None:
        ctx.fail("index mark follows yaw", "could not read index mark AABBs")
    else:
        rest_center = (
            (rest_mark[0][0] + rest_mark[1][0]) / 2.0,
            (rest_mark[0][1] + rest_mark[1][1]) / 2.0,
        )
        turned_center = (
            (turned_mark[0][0] + turned_mark[1][0]) / 2.0,
            (turned_mark[0][1] + turned_mark[1][1]) / 2.0,
        )
        ctx.check(
            "index mark follows yaw",
            rest_center[1] > 0.060 and turned_center[0] < -0.060,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    return ctx.report()


object_model = build_object_model()
