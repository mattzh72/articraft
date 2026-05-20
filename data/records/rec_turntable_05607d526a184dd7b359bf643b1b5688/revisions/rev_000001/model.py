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


PLINTH_SIZE = (0.50, 0.38, 0.08)
PLATTER_RADIUS = 0.165
PLATTER_CENTER_Z = 0.116
PLATTER_THICKNESS = 0.042
TONEARM_PIVOT = (0.170, 0.110, 0.215)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused rectangular plinth, centered on the local origin."""
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges("|Z")
        .fillet(radius)
    )


def _annulus(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """Thin record-groove ring in the XY plane, extruded upward from z=0."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("warm_walnut", rgba=(0.46, 0.25, 0.12, 1.0))
    black = model.material("satin_black", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    vinyl = model.material("deep_vinyl", rgba=(0.002, 0.002, 0.003, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.73, 0.69, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.08, 0.085, 1.0))
    label_red = model.material("muted_red_label", rgba=(0.65, 0.08, 0.045, 1.0))
    ivory = model.material("ivory_mark", rgba=(0.92, 0.86, 0.68, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(_rounded_box(PLINTH_SIZE, 0.028), "rounded_plinth"),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_SIZE[2] / 2.0)),
        material=walnut,
        name="rounded_plinth",
    )
    plinth.visual(
        Box((0.44, 0.32, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=black,
        name="top_plate",
    )
    plinth.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=dark_metal,
        name="bearing_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.043, length=0.014),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], 0.092)),
        material=dark_metal,
        name="tonearm_base_foot",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.130),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], 0.150)),
        material=aluminum,
        name="tonearm_post",
    )
    # Four shallow feet touch and support the plinth without becoming separate parts.
    for i, (x, y) in enumerate(
        ((-0.185, -0.135), (0.185, -0.135), (-0.185, 0.135), (0.185, 0.135))
    ):
        plinth.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(x, y, 0.002)),
            material=rubber,
            name=f"foot_{i}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=PLATTER_RADIUS, length=PLATTER_THICKNESS),
        origin=Origin(),
        material=aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.155, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.150, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=vinyl,
        name="vinyl_record",
    )
    platter.visual(
        mesh_from_cadquery(_annulus(0.132, 0.126, 0.001), "outer_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.0297)),
        material=dark_metal,
        name="outer_groove",
    )
    platter.visual(
        mesh_from_cadquery(_annulus(0.096, 0.090, 0.001), "inner_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.0297)),
        material=dark_metal,
        name="inner_groove",
    )
    platter.visual(
        Cylinder(radius=0.035, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.03075)),
        material=label_red,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.04825)),
        material=aluminum,
        name="center_spindle",
    )
    # A small cream timing mark makes the continuous platter rotation visible.
    platter.visual(
        Box((0.028, 0.006, 0.002)),
        origin=Origin(xyz=(0.134, 0.0, 0.0307)),
        material=ivory,
        name="index_mark",
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="pivot_hub",
    )
    tonearm.visual(
        Box((0.245, 0.009, 0.009)),
        origin=Origin(xyz=(-0.126, 0.0, 0.020)),
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.041, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.040, 0.026, 0.012)),
        origin=Origin(xyz=(-0.265, 0.0, 0.010)),
        material=dark_metal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.022, 0.014, 0.020)),
        origin=Origin(xyz=(-0.276, 0.0, -0.006)),
        material=black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0015, length=0.053),
        origin=Origin(xyz=(-0.277, 0.0, -0.0425)),
        material=ivory,
        name="stylus",
    )

    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=TONEARM_PIVOT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.2, lower=-0.25, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("plinth_to_platter")
    tonearm_swing = object_model.get_articulation("plinth_to_tonearm")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disc",
        negative_elem="bearing_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter rests on bearing pedestal",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_disc",
        elem_b="bearing_pedestal",
        min_overlap=0.10,
        name="central platter is supported from below",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_hub",
        negative_elem="tonearm_post",
        max_gap=0.001,
        max_penetration=0.0,
        name="tonearm hub sits on tall side post",
    )

    with ctx.pose({tonearm_swing: 0.50}):
        ctx.expect_within(
            tonearm,
            platter,
            axes="xy",
            inner_elem="stylus",
            outer_elem="vinyl_record",
            margin=0.0,
            name="tonearm swing places stylus over the record",
        )

    with ctx.pose({platter_spin: math.pi / 2.0}):
        mark_aabb = ctx.part_element_world_aabb(platter, elem="index_mark")
    ctx.check(
        "platter index mark rotates visibly",
        mark_aabb is not None
        and abs(((mark_aabb[0][0] + mark_aabb[1][0]) / 2.0)) < 0.02
        and ((mark_aabb[0][1] + mark_aabb[1][1]) / 2.0) > 0.12,
        details=f"rotated index mark aabb={mark_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
