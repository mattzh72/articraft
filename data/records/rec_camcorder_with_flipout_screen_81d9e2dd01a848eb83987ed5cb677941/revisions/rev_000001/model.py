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


def _zoom_ring_shape() -> cq.Workplane:
    """Rubberized annular sleeve aligned to the local X/optical axis."""
    length = 0.026
    outer_radius = 0.027
    inner_radius = 0.022
    rib_height = 0.0022
    rib_width = 0.0022

    ring = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )

    for index in range(18):
        angle = index * 360.0 / 18.0
        rib = (
            cq.Workplane("XY")
            .box(length, rib_width, rib_height)
            .translate((0.0, 0.0, outer_radius + rib_height / 2.0))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
        )
        ring = ring.union(rib)

    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_camcorder")

    graphite = Material("graphite_plastic", rgba=(0.10, 0.105, 0.11, 1.0))
    dark = Material("matte_black", rgba=(0.015, 0.015, 0.017, 1.0))
    rubber = Material("soft_black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))
    satin = Material("satin_silver", rgba=(0.48, 0.50, 0.52, 1.0))
    glass = Material("blue_black_glass", rgba=(0.03, 0.07, 0.11, 0.88))
    screen = Material("dim_lcd_glass", rgba=(0.01, 0.025, 0.035, 1.0))
    white = Material("white_print", rgba=(0.86, 0.88, 0.82, 1.0))

    body_shape = (
        cq.Workplane("XY")
        .box(0.150, 0.065, 0.072)
        .edges("|Z")
        .fillet(0.007)
    )
    top_hump = (
        cq.Workplane("XY")
        .box(0.075, 0.034, 0.016)
        .edges("|Z")
        .fillet(0.004)
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shape, "main_body"),
        material=graphite,
        name="main_body",
    )
    body.visual(
        mesh_from_cadquery(top_hump, "top_hump"),
        origin=Origin(xyz=(0.006, 0.000, 0.044)),
        material=graphite,
        name="top_hump",
    )

    # Right-side hand strap and end anchors, mounted to the camera's right side.
    body.visual(
        Box((0.104, 0.006, 0.018)),
        origin=Origin(xyz=(0.000, -0.049, -0.006)),
        material=rubber,
        name="hand_strap",
    )
    for idx, x in enumerate((-0.056, 0.056)):
        body.visual(
            Box((0.018, 0.018, 0.028)),
            origin=Origin(xyz=(x, -0.039, -0.006)),
            material=dark,
            name=f"strap_anchor_{idx}",
        )

    # Cylindrical lens barrel fixed to the body; +X is the optical axis.
    cyl_x = (0.0, math.pi / 2.0, 0.0)
    body.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.085, 0.000, 0.006), rpy=cyl_x),
        material=satin,
        name="rear_lens_collar",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.066),
        origin=Origin(xyz=(0.125, 0.000, 0.006), rpy=cyl_x),
        material=dark,
        name="inner_lens_tube",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.160, 0.000, 0.006), rpy=cyl_x),
        material=satin,
        name="front_lens_rim",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.002),
        origin=Origin(xyz=(0.164, 0.000, 0.006), rpy=cyl_x),
        material=glass,
        name="front_glass",
    )

    # Rear eyepiece block gives the camcorder a clear back end.
    body.visual(
        Box((0.006, 0.030, 0.022)),
        origin=Origin(xyz=(-0.078, 0.000, 0.004)),
        material=dark,
        name="rear_eyepiece",
    )

    # Body-side LCD hinge knuckles and a narrow hinge land on the left side.
    hinge_x = -0.060
    hinge_y = 0.039
    body.visual(
        Box((0.010, 0.003, 0.064)),
        origin=Origin(xyz=(hinge_x, 0.034, 0.000)),
        material=graphite,
        name="lcd_hinge_land",
    )
    for idx, z in enumerate((-0.0245, 0.0245)):
        body.visual(
            Cylinder(radius=0.0035, length=0.015),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=graphite,
            name=f"lcd_hinge_knuckle_{idx}",
        )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_cadquery(_zoom_ring_shape(), "rubber_ring", tolerance=0.0006),
        material=rubber,
        name="rubber_ring",
    )

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(),
        material=dark,
        name="hinge_barrel",
    )
    lcd_panel.visual(
        Box((0.085, 0.006, 0.055)),
        origin=Origin(xyz=(0.044, 0.004, 0.000)),
        material=graphite,
        name="panel_shell",
    )
    lcd_panel.visual(
        Box((0.071, 0.001, 0.043)),
        origin=Origin(xyz=(0.047, 0.0075, 0.000)),
        material=screen,
        name="screen_glass",
    )
    lcd_panel.visual(
        Box((0.018, 0.001, 0.004)),
        origin=Origin(xyz=(0.012, 0.0077, -0.021)),
        material=white,
        name="lcd_logo",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=dark,
        name="dial_cap",
    )
    mode_dial.visual(
        Box((0.002, 0.010, 0.001)),
        origin=Origin(xyz=(0.000, 0.006, 0.0105)),
        material=white,
        name="mode_mark",
    )

    model.articulation(
        "body_to_zoom_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=zoom_ring,
        origin=Origin(xyz=(0.108, 0.000, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "body_to_lcd_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_panel,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_mode_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.056, -0.020, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-2.6, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lcd = object_model.get_part("lcd_panel")
    zoom = object_model.get_part("zoom_ring")
    dial = object_model.get_part("mode_dial")
    lcd_joint = object_model.get_articulation("body_to_lcd_panel")
    zoom_joint = object_model.get_articulation("body_to_zoom_ring")
    dial_joint = object_model.get_articulation("body_to_mode_dial")

    ctx.check(
        "zoom ring uses continuous optical-axis rotation",
        zoom_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in zoom_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={zoom_joint.articulation_type}, axis={zoom_joint.axis}",
    )
    ctx.check(
        "lcd hinge is vertical and side mounted",
        tuple(round(v, 3) for v in lcd_joint.axis) == (0.0, 0.0, 1.0)
        and lcd_joint.motion_limits is not None
        and lcd_joint.motion_limits.upper >= 1.5,
        details=f"axis={lcd_joint.axis}, limits={lcd_joint.motion_limits}",
    )
    ctx.check(
        "mode dial rotates around vertical top axis",
        tuple(round(v, 3) for v in dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={dial_joint.axis}",
    )

    with ctx.pose({lcd_joint: 0.0}):
        ctx.expect_gap(
            lcd,
            body,
            axis="y",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="panel_shell",
            negative_elem="main_body",
            name="closed lcd panel sits just outside left body side",
        )
        closed_aabb = ctx.part_element_world_aabb(lcd, elem="panel_shell")

    with ctx.pose({lcd_joint: 1.35}):
        opened_aabb = ctx.part_element_world_aabb(lcd, elem="panel_shell")
        ctx.check(
            "flip-out lcd swings outward to the left",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][1] > closed_aabb[1][1] + 0.045,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    ctx.expect_overlap(
        zoom,
        body,
        axes="x",
        min_overlap=0.020,
        elem_a="rubber_ring",
        elem_b="inner_lens_tube",
        name="zoom ring surrounds the lens tube along the optical axis",
    )
    ctx.expect_within(
        body,
        zoom,
        axes="yz",
        margin=0.004,
        inner_elem="inner_lens_tube",
        outer_elem="rubber_ring",
        name="lens tube is centered inside the zoom ring envelope",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="dial_cap",
        negative_elem="main_body",
        name="mode dial is seated on the top body surface",
    )

    return ctx.report()


object_model = build_object_model()
