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


MM = 0.001


def _body_shell_mm() -> cq.Workplane:
    """Rounded compact plastic flash-drive body, authored in millimetres."""
    shell = (
        cq.Workplane("XY")
        .box(56.0, 16.0, 6.0)
        .edges()
        .fillet(1.2)
        .translate((10.0, 0.0, 0.0))
    )
    # A shallow top recess gives the molded plastic body a real product finish.
    recess = cq.Workplane("XY").box(29.0, 10.4, 0.34).translate((12.5, 0.0, 3.03))
    return shell.cut(recess)


def _usb_shell_mm() -> cq.Workplane:
    """Thin hollow USB-A style plug shell with top retention holes."""
    outer = cq.Workplane("XY").box(14.4, 12.0, 4.4).translate((44.8, 0.0, 0.0))
    cavity = cq.Workplane("XY").box(16.4, 10.45, 3.05).translate((45.2, 0.0, -0.12))
    shell = outer.cut(cavity)
    for y in (-3.05, 3.05):
        hole = cq.Workplane("XY").box(2.4, 1.9, 1.4).translate((45.6, y, 1.95))
        shell = shell.cut(hole)
    return shell.edges().fillet(0.16)


def _cover_shell_mm() -> cq.Workplane:
    """One-piece pierced metal swivel cover, with a real hollow clearance."""
    outer = (
        cq.Workplane("XY")
        .box(72.0, 24.0, 11.0)
        .translate((23.0, 0.0, 0.0))
        .edges()
        .fillet(1.05)
    )
    # Through-clearance for the plastic body and USB plug; this makes the cover
    # a true protective sleeve rather than a solid block occupying the body.
    inner_clearance = cq.Workplane("XY").box(76.0, 19.0, 8.4).translate((24.0, 0.0, 0.0))
    cover = outer.cut(inner_clearance)

    # Pivot hole through both metal skins for the rivet shank.
    pivot_hole = cq.Workplane("XY").circle(3.75).extrude(18.0).translate((0.0, 0.0, -9.0))
    cover = cover.cut(pivot_hole)

    # Small front mouth bevels keep the sleeve from reading as a raw tube.
    mouth_notch = cq.Workplane("XY").box(5.5, 15.0, 14.0).translate((59.0, 0.0, 0.0))
    return cover.cut(mouth_notch).edges().fillet(0.22)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_usb_flash_drive")

    satin_metal = model.material("satin_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("dark_etched_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.025, 0.028, 0.032, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.005, 0.006, 0.007, 1.0))
    label = model.material("charcoal_label", rgba=(0.13, 0.14, 0.15, 1.0))
    led_green = model.material("tiny_green_lens", rgba=(0.05, 0.85, 0.35, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_mm(), "body_shell", unit_scale=MM, tolerance=0.08),
        material=graphite,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_usb_shell_mm(), "usb_shell", unit_scale=MM, tolerance=0.04),
        material=satin_metal,
        name="usb_shell",
    )
    body.visual(
        Box((0.0124, 0.0071, 0.0008)),
        origin=Origin(xyz=(0.0434, 0.0, -0.00055)),
        material=graphite,
        name="plug_tongue",
    )
    body.visual(
        Box((0.0272, 0.0094, 0.00022)),
        origin=Origin(xyz=(0.0125, 0.0, 0.00305)),
        material=label,
        name="top_label",
    )
    body.visual(
        Box((0.026, 0.00055, 0.0032)),
        origin=Origin(xyz=(0.011, 0.00825, 0.0)),
        material=rubber,
        name="side_grip_0",
    )
    body.visual(
        Box((0.026, 0.00055, 0.0032)),
        origin=Origin(xyz=(0.011, -0.00825, 0.0)),
        material=rubber,
        name="side_grip_1",
    )
    body.visual(
        Cylinder(radius=0.00135, length=0.00032),
        origin=Origin(xyz=(0.024, -0.0045, 0.00328)),
        material=led_green,
        name="status_lens",
    )

    # The visible pivot is carried by the inner body and captures the cover.
    body.visual(
        Cylinder(radius=0.00255, length=0.0122),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_shank",
    )
    body.visual(
        Cylinder(radius=0.00515, length=0.00118),
        origin=Origin(xyz=(0.0, 0.0, 0.00609)),
        material=satin_metal,
        name="pivot_head_top",
    )
    body.visual(
        Cylinder(radius=0.00515, length=0.00118),
        origin=Origin(xyz=(0.0, 0.0, -0.00609)),
        material=satin_metal,
        name="pivot_head_bottom",
    )
    body.visual(
        Box((0.0067, 0.0011, 0.00024)),
        origin=Origin(xyz=(0.0, 0.0, 0.00658)),
        material=dark_metal,
        name="screw_slot",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shell_mm(), "cover_shell", unit_scale=MM, tolerance=0.08),
        material=satin_metal,
        name="cover_shell",
    )
    for i, y in enumerate((-0.0065, -0.0022, 0.0022, 0.0065)):
        cover.visual(
            Box((0.039, 0.00028, 0.00010)),
            origin=Origin(xyz=(0.0275, y, 0.00547)),
            material=dark_metal,
            name=f"brush_line_{i}",
        )
    cover.visual(
        Box((0.010, 0.0013, 0.00012)),
        origin=Origin(xyz=(0.044, 0.0, 0.00548)),
        material=dark_metal,
        name="cover_mark",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=0.0, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_shell",
        elem_b="usb_shell",
        min_overlap=0.010,
        name="closed metal cover shields the USB plug",
    )
    ctx.expect_within(
        body,
        cover,
        axes="y",
        inner_elem="usb_shell",
        outer_elem="cover_shell",
        margin=0.001,
        name="USB plug sits inside the closed cover width",
    )
    ctx.expect_within(
        body,
        cover,
        axes="xy",
        inner_elem="pivot_shank",
        outer_elem="cover_shell",
        margin=0.002,
        name="pivot shank is centered in the cover eyelet",
    )

    with ctx.pose({swivel: math.pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="usb_shell",
            negative_elem="cover_shell",
            min_gap=0.020,
            name="rotated cover clears and exposes the USB plug",
        )

    return ctx.report()


object_model = build_object_model()
