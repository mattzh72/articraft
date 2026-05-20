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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_LEN = 0.125
BODY_W = 0.050
BODY_H = 0.066
LENS_Z = 0.040
SCREEN_W = 0.072
SCREEN_H = 0.048
SCREEN_T = 0.007
SPINE_R = 0.004


def _build_body_mesh():
    body = (
        cq.Workplane("XY")
        .box(BODY_LEN, BODY_W, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.004)
    )

    right_grip = (
        cq.Workplane("XY")
        .box(0.070, 0.018, 0.050, centered=(True, True, False))
        .translate((0.004, -BODY_W * 0.5 - 0.005, 0.008))
    )

    front_post = (
        cq.Workplane("XY")
        .box(0.012, 0.016, 0.014, centered=(True, True, False))
        .translate((0.022, 0.0, BODY_H))
    )
    rear_post = (
        cq.Workplane("XY")
        .box(0.012, 0.016, 0.014, centered=(True, True, False))
        .translate((-0.020, 0.0, BODY_H))
    )
    handle_bridge = (
        cq.Workplane("XY")
        .box(0.064, 0.018, 0.010, centered=(True, True, False))
        .translate((0.001, 0.0, BODY_H + 0.014))
    )

    hinge_mount = (
        cq.Workplane("XY")
        .box(0.014, 0.004, 0.040, centered=(True, True, False))
        .translate((0.022, BODY_W * 0.5 + 0.0005, 0.017))
    )

    eyepiece = (
        cq.Workplane("XY")
        .box(0.020, 0.022, 0.016, centered=(True, True, False))
        .translate((-BODY_LEN * 0.5 - 0.004, 0.0, 0.040))
    )

    return body.union(right_grip).union(front_post).union(rear_post).union(handle_bridge).union(
        hinge_mount
    ).union(eyepiece)


def _build_lens_mesh():
    rear_mount = cq.Workplane("YZ").circle(0.0185).extrude(0.014)
    barrel_mid = cq.Workplane("YZ").circle(0.0205).extrude(0.030).translate((0.014, 0.0, 0.0))
    hood = cq.Workplane("YZ").circle(0.0240).extrude(0.024).translate((0.044, 0.0, 0.0))
    front_lip = cq.Workplane("YZ").circle(0.0260).extrude(0.004).translate((0.068, 0.0, 0.0))
    bore = cq.Workplane("YZ").circle(0.0145).extrude(0.070).translate((0.010, 0.0, 0.0))
    return rear_mount.union(barrel_mid).union(hood).union(front_lip).cut(bore)


def _build_focus_ring_mesh():
    outer = cq.Workplane("YZ").circle(0.0245).extrude(0.006, both=True)
    inner = cq.Workplane("YZ").circle(0.0205).extrude(0.007, both=True)
    front_rib = cq.Workplane("YZ").circle(0.0250).extrude(0.0012, both=True).translate((0.0030, 0.0, 0.0))
    rear_rib = cq.Workplane("YZ").circle(0.0250).extrude(0.0012, both=True).translate((-0.0030, 0.0, 0.0))
    return outer.cut(inner).union(front_rib.cut(inner)).union(rear_rib.cut(inner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_camcorder")

    shell = model.material("shell", rgba=(0.13, 0.14, 0.15, 1.0))
    trim = model.material("trim", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.20, 0.24, 0.40))
    accent = model.material("accent", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_mesh(), "camcorder_body"),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.002),
        origin=Origin(xyz=(-0.046, 0.0, BODY_H + 0.001)),
        material=trim,
        name="dial_pad",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.155, 0.064, 0.092)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        mesh_from_cadquery(_build_lens_mesh(), "camcorder_lens_barrel"),
        material=trim,
        name="barrel_shell",
    )
    lens_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.072),
        mass=0.12,
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_build_focus_ring_mesh(), "camcorder_focus_ring"),
        material=rubber,
        name="ring_shell",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.012),
        mass=0.02,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    screen = model.part("screen")
    screen.visual(
        Cylinder(radius=SPINE_R, length=SCREEN_H + 0.004),
        origin=Origin(xyz=(0.0, 0.0030, 0.0)),
        material=trim,
        name="hinge_spine",
    )
    screen.visual(
        Box((0.008, SCREEN_T, SCREEN_H)),
        origin=Origin(xyz=(-0.004, SCREEN_T * 0.5, 0.0)),
        material=trim,
        name="hinge_block",
    )
    screen.visual(
        Box((SCREEN_W, SCREEN_T, SCREEN_H)),
        origin=Origin(xyz=(-(SCREEN_W * 0.5 + 0.008), SCREEN_T * 0.5, 0.0)),
        material=shell,
        name="screen_shell",
    )
    screen.visual(
        Box((0.054, 0.0015, 0.031)),
        origin=Origin(xyz=(-0.042, 0.0010, 0.0)),
        material=glass,
        name="display",
    )
    screen.inertial = Inertial.from_geometry(
        Box((SCREEN_W + 0.008, SCREEN_T, SCREEN_H)),
        mass=0.10,
        origin=Origin(xyz=(-0.040, SCREEN_T * 0.5, 0.0)),
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim,
        name="dial_spindle",
    )
    mode_dial.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=accent,
        name="dial_shell",
    )
    mode_dial.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=trim,
        name="dial_cap",
    )
    mode_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.008),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_lens_barrel",
        ArticulationType.FIXED,
        parent=body,
        child=lens_barrel,
        origin=Origin(xyz=(BODY_LEN * 0.5, 0.0, LENS_Z)),
    )
    model.articulation(
        "lens_barrel_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "body_to_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(0.024, BODY_W * 0.5 + SPINE_R - 0.0005, 0.037)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "body_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.046, 0.0, BODY_H + 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.1, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lens_barrel = object_model.get_part("lens_barrel")
    focus_ring = object_model.get_part("focus_ring")
    screen = object_model.get_part("screen")
    mode_dial = object_model.get_part("mode_dial")

    screen_hinge = object_model.get_articulation("body_to_screen")

    ctx.allow_overlap(
        focus_ring,
        lens_barrel,
        elem_a="ring_shell",
        elem_b="barrel_shell",
        reason="The focus ring is intentionally represented as a sleeve rotating around the lens barrel.",
    )

    ctx.expect_gap(
        screen,
        body,
        axis="y",
        positive_elem="screen_shell",
        negative_elem="body_shell",
        max_gap=0.008,
        max_penetration=0.0,
        name="screen rests close to the left side of the body",
    )
    ctx.expect_overlap(
        screen,
        body,
        axes="xz",
        elem_a="screen_shell",
        elem_b="body_shell",
        min_overlap=0.030,
        name="closed screen overlaps the body side footprint",
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="yz",
        elem_a="ring_shell",
        elem_b="barrel_shell",
        min_overlap=0.040,
        name="focus ring stays concentric with the lens barrel",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_spindle",
        negative_elem="dial_pad",
        max_gap=0.0,
        max_penetration=0.0,
        name="mode dial spindle seats on the top shell pad",
    )
    ctx.expect_overlap(
        mode_dial,
        body,
        axes="xy",
        elem_a="dial_shell",
        elem_b="dial_pad",
        min_overlap=0.018,
        name="mode dial remains centered on the top shell",
    )

    closed_aabb = ctx.part_element_world_aabb(screen, elem="screen_shell")
    with ctx.pose({screen_hinge: math.radians(105.0)}):
        ctx.expect_gap(
            screen,
            body,
            axis="y",
            positive_elem="screen_shell",
            negative_elem="body_shell",
            min_gap=0.006,
            name="open screen swings outward from the body",
        )
        open_aabb = ctx.part_element_world_aabb(screen, elem="screen_shell")

    screen_swung_out = False
    if closed_aabb is not None and open_aabb is not None:
        closed_max_y = float(closed_aabb[1][1])
        open_max_y = float(open_aabb[1][1])
        screen_swung_out = open_max_y > closed_max_y + 0.030

    ctx.check(
        "screen free edge moves outward when opened",
        screen_swung_out,
        details=f"closed_aabb={closed_aabb!r}, open_aabb={open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
