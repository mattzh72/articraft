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


def _lens_barrel_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").box(0.018, 0.058, 0.058).translate((0.009, 0.0, 0.0))
    barrel = (
        cq.Workplane("YZ")
        .circle(0.0262)
        .extrude(0.104)
        .cut(cq.Workplane("YZ").circle(0.0218).extrude(0.104))
    )
    hood = (
        cq.Workplane("YZ")
        .circle(0.0305)
        .circle(0.0240)
        .extrude(0.018)
        .translate((0.104, 0.0, 0.0))
    )
    front_bezel = (
        cq.Workplane("YZ")
        .circle(0.033)
        .circle(0.0255)
        .extrude(0.006)
        .translate((0.122, 0.0, 0.0))
    )
    return collar.union(barrel).union(hood).union(front_bezel)


def _focus_ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(0.0298)
        .extrude(0.018)
        .translate((-0.009, 0.0, 0.0))
        .cut(
            cq.Workplane("YZ")
            .circle(0.0260)
            .extrude(0.022)
            .translate((-0.011, 0.0, 0.0))
        )
    )
    for angle_deg in range(0, 360, 24):
        grip = (
            cq.Workplane("XY")
            .box(0.018, 0.0028, 0.0055)
            .translate((0.0, 0.0292, 0.0))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        )
        ring = ring.union(grip)
    return ring


def _main_shell_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .rect(0.078, 0.082)
        .workplane(offset=0.050)
        .rect(0.076, 0.082)
        .workplane(offset=0.040)
        .rect(0.070, 0.078)
        .workplane(offset=0.030)
        .rect(0.060, 0.070)
        .loft(combine=True)
    )


def _grip_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(0.066, 0.022)
        .workplane(offset=0.038)
        .center(0.003, 0.0)
        .rect(0.060, 0.020)
        .workplane(offset=0.048)
        .center(0.010, 0.0)
        .rect(0.042, 0.018)
        .loft(combine=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_camcorder")

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    shell_black = model.material("shell_black", rgba=(0.17, 0.17, 0.18, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.23, 0.24, 1.0))
    metal = model.material("metal", rgba=(0.54, 0.56, 0.58, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.34, 0.40, 0.45))
    screen_black = model.material("screen_black", rgba=(0.06, 0.06, 0.07, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_main_shell_shape(), "camcorder_main_shell"),
        origin=Origin(xyz=(-0.065, 0.0, 0.041)),
        material=body_black,
        name="main_shell",
    )
    body.visual(
        Box((0.074, 0.076, 0.022)),
        origin=Origin(xyz=(0.012, 0.0, 0.086)),
        material=shell_black,
        name="upper_shell",
    )
    body.visual(
        Box((0.040, 0.058, 0.068)),
        origin=Origin(xyz=(-0.078, 0.0, 0.049)),
        material=shell_black,
        name="battery_pack",
    )
    body.visual(
        Box((0.022, 0.060, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, 0.044)),
        material=body_black,
        name="front_cheek",
    )
    body.visual(
        Box((0.016, 0.016, 0.030)),
        origin=Origin(xyz=(0.050, 0.0, 0.093)),
        material=body_black,
        name="handle_front_post",
    )
    body.visual(
        Box((0.016, 0.016, 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, 0.093)),
        material=body_black,
        name="handle_rear_post",
    )
    body.visual(
        Box((0.084, 0.016, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.114)),
        material=body_black,
        name="handle_bridge",
    )
    body.visual(
        mesh_from_cadquery(_grip_shape(), "camcorder_grip_shell"),
        origin=Origin(xyz=(0.000, -0.048, 0.002)),
        material=rubber_black,
        name="grip_shell",
    )
    body.visual(
        Box((0.052, 0.004, 0.056)),
        origin=Origin(xyz=(0.034, -0.059, 0.047)),
        material=dark_grey,
        name="hand_strap",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(-0.096, 0.0, 0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screen_black,
        name="eyepiece_tube",
    )
    body.visual(
        Box((0.012, 0.026, 0.018)),
        origin=Origin(xyz=(-0.108, 0.0, 0.068)),
        material=rubber_black,
        name="eyecup",
    )
    body.visual(
        Box((0.008, 0.006, 0.014)),
        origin=Origin(xyz=(-0.055, 0.034, 0.062)),
        material=body_black,
        name="monitor_hinge_upper",
    )
    body.visual(
        Box((0.008, 0.006, 0.014)),
        origin=Origin(xyz=(-0.055, 0.034, 0.026)),
        material=body_black,
        name="monitor_hinge_lower",
    )
    body.visual(
        Box((0.010, 0.008, 0.054)),
        origin=Origin(xyz=(-0.055, 0.031, 0.044)),
        material=body_black,
        name="monitor_hinge_plate",
    )
    body.visual(
        Box((0.020, 0.016, 0.016)),
        origin=Origin(xyz=(-0.045, -0.016, 0.090)),
        material=dark_grey,
        name="dial_plinth",
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        mesh_from_cadquery(_lens_barrel_shape(), "camcorder_lens_barrel"),
        material=shell_black,
        name="barrel_shell",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_focus_ring_shape(), "camcorder_focus_ring"),
        material=rubber_black,
        name="focus_ring",
    )

    monitor = model.part("monitor")
    monitor.visual(
        Cylinder(radius=0.0032, length=0.022),
        origin=Origin(xyz=(0.0, 0.0010, 0.044)),
        material=metal,
        name="monitor_hinge",
    )
    monitor.visual(
        Box((0.010, 0.006, 0.050)),
        origin=Origin(xyz=(0.005, 0.0030, 0.044)),
        material=screen_black,
        name="monitor_arm",
    )
    monitor.visual(
        Box((0.074, 0.007, 0.050)),
        origin=Origin(xyz=(0.037, 0.0035, 0.044)),
        material=screen_black,
        name="monitor_panel",
    )
    monitor.visual(
        Box((0.058, 0.0016, 0.040)),
        origin=Origin(xyz=(0.039, 0.0011, 0.044)),
        material=glass,
        name="screen_glass",
    )
    monitor.visual(
        Box((0.070, 0.0015, 0.046)),
        origin=Origin(xyz=(0.037, 0.0063, 0.044)),
        material=dark_grey,
        name="monitor_back",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="dial_spindle",
    )
    mode_dial.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_grey,
        name="dial_body",
    )
    mode_dial.visual(
        Cylinder(radius=0.015, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=body_black,
        name="dial_cap",
    )
    mode_dial.visual(
        Box((0.010, 0.0025, 0.0015)),
        origin=Origin(xyz=(0.006, 0.0, 0.0108)),
        material=metal,
        name="dial_pointer",
    )

    model.articulation(
        "body_to_lens_barrel",
        ArticulationType.FIXED,
        parent=body,
        child=lens_barrel,
        origin=Origin(xyz=(0.071, 0.0, 0.044)),
    )
    model.articulation(
        "lens_barrel_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=12.0),
    )
    model.articulation(
        "body_to_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(-0.055, 0.0405, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "body_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.045, -0.016, 0.0975)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lens_barrel = object_model.get_part("lens_barrel")
    focus_ring = object_model.get_part("focus_ring")
    monitor = object_model.get_part("monitor")
    mode_dial = object_model.get_part("mode_dial")

    monitor_hinge = object_model.get_articulation("body_to_monitor")
    dial_joint = object_model.get_articulation("body_to_mode_dial")

    ctx.expect_gap(
        monitor,
        body,
        axis="y",
        positive_elem="monitor_panel",
        negative_elem="main_shell",
        max_gap=0.010,
        max_penetration=0.0,
        name="monitor panel sits close to the body side when closed",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="xz",
        elem_a="monitor_panel",
        elem_b="main_shell",
        min_overlap=0.040,
        name="closed monitor covers the camcorder side wall",
    )
    ctx.expect_origin_distance(
        focus_ring,
        lens_barrel,
        axes="yz",
        max_dist=0.001,
        name="focus ring stays concentric with the lens barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="x",
        elem_a="focus_ring",
        elem_b="barrel_shell",
        min_overlap=0.012,
        name="focus ring wraps a retained length around the lens barrel",
    )
    ctx.allow_overlap(
        focus_ring,
        lens_barrel,
        elem_a="focus_ring",
        elem_b="barrel_shell",
        reason="The focus ring is intentionally represented as a snug rotating sleeve nested over the fixed lens barrel.",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_body",
        negative_elem="upper_shell",
        max_gap=0.006,
        max_penetration=0.0,
        name="mode dial sits just above the top shell",
    )

    closed_glass = ctx.part_element_world_aabb(monitor, elem="screen_glass")
    with ctx.pose({monitor_hinge: math.radians(95.0)}):
        open_glass = ctx.part_element_world_aabb(monitor, elem="screen_glass")
    ctx.check(
        "monitor swings outward from the side hinge",
        closed_glass is not None
        and open_glass is not None
        and open_glass[1][1] > closed_glass[1][1] + 0.045,
        details=f"closed={closed_glass}, open={open_glass}",
    )

    dial_rest = ctx.part_element_world_aabb(mode_dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.radians(60.0)}):
        dial_turned = ctx.part_element_world_aabb(mode_dial, elem="dial_pointer")
    ctx.check(
        "mode dial pointer rotates around the top axis",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_turned[1][0] - dial_rest[1][0]) > 0.001,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    return ctx.report()


object_model = build_object_model()
