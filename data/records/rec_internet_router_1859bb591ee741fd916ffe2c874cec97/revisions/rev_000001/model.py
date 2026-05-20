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


BODY_W = 0.220
BODY_D = 0.140
BODY_H = 0.035
BACK_Y = BODY_D / 2.0
SIDE_X = BODY_W / 2.0


def _router_body_shell() -> cq.Workplane:
    """Rounded slab with a real side pocket for the rocker switch."""
    recess_y = -0.030
    recess_w = 0.050
    recess_h = 0.026
    recess_depth = 0.008

    shell = cq.Workplane("XY").rect(BODY_W, BODY_D).extrude(BODY_H)
    shell = shell.edges("|Z").fillet(0.018)
    shell = shell.edges(">Z").fillet(0.004)
    shell = shell.edges("<Z").fillet(0.002)

    cutter = (
        cq.Workplane("XY")
        .box(recess_depth + 0.010, recess_w, recess_h)
        .translate((SIDE_X - recess_depth / 2.0 + 0.005, recess_y, 0.020))
    )
    return shell.cut(cutter)


def _rounded_paddle() -> cq.Workplane:
    """One flat fold-out antenna paddle, authored in its hinge-local frame."""
    width = 0.038
    length = 0.165
    thickness = 0.007
    root_y = 0.032
    z_min = 0.0025

    paddle = (
        cq.Workplane("XY")
        .rect(width, length)
        .extrude(thickness)
        .translate((0.0, root_y + length / 2.0, z_min))
    )
    paddle = paddle.edges("|Z").fillet(0.007)
    paddle = paddle.edges(">Z").fillet(0.0015)
    return paddle


def _rocker_cap() -> cq.Workplane:
    """Small beveled rocker cap in the switch-local pivot frame."""
    cap = cq.Workplane("YZ").rect(0.026, 0.015).extrude(0.006)
    cap = cap.edges().fillet(0.0012)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_home_router")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.055, 0.058, 0.065, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("soft_black", rgba=(0.005, 0.005, 0.006, 1.0))
    green_led = model.material("green_led", rgba=(0.10, 0.95, 0.28, 1.0))
    blue_led = model.material("blue_led", rgba=(0.12, 0.38, 1.0, 1.0))
    white_mark = model.material("white_mark", rgba=(0.92, 0.92, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_router_body_shell(), "rounded_router_slab", tolerance=0.0006),
        material=matte_black,
        name="rounded_slab",
    )

    # A darker bottom band and small top details make the slab read like a home router.
    body.visual(
        Box((0.205, 0.0016, 0.006)),
        origin=Origin(xyz=(0.0, -BACK_Y - 0.0008, 0.020)),
        material=graphite,
        name="front_led_strip",
    )
    for i, x in enumerate((-0.050, -0.025, 0.000, 0.025)):
        body.visual(
            Box((0.008, 0.0012, 0.0032)),
            origin=Origin(xyz=(x, -BACK_Y - 0.0016, 0.020)),
            material=green_led if i != 3 else blue_led,
            name=f"status_light_{i}",
        )

    for i, x in enumerate((-0.045, -0.025, -0.005, 0.015, 0.035, 0.055)):
        body.visual(
            Box((0.010, 0.040, 0.0012)),
            origin=Origin(xyz=(x, 0.010, BODY_H + 0.0006)),
            material=dark_gray,
            name=f"top_vent_{i}",
        )

    # The side recess is cut into the body mesh; this dark plate is the back wall.
    body.visual(
        Box((0.0012, 0.044, 0.022)),
        origin=Origin(xyz=(SIDE_X - 0.0086, -0.030, 0.020)),
        material=dark_gray,
        name="switch_recess",
    )

    hinge_xs = (-0.055, 0.055)
    for idx, x in enumerate(hinge_xs):
        body.visual(
            Box((0.046, 0.022, 0.010)),
            origin=Origin(xyz=(x, BACK_Y + 0.006, BODY_H + 0.005)),
            material=graphite,
            name=f"pod_base_{idx}",
        )
        for side, dx in enumerate((-0.019, 0.019)):
            body.visual(
                Box((0.006, 0.014, 0.018)),
                origin=Origin(xyz=(x + dx, BACK_Y + 0.007, BODY_H + 0.014)),
                material=graphite,
                name=f"pod_cheek_{idx}_{side}",
            )

    antennas = []
    for idx, x in enumerate(hinge_xs):
        antenna = model.part(f"antenna_{idx}")
        antenna.visual(
            Cylinder(radius=0.008, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="root_barrel",
        )
        antenna.visual(
            Box((0.018, 0.034, 0.010)),
            origin=Origin(xyz=(0.0, 0.021, 0.006)),
            material=rubber,
            name="root_neck",
        )
        antenna.visual(
            mesh_from_cadquery(_rounded_paddle(), f"antenna_paddle_{idx}", tolerance=0.0005),
            material=rubber,
            name="paddle",
        )
        model.articulation(
            f"body_to_antenna_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x, BACK_Y + 0.007, BODY_H + 0.018)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.75),
        )
        antennas.append(antenna)

    power_switch = model.part("power_switch")
    power_switch.visual(
        mesh_from_cadquery(_rocker_cap(), "rocker_cap", tolerance=0.00035),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="rocker_cap",
    )
    power_switch.visual(
        Cylinder(radius=0.0022, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="pivot_pin",
    )
    power_switch.visual(
        Box((0.0008, 0.0035, 0.007)),
        origin=Origin(xyz=(0.0064, -0.006, 0.0030)),
        material=white_mark,
        name="power_mark",
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(SIDE_X - 0.0058, -0.030, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.5, lower=-0.28, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    switch = object_model.get_part("power_switch")
    switch_joint = object_model.get_articulation("body_to_power_switch")

    ctx.expect_within(
        switch,
        body,
        axes="yz",
        inner_elem="rocker_cap",
        outer_elem="switch_recess",
        margin=0.001,
        name="rocker cap stays inside the side recess opening",
    )
    ctx.expect_gap(
        switch,
        body,
        axis="x",
        positive_elem="rocker_cap",
        negative_elem="switch_recess",
        min_gap=0.0005,
        max_gap=0.004,
        name="rocker cap is proud of the recessed back wall",
    )

    rest_switch_aabb = ctx.part_world_aabb(switch)
    with ctx.pose({switch_joint: 0.28}):
        pressed_switch_aabb = ctx.part_world_aabb(switch)
    ctx.check(
        "rocker switch pivots outward at one end",
        rest_switch_aabb is not None
        and pressed_switch_aabb is not None
        and pressed_switch_aabb[1][0] > rest_switch_aabb[1][0] + 0.0008,
        details=f"rest={rest_switch_aabb}, pressed={pressed_switch_aabb}",
    )

    for idx in (0, 1):
        antenna = object_model.get_part(f"antenna_{idx}")
        hinge = object_model.get_articulation(f"body_to_antenna_{idx}")
        ctx.expect_gap(
            antenna,
            body,
            axis="z",
            positive_elem="root_barrel",
            negative_elem=f"pod_base_{idx}",
            min_gap=-0.0005,
            max_gap=0.0015,
            name=f"antenna {idx} barrel sits on its hinge pod",
        )
        folded_aabb = ctx.part_world_aabb(antenna)
        with ctx.pose({hinge: 1.45}):
            raised_aabb = ctx.part_world_aabb(antenna)
            ctx.expect_gap(
                antenna,
                body,
                axis="z",
                positive_elem="paddle",
                negative_elem="rounded_slab",
                min_gap=0.005,
                name=f"raised antenna {idx} clears the router body",
            )
        ctx.check(
            f"antenna {idx} folds upward on a horizontal hinge",
            folded_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > folded_aabb[1][2] + 0.10,
            details=f"folded={folded_aabb}, raised={raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
