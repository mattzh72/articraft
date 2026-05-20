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


BASE_W = 0.158
BASE_D = 0.138
BASE_H = 0.056
SEAT_R = 0.029
SEAT_H = 0.012
COLLAR_H = 0.018


def make_base_body() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_H)
        .translate((0.0, 0.0, BASE_H / 2.0))
        .edges("|Z")
        .fillet(0.016)
        .edges(">Z")
        .fillet(0.010)
    )
    switch_recess = (
        cq.Workplane("XY")
        .box(0.050, 0.012, 0.024)
        .translate((0.0, -BASE_D / 2.0 + 0.003, 0.024))
    )
    top_relief = (
        cq.Workplane("XY")
        .circle(0.039)
        .extrude(0.003)
        .translate((0.0, 0.0, BASE_H - 0.001))
    )
    return body.cut(switch_recess).union(top_relief)


def make_bottle_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.041)
        .workplane(offset=0.110)
        .circle(0.038)
        .workplane(offset=0.070)
        .circle(0.031)
        .workplane(offset=0.026)
        .circle(0.027)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(0.038)
        .workplane(offset=0.113)
        .circle(0.035)
        .workplane(offset=0.070)
        .circle(0.028)
        .workplane(offset=0.029)
        .circle(0.024)
        .loft(combine=True)
    )
    return outer.cut(inner).translate((0.0, 0.0, COLLAR_H - 0.0010))


def make_bottle_collar() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(0.049)
        .circle(0.031)
        .extrude(COLLAR_H)
    )
    lock_tab = (
        cq.Workplane("XY")
        .box(0.018, 0.010, 0.008)
        .translate((0.0, -0.047, 0.007))
    )
    return collar.union(lock_tab)


def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_blender")

    body_color = model.material("body", rgba=(0.20, 0.21, 0.23, 1.0))
    trim_color = model.material("trim", rgba=(0.10, 0.11, 0.12, 1.0))
    switch_color = model.material("switch", rgba=(0.86, 0.87, 0.89, 1.0))
    steel_color = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    jar_color = model.material("jar", rgba=(0.80, 0.87, 0.92, 0.35))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_body(), "base_body"),
        material=body_color,
        name="body",
    )
    base.visual(
        Cylinder(radius=SEAT_R, length=SEAT_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + SEAT_H / 2.0 - 0.0005)),
        material=trim_color,
        name="seat",
    )
    base.visual(
        Box((0.050, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.065, 0.024)),
        material=trim_color,
        name="switch_mount",
    )

    switch = model.part("switch")
    switch.visual(
        Cylinder(radius=0.003, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="hinge_barrel",
    )
    switch.visual(
        Box((0.044, 0.011, 0.022)),
        origin=Origin(xyz=(0.0, -0.008, 0.013), rpy=(0.08, 0.0, 0.0)),
        material=switch_color,
        name="paddle",
    )
    switch.visual(
        Box((0.034, 0.0035, 0.008)),
        origin=Origin(xyz=(0.0, 0.001, 0.004)),
        material=trim_color,
        name="pivot_tongue",
    )

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(make_bottle_shell(), "bottle_shell"),
        material=jar_color,
        name="shell",
    )
    bottle.visual(
        mesh_from_cadquery(make_bottle_collar(), "bottle_collar"),
        material=trim_color,
        name="collar",
    )
    bottle.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=trim_color,
        name="bearing",
    )
    bottle.visual(
        Box((0.027, 0.003, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.020)),
        material=trim_color,
        name="bearing_arm_0",
    )
    bottle.visual(
        Box((0.027, 0.003, 0.004)),
        origin=Origin(xyz=(-0.018, 0.0, 0.020)),
        material=trim_color,
        name="bearing_arm_1",
    )
    bottle.visual(
        Box((0.003, 0.027, 0.004)),
        origin=Origin(xyz=(0.0, 0.018, 0.020)),
        material=trim_color,
        name="bearing_arm_2",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.007, length=0.010),
        material=steel_color,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=steel_color,
        name="shaft",
    )
    blade.visual(
        Box((0.026, 0.006, 0.0018)),
        origin=Origin(xyz=(0.014, 0.0, 0.0015), rpy=(0.0, 0.22, 0.0)),
        material=steel_color,
        name="blade_tip",
    )
    blade.visual(
        Box((0.020, 0.006, 0.0018)),
        origin=Origin(xyz=(-0.011, 0.0, 0.0010), rpy=(0.0, -0.18, 0.0)),
        material=steel_color,
        name="blade_back",
    )
    blade.visual(
        Box((0.006, 0.022, 0.0016)),
        origin=Origin(xyz=(0.0, 0.012, 0.0020), rpy=(0.20, 0.0, 0.0)),
        material=steel_color,
        name="blade_cross",
    )

    model.articulation(
        "base_to_switch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=switch,
        origin=Origin(xyz=(0.0, -0.067, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.20, upper=0.20, effort=1.0, velocity=4.0),
    )
    model.articulation(
        "base_to_bottle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bottle,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + 0.0008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=2.0, velocity=2.0),
    )
    model.articulation(
        "bottle_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    switch = object_model.get_part("switch")
    bottle = object_model.get_part("bottle")
    blade = object_model.get_part("blade")

    switch_joint = object_model.get_articulation("base_to_switch")
    bottle_joint = object_model.get_articulation("base_to_bottle")
    blade_joint = object_model.get_articulation("bottle_to_blade")

    ctx.allow_overlap(
        base,
        switch,
        elem_a="switch_mount",
        elem_b="pivot_tongue",
        reason="The rocker uses a hidden pivot tongue captured inside the front switch mount.",
    )
    ctx.allow_overlap(
        bottle,
        blade,
        elem_a="bearing",
        elem_b="shaft",
        reason="The spinning blade shaft is intentionally represented as passing through the sealed bearing boss at the bottle base.",
    )

    ctx.expect_overlap(
        bottle,
        base,
        axes="xy",
        elem_a="collar",
        elem_b="seat",
        min_overlap=0.055,
        name="bottle collar stays centered over the seat",
    )
    ctx.expect_gap(
        bottle,
        base,
        axis="z",
        positive_elem="shell",
        negative_elem="body",
        min_gap=0.015,
        name="bottle body clears the low motor housing",
    )
    ctx.expect_overlap(
        switch,
        base,
        axes="xz",
        elem_a="paddle",
        elem_b="body",
        min_overlap=0.018,
        name="front rocker sits on the front panel zone",
    )
    ctx.expect_within(
        blade,
        bottle,
        axes="xy",
        inner_elem="blade_tip",
        outer_elem="shell",
        margin=0.012,
        name="blade stays inside the bottle footprint",
    )

    switch_rest = aabb_center(ctx.part_element_world_aabb(switch, elem="paddle"))
    with ctx.pose({switch_joint: 0.18}):
        switch_toggled = aabb_center(ctx.part_element_world_aabb(switch, elem="paddle"))
    ctx.check(
        "rocker switch tips outward at positive travel",
        switch_rest is not None
        and switch_toggled is not None
        and switch_toggled[1] < switch_rest[1] - 0.002,
        details=f"rest={switch_rest}, toggled={switch_toggled}",
    )

    collar_rest = ctx.part_element_world_aabb(bottle, elem="collar")
    with ctx.pose({bottle_joint: 0.45}):
        collar_turned = ctx.part_element_world_aabb(bottle, elem="collar")
        ctx.expect_overlap(
            bottle,
            base,
            axes="xy",
            elem_a="collar",
            elem_b="seat",
            min_overlap=0.055,
            name="bottle remains seated while twisting on the dock",
        )
    ctx.check(
        "twist-lock bottle rotates around the seat",
        collar_rest is not None
        and collar_turned is not None
        and collar_turned[1][0] > collar_rest[1][0] + 0.006,
        details=f"rest={collar_rest}, turned={collar_turned}",
    )

    blade_rest = aabb_center(ctx.part_element_world_aabb(blade, elem="blade_tip"))
    with ctx.pose({blade_joint: 1.20}):
        blade_spun = aabb_center(ctx.part_element_world_aabb(blade, elem="blade_tip"))
    ctx.check(
        "blade spins continuously around the bottle axis",
        blade_rest is not None
        and blade_spun is not None
        and abs(blade_spun[1] - blade_rest[1]) > 0.010,
        details=f"rest={blade_rest}, spun={blade_spun}",
    )

    return ctx.report()


object_model = build_object_model()
