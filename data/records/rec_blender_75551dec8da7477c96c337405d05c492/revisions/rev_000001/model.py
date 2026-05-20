from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_WIDTH = 0.22
BASE_DEPTH = 0.22
BASE_BODY_HEIGHT = 0.158
BASE_MOUNT_HEIGHT = 0.008
BASE_TOTAL_HEIGHT = BASE_BODY_HEIGHT + BASE_MOUNT_HEIGHT

PITCHER_HEIGHT = 0.205
PITCHER_BOTTOM = 0.122
PITCHER_TOP = 0.158
PITCHER_WALL = 0.005
PITCHER_BASE_THICKNESS = 0.014

FRAME_RING_HEIGHT = 0.020
FRAME_FRONT_THICKNESS = 0.008
OPENING_WIDTH = 0.104
OPENING_HEIGHT = 0.090
OPENING_BOTTOM = 0.094
HINGE_AXIS_Z = 0.181
HINGE_AXIS_X = PITCHER_TOP / 2.0 + 0.017

KNOB_DIAMETER = 0.040
KNOB_DEPTH = 0.024
KNOB_X = 0.046
KNOB_Z = 0.082

BLADE_Z = 0.020


def make_base_shell() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, 0.115, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    shoulder = (
        cq.Workplane("XY")
        .rect(BASE_WIDTH * 0.94, BASE_DEPTH * 0.94)
        .workplane(offset=0.043)
        .rect(BASE_WIDTH * 0.80, BASE_DEPTH * 0.80)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.115))
    )
    mount = (
        cq.Workplane("XY")
        .box(0.094, 0.094, BASE_MOUNT_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, BASE_BODY_HEIGHT))
    )
    return lower.union(shoulder).union(mount)


def make_pitcher_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(PITCHER_BOTTOM, PITCHER_BOTTOM)
        .workplane(offset=PITCHER_HEIGHT - FRAME_RING_HEIGHT)
        .rect(PITCHER_TOP, PITCHER_TOP)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=PITCHER_BASE_THICKNESS)
        .rect(PITCHER_BOTTOM - 2.0 * PITCHER_WALL, PITCHER_BOTTOM - 2.0 * PITCHER_WALL)
        .workplane(offset=PITCHER_HEIGHT - FRAME_RING_HEIGHT - PITCHER_BASE_THICKNESS)
        .rect(PITCHER_TOP - 2.0 * PITCHER_WALL, PITCHER_TOP - 2.0 * PITCHER_WALL)
        .loft(combine=True)
    )
    shell = outer.cut(inner)

    handle_outer = (
        cq.Workplane("XY")
        .box(0.032, 0.022, 0.145, centered=(True, True, False))
        .translate((0.0, -(PITCHER_TOP / 2.0 + 0.018), 0.030))
    )
    handle_inner = (
        cq.Workplane("XY")
        .box(0.018, 0.038, 0.090, centered=(True, True, False))
        .translate((0.0, -(PITCHER_TOP / 2.0 + 0.018), 0.057))
    )
    handle = handle_outer.cut(handle_inner)

    top_mount = (
        cq.Workplane("XY")
        .box(0.020, 0.032, 0.028, centered=(True, True, False))
        .translate((0.0, -(PITCHER_TOP / 2.0 + 0.008), 0.148))
    )
    bottom_mount = (
        cq.Workplane("XY")
        .box(0.018, 0.028, 0.024, centered=(True, True, False))
        .translate((0.0, -(PITCHER_BOTTOM / 2.0 + 0.016), 0.026))
    )

    spout = (
        cq.Workplane("YZ")
        .center(0.0, PITCHER_HEIGHT - FRAME_RING_HEIGHT + 0.003)
        .rect(0.038, 0.010)
        .extrude(0.014)
        .translate((PITCHER_TOP / 2.0 - 0.006, 0.0, 0.0))
    )

    return shell.union(handle).union(top_mount).union(bottom_mount).union(spout)


def make_pitcher_frame() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .workplane(offset=PITCHER_HEIGHT - FRAME_RING_HEIGHT)
        .rect(PITCHER_TOP + 0.014, PITCHER_TOP + 0.014)
        .extrude(FRAME_RING_HEIGHT)
        .cut(
            cq.Workplane("XY")
            .workplane(offset=PITCHER_HEIGHT - FRAME_RING_HEIGHT - 0.001)
            .rect(PITCHER_TOP - 0.038, PITCHER_TOP - 0.038)
            .extrude(FRAME_RING_HEIGHT + 0.004)
        )
    )

    front_frame = (
        cq.Workplane("YZ")
        .center(0.0, OPENING_BOTTOM + OPENING_HEIGHT / 2.0)
        .rect(OPENING_WIDTH + 0.020, OPENING_HEIGHT + 0.022)
        .extrude(FRAME_FRONT_THICKNESS)
        .cut(
            cq.Workplane("YZ")
            .center(0.0, OPENING_BOTTOM + OPENING_HEIGHT / 2.0)
            .rect(OPENING_WIDTH, OPENING_HEIGHT)
            .extrude(FRAME_FRONT_THICKNESS + 0.004)
        )
        .translate((PITCHER_TOP / 2.0 + 0.004, 0.0, 0.0))
    )

    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.014, 0.030, 0.010, centered=(True, True, False))
        .translate((PITCHER_TOP / 2.0 + 0.010, 0.0, HINGE_AXIS_Z - 0.005))
    )
    hinge_pin = (
        cq.Workplane("XZ")
        .center(HINGE_AXIS_X, HINGE_AXIS_Z)
        .circle(0.0035)
        .extrude(0.030, both=True)
    )

    return collar.union(front_frame).union(hinge_bridge).union(hinge_pin)


def make_shield_flap() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(0.0035, OPENING_WIDTH - 0.012, OPENING_HEIGHT - 0.004, centered=(True, True, False))
        .translate((0.014, 0.0, -(OPENING_HEIGHT - 0.004)))
    )
    top_rail = (
        cq.Workplane("XY")
        .box(0.010, OPENING_WIDTH - 0.004, 0.014, centered=(True, True, False))
        .translate((0.011, 0.0, -0.014))
    )
    return panel.union(top_rail)


def make_shield_hinge() -> cq.Workplane:
    barrel_left = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(0.004)
        .extrude(0.022, both=True)
        .translate((0.0, -0.034, 0.0))
    )
    barrel_right = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(0.004)
        .extrude(0.022, both=True)
        .translate((0.0, 0.034, 0.0))
    )
    lug_left = (
        cq.Workplane("XY")
        .box(0.008, 0.012, 0.010, centered=(True, True, False))
        .translate((0.004, -0.034, -0.010))
    )
    lug_right = (
        cq.Workplane("XY")
        .box(0.008, 0.012, 0.010, centered=(True, True, False))
        .translate((0.004, 0.034, -0.010))
    )
    return barrel_left.union(barrel_right).union(lug_left).union(lug_right)


def make_blade() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.012).extrude(0.006)
    spindle = cq.Workplane("XY").circle(0.0045).extrude(0.016).translate((0.0, 0.0, -0.004))
    blade_blank = (
        cq.Workplane("XY")
        .center(0.027, 0.0)
        .rect(0.040, 0.010)
        .extrude(0.0018)
        .translate((0.0, 0.0, 0.004))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 14.0)
    )
    blade_set = (
        blade_blank
        .union(blade_blank.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0))
        .union(blade_blank.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0))
        .union(blade_blank.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 270.0))
    )
    return hub.union(spindle).union(blade_set)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bar_blender")

    base_dark = model.material("base_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    lid_black = model.material("lid_black", rgba=(0.07, 0.07, 0.08, 1.0))
    polycarbonate = model.material("polycarbonate", rgba=(0.70, 0.76, 0.82, 0.35))
    shield_clear = model.material("shield_clear", rgba=(0.76, 0.84, 0.90, 0.32))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shell(), "base_shell"),
        material=base_dark,
        name="shell",
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        mesh_from_cadquery(make_pitcher_shell(), "pitcher_shell"),
        material=polycarbonate,
        name="shell",
    )
    pitcher.visual(
        mesh_from_cadquery(make_pitcher_frame(), "pitcher_frame"),
        material=lid_black,
        name="frame",
    )

    shield = model.part("shield")
    shield.visual(
        mesh_from_cadquery(make_shield_flap(), "shield_flap"),
        material=shield_clear,
        name="panel",
    )
    shield.visual(
        mesh_from_cadquery(make_shield_hinge(), "shield_hinge"),
        material=shield_clear,
        name="hinge",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_DEPTH,
                body_style="tapered",
                top_diameter=0.032,
                edge_radius=0.0012,
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(make_blade(), "blade"),
        material=steel,
        name="assembly",
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
    )
    model.articulation(
        "pitcher_to_shield",
        ArticulationType.REVOLUTE,
        parent=pitcher,
        child=shield,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(KNOB_X, BASE_DEPTH / 2.0, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, BLADE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    shield = object_model.get_part("shield")
    knob = object_model.get_part("knob")
    blade = object_model.get_part("blade")
    shield_hinge = object_model.get_articulation("pitcher_to_shield")

    ctx.allow_overlap(
        blade,
        pitcher,
        elem_a="assembly",
        elem_b="shell",
        reason="The blade intentionally spins inside the pitcher cavity, while the clear jar is represented as a single shell mesh.",
    )
    ctx.allow_overlap(
        shield,
        pitcher,
        elem_a="hinge",
        elem_b="frame",
        reason="The splash shield hinge hardware intentionally nests against the short top hinge support on the pitcher frame.",
    )

    ctx.expect_gap(
        pitcher,
        base,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        name="pitcher seats on the motor base",
    )
    ctx.expect_overlap(
        pitcher,
        base,
        axes="xy",
        min_overlap=0.080,
        name="pitcher footprint stays over the square base",
    )
    ctx.expect_gap(
        knob,
        base,
        axis="y",
        positive_elem="dial",
        negative_elem="shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="speed knob mounts flush on the right side panel",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        inner_elem="assembly",
        outer_elem="shell",
        margin=0.015,
        name="blade stays centered inside the pitcher footprint",
    )

    with ctx.pose({shield_hinge: 0.0}):
        ctx.expect_gap(
            shield,
            pitcher,
            axis="x",
            positive_elem="panel",
            negative_elem="frame",
            min_gap=0.001,
            max_gap=0.020,
            name="closed splash shield sits just in front of the front frame",
        )
        ctx.expect_overlap(
            shield,
            pitcher,
            axes="yz",
            elem_a="panel",
            elem_b="frame",
            min_overlap=0.060,
            name="closed splash shield covers the front opening",
        )

    closed_aabb = ctx.part_element_world_aabb(shield, elem="panel")
    open_aabb = None
    with ctx.pose({shield_hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(shield, elem="panel")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    closed_center = aabb_center(closed_aabb)
    open_center = aabb_center(open_aabb)
    ctx.check(
        "shield swings upward and forward",
        closed_center is not None
        and open_center is not None
        and open_center[2] > closed_center[2] + 0.020
        and open_center[0] > closed_center[0] + 0.030,
        details=f"closed_center={closed_center!r}, open_center={open_center!r}",
    )

    return ctx.report()


object_model = build_object_model()
