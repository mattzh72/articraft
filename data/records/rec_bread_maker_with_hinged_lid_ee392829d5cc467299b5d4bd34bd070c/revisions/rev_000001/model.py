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


BODY_WIDTH = 0.22
BODY_DEPTH = 0.25
BODY_HEIGHT = 0.288
UPPER_WIDTH = 0.205
UPPER_DEPTH = 0.235
UPPER_HEIGHT = 0.218
TOP_Z = 0.07
HINGE_Z = BODY_HEIGHT
HINGE_Y = -0.110


def _body_shell() -> cq.Workplane:
    lower = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, TOP_Z).translate((0.0, 0.0, TOP_Z / 2.0))
    upper = (
        cq.Workplane("XY")
        .box(UPPER_WIDTH, UPPER_DEPTH, UPPER_HEIGHT)
        .translate((0.0, 0.0, TOP_Z + UPPER_HEIGHT / 2.0))
    )
    pod = cq.Workplane("XY").box(0.122, 0.040, 0.088).translate((0.0, 0.1375, 0.168))
    pod_step = cq.Workplane("XY").box(0.092, 0.019, 0.032).translate((0.0, 0.1280, 0.212))

    shell = lower.union(upper).union(pod).union(pod_step)

    cavity = cq.Workplane("XY").box(0.126, 0.172, 0.310).translate((0.0, -0.004, 0.174))
    shaft_bore = cq.Workplane("XZ").circle(0.007).extrude(0.050).translate((0.0, 0.1175, 0.194))
    spindle_well = cq.Workplane("XY").circle(0.010).extrude(0.010).translate((0.0, -0.004, 0.019))

    return shell.cut(cavity).cut(shaft_bore).cut(spindle_well)


def _lid_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.156, 0.215, 0.018).translate((0.0, 0.1075, 0.009))

    underside_pocket = cq.Workplane("XY").box(0.128, 0.170, 0.013).translate((0.0, 0.102, 0.0065))
    vent_slot_a = cq.Workplane("XY").box(0.006, 0.020, 0.020).translate((-0.008, 0.050, 0.010))
    vent_slot_b = cq.Workplane("XY").box(0.006, 0.020, 0.020).translate((0.008, 0.050, 0.010))

    return outer.cut(underside_pocket).cut(vent_slot_a).cut(vent_slot_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bread_maker")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "bread_maker_body"),
        material=body_white,
        name="shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "bread_maker_lid"),
        material=body_white,
        name="lid_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.0048, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Box((0.028, 0.010, 0.006)),
        origin=Origin(xyz=(0.008, 0.0, 0.024)),
        material=charcoal,
        name="blade",
    )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        Box((0.050, 0.032, 0.009)),
        origin=Origin(xyz=(0.0, 0.016, 0.0045)),
        material=charcoal,
        name="cap_shell",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="dial_shaft",
    )
    dial.visual(
        Box((0.006, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.008, 0.022)),
        material=steel,
        name="dial_pointer",
    )

    rocker_0 = model.part("rocker_0")
    rocker_0.visual(
        Box((0.026, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=charcoal,
        name="button_cap",
    )

    rocker_1 = model.part("rocker_1")
    rocker_1.visual(
        Box((0.026, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=charcoal,
        name="button_cap",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    model.articulation(
        "vent_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, 0.034, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.75),
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(0.0, -0.004, 0.0190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.1665, 0.194)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    model.articulation(
        "rocker_0_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker_0,
        origin=Origin(xyz=(-0.028, 0.1575, 0.146)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.22, upper=0.22),
    )

    model.articulation(
        "rocker_1_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker_1,
        origin=Origin(xyz=(0.028, 0.1575, 0.146)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.22, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    spindle = object_model.get_part("spindle")
    vent_cap = object_model.get_part("vent_cap")
    dial = object_model.get_part("dial")
    rocker_0 = object_model.get_part("rocker_0")
    rocker_1 = object_model.get_part("rocker_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    vent_hinge = object_model.get_articulation("vent_hinge")
    spindle_spin = object_model.get_articulation("spindle_spin")
    dial_spin = object_model.get_articulation("dial_spin")
    rocker_0_pivot = object_model.get_articulation("rocker_0_pivot")
    rocker_1_pivot = object_model.get_articulation("rocker_1_pivot")

    ctx.allow_overlap(
        body,
        spindle,
        elem_a="shell",
        elem_b="hub",
        reason="The kneading spindle hub is intentionally seated into the simplified drive socket at the loaf-pan floor.",
    )
    ctx.allow_overlap(
        body,
        dial,
        elem_a="shell",
        elem_b="dial_shaft",
        reason="The selector dial shaft intentionally passes into the stepped control pod behind the front knob face.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", max_gap=0.004, max_penetration=0.0, name="lid rests just above the body rim")

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.10}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={opened_lid_aabb}",
    )

    with ctx.pose({vent_hinge: 0.0}):
        ctx.expect_gap(vent_cap, lid, axis="z", max_gap=0.002, max_penetration=0.0, name="vent cap sits on the lid skin")

    vent_closed = ctx.part_element_world_aabb(vent_cap, elem="cap_shell")
    with ctx.pose({vent_hinge: 0.55}):
        vent_open = ctx.part_element_world_aabb(vent_cap, elem="cap_shell")
    ctx.check(
        "vent cap flips upward",
        vent_closed is not None
        and vent_open is not None
        and vent_open[1][2] > vent_closed[1][2] + 0.010,
        details=f"closed={vent_closed}, open={vent_open}",
    )

    ctx.expect_origin_distance(spindle, body, axes="xy", max_dist=0.010, name="spindle stays on the loaf cavity centerline")

    spindle_blade_rest = ctx.part_element_world_aabb(spindle, elem="blade")
    with ctx.pose({spindle_spin: math.pi / 3.0}):
        spindle_blade_turn = ctx.part_element_world_aabb(spindle, elem="blade")
    ctx.check(
        "spindle rotation changes blade orientation",
        spindle_blade_rest is not None
        and spindle_blade_turn is not None
        and abs((spindle_blade_rest[1][0] - spindle_blade_rest[0][0]) - (spindle_blade_turn[1][0] - spindle_blade_turn[0][0])) > 0.003,
        details=f"rest={spindle_blade_rest}, turned={spindle_blade_turn}",
    )

    dial_pointer_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_spin: math.pi / 2.0}):
        dial_pointer_turn = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "dial pointer visibly rotates",
        dial_pointer_rest is not None
        and dial_pointer_turn is not None
        and abs(
            ((dial_pointer_rest[0][0] + dial_pointer_rest[1][0]) / 2.0)
            - ((dial_pointer_turn[0][0] + dial_pointer_turn[1][0]) / 2.0)
        )
        > 0.010,
        details=f"rest={dial_pointer_rest}, turned={dial_pointer_turn}",
    )

    rocker_1_rest = ctx.part_element_world_aabb(rocker_1, elem="button_cap")
    rocker_0_rest = ctx.part_element_world_aabb(rocker_0, elem="button_cap")
    with ctx.pose({rocker_0_pivot: 0.18}):
        rocker_0_tipped = ctx.part_element_world_aabb(rocker_0, elem="button_cap")
        rocker_1_unchanged = ctx.part_element_world_aabb(rocker_1, elem="button_cap")
    ctx.check(
        "rocker buttons move independently",
        rocker_0_rest is not None
        and rocker_0_tipped is not None
        and rocker_1_rest is not None
        and rocker_1_unchanged is not None
        and abs(rocker_0_tipped[1][1] - rocker_0_rest[1][1]) > 0.001
        and abs(rocker_1_unchanged[1][1] - rocker_1_rest[1][1]) < 1e-6,
        details=f"rocker_0_rest={rocker_0_rest}, rocker_0_tipped={rocker_0_tipped}, rocker_1_rest={rocker_1_rest}, rocker_1_unchanged={rocker_1_unchanged}",
    )

    with ctx.pose({rocker_1_pivot: -0.18}):
        rocker_1_tipped = ctx.part_element_world_aabb(rocker_1, elem="button_cap")
    ctx.check(
        "second rocker also pivots",
        rocker_1_rest is not None
        and rocker_1_tipped is not None
        and abs(rocker_1_tipped[1][1] - rocker_1_rest[1][1]) > 0.001,
        details=f"rest={rocker_1_rest}, tipped={rocker_1_tipped}",
    )

    return ctx.report()


object_model = build_object_model()
