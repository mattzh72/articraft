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


BODY_W = 0.162
BODY_D = 0.118
BODY_H = 0.128
BODY_WALL = 0.005

DRAWER_W = 0.128
DRAWER_H = 0.042
DRAWER_FACE_T = 0.006
DRAWER_FACE_W = 0.140
DRAWER_FACE_H = 0.050
DRAWER_DEPTH = 0.086
DRAWER_WALL = 0.003
DRAWER_BOT_T = 0.004
DRAWER_Z = -0.028
DRAWER_TRAVEL = 0.055

ENTRY_Z = 0.022
ENTRY_R = 0.0046
CHAMBER_R = 0.015

DIAL_OUTER_R = 0.024
DIAL_INNER_R = 0.0066
DIAL_T = 0.008

HUB_BOSS_R = 0.019
HUB_BOSS_L = 0.014
CRANK_Z = 0.006


def _body_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    shell = shell.edges("|Z").fillet(0.010)

    drawer_cavity = (
        cq.Workplane("XY")
        .box(DRAWER_W + 0.006, DRAWER_DEPTH + 0.006, DRAWER_H + 0.004)
        .translate((0.0, -BODY_D / 2.0 + (DRAWER_DEPTH + 0.006) / 2.0, DRAWER_Z))
    )

    entry_bore = (
        cq.Workplane("XZ")
        .circle(ENTRY_R)
        .extrude(0.028)
        .translate((0.0, -BODY_D / 2.0 - 0.001, ENTRY_Z))
    )
    cutter_chamber = (
        cq.Workplane("XZ")
        .circle(CHAMBER_R)
        .extrude(0.044)
        .translate((0.0, -0.040, ENTRY_Z))
    )
    chute = cq.Workplane("XY").box(0.034, 0.056, 0.050).translate((0.0, -0.012, -0.001))

    side_hub = (
        cq.Workplane("YZ")
        .circle(HUB_BOSS_R)
        .extrude(HUB_BOSS_L)
        .translate((BODY_W / 2.0, 0.0, CRANK_Z))
    )

    return shell.cut(drawer_cavity).cut(entry_bore).cut(cutter_chamber).cut(chute).union(side_hub)


def _drawer_shape() -> cq.Workplane:
    front_panel = (
        cq.Workplane("XY")
        .box(DRAWER_FACE_W, DRAWER_FACE_T, DRAWER_FACE_H)
        .translate((0.0, DRAWER_FACE_T / 2.0, 0.0))
    )
    tray_depth = DRAWER_DEPTH - DRAWER_FACE_T
    tray_width = DRAWER_W - 2.0 * DRAWER_WALL
    tray_wall_h = DRAWER_H - DRAWER_BOT_T

    bottom = cq.Workplane("XY").box(tray_width, tray_depth, DRAWER_BOT_T).translate(
        (0.0, DRAWER_FACE_T + tray_depth / 2.0, -DRAWER_H / 2.0 + DRAWER_BOT_T / 2.0)
    )
    left_wall = cq.Workplane("XY").box(DRAWER_WALL, tray_depth, tray_wall_h).translate(
        (DRAWER_W / 2.0 - DRAWER_WALL / 2.0, DRAWER_FACE_T + tray_depth / 2.0, DRAWER_BOT_T / 2.0)
    )
    right_wall = cq.Workplane("XY").box(DRAWER_WALL, tray_depth, tray_wall_h).translate(
        (-DRAWER_W / 2.0 + DRAWER_WALL / 2.0, DRAWER_FACE_T + tray_depth / 2.0, DRAWER_BOT_T / 2.0)
    )
    back_wall = cq.Workplane("XY").box(tray_width, DRAWER_WALL, tray_wall_h).translate(
        (0.0, DRAWER_DEPTH - DRAWER_WALL / 2.0, DRAWER_BOT_T / 2.0)
    )

    finger_pull = cq.Workplane("XY").box(0.044, 0.014, 0.016).translate((0.0, 0.001, 0.010))
    finger_pull = finger_pull.edges("|Z").fillet(0.003)

    drawer = front_panel.union(bottom).union(left_wall).union(right_wall).union(back_wall)
    return drawer.cut(finger_pull)


def _dial_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(DIAL_OUTER_R)
        .circle(DIAL_INNER_R)
        .extrude(DIAL_T)
        .translate((0.0, -DIAL_T / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_pencil_sharpener")

    body_finish = model.material("body_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.12, 0.13, 0.15, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.11, 0.10, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "sharpener_body"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((BODY_W - 0.030, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.002, -BODY_H / 2.0 + 0.018)),
        material=dark_trim,
        name="rear_band",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "sharpener_drawer"),
        material=drawer_finish,
        name="drawer_shell",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "sharpener_dial"),
        material=trim_metal,
        name="dial_ring",
    )
    dial.visual(
        Box((0.010, DIAL_T * 0.70, 0.006)),
        origin=Origin(xyz=(0.0, -DIAL_T * 0.35, DIAL_OUTER_R + 0.001)),
        material=dark_trim,
        name="dial_pointer",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.023, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_metal,
        name="hub_cap",
    )
    crank.visual(
        Box((0.008, 0.010, 0.046)),
        origin=Origin(xyz=(0.004, 0.0, 0.023)),
        material=trim_metal,
        name="crank_arm",
    )
    crank.visual(
        Box((0.008, 0.020, 0.008)),
        origin=Origin(xyz=(0.004, 0.010, 0.046)),
        material=trim_metal,
        name="grip_shaft",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.009, 0.021, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="handle_knob",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, DRAWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, ENTRY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_W / 2.0 + HUB_BOSS_L, 0.0, CRANK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("dial")
    crank = object_model.get_part("crank")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    dial_spin = object_model.get_articulation("body_to_dial")
    crank_spin = object_model.get_articulation("body_to_crank")

    ctx.allow_overlap(
        body,
        drawer,
        elem_a="body_shell",
        elem_b="drawer_shell",
        reason="The shavings drawer is intentionally represented as a nested tray inside a simplified housing cavity shell.",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((float(lo[i]) + float(hi[i])) * 0.5 for i in range(3))

    ctx.check(
        "drawer uses prismatic articulation",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_slide.articulation_type}",
    )
    ctx.check(
        "dial uses continuous articulation",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_spin.articulation_type}",
    )
    ctx.check(
        "crank uses continuous articulation",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_spin.articulation_type}",
    )

    body_aabb = ctx.part_world_aabb(body)
    drawer_limits = drawer_slide.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: 0.0}):
            drawer_closed = ctx.part_world_aabb(drawer)
            ctx.check(
                "drawer front sits nearly flush with body front",
                body_aabb is not None
                and drawer_closed is not None
                and abs(float(drawer_closed[0][1]) - float(body_aabb[0][1])) <= 0.002,
                details=f"body_min_y={None if body_aabb is None else body_aabb[0][1]}, drawer_min_y={None if drawer_closed is None else drawer_closed[0][1]}",
            )

        with ctx.pose({drawer_slide: drawer_limits.upper}):
            drawer_open = ctx.part_world_aabb(drawer)
            ctx.expect_within(
                drawer,
                body,
                axes="xz",
                margin=0.002,
                name="drawer stays guided within body envelope",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                min_overlap=0.030,
                name="drawer remains retained at full extension",
            )
            ctx.check(
                "drawer extends forward",
                drawer_closed is not None
                and drawer_open is not None
                and float(drawer_open[0][1]) < float(drawer_closed[0][1]) - 0.045,
                details=f"closed_min_y={None if drawer_closed is None else drawer_closed[0][1]}, open_min_y={None if drawer_open is None else drawer_open[0][1]}",
            )

    dial_center_0 = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    with ctx.pose({dial_spin: math.pi / 2.0}):
        dial_center_90 = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    ctx.check(
        "dial pointer orbits the pencil entry axis",
        dial_center_0 is not None
        and dial_center_90 is not None
        and abs(dial_center_0[0] - dial_center_90[0]) > 0.012
        and abs(dial_center_0[2] - dial_center_90[2]) > 0.010,
        details=f"rest={dial_center_0}, quarter_turn={dial_center_90}",
    )

    crank_center_0 = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_knob"))
    with ctx.pose({crank_spin: math.pi / 2.0}):
        crank_center_90 = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_knob"))
    ctx.check(
        "crank handle orbits around the side hub",
        crank_center_0 is not None
        and crank_center_90 is not None
        and abs(crank_center_0[1] - crank_center_90[1]) > 0.030
        and abs(crank_center_0[2] - crank_center_90[2]) > 0.020,
        details=f"rest={crank_center_0}, quarter_turn={crank_center_90}",
    )

    return ctx.report()


object_model = build_object_model()
