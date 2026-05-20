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


BODY_D = 0.104
BODY_W = 0.068
BODY_H = 0.102
BODY_FRONT_X = BODY_D / 2.0
BODY_SIDE_Y = BODY_W / 2.0

SHELL_WALL = 0.003
LOWER_CAVITY_Z = 0.009

TRAY_D = 0.059
TRAY_W = 0.048
TRAY_H = 0.022
TRAY_WALL = 0.0025
TRAY_FRONT_T = 0.003
TRAY_TRAVEL = 0.032

PORT_Z = 0.060
PORT_R = 0.0068
PORT_RECESS_R = 0.0105

FLAP_HINGE_X = BODY_FRONT_X + 0.0015
FLAP_HINGE_Z = 0.070
FLAP_PANEL_T = 0.0026
FLAP_PANEL_W = 0.024
FLAP_PANEL_H = 0.013
FLAP_BARREL_R = 0.0025
FLAP_BARREL_L = 0.0104
FLAP_OPEN = 1.10

CRANK_AXLE_X = 0.017
CRANK_AXLE_Z = 0.057
CRANK_BOSS_R = 0.010
CRANK_BOSS_L = 0.004
CRANK_JOINT_Y = BODY_SIDE_Y + CRANK_BOSS_L - 0.0003
CRANK_GRIP_R = 0.0045
CRANK_GRIP_L = 0.012
CRANK_GRIP_CENTER = (0.034, 0.017, -0.019)
CRANK_POSE_SAMPLE = 1.20


def _build_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H - 0.028, centered=(True, True, False))
        .translate((0.0, 0.0, 0.028))
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .edges()
        .fillet(0.012)
    )

    upper_cavity = (
        cq.Workplane("XY")
        .box(0.058, 0.040, 0.032, centered=(True, True, True))
        .translate((0.004, 0.0, 0.063))
    )
    port_tunnel = (
        cq.Workplane("YZ")
        .center(0.0, PORT_Z)
        .circle(PORT_R)
        .extrude(0.028)
        .translate((BODY_FRONT_X - 0.028 + 0.0004, 0.0, 0.0))
    )
    port_recess = (
        cq.Workplane("YZ")
        .center(0.0, PORT_Z)
        .circle(PORT_RECESS_R)
        .extrude(0.0045)
        .translate((BODY_FRONT_X - 0.0043, 0.0, 0.0))
    )
    cutter_chamber = (
        cq.Workplane("YZ")
        .center(0.0, PORT_Z - 0.001)
        .circle(0.013)
        .extrude(0.016)
        .translate((BODY_FRONT_X - 0.019, 0.0, 0.0))
    )

    return shell.cut(upper_cavity).cut(port_tunnel).cut(port_recess).cut(cutter_chamber)


def _build_flap_hinge_mount() -> cq.Workplane:
    brow = cq.Workplane("XY").box(0.006, 0.030, 0.006, centered=(True, True, True)).translate((BODY_FRONT_X - 0.0015, 0.0, FLAP_HINGE_Z))
    lug_a = cq.Workplane("XY").box(0.0046, 0.005, 0.0054, centered=(True, True, True)).translate((FLAP_HINGE_X - 0.0003, 0.0077, FLAP_HINGE_Z))
    lug_b = cq.Workplane("XY").box(0.0046, 0.005, 0.0054, centered=(True, True, True)).translate((FLAP_HINGE_X - 0.0003, -0.0077, FLAP_HINGE_Z))
    return brow.union(lug_a).union(lug_b)


def _build_tray_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(TRAY_D, TRAY_W, TRAY_H, centered=(True, True, True))
        .translate((-(TRAY_D - TRAY_FRONT_T) / 2.0, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(TRAY_D - TRAY_FRONT_T - TRAY_WALL, TRAY_W - (2.0 * TRAY_WALL), TRAY_H - TRAY_WALL, centered=(True, True, True))
        .translate(((-TRAY_D + TRAY_WALL) / 2.0, 0.0, TRAY_WALL / 2.0))
    )
    pull = cq.Workplane("YZ").center(0.0, 0.0).circle(0.0055).extrude(0.0022).translate((0.0013, 0.0, 0.0))
    return outer.cut(inner).cut(pull)


def _build_crank_arm() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.0065)
        .extrude(0.009)
        .translate((0.0, 0.0, -0.0045))
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, 0.0045, 0.0))
    )
    spacer = cq.Workplane("XY").box(0.006, 0.020, 0.006, centered=(True, True, True)).translate((0.003, 0.011, -0.001))
    arm = cq.Workplane("XY").box(0.026, 0.006, 0.006, centered=(True, True, True)).translate((0.016, 0.017, -0.007))
    drop = cq.Workplane("XY").box(0.006, 0.007, 0.014, centered=(True, True, True)).translate((0.028, 0.017, -0.014))
    tip = cq.Workplane("XY").box(0.008, 0.008, 0.008, centered=(True, True, True)).translate((0.031, 0.017, -0.017))
    return hub.union(spacer).union(arm).union(drop).union(tip)


def _build_flap_panel() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(FLAP_PANEL_T, FLAP_PANEL_W, FLAP_PANEL_H, centered=(True, True, True))
        .translate((FLAP_PANEL_T / 2.0, 0.0, -FLAP_PANEL_H / 2.0))
    )
    bridge = cq.Workplane("XY").box(0.0032, 0.012, 0.004, centered=(True, True, True)).translate((0.0013, 0.0, -0.002))
    return panel.union(bridge)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][idx] + aabb[1][idx]) / 2.0 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pencil_sharpener")

    model.material("shell", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("shell_detail", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("tray", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("crank", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("grip", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("flap", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shell(), "sharpener_shell"), material="shell", name="shell")
    body.visual(
        Box((0.074, 0.060, 0.004)),
        origin=Origin(xyz=(0.015, 0.0, 0.002)),
        material="shell_detail",
        name="base_floor",
    )
    body.visual(
        Box((0.066, 0.0065, 0.029)),
        origin=Origin(xyz=(0.018, 0.03075, 0.0145)),
        material="shell_detail",
        name="side_wall_0",
    )
    body.visual(
        Box((0.066, 0.0065, 0.029)),
        origin=Origin(xyz=(0.018, -0.03075, 0.0145)),
        material="shell_detail",
        name="side_wall_1",
    )
    body.visual(
        Box((0.004, 0.055, 0.029)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0145)),
        material="shell_detail",
        name="rear_wall",
    )
    body.visual(
        Box((0.006, 0.056, 0.005)),
        origin=Origin(xyz=(BODY_FRONT_X - 0.003, 0.0, 0.0025)),
        material="shell_detail",
        name="lower_rail",
    )
    body.visual(mesh_from_cadquery(_build_flap_hinge_mount(), "sharpener_flap_hinge"), material="shell_detail", name="flap_hinge")
    body.visual(
        Cylinder(radius=CRANK_BOSS_R, length=CRANK_BOSS_L),
        origin=Origin(
            xyz=(CRANK_AXLE_X, BODY_SIDE_Y + (CRANK_BOSS_L / 2.0) - 0.0003, CRANK_AXLE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="shell_detail",
        name="crank_boss",
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_build_tray_shell(), "sharpener_tray"), material="tray", name="tray_shell")

    crank = model.part("crank")
    crank.visual(mesh_from_cadquery(_build_crank_arm(), "sharpener_crank"), material="crank", name="arm")
    crank.visual(
        Cylinder(radius=CRANK_GRIP_R, length=CRANK_GRIP_L),
        origin=Origin(xyz=CRANK_GRIP_CENTER),
        material="grip",
        name="grip",
    )

    flap = model.part("flap")
    flap.visual(mesh_from_cadquery(_build_flap_panel(), "sharpener_flap_panel"), material="flap", name="panel")
    flap.visual(
        Cylinder(radius=FLAP_BARREL_R, length=FLAP_BARREL_L),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="flap",
        name="barrel",
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(BODY_FRONT_X - (TRAY_FRONT_T / 2.0), 0.0, LOWER_CAVITY_Z + (TRAY_H / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=18.0, velocity=0.20),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_AXLE_X, CRANK_JOINT_Y, CRANK_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FLAP_OPEN, effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    crank = object_model.get_part("crank")
    flap = object_model.get_part("flap")

    tray_joint = object_model.get_articulation("body_to_tray")
    crank_joint = object_model.get_articulation("body_to_crank")
    flap_joint = object_model.get_articulation("body_to_flap")

    tray_limits = tray_joint.motion_limits
    flap_limits = flap_joint.motion_limits

    with ctx.pose({tray_joint: 0.0}):
        shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
        tray_aabb = ctx.part_world_aabb(tray)
        ctx.expect_within(tray, body, axes="yz", margin=0.0025, name="tray stays aligned in the lower opening")
        ctx.check(
            "tray front sits flush with shell front",
            shell_aabb is not None
            and tray_aabb is not None
            and abs(tray_aabb[1][0] - shell_aabb[1][0]) <= 0.0015,
            details=f"shell_aabb={shell_aabb}, tray_aabb={tray_aabb}",
        )
        closed_tray_pos = ctx.part_world_position(tray)

    if tray_limits is not None and tray_limits.upper is not None:
        with ctx.pose({tray_joint: tray_limits.upper}):
            ctx.expect_within(tray, body, axes="yz", margin=0.0025, name="extended tray remains centered in the body")
            ctx.expect_overlap(tray, body, axes="x", min_overlap=0.020, name="extended tray remains partially inserted")
            open_tray_pos = ctx.part_world_position(tray)
        ctx.check(
            "tray pulls forward from the body",
            closed_tray_pos is not None
            and open_tray_pos is not None
            and open_tray_pos[0] > closed_tray_pos[0] + 0.020,
            details=f"closed={closed_tray_pos}, open={open_tray_pos}",
        )

    with ctx.pose({flap_joint: 0.0}):
        shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
        panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")
        closed_panel_gap = None if shell_aabb is None or panel_aabb is None else panel_aabb[0][0] - shell_aabb[1][0]
        closed_panel_center = _aabb_center(panel_aabb)
        closed_panel_low_z = None if panel_aabb is None else panel_aabb[0][2]
        ctx.check(
            "dust flap rests close over the pencil port",
            closed_panel_gap is not None and 0.0005 <= closed_panel_gap <= 0.0035,
            details=f"shell_aabb={shell_aabb}, panel_aabb={panel_aabb}, gap={closed_panel_gap}",
        )

    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({flap_joint: flap_limits.upper}):
            shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
            open_panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")
            open_panel_center = _aabb_center(open_panel_aabb)
            open_panel_low_z = None if open_panel_aabb is None else open_panel_aabb[0][2]
        ctx.check(
            "dust flap swings upward clear of the port",
            closed_panel_center is not None
            and open_panel_center is not None
            and closed_panel_low_z is not None
            and open_panel_low_z is not None
            and open_panel_center[0] > closed_panel_center[0] + 0.004
            and open_panel_low_z > closed_panel_low_z + 0.006,
            details=f"closed_center={closed_panel_center}, open_center={open_panel_center}, open_panel_aabb={open_panel_aabb}, closed_low_z={closed_panel_low_z}",
        )

    with ctx.pose({crank_joint: 0.0}):
        ctx.expect_gap(crank, body, axis="y", max_penetration=0.0, name="crank clears the body side at rest")
        rest_grip = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    with ctx.pose({crank_joint: CRANK_POSE_SAMPLE}):
        ctx.expect_gap(crank, body, axis="y", max_penetration=0.0, name="crank clears the body side while turned")
        turned_grip = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    ctx.check(
        "side crank rotates around its axle",
        rest_grip is not None
        and turned_grip is not None
        and abs(turned_grip[2] - rest_grip[2]) > 0.012
        and abs(turned_grip[0] - rest_grip[0]) > 0.008,
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
