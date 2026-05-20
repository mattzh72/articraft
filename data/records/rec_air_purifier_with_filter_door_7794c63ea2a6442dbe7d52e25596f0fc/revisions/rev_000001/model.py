from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_W = 0.50
BODY_H = 0.80
BODY_D = 0.18
SHELL_T = 0.0045
BEZEL_T = 0.014

DOOR_W = 0.418
DOOR_H = 0.470
DOOR_T = 0.018
DOOR_CENTER_Z = 0.041
DOOR_OPEN_W = DOOR_W + 0.004
DOOR_OPEN_H = DOOR_H + 0.004
DOOR_HINGE_X = BODY_D / 2.0 - 0.002
DOOR_HINGE_Y = DOOR_OPEN_W / 2.0
DOOR_OPEN_ANGLE = 1.32

DRAWER_W = 0.410
DRAWER_H = 0.140
DRAWER_FACE_T = 0.020
DRAWER_CENTER_Z = -0.282
DRAWER_OPEN_W = DRAWER_W + 0.004
DRAWER_OPEN_H = DRAWER_H + 0.002
DRAWER_TRAVEL = 0.100

FLAP_W = 0.120
FLAP_H = 0.038
FLAP_T = 0.010
FLAP_CENTER_Z = 0.313
FLAP_OPEN_W = FLAP_W + 0.002
FLAP_OPEN_H = FLAP_H
FLAP_OPEN_ANGLE = 1.05


def _body_shape() -> cq.Workplane:
    outer_shell = cq.Workplane("XY").box(BODY_D, BODY_W, BODY_H).faces(">X").shell(-SHELL_T)

    bezel = (
        cq.Workplane("XY")
        .box(BEZEL_T, BODY_W - 0.014, BODY_H - 0.014)
        .translate((BODY_D / 2.0 - BEZEL_T / 2.0, 0.0, 0.0))
    )
    bezel = bezel.cut(
        cq.Workplane("XY")
        .box(BEZEL_T + 0.020, DOOR_OPEN_W, DOOR_OPEN_H)
        .translate((BODY_D / 2.0 - BEZEL_T / 2.0, 0.0, DOOR_CENTER_Z))
    )
    bezel = bezel.cut(
        cq.Workplane("XY")
        .box(BEZEL_T + 0.020, DRAWER_OPEN_W, DRAWER_OPEN_H)
        .translate((BODY_D / 2.0 - BEZEL_T / 2.0, 0.0, DRAWER_CENTER_Z))
    )
    bezel = bezel.cut(
        cq.Workplane("XY")
        .box(BEZEL_T + 0.020, FLAP_OPEN_W, FLAP_OPEN_H)
        .translate((BODY_D / 2.0 - BEZEL_T / 2.0, 0.0, FLAP_CENTER_Z))
    )

    divider = (
        cq.Workplane("XY")
        .box(BODY_D - 2.0 * SHELL_T, BODY_W - 2.0 * SHELL_T, 0.008)
        .translate((0.000, 0.000, DRAWER_CENTER_Z + DRAWER_H / 2.0 + 0.014))
    )

    upper_backplate = (
        cq.Workplane("XY")
        .box(0.010, DOOR_W - 0.050, DOOR_H - 0.080)
        .translate((-BODY_D / 2.0 + SHELL_T + 0.005, 0.0, DOOR_CENTER_Z))
    )

    return outer_shell.union(bezel).union(divider).union(upper_backplate)


def _door_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DOOR_T, DOOR_W, DOOR_H).translate((0.000, -DOOR_W / 2.0, 0.0))
    inner_pocket = (
        cq.Workplane("XY")
        .box(DOOR_T - 0.004, DOOR_W - 0.050, DOOR_H - 0.055)
        .translate((-0.002, -DOOR_W / 2.0, -0.004))
    )
    front_recess = (
        cq.Workplane("XY")
        .box(0.003, DOOR_W - 0.090, DOOR_H - 0.120)
        .translate((DOOR_T / 2.0 - 0.0015, -DOOR_W / 2.0 - 0.006, 0.0))
    )
    return panel.cut(inner_pocket).cut(front_recess).translate((0.011, 0.0, 0.0))


def _drawer_shape() -> cq.Workplane:
    tray_outer_w = DRAWER_W - 0.036
    tray_outer_h = DRAWER_H - 0.030
    tray_depth = 0.126
    tray = (
        cq.Workplane("XY")
        .box(tray_depth, tray_outer_w, tray_outer_h)
        .translate((-DRAWER_FACE_T / 2.0 - tray_depth / 2.0, 0.0, -0.0055))
    )
    tray_cavity = (
        cq.Workplane("XY")
        .box(tray_depth - 0.016, tray_outer_w - 0.030, tray_outer_h - 0.018)
        .translate((-DRAWER_FACE_T / 2.0 - tray_depth / 2.0 - 0.002, 0.0, 0.008))
    )
    face = cq.Workplane("XY").box(DRAWER_FACE_T, DRAWER_W, DRAWER_H)
    finger_recess = (
        cq.Workplane("XY")
        .box(0.008, 0.165, 0.024)
        .translate((DRAWER_FACE_T / 2.0 - 0.002, 0.0, 0.024))
    )
    filter_block = (
        cq.Workplane("XY")
        .box(0.055, tray_outer_w - 0.040, tray_outer_h - 0.038)
        .translate((-0.072, 0.0, -0.0095))
    )
    return face.union(tray.cut(tray_cavity)).union(filter_block).cut(finger_recess)


def _flap_shape() -> cq.Workplane:
    flap = cq.Workplane("XY").box(FLAP_T, FLAP_W, FLAP_H).translate((0.000, 0.000, -FLAP_H / 2.0))
    rear_relief = (
        cq.Workplane("XY")
        .box(FLAP_T - 0.003, FLAP_W - 0.020, FLAP_H - 0.010)
        .translate((-0.0015, 0.000, -FLAP_H / 2.0 - 0.001))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.004, 0.060, 0.008)
        .translate((FLAP_T / 2.0 + 0.001, 0.000, -FLAP_H + 0.007))
    )
    return flap.cut(rear_relief).union(finger_lip).translate((0.007, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_panel_purifier")

    shell_finish = model.material("shell_finish", rgba=(0.93, 0.94, 0.95, 1.0))
    door_finish = model.material("door_finish", rgba=(0.96, 0.97, 0.98, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.87, 0.89, 0.91, 1.0))
    flap_finish = model.material("flap_finish", rgba=(0.82, 0.84, 0.86, 1.0))
    rail_finish = model.material("rail_finish", rgba=(0.22, 0.24, 0.27, 1.0))

    guide_y = DRAWER_W / 2.0 - 0.060
    guide_height = 0.051
    guide_z = -BODY_H / 2.0 + SHELL_T + guide_height / 2.0
    guide_length = 0.120
    runner_x = -0.070
    runner_z = -0.0575

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "purifier_body"),
        material=shell_finish,
        name="body_shell",
    )
    body.visual(
        Box((guide_length, 0.024, guide_height)),
        origin=Origin(xyz=(-0.004, guide_y, guide_z)),
        material=rail_finish,
        name="guide_0",
    )
    body.visual(
        Box((guide_length, 0.024, guide_height)),
        origin=Origin(xyz=(-0.004, -guide_y, guide_z)),
        material=rail_finish,
        name="guide_1",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "purifier_door"),
        material=door_finish,
        name="door_panel",
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "purifier_filter_drawer"),
        material=drawer_finish,
        name="drawer_shell",
    )
    filter_drawer.visual(
        Box((0.110, 0.018, 0.010)),
        origin=Origin(xyz=(runner_x, guide_y, runner_z)),
        material=rail_finish,
        name="runner_0",
    )
    filter_drawer.visual(
        Box((0.110, 0.018, 0.010)),
        origin=Origin(xyz=(runner_x, -guide_y, runner_z)),
        material=rail_finish,
        name="runner_1",
    )

    latch_flap = model.part("latch_flap")
    latch_flap.visual(
        mesh_from_cadquery(_flap_shape(), "purifier_latch_flap"),
        material=flap_finish,
        name="flap_panel",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )
    model.articulation(
        "body_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_drawer,
        origin=Origin(xyz=(BODY_D / 2.0 - DRAWER_FACE_T / 2.0, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_latch_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch_flap,
        origin=Origin(xyz=(BODY_D / 2.0 - 0.002, 0.0, FLAP_CENTER_Z + FLAP_H / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=FLAP_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    filter_drawer = object_model.get_part("filter_drawer")
    latch_flap = object_model.get_part("latch_flap")

    door_hinge = object_model.get_articulation("body_to_door")
    drawer_slide = object_model.get_articulation("body_to_filter_drawer")
    flap_hinge = object_model.get_articulation("body_to_latch_flap")

    body_box = ctx.part_world_aabb(body)
    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0, flap_hinge: 0.0}):
        door_box = ctx.part_world_aabb(door)
        drawer_box = ctx.part_world_aabb(filter_drawer)
        flap_box = ctx.part_world_aabb(latch_flap)
        closed_flush = (
            body_box is not None
            and door_box is not None
            and drawer_box is not None
            and flap_box is not None
            and body_box[1][0] - 0.020 <= door_box[1][0] <= body_box[1][0] + 0.030
            and body_box[1][0] - 0.020 <= drawer_box[1][0] <= body_box[1][0] + 0.015
            and body_box[1][0] - 0.020 <= flap_box[1][0] <= body_box[1][0] + 0.025
        )
        ctx.check(
            "closed door drawer and flap sit near the front plane",
            closed_flush,
            details=f"body={body_box}, door={door_box}, drawer={drawer_box}, flap={flap_box}",
        )
        ctx.expect_within(
            filter_drawer,
            body,
            axes="yz",
            margin=0.040,
            name="closed filter drawer stays within the lower housing footprint",
        )

    rest_drawer_pos = ctx.part_world_position(filter_drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        extended_drawer_pos = ctx.part_world_position(filter_drawer)
        ctx.expect_within(
            filter_drawer,
            body,
            axes="yz",
            margin=0.040,
            name="extended filter drawer remains aligned with the guide bay",
        )
        ctx.expect_overlap(
            filter_drawer,
            body,
            axes="x",
            min_overlap=0.040,
            name="extended filter drawer retains insertion in the housing",
        )
    ctx.check(
        "filter drawer slides outward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.080,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_door_box = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE}):
        open_door_box = ctx.part_world_aabb(door)
    ctx.check(
        "front door swings outward on its vertical hinge",
        closed_door_box is not None
        and open_door_box is not None
        and open_door_box[1][0] > closed_door_box[1][0] + 0.120,
        details=f"closed={closed_door_box}, open={open_door_box}",
    )

    closed_flap_box = ctx.part_world_aabb(latch_flap)
    with ctx.pose({flap_hinge: FLAP_OPEN_ANGLE}):
        open_flap_box = ctx.part_world_aabb(latch_flap)
    ctx.check(
        "top latch flap rotates upward and outward",
        closed_flap_box is not None
        and open_flap_box is not None
        and open_flap_box[1][0] > closed_flap_box[1][0] + 0.020
        and open_flap_box[0][2] > closed_flap_box[0][2] + 0.006,
        details=f"closed={closed_flap_box}, open={open_flap_box}",
    )

    return ctx.report()


object_model = build_object_model()
