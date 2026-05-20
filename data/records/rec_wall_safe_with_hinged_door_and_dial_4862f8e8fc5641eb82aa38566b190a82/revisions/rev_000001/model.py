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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SAFE_WIDTH = 0.40
SAFE_HEIGHT = 0.40
SAFE_DEPTH = 0.22
BACK_THICKNESS = 0.014
CAVITY_WIDTH = 0.34
CAVITY_HEIGHT = 0.34
FRAME_SIZE = 0.46
FRAME_PROUD = 0.012

DOOR_WIDTH = 0.335
DOOR_HEIGHT = 0.335
DOOR_THICKNESS = 0.055

DIAL_RADIUS = 0.032
DIAL_DEPTH = 0.018
DIAL_CAP_RADIUS = 0.015
DIAL_CAP_DEPTH = 0.006

HANDLE_HUB_RADIUS = 0.021
HANDLE_HUB_LENGTH = 0.024
HANDLE_SPOKE_LENGTH = 0.050
HANDLE_SPOKE_THICKNESS = 0.008
HANDLE_SPOKE_HEIGHT = 0.010
HANDLE_GRIP_RADIUS = 0.009

HINGE_RADIUS = 0.006
HINGE_LENGTH = 0.31

TRAY_DEPTH = 0.13
TRAY_WIDTH = 0.286
TRAY_HEIGHT = 0.045
TRAY_WALL = 0.004
TRAY_TRAVEL = 0.05

RUNNER_LENGTH = 0.12
RUNNER_WIDTH = 0.032
RUNNER_HEIGHT = 0.008
RUNNER_CENTER_X = -0.12
RUNNER_CENTER_Y = 0.156
RUNNER_TOP_Z = -0.03

TRAY_REAR_X = -0.18
TRAY_BOTTOM_Z = RUNNER_TOP_Z

DIAL_Z = 0.055
HANDLE_Z = -0.055


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def _body_shell_mesh():
    inner_depth = SAFE_DEPTH - BACK_THICKNESS
    shell = (
        cq.Workplane("XY")
        .box(SAFE_DEPTH, SAFE_WIDTH, SAFE_HEIGHT)
        .translate((-SAFE_DEPTH / 2.0, 0.0, 0.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(inner_depth, CAVITY_WIDTH, CAVITY_HEIGHT)
        .translate((-inner_depth / 2.0, 0.0, 0.0))
    )
    face_frame = (
        cq.Workplane("XY")
        .box(FRAME_PROUD, FRAME_SIZE, FRAME_SIZE)
        .translate((FRAME_PROUD / 2.0, 0.0, 0.0))
    )
    face_opening = (
        cq.Workplane("XY")
        .box(FRAME_PROUD + 0.002, CAVITY_WIDTH, CAVITY_HEIGHT)
        .translate((FRAME_PROUD / 2.0, 0.0, 0.0))
    )
    return mesh_from_cadquery(shell.cut(cavity).union(face_frame.cut(face_opening)), "body_shell")


def _door_panel_mesh():
    panel = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT, centered=(False, False, True))
    )
    panel = (
        panel.faces(">X")
        .workplane()
        .rect(DOOR_WIDTH - 0.070, DOOR_HEIGHT - 0.070)
        .cutBlind(-0.004)
    )
    return mesh_from_cadquery(panel, "door_panel")


def _tray_shell_mesh():
    tray = (
        cq.Workplane("XY")
        .box(TRAY_DEPTH, TRAY_WIDTH, TRAY_HEIGHT, centered=(False, True, False))
    )
    tray = (
        tray.faces(">Z")
        .workplane()
        .rect(TRAY_DEPTH - 2.0 * TRAY_WALL, TRAY_WIDTH - 2.0 * TRAY_WALL)
        .cutBlind(-(TRAY_HEIGHT - TRAY_WALL))
    )
    return mesh_from_cadquery(tray, "tray_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_safe")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    door_finish = model.material("door_finish", rgba=(0.11, 0.12, 0.13, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.68, 0.69, 0.71, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.33, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_finish, name="body_shell")
    for index, side in enumerate((1.0, -1.0)):
        body.visual(
            Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
            origin=Origin(
                xyz=(
                    RUNNER_CENTER_X,
                    side * RUNNER_CENTER_Y,
                    RUNNER_TOP_Z - RUNNER_HEIGHT / 2.0,
                )
            ),
            material=body_finish,
            name=f"runner_{index}",
        )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LENGTH),
        origin=Origin(xyz=(FRAME_PROUD - HINGE_RADIUS, -DOOR_WIDTH / 2.0, 0.0)),
        material=steel_finish,
        name="body_hinge_barrel",
    )

    door = model.part("door")
    door.visual(_door_panel_mesh(), material=door_finish, name="door_panel")
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LENGTH),
        origin=Origin(xyz=(HINGE_RADIUS, 0.0, 0.0)),
        material=steel_finish,
        name="door_hinge_barrel",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_DEPTH),
        origin=Origin(xyz=(DIAL_DEPTH / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_finish,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=DIAL_CAP_RADIUS, length=DIAL_CAP_DEPTH),
        origin=Origin(
            xyz=(DIAL_DEPTH - DIAL_CAP_DEPTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_finish,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(DIAL_DEPTH - 0.002, 0.0, DIAL_RADIUS - 0.006)),
        material=steel_finish,
        name="dial_marker",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_HUB_RADIUS, length=HANDLE_HUB_LENGTH),
        origin=Origin(
            xyz=(HANDLE_HUB_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_finish,
        name="hub",
    )
    spoke_center_radius = HANDLE_HUB_RADIUS + HANDLE_SPOKE_LENGTH / 2.0
    grip_radius = HANDLE_HUB_RADIUS + HANDLE_SPOKE_LENGTH
    for index, angle in enumerate(
        (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    ):
        cy = spoke_center_radius * math.cos(angle)
        cz = spoke_center_radius * math.sin(angle)
        gy = grip_radius * math.cos(angle)
        gz = grip_radius * math.sin(angle)
        handle.visual(
            Box((HANDLE_SPOKE_THICKNESS, HANDLE_SPOKE_LENGTH, HANDLE_SPOKE_HEIGHT)),
            origin=Origin(
                xyz=(HANDLE_HUB_LENGTH / 2.0, cy, cz),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel_finish,
            name=f"spoke_{index}",
        )
        handle.visual(
            Sphere(radius=HANDLE_GRIP_RADIUS),
            origin=Origin(xyz=(HANDLE_HUB_LENGTH / 2.0, gy, gz)),
            material=steel_finish,
            name=f"grip_{index}",
        )

    tray = model.part("tray")
    tray.visual(_tray_shell_mesh(), material=tray_finish, name="tray_shell")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(FRAME_PROUD, -DOOR_WIDTH / 2.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.9, effort=25.0, velocity=1.0),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DOOR_THICKNESS, DOOR_WIDTH / 2.0, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(DOOR_THICKNESS, DOOR_WIDTH / 2.0, HANDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.9, upper=0.9, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(TRAY_REAR_X, 0.0, TRAY_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=12.0, velocity=0.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_turn = object_model.get_articulation("door_to_handle")
    tray_slide = object_model.get_articulation("body_to_tray")

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="tray_shell",
            negative_elem="runner_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="tray sits on upper runner surface 0",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="tray_shell",
            negative_elem="runner_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="tray sits on upper runner surface 1",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_shell",
            elem_b="runner_0",
            min_overlap=0.10,
            name="tray overlaps runner length at rest",
        )

        body_aabb = ctx.part_world_aabb(body)
        tray_aabb = ctx.part_element_world_aabb(tray, elem="tray_shell")
        tray_clears_back = (
            body_aabb is not None
            and tray_aabb is not None
            and tray_aabb[0][0] > body_aabb[0][0] + 0.025
        )
        ctx.check(
            "tray stays off the back wall",
            tray_clears_back,
            details=f"body_aabb={body_aabb}, tray_aabb={tray_aabb}",
        )

    tray_rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="tray_shell",
            negative_elem="runner_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended tray stays on runner 0",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_shell",
            elem_b="runner_0",
            min_overlap=0.065,
            name="extended tray retains insertion on runner 0",
        )
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            elem_a="tray_shell",
            margin=0.03,
            name="tray remains centered inside the safe cavity",
        )
        tray_extended_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray slides forward",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[0] > tray_rest_pos[0] + 0.045,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    closed_aabb = None
    open_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.9}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door opens outward on the right hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    dial_marker_rest = None
    dial_marker_spun = None
    with ctx.pose({dial_spin: 0.0}):
        dial_marker_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    with ctx.pose({dial_spin: 1.0}):
        dial_marker_spun = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    ctx.check(
        "dial marker rotates around the center shaft",
        dial_marker_rest is not None
        and dial_marker_spun is not None
        and math.hypot(
            dial_marker_spun[1] - dial_marker_rest[1],
            dial_marker_spun[2] - dial_marker_rest[2],
        )
        > 0.010,
        details=f"rest={dial_marker_rest}, spun={dial_marker_spun}",
    )

    handle_spoke_rest = None
    handle_spoke_turned = None
    with ctx.pose({handle_turn: 0.0}):
        handle_spoke_rest = _aabb_center(ctx.part_element_world_aabb(handle, elem="spoke_0"))
    with ctx.pose({handle_turn: 0.7}):
        handle_spoke_turned = _aabb_center(ctx.part_element_world_aabb(handle, elem="spoke_0"))
    ctx.check(
        "handle rotates on its hub",
        handle_spoke_rest is not None
        and handle_spoke_turned is not None
        and math.hypot(
            handle_spoke_turned[1] - handle_spoke_rest[1],
            handle_spoke_turned[2] - handle_spoke_rest[2],
        )
        > 0.012,
        details=f"rest={handle_spoke_rest}, turned={handle_spoke_turned}",
    )

    ctx.expect_origin_gap(
        dial,
        handle,
        axis="z",
        min_gap=0.08,
        name="dial stays above the handle",
    )

    return ctx.report()


object_model = build_object_model()
