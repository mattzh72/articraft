from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_WIDTH = 0.40
BODY_HEIGHT = 0.40
BODY_DEPTH = 0.14
FRAME_LIP = 0.008
OPENING_WIDTH = 0.364
OPENING_HEIGHT = 0.364
BACK_WALL = 0.010

DOOR_WIDTH = 0.344
DOOR_HEIGHT = 0.356
DOOR_THICKNESS = 0.050
HINGE_PANEL_OFFSET = 0.009
HINGE_PIN_RADIUS = 0.0045
HINGE_BARREL_RADIUS = 0.010
HINGE_PIN_LENGTH = 0.360
HINGE_BARREL_LENGTH = 0.300
DOOR_HINGE_X = OPENING_WIDTH / 2.0 - HINGE_BARREL_RADIUS
DOOR_HINGE_Y = -(BODY_DEPTH / 2.0 + FRAME_LIP + DOOR_THICKNESS / 2.0)

DOOR_CENTER_X = -(DOOR_WIDTH / 2.0 + HINGE_PANEL_OFFSET)
DIAL_Z = 0.060
HANDLE_Z = -0.058
FLAP_CENTER_Z = 0.000
FLAP_WIDTH = 0.112
FLAP_HEIGHT = 0.046
FLAP_RECESS_DEPTH = 0.010
FLAP_SLOT_WIDTH = 0.090
FLAP_SLOT_HEIGHT = 0.014
DIAL_SHAFT_RADIUS = 0.008
HANDLE_SHAFT_RADIUS = 0.010
FLAP_THICKNESS = 0.010


def _build_body_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
    front_frame = cq.Workplane("XY").box(
        BODY_WIDTH,
        FRAME_LIP,
        BODY_HEIGHT,
    ).translate((0.0, -BODY_DEPTH / 2.0 - FRAME_LIP / 2.0, 0.0))
    cavity = cq.Workplane("XY").box(
        OPENING_WIDTH,
        BODY_DEPTH + FRAME_LIP - BACK_WALL,
        OPENING_HEIGHT,
    ).translate((0.0, -(FRAME_LIP + BACK_WALL) / 2.0, 0.0))
    return outer.union(front_frame).cut(cavity)


def _build_body_hinge_shape() -> cq.Workplane:
    hinge_pin = (
        cq.Workplane("XY")
        .circle(HINGE_PIN_RADIUS)
        .extrude(HINGE_PIN_LENGTH)
        .translate((DOOR_HINGE_X, DOOR_HINGE_Y, -HINGE_PIN_LENGTH / 2.0))
    )
    anchor_y = (-BODY_DEPTH / 2.0 - FRAME_LIP + DOOR_HINGE_Y) / 2.0
    upper_anchor = cq.Workplane("XY").box(
        0.028,
        0.030,
        0.036,
    ).translate((DOOR_HINGE_X + 0.008, anchor_y, 0.162))
    lower_anchor = cq.Workplane("XY").box(
        0.028,
        0.030,
        0.036,
    ).translate((DOOR_HINGE_X + 0.008, anchor_y, -0.162))
    return hinge_pin.union(upper_anchor).union(lower_anchor)


def _build_door_panel_shape() -> cq.Workplane:
    door = cq.Workplane("XY").box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT).translate(
        (DOOR_CENTER_X, 0.0, 0.0)
    )
    dial_bore = (
        cq.Workplane("XZ")
        .center(DOOR_CENTER_X, DIAL_Z)
        .circle(DIAL_SHAFT_RADIUS + 0.001)
        .extrude(0.012)
        .translate((0.0, -DOOR_THICKNESS / 2.0, 0.0))
    )
    handle_bore = (
        cq.Workplane("XZ")
        .center(DOOR_CENTER_X, HANDLE_Z)
        .circle(HANDLE_SHAFT_RADIUS + 0.001)
        .extrude(0.014)
        .translate((0.0, -DOOR_THICKNESS / 2.0, 0.0))
    )
    flap_recess = cq.Workplane("XY").box(
        FLAP_WIDTH + 0.010,
        FLAP_RECESS_DEPTH,
        FLAP_HEIGHT + 0.010,
    ).translate(
        (
            DOOR_CENTER_X,
            -DOOR_THICKNESS / 2.0 + FLAP_RECESS_DEPTH / 2.0,
            FLAP_CENTER_Z,
        )
    )
    flap_slot = (
        cq.Workplane("XZ")
        .center(DOOR_CENTER_X, FLAP_CENTER_Z - 0.003)
        .rect(FLAP_SLOT_WIDTH, FLAP_SLOT_HEIGHT)
        .extrude(0.018)
        .translate((0.0, -DOOR_THICKNESS / 2.0, 0.0))
    )
    return door.cut(dial_bore).cut(handle_bore).cut(flap_recess).cut(flap_slot)


def _build_door_hinge_shape() -> cq.Workplane:
    hinge_barrel = (
        cq.Workplane("XY")
        .circle(HINGE_BARREL_RADIUS)
        .extrude(HINGE_BARREL_LENGTH)
        .translate((0.0, 0.0, -HINGE_BARREL_LENGTH / 2.0))
    )
    hinge_bore = (
        cq.Workplane("XY")
        .circle(HINGE_PIN_RADIUS)
        .extrude(HINGE_BARREL_LENGTH + 0.002)
        .translate((0.0, 0.0, -(HINGE_BARREL_LENGTH + 0.002) / 2.0))
    )
    return hinge_barrel.cut(hinge_bore)


def _build_dial_shape() -> cq.Workplane:
    outer_ring = (
        cq.Workplane("XZ")
        .circle(0.026)
        .extrude(0.010)
        .translate((0.0, -0.016, 0.0))
    )
    inner_ring = (
        cq.Workplane("XZ")
        .circle(0.022)
        .extrude(0.008)
        .translate((0.0, -0.010, 0.0))
    )
    center_cap = (
        cq.Workplane("XZ")
        .circle(0.014)
        .extrude(0.006)
        .translate((0.0, -0.004, 0.0))
    )
    shaft = cq.Workplane("XZ").circle(DIAL_SHAFT_RADIUS).extrude(0.010)
    return outer_ring.union(inner_ring).union(center_cap).union(shaft)


def _build_handle_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .circle(0.021)
        .extrude(0.010)
        .translate((0.0, -0.014, 0.0))
    )
    hub_cap = (
        cq.Workplane("XZ")
        .circle(0.013)
        .extrude(0.004)
        .translate((0.0, -0.004, 0.0))
    )
    shaft = cq.Workplane("XZ").circle(HANDLE_SHAFT_RADIUS).extrude(0.012)

    handle = hub.union(hub_cap).union(shaft)
    for angle_deg in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("YZ")
            .center(-0.011, 0.0)
            .circle(0.0038)
            .extrude(0.052)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        )
        grip = (
            cq.Workplane("YZ")
            .center(-0.011, 0.0)
            .circle(0.006)
            .extrude(0.015)
            .translate((0.044, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        )
        handle = handle.union(spoke).union(grip)
    return handle


def _build_flap_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(FLAP_WIDTH, FLAP_THICKNESS, FLAP_HEIGHT).translate(
        (0.0, 0.0, -FLAP_HEIGHT / 2.0)
    )
    pull_lip = cq.Workplane("XY").box(
        FLAP_WIDTH * 0.80,
        0.006,
        0.008,
    ).translate((0.0, -0.001, -FLAP_HEIGHT + 0.004))
    return panel.union(pull_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_safe")

    body_finish = model.material("body_finish", rgba=(0.19, 0.21, 0.23, 1.0))
    door_finish = model.material("door_finish", rgba=(0.28, 0.29, 0.31, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.72, 0.73, 0.74, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.82, 0.79, 0.63, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell_shape(), "safe_body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_body_hinge_shape(), "safe_body_hinge"),
        material=body_finish,
        name="hinge_post",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_panel_shape(), "safe_door_panel"),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        mesh_from_cadquery(_build_door_hinge_shape(), "safe_door_hinge"),
        material=door_finish,
        name="door_hinge",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial_shape(), "safe_dial"),
        material=dial_finish,
        name="dial_body",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "safe_handle"),
        material=hardware_finish,
        name="handle_body",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap_shape(), "safe_flap"),
        material=door_finish,
        name="flap_panel",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=40.0, velocity=1.0),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DOOR_CENTER_X, -DOOR_THICKNESS / 2.0, DIAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(DOOR_CENTER_X, -DOOR_THICKNESS / 2.0, HANDLE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=0.75, effort=8.0, velocity=4.0),
    )
    model.articulation(
        "door_to_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=flap,
        origin=Origin(
            xyz=(
                DOOR_CENTER_X,
                -DOOR_THICKNESS / 2.0 + FLAP_RECESS_DEPTH / 2.0,
                FLAP_CENTER_Z + FLAP_HEIGHT / 2.0,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=4.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    flap = object_model.get_part("flap")
    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_spin = object_model.get_articulation("door_to_handle")
    flap_hinge = object_model.get_articulation("door_to_flap")
    door_limits = door_hinge.motion_limits
    handle_limits = handle_spin.motion_limits
    flap_limits = flap_hinge.motion_limits

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_post",
        elem_b="door_hinge",
        reason="The body-mounted hinge pin intentionally occupies the door hinge barrel.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_within(
            door,
            body,
            axes="xz",
            margin=0.024,
            name="door stays inside the face frame silhouette",
        )
        ctx.expect_contact(dial, door, contact_tol=0.001, name="dial stays mounted to the door")
        ctx.expect_contact(handle, door, contact_tol=0.001, name="handle stays mounted to the door")
        ctx.expect_contact(flap, door, contact_tol=0.001, name="deposit flap seats in the door face")

    if door_limits is not None and door_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        outward = (
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < rest_aabb[0][1] - 0.08
        )
        ctx.check(
            "door opens outward from the safe body",
            outward,
            details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
        )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_spin: pi / 3.0}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place on its shaft",
        dial_rest is not None and dial_turned is not None and dial_rest == dial_turned,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    if handle_limits is not None and handle_limits.upper is not None:
        handle_rest = ctx.part_world_aabb(handle)
        with ctx.pose({handle_spin: handle_limits.upper}):
            handle_turned = ctx.part_world_aabb(handle)
        ctx.check(
            "handle rotates on the hub",
            handle_rest is not None and handle_turned is not None and handle_rest != handle_turned,
            details=f"rest={handle_rest}, turned={handle_turned}",
        )

    if flap_limits is not None and flap_limits.upper is not None:
        flap_rest = ctx.part_world_aabb(flap)
        with ctx.pose({flap_hinge: flap_limits.upper}):
            flap_open = ctx.part_world_aabb(flap)
        ctx.check(
            "deposit flap lifts outward from the door face",
            flap_rest is not None
            and flap_open is not None
            and flap_open[0][1] < flap_rest[0][1] - 0.015,
            details=f"rest={flap_rest}, open={flap_open}",
        )

    return ctx.report()


object_model = build_object_model()
