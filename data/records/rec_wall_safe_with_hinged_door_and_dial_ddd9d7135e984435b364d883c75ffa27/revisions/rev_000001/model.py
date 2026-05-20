from __future__ import annotations

import math

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


BODY_W = 0.35
BODY_H = 0.72
BODY_D = 0.22
BODY_WALL = 0.012

TRIM_W = 0.41
TRIM_H = 0.78
TRIM_T = 0.012

OPEN_W = 0.29
OPEN_H = 0.66

DOOR_W = 0.33
DOOR_H = 0.70
DOOR_T = 0.03
DOOR_FRONT_RECESS = 0.004

HINGE_BARREL_R = 0.009
HINGE_BARREL_L = 0.11
HINGE_BARREL_Z = (-0.24, 0.0, 0.24)

DIAL_X = 0.205
DIAL_Z = 0.115
DIAL_R = 0.038
DIAL_DEPTH = 0.02

HANDLE_X = 0.215
HANDLE_Z = -0.05
HANDLE_HUB_R = 0.02
HANDLE_HUB_DEPTH = 0.014
HANDLE_ARM_L = 0.072
HANDLE_ARM_D = 0.012
HANDLE_ARM_H = 0.014
HANDLE_GRIP_R = 0.008
HANDLE_GRIP_L = 0.018

FLAP_SLOT_X = 0.18
FLAP_SLOT_Z = 0.225
FLAP_OPEN_W = 0.205
FLAP_OPEN_H = 0.05
FLAP_COVER_W = 0.22
FLAP_COVER_H = 0.066
FLAP_T = 0.008
FLAP_HINGE_R = 0.006


def _body_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0.0, -BODY_D / 2.0, 0.0))
    cavity = (
        cq.Workplane("XY")
        .box(BODY_W - (2.0 * BODY_WALL), BODY_D - BODY_WALL, BODY_H - (2.0 * BODY_WALL))
        .translate((0.0, -(BODY_D - BODY_WALL) / 2.0, 0.0))
    )
    trim = cq.Workplane("XY").box(TRIM_W, TRIM_T, TRIM_H).translate((0.0, TRIM_T / 2.0, 0.0))
    trim_opening = cq.Workplane("XY").box(OPEN_W, TRIM_T + 0.002, OPEN_H).translate((0.0, TRIM_T / 2.0, 0.0))

    return shell.cut(cavity).union(trim.cut(trim_opening))


def _door_shape() -> cq.Workplane:
    door = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H).translate((DOOR_W / 2.0, DOOR_T / 2.0, 0.0))

    deposit_opening = (
        cq.Workplane("XY")
        .box(FLAP_OPEN_W, DOOR_T + 0.002, FLAP_OPEN_H)
        .translate((FLAP_SLOT_X, DOOR_T / 2.0, FLAP_SLOT_Z))
    )
    door = door.cut(deposit_opening)

    for z_center in HINGE_BARREL_Z:
        barrel = (
            cq.Workplane("XY")
            .circle(HINGE_BARREL_R)
            .extrude(HINGE_BARREL_L)
            .translate((0.0, HINGE_BARREL_R, z_center - (HINGE_BARREL_L / 2.0)))
        )
        door = door.union(barrel)

    return door


def _dial_shape() -> cq.Workplane:
    dial = cq.Workplane("XY").circle(DIAL_R).extrude(DIAL_DEPTH).rotate((0, 0, 0), (1, 0, 0), -90)
    cap = (
        cq.Workplane("XY")
        .circle(DIAL_R * 0.58)
        .extrude(DIAL_DEPTH + 0.004)
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )
    pointer = (
        cq.Workplane("XY")
        .box(0.012, 0.004, 0.012)
        .translate((0.0, DIAL_DEPTH - 0.0015, DIAL_R + 0.006))
    )
    return dial.union(cap).union(pointer)


def _handle_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(HANDLE_HUB_R)
        .extrude(HANDLE_HUB_DEPTH)
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )
    boss = (
        cq.Workplane("XY")
        .circle(HANDLE_HUB_R * 0.55)
        .extrude(HANDLE_HUB_DEPTH + 0.006)
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )
    arm = (
        cq.Workplane("XY")
        .box(HANDLE_ARM_L, HANDLE_ARM_D, HANDLE_ARM_H)
        .translate((HANDLE_ARM_L / 2.0, HANDLE_HUB_DEPTH * 0.65, 0.0))
    )
    grip = (
        cq.Workplane("XY")
        .circle(HANDLE_GRIP_R)
        .extrude(HANDLE_GRIP_L)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((HANDLE_ARM_L - (HANDLE_GRIP_L * 0.4), HANDLE_HUB_DEPTH * 0.65, 0.0))
    )
    return hub.union(boss).union(arm).union(grip)


def _flap_shape() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .box(FLAP_COVER_W, FLAP_T, FLAP_COVER_H)
        .translate((0.0, FLAP_T / 2.0, -(FLAP_COVER_H / 2.0)))
    )
    hinge = (
        cq.Workplane("XY")
        .circle(FLAP_HINGE_R)
        .extrude(FLAP_COVER_W - 0.02)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((-(FLAP_COVER_W - 0.02) / 2.0, FLAP_HINGE_R, 0.0))
    )
    hem = (
        cq.Workplane("XY")
        .box(FLAP_COVER_W - 0.02, FLAP_T * 0.75, 0.008)
        .translate((0.0, (FLAP_T * 0.75) / 2.0, -FLAP_COVER_H + 0.004))
    )
    return cover.union(hinge).union(hem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    body_finish = model.material("body_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    door_finish = model.material("door_finish", rgba=(0.31, 0.32, 0.35, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.11, 0.12, 0.14, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "safe_body"), material=body_finish, name="shell")

    door = model.part("door")
    door.visual(mesh_from_cadquery(_door_shape(), "safe_door"), material=door_finish, name="panel")

    dial = model.part("dial")
    dial.visual(mesh_from_cadquery(_dial_shape(), "safe_dial"), material=dial_finish, name="dial_face")

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_handle_shape(), "safe_handle"), material=metal_finish, name="lever")

    flap = model.part("flap")
    flap.visual(mesh_from_cadquery(_flap_shape(), "deposit_flap"), material=door_finish, name="cover")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-(OPEN_W / 2.0) - ((DOOR_W - OPEN_W) / 2.0), TRIM_T, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.8, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DOOR_T, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(HANDLE_X, DOOR_T, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=0.85, effort=6.0, velocity=3.0),
    )
    model.articulation(
        "door_to_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=flap,
        origin=Origin(xyz=(FLAP_SLOT_X, DOOR_T, FLAP_SLOT_Z + (FLAP_COVER_H / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=3.0, velocity=2.0),
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
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_handle")
    flap_hinge = object_model.get_articulation("door_to_flap")

    with ctx.pose({door_hinge: 0.0, handle_joint: 0.0, flap_hinge: 0.0, dial_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="door seats against the front trim",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.30,
            name="door covers the tall safe opening",
        )
        ctx.expect_gap(
            dial,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="dial mounts on the door face",
        )
        ctx.expect_gap(
            handle,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="lever handle mounts on the door face",
        )
        ctx.expect_gap(
            flap,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="deposit flap closes against the door face",
        )
        ctx.expect_origin_gap(
            dial,
            handle,
            axis="z",
            min_gap=0.12,
            name="dial sits above the short lever handle",
        )
        ctx.expect_origin_gap(
            flap,
            handle,
            axis="z",
            min_gap=0.22,
            name="deposit flap sits high on the door panel",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: 0.0}):
            closed_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "main door swings outward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.18,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    flap_limits = flap_hinge.motion_limits
    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
            flap_closed_aabb = ctx.part_world_aabb(flap)
        with ctx.pose({door_hinge: 0.0, flap_hinge: flap_limits.upper}):
            flap_open_aabb = ctx.part_world_aabb(flap)
        ctx.check(
            "deposit flap opens on its top hinge",
            flap_closed_aabb is not None
            and flap_open_aabb is not None
            and flap_open_aabb[1][1] > flap_closed_aabb[1][1] + 0.035,
            details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
        )

    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({door_hinge: 0.0, handle_joint: handle_limits.lower}):
            handle_low_aabb = ctx.part_world_aabb(handle)
        with ctx.pose({door_hinge: 0.0, handle_joint: handle_limits.upper}):
            handle_high_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "lever handle rotates on its spindle",
            handle_low_aabb is not None
            and handle_high_aabb is not None
            and handle_low_aabb[1][2] > handle_high_aabb[1][2] + 0.04,
            details=f"lower={handle_low_aabb}, upper={handle_high_aabb}",
        )

    with ctx.pose({door_hinge: 0.0, dial_joint: 0.0}):
        dial_zero_aabb = ctx.part_world_aabb(dial)
    with ctx.pose({door_hinge: 0.0, dial_joint: math.pi / 2.0}):
        dial_quarter_aabb = ctx.part_world_aabb(dial)
    ctx.check(
        "dial rotates continuously about its center axis",
        dial_zero_aabb is not None
        and dial_quarter_aabb is not None
        and dial_zero_aabb[1][2] > dial_quarter_aabb[1][2] + 0.004
        and dial_quarter_aabb[1][0] > dial_zero_aabb[1][0] + 0.004,
        details=f"zero={dial_zero_aabb}, quarter={dial_quarter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
