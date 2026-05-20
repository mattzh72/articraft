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


BODY_W = 0.037
BODY_D = 0.046
BODY_H = 0.090

HEAD_W = 0.072
HEAD_D = 0.039
HEAD_H = 0.033

DOOR_T = 0.0022
DOOR_D = 0.029
DOOR_H = 0.060
DOOR_CENTER_Y = 0.001
DOOR_CENTER_Z = -0.003
DOOR_REAR_Y = DOOR_CENTER_Y - DOOR_D / 2.0

SWIVEL_Z = BODY_H / 2.0 + 0.006
TILT_Y = -0.003
TILT_Z = 0.025
HEAD_CENTER_Y = 0.017
HEAD_CENTER_Z = 0.0035


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(width, depth, height).edges().fillet(radius)


def _build_body_shell() -> cq.Workplane:
    shell = _rounded_box(BODY_W, BODY_D, BODY_H, 0.0032)
    return (
        shell.faces(">X")
        .workplane()
        .center(DOOR_CENTER_Y, DOOR_CENTER_Z)
        .rect(DOOR_D, DOOR_H)
        .cutBlind(-0.013)
    )


def _build_cradle() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.0105).extrude(0.0065)
    neck = cq.Workplane("XY").box(0.022, 0.018, 0.006).translate((0.0, -0.003, 0.009))
    tie_bar = cq.Workplane("XY").box(0.086, 0.006, 0.006).translate((0.0, -0.010, 0.008))
    arm_offset = HEAD_W / 2.0 + 0.0055
    arm_left = cq.Workplane("XY").box(0.003, 0.016, 0.020).translate((-arm_offset, -0.003, 0.015))
    arm_right = cq.Workplane("XY").box(0.003, 0.016, 0.020).translate((arm_offset, -0.003, 0.015))
    return base.union(neck).union(tie_bar).union(arm_left).union(arm_right)


def _build_head_shell() -> cq.Workplane:
    shell = _rounded_box(HEAD_W, HEAD_D, HEAD_H, 0.0025).translate((0.0, HEAD_CENTER_Y, HEAD_CENTER_Z))
    return (
        shell.faces(">Y")
        .workplane()
        .rect(0.061, 0.025)
        .cutBlind(-0.0022)
    )


def _build_battery_door() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DOOR_T, DOOR_D, DOOR_H).translate((-DOOR_T / 2.0, DOOR_D / 2.0, 0.0))
    hinge_spine = (
        cq.Workplane("XY")
        .circle(0.0012)
        .extrude(DOOR_H)
        .translate((-0.0005, 0.0, -DOOR_H / 2.0))
    )
    return panel.union(hinge_spine)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_flash")

    body_plastic = model.material("body_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    diffuser = model.material("diffuser", rgba=(0.95, 0.96, 0.98, 0.95))
    shoe_metal = model.material("shoe_metal", rgba=(0.72, 0.73, 0.76, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shell(), "flash_body_shell"), material=body_plastic, name="body_shell")
    body.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 + 0.003)),
        material=body_plastic,
        name="swivel_collar",
    )
    body.visual(
        Box((0.014, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_H / 2.0 - 0.005)),
        material=rubber,
        name="shoe_stem",
    )
    body.visual(
        Box((0.018, 0.022, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_H / 2.0 - 0.0115)),
        material=shoe_metal,
        name="shoe_mount",
    )

    cradle = model.part("cradle")
    cradle.visual(mesh_from_cadquery(_build_cradle(), "flash_cradle"), material=body_plastic, name="cradle_shell")

    head = model.part("head")
    head.visual(mesh_from_cadquery(_build_head_shell(), "flash_head_shell"), material=body_plastic, name="head_shell")
    head.visual(
        Box((0.060, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, HEAD_CENTER_Y + HEAD_D / 2.0 - 0.0019, HEAD_CENTER_Z)),
        material=diffuser,
        name="flash_window",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        head.visual(
            Cylinder(radius=0.0038, length=0.003),
            origin=Origin(
                xyz=(sign * (HEAD_W / 2.0 + 0.0006), -0.004, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber,
            name=f"trunnion_{index}",
        )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_build_battery_door(), "flash_battery_door"),
        material=body_plastic,
        name="door_panel",
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-1.9, upper=1.9, effort=3.0, velocity=4.0),
    )
    model.articulation(
        "cradle_to_head",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=head,
        origin=Origin(xyz=(0.0, TILT_Y, TILT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.65, effort=2.0, velocity=3.0),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(BODY_W / 2.0, DOOR_REAR_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.8, effort=1.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    head = object_model.get_part("head")
    battery_door = object_model.get_part("battery_door")

    swivel = object_model.get_articulation("body_to_cradle")
    tilt = object_model.get_articulation("cradle_to_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    ctx.expect_gap(head, body, axis="z", min_gap=0.008, name="head clears the top of the body")
    ctx.expect_overlap(head, body, axes="x", min_overlap=0.020, name="head stays centered over the body")
    ctx.expect_overlap(
        battery_door,
        body,
        axes="yz",
        min_overlap=0.020,
        name="battery door sits within the body side footprint when closed",
    )

    head_rest = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    if swivel.motion_limits is not None and swivel.motion_limits.upper is not None:
        with ctx.pose({swivel: swivel.motion_limits.upper}):
            head_swiveled = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
        ctx.check(
            "head swivels sideways",
            head_rest is not None and head_swiveled is not None and head_swiveled[0] > head_rest[0] + 0.005,
            details=f"rest={head_rest}, swiveled={head_swiveled}",
        )

    if tilt.motion_limits is not None and tilt.motion_limits.upper is not None:
        with ctx.pose({tilt: tilt.motion_limits.upper}):
            head_tilted = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
            ctx.expect_gap(head, body, axis="z", min_gap=0.010, name="tilted head still clears the body")
        ctx.check(
            "head tilts upward",
            head_rest is not None and head_tilted is not None and head_tilted[2] > head_rest[2] + 0.012,
            details=f"rest={head_rest}, tilted={head_tilted}",
        )

    door_rest = _aabb_center(ctx.part_element_world_aabb(battery_door, elem="door_panel"))
    if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None:
        with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
            door_open = _aabb_center(ctx.part_element_world_aabb(battery_door, elem="door_panel"))
        ctx.check(
            "battery door swings outward",
            door_rest is not None and door_open is not None and door_open[0] > door_rest[0] + 0.010,
            details=f"closed={door_rest}, open={door_open}",
        )

    return ctx.report()


object_model = build_object_model()
