from __future__ import annotations

from math import cos, pi, sin

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


BODY_WIDTH = 0.42
BODY_HEIGHT = 0.42
BODY_DEPTH = 0.18
OPENING_WIDTH = 0.34
OPENING_HEIGHT = 0.34
FRAME_FLANGE_WIDTH = 0.45
FRAME_FLANGE_HEIGHT = 0.45
FRAME_FLANGE_DEPTH = 0.010
DOOR_WIDTH = 0.334
DOOR_HEIGHT = 0.334
DOOR_THICKNESS = 0.055
DOOR_FACE_GAP = 0.002
SIDE_GAP = 0.003
DOOR_CENTER_Y = FRAME_FLANGE_DEPTH + DOOR_FACE_GAP + DOOR_THICKNESS / 2.0
CONTROL_MOUNT_Y = DOOR_THICKNESS / 2.0
DIAL_X = -0.168
DIAL_Z = 0.060
HANDLE_X = -0.168
HANDLE_Z = -0.074
KEY_COVER_HINGE_X = -0.116
KEY_COVER_Z = 0.030


def safe_body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, -BODY_DEPTH / 2.0, 0.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(OPENING_WIDTH, BODY_DEPTH - 0.04, OPENING_HEIGHT)
        .translate((0.0, -(BODY_DEPTH - 0.04) / 2.0, 0.0))
    )
    flange_outer = (
        cq.Workplane("XY")
        .box(FRAME_FLANGE_WIDTH, FRAME_FLANGE_DEPTH, FRAME_FLANGE_HEIGHT)
        .translate((0.0, FRAME_FLANGE_DEPTH / 2.0, 0.0))
    )
    flange_inner = (
        cq.Workplane("XY")
        .box(OPENING_WIDTH - 0.002, FRAME_FLANGE_DEPTH + 0.002, OPENING_HEIGHT - 0.002)
        .translate((0.0, FRAME_FLANGE_DEPTH / 2.0, 0.0))
    )
    return shell.cut(cavity).union(flange_outer.cut(flange_inner))


def safe_door_shape() -> cq.Workplane:
    slab = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)
        .translate((-DOOR_WIDTH / 2.0, 0.0, 0.0))
    )
    return slab


def handle_shape() -> cq.Workplane:
    thickness = 0.016
    hub_radius = 0.018
    collar_radius = 0.028
    spoke_length = 0.036
    spoke_width = 0.010
    grip_radius = 0.007

    handle = cq.Workplane("XZ").circle(hub_radius).extrude(thickness)
    handle = handle.union(cq.Workplane("XZ").circle(collar_radius).extrude(0.006))

    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(spoke_length, thickness * 0.55, spoke_width)
            .translate((hub_radius + spoke_length / 2.0, thickness * 0.275, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        grip = (
            cq.Workplane("XZ")
            .center(hub_radius + spoke_length, 0.0)
            .circle(grip_radius)
            .extrude(thickness * 0.9)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        handle = handle.union(spoke).union(grip)

    return handle


def dial_shape() -> cq.Workplane:
    rim = cq.Workplane("XZ").circle(0.035).extrude(0.010)
    face = cq.Workplane("XZ").circle(0.029).extrude(0.016)
    hub = cq.Workplane("XZ").circle(0.006).extrude(0.021)
    pointer_hole = (
        cq.Workplane("XY")
        .box(0.008, 0.003, 0.002)
        .translate((0.0, 0.0145, 0.026))
    )
    return rim.union(face).union(hub).cut(pointer_hole)


def key_cover_shape() -> cq.Workplane:
    width = 0.040
    height = 0.026
    thickness = 0.005
    knuckle_radius = 0.0023

    panel = (
        cq.Workplane("XY")
        .box(width, thickness, height)
        .translate((width / 2.0, thickness / 2.0, 0.0))
    )
    pull_lip = (
        cq.Workplane("XY")
        .box(0.004, 0.0025, height * 0.50)
        .translate((width + 0.0015, thickness * 0.70, 0.0))
    )
    lower_knuckle = (
        cq.Workplane("XY")
        .center(0.0, knuckle_radius)
        .circle(knuckle_radius)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.012))
    )
    upper_knuckle = (
        cq.Workplane("XY")
        .center(0.0, knuckle_radius)
        .circle(knuckle_radius)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.004))
    )

    return panel.union(pull_lip).union(lower_knuckle).union(upper_knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_safe")

    body_steel = model.material("body_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    door_steel = model.material("door_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    bright_metal = model.material("bright_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    cover_steel = model.material("cover_steel", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(safe_body_shape(), "body_shell"),
        material=body_steel,
        name="body_shell",
    )
    for index, z_center in enumerate((-0.125, 0.0, 0.125)):
        body.visual(
            Box((0.012, DOOR_CENTER_Y, 0.080)),
            origin=Origin(
                xyz=(OPENING_WIDTH / 2.0 - SIDE_GAP + 0.018, DOOR_CENTER_Y / 2.0, z_center)
            ),
            material=body_steel,
            name=f"hinge_lug_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(safe_door_shape(), "door_slab"),
        material=door_steel,
        name="door_slab",
    )
    door.visual(
        Box((0.014, 0.002, 0.004)),
        origin=Origin(xyz=(DIAL_X, CONTROL_MOUNT_Y + 0.001, DIAL_Z + 0.040)),
        material=bright_metal,
        name="dial_pointer",
    )
    for index, z_center in enumerate((-0.125, 0.0, 0.125)):
        door.visual(
            Box((0.012, DOOR_THICKNESS, 0.080)),
            origin=Origin(xyz=(0.006, 0.0, z_center)),
            material=door_steel,
            name=f"hinge_leaf_{index}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(dial_shape(), "dial_body"),
        origin=Origin(xyz=(0.0, 0.021, 0.0)),
        material=bright_metal,
        name="dial_body",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="handle_body",
    )
    handle.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="handle_collar",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        handle.visual(
            Box((0.040, 0.008, 0.010)),
            origin=Origin(
                xyz=(cos(angle) * 0.034, 0.004, sin(angle) * 0.034),
                rpy=(0.0, angle, 0.0),
            ),
            material=bright_metal,
            name=f"handle_spoke_{index}",
        )

    key_cover = model.part("key_cover")
    key_cover.visual(
        Box((0.040, 0.0045, 0.026)),
        origin=Origin(xyz=(0.020, 0.00225, 0.0)),
        material=cover_steel,
        name="key_cover",
    )
    key_cover.visual(
        Box((0.004, 0.002, 0.014)),
        origin=Origin(xyz=(0.042, 0.001, 0.0)),
        material=cover_steel,
        name="cover_lip",
    )
    for index, z_center in enumerate((-0.009, 0.009)):
        key_cover.visual(
            Cylinder(radius=0.0023, length=0.008),
            origin=Origin(xyz=(0.0, 0.004, z_center), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=cover_steel,
            name=f"cover_knuckle_{index}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 - SIDE_GAP, DOOR_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.8, effort=40.0, velocity=1.2),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_X, CONTROL_MOUNT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(HANDLE_X, CONTROL_MOUNT_Y, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.95, upper=0.95, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cover,
        origin=Origin(xyz=(KEY_COVER_HINGE_X, CONTROL_MOUNT_Y, KEY_COVER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=1.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    key_cover = object_model.get_part("key_cover")
    door_hinge = object_model.get_articulation("door_hinge")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.001,
        max_gap=0.006,
        elem_a="door_slab",
        elem_b="body_shell",
        name="door stands visibly proud of the frame ring",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.30,
        elem_a="door_slab",
        elem_b="body_shell",
        name="closed door covers the safe opening",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="hinge_leaf_1",
        elem_b="hinge_lug_1",
        name="hinge hardware visibly ties the door to the body",
    )
    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_body",
        elem_b="door_slab",
        name="combination dial mounts on the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_body",
        elem_b="door_slab",
        name="handle hub mounts on the door face",
    )
    ctx.expect_contact(
        key_cover,
        door,
        elem_a="key_cover",
        elem_b="door_slab",
        name="key override cover sits on the door face",
    )
    ctx.expect_origin_gap(
        dial,
        handle,
        axis="z",
        min_gap=0.11,
        max_gap=0.16,
        name="dial sits above the handle",
    )
    ctx.expect_origin_gap(
        key_cover,
        dial,
        axis="x",
        min_gap=0.035,
        max_gap=0.080,
        name="key override cover sits beside the dial",
    )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings outward on the right hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > rest_aabb[1][1] + 0.12,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(key_cover)
    with ctx.pose({cover_hinge: cover_hinge.motion_limits.upper}):
        open_cover_aabb = ctx.part_world_aabb(key_cover)

    ctx.check(
        "key cover swings outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.015,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
