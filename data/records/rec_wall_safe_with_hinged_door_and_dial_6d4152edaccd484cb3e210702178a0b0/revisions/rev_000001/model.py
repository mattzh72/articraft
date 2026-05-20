from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.47
BODY_DEPTH = 0.20
BODY_HEIGHT = 0.69
BODY_WALL = 0.012

FLANGE_WIDTH = 0.55
FLANGE_HEIGHT = 0.78
FLANGE_DEPTH = 0.035
FLANGE_BACKSET = 0.007

OPENING_WIDTH = 0.41
OPENING_HEIGHT = 0.62

DOOR_WIDTH = 0.45
DOOR_HEIGHT = 0.67
DOOR_THICKNESS = 0.038
DOOR_FRONT_OFFSET = FLANGE_DEPTH - FLANGE_BACKSET

DIAL_X = 0.245
DIAL_Z = 0.125
DIAL_DIAMETER = 0.088
DIAL_DEPTH = 0.020

HANDLE_X = 0.245
HANDLE_Z = -0.055
HANDLE_HUB_RADIUS = 0.018
HANDLE_HUB_DEPTH = 0.010
HANDLE_ARM_LENGTH = 0.078
HANDLE_ARM_WIDTH = 0.012
HANDLE_ARM_HEIGHT = 0.012
HANDLE_GRIP_RADIUS = 0.008
HANDLE_GRIP_LENGTH = 0.032

KEY_COVER_X = 0.292
KEY_COVER_Z = 0.082
KEY_COVER_WIDTH = 0.050
KEY_COVER_HEIGHT = 0.030
KEY_COVER_THICKNESS = 0.008


def build_body_shell_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .cut(
            cq.Workplane("XY").box(
                BODY_WIDTH - 2.0 * BODY_WALL,
                BODY_DEPTH - BODY_WALL,
                BODY_HEIGHT - 2.0 * BODY_WALL,
            ).translate((0.0, BODY_WALL / 2.0, 0.0))
        )
        .translate((0.0, -BODY_DEPTH / 2.0, 0.0))
    )


def build_front_flange_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FLANGE_WIDTH, FLANGE_DEPTH, FLANGE_HEIGHT)
        .cut(
            cq.Workplane("XY").box(
                OPENING_WIDTH,
                FLANGE_DEPTH + 0.01,
                OPENING_HEIGHT,
            )
        )
        .translate((0.0, (FLANGE_DEPTH / 2.0) - FLANGE_BACKSET, 0.0))
    )


def build_door_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT, centered=(False, False, True))
        .edges("|Z")
        .fillet(0.004)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(DOOR_WIDTH - 0.060, DOOR_HEIGHT - 0.080)
        .cutBlind(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    body_paint = model.material("body_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    flange_paint = model.material("flange_paint", rgba=(0.23, 0.24, 0.25, 1.0))
    door_paint = model.material("door_paint", rgba=(0.13, 0.14, 0.15, 1.0))
    hardware_metal = model.material("hardware_metal", rgba=(0.74, 0.75, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shell_shape(), "safe_body_shell"),
        material=body_paint,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(build_front_flange_shape(), "safe_front_flange"),
        material=flange_paint,
        name="front_flange",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(build_door_shape(), "safe_door"),
        material=door_paint,
        name="door_panel",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAL_DIAMETER,
                DIAL_DEPTH,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="knurled", count=42, depth=0.0010, helix_angle_deg=20.0),
                center=False,
            ),
            "safe_dial",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="dial_body",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_HUB_RADIUS, length=HANDLE_HUB_DEPTH),
        origin=Origin(xyz=(0.0, HANDLE_HUB_DEPTH / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="handle_hub",
    )
    handle.visual(
        Box((HANDLE_ARM_LENGTH, HANDLE_ARM_WIDTH, HANDLE_ARM_HEIGHT)),
        origin=Origin(xyz=(HANDLE_ARM_LENGTH / 2.0, HANDLE_HUB_DEPTH * 0.8, 0.0)),
        material=hardware_metal,
        name="handle_arm",
    )
    handle.visual(
        Cylinder(radius=HANDLE_GRIP_RADIUS, length=HANDLE_GRIP_LENGTH),
        origin=Origin(xyz=(HANDLE_ARM_LENGTH - 0.010, HANDLE_HUB_DEPTH * 0.8, 0.0)),
        material=hardware_metal,
        name="handle_grip",
    )

    key_cover = model.part("key_cover")
    key_cover.visual(
        Box((KEY_COVER_WIDTH, KEY_COVER_THICKNESS, KEY_COVER_HEIGHT)),
        origin=Origin(xyz=(KEY_COVER_WIDTH / 2.0, KEY_COVER_THICKNESS / 2.0, 0.0)),
        material=door_paint,
        name="key_cover_panel",
    )
    key_cover.visual(
        Cylinder(radius=KEY_COVER_THICKNESS / 2.0, length=KEY_COVER_HEIGHT),
        origin=Origin(xyz=(0.0, KEY_COVER_THICKNESS / 2.0, 0.0)),
        material=hardware_metal,
        name="key_cover_hinge",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, DOOR_FRONT_OFFSET, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.75,
            effort=40.0,
            velocity=1.2,
        ),
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DOOR_THICKNESS, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(HANDLE_X, DOOR_THICKNESS, HANDLE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.75,
            upper=0.80,
            effort=8.0,
            velocity=2.5,
        ),
    )

    model.articulation(
        "door_to_key_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cover,
        origin=Origin(xyz=(KEY_COVER_X, DOOR_THICKNESS, KEY_COVER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.35,
            effort=1.5,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    key_cover = object_model.get_part("key_cover")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_joint = object_model.get_articulation("door_to_handle")
    key_cover_joint = object_model.get_articulation("door_to_key_cover")

    limits = door_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({door_hinge: 0.0}):
            ctx.expect_overlap(
                door,
                body,
                axes="xz",
                min_overlap=0.35,
                elem_a="door_panel",
                elem_b="front_flange",
                name="closed door covers the safe opening",
            )
            ctx.expect_gap(
                door,
                body,
                axis="y",
                min_gap=0.0,
                max_gap=0.006,
                positive_elem="door_panel",
                negative_elem="front_flange",
                name="closed door sits just proud of the front flange",
            )
            ctx.expect_gap(
                dial,
                door,
                axis="y",
                min_gap=0.0,
                max_gap=0.001,
                positive_elem="dial_body",
                negative_elem="door_panel",
                name="dial mounts flush on the door face",
            )
            ctx.expect_gap(
                handle,
                door,
                axis="y",
                min_gap=0.0,
                max_gap=0.001,
                positive_elem="handle_hub",
                negative_elem="door_panel",
                name="handle spindle hub meets the door face",
            )
            ctx.expect_gap(
                key_cover,
                door,
                axis="y",
                min_gap=0.0,
                max_gap=0.001,
                positive_elem="key_cover_panel",
                negative_elem="door_panel",
                name="key override cover sits on the door face",
            )
            ctx.expect_origin_gap(
                dial,
                handle,
                axis="z",
                min_gap=0.14,
                name="dial stays above the lever handle",
            )

        rest_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: limits.upper}):
            opened_aabb = ctx.part_world_aabb(door)

        ctx.check(
            "door opens outward from the left hinge",
            rest_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][1] > rest_aabb[1][1] + 0.10,
            details=f"rest={rest_aabb}, opened={opened_aabb}",
        )

    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        handle_rest = ctx.part_world_aabb(handle)
        with ctx.pose({handle_joint: handle_limits.upper}):
            handle_raised = ctx.part_world_aabb(handle)

        ctx.check(
            "lever handle rotates upward on its spindle",
            handle_rest is not None
            and handle_raised is not None
            and handle_raised[1][2] > handle_rest[1][2] + 0.02,
            details=f"rest={handle_rest}, raised={handle_raised}",
        )

    key_cover_limits = key_cover_joint.motion_limits
    if key_cover_limits is not None and key_cover_limits.upper is not None:
        cover_rest = ctx.part_world_aabb(key_cover)
        with ctx.pose({key_cover_joint: key_cover_limits.upper}):
            cover_open = ctx.part_world_aabb(key_cover)

        ctx.check(
            "key override cover swings outward",
            cover_rest is not None
            and cover_open is not None
            and cover_open[1][1] > cover_rest[1][1] + 0.02,
            details=f"rest={cover_rest}, open={cover_open}",
        )

    return ctx.report()


object_model = build_object_model()
