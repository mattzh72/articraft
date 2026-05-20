from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FLANGE_WIDTH = 0.405
FLANGE_HEIGHT = 0.285
FLANGE_THICKNESS = 0.012
BODY_WIDTH = 0.342
BODY_HEIGHT = 0.228
BODY_DEPTH = 0.205
BODY_WALL = 0.010
OPENING_WIDTH = BODY_WIDTH - 2.0 * BODY_WALL
OPENING_HEIGHT = BODY_HEIGHT - 2.0 * BODY_WALL
DOOR_GAP = 0.003
DOOR_WIDTH = OPENING_WIDTH - 2.0 * DOOR_GAP
DOOR_HEIGHT = OPENING_HEIGHT - 2.0 * DOOR_GAP
DOOR_THICKNESS = 0.020
DOOR_PANEL_CENTER_Y = -0.007
HINGE_BARREL_Y = 0.026
HINGE_BARREL_RADIUS = 0.004
HINGE_BARREL_CENTER_X = OPENING_WIDTH / 2.0 + HINGE_BARREL_RADIUS
HINGE_PIN_X = HINGE_BARREL_CENTER_X
HINGE_LEAF_WIDTH = 0.006
HINGE_LEAF_X = HINGE_BARREL_CENTER_X - 0.001
DOOR_RIGHT_EDGE_OFFSET = 0.006
HINGE_VISIBLE_LENGTH = 0.182
DOOR_OPEN_MAX = math.radians(108.0)
DOOR_CENTER_X = -(DOOR_WIDTH / 2.0 + DOOR_RIGHT_EDGE_OFFSET)
DOOR_FRONT_Y = DOOR_PANEL_CENTER_Y + DOOR_THICKNESS / 2.0
DIAL_CENTER_Z = 0.028
HANDLE_CENTER_X = DOOR_CENTER_X + 0.060
HANDLE_CENTER_Z = 0.004
KEY_COVER_WIDTH = 0.042
KEY_COVER_HEIGHT = 0.028
KEY_COVER_CENTER_Z = -0.046


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_wall_safe")

    body_finish = model.material("body_finish", rgba=(0.16, 0.18, 0.20, 1.0))
    door_finish = model.material("door_finish", rgba=(0.20, 0.22, 0.24, 1.0))

    body = model.part("body")
    shell_center_y = -(BODY_DEPTH / 2.0 + FLANGE_THICKNESS / 2.0)
    flange_side_width = (FLANGE_WIDTH - OPENING_WIDTH) / 2.0
    flange_cap_height = (FLANGE_HEIGHT - OPENING_HEIGHT) / 2.0

    body.visual(
        Box((BODY_WIDTH, BODY_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, shell_center_y - BODY_DEPTH / 2.0 + BODY_WALL / 2.0, 0.0)),
        material=body_finish,
        name="back_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + BODY_WALL / 2.0, shell_center_y, 0.0)),
        material=body_finish,
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - BODY_WALL / 2.0, shell_center_y, 0.0)),
        material=body_finish,
        name="right_wall",
    )
    body.visual(
        Box((OPENING_WIDTH, BODY_DEPTH, BODY_WALL)),
        origin=Origin(xyz=(0.0, shell_center_y, BODY_HEIGHT / 2.0 - BODY_WALL / 2.0)),
        material=body_finish,
        name="top_wall",
    )
    body.visual(
        Box((OPENING_WIDTH, BODY_DEPTH, BODY_WALL)),
        origin=Origin(xyz=(0.0, shell_center_y, -BODY_HEIGHT / 2.0 + BODY_WALL / 2.0)),
        material=body_finish,
        name="bottom_wall",
    )
    body.visual(
        Box((flange_side_width, FLANGE_THICKNESS, FLANGE_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - flange_side_width / 2.0, 0.0, 0.0)),
        material=body_finish,
        name="left_flange",
    )
    body.visual(
        Box((flange_side_width, FLANGE_THICKNESS, FLANGE_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + flange_side_width / 2.0, 0.0, 0.0)),
        material=body_finish,
        name="right_flange",
    )
    body.visual(
        Box((FLANGE_WIDTH, FLANGE_THICKNESS, flange_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT / 2.0 + flange_cap_height / 2.0)),
        material=body_finish,
        name="top_flange",
    )
    body.visual(
        Box((FLANGE_WIDTH, FLANGE_THICKNESS, flange_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, -OPENING_HEIGHT / 2.0 - flange_cap_height / 2.0)),
        material=body_finish,
        name="bottom_flange",
    )
    body.visual(
        Box((HINGE_LEAF_WIDTH, 0.026, HINGE_VISIBLE_LENGTH)),
        origin=Origin(xyz=(HINGE_LEAF_X, 0.013, 0.0)),
        material=body_finish,
        name="hinge_leaf",
    )
    body.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_VISIBLE_LENGTH),
        origin=Origin(xyz=(HINGE_BARREL_CENTER_X, HINGE_BARREL_Y, 0.0)),
        material=body_finish,
        name="hinge_barrel",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_PANEL_CENTER_Y, 0.0)),
        material=door_finish,
        name="door_slab",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.032, 0.004, DOOR_HEIGHT - 0.032)),
        origin=Origin(
            xyz=(
                DOOR_CENTER_X,
                DOOR_PANEL_CENTER_Y + DOOR_THICKNESS / 2.0 - 0.002,
                0.0,
            )
        ),
        material=door_finish,
        name="door_plate",
    )
    door.visual(
        Box((0.002, 0.018, HINGE_VISIBLE_LENGTH)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=door_finish,
        name="door_hinge_return",
    )

    dial_finish = model.material("dial_finish", rgba=(0.75, 0.78, 0.80, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.62, 0.64, 0.66, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.18, 0.19, 0.21, 1.0))

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.031, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_finish,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_finish,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(0.0, 0.0075, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_finish,
        name="dial_pointer",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_finish,
        name="handle_hub",
    )
    handle.visual(
        Box((0.040, 0.010, 0.010)),
        origin=Origin(xyz=(0.020, 0.010, 0.0)),
        material=handle_finish,
        name="handle_spoke",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.040, 0.010, 0.0)),
        material=handle_finish,
        name="handle_grip",
    )

    key_cover = model.part("key_cover")
    key_cover.visual(
        Box((KEY_COVER_WIDTH, 0.006, KEY_COVER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.003, -KEY_COVER_HEIGHT / 2.0)),
        material=cover_finish,
        name="cover_panel",
    )
    key_cover.visual(
        Box((KEY_COVER_WIDTH - 0.008, 0.002, KEY_COVER_HEIGHT - 0.008)),
        origin=Origin(xyz=(0.0, 0.006, -KEY_COVER_HEIGHT / 2.0)),
        material=cover_finish,
        name="cover_face",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_PIN_X, HINGE_BARREL_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=DOOR_OPEN_MAX,
        ),
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_FRONT_Y, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(HANDLE_CENTER_X, DOOR_FRONT_Y, HANDLE_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.1,
            upper=1.1,
        ),
    )

    model.articulation(
        "door_to_key_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cover,
        origin=Origin(
            xyz=(
                DOOR_CENTER_X,
                DOOR_FRONT_Y,
                KEY_COVER_CENTER_Z + KEY_COVER_HEIGHT / 2.0,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=0.0,
            upper=1.4,
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
    dial_joint = object_model.get_articulation("door_to_dial")
    key_cover_joint = object_model.get_articulation("door_to_key_cover")
    limits = door_hinge.motion_limits

    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a="door_slab",
        elem_b="right_flange",
        min_overlap=0.18,
        name="door spans the front opening height",
    )
    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_body",
        elem_b="door_slab",
        name="dial seats on the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_hub",
        elem_b="door_slab",
        name="handle hub seats on the door face",
    )
    ctx.expect_contact(
        key_cover,
        door,
        elem_a="cover_panel",
        elem_b="door_slab",
        name="key cover closes against the door face",
    )
    ctx.expect_origin_gap(
        dial,
        key_cover,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        name="key cover sits below the centered dial",
    )

    if limits is not None and limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door opens outward from right hinge",
            rest_aabb is not None
            and open_aabb is not None
            and float(open_aabb[1][1]) > float(rest_aabb[1][1]) + 0.08,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    door_aabb = ctx.part_world_aabb(door)
    dial_pos = ctx.part_world_position(dial)
    ctx.check(
        "dial stays centered across the door width",
        door_aabb is not None
        and dial_pos is not None
        and abs(float(dial_pos[0]) - float((door_aabb[0][0] + door_aabb[1][0]) / 2.0)) <= 0.01,
        details=f"door_aabb={door_aabb}, dial_pos={dial_pos}",
    )

    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        rest_handle_aabb = ctx.part_world_aabb(handle)
        with ctx.pose({handle_joint: handle_limits.upper}):
            turned_handle_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "handle rotates on its front hub",
            rest_handle_aabb is not None
            and turned_handle_aabb is not None
            and float(turned_handle_aabb[1][2]) > float(rest_handle_aabb[1][2]) + 0.003,
            details=f"rest={rest_handle_aabb}, turned={turned_handle_aabb}",
        )

    with ctx.pose({dial_joint: 1.0}):
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_body",
            elem_b="door_slab",
            name="dial remains seated while spinning",
        )

    key_cover_limits = key_cover_joint.motion_limits
    if key_cover_limits is not None and key_cover_limits.upper is not None:
        rest_cover_aabb = ctx.part_world_aabb(key_cover)
        with ctx.pose({key_cover_joint: key_cover_limits.upper}):
            open_cover_aabb = ctx.part_world_aabb(key_cover)
        ctx.check(
            "key cover flips outward",
            rest_cover_aabb is not None
            and open_cover_aabb is not None
            and float(open_cover_aabb[1][1]) > float(rest_cover_aabb[1][1]) + 0.015,
            details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
