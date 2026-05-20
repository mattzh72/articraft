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


BODY_W = 0.380
BODY_H = 0.270
BODY_D = 0.260
BACK_T = 0.016

FLANGE_W = 0.440
FLANGE_H = 0.320
FLANGE_T = 0.008

OPEN_W = 0.304
OPEN_H = 0.192

DOOR_W = 0.294
DOOR_H = 0.182
DOOR_T = 0.028
FRONT_FACE_Y = BODY_D / 2.0 + FLANGE_T
DOOR_Y = FRONT_FACE_Y - DOOR_T / 2.0
HINGE_X = 0.156
HINGE_R = 0.007
BODY_KNUCKLE_LEN = 0.046
DOOR_KNUCKLE_LEN = 0.072
BODY_KNUCKLE_Z = 0.064
DOOR_PANEL_OFFSET = DOOR_W / 2.0 + HINGE_R - 0.002

DIAL_RADIUS = 0.022
DIAL_BEZEL_RADIUS = 0.029
DIAL_DEPTH = 0.018
DIAL_X = -DOOR_PANEL_OFFSET

HANDLE_HUB_R = 0.015
HANDLE_HUB_DEPTH = 0.018
HANDLE_SPOKE_R = 0.0065
HANDLE_SPOKE_LEN = 0.052
HANDLE_X = -DOOR_PANEL_OFFSET
HANDLE_Z = -0.056

SIDE_T = (BODY_W - OPEN_W) / 2.0
TOP_T = (BODY_H - OPEN_H) / 2.0
FLANGE_SIDE_T = (FLANGE_W - OPEN_W) / 2.0
FLANGE_TOP_T = (FLANGE_H - OPEN_H) / 2.0

FLAP_W = 0.250
FLAP_H = 0.060
FLAP_T = 0.004
FLAP_Y = 0.045
FLAP_Z = OPEN_H / 2.0 - 0.008
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_wall_safe")

    body_finish = model.material("body_finish", rgba=(0.18, 0.18, 0.20, 1.0))
    door_finish = model.material("door_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    bright_metal = model.material("bright_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    flap_finish = model.material("flap_finish", rgba=(0.66, 0.68, 0.71, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BACK_T, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + BACK_T / 2.0, 0.0)),
        material=body_finish,
        name="back_wall",
    )
    body.visual(
        Box((SIDE_T, 0.226, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - SIDE_T / 2.0, -0.001, 0.0)),
        material=body_finish,
        name="right_wall",
    )
    body.visual(
        Box((SIDE_T, 0.226, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SIDE_T / 2.0, -0.001, 0.0)),
        material=body_finish,
        name="left_wall",
    )
    body.visual(
        Box((OPEN_W, BODY_D - BACK_T + 0.002, TOP_T)),
        origin=Origin(xyz=(0.0, 0.008, BODY_H / 2.0 - TOP_T / 2.0)),
        material=body_finish,
        name="top_wall",
    )
    body.visual(
        Box((OPEN_W, BODY_D - BACK_T + 0.002, TOP_T)),
        origin=Origin(xyz=(0.0, 0.008, -BODY_H / 2.0 + TOP_T / 2.0)),
        material=body_finish,
        name="bottom_wall",
    )
    body.visual(
        Box((FLANGE_W, FLANGE_T, FLANGE_TOP_T)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 + FLANGE_T / 2.0, OPEN_H / 2.0 + FLANGE_TOP_T / 2.0)
        ),
        material=body_finish,
        name="flange_top",
    )
    body.visual(
        Box((FLANGE_W, FLANGE_T, FLANGE_TOP_T)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 + FLANGE_T / 2.0, -OPEN_H / 2.0 - FLANGE_TOP_T / 2.0)
        ),
        material=body_finish,
        name="flange_bottom",
    )
    body.visual(
        Box((FLANGE_SIDE_T, FLANGE_T, OPEN_H)),
        origin=Origin(
            xyz=(OPEN_W / 2.0 + FLANGE_SIDE_T / 2.0, BODY_D / 2.0 + FLANGE_T / 2.0, 0.0)
        ),
        material=body_finish,
        name="flange_right",
    )
    body.visual(
        Box((FLANGE_SIDE_T, FLANGE_T, OPEN_H)),
        origin=Origin(
            xyz=(-OPEN_W / 2.0 - FLANGE_SIDE_T / 2.0, BODY_D / 2.0 + FLANGE_T / 2.0, 0.0)
        ),
        material=body_finish,
        name="flange_left",
    )
    body.visual(
        Box((OPEN_W + 0.020, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, FLAP_Y - 0.012, FLAP_Z)),
        material=body_finish,
        name="flap_header",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_LEN),
        origin=Origin(xyz=(HINGE_X, DOOR_Y, BODY_KNUCKLE_Z)),
        material=body_finish,
        name="hinge_knuckle_top",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_LEN),
        origin=Origin(xyz=(HINGE_X, DOOR_Y, -BODY_KNUCKLE_Z)),
        material=body_finish,
        name="hinge_knuckle_bottom",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_PANEL_OFFSET, 0.0, 0.0)),
        material=door_finish,
        name="door_leaf",
    )
    door.visual(
        Cylinder(radius=HINGE_R, length=DOOR_KNUCKLE_LEN),
        material=door_finish,
        name="door_knuckle",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_BEZEL_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="dial_body",
    )
    dial.visual(
        Box((0.010, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, DIAL_DEPTH - 0.001, DIAL_RADIUS - 0.002)),
        material=bright_metal,
        name="dial_pointer",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_HUB_R, length=HANDLE_HUB_DEPTH),
        origin=Origin(xyz=(0.0, HANDLE_HUB_DEPTH / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=HANDLE_SPOKE_R, length=HANDLE_SPOKE_LEN),
        origin=Origin(
            xyz=(HANDLE_SPOKE_LEN / 2.0 + 0.010, HANDLE_HUB_DEPTH / 2.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bright_metal,
        name="handle_spoke",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(HANDLE_SPOKE_LEN + 0.010, HANDLE_HUB_DEPTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bright_metal,
        name="handle_tip",
    )

    flap = model.part("document_flap")
    flap.visual(
        Cylinder(radius=0.004, length=FLAP_W),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flap_finish,
        name="flap_barrel",
    )
    flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, -FLAP_T / 2.0, -FLAP_H / 2.0)),
        material=flap_finish,
        name="flap_panel",
    )
    flap.visual(
        Box((FLAP_W - 0.020, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, -FLAP_H + 0.005)),
        material=flap_finish,
        name="flap_lip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, DOOR_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=20.0, velocity=1.5),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DOOR_T / 2.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(HANDLE_X, DOOR_T / 2.0, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.0, effort=6.0, velocity=3.0),
    )
    model.articulation(
        "body_to_document_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, FLAP_Y, FLAP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    flap = object_model.get_part("document_flap")

    door_hinge = object_model.get_articulation("body_to_door")
    flap_hinge = object_model.get_articulation("body_to_document_flap")

    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_bezel",
        elem_b="door_leaf",
        name="dial is seated on the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_hub",
        elem_b="door_leaf",
        name="handle hub is seated on the door face",
    )
    ctx.expect_gap(
        door,
        flap,
        axis="y",
        positive_elem="door_leaf",
        negative_elem="flap_panel",
        min_gap=0.040,
        name="document flap stays behind the closed door",
    )
    ctx.expect_origin_gap(
        flap,
        door,
        axis="z",
        min_gap=0.070,
        name="document flap is mounted high inside the opening",
    )
    ctx.expect_origin_gap(
        dial,
        handle,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        name="handle sits below the centered dial",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_leaf")
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_door = ctx.part_element_world_aabb(door, elem="door_leaf")
    ctx.check(
        "door swings outward from the right hinge",
        closed_door is not None
        and open_door is not None
        and open_door[1][1] > closed_door[1][1] + 0.10,
        details=f"closed={closed_door}, open={open_door}",
    )

    closed_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "document flap folds inward into the cavity",
        closed_flap is not None
        and open_flap is not None
        and open_flap[0][1] < closed_flap[0][1] - 0.020,
        details=f"closed={closed_flap}, open={open_flap}",
    )

    return ctx.report()


object_model = build_object_model()
