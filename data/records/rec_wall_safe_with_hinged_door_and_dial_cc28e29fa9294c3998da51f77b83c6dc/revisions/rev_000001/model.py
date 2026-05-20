from __future__ import annotations

from math import cos, pi, sin

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


FLANGE_W = 0.43
FLANGE_H = 0.31
FLANGE_T = 0.012

BODY_W = 0.36
BODY_H = 0.24
BODY_D = 0.20

OPENING_W = 0.292
OPENING_H = 0.198
CAVITY_D = 0.182

DOOR_W = 0.282
DOOR_H = 0.188
DOOR_T = 0.022
DOOR_FACE_GAP = 0.0015
HINGE_RADIUS = 0.006

DIAL_D = 0.064
DIAL_T = 0.011

HANDLE_HUB_R = 0.014
HANDLE_HUB_T = 0.016
HANDLE_SPOKE_LEN = 0.040
HANDLE_SPOKE_T = 0.007
HANDLE_SPOKE_W = 0.012
HANDLE_CENTER_Z = -0.068

SHELF_T = 0.006
SHELF_W = OPENING_W
SHELF_D = 0.155
SHELF_Y = -0.1045
SHELF_Z = -0.032

RUNNER_W = 0.010
RUNNER_L = 0.100
RUNNER_H = 0.007
RUNNER_Y = -0.083
RUNNER_Z = SHELF_Z - (SHELF_T / 2.0) - 0.005 - (RUNNER_H / 2.0)
RUNNER_X = (OPENING_W / 2.0) - (RUNNER_W / 2.0)

TRAY_W = 0.236
TRAY_D = 0.142
TRAY_H = 0.030
TRAY_WALL = 0.003
TRAY_CLOSED_Y = -0.108
TRAY_Z = -0.056
TRAY_TRAVEL = 0.072


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_wall_safe")

    body_finish = model.material("body_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    shelf_finish = model.material("shelf_finish", rgba=(0.26, 0.27, 0.29, 1.0))
    door_finish = model.material("door_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    runner_finish = model.material("runner_finish", rgba=(0.58, 0.60, 0.63, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.72, 0.63, 0.38, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.13, 0.13, 0.14, 1.0))

    body = model.part("body")
    side_wall_t = (BODY_W - OPENING_W) / 2.0
    top_wall_t = (BODY_H - OPENING_H) / 2.0
    back_wall_t = BODY_D - CAVITY_D
    flange_side_w = (FLANGE_W - OPENING_W) / 2.0
    flange_top_h = (FLANGE_H - OPENING_H) / 2.0

    body.visual(
        Box((BODY_W, back_wall_t, BODY_H)),
        origin=Origin(xyz=(0.0, -CAVITY_D - back_wall_t / 2.0, 0.0)),
        material=body_finish,
        name="back_wall",
    )
    for idx, x_pos in enumerate((-(OPENING_W / 2.0 + side_wall_t / 2.0), OPENING_W / 2.0 + side_wall_t / 2.0)):
        body.visual(
            Box((side_wall_t, CAVITY_D, BODY_H)),
            origin=Origin(xyz=(x_pos, -CAVITY_D / 2.0, 0.0)),
            material=body_finish,
            name=f"side_wall_{idx}",
        )
    for idx, z_pos in enumerate((OPENING_H / 2.0 + top_wall_t / 2.0, -(OPENING_H / 2.0 + top_wall_t / 2.0))):
        body.visual(
            Box((OPENING_W, CAVITY_D, top_wall_t)),
            origin=Origin(xyz=(0.0, -CAVITY_D / 2.0, z_pos)),
            material=body_finish,
            name=f"horizontal_wall_{idx}",
        )
    for idx, x_pos in enumerate((-(OPENING_W / 2.0 + flange_side_w / 2.0), OPENING_W / 2.0 + flange_side_w / 2.0)):
        body.visual(
            Box((flange_side_w, FLANGE_T, FLANGE_H)),
            origin=Origin(xyz=(x_pos, FLANGE_T / 2.0, 0.0)),
            material=body_finish,
            name=f"flange_side_{idx}",
        )
    for idx, z_pos in enumerate((OPENING_H / 2.0 + flange_top_h / 2.0, -(OPENING_H / 2.0 + flange_top_h / 2.0))):
        body.visual(
            Box((OPENING_W, FLANGE_T, flange_top_h)),
            origin=Origin(xyz=(0.0, FLANGE_T / 2.0, z_pos)),
            material=body_finish,
            name=f"flange_cap_{idx}",
        )
    body.visual(
        Box((SHELF_W, SHELF_D, SHELF_T)),
        origin=Origin(xyz=(0.0, SHELF_Y, SHELF_Z)),
        material=shelf_finish,
        name="shelf",
    )
    for idx, x_pos in enumerate((-RUNNER_X, RUNNER_X)):
        body.visual(
            Box((RUNNER_W, RUNNER_L, RUNNER_H)),
            origin=Origin(xyz=(x_pos, RUNNER_Y, RUNNER_Z)),
            material=runner_finish,
            name=f"runner_{idx}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_FACE_GAP + DOOR_T / 2.0, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    face_raise_t = 0.004
    stile_w = 0.022
    rail_h = 0.022
    for idx, x_pos in enumerate((-DOOR_W + stile_w / 2.0, -stile_w / 2.0)):
        door.visual(
            Box((stile_w, face_raise_t, DOOR_H - 0.010)),
            origin=Origin(xyz=(x_pos, DOOR_FACE_GAP + DOOR_T + face_raise_t / 2.0, 0.0)),
            material=door_finish,
            name=f"face_stile_{idx}",
        )
    for idx, z_pos in enumerate((DOOR_H / 2.0 - rail_h / 2.0, -(DOOR_H / 2.0 - rail_h / 2.0))):
        door.visual(
            Box((DOOR_W - 2.0 * stile_w, face_raise_t, rail_h)),
            origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_FACE_GAP + DOOR_T + face_raise_t / 2.0, z_pos)),
            material=door_finish,
            name=f"face_rail_{idx}",
        )
    for idx, z_pos in enumerate((-0.058, 0.0, 0.058)):
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.046),
            origin=Origin(xyz=(0.0, DOOR_FACE_GAP + (DOOR_T * 0.55), z_pos)),
            material=runner_finish,
            name=f"hinge_knuckle_{idx}",
        )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_D / 2.0, length=DIAL_T),
        origin=Origin(xyz=(0.0, DIAL_T / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dial_finish,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=DIAL_D * 0.22, length=0.008),
        origin=Origin(xyz=(0.0, DIAL_T + 0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=runner_finish,
        name="dial_hub",
    )
    dial.visual(
        Box((0.006, 0.0015, 0.012)),
        origin=Origin(xyz=(0.0, DIAL_T + 0.00075, DIAL_D * 0.33)),
        material=runner_finish,
        name="dial_pointer",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_HUB_R, length=HANDLE_HUB_T),
        origin=Origin(xyz=(0.0, HANDLE_HUB_T / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=handle_finish,
        name="hub",
    )
    short_spoke_len = 0.028
    short_spoke_w = 0.010
    spoke_offset = 0.012
    for idx, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        handle.visual(
            Box((short_spoke_len, HANDLE_SPOKE_T, short_spoke_w)),
            origin=Origin(
                xyz=(spoke_offset * cos(angle), HANDLE_HUB_T / 2.0, spoke_offset * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=handle_finish,
            name=f"spoke_{idx}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, -TRAY_H / 2.0 + TRAY_WALL / 2.0)),
        material=tray_finish,
        name="tray_floor",
    )
    for idx, x_pos in enumerate((-(TRAY_W / 2.0 - TRAY_WALL / 2.0), TRAY_W / 2.0 - TRAY_WALL / 2.0)):
        tray.visual(
            Box((TRAY_WALL, TRAY_D, TRAY_H)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=tray_finish,
            name=f"tray_side_{idx}",
        )
    tray.visual(
        Box((TRAY_W - 2.0 * TRAY_WALL, TRAY_WALL, TRAY_H)),
        origin=Origin(xyz=(0.0, -(TRAY_D / 2.0 - TRAY_WALL / 2.0), 0.0)),
        material=tray_finish,
        name="tray_back",
    )
    tray.visual(
        Box((TRAY_W - 2.0 * TRAY_WALL, TRAY_WALL, TRAY_H * 0.55)),
        origin=Origin(
            xyz=(0.0, TRAY_D / 2.0 - TRAY_WALL / 2.0, -(TRAY_H - (TRAY_H * 0.55)) / 2.0)
        ),
        material=tray_finish,
        name="tray_front_lip",
    )
    shoe_w = 0.018
    shoe_d = 0.095
    shoe_h = RUNNER_H
    shoe_z = RUNNER_Z - TRAY_Z
    for idx, x_pos in enumerate((-(TRAY_W / 2.0 + shoe_w / 2.0), TRAY_W / 2.0 + shoe_w / 2.0)):
        tray.visual(
            Box((shoe_w, shoe_d, shoe_h)),
            origin=Origin(xyz=(x_pos, -0.010, shoe_z)),
            material=runner_finish,
            name=f"tray_shoe_{idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=25.0, velocity=1.1),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_FACE_GAP + DOOR_T, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_FACE_GAP + DOOR_T, HANDLE_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.80, upper=0.80, effort=4.0, velocity=2.5),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_CLOSED_Y, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=20.0, velocity=0.20),
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
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_handle")
    tray_slide = object_model.get_articulation("body_to_tray")

    ctx.check(
        "articulation types match prompt",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and handle_joint.articulation_type == ArticulationType.REVOLUTE
        and tray_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"door={door_hinge.articulation_type}, dial={dial_joint.articulation_type}, "
            f"handle={handle_joint.articulation_type}, tray={tray_slide.articulation_type}"
        ),
    )
    ctx.check(
        "dial is centered and handle sits below it",
        abs(dial_joint.origin.xyz[0] + DOOR_W / 2.0) < 1e-9
        and abs(dial_joint.origin.xyz[2]) < 1e-9
        and abs(handle_joint.origin.xyz[0] + DOOR_W / 2.0) < 1e-9
        and handle_joint.origin.xyz[2] < dial_joint.origin.xyz[2] - 0.04,
        details=f"dial_origin={dial_joint.origin.xyz}, handle_origin={handle_joint.origin.xyz}",
    )
    ctx.expect_contact(dial, door, name="dial mounts on the door face")
    ctx.expect_contact(handle, door, name="handle hub mounts on the door face")
    ctx.expect_gap(
        body,
        tray,
        axis="z",
        positive_elem="shelf",
        min_gap=0.004,
        max_gap=0.010,
        name="tray stays just below the main shelf",
    )

    door_limits = door_hinge.motion_limits
    tray_limits = tray_slide.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        with ctx.pose({door_hinge: door_limits.upper}):
            opened_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
            ctx.check(
                "door opens outward from the right hinge",
                closed_aabb is not None
                and opened_aabb is not None
                and opened_aabb[1][1] > closed_aabb[1][1] + 0.10,
                details=f"closed={closed_aabb}, opened={opened_aabb}",
            )

    if (
        door_limits is not None
        and door_limits.upper is not None
        and tray_limits is not None
        and tray_limits.upper is not None
    ):
        tray_rest = ctx.part_world_position(tray)
        with ctx.pose({door_hinge: door_limits.upper, tray_slide: tray_limits.upper}):
            tray_extended = ctx.part_world_position(tray)
            ctx.expect_within(
                tray,
                body,
                axes="xz",
                margin=0.002,
                name="extended tray stays laterally aligned within the safe body",
            )
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                min_overlap=0.065,
                name="extended tray remains engaged on its interior runners",
            )
            ctx.expect_gap(
                body,
                tray,
                axis="z",
                positive_elem="shelf",
                min_gap=0.004,
                max_gap=0.010,
                name="extended tray still clears the shelf underside",
            )
            ctx.check(
                "tray slides forward when extended",
                tray_rest is not None and tray_extended is not None and tray_extended[1] > tray_rest[1] + 0.05,
                details=f"rest={tray_rest}, extended={tray_extended}",
            )

    return ctx.report()


object_model = build_object_model()
