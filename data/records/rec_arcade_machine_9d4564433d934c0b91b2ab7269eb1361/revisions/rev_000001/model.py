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

WIDTH = 0.74
SIDE_THICKNESS = 0.018
INNER_WIDTH = WIDTH - 2.0 * SIDE_THICKNESS
BODY_FRONT_X = 0.24
BODY_REAR_X = -0.36
HEIGHT = 1.78

OUTER_PROFILE = [
    (BODY_REAR_X, 0.00),
    (BODY_FRONT_X, 0.00),
    (BODY_FRONT_X, 0.94),
    (0.07, 1.10),
    (-0.02, 1.42),
    (0.10, 1.58),
    (0.10, HEIGHT),
    (-0.27, HEIGHT),
    (BODY_REAR_X, 1.42),
]

INNER_PROFILE = [
    (-0.32, 0.10),
    (0.19, 0.10),
    (0.19, 0.90),
    (0.03, 1.05),
    (-0.05, 1.38),
    (0.05, 1.54),
    (0.05, 1.72),
    (-0.23, 1.72),
    (-0.32, 1.38),
]

DOOR_WIDTH = 0.458
DOOR_HEIGHT = 0.578
DOOR_THICKNESS = 0.016
DOOR_BOTTOM_Z = 0.13
DOOR_HINGE_Y = DOOR_WIDTH / 2.0 + 0.004
DOOR_HINGE_X = BODY_FRONT_X

FLAP_WIDTH = 0.104
FLAP_HEIGHT = 0.056
FLAP_THICKNESS = 0.007
FLAP_HINGE_X = BODY_FRONT_X
FLAP_HINGE_Y = 0.10
FLAP_HINGE_Z = 0.86

COIN_SLOT_CENTER = (-0.11, 0.885)

DECK_ORIGIN_X = BODY_FRONT_X
DECK_ORIGIN_Z = 0.94
DECK_WIDTH = 0.64
DECK_PROFILE = [
    (0.00, 0.00),
    (0.20, 0.00),
    (0.20, 0.05),
    (0.00, 0.15),
]

DECK_TOP_REAR = (0.00, 0.15)
DECK_TOP_FRONT = (0.20, 0.05)
DECK_TOP_DX = DECK_TOP_FRONT[0] - DECK_TOP_REAR[0]
DECK_TOP_DZ = DECK_TOP_FRONT[1] - DECK_TOP_REAR[1]
DECK_TOP_NORMAL_LEN = math.hypot(-DECK_TOP_DZ, DECK_TOP_DX)
DECK_TOP_NORMAL = (
    -DECK_TOP_DZ / DECK_TOP_NORMAL_LEN,
    0.0,
    DECK_TOP_DX / DECK_TOP_NORMAL_LEN,
)
DECK_TOP_PITCH = math.atan2(DECK_TOP_NORMAL[0], DECK_TOP_NORMAL[2])

MONITOR_PITCH = math.radians(-15.7)
SCREEN_ORIGIN = (0.02, 0.0, 1.27)
SCREEN_NORMAL = (math.cos(MONITOR_PITCH), 0.0, -math.sin(MONITOR_PITCH))

MARQUEE_ORIGIN = (0.06, 0.0, 1.64)
MARQUEE_PITCH = math.radians(-3.0)


def _extruded_profile(profile: list[tuple[float, float]], width: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(width)
        .translate((0.0, width / 2.0, 0.0))
    )


def _rotated_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    pitch_deg: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(*size)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), pitch_deg)
        .translate(center)
    )


def _body_shell() -> cq.Workplane:
    left_side = _extruded_profile(OUTER_PROFILE, SIDE_THICKNESS).translate((0.0, WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0))
    right_side = _extruded_profile(OUTER_PROFILE, SIDE_THICKNESS).translate((0.0, -WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0))

    bottom_panel = cq.Workplane("XY").box(0.48, INNER_WIDTH, SIDE_THICKNESS).translate((-0.08, 0.0, SIDE_THICKNESS / 2.0))
    lower_rear = cq.Workplane("XY").box(SIDE_THICKNESS, INNER_WIDTH, 1.42).translate((BODY_REAR_X + SIDE_THICKNESS / 2.0, 0.0, 0.71))
    upper_rear = cq.Workplane("XY").box(SIDE_THICKNESS, INNER_WIDTH, 0.36).translate((-0.27, 0.0, 1.60))
    roof = cq.Workplane("XY").box(0.37, INNER_WIDTH, SIDE_THICKNESS).translate((-0.085, 0.0, HEIGHT - SIDE_THICKNESS / 2.0))

    door_open_width = DOOR_WIDTH + 0.006
    stile_width = (INNER_WIDTH - door_open_width) / 2.0
    stile_center_y = door_open_width / 2.0 + stile_width / 2.0

    bottom_rail = cq.Workplane("XY").box(SIDE_THICKNESS, INNER_WIDTH, 0.13).translate((BODY_FRONT_X - SIDE_THICKNESS / 2.0, 0.0, 0.065))
    left_stile = cq.Workplane("XY").box(SIDE_THICKNESS, stile_width, 0.59).translate(
        (BODY_FRONT_X - SIDE_THICKNESS / 2.0, -stile_center_y, 0.425)
    )
    right_stile = cq.Workplane("XY").box(SIDE_THICKNESS, stile_width, 0.59).translate(
        (BODY_FRONT_X - SIDE_THICKNESS / 2.0, stile_center_y, 0.425)
    )

    flap_cut = cq.Workplane("XY").box(0.05, FLAP_WIDTH + 0.006, FLAP_HEIGHT + 0.006).translate(
        (BODY_FRONT_X - 0.012, FLAP_HINGE_Y, FLAP_HINGE_Z - FLAP_HEIGHT / 2.0)
    )
    coin_panel = (
        cq.Workplane("XY")
        .box(SIDE_THICKNESS, INNER_WIDTH, 0.20)
        .translate((BODY_FRONT_X - SIDE_THICKNESS / 2.0, 0.0, 0.82))
        .cut(flap_cut)
    )

    deck_shelf = cq.Workplane("XY").box(0.22, INNER_WIDTH, SIDE_THICKNESS).translate((0.04, 0.0, 0.93))
    deck_mount = cq.Workplane("XY").box(0.10, INNER_WIDTH, 0.12).translate((0.19, 0.0, 1.00))

    monitor_frame = _rotated_box((SIDE_THICKNESS, INNER_WIDTH, 0.74), (0.02, 0.0, 1.27), pitch_deg=math.degrees(MONITOR_PITCH))
    monitor_opening = _rotated_box((0.05, 0.50, 0.62), (0.02, 0.0, 1.27), pitch_deg=math.degrees(MONITOR_PITCH))
    monitor_frame = monitor_frame.cut(monitor_opening)

    marquee_frame = _rotated_box((SIDE_THICKNESS, INNER_WIDTH, 0.22), (0.091, 0.0, 1.64), pitch_deg=math.degrees(MARQUEE_PITCH))
    marquee_opening = _rotated_box((0.05, 0.56, 0.18), (0.091, 0.0, 1.64), pitch_deg=math.degrees(MARQUEE_PITCH))
    marquee_frame = marquee_frame.cut(marquee_opening)

    body = left_side.union(right_side)
    for piece in (
        bottom_panel,
        lower_rear,
        upper_rear,
        roof,
        bottom_rail,
        left_stile,
        right_stile,
        coin_panel,
        deck_shelf,
        deck_mount,
        monitor_frame,
        marquee_frame,
    ):
        body = body.union(piece)

    return body


def _control_panel_shell() -> cq.Workplane:
    local_profile = [(x, z) for x, z in DECK_PROFILE]
    return _extruded_profile(local_profile, DECK_WIDTH)


def _deck_top_z(local_x: float) -> float:
    t = local_x / DECK_TOP_DX
    return DECK_TOP_REAR[1] + t * DECK_TOP_DZ


def _deck_mount_center(local_x: float, local_y: float, length: float, embed: float) -> tuple[float, float, float]:
    top_z = _deck_top_z(local_x)
    offset = length / 2.0 - embed
    return (
        local_x + DECK_TOP_NORMAL[0] * offset,
        local_y,
        top_z + DECK_TOP_NORMAL[2] * offset,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_cabinet")

    body_finish = model.material("body_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.15, 0.15, 0.16, 1.0))
    door_finish = model.material("door_finish", rgba=(0.18, 0.18, 0.19, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.63, 0.65, 0.69, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.04, 0.05, 0.06, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.10, 0.12, 0.15, 0.94))
    marquee_finish = model.material("marquee_finish", rgba=(0.92, 0.54, 0.18, 0.96))
    red_finish = model.material("red_finish", rgba=(0.76, 0.12, 0.10, 1.0))
    yellow_finish = model.material("yellow_finish", rgba=(0.86, 0.72, 0.16, 1.0))
    blue_finish = model.material("blue_finish", rgba=(0.12, 0.34, 0.78, 1.0))
    black_finish = model.material("black_finish", rgba=(0.03, 0.03, 0.03, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell(), "arcade_body"), material=body_finish, name="body_shell")
    body.visual(
        Box((0.008, 0.12, 0.09)),
        origin=Origin(xyz=(0.236, COIN_SLOT_CENTER[0], COIN_SLOT_CENTER[1])),
        material=metal_finish,
        name="coin_trim",
    )
    body.visual(
        Box((0.003, 0.070, 0.007)),
        origin=Origin(xyz=(0.239, COIN_SLOT_CENTER[0], COIN_SLOT_CENTER[1] + 0.010)),
        material=black_finish,
        name="coin_slot",
    )
    body.visual(
        Box((0.008, 0.500, 0.620)),
        origin=Origin(
            xyz=(
                SCREEN_ORIGIN[0] + SCREEN_NORMAL[0] * 0.002,
                0.0,
                SCREEN_ORIGIN[2] + SCREEN_NORMAL[2] * 0.002,
            ),
            rpy=(0.0, MONITOR_PITCH, 0.0),
        ),
        material=glass_finish,
        name="screen_glass",
    )
    body.visual(
        Box((0.004, 0.420, 0.520)),
        origin=Origin(
            xyz=(
                SCREEN_ORIGIN[0] - SCREEN_NORMAL[0] * 0.004,
                0.0,
                SCREEN_ORIGIN[2] - SCREEN_NORMAL[2] * 0.004,
            ),
            rpy=(0.0, MONITOR_PITCH, 0.0),
        ),
        material=screen_finish,
        name="screen_image",
    )
    body.visual(
        Box((0.020, 0.568, 0.188)),
        origin=Origin(xyz=(MARQUEE_ORIGIN[0] + 0.012, 0.0, MARQUEE_ORIGIN[2]), rpy=(0.0, MARQUEE_PITCH, 0.0)),
        material=marquee_finish,
        name="marquee_panel",
    )
    body.visual(
        Box((0.006, 0.500, 0.110)),
        origin=Origin(xyz=(MARQUEE_ORIGIN[0] + 0.009, 0.0, MARQUEE_ORIGIN[2]), rpy=(0.0, MARQUEE_PITCH, 0.0)),
        material=glass_finish,
        name="marquee_lens",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_control_panel_shell(), "control_panel"),
        material=trim_finish,
        name="deck_shell",
    )

    joystick_base_center = _deck_mount_center(local_x=0.075, local_y=0.17, length=0.018, embed=0.005)
    joystick_stem_center = _deck_mount_center(local_x=0.075, local_y=0.17, length=0.120, embed=-0.002)
    joystick_ball_center = _deck_mount_center(local_x=0.075, local_y=0.17, length=0.162, embed=0.0)
    button_red_center = _deck_mount_center(local_x=0.118, local_y=-0.14, length=0.018, embed=0.005)
    button_yellow_center = _deck_mount_center(local_x=0.145, local_y=0.01, length=0.018, embed=0.005)
    button_blue_center = _deck_mount_center(local_x=0.165, local_y=0.15, length=0.018, embed=0.005)

    control_panel.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=joystick_base_center, rpy=(0.0, DECK_TOP_PITCH, 0.0)),
        material=black_finish,
        name="joystick_base",
    )
    control_panel.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=joystick_stem_center, rpy=(0.0, DECK_TOP_PITCH, 0.0)),
        material=metal_finish,
        name="joystick_stem",
    )
    control_panel.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=joystick_ball_center),
        material=red_finish,
        name="joystick_ball",
    )
    control_panel.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=button_red_center, rpy=(0.0, DECK_TOP_PITCH, 0.0)),
        material=red_finish,
        name="button_red",
    )
    control_panel.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=button_yellow_center, rpy=(0.0, DECK_TOP_PITCH, 0.0)),
        material=yellow_finish,
        name="button_yellow",
    )
    control_panel.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=button_blue_center, rpy=(0.0, DECK_TOP_PITCH, 0.0)),
        material=blue_finish,
        name="button_blue",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_THICKNESS / 2.0, -DOOR_WIDTH / 2.0, DOOR_HEIGHT / 2.0)),
        material=door_finish,
        name="door_panel",
    )
    service_door.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(DOOR_THICKNESS + 0.005, -DOOR_WIDTH + 0.060, DOOR_HEIGHT * 0.52),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_finish,
        name="door_lock",
    )

    coin_flap = model.part("coin_flap")
    coin_flap.visual(
        Box((FLAP_THICKNESS, FLAP_WIDTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(FLAP_THICKNESS / 2.0, 0.0, -FLAP_HEIGHT / 2.0)),
        material=metal_finish,
        name="flap_panel",
    )
    coin_flap.visual(
        Box((0.006, FLAP_WIDTH * 0.72, 0.012)),
        origin=Origin(xyz=(FLAP_THICKNESS + 0.001, 0.0, -FLAP_HEIGHT + 0.008)),
        material=trim_finish,
        name="flap_lip",
    )
    coin_flap.visual(
        Cylinder(radius=0.0035, length=FLAP_WIDTH * 0.98),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="flap_hinge",
    )

    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(DECK_ORIGIN_X, 0.0, DECK_ORIGIN_Z)),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_coin_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=coin_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_panel = object_model.get_part("control_panel")
    service_door = object_model.get_part("service_door")
    coin_flap = object_model.get_part("coin_flap")
    door_hinge = object_model.get_articulation("body_to_service_door")
    flap_hinge = object_model.get_articulation("body_to_coin_flap")

    body_aabb = ctx.part_world_aabb(body)
    deck_aabb = ctx.part_world_aabb(control_panel)
    ctx.check(
        "control panel projects clearly from cabinet face",
        body_aabb is not None
        and deck_aabb is not None
        and deck_aabb[1][0] > body_aabb[1][0] + 0.17
        and (deck_aabb[1][1] - deck_aabb[0][1]) > 0.60,
        details=f"body_aabb={body_aabb}, deck_aabb={deck_aabb}",
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_overlap(
            service_door,
            body,
            axes="yz",
            elem_a="door_panel",
            min_overlap=0.42,
            name="service door covers the lower front opening",
        )

        closed_door = ctx.part_element_world_aabb(service_door, elem="door_panel")
        closed_flap = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")
        slot_aabb = ctx.part_element_world_aabb(body, elem="coin_slot")
        body_aabb = ctx.part_world_aabb(body)

        ctx.check(
            "service door mounts on the lower front face",
            closed_door is not None
            and body_aabb is not None
            and abs(closed_door[0][0] - body_aabb[1][0]) <= 0.003
            and closed_door[1][0] > body_aabb[1][0] + 0.010,
            details=f"door_aabb={closed_door}, body_aabb={body_aabb}",
        )
        ctx.check(
            "service door stays below the coin area",
            closed_door is not None
            and slot_aabb is not None
            and closed_door[1][2] < slot_aabb[0][2] - 0.08,
            details=f"door_aabb={closed_door}, slot_aabb={slot_aabb}",
        )
        ctx.check(
            "coin flap sits beside and below the coin slot",
            closed_flap is not None
            and slot_aabb is not None
            and ((closed_flap[0][1] + closed_flap[1][1]) / 2.0)
            > ((slot_aabb[0][1] + slot_aabb[1][1]) / 2.0) + 0.12
            and ((closed_flap[0][2] + closed_flap[1][2]) / 2.0)
            < ((slot_aabb[0][2] + slot_aabb[1][2]) / 2.0) - 0.02,
            details=f"flap_aabb={closed_flap}, slot_aabb={slot_aabb}",
        )
        ctx.check(
            "coin flap remains a front-face part",
            closed_flap is not None
            and body_aabb is not None
            and abs(closed_flap[1][0] - body_aabb[1][0]) <= 0.01,
            details=f"flap_aabb={closed_flap}, body_aabb={body_aabb}",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            open_door = ctx.part_element_world_aabb(service_door, elem="door_panel")
            body_aabb = ctx.part_world_aabb(body)
            ctx.check(
                "service door swings outward on its side hinge",
                open_door is not None
                and body_aabb is not None
                and open_door[1][0] > body_aabb[1][0] + 0.10,
                details=f"open_door={open_door}, body_aabb={body_aabb}",
            )

    flap_limits = flap_hinge.motion_limits
    if flap_limits is not None and flap_limits.upper is not None:
        closed_flap = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")
        with ctx.pose({flap_hinge: flap_limits.upper}):
            open_flap = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")
            ctx.check(
                "coin flap rotates outward on its short horizontal hinge",
                closed_flap is not None
                and open_flap is not None
                and open_flap[1][0] > closed_flap[1][0] + 0.03,
                details=f"closed_flap={closed_flap}, open_flap={open_flap}",
            )

    return ctx.report()


object_model = build_object_model()
