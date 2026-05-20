from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

WIDTH = 0.78
HEIGHT = 1.74
SIDE_THK = 0.018
WALL = 0.018
BRIDGE_WIDTH = WIDTH - 2.0 * SIDE_THK + 0.004

LOWER_FRONT_Y = 0.06
BACK_Y = -0.46

FRONT_FACE_HEIGHT = 0.96
DOOR_W = 0.52
DOOR_H = 0.46
DOOR_T = 0.022
DOOR_BOTTOM_Z = 0.22
DOOR_CENTER_Z = DOOR_BOTTOM_Z + DOOR_H / 2.0
DOOR_HINGE_OFFSET = 0.003

TRAY_W = 0.18
TRAY_H = 0.055
TRAY_T = 0.016
TRAY_BOTTOM_Z = 0.082
TRAY_CENTER_Z = TRAY_BOTTOM_Z + TRAY_H / 2.0

DECK_ANGLE_DEG = -18.0
DECK_ANGLE_RAD = math.radians(DECK_ANGLE_DEG)
MONITOR_ANGLE_DEG = -16.0
MONITOR_ANGLE_RAD = math.radians(MONITOR_ANGLE_DEG)


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _ring_panel(
    outer_size: tuple[float, float, float],
    inner_size: tuple[float, float],
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(*outer_size)
    inner = cq.Workplane("XY").box(inner_size[0], outer_size[1] + 0.02, inner_size[1])
    return outer.cut(inner)


def _side_panel_shape() -> cq.Workplane:
    profile = [
        (LOWER_FRONT_Y, 0.0),
        (LOWER_FRONT_Y, 0.94),
        (0.18, 0.99),
        (0.03, 1.08),
        (0.11, 1.23),
        (0.03, 1.47),
        (0.03, HEIGHT),
        (-0.22, HEIGHT),
        (-0.33, 1.48),
        (BACK_Y, 0.90),
        (BACK_Y, 0.0),
    ]
    return cq.Workplane("YZ").polyline(profile).close().extrude(SIDE_THK)


def _front_face_shape() -> cq.Workplane:
    center_y = LOWER_FRONT_Y - WALL / 2.0
    door_side_w = (BRIDGE_WIDTH - DOOR_W) / 2.0
    tray_side_w = (BRIDGE_WIDTH - TRAY_W) / 2.0

    shapes = [
        _box_shape((BRIDGE_WIDTH, WALL, TRAY_BOTTOM_Z), (0.0, center_y, TRAY_BOTTOM_Z / 2.0)),
        _box_shape(
            (tray_side_w, WALL, TRAY_H),
            (-(TRAY_W + tray_side_w) / 2.0, center_y, TRAY_CENTER_Z),
        ),
        _box_shape(
            (tray_side_w, WALL, TRAY_H),
            ((TRAY_W + tray_side_w) / 2.0, center_y, TRAY_CENTER_Z),
        ),
        _box_shape(
            (BRIDGE_WIDTH, WALL, DOOR_BOTTOM_Z - (TRAY_BOTTOM_Z + TRAY_H)),
            (0.0, center_y, (DOOR_BOTTOM_Z + TRAY_BOTTOM_Z + TRAY_H) / 2.0),
        ),
        _box_shape((door_side_w, WALL, DOOR_H), (-(DOOR_W + door_side_w) / 2.0, center_y, DOOR_CENTER_Z)),
        _box_shape(((door_side_w), WALL, DOOR_H), (((DOOR_W + door_side_w) / 2.0), center_y, DOOR_CENTER_Z)),
        _box_shape(
            (BRIDGE_WIDTH, WALL, FRONT_FACE_HEIGHT - (DOOR_BOTTOM_Z + DOOR_H)),
            (0.0, center_y, (FRONT_FACE_HEIGHT + DOOR_BOTTOM_Z + DOOR_H) / 2.0),
        ),
    ]

    front_face = shapes[0]
    for shape in shapes[1:]:
        front_face = front_face.union(shape)
    return front_face


def _coin_panel_shape() -> cq.Workplane:
    panel = _box_shape((0.31, 0.012, 0.18), (0.0, LOWER_FRONT_Y + 0.006, 0.81))
    left_slot = _box_shape((0.034, 0.03, 0.008), (-0.085, LOWER_FRONT_Y + 0.006, 0.85))
    right_slot = _box_shape((0.034, 0.03, 0.008), (0.085, LOWER_FRONT_Y + 0.006, 0.85))
    return panel.cut(left_slot).cut(right_slot)


def _tray_surround_shape() -> cq.Workplane:
    surround = _ring_panel((0.28, 0.012, 0.13), (TRAY_W + 0.018, TRAY_H + 0.012))
    return surround.translate((0.0, LOWER_FRONT_Y + 0.006, TRAY_CENTER_Z))


def _monitor_frame_shape() -> cq.Workplane:
    frame = _ring_panel((0.64, WALL, 0.60), (0.50, 0.38))
    return frame.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), MONITOR_ANGLE_DEG).translate(
        (0.0, -0.02, 1.33)
    )


def _service_door_shape() -> cq.Workplane:
    return _box_shape(
        (DOOR_W, DOOR_T, DOOR_H),
        (DOOR_HINGE_OFFSET + DOOR_W / 2.0, -DOOR_T / 2.0, DOOR_H / 2.0),
    )


def _tray_flap_shape() -> cq.Workplane:
    return _box_shape((TRAY_W, TRAY_T, TRAY_H), (0.0, -TRAY_T / 2.0, TRAY_H / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_cabinet")

    cabinet_black = model.material("cabinet_black", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    deck_blue = model.material("deck_blue", rgba=(0.16, 0.21, 0.29, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.03, 0.05, 0.07, 1.0))
    door_gray = model.material("door_gray", rgba=(0.26, 0.27, 0.29, 1.0))

    body = model.part("body")
    side_panel = _side_panel_shape()
    body.visual(
        mesh_from_cadquery(side_panel.translate((-WIDTH / 2.0, 0.0, 0.0)), "left_side"),
        material=cabinet_black,
        name="left_side",
    )
    body.visual(
        mesh_from_cadquery(side_panel.translate((WIDTH / 2.0 - SIDE_THK, 0.0, 0.0)), "right_side"),
        material=cabinet_black,
        name="right_side",
    )
    body.visual(
        mesh_from_cadquery(_coin_panel_shape(), "coin_panel"),
        material=trim_gray,
        name="coin_panel",
    )
    door_side_w = (BRIDGE_WIDTH - DOOR_W) / 2.0
    tray_side_w = (BRIDGE_WIDTH - TRAY_W) / 2.0
    center_y = LOWER_FRONT_Y - WALL / 2.0
    body.visual(
        Box((BRIDGE_WIDTH, WALL, TRAY_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, center_y, TRAY_BOTTOM_Z / 2.0)),
        material=cabinet_black,
        name="front_base",
    )
    body.visual(
        Box((tray_side_w, WALL, TRAY_H)),
        origin=Origin(xyz=(-(TRAY_W + tray_side_w) / 2.0, center_y, TRAY_CENTER_Z)),
        material=cabinet_black,
        name="tray_side_0",
    )
    body.visual(
        Box((tray_side_w, WALL, TRAY_H)),
        origin=Origin(xyz=((TRAY_W + tray_side_w) / 2.0, center_y, TRAY_CENTER_Z)),
        material=cabinet_black,
        name="tray_side_1",
    )
    body.visual(
        Box((BRIDGE_WIDTH, WALL, DOOR_BOTTOM_Z - (TRAY_BOTTOM_Z + TRAY_H))),
        origin=Origin(xyz=(0.0, center_y, (DOOR_BOTTOM_Z + TRAY_BOTTOM_Z + TRAY_H) / 2.0)),
        material=cabinet_black,
        name="mid_rail",
    )
    body.visual(
        Box((door_side_w, WALL, DOOR_H)),
        origin=Origin(xyz=(-(DOOR_W + door_side_w) / 2.0, center_y, DOOR_CENTER_Z)),
        material=cabinet_black,
        name="door_stile_0",
    )
    body.visual(
        Box((door_side_w, WALL, DOOR_H)),
        origin=Origin(xyz=((DOOR_W + door_side_w) / 2.0, center_y, DOOR_CENTER_Z)),
        material=cabinet_black,
        name="door_stile_1",
    )
    body.visual(
        Box((BRIDGE_WIDTH, WALL, FRONT_FACE_HEIGHT - (DOOR_BOTTOM_Z + DOOR_H))),
        origin=Origin(xyz=(0.0, center_y, (FRONT_FACE_HEIGHT + DOOR_BOTTOM_Z + DOOR_H) / 2.0)),
        material=cabinet_black,
        name="front_top",
    )
    body.visual(
        mesh_from_cadquery(_tray_surround_shape(), "tray_surround"),
        material=trim_gray,
        name="tray_surround",
    )
    body.visual(
        Box((BRIDGE_WIDTH, WALL, 0.12)),
        origin=Origin(xyz=(0.0, 0.132, 0.94)),
        material=trim_gray,
        name="control_apron",
    )
    body.visual(
        Box((BRIDGE_WIDTH, 0.30, 0.024)),
        origin=Origin(xyz=(0.0, 0.04, 1.035), rpy=(DECK_ANGLE_RAD, 0.0, 0.0)),
        material=deck_blue,
        name="control_deck",
    )
    body.visual(
        mesh_from_cadquery(_monitor_frame_shape(), "monitor_frame"),
        material=trim_gray,
        name="monitor_frame",
    )
    body.visual(
        Box((0.58, 0.06, 0.48)),
        origin=Origin(xyz=(0.0, -0.055, 1.31), rpy=(MONITOR_ANGLE_RAD, 0.0, 0.0)),
        material=cabinet_black,
        name="monitor_mount",
    )
    body.visual(
        Box((0.50, 0.018, 0.38)),
        origin=Origin(xyz=(0.0, -0.012, 1.325), rpy=(MONITOR_ANGLE_RAD, 0.0, 0.0)),
        material=screen_dark,
        name="screen_glass",
    )
    body.visual(
        Box((BRIDGE_WIDTH, WALL, 0.22)),
        origin=Origin(xyz=(0.0, 0.01, 1.60)),
        material=trim_gray,
        name="marquee_face",
    )
    body.visual(
        Box((BRIDGE_WIDTH, 0.26, WALL)),
        origin=Origin(xyz=(0.0, -0.10, HEIGHT - WALL / 2.0)),
        material=cabinet_black,
        name="roof",
    )
    body.visual(
        Box((BRIDGE_WIDTH, WALL, 0.96)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL / 2.0, 0.48)),
        material=cabinet_black,
        name="rear_lower",
    )
    body.visual(
        Box((BRIDGE_WIDTH, WALL, 0.72)),
        origin=Origin(xyz=(0.0, -0.31, 1.38)),
        material=cabinet_black,
        name="rear_upper",
    )
    body.visual(
        Box((BRIDGE_WIDTH, 0.49, WALL)),
        origin=Origin(xyz=(0.0, -0.20, WALL / 2.0)),
        material=cabinet_black,
        name="floor",
    )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(_service_door_shape(), "service_door"),
        material=door_gray,
        name="service_panel",
    )
    service_door.visual(
        Box((0.034, 0.028, 0.12)),
        origin=Origin(xyz=(DOOR_HINGE_OFFSET + DOOR_W * 0.78, 0.008, DOOR_H * 0.56)),
        material=trim_gray,
        name="service_handle",
    )

    tray_flap = model.part("tray_flap")
    tray_flap.visual(
        mesh_from_cadquery(_tray_flap_shape(), "tray_flap"),
        material=door_gray,
        name="tray_panel",
    )
    tray_flap.visual(
        Box((0.10, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, TRAY_H * 0.62)),
        material=trim_gray,
        name="tray_pull",
    )

    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(
            xyz=(
                -DOOR_W / 2.0 - DOOR_HINGE_OFFSET,
                LOWER_FRONT_Y + 0.004,
                DOOR_BOTTOM_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.22, effort=20.0, velocity=1.2),
    )
    model.articulation(
        "body_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray_flap,
        origin=Origin(xyz=(0.0, LOWER_FRONT_Y + 0.012, TRAY_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.12, effort=4.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    service_door = object_model.get_part("service_door")
    tray_flap = object_model.get_part("tray_flap")
    service_hinge = object_model.get_articulation("body_to_service_door")
    tray_hinge = object_model.get_articulation("body_to_tray_flap")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected a world AABB for the cabinet body.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        width = float(maxs[0] - mins[0])
        depth = float(maxs[1] - mins[1])
        height = float(maxs[2] - mins[2])
        ctx.check("arcade_width", 0.72 <= width <= 0.86, details=f"width={width:.3f}")
        ctx.check("arcade_depth", 0.58 <= depth <= 0.75, details=f"depth={depth:.3f}")
        ctx.check("arcade_height", 1.65 <= height <= 1.85, details=f"height={height:.3f}")

    ctx.expect_overlap(
        service_door,
        body,
        axes="xz",
        elem_a="service_panel",
        min_overlap=0.20,
        name="service door covers the lower cabinet opening",
    )
    ctx.expect_overlap(
        tray_flap,
        body,
        axes="xz",
        elem_a="tray_panel",
        elem_b="tray_surround",
        min_overlap=0.04,
        name="tray flap stays aligned with the tray surround",
    )

    door_panel_aabb = ctx.part_element_world_aabb(service_door, elem="service_panel")
    door_stile_aabb = ctx.part_element_world_aabb(body, elem="door_stile_0")
    tray_panel_aabb = ctx.part_element_world_aabb(tray_flap, elem="tray_panel")
    tray_surround_aabb = ctx.part_element_world_aabb(body, elem="tray_surround")
    coin_panel_aabb = ctx.part_element_world_aabb(body, elem="coin_panel")

    def elem_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) / 2.0 for i in range(3))

    ctx.check(
        "service door sits near front face",
        door_panel_aabb is not None
        and door_stile_aabb is not None
        and 0.0 <= float(door_panel_aabb[1][1] - door_stile_aabb[1][1]) <= 0.012,
        details=f"door={door_panel_aabb}, stile={door_stile_aabb}",
    )
    ctx.check(
        "tray flap sits near tray surround",
        tray_panel_aabb is not None
        and tray_surround_aabb is not None
        and 0.0 <= float(tray_panel_aabb[1][1] - tray_surround_aabb[1][1]) <= 0.010,
        details=f"tray={tray_panel_aabb}, surround={tray_surround_aabb}",
    )
    ctx.check(
        "coin area stays above service door",
        door_panel_aabb is not None
        and coin_panel_aabb is not None
        and float(coin_panel_aabb[0][2]) > float(door_panel_aabb[1][2]) + 0.02,
        details=f"door={door_panel_aabb}, coin={coin_panel_aabb}",
    )

    door_rest = elem_center(door_panel_aabb)
    tray_rest = elem_center(tray_panel_aabb)
    service_upper = service_hinge.motion_limits.upper if service_hinge.motion_limits is not None else None
    tray_upper = tray_hinge.motion_limits.upper if tray_hinge.motion_limits is not None else None

    door_open = None
    if service_upper is not None:
        with ctx.pose({service_hinge: service_upper}):
            door_open = elem_center(ctx.part_element_world_aabb(service_door, elem="service_panel"))
    ctx.check(
        "service door swings outward",
        door_rest is not None
        and door_open is not None
        and float(door_open[1]) > float(door_rest[1]) + 0.10,
        details=f"rest={door_rest}, open={door_open}",
    )

    tray_open = None
    if tray_upper is not None:
        with ctx.pose({tray_hinge: tray_upper}):
            tray_open = elem_center(ctx.part_element_world_aabb(tray_flap, elem="tray_panel"))
    ctx.check(
        "tray flap opens downward and outward",
        tray_rest is not None
        and tray_open is not None
        and float(tray_open[1]) > float(tray_rest[1]) + 0.015
        and float(tray_open[2]) < float(tray_rest[2]) - 0.005,
        details=f"rest={tray_rest}, open={tray_open}",
    )

    return ctx.report()


object_model = build_object_model()
