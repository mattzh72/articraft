from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.598
BODY_DEPTH = 0.600
BODY_HEIGHT = 0.860
SIDE_WALL = 0.018
BACK_WALL = 0.016
TOP_WALL = 0.018
TUB_FLOOR_Z = 0.145
TOE_KICK_HEIGHT = 0.100
TOE_RECESS_DEPTH = 0.075

DOOR_WIDTH = 0.590
DOOR_HEIGHT = 0.750
DOOR_THICKNESS = 0.032
DOOR_BOTTOM_Z = 0.098
DOOR_OUTER_SKIN = 0.0032
DOOR_FRAME = 0.026
DOOR_LINER_DEPTH = 0.018
DOOR_LINER_WALL = 0.0025

LOWER_RACK_WIDTH = 0.462
LOWER_RACK_DEPTH = 0.460
LOWER_RACK_HEIGHT = 0.145
LOWER_RACK_Y = -0.090
LOWER_RACK_Z = 0.234
LOWER_RACK_TRAVEL = 0.330

UPPER_RACK_WIDTH = 0.450
UPPER_RACK_DEPTH = 0.445
UPPER_RACK_HEIGHT = 0.115
UPPER_RACK_Y = -0.100
UPPER_RACK_Z = 0.448
UPPER_RACK_TRAVEL = 0.290

TRAY_WIDTH = 0.432
TRAY_DEPTH = 0.420
TRAY_HEIGHT = 0.052
TRAY_Y = -0.112
TRAY_Z = 0.646
TRAY_TRAVEL = 0.235

TRACK_X_INNER = 0.238
TRACK_X_OUTER = 0.272
TRACK_Y_REAR = -0.525
TRACK_Y_FRONT = -0.110

LOWER_SPRAY_Y = -0.320
LOWER_SPRAY_Z = 0.180
UPPER_SPRAY_LOCAL_Y = -0.250

CONTROL_BANK_Z = DOOR_HEIGHT - 0.150
CONTROL_MOUNT_Y = DOOR_THICKNESS - DOOR_OUTER_SKIN
BUTTON_XS = (-0.128, -0.082, 0.082, 0.128)
RINSE_AID_X = DOOR_WIDTH / 2.0 - 0.120
RINSE_AID_Z = 0.192


def _box_from_bounds(
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0)
        .translate(((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0))
    )


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    merged = shapes[0]
    for shape in shapes[1:]:
        merged = merged.union(shape)
    return merged


def _track_pair_shape(z0: float, z1: float, x_inner: float, x_outer: float) -> cq.Workplane:
    left_shapes = [
        _box_from_bounds(-x_outer, -x_inner, TRACK_Y_REAR, TRACK_Y_FRONT, z0, z1),
    ]
    right_shapes = [
        _box_from_bounds(x_inner, x_outer, TRACK_Y_REAR, TRACK_Y_FRONT, z0, z1),
    ]
    for y_center in (-0.500, -0.320, -0.145):
        left_shapes.append(
            _box_from_bounds(
                -BODY_WIDTH / 2.0 + SIDE_WALL - 0.001,
                -x_outer + 0.003,
                y_center - 0.015,
                y_center + 0.015,
                z0,
                z1,
            )
        )
        right_shapes.append(
            _box_from_bounds(
                x_outer - 0.003,
                BODY_WIDTH / 2.0 - SIDE_WALL + 0.001,
                y_center - 0.015,
                y_center + 0.015,
                z0,
                z1,
            )
        )
    return _union_all([_union_all(left_shapes), _union_all(right_shapes)])


def _body_shape() -> cq.Workplane:
    outer = _box_from_bounds(
        -BODY_WIDTH / 2.0,
        BODY_WIDTH / 2.0,
        -BODY_DEPTH,
        0.0,
        0.0,
        BODY_HEIGHT,
    )
    tub_cavity = _box_from_bounds(
        -BODY_WIDTH / 2.0 + SIDE_WALL,
        BODY_WIDTH / 2.0 - SIDE_WALL,
        -BODY_DEPTH + BACK_WALL,
        0.025,
        TUB_FLOOR_Z,
        BODY_HEIGHT - TOP_WALL,
    )
    toe_recess = _box_from_bounds(
        -BODY_WIDTH / 2.0 + 0.035,
        BODY_WIDTH / 2.0 - 0.035,
        -TOE_RECESS_DEPTH,
        0.015,
        0.0,
        TOE_KICK_HEIGHT,
    )
    body = outer.cut(tub_cavity).cut(toe_recess)
    return body.union(
        _union_all(
            [
                _box_from_bounds(
                    -BODY_WIDTH / 2.0 + SIDE_WALL,
                    BODY_WIDTH / 2.0 - SIDE_WALL,
                    -BODY_DEPTH + BACK_WALL,
                    -BODY_DEPTH + BACK_WALL + 0.020,
                    TUB_FLOOR_Z,
                    TUB_FLOOR_Z + 0.018,
                ),
                _track_pair_shape(LOWER_RACK_Z, LOWER_RACK_Z + 0.010, 0.2585, 0.276),
                _track_pair_shape(UPPER_RACK_Z, UPPER_RACK_Z + 0.010, 0.2525, 0.270),
                _track_pair_shape(TRAY_Z, TRAY_Z + 0.008, 0.237, 0.251),
                _box_from_bounds(
                    -0.022,
                    0.022,
                    LOWER_SPRAY_Y - 0.022,
                    LOWER_SPRAY_Y + 0.022,
                    TUB_FLOOR_Z,
                    LOWER_SPRAY_Z - 0.004,
                ),
                _box_from_bounds(
                    -0.032,
                    0.032,
                    LOWER_SPRAY_Y - 0.032,
                    LOWER_SPRAY_Y + 0.032,
                    LOWER_SPRAY_Z - 0.010,
                    LOWER_SPRAY_Z - 0.006,
                ),
            ]
        )
    )


def _liner_pan_shape() -> cq.Workplane:
    pan_outer = _box_from_bounds(
        -DOOR_WIDTH / 2.0 + DOOR_FRAME,
        DOOR_WIDTH / 2.0 - DOOR_FRAME,
        0.0,
        DOOR_LINER_DEPTH,
        DOOR_FRAME,
        DOOR_HEIGHT - DOOR_FRAME,
    )
    pan_inner = _box_from_bounds(
        -DOOR_WIDTH / 2.0 + DOOR_FRAME + 0.016,
        DOOR_WIDTH / 2.0 - DOOR_FRAME - 0.016,
        -0.002,
        DOOR_LINER_DEPTH - DOOR_LINER_WALL,
        DOOR_FRAME + 0.016,
        DOOR_HEIGHT - DOOR_FRAME - 0.016,
    )
    return pan_outer.cut(pan_inner)


def _door_liner_shape() -> cq.Workplane:
    return _liner_pan_shape().union(
        _box_from_bounds(
            RINSE_AID_X - 0.030,
            RINSE_AID_X + 0.030,
            DOOR_LINER_DEPTH - 0.004,
            DOOR_LINER_DEPTH,
            RINSE_AID_Z - 0.028,
            RINSE_AID_Z + 0.028,
        )
    )


def _door_outer_shape() -> cq.Workplane:
    outer_skin = _box_from_bounds(
        -DOOR_WIDTH / 2.0,
        DOOR_WIDTH / 2.0,
        DOOR_THICKNESS - DOOR_OUTER_SKIN,
        DOOR_THICKNESS,
        0.0,
        DOOR_HEIGHT,
    )
    left_frame = _box_from_bounds(
        -DOOR_WIDTH / 2.0,
        -DOOR_WIDTH / 2.0 + DOOR_FRAME,
        0.0,
        DOOR_THICKNESS,
        0.0,
        DOOR_HEIGHT,
    )
    right_frame = _box_from_bounds(
        DOOR_WIDTH / 2.0 - DOOR_FRAME,
        DOOR_WIDTH / 2.0,
        0.0,
        DOOR_THICKNESS,
        0.0,
        DOOR_HEIGHT,
    )
    top_frame = _box_from_bounds(
        -DOOR_WIDTH / 2.0,
        DOOR_WIDTH / 2.0,
        0.0,
        DOOR_THICKNESS,
        DOOR_HEIGHT - DOOR_FRAME,
        DOOR_HEIGHT,
    )
    bottom_frame = _box_from_bounds(
        -DOOR_WIDTH / 2.0,
        DOOR_WIDTH / 2.0,
        0.0,
        DOOR_THICKNESS,
        0.0,
        DOOR_FRAME,
    )
    door = outer_skin.union(left_frame).union(right_frame).union(top_frame).union(bottom_frame)

    handle_pocket = _box_from_bounds(
        -0.220,
        0.220,
        DOOR_THICKNESS - 0.026,
        DOOR_THICKNESS + 0.004,
        DOOR_HEIGHT - 0.118,
        DOOR_HEIGHT - 0.062,
    )
    handle_bar = _box_from_bounds(
        -0.175,
        0.175,
        DOOR_THICKNESS - 0.020,
        DOOR_THICKNESS - 0.010,
        DOOR_HEIGHT - 0.101,
        DOOR_HEIGHT - 0.084,
    )
    left_handle_bridge = _box_from_bounds(
        -0.236,
        -0.175,
        DOOR_THICKNESS - 0.020,
        DOOR_THICKNESS - 0.006,
        DOOR_HEIGHT - 0.110,
        DOOR_HEIGHT - 0.075,
    )
    right_handle_bridge = _box_from_bounds(
        0.175,
        0.236,
        DOOR_THICKNESS - 0.020,
        DOOR_THICKNESS - 0.006,
        DOOR_HEIGHT - 0.110,
        DOOR_HEIGHT - 0.075,
    )
    control_bezel = _box_from_bounds(
        -0.180,
        0.180,
        DOOR_THICKNESS - 0.0020,
        DOOR_THICKNESS + 0.0020,
        CONTROL_BANK_Z - 0.030,
        CONTROL_BANK_Z + 0.030,
    )
    door = door.cut(handle_pocket).cut(control_bezel)
    door = door.cut(
        _box_from_bounds(
            -0.015,
            0.015,
            DOOR_THICKNESS - DOOR_OUTER_SKIN - 0.002,
            DOOR_THICKNESS + 0.002,
            CONTROL_BANK_Z - 0.015,
            CONTROL_BANK_Z + 0.015,
        )
    )
    for x_pos in BUTTON_XS:
        door = door.cut(
            _box_from_bounds(
                x_pos - 0.014,
                x_pos + 0.014,
                DOOR_THICKNESS - DOOR_OUTER_SKIN - 0.002,
                DOOR_THICKNESS + 0.002,
                CONTROL_BANK_Z - 0.014,
                CONTROL_BANK_Z + 0.014,
            )
        )
    return door.union(handle_bar).union(left_handle_bridge).union(right_handle_bridge)


def _rack_shape(
    width: float,
    depth: float,
    height: float,
    *,
    tray: bool = False,
    tines: int = 0,
    spray_mount: bool = False,
) -> cq.Workplane:
    rod = 0.006 if tray else 0.008
    runner_w = 0.012 if tray else 0.015
    runner_h = 0.008 if tray else 0.010
    floor_z = 0.010 if tray else 0.020
    top_z = height
    runner_center = width / 2.0 + (0.015 if tray else 0.020)
    basket_y_front = 0.0
    basket_y_rear = -depth

    shapes = [
        _box_from_bounds(-runner_center - runner_w / 2.0, -runner_center + runner_w / 2.0, basket_y_rear + 0.028, basket_y_front + 0.018, 0.0, runner_h),
        _box_from_bounds(runner_center - runner_w / 2.0, runner_center + runner_w / 2.0, basket_y_rear + 0.028, basket_y_front + 0.018, 0.0, runner_h),
        _box_from_bounds(-width / 2.0, width / 2.0, basket_y_front - rod, basket_y_front, floor_z, floor_z + rod),
        _box_from_bounds(-width / 2.0, width / 2.0, basket_y_rear, basket_y_rear + rod, floor_z, floor_z + rod),
        _box_from_bounds(-width / 2.0, -width / 2.0 + rod, basket_y_rear, basket_y_front, floor_z, floor_z + rod),
        _box_from_bounds(width / 2.0 - rod, width / 2.0, basket_y_rear, basket_y_front, floor_z, floor_z + rod),
        _box_from_bounds(-width / 2.0, width / 2.0, basket_y_front - rod, basket_y_front, top_z, top_z + rod),
        _box_from_bounds(-width / 2.0, width / 2.0, basket_y_rear, basket_y_rear + rod, top_z, top_z + rod),
        _box_from_bounds(-width / 2.0, -width / 2.0 + rod, basket_y_rear, basket_y_front, top_z, top_z + rod),
        _box_from_bounds(width / 2.0 - rod, width / 2.0, basket_y_rear, basket_y_front, top_z, top_z + rod),
        _box_from_bounds(-width / 2.0, -width / 2.0 + rod, basket_y_front - rod, basket_y_front, floor_z, top_z + rod),
        _box_from_bounds(width / 2.0 - rod, width / 2.0, basket_y_front - rod, basket_y_front, floor_z, top_z + rod),
        _box_from_bounds(-width / 2.0, -width / 2.0 + rod, basket_y_rear, basket_y_rear + rod, floor_z, top_z + rod),
        _box_from_bounds(width / 2.0 - rod, width / 2.0, basket_y_rear, basket_y_rear + rod, floor_z, top_z + rod),
        _box_from_bounds(-runner_center, -width / 2.0 + rod, basket_y_front - rod, basket_y_front + 0.016, runner_h - 0.001, floor_z + rod),
        _box_from_bounds(runner_center - rod, width / 2.0, basket_y_front - rod, basket_y_front + 0.016, runner_h - 0.001, floor_z + rod),
        _box_from_bounds(-runner_center, -width / 2.0 + rod, basket_y_rear - 0.016, basket_y_rear + rod, runner_h - 0.001, floor_z + rod),
        _box_from_bounds(runner_center - rod, width / 2.0, basket_y_rear - 0.016, basket_y_rear + rod, runner_h - 0.001, floor_z + rod),
    ]

    longitudinal_count = 5 if tray else 6
    for index in range(longitudinal_count):
        ratio = (index + 1) / (longitudinal_count + 1)
        x_mid = -width / 2.0 + 0.035 + ratio * (width - 0.070)
        shapes.append(
            _box_from_bounds(
                x_mid - rod / 2.0,
                x_mid + rod / 2.0,
                basket_y_rear + 0.020,
                basket_y_front - 0.006,
                floor_z,
                floor_z + rod,
            )
        )

    transverse_count = 4 if tray else 5
    for index in range(transverse_count):
        ratio = (index + 1) / (transverse_count + 1)
        y_mid = basket_y_rear + 0.040 + ratio * (depth - 0.080)
        shapes.append(
            _box_from_bounds(
                -width / 2.0 + 0.018,
                width / 2.0 - 0.018,
                y_mid - rod / 2.0,
                y_mid + rod / 2.0,
                floor_z,
                floor_z + rod,
            )
        )

    if tray:
        for div_x in (-0.105, 0.0, 0.105):
            shapes.append(
                _box_from_bounds(
                    div_x - rod / 2.0,
                    div_x + rod / 2.0,
                    basket_y_rear + 0.030,
                    basket_y_front - 0.010,
                    floor_z + rod,
                    top_z - 0.004,
                )
            )
        shapes.append(
            _box_from_bounds(
                -width / 2.0,
                width / 2.0,
                basket_y_front - 0.018,
                basket_y_front - 0.006,
                top_z - 0.004,
                top_z + 0.006,
            )
        )
    else:
        tine_height = 0.062 if height > 0.13 else 0.045
        row_ys = (-depth * 0.30, -depth * 0.58) if tines > 1 else (-depth * 0.46,)
        for row_y in row_ys:
            for index in range(7):
                ratio = index / 6.0
                x_mid = -width / 2.0 + 0.040 + ratio * (width - 0.080)
                shapes.append(
                    _box_from_bounds(
                        x_mid - rod / 2.5,
                        x_mid + rod / 2.5,
                        row_y - rod / 2.0,
                        row_y + rod / 2.0,
                        floor_z + rod,
                        floor_z + rod + tine_height,
                    )
                )

    if spray_mount:
        shapes.append(
            _box_from_bounds(
                -0.060,
                0.060,
                UPPER_SPRAY_LOCAL_Y - 0.008,
                UPPER_SPRAY_LOCAL_Y + 0.008,
                floor_z - 0.002,
                floor_z + 0.006,
            )
        )
        shapes.append(
            _box_from_bounds(
                -0.018,
                0.018,
                UPPER_SPRAY_LOCAL_Y - 0.018,
                UPPER_SPRAY_LOCAL_Y + 0.018,
                0.004,
                floor_z + 0.012,
            )
        )

    return _union_all(shapes)


def _add_box_visual(part, size, xyz, material, name: str | None = None) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_rack_visuals(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    material,
    tray: bool = False,
    tines: int = 0,
    spray_mount: bool = False,
) -> None:
    rod = 0.006 if tray else 0.008
    runner_w = 0.012 if tray else 0.015
    runner_h = 0.008 if tray else 0.010
    floor_z = 0.010 if tray else 0.020
    top_z = height
    runner_center = width / 2.0 + (0.015 if tray else 0.020)

    def add_box(x0, x1, y0, y1, z0, z1, name: str | None = None) -> None:
        x_min, x_max = sorted((x0, x1))
        y_min, y_max = sorted((y0, y1))
        z_min, z_max = sorted((z0, z1))
        _add_box_visual(
            part,
            (x_max - x_min, y_max - y_min, z_max - z_min),
            ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, (z_min + z_max) / 2.0),
            material,
            name=name,
        )

    add_box(-runner_center - runner_w / 2.0, -runner_center + runner_w / 2.0, -depth + 0.028, 0.018, 0.0, runner_h, name="runner_left")
    add_box(runner_center - runner_w / 2.0, runner_center + runner_w / 2.0, -depth + 0.028, 0.018, 0.0, runner_h, name="runner_right")
    add_box(-width / 2.0, width / 2.0, -rod, 0.0, floor_z, floor_z + rod)
    add_box(-width / 2.0, width / 2.0, -depth, -depth + rod, floor_z, floor_z + rod)
    add_box(-width / 2.0, -width / 2.0 + rod, -depth, 0.0, floor_z, floor_z + rod)
    add_box(width / 2.0 - rod, width / 2.0, -depth, 0.0, floor_z, floor_z + rod)
    add_box(-width / 2.0, width / 2.0, -rod, 0.0, top_z, top_z + rod)
    add_box(-width / 2.0, width / 2.0, -depth, -depth + rod, top_z, top_z + rod)
    add_box(-width / 2.0, -width / 2.0 + rod, -depth, 0.0, top_z, top_z + rod)
    add_box(width / 2.0 - rod, width / 2.0, -depth, 0.0, top_z, top_z + rod)
    add_box(-width / 2.0, -width / 2.0 + rod, -rod, 0.0, floor_z, top_z + rod)
    add_box(width / 2.0 - rod, width / 2.0, -rod, 0.0, floor_z, top_z + rod)
    add_box(-width / 2.0, -width / 2.0 + rod, -depth, -depth + rod, floor_z, top_z + rod)
    add_box(width / 2.0 - rod, width / 2.0, -depth, -depth + rod, floor_z, top_z + rod)
    add_box(-runner_center, -width / 2.0 + rod, -rod, 0.016, runner_h - 0.001, floor_z + rod)
    add_box(width / 2.0 - rod, runner_center, -rod, 0.016, runner_h - 0.001, floor_z + rod)
    add_box(-runner_center, -width / 2.0 + rod, -depth - 0.016, -depth + rod, runner_h - 0.001, floor_z + rod)
    add_box(width / 2.0 - rod, runner_center, -depth - 0.016, -depth + rod, runner_h - 0.001, floor_z + rod)

    longitudinal_count = 5 if tray else 6
    for index in range(longitudinal_count):
        ratio = (index + 1) / (longitudinal_count + 1)
        x_mid = -width / 2.0 + 0.035 + ratio * (width - 0.070)
        add_box(x_mid - rod / 2.0, x_mid + rod / 2.0, -depth + 0.020, -0.006, floor_z, floor_z + rod)

    transverse_count = 4 if tray else 5
    for index in range(transverse_count):
        ratio = (index + 1) / (transverse_count + 1)
        y_mid = -depth + 0.040 + ratio * (depth - 0.080)
        add_box(-width / 2.0 + 0.018, width / 2.0 - 0.018, y_mid - rod / 2.0, y_mid + rod / 2.0, floor_z, floor_z + rod)

    if tray:
        for div_x in (-0.105, 0.0, 0.105):
            add_box(div_x - rod / 2.0, div_x + rod / 2.0, -depth + 0.030, -0.010, floor_z + rod, top_z - 0.004)
        add_box(-width / 2.0, width / 2.0, -0.018, -0.006, top_z - 0.004, top_z + 0.006)
    else:
        tine_height = 0.062 if height > 0.13 else 0.045
        row_ys = (-depth * 0.30, -depth * 0.58) if tines > 1 else (-depth * 0.46,)
        for row_y in row_ys:
            add_box(-width / 2.0 + 0.020, width / 2.0 - 0.020, row_y - rod / 2.0, row_y + rod / 2.0, floor_z, floor_z + rod)
            for index in range(7):
                ratio = index / 6.0
                x_mid = -width / 2.0 + 0.040 + ratio * (width - 0.080)
                add_box(x_mid - rod / 2.5, x_mid + rod / 2.5, row_y - rod / 2.0, row_y + rod / 2.0, floor_z + rod, floor_z + rod + tine_height)

    if spray_mount:
        add_box(-0.060, 0.060, UPPER_SPRAY_LOCAL_Y - 0.008, UPPER_SPRAY_LOCAL_Y + 0.008, floor_z - 0.002, floor_z + 0.006)
        part.visual(
            Cylinder(radius=0.018, length=floor_z + 0.008),
            origin=Origin(xyz=(0.0, UPPER_SPRAY_LOCAL_Y, (floor_z + 0.008) / 2.0)),
            material=material,
            name="spray_mount",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_tub_dishwasher")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    button_gray = model.material("button_gray", rgba=(0.88, 0.89, 0.90, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_body_shape(), "cabinet_shell"),
        material=stainless,
        name="shell",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _box_from_bounds(
                -BODY_WIDTH / 2.0,
                BODY_WIDTH / 2.0,
                -0.012,
                0.0,
                BODY_HEIGHT - 0.030,
                BODY_HEIGHT,
            ),
            "countertop_edge",
        ),
        material=dark_trim,
        name="top_edge",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_outer_shape(), "door_outer"),
        material=panel_gray,
        name="outer_shell",
    )
    door.visual(
        mesh_from_cadquery(_door_liner_shape(), "door_liner"),
        material=stainless,
        name="liner",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, DOOR_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    lower_rack = model.part("lower_rack")
    _add_rack_visuals(
        lower_rack,
        width=LOWER_RACK_WIDTH,
        depth=LOWER_RACK_DEPTH,
        height=LOWER_RACK_HEIGHT,
        material=rack_gray,
        tines=2,
    )
    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, LOWER_RACK_Y, LOWER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=LOWER_RACK_TRAVEL,
        ),
    )

    upper_rack = model.part("upper_rack")
    _add_rack_visuals(
        upper_rack,
        width=UPPER_RACK_WIDTH,
        depth=UPPER_RACK_DEPTH,
        height=UPPER_RACK_HEIGHT,
        material=rack_gray,
        tines=1,
        spray_mount=True,
    )
    model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, UPPER_RACK_Y, UPPER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.35,
            lower=0.0,
            upper=UPPER_RACK_TRAVEL,
        ),
    )

    utensil_tray = model.part("utensil_tray")
    _add_rack_visuals(
        utensil_tray,
        width=TRAY_WIDTH,
        depth=TRAY_DEPTH,
        height=TRAY_HEIGHT,
        material=rack_gray,
        tray=True,
    )
    model.articulation(
        "cabinet_to_utensil_tray",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=utensil_tray,
        origin=Origin(xyz=(0.0, TRAY_Y, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.30,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        Cylinder(radius=0.024, length=0.008),
        material=dark_trim,
        name="hub",
    )
    lower_spray_arm.visual(
        Box((0.395, 0.032, 0.008)),
        material=dark_trim,
        name="blade",
    )
    lower_spray_arm.visual(
        Box((0.110, 0.018, 0.008)),
        origin=Origin(xyz=(0.101, 0.024, 0.0)),
        material=dark_trim,
        name="nozzle_rib",
    )
    model.articulation(
        "cabinet_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, LOWER_SPRAY_Y, LOWER_SPRAY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        Cylinder(radius=0.020, length=0.008),
        material=dark_trim,
        name="hub",
    )
    upper_spray_arm.visual(
        Box((0.305, 0.026, 0.008)),
        material=dark_trim,
        name="blade",
    )
    upper_spray_arm.visual(
        Box((0.120, 0.018, 0.008)),
        origin=Origin(xyz=(-0.058, -0.022, 0.0)),
        material=dark_trim,
        name="nozzle_rib",
    )
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, UPPER_SPRAY_LOCAL_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=18.0),
    )

    rinse_aid_cap = model.part("rinse_aid_cap")
    rinse_aid_cap.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, -0.0055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="cap",
    )
    rinse_aid_cap.visual(
        Cylinder(radius=0.010, length=0.0025),
        origin=Origin(xyz=(0.0, -0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_gray,
        name="center",
    )
    model.articulation(
        "door_to_rinse_aid_cap",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=rinse_aid_cap,
        origin=Origin(xyz=(RINSE_AID_X, DOOR_LINER_DEPTH - 0.0005, RINSE_AID_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    control_dial = model.part("control_dial")
    control_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.018,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.052, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised"),
                center=False,
            ),
            "control_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="dial",
    )
    model.articulation(
        "door_to_control_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=control_dial,
        origin=Origin(xyz=(0.0, CONTROL_MOUNT_Y, CONTROL_BANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=10.0),
    )

    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"option_button_{index}")
        button.visual(
            Box((0.022, 0.007, 0.014)),
            origin=Origin(xyz=(0.0, 0.0035, 0.007)),
            material=button_gray,
            name="cap",
        )
        button.visual(
            Box((0.010, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, -0.004, 0.006)),
            material=dark_trim,
            name="stem",
        )
        model.articulation(
            f"door_to_option_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_MOUNT_Y, CONTROL_BANK_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=0.06,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    utensil_tray = object_model.get_part("utensil_tray")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")
    control_dial = object_model.get_part("control_dial")
    rinse_aid_cap = object_model.get_part("rinse_aid_cap")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    lower_joint = object_model.get_articulation("cabinet_to_lower_rack")
    upper_joint = object_model.get_articulation("cabinet_to_upper_rack")
    tray_joint = object_model.get_articulation("cabinet_to_utensil_tray")
    button_joint = object_model.get_articulation("door_to_option_button_0")

    for rack in (lower_rack, upper_rack, utensil_tray):
        ctx.allow_overlap(
            cabinet,
            rack,
            elem_a="shell",
            elem_b="runner_left",
            reason="The rack runner is intentionally represented as bearing against a simplified side-rail proxy in the cabinet shell.",
        )
        ctx.allow_overlap(
            cabinet,
            rack,
            elem_a="shell",
            elem_b="runner_right",
            reason="The rack runner is intentionally represented as bearing against a simplified side-rail proxy in the cabinet shell.",
        )

    ctx.allow_overlap(
        cabinet,
        lower_spray_arm,
        elem_a="shell",
        elem_b="hub",
        reason="The lower spray arm hub is intentionally seated on a simplified tub-floor hub proxy inside the cabinet shell.",
    )
    ctx.allow_overlap(
        cabinet,
        lower_spray_arm,
        elem_a="shell",
        elem_b="blade",
        reason="The lower spray arm is seated on a simplified tub-floor hub proxy inside the cabinet shell.",
    )
    ctx.allow_overlap(
        door,
        rinse_aid_cap,
        elem_a="liner",
        elem_b="cap",
        reason="The rinse-aid cap is intentionally seated into a shallow simplified boss on the inner door liner.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed door sits flush with cabinet front",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.45,
            name="closed door covers the tub opening footprint",
        )
        ctx.expect_gap(
            lower_rack,
            lower_spray_arm,
            axis="z",
            min_gap=0.050,
            name="lower rack clears the lower spray arm",
        )
        ctx.expect_gap(
            utensil_tray,
            upper_rack,
            axis="z",
            min_gap=0.060,
            name="utensil tray stays above the upper rack",
        )
        ctx.expect_overlap(
            control_dial,
            door,
            axes="xz",
            min_overlap=0.035,
            name="dial sits within the top control bank zone",
        )
        ctx.expect_overlap(
            rinse_aid_cap,
            door,
            axes="xz",
            min_overlap=0.030,
            name="rinse aid cap is mounted on the inner door liner",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door opens downward and outward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.45
            and open_aabb[1][2] < closed_aabb[1][2] - 0.50,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for rack, joint, min_travel, min_overlap_y, label in (
        (lower_rack, lower_joint, 0.28, 0.18, "lower rack"),
        (upper_rack, upper_joint, 0.22, 0.14, "upper rack"),
        (utensil_tray, tray_joint, 0.18, 0.12, "utensil tray"),
    ):
        rest_pos = ctx.part_world_position(rack)
        with ctx.pose({joint: 0.0}):
            ctx.expect_within(
                rack,
                cabinet,
                axes="x",
                margin=0.030,
                name=f"{label} stays centered between the side walls",
            )
        joint_limits = joint.motion_limits
        if joint_limits is not None and joint_limits.upper is not None:
            with ctx.pose({joint: joint_limits.upper}):
                ctx.expect_within(
                    rack,
                    cabinet,
                    axes="x",
                    margin=0.030,
                    name=f"{label} remains aligned on its rails when extended",
                )
                ctx.expect_overlap(
                    rack,
                    cabinet,
                    axes="y",
                    min_overlap=min_overlap_y,
                    name=f"{label} retains insertion on its rails",
                )
                extended_pos = ctx.part_world_position(rack)
            ctx.check(
                f"{label} extends outward",
                rest_pos is not None
                and extended_pos is not None
                and extended_pos[1] > rest_pos[1] + min_travel,
                details=f"rest={rest_pos}, extended={extended_pos}",
            )

    upper_rest = ctx.part_world_position(upper_spray_arm)
    upper_limits = upper_joint.motion_limits
    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({upper_joint: upper_limits.upper}):
            upper_extended = ctx.part_world_position(upper_spray_arm)
        ctx.check(
            "upper spray arm travels with the upper rack",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[1] > upper_rest[1] + 0.20,
            details=f"rest={upper_rest}, extended={upper_extended}",
        )

    button_rest = ctx.part_world_position(object_model.get_part("option_button_0"))
    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.upper}):
            button_pressed = ctx.part_world_position(object_model.get_part("option_button_0"))
        ctx.check(
            "option button presses into the door",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] < button_rest[1] - 0.003,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
