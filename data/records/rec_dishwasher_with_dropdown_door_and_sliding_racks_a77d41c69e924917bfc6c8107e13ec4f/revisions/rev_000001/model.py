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


WIDTH = 0.598
DEPTH = 0.570
HEIGHT = 0.815

TUB_WIDTH = 0.540
TUB_DEPTH = 0.500
TUB_HEIGHT = 0.675
TUB_BOTTOM_Z = 0.115

OPENING_WIDTH = 0.560
OPENING_HEIGHT = 0.720
OPENING_BOTTOM_Z = 0.052

DOOR_WIDTH = 0.594
DOOR_HEIGHT = 0.720
DOOR_THICK = 0.036

RACK_SEAT_Y = 0.235
LOWER_RACK_WIDTH = 0.508
LOWER_RACK_DEPTH = 0.455
LOWER_RACK_HEIGHT = 0.152
UPPER_RACK_WIDTH = 0.500
UPPER_RACK_DEPTH = 0.438
UPPER_RACK_HEIGHT = 0.138
TRAY_WIDTH = 0.494
TRAY_DEPTH = 0.425
TRAY_HEIGHT = 0.050

LOWER_RACK_Z = 0.150
UPPER_RACK_Z = 0.395
TRAY_Z = 0.610
LOWER_ARM_Z = 0.138
UPPER_ARM_Z = 0.380

BUTTON_COUNT = 5


def _box(
    sx: float,
    sy: float,
    sz: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate((x, y, z))


def _cyl_z(
    radius: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z))


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    merged = shapes[0]
    for shape in shapes[1:]:
        merged = merged.union(shape)
    return merged


def _body_shape() -> cq.Workplane:
    tub_outer_width = WIDTH - 0.024
    tub_outer_depth = TUB_DEPTH + 0.016
    tub_wall = 0.012
    tub_outer_height = TUB_HEIGHT + (tub_wall * 2.0)
    front_frame_depth = 0.040
    guide_bridge_width = 0.014
    guide_width = 0.016

    shapes = [
        _box(0.014, DEPTH, HEIGHT, x=-(WIDTH / 2.0 - 0.007), z=HEIGHT / 2.0),
        _box(0.014, DEPTH, HEIGHT, x=WIDTH / 2.0 - 0.007, z=HEIGHT / 2.0),
        _box(WIDTH, 0.014, HEIGHT, y=-(DEPTH / 2.0 - 0.007), z=HEIGHT / 2.0),
        _box(WIDTH, DEPTH, 0.014, z=HEIGHT - 0.007),
        _box(tub_outer_width, tub_outer_depth, tub_wall, z=TUB_BOTTOM_Z - tub_wall / 2.0),
        _box(tub_outer_width, tub_outer_depth, tub_wall, z=TUB_BOTTOM_Z + TUB_HEIGHT + tub_wall / 2.0),
        _box(tub_wall, tub_outer_depth, tub_outer_height, x=-(tub_outer_width / 2.0 - tub_wall / 2.0), z=TUB_BOTTOM_Z + TUB_HEIGHT / 2.0),
        _box(tub_wall, tub_outer_depth, tub_outer_height, x=tub_outer_width / 2.0 - tub_wall / 2.0, z=TUB_BOTTOM_Z + TUB_HEIGHT / 2.0),
        _box(tub_outer_width, tub_wall, tub_outer_height, y=-(tub_outer_depth / 2.0 - tub_wall / 2.0), z=TUB_BOTTOM_Z + TUB_HEIGHT / 2.0),
        _box(0.026, front_frame_depth, OPENING_HEIGHT + 0.042, x=-(OPENING_WIDTH / 2.0 + 0.013), y=DEPTH / 2.0 - front_frame_depth / 2.0, z=OPENING_BOTTOM_Z + (OPENING_HEIGHT + 0.042) / 2.0),
        _box(0.026, front_frame_depth, OPENING_HEIGHT + 0.042, x=OPENING_WIDTH / 2.0 + 0.013, y=DEPTH / 2.0 - front_frame_depth / 2.0, z=OPENING_BOTTOM_Z + (OPENING_HEIGHT + 0.042) / 2.0),
        _box(OPENING_WIDTH + 0.052, front_frame_depth, 0.050, y=DEPTH / 2.0 - front_frame_depth / 2.0, z=HEIGHT - 0.025),
        _box(OPENING_WIDTH + 0.052, front_frame_depth, OPENING_BOTTOM_Z, y=DEPTH / 2.0 - front_frame_depth / 2.0, z=OPENING_BOTTOM_Z / 2.0),
        _box(0.042, 0.090, OPENING_BOTTOM_Z, x=-(WIDTH / 2.0 - 0.038), y=DEPTH / 2.0 - 0.045, z=OPENING_BOTTOM_Z / 2.0),
        _box(0.042, 0.090, OPENING_BOTTOM_Z, x=WIDTH / 2.0 - 0.038, y=DEPTH / 2.0 - 0.045, z=OPENING_BOTTOM_Z / 2.0),
        _cyl_z(0.068, 0.020, z=TUB_BOTTOM_Z - 0.002),
        _box(0.052, 0.036, 0.130, y=-0.220, z=TUB_BOTTOM_Z + 0.065),
        _box(0.032, 0.026, 0.055, y=-0.220, z=TUB_BOTTOM_Z + 0.157),
        _cyl_z(0.020, 0.018, z=LOWER_ARM_Z - 0.018),
        _cyl_z(0.018, 0.018, z=UPPER_ARM_Z - 0.018),
    ]

    for z_pos, rail_length in (
        (LOWER_RACK_Z + 0.020, 0.470),
        (UPPER_RACK_Z + 0.018, 0.450),
        (TRAY_Z + 0.014, 0.432),
    ):
        for sign in (-1.0, 1.0):
            guide_x = sign * 0.261
            bridge_x = sign * 0.271
            shapes.append(_box(guide_width, rail_length, 0.010, x=guide_x, y=0.010, z=z_pos))
            shapes.append(_box(guide_bridge_width, rail_length, 0.010, x=bridge_x, y=0.010, z=z_pos))

    return _union_all(shapes)


def _door_shape() -> cq.Workplane:
    shell = _box(DOOR_WIDTH, DOOR_THICK, DOOR_HEIGHT, y=DOOR_THICK / 2.0, z=DOOR_HEIGHT / 2.0)
    inner_recess = _box(
        DOOR_WIDTH - 0.072,
        DOOR_THICK - 0.014,
        DOOR_HEIGHT - 0.098,
        y=(DOOR_THICK - 0.014) / 2.0,
        z=(DOOR_HEIGHT / 2.0) + 0.004,
    )
    detergent_pocket = _box(
        0.176,
        0.016,
        0.098,
        x=0.088,
        y=0.008,
        z=0.432,
    )
    control_strip_recess = _box(
        0.228,
        0.014,
        0.004,
        y=DOOR_THICK * 0.52,
        z=DOOR_HEIGHT - 0.002,
    )
    handle_shadow = _box(
        0.300,
        0.010,
        0.022,
        y=DOOR_THICK * 0.58,
        z=DOOR_HEIGHT - 0.050,
    )

    return shell.cut(inner_recess).cut(detergent_pocket).cut(control_strip_recess).cut(handle_shadow)


def _rack_shape(
    *,
    width: float,
    depth: float,
    height: float,
    rod: float,
    front_handle_width: float,
    tine_rows: tuple[float, ...],
    tine_height: float,
) -> cq.Workplane:
    shapes = [
        _box(width, rod, rod, y=-rod / 2.0, z=rod / 2.0),
        _box(width, rod, rod, y=-depth + rod / 2.0, z=rod / 2.0),
        _box(rod, depth - rod, rod, x=width / 2.0 - rod / 2.0, y=-depth / 2.0, z=rod / 2.0),
        _box(rod, depth - rod, rod, x=-width / 2.0 + rod / 2.0, y=-depth / 2.0, z=rod / 2.0),
        _box(width, rod, rod, y=-rod / 2.0, z=height - rod / 2.0),
        _box(width, rod, rod, y=-depth + rod / 2.0, z=height - rod / 2.0),
        _box(rod, depth - rod, rod, x=width / 2.0 - rod / 2.0, y=-depth / 2.0, z=height - rod / 2.0),
        _box(rod, depth - rod, rod, x=-width / 2.0 + rod / 2.0, y=-depth / 2.0, z=height - rod / 2.0),
    ]

    for x_pos in (
        -width / 2.0 + rod / 2.0,
        width / 2.0 - rod / 2.0,
    ):
        for y_pos in (-rod / 2.0, -depth + rod / 2.0):
            shapes.append(_box(rod, rod, height, x=x_pos, y=y_pos, z=height / 2.0))

    for y_pos in (
        -depth * 0.18,
        -depth * 0.34,
        -depth * 0.50,
        -depth * 0.66,
        -depth * 0.82,
    ):
        shapes.append(_box(width - (rod * 2.0), rod * 0.78, rod, y=y_pos, z=rod / 2.0))

    for x_pos in (
        -width * 0.32,
        -width * 0.16,
        0.0,
        width * 0.16,
        width * 0.32,
    ):
        shapes.append(_box(rod, depth - 0.060, rod * 0.78, x=x_pos, y=-(depth - 0.060) / 2.0 - 0.018, z=rod / 2.0))

    shapes.extend(
        [
            _box(front_handle_width, rod * 1.25, rod * 1.25, y=0.012, z=height + 0.020),
            _box(rod * 1.10, rod, 0.034, x=-front_handle_width / 2.0 + rod * 0.60, y=0.006, z=height + 0.007),
            _box(rod * 1.10, rod, 0.034, x=front_handle_width / 2.0 - rod * 0.60, y=0.006, z=height + 0.007),
        ]
    )

    for row_y in tine_rows:
        for x_pos in (
            -width * 0.34,
            -width * 0.22,
            -width * 0.10,
            width * 0.02,
            width * 0.14,
            width * 0.26,
        ):
            shapes.append(
                _box(
                    rod * 0.75,
                    rod * 0.75,
                    tine_height,
                    x=x_pos,
                    y=row_y,
                    z=(rod * 0.40) + (tine_height / 2.0),
                )
            )

    return _union_all(shapes)


def _cutlery_tray_shape() -> cq.Workplane:
    rod = 0.005
    height = TRAY_HEIGHT
    width = TRAY_WIDTH
    depth = TRAY_DEPTH
    shapes = [
        _box(width, rod, rod, y=-rod / 2.0, z=rod / 2.0),
        _box(width, rod, rod, y=-depth + rod / 2.0, z=rod / 2.0),
        _box(rod, depth - rod, rod, x=width / 2.0 - rod / 2.0, y=-depth / 2.0, z=rod / 2.0),
        _box(rod, depth - rod, rod, x=-width / 2.0 + rod / 2.0, y=-depth / 2.0, z=rod / 2.0),
        _box(width, rod, rod, y=-rod / 2.0, z=height - rod / 2.0),
        _box(width, rod, rod, y=-depth + rod / 2.0, z=height - rod / 2.0),
        _box(rod, depth - rod, rod, x=width / 2.0 - rod / 2.0, y=-depth / 2.0, z=height - rod / 2.0),
        _box(rod, depth - rod, rod, x=-width / 2.0 + rod / 2.0, y=-depth / 2.0, z=height - rod / 2.0),
        _box(width * 0.30, rod * 1.20, rod * 1.20, y=0.010, z=height + 0.012),
    ]

    for x_pos in (-width / 2.0 + rod / 2.0, width / 2.0 - rod / 2.0):
        for y_pos in (-rod / 2.0, -depth + rod / 2.0):
            shapes.append(_box(rod, rod, height, x=x_pos, y=y_pos, z=height / 2.0))

    for x_pos in (-width * 0.18, 0.0, width * 0.18):
        shapes.append(_box(rod, depth - 0.028, height - rod, x=x_pos, y=-(depth - 0.028) / 2.0 - 0.010, z=(height - rod) / 2.0 + rod / 2.0))

    for y_pos in (-depth * 0.22, -depth * 0.45, -depth * 0.68):
        shapes.append(_box(width - 0.020, rod * 0.80, rod, y=y_pos, z=rod / 2.0))

    return _union_all(shapes)


def _spray_arm_shape(*, span: float, paddle_width: float) -> cq.Workplane:
    shapes = [
        _cyl_z(0.028, 0.012, z=-0.006),
        _box(span, 0.022, 0.010, z=0.0),
        _box(0.094, paddle_width, 0.010, x=span * 0.28, z=0.0),
        _box(0.094, paddle_width, 0.010, x=-span * 0.28, z=0.0),
        _box(0.020, 0.010, 0.010, x=span * 0.44, y=0.010, z=0.0),
        _box(0.020, 0.010, 0.010, x=-span * 0.44, y=-0.010, z=0.0),
    ]
    return _union_all(shapes)


def _stemware_shelf_shape() -> cq.Workplane:
    thickness = 0.006
    span_y = 0.262
    drop = 0.108
    shapes = [
        _box(0.010, span_y, thickness, x=0.0, y=0.0, z=-thickness / 2.0),
        _box(thickness, span_y, thickness, x=-0.098, y=0.0, z=-thickness / 2.0),
        _box(0.010, thickness, 0.018, x=0.0, y=span_y / 2.0 - thickness / 2.0, z=-0.009),
        _box(0.010, thickness, 0.018, x=0.0, y=-span_y / 2.0 + thickness / 2.0, z=-0.009),
        _box(thickness, thickness, drop, x=-0.003, y=span_y / 2.0 - thickness / 2.0, z=-drop / 2.0),
        _box(thickness, thickness, drop, x=-0.003, y=-span_y / 2.0 + thickness / 2.0, z=-drop / 2.0),
        _box(thickness, thickness, drop, x=-0.098, y=span_y / 2.0 - thickness / 2.0, z=-drop / 2.0),
        _box(thickness, thickness, drop, x=-0.098, y=-span_y / 2.0 + thickness / 2.0, z=-drop / 2.0),
        _box(0.104, thickness, thickness, x=-0.052, y=span_y / 2.0 - thickness / 2.0, z=-drop + thickness / 2.0),
        _box(0.104, thickness, thickness, x=-0.052, y=-span_y / 2.0 + thickness / 2.0, z=-drop + thickness / 2.0),
        _box(0.104, thickness, thickness, x=-0.052, y=0.0, z=-drop + thickness / 2.0),
    ]
    return _union_all(shapes)


def _detergent_cover_shape() -> cq.Workplane:
    cover_width = 0.164
    cover_height = 0.084
    cover_thick = 0.004
    frame_depth = 0.010
    shapes = [
        _box(cover_width, cover_thick, cover_height, y=cover_thick / 2.0, z=-cover_height / 2.0),
        _box(cover_width, frame_depth, cover_thick, y=frame_depth / 2.0, z=-cover_thick / 2.0),
    ]
    return _union_all(shapes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_dishwasher")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    tub_steel = model.material("tub_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    for name, geometry, origin in (
        ("left_outer_panel", Box((0.014, DEPTH, HEIGHT)), Origin(xyz=(-(WIDTH / 2.0 - 0.007), 0.0, HEIGHT / 2.0))),
        ("right_outer_panel", Box((0.014, DEPTH, HEIGHT)), Origin(xyz=(WIDTH / 2.0 - 0.007, 0.0, HEIGHT / 2.0))),
        ("rear_panel", Box((WIDTH, 0.014, HEIGHT)), Origin(xyz=(0.0, -(DEPTH / 2.0 - 0.007), HEIGHT / 2.0))),
        ("top_panel", Box((WIDTH, DEPTH, 0.014)), Origin(xyz=(0.0, 0.0, HEIGHT - 0.007))),
        ("tub_floor", Box((WIDTH - 0.024, TUB_DEPTH + 0.016, 0.012)), Origin(xyz=(0.0, 0.0, TUB_BOTTOM_Z - 0.006))),
        ("tub_ceiling", Box((WIDTH - 0.024, TUB_DEPTH + 0.016, 0.012)), Origin(xyz=(0.0, 0.0, TUB_BOTTOM_Z + TUB_HEIGHT + 0.006))),
        ("tub_left_wall", Box((0.012, TUB_DEPTH + 0.016, TUB_HEIGHT + 0.024)), Origin(xyz=(-(WIDTH - 0.024) / 2.0 + 0.006, 0.0, TUB_BOTTOM_Z + TUB_HEIGHT / 2.0))),
        ("tub_right_wall", Box((0.012, TUB_DEPTH + 0.016, TUB_HEIGHT + 0.024)), Origin(xyz=(((WIDTH - 0.024) / 2.0) - 0.006, 0.0, TUB_BOTTOM_Z + TUB_HEIGHT / 2.0))),
        ("tub_back_wall", Box((WIDTH - 0.024, 0.012, TUB_HEIGHT + 0.024)), Origin(xyz=(0.0, -((TUB_DEPTH + 0.016) / 2.0 - 0.006), TUB_BOTTOM_Z + TUB_HEIGHT / 2.0))),
        ("left_front_stile", Box((0.026, 0.040, OPENING_HEIGHT + 0.042)), Origin(xyz=(-(OPENING_WIDTH / 2.0 + 0.013), DEPTH / 2.0 - 0.020, OPENING_BOTTOM_Z + (OPENING_HEIGHT + 0.042) / 2.0))),
        ("right_front_stile", Box((0.026, 0.040, OPENING_HEIGHT + 0.042)), Origin(xyz=(OPENING_WIDTH / 2.0 + 0.013, DEPTH / 2.0 - 0.020, OPENING_BOTTOM_Z + (OPENING_HEIGHT + 0.042) / 2.0))),
        ("top_front_rail", Box((OPENING_WIDTH + 0.052, 0.040, 0.050)), Origin(xyz=(0.0, DEPTH / 2.0 - 0.020, HEIGHT - 0.025))),
        ("bottom_front_rail", Box((OPENING_WIDTH + 0.052, 0.040, OPENING_BOTTOM_Z)), Origin(xyz=(0.0, DEPTH / 2.0 - 0.020, OPENING_BOTTOM_Z / 2.0))),
        ("left_toe_leg", Box((0.042, 0.090, OPENING_BOTTOM_Z)), Origin(xyz=(-(WIDTH / 2.0 - 0.038), DEPTH / 2.0 - 0.045, OPENING_BOTTOM_Z / 2.0))),
        ("right_toe_leg", Box((0.042, 0.090, OPENING_BOTTOM_Z)), Origin(xyz=(WIDTH / 2.0 - 0.038, DEPTH / 2.0 - 0.045, OPENING_BOTTOM_Z / 2.0))),
    ):
        body.visual(geometry, origin=origin, material=tub_steel, name=name)

    body.visual(
        Cylinder(radius=0.068, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, TUB_BOTTOM_Z + 0.010)),
        material=tub_steel,
        name="sump",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, -0.010, LOWER_ARM_Z - 0.012)),
        material=tub_steel,
        name="lower_spindle",
    )
    for name, x_pos, y_span, z_pos in (
        ("lower_rail_left", -0.261, 0.470, LOWER_RACK_Z + 0.020),
        ("lower_rail_right", 0.261, 0.470, LOWER_RACK_Z + 0.020),
        ("upper_rail_left", -0.261, 0.450, UPPER_RACK_Z + 0.018),
        ("upper_rail_right", 0.261, 0.450, UPPER_RACK_Z + 0.018),
        ("tray_rail_left", -0.261, 0.432, TRAY_Z + 0.014),
        ("tray_rail_right", 0.261, 0.432, TRAY_Z + 0.014),
    ):
        body.visual(
            Box((0.016, y_span, 0.010)),
            origin=Origin(xyz=(x_pos, 0.010, z_pos)),
            material=trim_gray,
            name=name,
        )
        body.visual(
            Box((0.014, y_span, 0.010)),
            origin=Origin(xyz=(x_pos * 1.038, 0.010, z_pos)),
            material=trim_gray,
            name=f"{name}_bridge",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "dishwasher_door"),
        material=cabinet_white,
        name="door_shell",
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(
            _rack_shape(
                width=LOWER_RACK_WIDTH,
                depth=LOWER_RACK_DEPTH,
                height=LOWER_RACK_HEIGHT,
                rod=0.006,
                front_handle_width=0.250,
                tine_rows=(-LOWER_RACK_DEPTH * 0.56, -LOWER_RACK_DEPTH * 0.76),
                tine_height=0.082,
            ),
            "lower_rack_mesh",
        ),
        material=rack_gray,
        name="basket",
    )
    lower_rack.visual(
        Box((0.016, 0.320, 0.014)),
        origin=Origin(xyz=(LOWER_RACK_WIDTH / 2.0 - 0.001, -0.170, 0.020)),
        material=trim_gray,
        name="runner_right",
    )
    lower_rack.visual(
        Box((0.016, 0.320, 0.014)),
        origin=Origin(xyz=(-(LOWER_RACK_WIDTH / 2.0) + 0.001, -0.170, 0.020)),
        material=trim_gray,
        name="runner_left",
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(
            _rack_shape(
                width=UPPER_RACK_WIDTH,
                depth=UPPER_RACK_DEPTH,
                height=UPPER_RACK_HEIGHT,
                rod=0.0055,
                front_handle_width=0.220,
                tine_rows=(-UPPER_RACK_DEPTH * 0.48, -UPPER_RACK_DEPTH * 0.68),
                tine_height=0.060,
            ),
            "upper_rack_mesh",
        ),
        material=rack_gray,
        name="basket",
    )
    upper_rack.visual(
        Box((0.012, 0.024, 0.024)),
        origin=Origin(xyz=(UPPER_RACK_WIDTH / 2.0 - 0.006, -0.082, UPPER_RACK_HEIGHT - 0.008)),
        material=trim_gray,
        name="shelf_pivot_front",
    )
    upper_rack.visual(
        Box((0.012, 0.024, 0.024)),
        origin=Origin(xyz=(UPPER_RACK_WIDTH / 2.0 - 0.006, -0.338, UPPER_RACK_HEIGHT - 0.008)),
        material=trim_gray,
        name="shelf_pivot_rear",
    )
    upper_rack.visual(
        Box((0.016, 0.296, 0.012)),
        origin=Origin(xyz=(UPPER_RACK_WIDTH / 2.0 - 0.001, -0.182, 0.020)),
        material=trim_gray,
        name="runner_right",
    )
    upper_rack.visual(
        Box((0.016, 0.296, 0.012)),
        origin=Origin(xyz=(-(UPPER_RACK_WIDTH / 2.0) + 0.001, -0.182, 0.020)),
        material=trim_gray,
        name="runner_left",
    )
    upper_rack.visual(
        Box((0.036, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, -0.210, -0.010)),
        material=trim_gray,
        name="spray_mount",
    )

    cutlery_tray = model.part("cutlery_tray")
    cutlery_tray.visual(
        mesh_from_cadquery(_cutlery_tray_shape(), "cutlery_tray_mesh"),
        material=rack_gray,
        name="tray",
    )
    cutlery_tray.visual(
        Box((0.022, 0.290, 0.010)),
        origin=Origin(xyz=(TRAY_WIDTH / 2.0 - 0.005, -0.175, 0.018)),
        material=trim_gray,
        name="runner_right",
    )
    cutlery_tray.visual(
        Box((0.022, 0.290, 0.010)),
        origin=Origin(xyz=(-(TRAY_WIDTH / 2.0) + 0.005, -0.175, 0.018)),
        material=trim_gray,
        name="runner_left",
    )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        mesh_from_cadquery(_spray_arm_shape(span=0.330, paddle_width=0.040), "lower_spray_arm_mesh"),
        material=arm_gray,
        name="spray_arm",
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        mesh_from_cadquery(_spray_arm_shape(span=0.290, paddle_width=0.034), "upper_spray_arm_mesh"),
        material=arm_gray,
        name="spray_arm",
    )

    stemware_shelf = model.part("stemware_shelf")
    stemware_shelf.visual(
        mesh_from_cadquery(_stemware_shelf_shape(), "stemware_shelf_mesh"),
        material=rack_gray,
        name="shelf",
    )

    detergent_cover = model.part("detergent_cover")
    detergent_cover.visual(
        mesh_from_cadquery(_detergent_cover_shape(), "detergent_cover_mesh"),
        material=trim_gray,
        name="cover",
    )
    detergent_cover.visual(
        Box((0.014, 0.014, 0.012)),
        origin=Origin(xyz=(-0.060, 0.007, -0.006)),
        material=trim_gray,
        name="hinge_lug_front",
    )
    detergent_cover.visual(
        Box((0.014, 0.014, 0.012)),
        origin=Origin(xyz=(0.060, 0.007, -0.006)),
        material=trim_gray,
        name="hinge_lug_rear",
    )

    button_parts = []
    button_x_positions = (-0.096, -0.054, -0.012, 0.030, 0.072)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.010, 0.0045)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=control_black,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=control_black,
            name="button_stem",
        )
        button_parts.append((button, x_pos))

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, DEPTH / 2.0, OPENING_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.58),
    )
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, RACK_SEAT_Y, LOWER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.45, lower=0.0, upper=0.300),
    )
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, RACK_SEAT_Y - 0.010, UPPER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.40, lower=0.0, upper=0.282),
    )
    model.articulation(
        "cutlery_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cutlery_tray,
        origin=Origin(xyz=(0.0, RACK_SEAT_Y - 0.020, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.245),
    )
    model.articulation(
        "lower_spray_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, -0.010, LOWER_ARM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "upper_spray_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, -0.210, -0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "stemware_shelf_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=stemware_shelf,
        origin=Origin(xyz=(UPPER_RACK_WIDTH / 2.0 - 0.017, -0.210, UPPER_RACK_HEIGHT - 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.2, lower=0.0, upper=1.48),
    )
    model.articulation(
        "detergent_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_cover,
        origin=Origin(xyz=(0.088, 0.008, 0.472)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.55),
    )

    for index, (button, x_pos) in enumerate(button_parts):
        model.articulation(
            f"door_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x_pos, DOOR_THICK * 0.50, DOOR_HEIGHT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cutlery_tray = object_model.get_part("cutlery_tray")
    stemware_shelf = object_model.get_part("stemware_shelf")
    detergent_cover = object_model.get_part("detergent_cover")
    button_0 = object_model.get_part("button_0")

    door_hinge = object_model.get_articulation("body_to_door")
    lower_slide = object_model.get_articulation("lower_rack_slide")
    upper_slide = object_model.get_articulation("upper_rack_slide")
    tray_slide = object_model.get_articulation("cutlery_tray_slide")
    shelf_hinge = object_model.get_articulation("stemware_shelf_hinge")
    cover_hinge = object_model.get_articulation("detergent_cover_hinge")
    button_press = object_model.get_articulation("door_to_button_0")

    ctx.allow_overlap(
        body,
        lower_rack,
        elem_a="lower_rail_left",
        elem_b="runner_left",
        reason="The lower rack runner is intentionally simplified as a nested slide shoe on the fixed guide rail.",
    )
    ctx.allow_overlap(
        body,
        lower_rack,
        elem_a="lower_rail_right",
        elem_b="runner_right",
        reason="The lower rack runner is intentionally simplified as a nested slide shoe on the fixed guide rail.",
    )
    ctx.allow_overlap(
        door,
        detergent_cover,
        elem_a="door_shell",
        elem_b="hinge_lug_front",
        reason="The detergent cover hinge lug is intentionally simplified as a compact knuckle nested into the inner door liner.",
    )
    ctx.allow_overlap(
        door,
        detergent_cover,
        elem_a="door_shell",
        elem_b="hinge_lug_rear",
        reason="The detergent cover hinge lug is intentionally simplified as a compact knuckle nested into the inner door liner.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.0,
            max_gap=0.003,
            name="closed door sits just ahead of the body",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.50,
            name="closed door covers the front opening",
        )

    closed_door_pos = elem_center(door, "door_shell")
    with ctx.pose({door_hinge: 1.58}):
        open_door_pos = elem_center(door, "door_shell")
    ctx.check(
        "door opens downward and outward",
        closed_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[1] > closed_door_pos[1] + 0.20
        and open_door_pos[2] < closed_door_pos[2] - 0.20,
        details=f"closed={closed_door_pos}, open={open_door_pos}",
    )

    with ctx.pose({door_hinge: 1.58, lower_slide: 0.0, upper_slide: 0.0, tray_slide: 0.0}):
        ctx.expect_within(lower_rack, body, axes="xz", margin=0.012, name="lower rack stays centered in the tub")
        ctx.expect_within(upper_rack, body, axes="xz", margin=0.012, name="upper rack stays centered in the tub")
        ctx.expect_within(cutlery_tray, body, axes="xz", margin=0.014, name="cutlery tray stays centered in the tub")

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({door_hinge: 1.58, lower_slide: 0.300}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            min_overlap=0.18,
            name="lower rack retains insertion at full extension",
        )
    ctx.check(
        "lower rack slides forward",
        lower_rest is not None and lower_extended is not None and lower_extended[1] > lower_rest[1] + 0.20,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({door_hinge: 1.58, upper_slide: 0.282}):
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            min_overlap=0.16,
            name="upper rack retains insertion at full extension",
        )
    ctx.check(
        "upper rack slides forward",
        upper_rest is not None and upper_extended is not None and upper_extended[1] > upper_rest[1] + 0.18,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    tray_rest = ctx.part_world_position(cutlery_tray)
    with ctx.pose({door_hinge: 1.58, tray_slide: 0.245}):
        tray_extended = ctx.part_world_position(cutlery_tray)
        ctx.expect_overlap(
            cutlery_tray,
            body,
            axes="y",
            min_overlap=0.12,
            name="cutlery tray retains insertion at full extension",
        )
    ctx.check(
        "cutlery tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[1] > tray_rest[1] + 0.15,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    shelf_rest = elem_center(stemware_shelf, "shelf")
    with ctx.pose({shelf_hinge: 1.48}):
        shelf_open = elem_center(stemware_shelf, "shelf")
    ctx.check(
        "stemware shelf folds inward from the side pivots",
        shelf_rest is not None
        and shelf_open is not None
        and shelf_open[0] < shelf_rest[0] - 0.006
        and shelf_open[2] > shelf_rest[2] + 0.08,
        details=f"rest={shelf_rest}, open={shelf_open}",
    )

    with ctx.pose({door_hinge: 0.0, cover_hinge: 0.0}):
        cover_rest = elem_center(detergent_cover, "cover")
    with ctx.pose({door_hinge: 0.0, cover_hinge: 1.55}):
        cover_open = elem_center(detergent_cover, "cover")
    ctx.check(
        "detergent cover flips into the tub",
        cover_rest is not None and cover_open is not None and cover_open[1] < cover_rest[1] - 0.03,
        details=f"rest={cover_rest}, open={cover_open}",
    )

    with ctx.pose({door_hinge: 0.0, button_press: 0.0}):
        button_rest = ctx.part_world_position(button_0)
    with ctx.pose({door_hinge: 0.0, button_press: 0.0025}):
        button_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "top-edge control button depresses downward",
        button_rest is not None and button_pressed is not None and button_pressed[2] < button_rest[2] - 0.0015,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
