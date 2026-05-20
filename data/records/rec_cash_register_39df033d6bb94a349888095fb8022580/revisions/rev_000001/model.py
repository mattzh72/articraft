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


BASE_W = 0.430
BASE_D = 0.360
BASE_H = 0.115

DRAWER_CAVITY_W = 0.392
DRAWER_CAVITY_D = 0.310
DRAWER_CAVITY_H = 0.082
DRAWER_CAVITY_CENTER_Y = -0.035
DRAWER_CAVITY_CENTER_Z = 0.055
DRAWER_JOINT_Y = -0.168
DRAWER_JOINT_Z = 0.014
DRAWER_TRAY_W = 0.382
DRAWER_TRAY_D = 0.300
DRAWER_TRAY_H = 0.076
DRAWER_FACE_W = 0.394
DRAWER_FACE_T = 0.012
DRAWER_FACE_H = 0.080
DRAWER_TRAVEL = 0.105

CONSOLE_W = 0.300
CONSOLE_FRONT_BASE_Y = -0.108
CONSOLE_REAR_BASE_Y = 0.118
CONSOLE_FRONT_TOP_Y = -0.086
CONSOLE_ROOF_START_Y = 0.030
CONSOLE_FRONT_Z = 0.148
CONSOLE_REAR_Z = 0.238
CONSOLE_SLOPE = math.atan2(
    CONSOLE_REAR_Z - CONSOLE_FRONT_Z,
    CONSOLE_ROOF_START_Y - CONSOLE_FRONT_TOP_Y,
)

KEY_ROWS = 5
KEY_COLS = 6
KEYPAD_CENTER_X = 0.070
KEYPAD_CENTER_Y = -0.020
KEYPAD_OPEN_W = 0.142
KEYPAD_OPEN_H = 0.120
KEYPAD_POCKET_D = 0.0095
KEY_CAP_W = 0.0175
KEY_CAP_H = 0.0150
KEY_CAP_T = 0.0052
KEY_STEM_W = 0.0130
KEY_STEM_H = 0.0110
KEY_STEM_T = 0.0065
KEY_PITCH_X = 0.0215
KEY_PITCH_Y = 0.0205
KEY_TRAVEL = 0.0015

OP_SCREEN_CENTER_X = -0.078
OP_SCREEN_CENTER_Y = 0.010
OP_SCREEN_W = 0.098
OP_SCREEN_H = 0.052
OP_SCREEN_RECESS_D = 0.006

DISPLAY_HINGE_Y = 0.160
DISPLAY_HINGE_Z = 0.246
DISPLAY_TILT = 0.36


def _console_surface_z(y: float) -> float:
    return CONSOLE_FRONT_Z + (y - CONSOLE_FRONT_TOP_Y) * math.tan(CONSOLE_SLOPE)


def _sloped_origin(x: float, y: float, *, z_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=(
            x,
            y - math.sin(CONSOLE_SLOPE) * z_offset,
            _console_surface_z(y) + math.cos(CONSOLE_SLOPE) * z_offset,
        ),
        rpy=(CONSOLE_SLOPE, 0.0, 0.0),
    )


def _build_console_mesh():
    console = (
        cq.Workplane("YZ")
        .polyline(
            [
                (CONSOLE_FRONT_BASE_Y, BASE_H - 0.002),
                (CONSOLE_REAR_BASE_Y, BASE_H - 0.002),
                (CONSOLE_REAR_BASE_Y, CONSOLE_REAR_Z),
                (CONSOLE_ROOF_START_Y, CONSOLE_REAR_Z),
                (CONSOLE_FRONT_TOP_Y, CONSOLE_FRONT_Z),
            ]
        )
        .close()
        .extrude(CONSOLE_W)
        .translate((-CONSOLE_W * 0.5, 0.0, 0.0))
    )

    keypad_pocket = (
        cq.Workplane("XY")
        .box(KEYPAD_OPEN_W, KEYPAD_OPEN_H, KEYPAD_POCKET_D)
        .translate((0.0, 0.0, -KEYPAD_POCKET_D * 0.5))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(CONSOLE_SLOPE))
        .translate((KEYPAD_CENTER_X, KEYPAD_CENTER_Y, _console_surface_z(KEYPAD_CENTER_Y)))
    )

    operator_screen_recess = (
        cq.Workplane("XY")
        .box(OP_SCREEN_W, OP_SCREEN_H, OP_SCREEN_RECESS_D)
        .translate((0.0, 0.0, -OP_SCREEN_RECESS_D * 0.5))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(CONSOLE_SLOPE))
        .translate((OP_SCREEN_CENTER_X, OP_SCREEN_CENTER_Y, _console_surface_z(OP_SCREEN_CENTER_Y)))
    )

    return console.cut(keypad_pocket).cut(operator_screen_recess)


def _build_drawer_mesh():
    tray = cq.Workplane("XY").box(DRAWER_TRAY_W, DRAWER_TRAY_D, DRAWER_TRAY_H).translate(
        (0.0, DRAWER_TRAY_D * 0.5, DRAWER_TRAY_H * 0.5)
    )
    front_panel = cq.Workplane("XY").box(DRAWER_FACE_W, DRAWER_FACE_T, DRAWER_FACE_H).translate(
        (0.0, -DRAWER_FACE_T * 0.5, DRAWER_FACE_H * 0.5)
    )
    finger_pull = cq.Workplane("XY").box(0.100, 0.020, 0.018).translate((0.0, -0.001, 0.052))
    return tray.union(front_panel).cut(finger_pull)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_cash_register")

    housing = model.material("housing", rgba=(0.16, 0.17, 0.19, 1.0))
    drawer_grey = model.material("drawer_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    trim = model.material("trim", rgba=(0.37, 0.39, 0.42, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.43, 0.47, 0.42))
    keypad_bed = model.material("keypad_bed", rgba=(0.08, 0.09, 0.10, 1.0))
    key_light = model.material("key_light", rgba=(0.86, 0.88, 0.90, 1.0))
    key_dark = model.material("key_dark", rgba=(0.43, 0.46, 0.49, 1.0))
    key_warn = model.material("key_warn", rgba=(0.83, 0.62, 0.18, 1.0))
    key_confirm = model.material("key_confirm", rgba=(0.21, 0.62, 0.36, 1.0))
    key_cancel = model.material("key_cancel", rgba=(0.73, 0.21, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((BASE_W, BASE_D, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=housing,
        name="bottom_plate",
    )
    body.visual(
        Box((0.014, BASE_D, BASE_H - 0.014)),
        origin=Origin(xyz=(-0.208, 0.0, 0.0645)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((0.014, BASE_D, BASE_H - 0.014)),
        origin=Origin(xyz=(0.208, 0.0, 0.0645)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((BASE_W - 0.028, 0.014, BASE_H - 0.014)),
        origin=Origin(xyz=(0.0, 0.173, 0.0645)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((BASE_W, BASE_D, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=housing,
        name="top_plate",
    )
    body.visual(
        Box((0.019, 0.014, 0.090)),
        origin=Origin(xyz=(-0.2055, -0.173, 0.045)),
        material=housing,
        name="front_post_0",
    )
    body.visual(
        Box((0.019, 0.014, 0.090)),
        origin=Origin(xyz=(0.2055, -0.173, 0.045)),
        material=housing,
        name="front_post_1",
    )
    body.visual(
        Box((0.392, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.173, 0.106)),
        material=housing,
        name="front_top_strip",
    )
    body.visual(
        mesh_from_cadquery(_build_console_mesh(), "cash_register_console"),
        material=housing,
        name="console_shell",
    )
    body.visual(
        Box((0.050, 0.028, 0.118)),
        origin=Origin(xyz=(0.0, 0.145, 0.170)),
        material=housing,
        name="neck",
    )
    body.visual(
        Box((0.126, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.160, 0.224)),
        material=housing,
        name="yoke_bridge",
    )
    body.visual(
        Box((0.016, 0.020, 0.028)),
        origin=Origin(xyz=(-0.055, 0.154, 0.236)),
        material=housing,
        name="cheek_0",
    )
    body.visual(
        Box((0.016, 0.020, 0.028)),
        origin=Origin(xyz=(0.055, 0.154, 0.236)),
        material=housing,
        name="cheek_1",
    )
    body.visual(
        Box((0.388, 0.292, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, 0.012)),
        material=trim,
        name="drawer_guide_floor",
    )
    body.visual(
        Box((KEYPAD_OPEN_W + 0.004, KEYPAD_OPEN_H + 0.004, 0.002)),
        origin=_sloped_origin(KEYPAD_CENTER_X, KEYPAD_CENTER_Y, z_offset=-0.001),
        material=keypad_bed,
        name="keypad_bed",
    )
    body.visual(
        Box((OP_SCREEN_W - 0.010, OP_SCREEN_H - 0.010, 0.003)),
        origin=_sloped_origin(OP_SCREEN_CENTER_X, OP_SCREEN_CENTER_Y, z_offset=-0.0045),
        material=glass,
        name="operator_screen",
    )

    drawer = model.part("cash_drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_mesh(), "cash_register_drawer"),
        material=drawer_grey,
        name="tray",
    )
    drawer.visual(
        Box((0.110, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.052)),
        material=trim,
        name="pull_trim",
    )
    model.articulation(
        "body_to_cash_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_JOINT_Y, DRAWER_JOINT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    display = model.part("customer_display")
    display.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    display.visual(
        Box((0.182, 0.018, 0.096)),
        origin=Origin(xyz=(0.0, 0.009, 0.052)),
        material=trim,
        name="bezel",
    )
    display.visual(
        Box((0.186, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.005, 0.103)),
        material=trim,
        name="visor",
    )
    display.visual(
        Box((0.154, 0.002, 0.064)),
        origin=Origin(xyz=(0.0, 0.018, 0.054)),
        material=glass,
        name="screen",
    )
    model.articulation(
        "body_to_customer_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, DISPLAY_HINGE_Y, DISPLAY_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-DISPLAY_TILT,
            upper=DISPLAY_TILT,
        ),
    )

    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            key = model.part(f"key_{row}_{col}")
            if row == KEY_ROWS - 1 and col == KEY_COLS - 1:
                material = key_confirm
            elif row == KEY_ROWS - 1 and col == KEY_COLS - 2:
                material = key_cancel
            elif col == KEY_COLS - 1 or row == 0:
                material = key_warn
            elif row == 1:
                material = key_dark
            else:
                material = key_light

            key.visual(
                Box((KEY_CAP_W, KEY_CAP_H, KEY_CAP_T)),
                origin=Origin(xyz=(0.0, 0.0, 0.00260)),
                material=material,
                name="cap",
            )

            x = KEYPAD_CENTER_X + (col - (KEY_COLS - 1) * 0.5) * KEY_PITCH_X
            y = KEYPAD_CENTER_Y + ((KEY_ROWS - 1) * 0.5 - row) * KEY_PITCH_Y
            model.articulation(
                f"body_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=_sloped_origin(x, y),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=2.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=KEY_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("cash_drawer")
    display = object_model.get_part("customer_display")
    drawer_joint = object_model.get_articulation("body_to_cash_drawer")
    display_joint = object_model.get_articulation("body_to_customer_display")
    sample_key = object_model.get_part("key_2_2")
    sample_key_joint = object_model.get_articulation("body_to_key_2_2")

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    key_joints = [joint for joint in object_model.articulations if joint.name.startswith("body_to_key_")]
    ctx.check(
        "every keypad button is a separate articulated part",
        len(key_parts) == KEY_ROWS * KEY_COLS and len(key_joints) == KEY_ROWS * KEY_COLS,
        details=f"parts={len(key_parts)}, joints={len(key_joints)}",
    )

    for key_part in key_parts:
        ctx.allow_overlap(
            body,
            key_part,
            elem_a="keypad_bed",
            elem_b="cap",
            reason="The register keypad is simplified as a continuous membrane panel beneath individual push-button caps.",
        )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray",
            elem_b="drawer_guide_floor",
            min_overlap=0.375,
            name="drawer stays laterally centered in the base",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="tray",
            elem_b="drawer_guide_floor",
            min_overlap=0.285,
            name="closed drawer sits fully on the guide floor",
        )
        drawer_rest = ctx.part_world_position(drawer)

    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="tray",
            elem_b="drawer_guide_floor",
            min_overlap=0.175,
            name="extended drawer retains insertion in the guide floor",
        )
        drawer_extended = ctx.part_world_position(drawer)

    ctx.check(
        "cash drawer opens toward the cashier side",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.09,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    screen_rest = ctx.part_element_world_aabb(display, elem="screen")
    with ctx.pose({display_joint: DISPLAY_TILT}):
        screen_tilted = ctx.part_element_world_aabb(display, elem="screen")
    ctx.check(
        "customer display visibly tilts about its neck hinge",
        screen_rest is not None
        and screen_tilted is not None
        and screen_tilted[0][1] < screen_rest[0][1] - 0.020,
        details=f"rest={screen_rest}, tilted={screen_tilted}",
    )

    key_rest = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: KEY_TRAVEL}):
        key_pressed = ctx.part_world_position(sample_key)
    ctx.check(
        "sample keypad button depresses into the sloped console",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.0010
        and key_pressed[1] > key_rest[1] + 0.0008,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
