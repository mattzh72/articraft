from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.034
BODY_T = 0.012
BODY_H = 0.065
BODY_CORNER = 0.005
BODY_FACE_SOFTEN = 0.0011

PANEL_W = 0.026
PANEL_H = 0.041
PANEL_DEPTH = 0.0006
PANEL_Z = -0.002
PANEL_CORNER = 0.0032

BUTTON_W = 0.020
BUTTON_H = 0.008
BUTTON_CORNER = 0.0026
BUTTON_TOP_SOFTEN = 0.0006
BUTTON_CAP_DEPTH = 0.0024
BUTTON_CAVITY_DEPTH = 0.0025
BUTTON_CLEARANCE = 0.0006
BUTTON_TRAVEL = 0.0010
BUTTON_CENTERS_Z = (0.013, 0.0, -0.013)

SWITCH_Z = 0.019
SWITCH_HEIGHT = 0.0056
SWITCH_LENGTH = 0.008
SWITCH_SLOT_DEPTH = 0.0018
SWITCH_TOTAL_DEPTH = 0.0027
SWITCH_TRAVEL = 0.005
SWITCH_CLEARANCE = 0.0004
SWITCH_SLOT_RADIUS = 0.0010

KEY_RING_RADIUS = 0.0031
KEY_RING_Z = 0.022

POSITION_TOL = 5e-5


def _rounded_block(width: float, depth: float, height: float, corner_radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(width, depth, height)
    if corner_radius > 0.0:
        shape = shape.edges("|Y").fillet(corner_radius)
    return shape


def _build_body_shape() -> cq.Workplane:
    body = _rounded_block(BODY_W, BODY_T, BODY_H, BODY_CORNER)
    body = body.faces(">Y").edges().fillet(BODY_FACE_SOFTEN)
    body = body.faces("<Y").edges().fillet(BODY_FACE_SOFTEN * 0.8)

    panel = _rounded_block(PANEL_W, PANEL_DEPTH, PANEL_H, PANEL_CORNER).translate(
        (0.0, BODY_T * 0.5 - PANEL_DEPTH * 0.5, PANEL_Z)
    )
    body = body.cut(panel)

    cavity_y = BODY_T * 0.5 - BUTTON_CAVITY_DEPTH * 0.5
    for center_z in BUTTON_CENTERS_Z:
        cavity = _rounded_block(
            BUTTON_W + 2.0 * BUTTON_CLEARANCE,
            BUTTON_CAVITY_DEPTH,
            BUTTON_H + 2.0 * BUTTON_CLEARANCE,
            BUTTON_CORNER + BUTTON_CLEARANCE * 0.8,
        ).translate((0.0, cavity_y, center_z))
        body = body.cut(cavity)

    switch_slot = (
        cq.Workplane("XY")
        .box(
            SWITCH_SLOT_DEPTH,
            SWITCH_HEIGHT + 2.0 * SWITCH_CLEARANCE,
            SWITCH_LENGTH + SWITCH_TRAVEL + 2.0 * SWITCH_CLEARANCE,
        )
        .edges("|X")
        .fillet(SWITCH_SLOT_RADIUS)
        .translate((BODY_W * 0.5 - SWITCH_SLOT_DEPTH * 0.5, 0.0, SWITCH_Z))
    )
    body = body.cut(switch_slot)

    key_ring_hole = (
        cq.Workplane("XZ")
        .center(0.0, KEY_RING_Z)
        .circle(KEY_RING_RADIUS)
        .extrude(BODY_T, both=True)
    )
    body = body.cut(key_ring_hole)

    return body


def _build_button_shape() -> cq.Workplane:
    button = _rounded_block(BUTTON_W, BUTTON_CAP_DEPTH, BUTTON_H, BUTTON_CORNER)
    button = button.faces(">Y").edges().fillet(BUTTON_TOP_SOFTEN)
    rail_depth = BUTTON_CAP_DEPTH * 0.72
    rail_height = BUTTON_H * 0.42
    rail = cq.Workplane("XY").box(BUTTON_CLEARANCE, rail_depth, rail_height)
    left_rail = rail.translate(
        (
            -(BUTTON_W * 0.5 + BUTTON_CLEARANCE * 0.5),
            0.0,
            0.0,
        )
    )
    right_rail = rail.translate(
        (
            BUTTON_W * 0.5 + BUTTON_CLEARANCE * 0.5,
            0.0,
            0.0,
        )
    )
    button = button.union(left_rail).union(right_rail)
    return button


def _build_lock_switch_shape() -> cq.Workplane:
    slider = (
        cq.Workplane("XY")
        .box(SWITCH_TOTAL_DEPTH, SWITCH_HEIGHT, SWITCH_LENGTH)
        .edges("|X")
        .fillet(SWITCH_HEIGHT * 0.24)
    )
    slider = slider.faces(">X").edges("%Line").fillet(0.00045)
    return slider


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="key_fob_remote")

    body_plastic = model.material("body_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    button_rubber = model.material("button_rubber", rgba=(0.22, 0.22, 0.23, 1.0))
    switch_accent = model.material("switch_accent", rgba=(0.88, 0.34, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "remote_body"),
        material=body_plastic,
        name="shell",
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "remote_button")
    button_visual_y = BUTTON_TRAVEL + (BUTTON_CAVITY_DEPTH - BUTTON_CAP_DEPTH) + BUTTON_CAP_DEPTH * 0.5
    button_floor_y = BODY_T * 0.5 - BUTTON_CAVITY_DEPTH

    for index, center_z in enumerate(BUTTON_CENTERS_Z):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, button_visual_y, 0.0)),
            material=button_rubber,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.0, button_floor_y, center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    lock_switch = model.part("lock_switch")
    lock_switch.visual(
        mesh_from_cadquery(_build_lock_switch_shape(), "remote_lock_switch"),
        origin=Origin(xyz=(SWITCH_TOTAL_DEPTH * 0.5, 0.0, 0.0)),
        material=switch_accent,
        name="thumb",
    )
    model.articulation(
        "body_to_lock_switch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lock_switch,
        origin=Origin(
            xyz=(
                BODY_W * 0.5 - SWITCH_SLOT_DEPTH,
                0.0,
                SWITCH_Z - SWITCH_TRAVEL * 0.5,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=0.05,
            lower=0.0,
            upper=SWITCH_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]
    lock_switch = object_model.get_part("lock_switch")
    lock_joint = object_model.get_articulation("body_to_lock_switch")

    for index, button in enumerate(buttons):
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            min_overlap=0.007,
            name=f"button_{index} stays centered on the front control band",
        )

    ctx.expect_overlap(
        lock_switch,
        body,
        axes="yz",
        min_overlap=0.004,
        name="lock switch stays aligned to the side slot footprint",
    )

    rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, joint in enumerate(button_joints):
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_positions = [ctx.part_world_position(button) for button in buttons]

        moved_button = (
            rest_positions[index] is not None
            and pressed_positions[index] is not None
            and pressed_positions[index][1] < rest_positions[index][1] - 0.0008
            and abs(pressed_positions[index][0] - rest_positions[index][0]) < POSITION_TOL
            and abs(pressed_positions[index][2] - rest_positions[index][2]) < POSITION_TOL
        )

        other_buttons_still = True
        for other_index, rest in enumerate(rest_positions):
            if other_index == index:
                continue
            posed = pressed_positions[other_index]
            if rest is None or posed is None:
                other_buttons_still = False
                break
            if (
                abs(posed[0] - rest[0]) > POSITION_TOL
                or abs(posed[1] - rest[1]) > POSITION_TOL
                or abs(posed[2] - rest[2]) > POSITION_TOL
            ):
                other_buttons_still = False
                break

        ctx.check(
            f"button_{index} depresses independently",
            moved_button and other_buttons_still,
            details=f"rest={rest_positions}, pressed={pressed_positions}",
        )

    switch_rest = ctx.part_world_position(lock_switch)
    with ctx.pose({lock_joint: SWITCH_TRAVEL}):
        switch_extended = ctx.part_world_position(lock_switch)
        ctx.expect_overlap(
            lock_switch,
            body,
            axes="yz",
            min_overlap=0.004,
            name="lock switch remains captured in the side slot at full travel",
        )

    ctx.check(
        "lock switch slides upward along the side slot",
        switch_rest is not None
        and switch_extended is not None
        and switch_extended[2] > switch_rest[2] + 0.0045
        and abs(switch_extended[0] - switch_rest[0]) < POSITION_TOL
        and abs(switch_extended[1] - switch_rest[1]) < POSITION_TOL,
        details=f"rest={switch_rest}, extended={switch_extended}",
    )

    return ctx.report()


object_model = build_object_model()
