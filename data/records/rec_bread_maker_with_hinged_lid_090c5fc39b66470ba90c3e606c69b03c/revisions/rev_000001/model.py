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


BODY_W = 0.42
BODY_D = 0.30
BODY_H = 0.255

TRAY_W = 0.30
TRAY_D = 0.09
TRAY_DEPTH = 0.012
TRAY_Y = -0.083

OPENING_W = 0.322
OPENING_D = 0.176
OPENING_DEPTH = 0.060
OPENING_Y = 0.058

PAN_W = 0.294
PAN_D = 0.166
PAN_H = 0.145
PAN_WALL = 0.0045
PAN_BOTTOM_Z = 0.060

LID_W = 0.350
LID_D = 0.205
LID_H = 0.050
HINGE_Y = 0.149
HINGE_Z = BODY_H

CONTROL_SURFACE_Z = BODY_H - TRAY_DEPTH + 0.002


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.030)

    tray_cutter = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_D, TRAY_DEPTH + 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, TRAY_Y, BODY_H - TRAY_DEPTH))
    )
    opening_cutter = (
        cq.Workplane("XY")
        .box(OPENING_W, OPENING_D, OPENING_DEPTH + 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, OPENING_Y, BODY_H - OPENING_DEPTH))
    )
    pan_well_cutter = (
        cq.Workplane("XY")
        .box(PAN_W + 0.018, PAN_D + 0.018, PAN_H + 0.030, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, OPENING_Y, PAN_BOTTOM_Z - 0.006))
    )

    return body.cut(tray_cutter).cut(opening_cutter).cut(pan_well_cutter)


def _pan_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(PAN_W, PAN_D, PAN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
    )
    inner = (
        cq.Workplane("XY")
        .box(PAN_W - 2.0 * PAN_WALL, PAN_D - 2.0 * PAN_WALL, PAN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, PAN_WALL))
    )
    return outer.cut(inner)


def _lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, -LID_D / 2.0, 0.0))
    )
    window_cutter = cq.Workplane("XY").box(
        0.188,
        0.098,
        0.008,
        centered=(True, True, False),
    ).translate((0.0, -0.110, LID_H - 0.008))
    return lid.cut(window_cutter)


def _dial_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.026,
            body_style="skirted",
            top_diameter=0.038,
            skirt=KnobSkirt(0.060, 0.006, flare=0.08),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "selector_dial",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_bread_maker")

    body_finish = model.material("body_finish", rgba=(0.92, 0.92, 0.88, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.17, 0.17, 0.18, 1.0))
    pan_finish = model.material("pan_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    paddle_finish = model.material("paddle_finish", rgba=(0.62, 0.63, 0.65, 1.0))
    button_finish = model.material("button_finish", rgba=(0.80, 0.81, 0.83, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.34, 0.39, 0.44, 0.35))
    dial_finish = model.material("dial_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "bread_maker_body"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((TRAY_W - 0.010, TRAY_D - 0.010, 0.002)),
        origin=Origin(xyz=(0.0, TRAY_Y, BODY_H - TRAY_DEPTH + 0.001)),
        material=panel_finish,
        name="control_panel",
    )
    for index, x_pos in enumerate((-0.1515, 0.1515)):
        body.visual(
            Box((0.009, 0.118, 0.028)),
            origin=Origin(xyz=(x_pos, OPENING_Y, 0.132)),
            material=panel_finish,
            name=f"pan_support_{index}",
        )

    pan = model.part("pan")
    pan.visual(
        mesh_from_cadquery(_pan_shape(), "bread_pan"),
        material=pan_finish,
        name="pan_shell",
    )
    for index, x_pos in enumerate((-0.063, 0.063)):
        pan.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(xyz=(x_pos, 0.0, 0.007)),
            material=paddle_finish,
            name=f"spindle_mount_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "bread_maker_lid"),
        material=body_finish,
        name="lid_shell",
    )
    lid.visual(
        Box((0.176, 0.086, 0.004)),
        origin=Origin(xyz=(0.0, -0.110, LID_H - 0.006)),
        material=glass_finish,
        name="lid_window",
    )
    lid.visual(
        Box((0.160, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -LID_D + 0.016, 0.046)),
        material=panel_finish,
        name="front_grip",
    )

    model.articulation(
        "body_to_pan",
        ArticulationType.FIXED,
        parent=body,
        child=pan,
        origin=Origin(xyz=(0.0, OPENING_Y, PAN_BOTTOM_Z)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    paddle_positions = (-0.063, 0.063)
    for index, x_pos in enumerate(paddle_positions):
        paddle = model.part(f"paddle_{index}")
        paddle.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=paddle_finish,
            name="hub",
        )
        paddle.visual(
            Box((0.064, 0.022, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material=paddle_finish,
            name="blade",
        )
        paddle.visual(
            Box((0.010, 0.034, 0.018)),
            origin=Origin(xyz=(0.018, 0.0, 0.018)),
            material=paddle_finish,
            name="fin",
        )
        model.articulation(
            f"pan_to_paddle_{index}",
            ArticulationType.CONTINUOUS,
            parent=pan,
            child=paddle,
            origin=Origin(xyz=(x_pos, 0.0, 0.014)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=12.0),
        )

    dial = model.part("selector_dial")
    dial.visual(_dial_mesh(), material=dial_finish, name="dial_shell")
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.112, TRAY_Y, CONTROL_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    button_x_positions = (-0.108, -0.064, -0.020, 0.024)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.026, 0.016, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, 0.0085)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.012, 0.008, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=button_finish,
            name="button_stem",
        )
        model.articulation(
            f"body_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, TRAY_Y, CONTROL_SURFACE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.050,
                lower=-0.0035,
                upper=0.0,
            ),
        )

    return model


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[i] - mins[i]) for i in range(3))


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float((mins[i] + maxs[i]) * 0.5) for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pan = object_model.get_part("pan")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("selector_dial")
    paddle_0 = object_model.get_part("paddle_0")
    paddle_1 = object_model.get_part("paddle_1")
    buttons = [object_model.get_part(f"program_button_{index}") for index in range(4)]

    lid_joint = object_model.get_articulation("body_to_lid")
    dial_joint = object_model.get_articulation("body_to_selector_dial")
    paddle_0_joint = object_model.get_articulation("pan_to_paddle_0")
    paddle_1_joint = object_model.get_articulation("pan_to_paddle_1")
    button_joints = [object_model.get_articulation(f"body_to_program_button_{index}") for index in range(4)]

    body_size = _aabb_size(ctx.part_world_aabb(body))
    pan_size = _aabb_size(ctx.part_world_aabb(pan))
    ctx.check(
        "countertop_scale_body",
        body_size is not None and body_size[0] >= 0.39 and body_size[1] >= 0.27 and body_size[2] >= 0.24,
        details=f"body_size={body_size!r}",
    )
    ctx.check(
        "broad_pan_cavity_scale",
        pan_size is not None and pan_size[0] >= 0.28 and pan_size[1] >= 0.15 and pan_size[2] >= 0.14,
        details=f"pan_size={pan_size!r}",
    )

    ctx.check(
        "selector_dial_is_continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "kneading_paddles_are_continuous",
        paddle_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and paddle_1_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={(paddle_0_joint.articulation_type, paddle_1_joint.articulation_type)!r}",
    )
    ctx.check(
        "all_program_buttons_are_prismatic",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints),
        details=f"types={[joint.articulation_type for joint in button_joints]!r}",
    )

    ctx.expect_within(
        paddle_0,
        pan,
        axes="xy",
        margin=0.015,
        name="paddle_0_stays_within_pan_plan",
    )
    ctx.expect_within(
        paddle_1,
        pan,
        axes="xy",
        margin=0.015,
        name="paddle_1_stays_within_pan_plan",
    )
    ctx.expect_origin_distance(
        paddle_0,
        paddle_1,
        axes="x",
        min_dist=0.11,
        max_dist=0.14,
        name="matched_paddle_spacing",
    )
    ctx.expect_origin_distance(
        paddle_0,
        paddle_1,
        axes="y",
        max_dist=0.005,
        name="matched_paddles_share_one_row",
    )
    ctx.expect_origin_distance(
        dial,
        buttons[-1],
        axes="x",
        min_dist=0.05,
        max_dist=0.11,
        name="dial_sits_beside_button_row",
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed_lid_sits_just_above_housing",
        )
        closed_grip = _aabb_center(ctx.part_element_world_aabb(lid, elem="front_grip"))

    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    open_grip = None
    if lid_upper is not None:
        with ctx.pose({lid_joint: lid_upper}):
            open_grip = _aabb_center(ctx.part_element_world_aabb(lid, elem="front_grip"))

    ctx.check(
        "rear_hinged_lid_front_rises_when_opened",
        closed_grip is not None
        and open_grip is not None
        and open_grip[2] > closed_grip[2] + 0.11
        and open_grip[1] > closed_grip[1] + 0.05,
        details=f"closed_grip={closed_grip!r}, open_grip={open_grip!r}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_rotated = ctx.part_world_position(dial)
    ctx.check(
        "selector_dial_rotates_in_place",
        dial_rest is not None
        and dial_rotated is not None
        and all(abs(dial_rest[i] - dial_rotated[i]) <= 1e-6 for i in range(3)),
        details=f"dial_rest={dial_rest!r}, dial_rotated={dial_rotated!r}",
    )

    button_rest_positions = [ctx.part_world_position(button) for button in buttons]
    press_pose = {
        button_joints[0]: button_joints[0].motion_limits.lower,
        button_joints[3]: button_joints[3].motion_limits.lower,
    }
    with ctx.pose(press_pose):
        button_pressed_positions = [ctx.part_world_position(buttons[index]) for index in (0, 3)]
    ctx.check(
        "program_buttons_press_downward",
        button_rest_positions[0] is not None
        and button_rest_positions[3] is not None
        and button_pressed_positions[0] is not None
        and button_pressed_positions[1] is not None
        and button_pressed_positions[0][2] < button_rest_positions[0][2] - 0.003
        and button_pressed_positions[1][2] < button_rest_positions[3][2] - 0.003,
        details=(
            f"rest={(button_rest_positions[0], button_rest_positions[3])!r}, "
            f"pressed={button_pressed_positions!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
