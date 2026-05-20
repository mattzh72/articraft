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

BODY_W = 0.52
BODY_D = 0.40
BODY_H = 0.31
SHELL_T = 0.012
FOOT_H = 0.014
SHELL_BOTTOM_Z = FOOT_H
FRONT_Y = -BODY_D / 2.0
REAR_Y = BODY_D / 2.0

DOOR_W = 0.372
DOOR_H = 0.190
DOOR_T = 0.020
DOOR_X = -0.066
DOOR_HINGE_Z = 0.058

CONTROL_W = 0.120
CONTROL_X = 0.190
DIAL_X = 0.176
DIAL_Z = 0.161
BUTTON_X = 0.223
BUTTON_ZS = (0.194, 0.128)

TRAY_W = 0.342
TRAY_DEPTH = 0.308
TRAY_TRAVEL = 0.120
TRAY_JOINT_Z = 0.026


def _add_body(model: ArticulatedObject):
    steel = model.material("steel", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.13, 0.14, 0.15, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.23, 0.27, 0.35))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    shell_inner_z_min = SHELL_BOTTOM_Z + SHELL_T
    shell_inner_z_max = BODY_H - SHELL_T
    shell_inner_height = shell_inner_z_max - shell_inner_z_min

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_BOTTOM_Z + SHELL_T / 2.0)),
        material=steel,
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - SHELL_T / 2.0)),
        material=steel,
        name="top_shell",
    )
    body.visual(
        Box((SHELL_T, BODY_D, shell_inner_height)),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + SHELL_T / 2.0, 0.0, (shell_inner_z_min + shell_inner_z_max) / 2.0)
        ),
        material=steel,
        name="side_shell_0",
    )
    body.visual(
        Box((SHELL_T, BODY_D, shell_inner_height)),
        origin=Origin(
            xyz=(BODY_W / 2.0 - SHELL_T / 2.0, 0.0, (shell_inner_z_min + shell_inner_z_max) / 2.0)
        ),
        material=steel,
        name="side_shell_1",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, shell_inner_height)),
        origin=Origin(
            xyz=(0.0, REAR_Y - SHELL_T / 2.0, (shell_inner_z_min + shell_inner_z_max) / 2.0)
        ),
        material=steel,
        name="rear_shell",
    )

    front_frame_w = DOOR_W + 0.028
    body.visual(
        Box((front_frame_w, SHELL_T, BODY_H - (DOOR_HINGE_Z + DOOR_H))),
        origin=Origin(
            xyz=(
                DOOR_X,
                FRONT_Y + SHELL_T / 2.0,
                DOOR_HINGE_Z + DOOR_H + (BODY_H - (DOOR_HINGE_Z + DOOR_H)) / 2.0,
            )
        ),
        material=steel,
        name="top_bezel",
    )

    jamb_w = 0.014
    body.visual(
        Box((jamb_w, SHELL_T, DOOR_H)),
        origin=Origin(
            xyz=(DOOR_X - DOOR_W / 2.0 - jamb_w / 2.0, FRONT_Y + SHELL_T / 2.0, DOOR_HINGE_Z + DOOR_H / 2.0)
        ),
        material=steel,
        name="door_jamb_0",
    )
    body.visual(
        Box((jamb_w, SHELL_T, DOOR_H)),
        origin=Origin(
            xyz=(DOOR_X + DOOR_W / 2.0 + jamb_w / 2.0, FRONT_Y + SHELL_T / 2.0, DOOR_HINGE_Z + DOOR_H / 2.0)
        ),
        material=steel,
        name="door_jamb_1",
    )

    body.visual(
        Box((CONTROL_W, SHELL_T, BODY_H - 2.0 * SHELL_T)),
        origin=Origin(
            xyz=(
                CONTROL_X,
                FRONT_Y + SHELL_T / 2.0,
                SHELL_BOTTOM_Z + SHELL_T + (BODY_H - 2.0 * SHELL_T - SHELL_BOTTOM_Z) / 2.0,
            )
        ),
        material=dark_trim,
        name="control_panel",
    )
    body.visual(
        Box((CONTROL_W - 0.018, 0.004, BODY_H - 0.110)),
        origin=Origin(xyz=(CONTROL_X, FRONT_Y + 0.002, 0.165)),
        material=glass,
        name="control_gloss",
    )

    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, BODY_D - SHELL_T, 0.004)),
        origin=Origin(xyz=(0.0, -0.003, 0.050)),
        material=dark_trim,
        name="cavity_floor",
    )
    foot_offsets = (
        (-0.195, -0.145),
        (0.195, -0.145),
        (-0.195, 0.145),
        (0.195, 0.145),
    )
    for index, (x_pos, y_pos) in enumerate(foot_offsets):
        body.visual(
            Box((0.050, 0.030, FOOT_H)),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_H / 2.0)),
            material=rubber,
            name=f"foot_{index}",
        )

    return body


def _add_door(model: ArticulatedObject, body):
    steel = model.material("door_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    glass = model.material("door_glass", rgba=(0.17, 0.20, 0.23, 0.32))
    handle_finish = model.material("handle_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    door = model.part("door")
    stile_w = 0.030
    top_h = 0.028
    bottom_h = 0.045
    glass_h = 0.120

    door.visual(
        Box((DOOR_W, DOOR_T, bottom_h)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, bottom_h / 2.0)),
        material=steel,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, top_h)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, DOOR_H - top_h / 2.0)),
        material=steel,
        name="top_rail",
    )
    door.visual(
        Box((stile_w, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + stile_w / 2.0, -DOOR_T / 2.0, DOOR_H / 2.0)),
        material=steel,
        name="stile_0",
    )
    door.visual(
        Box((stile_w, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0 - stile_w / 2.0, -DOOR_T / 2.0, DOOR_H / 2.0)),
        material=steel,
        name="stile_1",
    )
    door.visual(
        Box((DOOR_W - 0.072, 0.006, glass_h)),
        origin=Origin(xyz=(0.0, -0.006, 0.105)),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        Box((0.020, 0.024, 0.020)),
        origin=Origin(xyz=(-0.060, -0.018, 0.132)),
        material=steel,
        name="handle_post_0",
    )
    door.visual(
        Box((0.020, 0.024, 0.020)),
        origin=Origin(xyz=(0.060, -0.018, 0.132)),
        material=steel,
        name="handle_post_1",
    )
    door.visual(
        Box((0.196, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.031, 0.132)),
        material=handle_finish,
        name="handle_bar",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_X, FRONT_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    return door


def _add_dial(model: ArticulatedObject, body):
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    marker_finish = model.material("marker_finish", rgba=(0.88, 0.32, 0.22, 1.0))

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="dial_shaft",
    )
    dial.visual(
        Box((0.003, 0.002, 0.016)),
        origin=Origin(xyz=(0.0, -0.019, 0.010)),
        material=marker_finish,
        name="dial_marker",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, FRONT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    return dial


def _add_button(model: ArticulatedObject, body, *, name: str, z_pos: float):
    cap_finish = model.material(f"{name}_cap_finish", rgba=(0.16, 0.17, 0.18, 1.0))

    button = model.part(name)
    button.visual(
        Box((0.028, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=cap_finish,
        name="cap",
    )
    button.visual(
        Box((0.016, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=cap_finish,
        name="stem",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(BUTTON_X, FRONT_Y, z_pos)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.12, lower=0.0, upper=0.006),
    )
    return button


def _add_crumb_tray(model: ArticulatedObject, body):
    tray_finish = model.material("tray_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    pull_finish = model.material("tray_pull", rgba=(0.11, 0.11, 0.12, 1.0))

    tray = model.part("crumb_tray")
    tray.visual(
        Box((TRAY_W, TRAY_DEPTH, 0.004)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH / 2.0, 0.002)),
        material=tray_finish,
        name="tray_base",
    )
    tray.visual(
        Box((0.004, TRAY_DEPTH, 0.018)),
        origin=Origin(xyz=(-TRAY_W / 2.0 + 0.002, TRAY_DEPTH / 2.0, 0.009)),
        material=tray_finish,
        name="tray_side_0",
    )
    tray.visual(
        Box((0.004, TRAY_DEPTH, 0.018)),
        origin=Origin(xyz=(TRAY_W / 2.0 - 0.002, TRAY_DEPTH / 2.0, 0.009)),
        material=tray_finish,
        name="tray_side_1",
    )
    tray.visual(
        Box((TRAY_W, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH - 0.002, 0.009)),
        material=tray_finish,
        name="tray_back",
    )
    tray.visual(
        Box((TRAY_W, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.006, 0.012)),
        material=tray_finish,
        name="tray_front",
    )
    tray.visual(
        Box((0.110, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, 0.012)),
        material=pull_finish,
        name="tray_pull",
    )

    model.articulation(
        "body_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(DOOR_X, FRONT_Y, TRAY_JOINT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=0.20, lower=0.0, upper=TRAY_TRAVEL),
    )
    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_toaster_oven")

    body = _add_body(model)
    _add_door(model, body)
    _add_dial(model, body)
    _add_button(model, body, name="button_0", z_pos=BUTTON_ZS[0])
    _add_button(model, body, name="button_1", z_pos=BUTTON_ZS[1])
    _add_crumb_tray(model, body)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    crumb_tray = object_model.get_part("crumb_tray")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")
    tray_joint = object_model.get_articulation("body_to_crumb_tray")

    ctx.allow_overlap(
        body,
        button_0,
        elem_a="control_panel",
        elem_b="stem",
        reason="The button stem is intentionally represented as entering the simplified front control panel.",
    )
    ctx.allow_overlap(
        body,
        button_1,
        elem_a="control_panel",
        elem_b="stem",
        reason="The button stem is intentionally represented as entering the simplified front control panel.",
    )
    ctx.allow_overlap(
        body,
        dial,
        elem_a="control_panel",
        elem_b="dial_shaft",
        reason="The dial shaft is intentionally represented as passing into the simplified control panel.",
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        name="door closes flush with the front face",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.16,
        name="door covers the oven opening footprint",
    )
    ctx.expect_within(
        crumb_tray,
        body,
        axes="xz",
        margin=0.003,
        name="crumb tray stays aligned in the lower slot",
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "center dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"type={dial_joint.articulation_type!r}, limits={dial_limits!r}",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        opened_door_aabb = ctx.part_world_aabb(door)
    door_opens_down = False
    if closed_door_aabb is not None and opened_door_aabb is not None:
        closed_min, closed_max = closed_door_aabb
        opened_min, opened_max = opened_door_aabb
        door_opens_down = (
            opened_max[2] < closed_max[2] - 0.07 and opened_min[1] < closed_min[1] - 0.08
        )
        door_details = f"closed={closed_door_aabb!r}, open={opened_door_aabb!r}"
    else:
        door_details = f"closed={closed_door_aabb!r}, open={opened_door_aabb!r}"
    ctx.check("door drops forward and downward when opened", door_opens_down, details=door_details)

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_0_joint: 0.006}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_during_button_0 = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.006}):
        button_1_pressed = ctx.part_world_position(button_1)
        button_0_during_button_1 = ctx.part_world_position(button_0)

    ctx.check(
        "button_0 depresses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[1] > button_0_rest[1] + 0.004,
        details=f"rest={button_0_rest!r}, pressed={button_0_pressed!r}",
    )
    ctx.check(
        "button_1 depresses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[1] > button_1_rest[1] + 0.004,
        details=f"rest={button_1_rest!r}, pressed={button_1_pressed!r}",
    )
    ctx.check(
        "buttons articulate independently",
        button_1_rest is not None
        and button_1_during_button_0 is not None
        and button_0_rest is not None
        and button_0_during_button_1 is not None
        and abs(button_1_during_button_0[1] - button_1_rest[1]) < 1e-6
        and abs(button_0_during_button_1[1] - button_0_rest[1]) < 1e-6,
        details=(
            f"button_1_rest={button_1_rest!r}, button_1_during_button_0={button_1_during_button_0!r}, "
            f"button_0_rest={button_0_rest!r}, button_0_during_button_1={button_0_during_button_1!r}"
        ),
    )

    tray_rest = ctx.part_world_position(crumb_tray)
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        tray_open = ctx.part_world_position(crumb_tray)
        ctx.expect_within(
            crumb_tray,
            body,
            axes="xz",
            margin=0.003,
            name="crumb tray remains guided when extended",
        )
    ctx.check(
        "crumb tray slides forward",
        tray_rest is not None and tray_open is not None and tray_open[1] < tray_rest[1] - 0.08,
        details=f"rest={tray_rest!r}, open={tray_open!r}",
    )

    ctx.check("major parts present", all(part is not None for part in (body, door, dial, button_0, button_1, crumb_tray)))
    return ctx.report()


object_model = build_object_model()
