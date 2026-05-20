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


BODY_W = 0.280
BODY_D = 0.330
BODY_H = 0.405
BODY_WALL = 0.018
BODY_BASE_H = 0.064
BODY_TOP_T = 0.018

DISPLAY_W = 0.204
DISPLAY_H = 0.142
DISPLAY_T = 0.006
DISPLAY_Z = 0.321
DISPLAY_FRONT_Y = -BODY_D / 2.0 - DISPLAY_T / 2.0

OPENING_LIP_H = 0.010
OPENING_LIP_Z = 0.244

DRAWER_W = 0.226
DRAWER_D = 0.238
DRAWER_H = 0.158
DRAWER_WALL = 0.006
DRAWER_FLOOR = 0.005
DRAWER_FACE_W = 0.248
DRAWER_FACE_H = 0.156
DRAWER_FACE_D = 0.022
DRAWER_Z = 0.078
DRAWER_Y = -BODY_D / 2.0
DRAWER_TRAVEL = 0.170

HANDLE_POST_W = 0.016
HANDLE_POST_D = 0.040
HANDLE_POST_H = 0.034
HANDLE_POST_Y = -0.014
HANDLE_POST_Z = 0.122
HANDLE_BAR_W = 0.034
HANDLE_BAR_D = 0.022
HANDLE_BAR_H = 0.016
HANDLE_BAR_Y = -0.040
HANDLE_BAR_Z = 0.142

RELEASE_W = 0.028
RELEASE_D = 0.016
RELEASE_H = 0.006
RELEASE_TRAVEL = 0.0025
RELEASE_Y = -0.040
RELEASE_Z = 0.150

KNOB_RADIUS = 0.028
KNOB_DEPTH = 0.024
KNOB_Z = DISPLAY_Z
KNOB_CENTER_Y = DISPLAY_FRONT_Y - KNOB_DEPTH / 2.0

BUTTON_W = 0.034
BUTTON_D = 0.010
BUTTON_H = 0.018
BUTTON_TRAVEL = 0.003
BUTTON_CENTER_Y = DISPLAY_FRONT_Y - BUTTON_D / 2.0
BUTTON_LAYOUT = (
    ("preset_button_0", (-0.060, 0.040)),
    ("preset_button_1", (0.000, 0.056)),
    ("preset_button_2", (0.060, 0.040)),
    ("preset_button_3", (-0.060, -0.040)),
    ("preset_button_4", (0.000, -0.056)),
    ("preset_button_5", (0.060, -0.040)),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    band_finish = model.material("band_finish", rgba=(0.05, 0.06, 0.07, 1.0))
    window_finish = model.material("window_finish", rgba=(0.16, 0.30, 0.34, 0.52))
    drawer_finish = model.material("drawer_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    control_finish = model.material("control_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.72, 0.76, 0.80, 1.0))

    body = model.part("body")
    upper_h = BODY_H - BODY_BASE_H
    body.visual(
        Box((BODY_W, BODY_D, BODY_BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_H / 2.0)),
        material=body_finish,
        name="base_shell",
    )
    body.visual(
        Box((BODY_WALL, BODY_D, upper_h)),
        origin=Origin(xyz=(-BODY_W / 2.0 + BODY_WALL / 2.0, 0.0, BODY_BASE_H + upper_h / 2.0)),
        material=body_finish,
        name="left_shell",
    )
    body.visual(
        Box((BODY_WALL, BODY_D, upper_h)),
        origin=Origin(xyz=(BODY_W / 2.0 - BODY_WALL / 2.0, 0.0, BODY_BASE_H + upper_h / 2.0)),
        material=body_finish,
        name="right_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_WALL, upper_h)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - BODY_WALL / 2.0, BODY_BASE_H + upper_h / 2.0)
        ),
        material=body_finish,
        name="rear_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - BODY_TOP_T / 2.0)),
        material=body_finish,
        name="top_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_WALL, DISPLAY_H + 0.002)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + BODY_WALL / 2.0, DISPLAY_Z)
        ),
        material=body_finish,
        name="band_backer",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_WALL, OPENING_LIP_H)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + BODY_WALL / 2.0, OPENING_LIP_Z)
        ),
        material=body_finish,
        name="opening_lip",
    )
    body.visual(
        Box((DISPLAY_W, DISPLAY_T, DISPLAY_H)),
        origin=Origin(xyz=(0.0, DISPLAY_FRONT_Y, DISPLAY_Z)),
        material=band_finish,
        name="display_panel",
    )
    body.visual(
        Box((0.096, 0.002, 0.014)),
        origin=Origin(xyz=(0.0, DISPLAY_FRONT_Y - 0.002, DISPLAY_Z + 0.052)),
        material=window_finish,
        name="display_window",
    )

    basket = model.part("basket")
    basket.visual(
        Box((DRAWER_W, DRAWER_D, DRAWER_FLOOR)),
        origin=Origin(xyz=(0.0, DRAWER_D / 2.0, DRAWER_FLOOR / 2.0)),
        material=drawer_finish,
        name="basket_floor",
    )
    basket.visual(
        Box((DRAWER_WALL, DRAWER_D, DRAWER_H)),
        origin=Origin(
            xyz=(-DRAWER_W / 2.0 + DRAWER_WALL / 2.0, DRAWER_D / 2.0, DRAWER_H / 2.0)
        ),
        material=drawer_finish,
        name="basket_side_0",
    )
    basket.visual(
        Box((DRAWER_WALL, DRAWER_D, DRAWER_H)),
        origin=Origin(
            xyz=(DRAWER_W / 2.0 - DRAWER_WALL / 2.0, DRAWER_D / 2.0, DRAWER_H / 2.0)
        ),
        material=drawer_finish,
        name="basket_side_1",
    )
    basket.visual(
        Box((DRAWER_W, DRAWER_WALL, DRAWER_H)),
        origin=Origin(xyz=(0.0, DRAWER_D - DRAWER_WALL / 2.0, DRAWER_H / 2.0)),
        material=drawer_finish,
        name="basket_rear",
    )
    basket.visual(
        Box((DRAWER_FACE_W, DRAWER_FACE_D, DRAWER_FACE_H)),
        origin=Origin(xyz=(0.0, -DRAWER_FACE_D / 2.0, DRAWER_FACE_H / 2.0)),
        material=drawer_finish,
        name="drawer_face",
    )
    basket.visual(
        Box((HANDLE_POST_W, HANDLE_POST_D, HANDLE_POST_H)),
        origin=Origin(xyz=(-0.042, HANDLE_POST_Y, HANDLE_POST_Z)),
        material=handle_finish,
        name="handle_post_0",
    )
    basket.visual(
        Box((HANDLE_POST_W, HANDLE_POST_D, HANDLE_POST_H)),
        origin=Origin(xyz=(0.042, HANDLE_POST_Y, HANDLE_POST_Z)),
        material=handle_finish,
        name="handle_post_1",
    )
    basket.visual(
        Box((HANDLE_BAR_W, HANDLE_BAR_D, HANDLE_BAR_H)),
        origin=Origin(xyz=(-0.025, HANDLE_BAR_Y, HANDLE_BAR_Z)),
        material=handle_finish,
        name="handle_bar_0",
    )
    basket.visual(
        Box((HANDLE_BAR_W, HANDLE_BAR_D, HANDLE_BAR_H)),
        origin=Origin(xyz=(0.025, HANDLE_BAR_Y, HANDLE_BAR_Z)),
        material=handle_finish,
        name="handle_bar_1",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((RELEASE_W, RELEASE_D, RELEASE_H)),
        origin=Origin(xyz=(0.0, 0.0, RELEASE_H / 2.0)),
        material=accent_finish,
        name="release_cap",
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.002, 0.016)),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH / 2.0 - 0.001, KNOB_RADIUS * 0.58)),
        material=accent_finish,
        name="knob_indicator",
    )

    button_parts = []
    for name, (x_pos, z_pos) in BUTTON_LAYOUT:
        button = model.part(name)
        button.visual(
            Box((BUTTON_W, BUTTON_D, BUTTON_H)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=control_finish,
            name="button_cap",
        )
        button_parts.append((name, button, x_pos, z_pos))

    model.articulation(
        "body_to_basket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, DRAWER_Y, DRAWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "basket_to_release_button",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=release_button,
        origin=Origin(xyz=(0.0, RELEASE_Y, RELEASE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=RELEASE_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, KNOB_CENTER_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    for name, button, x_pos, z_pos in button_parts:
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_CENTER_Y, KNOB_Z + z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    release_button = object_model.get_part("release_button")
    knob = object_model.get_part("selector_knob")
    drawer_joint = object_model.get_articulation("body_to_basket")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    release_joint = object_model.get_articulation("basket_to_release_button")

    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.expect_overlap(
        basket,
        body,
        axes="xz",
        min_overlap=0.15,
        name="basket stays centered in the fryer cavity",
    )

    rest_basket = ctx.part_world_position(basket)
    open_basket = None
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            basket,
            body,
            axes="xz",
            min_overlap=0.15,
            name="extended basket remains aligned with the body opening",
        )
        open_basket = ctx.part_world_position(basket)

    ctx.check(
        "basket extends forward",
        rest_basket is not None
        and open_basket is not None
        and open_basket[1] < rest_basket[1] - 0.12,
        details=f"rest={rest_basket}, open={open_basket}",
    )

    knob_rest = ctx.part_world_position(knob)
    knob_rotated = None
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "selector knob rotates about a fixed center",
        knob_rest is not None
        and knob_rotated is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(knob_rest, knob_rotated)),
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    button_names = [name for name, _ in BUTTON_LAYOUT]
    button_joints = [object_model.get_articulation(f"body_to_{name}") for name in button_names]
    button_parts = [object_model.get_part(name) for name in button_names]
    ctx.check(
        "six preset buttons are prismatic",
        len(button_joints) == 6
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints),
        details=str([joint.articulation_type for joint in button_joints]),
    )

    rest_positions = [ctx.part_world_position(part) for part in button_parts]
    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        moved_positions = [ctx.part_world_position(part) for part in button_parts]
    moved_first = (
        rest_positions[0] is not None
        and moved_positions[0] is not None
        and moved_positions[0][1] > rest_positions[0][1] + 0.001
    )
    others_still = all(
        rest is not None
        and moved is not None
        and abs(moved[1] - rest[1]) < 1e-6
        for rest, moved in zip(rest_positions[1:], moved_positions[1:])
    )
    ctx.check(
        "preset buttons move independently",
        moved_first and others_still,
        details=f"rest={rest_positions}, moved={moved_positions}",
    )

    rest_release = ctx.part_world_position(release_button)
    pressed_release = None
    with ctx.pose({release_joint: RELEASE_TRAVEL}):
        pressed_release = ctx.part_world_position(release_button)
    ctx.check(
        "release button presses downward",
        rest_release is not None
        and pressed_release is not None
        and pressed_release[2] < rest_release[2] - 0.0015,
        details=f"rest={rest_release}, pressed={pressed_release}",
    )

    return ctx.report()


object_model = build_object_model()
