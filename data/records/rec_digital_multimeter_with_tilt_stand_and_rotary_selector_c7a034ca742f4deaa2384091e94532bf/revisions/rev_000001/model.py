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


BODY_W = 0.088
BODY_D = 0.050
BODY_H = 0.168
FRONT_Y = -BODY_D * 0.5
BACK_Y = BODY_D * 0.5

DISPLAY_Z = 0.128
SELECTOR_Z = 0.084
BUTTON_Z = 0.055
JACK_Z = 0.027


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_multimeter")

    housing_yellow = model.material("housing_yellow", rgba=(0.93, 0.78, 0.18, 1.0))
    panel_charcoal = model.material("panel_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    button_gray = model.material("button_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    rocker_red = model.material("rocker_red", rgba=(0.72, 0.18, 0.12, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.18, 0.40, 0.34, 0.52))
    socket_black = model.material("socket_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D - 0.020, BODY_H)),
        origin=Origin(xyz=(0.0, 0.010, BODY_H * 0.5)),
        material=housing_yellow,
        name="housing_shell",
    )
    body.visual(
        Box((BODY_W - 0.014, BODY_D - 0.024, 0.034)),
        origin=Origin(xyz=(0.0, 0.010, BODY_H - 0.017)),
        material=housing_yellow,
        name="upper_taper",
    )
    body.visual(
        Box((0.014, 0.022, BODY_H - 0.014)),
        origin=Origin(xyz=(-BODY_W * 0.5 + 0.007, -0.014, BODY_H * 0.5)),
        material=housing_yellow,
        name="front_left_wall",
    )
    body.visual(
        Box((0.014, 0.022, BODY_H - 0.014)),
        origin=Origin(xyz=(BODY_W * 0.5 - 0.007, -0.014, BODY_H * 0.5)),
        material=housing_yellow,
        name="front_right_wall",
    )
    body.visual(
        Box((BODY_W - 0.026, 0.022, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, BODY_H - 0.012)),
        material=housing_yellow,
        name="front_top_bridge",
    )
    body.visual(
        Box((BODY_W - 0.022, 0.022, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, 0.012)),
        material=housing_yellow,
        name="front_bottom_bridge",
    )
    body.visual(
        Box((BODY_W + 0.006, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, BODY_H - 0.018)),
        material=housing_yellow,
        name="bumper_top",
    )
    body.visual(
        Box((BODY_W + 0.006, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, 0.008)),
        material=housing_yellow,
        name="bumper_bottom",
    )
    body.visual(
        Box((0.010, 0.004, BODY_H - 0.022)),
        origin=Origin(xyz=(-BODY_W * 0.5 - 0.002, FRONT_Y + 0.001, BODY_H * 0.5)),
        material=housing_yellow,
        name="bumper_left",
    )
    body.visual(
        Box((0.010, 0.004, BODY_H - 0.022)),
        origin=Origin(xyz=(BODY_W * 0.5 + 0.002, FRONT_Y + 0.001, BODY_H * 0.5)),
        material=housing_yellow,
        name="bumper_right",
    )
    body.visual(
        Box((0.072, 0.022, 0.041)),
        origin=Origin(xyz=(0.0, -0.014, 0.032)),
        material=panel_charcoal,
        name="jack_panel",
    )
    body.visual(
        Box((0.050, 0.022, 0.034)),
        origin=Origin(xyz=(0.0, -0.014, DISPLAY_Z)),
        material=panel_charcoal,
        name="display_module",
    )
    body.visual(
        Box((0.040, 0.0014, 0.025)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0007, DISPLAY_Z)),
        material=screen_glass,
        name="display_face",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.0018),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0009, SELECTOR_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_dark,
        name="selector_plate",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.0, -0.015, SELECTOR_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_dark,
        name="selector_boss",
    )

    for index, x_pos in enumerate((-0.022, 0.0, 0.022)):
        body.visual(
            Cylinder(radius=0.0057, length=0.004),
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.002, JACK_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=socket_black,
            name=f"jack_socket_{index}",
        )

    for x_pos in (-0.012, 0.012):
        body.visual(
            Cylinder(radius=0.003, length=0.010),
            origin=Origin(xyz=(x_pos, BACK_Y + 0.002, 0.010), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=panel_charcoal,
            name=f"stand_barrel_{'inboard' if x_pos > 0 else 'outboard'}",
        )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.0195, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="knob_skirt",
    )
    selector.visual(
        Cylinder(radius=0.016, length=0.013),
        origin=Origin(xyz=(0.0, -0.0085, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )
    selector.visual(
        Box((0.003, 0.0016, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.011)),
        material=trim_dark,
        name="pointer_rib",
    )
    selector.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_dark,
        name="shaft_stub",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, SELECTOR_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    for index, x_pos in enumerate((-0.020, 0.0, 0.020)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.016, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.002, 0.005)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, 0.002, 0.004)),
            material=trim_dark,
            name="button_stem",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.004, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=0.0, upper=0.0022),
        )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.0042, 0.010, 0.024)),
        origin=Origin(xyz=(0.0021, 0.0, 0.0)),
        material=rocker_red,
        name="rocker_cap",
    )
    power_switch.visual(
        Box((0.0036, 0.007, 0.010)),
        origin=Origin(xyz=(0.0022, 0.0, 0.0)),
        material=trim_dark,
        name="rocker_body",
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(BODY_W * 0.5, 0.0, 0.106)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=math.radians(-18.0),
            upper=math.radians(18.0),
        ),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.0026, length=0.012),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stand_dark,
        name="stand_barrel",
    )
    stand.visual(
        Box((0.034, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, 0.002, 0.031)),
        material=stand_dark,
        name="stand_panel",
    )
    stand.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.003, 0.058)),
        material=stand_dark,
        name="stand_foot",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BACK_Y + 0.002, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    return model


def _aabb_center(aabb):
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    power_switch = object_model.get_part("power_switch")
    middle_button = object_model.get_part("mode_button_1")

    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")
    switch_joint = object_model.get_articulation("body_to_power_switch")
    button_joint = object_model.get_articulation("body_to_mode_button_1")

    ctx.allow_overlap(
        body,
        selector,
        elem_a="selector_boss",
        elem_b="shaft_stub",
        reason="The selector shaft intentionally nests inside the selector bushing.",
    )

    ctx.expect_contact(
        selector,
        body,
        elem_a="shaft_stub",
        elem_b="selector_boss",
        name="selector shaft seats at body boss",
    )

    display_aabb = ctx.part_element_world_aabb(body, elem="display_face")
    selector_aabb = ctx.part_world_aabb(selector)
    jack_aabb = ctx.part_element_world_aabb(body, elem="jack_socket_1")
    button_aabb = ctx.part_world_aabb(middle_button)

    display_center = _aabb_center(display_aabb) if display_aabb is not None else None
    selector_center = _aabb_center(selector_aabb) if selector_aabb is not None else None
    jack_center = _aabb_center(jack_aabb) if jack_aabb is not None else None
    button_center = _aabb_center(button_aabb) if button_aabb is not None else None

    ctx.check(
        "display sits above selector",
        display_center is not None and selector_center is not None and display_center[2] > selector_center[2] + 0.028,
        details=f"display_center={display_center}, selector_center={selector_center}",
    )
    ctx.check(
        "mode buttons sit above input jacks",
        button_center is not None and jack_center is not None and button_center[2] > jack_center[2] + 0.020,
        details=f"button_center={button_center}, jack_center={jack_center}",
    )
    ctx.expect_origin_gap(
        power_switch,
        middle_button,
        axis="x",
        min_gap=0.028,
        name="side rocker stays separate from front button bank",
    )

    selector_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_joint: 1.4}):
        selector_turned = ctx.part_world_position(selector)
    ctx.check(
        "selector rotates about fixed shaft center",
        selector_rest is not None
        and selector_turned is not None
        and max(abs(selector_rest[i] - selector_turned[i]) for i in range(3)) < 1e-6,
        details=f"rest={selector_rest}, turned={selector_turned}",
    )

    button_rest = ctx.part_world_position(middle_button)
    with ctx.pose({button_joint: 0.0022}):
        button_pressed = ctx.part_world_position(middle_button)
    ctx.check(
        "mode button depresses inward",
        button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.0015,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    switch_rest = ctx.part_element_world_aabb(power_switch, elem="rocker_cap")
    with ctx.pose({switch_joint: math.radians(18.0)}):
        switch_on = ctx.part_element_world_aabb(power_switch, elem="rocker_cap")
    switch_rest_center = _aabb_center(switch_rest) if switch_rest is not None else None
    switch_on_center = _aabb_center(switch_on) if switch_on is not None else None
    ctx.check(
        "power rocker tips outward at upper end",
        switch_rest is not None
        and switch_on is not None
        and switch_on[1][0] > switch_rest[1][0] + 0.0012
        and switch_on_center is not None
        and switch_rest_center is not None,
        details=f"rest={switch_rest_center}, tipped_max_x={switch_on[1][0]}, rest_max_x={switch_rest[1][0]}",
    )

    stand_rest = ctx.part_world_aabb(stand)
    with ctx.pose({stand_joint: math.radians(62.0)}):
        stand_open = ctx.part_world_aabb(stand)
    stand_rest_center = _aabb_center(stand_rest) if stand_rest is not None else None
    stand_open_center = _aabb_center(stand_open) if stand_open is not None else None
    ctx.check(
        "rear stand swings outward from the back",
        stand_rest_center is not None
        and stand_open_center is not None
        and stand_open_center[1] < stand_rest_center[1] - 0.020,
        details=f"rest={stand_rest_center}, open={stand_open_center}",
    )

    return ctx.report()


object_model = build_object_model()
