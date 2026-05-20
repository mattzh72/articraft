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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speedlight")

    shell_black = model.material("shell_black", rgba=(0.14, 0.14, 0.15, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.21, 0.21, 0.22, 1.0))
    diffuser = model.material("diffuser", rgba=(0.88, 0.89, 0.90, 0.92))
    metal = model.material("metal", rgba=(0.50, 0.50, 0.52, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.046, 0.034, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=shell_black,
        name="battery_body",
    )
    body.visual(
        Box((0.040, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=shell_dark,
        name="upper_shoulder",
    )
    body.visual(
        Box((0.024, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.077)),
        material=shell_dark,
        name="shoe_collar",
    )
    body.visual(
        Box((0.020, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0835)),
        material=metal,
        name="shoe_foot",
    )
    body.visual(
        Box((0.020, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, 0.0045, -0.0825)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.020, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, -0.0045, -0.0825)),
        material=metal,
        name="shoe_rail_1",
    )

    aiming_neck = model.part("aiming_neck")
    aiming_neck.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell_dark,
        name="swivel_collar",
    )
    aiming_neck.visual(
        Box((0.014, 0.018, 0.028)),
        origin=Origin(xyz=(-0.005, 0.0, 0.024)),
        material=shell_dark,
        name="neck_spine",
    )
    aiming_neck.visual(
        Box((0.022, 0.036, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.040)),
        material=shell_dark,
        name="support_bridge",
    )
    aiming_neck.visual(
        Box((0.020, 0.018, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, 0.037)),
        material=shell_dark,
        name="head_cradle",
    )
    aiming_neck.visual(
        Box((0.012, 0.004, 0.020)),
        origin=Origin(xyz=(0.008, 0.018, 0.046)),
        material=shell_dark,
        name="cheek_0",
    )
    aiming_neck.visual(
        Box((0.012, 0.004, 0.020)),
        origin=Origin(xyz=(0.008, -0.018, 0.046)),
        material=shell_dark,
        name="cheek_1",
    )

    head = model.part("head")
    head.visual(
        Box((0.074, 0.066, 0.032)),
        origin=Origin(xyz=(0.040, 0.0, 0.012)),
        material=shell_black,
        name="head_shell",
    )
    head.visual(
        Box((0.006, 0.056, 0.024)),
        origin=Origin(xyz=(0.077, 0.0, 0.012)),
        material=diffuser,
        name="diffuser",
    )
    head.visual(
        Box((0.014, 0.030, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 0.010)),
        material=shell_dark,
        name="pivot_block",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.022, 0.004, 0.048)),
        origin=Origin(),
        material=shell_dark,
        name="panel",
    )
    control_strip.visual(
        Box((0.024, 0.0015, 0.052)),
        origin=Origin(xyz=(0.0, 0.00275, 0.0)),
        material=shell_black,
        name="bezel",
    )

    mode_buttons = []
    for index, z_pos in enumerate((0.016, 0.005, -0.006, -0.017)):
        mode_button = model.part(f"mode_button_{index}")
        mode_button.visual(
            Box((0.012, 0.0045, 0.007)),
            origin=Origin(xyz=(0.0, 0.00225, 0.0)),
            material=shell_black,
            name="cap",
        )
        mode_buttons.append((mode_button, z_pos))

    model.articulation(
        "body_to_aiming_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aiming_neck,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-math.radians(95.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "aiming_neck_to_head",
        ArticulationType.REVOLUTE,
        parent=aiming_neck,
        child=head,
        origin=Origin(xyz=(0.010, 0.0, 0.046)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(-0.003, 0.019, -0.032)),
    )
    for index, (mode_button, z_pos) in enumerate(mode_buttons):
        model.articulation(
            f"control_strip_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=mode_button,
            origin=Origin(xyz=(0.0, 0.002, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=0.04,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    aiming_neck = object_model.get_part("aiming_neck")
    head = object_model.get_part("head")
    control_strip = object_model.get_part("control_strip")
    swivel = object_model.get_articulation("body_to_aiming_neck")
    tilt = object_model.get_articulation("aiming_neck_to_head")

    ctx.expect_contact(
        aiming_neck,
        body,
        elem_a="swivel_collar",
        elem_b="upper_shoulder",
        name="swivel collar seats on the body roof",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.03,
        name="lamp head stays clearly above the battery body",
    )
    ctx.expect_contact(
        control_strip,
        body,
        elem_a="panel",
        elem_b="battery_body",
        name="control strip is mounted on the body side",
    )
    ctx.expect_gap(
        head,
        control_strip,
        axis="z",
        min_gap=0.04,
        name="lamp head stays separate from the side control strip",
    )
    ctx.expect_gap(
        aiming_neck,
        control_strip,
        axis="z",
        min_gap=0.005,
        name="aiming neck stays above the side controls",
    )

    rest_diffuser = ctx.part_element_world_aabb(head, elem="diffuser")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_diffuser = ctx.part_element_world_aabb(head, elem="diffuser")

    rest_shell = ctx.part_element_world_aabb(head, elem="head_shell")
    with ctx.pose({swivel: swivel.motion_limits.upper}):
        swiveled_shell = ctx.part_element_world_aabb(head, elem="head_shell")

    rest_diffuser_top = rest_diffuser[1][2] if rest_diffuser is not None else None
    tilted_diffuser_top = tilted_diffuser[1][2] if tilted_diffuser is not None else None
    rest_shell_center_y = (
        (rest_shell[0][1] + rest_shell[1][1]) * 0.5 if rest_shell is not None else None
    )
    swiveled_shell_center_y = (
        (swiveled_shell[0][1] + swiveled_shell[1][1]) * 0.5 if swiveled_shell is not None else None
    )

    ctx.check(
        "lamp head tilts upward",
        rest_diffuser_top is not None
        and tilted_diffuser_top is not None
        and tilted_diffuser_top > rest_diffuser_top + 0.03,
        details=f"rest_top={rest_diffuser_top}, tilted_top={tilted_diffuser_top}",
    )
    ctx.check(
        "lamp head swivels sideways",
        rest_shell_center_y is not None
        and swiveled_shell_center_y is not None
        and abs(swiveled_shell_center_y - rest_shell_center_y) > 0.015,
        details=f"rest_y={rest_shell_center_y}, swiveled_y={swiveled_shell_center_y}",
    )

    button_positions = {}
    button_joints = {}
    for index in range(4):
        button = object_model.get_part(f"mode_button_{index}")
        button_joint = object_model.get_articulation(f"control_strip_to_mode_button_{index}")
        button_positions[index] = ctx.part_world_position(button)
        button_joints[index] = button_joint

    for active_index in range(4):
        with ctx.pose({button_joints[active_index]: button_joints[active_index].motion_limits.upper}):
            pressed_positions = {
                index: ctx.part_world_position(object_model.get_part(f"mode_button_{index}"))
                for index in range(4)
            }

        active_rest = button_positions[active_index]
        active_pressed = pressed_positions[active_index]
        ctx.check(
            f"mode button {active_index} depresses inward",
            active_rest is not None
            and active_pressed is not None
            and active_pressed[1] < active_rest[1] - 0.001,
            details=f"rest={active_rest}, pressed={active_pressed}",
        )

        other_ok = True
        other_details = []
        for other_index in range(4):
            if other_index == active_index:
                continue
            rest_pos = button_positions[other_index]
            pressed_pos = pressed_positions[other_index]
            stable = (
                rest_pos is not None
                and pressed_pos is not None
                and abs(pressed_pos[1] - rest_pos[1]) < 1e-9
            )
            other_ok = other_ok and stable
            other_details.append(
                f"{other_index}:rest={rest_pos},pressed={pressed_pos},stable={stable}"
            )

        ctx.check(
            f"mode button {active_index} moves independently",
            other_ok,
            details="; ".join(other_details),
        )

    return ctx.report()


object_model = build_object_model()
