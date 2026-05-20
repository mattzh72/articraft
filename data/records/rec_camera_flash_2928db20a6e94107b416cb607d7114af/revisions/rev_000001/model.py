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
    model = ArticulatedObject(name="mirrorless_hot_shoe_flash")

    plastic_black = model.material("plastic_black", rgba=(0.12, 0.12, 0.13, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    plastic_gray = model.material("plastic_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    button_gray = model.material("button_gray", rgba=(0.50, 0.51, 0.53, 1.0))
    lens_white = model.material("lens_white", rgba=(0.93, 0.94, 0.96, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.09, 0.11, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.68, 0.70, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.045, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=plastic_black,
        name="battery_body",
    )
    body.visual(
        Box((0.036, 0.056, 0.016)),
        origin=Origin(xyz=(0.004, 0.0, 0.083)),
        material=plastic_dark,
        name="top_shoulder",
    )
    body.visual(
        Box((0.014, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=plastic_dark,
        name="shoe_stem",
    )
    body.visual(
        Box((0.016, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=metal,
        name="shoe_foot",
    )
    body.visual(
        Box((0.0015, 0.036, 0.019)),
        origin=Origin(xyz=(-0.02325, 0.0, 0.064)),
        material=screen_dark,
        name="display",
    )
    body.visual(
        Box((0.0020, 0.046, 0.025)),
        origin=Origin(xyz=(-0.0235, 0.0, 0.034)),
        material=plastic_dark,
        name="rear_panel",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(-0.0235, 0.0, 0.033), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_gray,
        name="rear_dial",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.002),
        origin=Origin(xyz=(-0.024, 0.0, 0.033), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_gray,
        name="rear_select",
    )
    body.visual(
        Box((0.002, 0.012, 0.005)),
        origin=Origin(xyz=(-0.024, 0.0, 0.049)),
        material=button_gray,
        name="rear_menu_button",
    )
    body.visual(
        Box((0.002, 0.012, 0.005)),
        origin=Origin(xyz=(-0.024, 0.0, 0.020)),
        material=button_gray,
        name="rear_set_button",
    )

    side_strip = model.part("side_strip")
    side_strip.visual(
        Box((0.028, 0.005, 0.050)),
        material=plastic_dark,
        name="strip_body",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=plastic_dark,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.016, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=plastic_black,
        name="neck_stalk",
    )
    neck.visual(
        Box((0.008, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=plastic_dark,
        name="tilt_bridge",
    )

    head = model.part("head")
    head.visual(
        Box((0.014, 0.040, 0.016)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=plastic_dark,
        name="hinge_block",
    )
    head.visual(
        Box((0.070, 0.088, 0.046)),
        origin=Origin(xyz=(0.042, 0.0, 0.015)),
        material=plastic_black,
        name="head_shell",
    )
    head.visual(
        Box((0.0025, 0.074, 0.034)),
        origin=Origin(xyz=(0.07825, 0.0, 0.015)),
        material=lens_white,
        name="front_lens",
    )
    head.visual(
        Box((0.050, 0.080, 0.006)),
        origin=Origin(xyz=(0.044, 0.0, 0.041)),
        material=plastic_dark,
        name="top_cap",
    )

    for index, z_pos in enumerate((0.014, 0.0, -0.014)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.012, 0.0028, 0.010)),
            material=button_gray,
            name="button_cap",
        )
        model.articulation(
            f"side_strip_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=side_strip,
            child=button,
            origin=Origin(xyz=(0.0, 0.0039, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.02,
                lower=0.0,
                upper=0.001,
            ),
        )

    model.articulation(
        "body_to_side_strip",
        ArticulationType.FIXED,
        parent=body,
        child=side_strip,
        origin=Origin(xyz=(0.006, 0.0375, 0.050)),
    )
    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.004, 0.0, 0.091)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.006, 0.0, 0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.43,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    side_strip = object_model.get_part("side_strip")
    head = object_model.get_part("head")
    buttons = [object_model.get_part(f"mode_button_{index}") for index in range(3)]

    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    button_joints = [
        object_model.get_articulation(f"side_strip_to_mode_button_{index}") for index in range(3)
    ]

    ctx.expect_contact(side_strip, body, name="side strip mounts to battery body")
    ctx.expect_gap(head, body, axis="z", min_gap=0.006, name="head sits above body at rest")

    rest_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    if swivel.motion_limits is not None and swivel.motion_limits.upper is not None:
        with ctx.pose({swivel: swivel.motion_limits.upper}):
            swiveled_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
        ctx.check(
            "neck swivel turns the flash head sideways",
            rest_lens is not None
            and swiveled_lens is not None
            and swiveled_lens[1] > rest_lens[1] + 0.045,
            details=f"rest={rest_lens}, swiveled={swiveled_lens}",
        )

    if tilt.motion_limits is not None and tilt.motion_limits.upper is not None:
        with ctx.pose({tilt: tilt.motion_limits.upper}):
            tilted_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
        ctx.check(
            "head tilt lifts the lens upward",
            rest_lens is not None
            and tilted_lens is not None
            and tilted_lens[2] > rest_lens[2] + 0.030,
            details=f"rest={rest_lens}, tilted={tilted_lens}",
        )

    rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        upper = 0.0
        if joint.motion_limits is not None and joint.motion_limits.upper is not None:
            upper = joint.motion_limits.upper

        with ctx.pose({joint: upper}):
            pressed_position = ctx.part_world_position(button)
            other_positions = [ctx.part_world_position(other) for other in buttons]

        ctx.check(
            f"mode button {index} depresses inward",
            rest_positions[index] is not None
            and pressed_position is not None
            and pressed_position[1] < rest_positions[index][1] - 0.0008,
            details=f"rest={rest_positions[index]}, pressed={pressed_position}",
        )
        ctx.check(
            f"mode button {index} moves independently",
            all(
                other_positions[other_index] is not None
                and rest_positions[other_index] is not None
                and (
                    other_index == index
                    or abs(other_positions[other_index][1] - rest_positions[other_index][1]) < 1e-9
                )
                for other_index in range(len(buttons))
            ),
            details=f"rest={rest_positions}, posed={other_positions}",
        )

    return ctx.report()


object_model = build_object_model()
