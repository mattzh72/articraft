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
    model = ArticulatedObject(name="digital_camera_slate")

    board_black = model.material("board_black", rgba=(0.08, 0.09, 0.10, 1.0))
    header_black = model.material("header_black", rgba=(0.14, 0.15, 0.16, 1.0))
    clap_white = model.material("clap_white", rgba=(0.93, 0.93, 0.90, 1.0))
    stripe_black = model.material("stripe_black", rgba=(0.03, 0.03, 0.03, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    display_red = model.material("display_red", rgba=(0.58, 0.07, 0.06, 1.0))
    button_gray = model.material("button_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.64, 0.66, 1.0))

    board_w = 0.29
    board_t = 0.008
    board_h = 0.21

    board = model.part("board")
    board.visual(
        Box((board_w, board_t, board_h)),
        material=board_black,
        name="panel",
    )
    board.visual(
        Box((board_w, board_t + 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, board_h / 2 - 0.009)),
        material=header_black,
        name="header",
    )
    board.visual(
        Cylinder(radius=0.0035, length=board_w * 0.86),
        origin=Origin(
            xyz=(0.0, board_t / 2 + 0.0015, board_h / 2 - 0.006),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=metal,
        name="hinge_rod",
    )

    display_module = model.part("display_module")
    display_module.visual(
        Box((0.13, 0.016, 0.092)),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=housing_gray,
        name="housing",
    )
    display_module.visual(
        Box((0.096, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, 0.0175, 0.020)),
        material=bezel_black,
        name="display_bezel",
    )
    display_module.visual(
        Box((0.080, 0.0012, 0.024)),
        origin=Origin(xyz=(0.0, 0.0196, 0.020)),
        material=display_red,
        name="display_window",
    )
    display_module.visual(
        Box((0.110, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, 0.0175, -0.028)),
        material=bezel_black,
        name="button_rail",
    )
    model.articulation(
        "board_to_display_module",
        ArticulationType.FIXED,
        parent=board,
        child=display_module,
        origin=Origin(xyz=(0.0, board_t / 2, 0.016)),
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((0.30, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.005, -0.020)),
        material=clap_white,
        name="bar",
    )
    clapstick.visual(
        Box((0.255, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.003, -0.004)),
        material=metal,
        name="hinge_leaf",
    )
    for index, x_pos in enumerate((-0.110, -0.055, 0.0, 0.055, 0.110)):
        clapstick.visual(
            Box((0.036, 0.0105, 0.040)),
            origin=Origin(xyz=(x_pos, 0.005, -0.020)),
            material=stripe_black,
            name=f"stripe_{index}",
        )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, board_t / 2 + 0.0015, board_h / 2 - 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=1.30,
        ),
    )

    button_x_positions = (-0.039, -0.013, 0.013, 0.039)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.016, 0.0024, 0.010)),
            origin=Origin(xyz=(0.0, 0.0012, 0.0)),
            material=button_gray,
            name="cap",
        )
        model.articulation(
            f"display_module_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display_module,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0190, -0.028)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0006,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    display_module = object_model.get_part("display_module")
    clapstick = object_model.get_part("clapstick")
    buttons = [object_model.get_part(f"button_{index}") for index in range(4)]

    clap_joint = object_model.get_articulation("board_to_clapstick")
    button_joints = [
        object_model.get_articulation(f"display_module_to_button_{index}")
        for index in range(4)
    ]

    with ctx.pose({clap_joint: 0.0}):
        ctx.expect_overlap(
            clapstick,
            board,
            axes="xz",
            elem_a="bar",
            elem_b="panel",
            min_overlap=0.03,
            name="closed clapstick spans the slate face",
        )
        ctx.expect_gap(
            clapstick,
            board,
            axis="y",
            positive_elem="bar",
            negative_elem="panel",
            min_gap=0.001,
            max_gap=0.003,
            name="closed clapstick sits just proud of the board",
        )

    ctx.expect_gap(
        display_module,
        board,
        axis="y",
        positive_elem="display_window",
        negative_elem="panel",
        min_gap=0.014,
        name="timecode window stands proud of the slate face",
    )
    ctx.expect_gap(
        buttons[0],
        board,
        axis="y",
        positive_elem="cap",
        negative_elem="panel",
        min_gap=0.015,
        name="button bank stands proud of the slate face",
    )
    ctx.expect_gap(
        display_module,
        buttons[0],
        axis="z",
        positive_elem="display_window",
        negative_elem="cap",
        min_gap=0.020,
        name="button row sits below the display window",
    )

    clap_limits = clap_joint.motion_limits
    if clap_limits is not None and clap_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(clapstick)
        with ctx.pose({clap_joint: clap_limits.upper}):
            open_aabb = ctx.part_world_aabb(clapstick)
        ctx.check(
            "clapstick opens outward on the top hinge",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > rest_aabb[1][1] + 0.030
            and open_aabb[0][2] > rest_aabb[0][2] + 0.020,
            details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
        )

    for button, joint in zip(buttons, button_joints):
        limits = joint.motion_limits
        rest_pos = ctx.part_world_position(button)
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"{button.name} depresses independently",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] < rest_pos[1] - 0.0005,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    reference_rest = ctx.part_world_position(buttons[1])
    first_button_limits = button_joints[0].motion_limits
    if first_button_limits is not None and first_button_limits.upper is not None:
        with ctx.pose({button_joints[0]: first_button_limits.upper}):
            reference_pressed = ctx.part_world_position(buttons[1])
        ctx.check(
            "buttons move independently",
            reference_rest is not None
            and reference_pressed is not None
            and abs(reference_pressed[1] - reference_rest[1]) < 1e-7,
            details=f"rest={reference_rest}, pressed={reference_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
