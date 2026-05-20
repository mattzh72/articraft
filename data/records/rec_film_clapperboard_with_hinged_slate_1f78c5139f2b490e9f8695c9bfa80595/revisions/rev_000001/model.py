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
    model = ArticulatedObject(name="television_slate")

    frame_black = model.material("frame_black", rgba=(0.09, 0.09, 0.10, 1.0))
    board_white = model.material("board_white", rgba=(0.96, 0.97, 0.95, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.98, 0.98, 0.96, 1.0))
    strip_dark = model.material("strip_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    button_grey = model.material("button_grey", rgba=(0.33, 0.35, 0.38, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    board_width = 0.340
    board_height = 0.240
    board_thickness = 0.012
    white_face_width = 0.298
    white_face_height = 0.168
    white_face_thickness = 0.0015
    strip_width = 0.270
    strip_depth = 0.008
    strip_height = 0.028
    strip_center_z = 0.074
    hinge_axis_z = 0.129

    body = model.part("body")
    body.visual(
        Box((board_width, board_thickness, board_height)),
        material=frame_black,
        name="board_shell",
    )
    body.visual(
        Box((white_face_width, white_face_thickness, white_face_height)),
        origin=Origin(xyz=(0.0, 0.0066, -0.026)),
        material=board_white,
        name="white_face",
    )
    body.visual(
        Box((strip_width, strip_depth, strip_height)),
        origin=Origin(xyz=(0.0, 0.010, strip_center_z)),
        material=strip_dark,
        name="electronics_strip",
    )
    body.visual(
        Box((0.330, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, 0.117)),
        material=hinge_dark,
        name="hinge_leaf",
    )
    for cheek_index, cheek_x in enumerate((-0.156, 0.156)):
        body.visual(
            Box((0.016, 0.010, 0.018)),
            origin=Origin(xyz=(cheek_x, -0.001, 0.111)),
            material=hinge_dark,
            name=f"hinge_cheek_{cheek_index}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Cylinder(radius=0.005, length=0.338),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="hinge_barrel",
    )
    clapstick.visual(
        Box((0.336, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.005, -0.006)),
        material=frame_black,
        name="hinge_web",
    )
    clapstick.visual(
        Box((0.348, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.016, -0.017)),
        material=frame_black,
        name="stick_body",
    )
    stripe_x_positions = (-0.132, -0.082, -0.032, 0.018, 0.068, 0.118)
    for stripe_index, stripe_x in enumerate(stripe_x_positions):
        clapstick.visual(
            Box((0.058, 0.0016, 0.008)),
            origin=Origin(xyz=(stripe_x, 0.0244, -0.017), rpy=(0.0, 0.58, 0.0)),
            material=stripe_white,
            name=f"stripe_{stripe_index}",
        )

    button_specs = (
        ("button_0", -0.040),
        ("button_1", 0.040),
    )
    for part_name, x_pos in button_specs:
        button = model.part(part_name)
        button.visual(
            Box((0.026, 0.003, 0.012)),
            material=button_grey,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0155, strip_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0012,
            ),
        )

    model.articulation(
        "body_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    clapstick = object_model.get_part("clapstick")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    clap_hinge = object_model.get_articulation("body_to_clapstick")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")

    ctx.expect_overlap(
        clapstick,
        body,
        axes="x",
        elem_a="stick_body",
        elem_b="board_shell",
        min_overlap=0.330,
        name="clapstick spans the slate width",
    )
    ctx.expect_gap(
        clapstick,
        body,
        axis="z",
        positive_elem="stick_body",
        negative_elem="electronics_strip",
        min_gap=0.010,
        max_gap=0.020,
        name="closed clapstick sits just above the electronics strip",
    )

    ctx.expect_overlap(
        button_0,
        body,
        axes="xz",
        elem_a="button_cap",
        elem_b="electronics_strip",
        min_overlap=0.010,
        name="button 0 is mounted within the strip footprint",
    )
    ctx.expect_gap(
        button_0,
        body,
        axis="y",
        positive_elem="button_cap",
        negative_elem="electronics_strip",
        max_gap=0.0002,
        max_penetration=0.0,
        name="button 0 sits against the strip face",
    )
    ctx.expect_overlap(
        button_1,
        body,
        axes="xz",
        elem_a="button_cap",
        elem_b="electronics_strip",
        min_overlap=0.010,
        name="button 1 is mounted within the strip footprint",
    )
    ctx.expect_gap(
        button_1,
        body,
        axis="y",
        positive_elem="button_cap",
        negative_elem="electronics_strip",
        max_gap=0.0002,
        max_penetration=0.0,
        name="button 1 sits against the strip face",
    )

    clap_limits = clap_hinge.motion_limits
    if clap_limits is not None and clap_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
        with ctx.pose({clap_hinge: clap_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
        ctx.check(
            "clapstick opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.025
            and open_aabb[1][1] > closed_aabb[1][1] + 0.005,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)

    button_0_limits = button_0_joint.motion_limits
    if button_0_limits is not None and button_0_limits.upper is not None:
        with ctx.pose({button_0_joint: button_0_limits.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_while_button_0_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "button 0 presses inward",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_0_pressed[1] < button_0_rest[1] - 0.0010,
            details=f"rest={button_0_rest}, pressed={button_0_pressed}",
        )
        ctx.check(
            "button 1 stays still when button 0 is pressed",
            button_1_rest is not None
            and button_1_while_button_0_pressed is not None
            and abs(button_1_while_button_0_pressed[1] - button_1_rest[1]) <= 1e-7,
            details=f"rest={button_1_rest}, during_button_0={button_1_while_button_0_pressed}",
        )

    button_1_limits = button_1_joint.motion_limits
    if button_1_limits is not None and button_1_limits.upper is not None:
        with ctx.pose({button_1_joint: button_1_limits.upper}):
            button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "button 1 presses inward",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_1_pressed[1] < button_1_rest[1] - 0.0010,
            details=f"rest={button_1_rest}, pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
