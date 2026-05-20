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


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_flash")

    body_black = model.material("body_black", rgba=(0.14, 0.15, 0.17, 1.0))
    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    lens_white = model.material("lens_white", rgba=(0.92, 0.93, 0.92, 1.0))
    screen_grey = model.material("screen_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    button_grey = model.material("button_grey", rgba=(0.50, 0.51, 0.54, 1.0))
    metal = model.material("metal", rgba=(0.63, 0.65, 0.69, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.074, 0.046, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=body_black,
        name="foot_plate",
    )
    body.visual(
        Box((0.052, 0.022, 0.008)),
        origin=Origin(xyz=(0.002, 0.000, 0.014)),
        material=metal,
        name="shoe_rail",
    )
    body.visual(
        Box((0.018, 0.032, 0.012)),
        origin=Origin(xyz=(-0.006, 0.000, 0.022)),
        material=body_dark,
        name="foot_post",
    )
    body.visual(
        Box((0.070, 0.062, 0.026)),
        origin=Origin(xyz=(-0.004, 0.000, 0.023)),
        material=body_black,
        name="lower_body",
    )
    body.visual(
        Box((0.060, 0.056, 0.074)),
        origin=Origin(xyz=(-0.006, 0.000, 0.073)),
        material=body_black,
        name="body_shell",
    )
    body.visual(
        Box((0.050, 0.048, 0.012)),
        origin=Origin(xyz=(-0.002, 0.000, 0.116)),
        material=body_dark,
        name="upper_shoulder",
    )
    body.visual(
        Box((0.026, 0.040, 0.022)),
        origin=Origin(xyz=(0.015, 0.000, 0.091)),
        material=body_dark,
        name="front_neck",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.122)),
        material=body_dark,
        name="swivel_socket",
    )
    body.visual(
        Box((0.002, 0.034, 0.046)),
        origin=Origin(xyz=(-0.035, 0.000, 0.075)),
        material=screen_grey,
        name="display_panel",
    )
    body.visual(
        Box((0.026, 0.034, 0.004)),
        origin=Origin(xyz=(-0.026, 0.000, 0.041)),
        material=rubber,
        name="rear_pad",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=body_dark,
        name="collar_ring",
    )
    swivel.visual(
        Box((0.024, 0.040, 0.032)),
        origin=Origin(xyz=(0.004, 0.000, 0.026)),
        material=body_black,
        name="neck_core",
    )
    swivel.visual(
        Box((0.014, 0.054, 0.018)),
        origin=Origin(xyz=(0.006, 0.000, 0.047)),
        material=body_black,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.018, 0.012, 0.026)),
        origin=Origin(xyz=(0.012, 0.032, 0.056)),
        material=body_black,
        name="arm_web_0",
    )
    swivel.visual(
        Box((0.018, 0.012, 0.026)),
        origin=Origin(xyz=(0.012, -0.032, 0.056)),
        material=body_black,
        name="arm_web_1",
    )
    swivel.visual(
        Box((0.028, 0.010, 0.032)),
        origin=Origin(xyz=(0.020, 0.041, 0.068)),
        material=body_black,
        name="yoke_arm_0",
    )
    swivel.visual(
        Box((0.028, 0.010, 0.032)),
        origin=Origin(xyz=(0.020, -0.041, 0.068)),
        material=body_black,
        name="yoke_arm_1",
    )
    swivel.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.020, 0.041, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="pivot_boss_0",
    )
    swivel.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.020, -0.041, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="pivot_boss_1",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.020, 0.008, 0.052)),
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
        material=body_dark,
        name="strip_plate",
    )

    for index, local_z in enumerate((0.014, 0.026, 0.038)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.011, 0.005, 0.010)),
            origin=Origin(xyz=(0.000, 0.0025, 0.000)),
            material=button_grey,
            name="button_cap",
        )
        model.articulation(
            f"control_strip_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(0.000, 0.004, local_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.030,
                lower=0.0,
                upper=0.0015,
            ),
        )

    head = model.part("head")
    head.visual(
        Box((0.072, 0.060, 0.050)),
        origin=Origin(xyz=(0.026, 0.000, 0.000)),
        material=body_black,
        name="head_shell",
    )
    head.visual(
        Box((0.006, 0.054, 0.036)),
        origin=Origin(xyz=(0.059, 0.000, 0.000)),
        material=body_dark,
        name="front_frame",
    )
    head.visual(
        Box((0.004, 0.048, 0.030)),
        origin=Origin(xyz=(0.064, 0.000, 0.000)),
        material=lens_white,
        name="lens_panel",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.072),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="pivot_shaft",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.000, 0.000, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(0.016, 0.032, 0.050)),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.020, 0.000, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-0.20,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    control_strip = object_model.get_part("control_strip")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    pitch_joint = object_model.get_articulation("swivel_to_head")

    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.035,
        name="head clears the control body at rest",
    )
    ctx.expect_contact(
        head,
        swivel,
        elem_a="pivot_shaft",
        elem_b="pivot_boss_0",
        name="upper head pivot is supported on one side",
    )
    ctx.expect_contact(
        head,
        swivel,
        elem_a="pivot_shaft",
        elem_b="pivot_boss_1",
        name="upper head pivot is supported on the other side",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens_panel"))
    with ctx.pose({swivel_joint: 1.2}):
        turned_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens_panel"))
    ctx.check(
        "head swivels around the vertical axis",
        rest_lens_center is not None
        and turned_lens_center is not None
        and turned_lens_center[1] > rest_lens_center[1] + 0.050,
        details=f"rest={rest_lens_center}, turned={turned_lens_center}",
    )

    with ctx.pose({pitch_joint: 1.2}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens_panel"))
    ctx.check(
        "head pitches upward between the yoke arms",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.040,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    for index in range(3):
        button = object_model.get_part(f"mode_button_{index}")
        button_joint = object_model.get_articulation(f"control_strip_to_mode_button_{index}")
        limits = button_joint.motion_limits

        ctx.expect_contact(
            button,
            control_strip,
            elem_a="button_cap",
            elem_b="strip_plate",
            name=f"mode button {index} seats on the side strip",
        )

        rest_position = ctx.part_world_position(button)
        other_rest_positions = [
            ctx.part_world_position(object_model.get_part(f"mode_button_{other_index}"))
            for other_index in range(3)
            if other_index != index
        ]
        with ctx.pose({button_joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            pressed_position = ctx.part_world_position(button)
            other_pressed_positions = [
                ctx.part_world_position(object_model.get_part(f"mode_button_{other_index}"))
                for other_index in range(3)
                if other_index != index
            ]

        neighbors_still = all(
            rest is not None
            and pressed is not None
            and abs(pressed[0] - rest[0]) < 1e-9
            and abs(pressed[1] - rest[1]) < 1e-9
            and abs(pressed[2] - rest[2]) < 1e-9
            for rest, pressed in zip(other_rest_positions, other_pressed_positions)
        )
        ctx.check(
            f"mode button {index} depresses independently",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] < rest_position[1] - 0.001
            and neighbors_still,
            details=(
                f"rest={rest_position}, pressed={pressed_position}, "
                f"other_rest={other_rest_positions}, other_pressed={other_pressed_positions}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
