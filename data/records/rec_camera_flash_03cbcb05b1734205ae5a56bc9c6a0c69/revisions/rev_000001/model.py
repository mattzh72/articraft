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
    model = ArticulatedObject(name="camera_flash")

    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.18, 0.28, 0.34, 1.0))
    diffuser = model.material("diffuser", rgba=(0.92, 0.93, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.048, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="foot_plate",
    )
    body.visual(
        Box((0.024, 0.016, 0.006)),
        origin=Origin(xyz=(0.004, 0.0, 0.009)),
        material=trim_gray,
        name="shoe_block",
    )
    body.visual(
        Box((0.020, 0.022, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.017)),
        material=trim_gray,
        name="stem",
    )
    body.visual(
        Box((0.028, 0.050, 0.020)),
        origin=Origin(xyz=(-0.002, 0.0, 0.032)),
        material=body_black,
        name="lower_body",
    )
    body.visual(
        Box((0.036, 0.064, 0.058)),
        origin=Origin(xyz=(-0.002, 0.0, 0.071)),
        material=body_black,
        name="upper_body",
    )
    body.visual(
        Box((0.030, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=body_black,
        name="top_collar",
    )
    body.visual(
        Box((0.002, 0.030, 0.020)),
        origin=Origin(xyz=(-0.020, 0.0, 0.075)),
        material=screen_glass,
        name="screen",
    )
    body.visual(
        Box((0.003, 0.020, 0.016)),
        origin=Origin(xyz=(-0.0195, 0.0, 0.049)),
        material=rubber,
        name="keypad",
    )
    body.visual(
        Box((0.003, 0.008, 0.008)),
        origin=Origin(xyz=(-0.0195, 0.014, 0.039)),
        material=rubber,
        name="button_0",
    )
    body.visual(
        Box((0.003, 0.008, 0.008)),
        origin=Origin(xyz=(-0.0195, -0.014, 0.039)),
        material=rubber,
        name="button_1",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(),
        material=trim_gray,
        name="turntable",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_gray,
        name="swivel_hub",
    )
    yoke.visual(
        Box((0.016, 0.018, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.007)),
        material=trim_gray,
        name="yoke_base",
    )
    yoke.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(0.004, 0.0415, 0.015)),
        material=trim_gray,
        name="yoke_arm_0",
    )
    yoke.visual(
        Box((0.012, 0.036, 0.008)),
        origin=Origin(xyz=(0.004, 0.024, 0.007)),
        material=trim_gray,
        name="yoke_shoulder_0",
    )
    yoke.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(0.004, -0.0415, 0.015)),
        material=trim_gray,
        name="yoke_arm_1",
    )
    yoke.visual(
        Box((0.012, 0.036, 0.008)),
        origin=Origin(xyz=(0.004, -0.024, 0.007)),
        material=trim_gray,
        name="yoke_shoulder_1",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.006, length=0.0045),
        origin=Origin(xyz=(0.0, 0.03625, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.0045),
        origin=Origin(xyz=(0.0, -0.03625, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="trunnion_1",
    )
    head.visual(
        Box((0.050, 0.068, 0.040)),
        origin=Origin(xyz=(0.026, 0.0, 0.023)),
        material=body_black,
        name="head_shell",
    )
    head.visual(
        Box((0.016, 0.058, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.038)),
        material=body_black,
        name="top_cap",
    )
    head.visual(
        Box((0.004, 0.060, 0.032)),
        origin=Origin(xyz=(0.053, 0.0, 0.023)),
        material=diffuser,
        name="lens",
    )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.004, 0.0, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-0.15,
            upper=1.75,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")

    pitch_limits = pitch.motion_limits
    swivel_limits = swivel.motion_limits

    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.018,
        name="head rests clearly above the control body",
    )
    ctx.expect_within(
        head,
        yoke,
        axes="y",
        margin=0.007,
        name="head stays between the side yokes in width",
    )

    if pitch_limits is not None and pitch_limits.upper is not None:
        rest_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
        with ctx.pose({pitch: pitch_limits.upper}):
            raised_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
            ctx.expect_gap(
                head,
                body,
                axis="z",
                min_gap=0.010,
                name="pitched head still clears the body",
            )
        ctx.check(
            "positive pitch lifts the flash head",
            rest_lens_center is not None
            and raised_lens_center is not None
            and raised_lens_center[2] > rest_lens_center[2] + 0.018,
            details=f"rest={rest_lens_center}, raised={raised_lens_center}",
        )

    if swivel_limits is not None:
        rest_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
        with ctx.pose({swivel: math.pi / 2.0}):
            swiveled_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
            ctx.expect_gap(
                head,
                body,
                axis="z",
                min_gap=0.010,
                name="swiveled head remains above the body",
            )
        ctx.check(
            "positive swivel turns the head to the side",
            rest_lens_center is not None
            and swiveled_lens_center is not None
            and swiveled_lens_center[1] > rest_lens_center[1] + 0.025,
            details=f"rest={rest_lens_center}, swiveled={swiveled_lens_center}",
        )

    return ctx.report()


object_model = build_object_model()
