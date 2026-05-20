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
    model = ArticulatedObject(name="hot_shoe_flash")

    shell_dark = model.material("shell_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.91, 0.93, 0.95, 0.92))
    screen_glass = model.material("screen_glass", rgba=(0.15, 0.28, 0.34, 0.55))
    metal_dark = model.material("metal_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.018, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal_dark,
        name="shoe_foot",
    )
    body.visual(
        Box((0.010, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=metal_dark,
        name="shoe_stem",
    )
    body.visual(
        Box((0.066, 0.038, 0.094)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=shell_dark,
        name="body_shell",
    )
    body.visual(
        Box((0.044, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=shell_mid,
        name="body_cap",
    )
    body.visual(
        Box((0.034, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.018, 0.078)),
        material=shell_mid,
        name="screen_bezel",
    )
    body.visual(
        Box((0.028, 0.002, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, 0.078)),
        material=screen_glass,
        name="screen",
    )
    body.visual(
        Box((0.018, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, 0.054)),
        material=screen_glass,
        name="sensor_window",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=shell_mid,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.018, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.006, 0.022)),
        material=shell_mid,
        name="neck_stem",
    )
    neck.visual(
        Box((0.098, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.041)),
        material=shell_mid,
        name="yoke_bridge",
    )
    for index, x_pos in enumerate((-0.044, 0.044)):
        neck.visual(
            Box((0.010, 0.018, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, 0.024)),
            material=shell_mid,
            name=f"yoke_arm_{index}",
        )

    head = model.part("head")
    head.visual(
        Box((0.070, 0.036, 0.046)),
        origin=Origin(xyz=(0.0, 0.021, 0.019)),
        material=shell_mid,
        name="head_shell",
    )
    head.visual(
        Box((0.060, 0.002, 0.034)),
        origin=Origin(xyz=(0.0, 0.039, 0.020)),
        material=diffuser_white,
        name="front_window",
    )
    head.visual(
        Box((0.026, 0.002, 0.008)),
        origin=Origin(xyz=(0.0, 0.0395, 0.038)),
        material=screen_glass,
        name="pilot_window",
    )
    for index, x_pos in enumerate((-0.028, 0.028)):
        head.visual(
            Box((0.012, 0.012, 0.012)),
            origin=Origin(xyz=(x_pos, 0.0, 0.003)),
            material=shell_mid,
            name=f"pivot_ear_{index}",
        )
    for index, x_pos in enumerate((-0.0365, 0.0365)):
        head.visual(
            Cylinder(radius=0.005, length=0.005),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name=f"pivot_pin_{index}",
        )

    command_wheel = model.part("command_wheel")
    command_wheel.visual(
        Cylinder(radius=0.0125, length=0.004),
        material=rubber_black,
        name="wheel_rim",
    )
    command_wheel.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=shell_mid,
        name="wheel_hub",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.25,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_command_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=command_wheel,
        origin=Origin(xyz=(0.0, -0.021, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("head")
    command_wheel = object_model.get_part("command_wheel")
    neck_swivel = object_model.get_articulation("body_to_neck")
    head_tilt = object_model.get_articulation("neck_to_head")
    wheel_joint = object_model.get_articulation("body_to_command_wheel")

    ctx.expect_gap(
        head,
        body,
        axis="z",
        positive_elem="head_shell",
        negative_elem="body_cap",
        min_gap=0.015,
        max_gap=0.030,
        name="head stays visibly above the battery body",
    )
    ctx.expect_gap(
        body,
        command_wheel,
        axis="y",
        positive_elem="body_shell",
        negative_elem="wheel_rim",
        max_gap=0.0005,
        max_penetration=0.0,
        name="command wheel seats against the rear body surface",
    )
    ctx.expect_overlap(
        command_wheel,
        body,
        axes="xz",
        elem_a="wheel_rim",
        elem_b="body_shell",
        min_overlap=0.010,
        name="command wheel sits centered on the rear body footprint",
    )

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    screen_aabb = ctx.part_element_world_aabb(body, elem="screen")
    wheel_aabb = ctx.part_element_world_aabb(command_wheel, elem="wheel_rim")
    wheel_center = _center(wheel_aabb)
    screen_center = _center(screen_aabb)
    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "command wheel sits below the rear screen",
        screen_center is not None and wheel_center is not None and screen_center[2] > wheel_center[2] + 0.015,
        details=f"screen_center={screen_center}, wheel_center={wheel_center}",
    )
    ctx.check(
        "command wheel is a proud raised control",
        body_aabb is not None and wheel_aabb is not None and wheel_aabb[0][1] < body_aabb[0][1] - 0.0015,
        details=f"body_aabb={body_aabb}, wheel_aabb={wheel_aabb}",
    )

    rest_front = _center(ctx.part_element_world_aabb(head, elem="front_window"))
    tilt_upper = head_tilt.motion_limits.upper if head_tilt.motion_limits is not None else None
    if tilt_upper is not None:
        with ctx.pose({head_tilt: tilt_upper}):
            tilted_front = _center(ctx.part_element_world_aabb(head, elem="front_window"))
        ctx.check(
            "flash head tilts upward at positive rotation",
            rest_front is not None and tilted_front is not None and tilted_front[2] > rest_front[2] + 0.015,
            details=f"rest_front={rest_front}, tilted_front={tilted_front}",
        )

    with ctx.pose({neck_swivel: math.pi / 2.0}):
        swiveled_front = _center(ctx.part_element_world_aabb(head, elem="front_window"))
    ctx.check(
        "neck swivel yaws the head around the vertical axis",
        rest_front is not None and swiveled_front is not None and abs(swiveled_front[0]) > 0.020,
        details=f"rest_front={rest_front}, swiveled_front={swiveled_front}",
    )
    ctx.check(
        "command wheel uses a continuous joint",
        getattr(wheel_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"articulation_type={getattr(wheel_joint, 'articulation_type', None)}",
    )

    return ctx.report()


object_model = build_object_model()
