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


def _rotated_local_origin(
    center_xyz: tuple[float, float, float],
    local_xyz: tuple[float, float, float],
    pitch: float,
) -> Origin:
    cx, cy, cz = center_xyz
    lx, ly, lz = local_xyz
    c = math.cos(pitch)
    s = math.sin(pitch)
    dx = c * lx + s * lz
    dz = -s * lx + c * lz
    return Origin(xyz=(cx + dx, cy + ly, cz + dz), rpy=(0.0, pitch, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_cash_register")

    body_beige = model.material("body_beige", rgba=(0.83, 0.81, 0.73, 1.0))
    trim_beige = model.material("trim_beige", rgba=(0.74, 0.72, 0.66, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    key_dark = model.material("key_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    key_light = model.material("key_light", rgba=(0.70, 0.66, 0.58, 1.0))
    display_glass = model.material("display_glass", rgba=(0.19, 0.43, 0.32, 0.65))

    body = model.part("body")
    body.visual(
        Box((0.380, 0.420, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=trim_beige,
        name="base_plate",
    )
    body.visual(
        Box((0.360, 0.018, 0.130)),
        origin=Origin(xyz=(0.000, 0.201, 0.065)),
        material=body_beige,
        name="side_wall_0",
    )
    body.visual(
        Box((0.360, 0.018, 0.130)),
        origin=Origin(xyz=(0.000, -0.201, 0.065)),
        material=body_beige,
        name="side_wall_1",
    )
    body.visual(
        Box((0.018, 0.384, 0.130)),
        origin=Origin(xyz=(-0.181, 0.000, 0.065)),
        material=body_beige,
        name="rear_wall",
    )
    body.visual(
        Box((0.380, 0.384, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.124)),
        material=body_beige,
        name="top_slab",
    )
    body.visual(
        Box((0.018, 0.384, 0.036)),
        origin=Origin(xyz=(0.181, 0.000, 0.106)),
        material=body_beige,
        name="front_fascia",
    )
    body.visual(
        Box((0.200, 0.340, 0.082)),
        origin=Origin(xyz=(-0.060, 0.000, 0.165)),
        material=body_beige,
        name="upper_body",
    )
    body.visual(
        Box((0.100, 0.120, 0.050)),
        origin=Origin(xyz=(-0.132, 0.000, 0.231)),
        material=trim_beige,
        name="display_plinth",
    )
    body.visual(
        Box((0.080, 0.180, 0.016)),
        origin=Origin(xyz=(-0.078, 0.000, 0.144)),
        material=charcoal,
        name="operator_window",
    )

    keypad_center = (0.020, 0.000, 0.212)
    keypad_pitch = math.radians(24.0)
    body.visual(
        Box((0.170, 0.290, 0.010)),
        origin=Origin(xyz=keypad_center, rpy=(0.0, keypad_pitch, 0.0)),
        material=trim_beige,
        name="keypad_panel",
    )

    key_positions = (
        (-0.042, -0.080),
        (-0.042, -0.030),
        (-0.042, 0.020),
        (-0.042, 0.070),
        (-0.012, -0.095),
        (-0.012, -0.045),
        (-0.012, 0.005),
        (-0.012, 0.055),
        (0.018, -0.080),
        (0.018, -0.030),
        (0.018, 0.020),
        (0.018, 0.070),
        (0.048, -0.095),
        (0.048, -0.045),
        (0.048, 0.005),
        (0.048, 0.055),
    )
    for index, (kx, ky) in enumerate(key_positions):
        key_material = key_dark if index % 3 else key_light
        body.visual(
            Box((0.018, 0.020, 0.008)),
            origin=_rotated_local_origin(keypad_center, (kx, ky, 0.008), keypad_pitch),
            material=key_material,
            name=f"key_{index}",
        )

    function_strip_y = (-0.094, -0.052, -0.010, 0.032, 0.074)
    for index, ky in enumerate(function_strip_y):
        body.visual(
            Box((0.016, 0.024, 0.006)),
            origin=_rotated_local_origin(keypad_center, (0.075, ky, 0.007), keypad_pitch),
            material=charcoal,
            name=f"function_key_{index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.335, 0.370, 0.075)),
        origin=Origin(xyz=(0.1675, 0.000, 0.0375)),
        material=drawer_gray,
        name="drawer_box",
    )
    drawer.visual(
        Box((0.020, 0.390, 0.086)),
        origin=Origin(xyz=(0.345, 0.000, 0.043)),
        material=drawer_gray,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.160, 0.016)),
        origin=Origin(xyz=(0.351, 0.000, 0.052)),
        material=charcoal,
        name="handle_bar",
    )
    drawer.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.351, 0.000, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="lock_cylinder",
    )

    display_post = model.part("display_post")
    display_post.visual(
        Box((0.070, 0.100, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=trim_beige,
        name="post_base",
    )
    display_post.visual(
        Box((0.032, 0.024, 0.270)),
        origin=Origin(xyz=(0.000, 0.000, 0.147)),
        material=charcoal,
        name="post_mast",
    )
    display_post.visual(
        Box((0.010, 0.072, 0.020)),
        origin=Origin(xyz=(0.014, 0.000, 0.272)),
        material=charcoal,
        name="post_yoke",
    )
    display_post.visual(
        Box((0.018, 0.012, 0.040)),
        origin=Origin(xyz=(0.000, 0.040, 0.282)),
        material=charcoal,
        name="post_ear_0",
    )
    display_post.visual(
        Box((0.018, 0.012, 0.040)),
        origin=Origin(xyz=(0.000, -0.040, 0.282)),
        material=charcoal,
        name="post_ear_1",
    )

    display_head = model.part("display_head")
    display_head.visual(
        Box((0.016, 0.064, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=charcoal,
        name="hinge_block",
    )
    display_head.visual(
        Box((0.018, 0.052, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=charcoal,
        name="head_neck",
    )
    display_head.visual(
        Box((0.034, 0.140, 0.082)),
        origin=Origin(xyz=(0.002, 0.000, 0.063)),
        material=charcoal,
        name="head_shell",
    )
    display_head.visual(
        Box((0.002, 0.112, 0.054)),
        origin=Origin(xyz=(-0.016, 0.000, 0.063)),
        material=display_glass,
        name="screen",
    )
    display_head.visual(
        Box((0.042, 0.140, 0.014)),
        origin=Origin(xyz=(0.004, 0.000, 0.102)),
        material=charcoal,
        name="sun_hood",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.145, 0.000, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=0.150,
        ),
    )
    model.articulation(
        "body_to_display_post",
        ArticulationType.FIXED,
        parent=body,
        child=display_post,
        origin=Origin(xyz=(-0.132, 0.000, 0.256)),
    )
    model.articulation(
        "display_post_to_display_head",
        ArticulationType.REVOLUTE,
        parent=display_post,
        child=display_head,
        origin=Origin(xyz=(0.000, 0.000, 0.291)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=-0.50,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    display_post = object_model.get_part("display_post")
    display_head = object_model.get_part("display_head")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    display_pitch = object_model.get_articulation("display_post_to_display_head")

    ctx.expect_gap(
        drawer,
        body,
        axis="x",
        positive_elem="drawer_front",
        negative_elem="front_fascia",
        max_gap=0.006,
        max_penetration=0.0001,
        name="drawer front sits nearly flush when closed",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        inner_elem="drawer_box",
        margin=0.0,
        name="drawer stays inside cabinet envelope laterally",
    )

    slide_limits = drawer_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: slide_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_box",
                min_overlap=0.140,
                name="drawer retains insertion at full extension",
            )
            open_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends forward",
            closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 0.10,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    ctx.expect_origin_gap(
        display_head,
        display_post,
        axis="z",
        min_gap=0.24,
        max_gap=0.30,
        name="customer display head sits atop the support post",
    )

    pitch_limits = display_pitch.motion_limits
    if pitch_limits is not None and pitch_limits.lower is not None and pitch_limits.upper is not None:
        with ctx.pose({display_pitch: pitch_limits.lower}):
            lower_aabb = ctx.part_world_aabb(display_head)
        with ctx.pose({display_pitch: pitch_limits.upper}):
            upper_aabb = ctx.part_world_aabb(display_head)
        ctx.check(
            "display pitches toward customer with positive motion",
            lower_aabb is not None
            and upper_aabb is not None
            and upper_aabb[0][0] < lower_aabb[0][0] - 0.015,
            details=f"lower={lower_aabb}, upper={upper_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
