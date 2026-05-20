from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _make_head_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .box(0.084, 0.074, 0.052)
        .translate((0.032, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    inner_cavity = cq.Workplane("XY").box(0.074, 0.064, 0.042).translate((0.034, 0.0, -0.002))
    front_window = cq.Workplane("XY").box(0.008, 0.066, 0.042).translate((0.072, 0.0, 0.0))
    top_slot = cq.Workplane("XY").box(0.008, 0.060, 0.010).translate((0.022, 0.0, 0.024))
    return outer.cut(inner_cavity).cut(front_window).cut(top_slot)


def _make_body_housing() -> object:
    lower = (
        cq.Workplane("XY")
        .box(0.040, 0.064, 0.086)
        .translate((-0.004, 0.0, 0.061))
        .edges("|Z")
        .fillet(0.004)
    )
    upper = (
        cq.Workplane("XY")
        .box(0.034, 0.056, 0.016)
        .translate((0.000, 0.0, 0.112))
        .edges("|Z")
        .fillet(0.003)
    )
    return lower.union(upper)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_flash")

    body_black = model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.22, 0.24, 0.55))
    flash_lens = model.material("flash_lens", rgba=(0.90, 0.92, 0.94, 0.85))
    bounce_white = model.material("bounce_white", rgba=(0.97, 0.97, 0.95, 1.0))
    accent = model.material("accent", rgba=(0.30, 0.31, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.056, 0.034, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.004)),
        material=trim_black,
        name="foot_base",
    )
    body.visual(
        Box((0.036, 0.018, 0.006)),
        origin=Origin(xyz=(0.008, 0.0, 0.011)),
        material=accent,
        name="shoe_tongue",
    )
    body.visual(
        Box((0.024, 0.022, 0.006)),
        origin=Origin(xyz=(0.004, 0.0, 0.017)),
        material=trim_black,
        name="foot_stem",
    )
    body.visual(
        mesh_from_cadquery(_make_body_housing(), "body_shell"),
        material=body_black,
        name="body_shell",
    )
    body.visual(
        Box((0.024, 0.046, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, 0.125)),
        material=trim_black,
        name="swivel_pedestal",
    )
    body.visual(
        Box((0.002, 0.034, 0.028)),
        origin=Origin(xyz=(-0.024, 0.0, 0.079)),
        material=dark_glass,
        name="screen",
    )
    body.visual(
        Box((0.002, 0.042, 0.016)),
        origin=Origin(xyz=(-0.024, 0.0, 0.048)),
        material=trim_black,
        name="button_panel",
    )
    for index, button_z in enumerate((0.047, 0.032)):
        for lateral, button_y in enumerate((-0.013, 0.0, 0.013)):
            body.visual(
                Box((0.003, 0.009, 0.009)),
                origin=Origin(xyz=(-0.0225, button_y, button_z)),
                material=accent,
                name=f"button_{index}_{lateral}",
            )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_black,
        name="swivel_collar",
    )
    yoke.visual(
        Box((0.012, 0.020, 0.026)),
        origin=Origin(xyz=(0.000, 0.0, 0.020)),
        material=trim_black,
        name="yoke_post",
    )
    yoke.visual(
        Box((0.016, 0.082, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.033)),
        material=trim_black,
        name="yoke_bridge",
    )
    for index, arm_y in enumerate((-0.041, 0.041)):
        yoke.visual(
            Box((0.022, 0.004, 0.036)),
            origin=Origin(xyz=(0.019, arm_y, 0.054)),
            material=trim_black,
            name=f"arm_{index}",
        )
        yoke.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(
                xyz=(0.030, arm_y, 0.066),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=accent,
            name=f"pivot_cap_{index}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shell(), "head_shell"),
        material=body_black,
        name="head_shell",
    )
    head.visual(
        Box((0.006, 0.064, 0.040)),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=flash_lens,
        name="flash_window",
    )
    for index, boss_y in enumerate((-0.038, 0.038)):
        head.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(
                xyz=(0.000, boss_y, 0.000),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=accent,
            name=f"pivot_boss_{index}",
        )

    bounce_card = model.part("bounce_card")
    bounce_card.visual(
        Box((0.0022, 0.056, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.0165)),
        material=bounce_white,
        name="card_plate",
    )
    bounce_card.visual(
        Box((0.004, 0.018, 0.004)),
        origin=Origin(xyz=(-0.0008, 0.0, 0.005)),
        material=bounce_white,
        name="card_tab",
    )
    for index, guide_y in enumerate((-0.028, 0.028)):
        bounce_card.visual(
            Box((0.005, 0.004, 0.012)),
            origin=Origin(xyz=(0.0, guide_y, -0.002)),
            material=bounce_white,
            name=f"card_guide_{index}",
        )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.006, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.030, 0.0, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.12,
            upper=1.45,
        ),
    )
    model.articulation(
        "head_to_bounce_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=bounce_card,
        origin=Origin(xyz=(0.022, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.05,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("head")
    bounce_card = object_model.get_part("bounce_card")

    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")
    card_slide = object_model.get_articulation("head_to_bounce_card")

    ctx.expect_origin_gap(
        head,
        body,
        axis="z",
        min_gap=0.10,
        name="head assembly sits above the control body",
    )

    ctx.expect_within(
        bounce_card,
        head,
        axes="xy",
        inner_elem="card_plate",
        outer_elem="head_shell",
        margin=0.001,
        name="retracted bounce card stays inside the head shell footprint",
    )

    swivel_limits = swivel.motion_limits
    if swivel_limits is not None and swivel_limits.upper is not None:
        rest_head_pos = ctx.part_world_position(head)
        with ctx.pose({swivel: swivel_limits.upper}):
            swivel_head_pos = ctx.part_world_position(head)
        ctx.check(
            "swivel joint turns the head assembly around the body",
            rest_head_pos is not None
            and swivel_head_pos is not None
            and swivel_head_pos[1] > rest_head_pos[1] + 0.02,
            details=f"rest={rest_head_pos}, swiveled={swivel_head_pos}",
        )

    pitch_limits = pitch.motion_limits
    if pitch_limits is not None and pitch_limits.upper is not None:
        rest_shell_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
        with ctx.pose({pitch: pitch_limits.upper}):
            pitched_shell_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
        ctx.check(
            "pitch joint raises the flash head",
            rest_shell_aabb is not None
            and pitched_shell_aabb is not None
            and pitched_shell_aabb[1][2] > rest_shell_aabb[1][2] + 0.03,
            details=f"rest={rest_shell_aabb}, pitched={pitched_shell_aabb}",
        )

    card_limits = card_slide.motion_limits
    if card_limits is not None and card_limits.upper is not None:
        rest_card_pos = ctx.part_world_position(bounce_card)
        with ctx.pose({card_slide: card_limits.upper}):
            extended_card_pos = ctx.part_world_position(bounce_card)
            ctx.expect_within(
                bounce_card,
                head,
                axes="xy",
                inner_elem="card_plate",
                outer_elem="head_shell",
                margin=0.001,
                name="extended bounce card remains centered in the head shell slot",
            )
            ctx.expect_overlap(
                bounce_card,
                head,
                axes="z",
                elem_a="card_plate",
                elem_b="head_shell",
                min_overlap=0.020,
                name="extended bounce card remains retained inside the head shell",
            )
        ctx.check(
            "bounce card slides upward out of the head",
            rest_card_pos is not None
            and extended_card_pos is not None
            and extended_card_pos[2] > rest_card_pos[2] + 0.012,
            details=f"rest={rest_card_pos}, extended={extended_card_pos}",
        )

    return ctx.report()


object_model = build_object_model()
