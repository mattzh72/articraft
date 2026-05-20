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


def _build_head_shell() -> object:
    head_depth = 0.034
    head_width = 0.066
    head_height = 0.040
    wall = 0.0025
    hinge_x = 0.006
    hinge_z = 0.009

    head_center = (head_depth * 0.5 - hinge_x, 0.0, head_height * 0.5 - hinge_z)

    outer = (
        cq.Workplane("XY")
        .box(head_depth, head_width, head_height)
        .edges("|Z")
        .fillet(0.003)
        .translate(head_center)
    )
    inner = (
        cq.Workplane("XY")
        .box(head_depth - 2.0 * wall, head_width - 2.0 * wall, head_height - wall)
        .translate((head_center[0], head_center[1], head_center[2] - wall * 0.5))
    )
    slot = cq.Workplane("XY").box(0.0030, 0.044, wall + 0.003).translate((0.001, 0.0, 0.031 - wall * 0.5))

    return outer.cut(inner).cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_camera_flash")

    body_black = model.material("body_black", rgba=(0.13, 0.13, 0.14, 1.0))
    matte_dark = model.material("matte_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.91, 0.93, 0.95, 0.95))
    card_white = model.material("card_white", rgba=(0.97, 0.98, 0.98, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.32, 0.33, 0.35, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.031, 0.037, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=body_black,
        name="body_shell",
    )
    body.visual(
        Box((0.027, 0.033, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, 0.0745)),
        material=body_black,
        name="body_shoulder",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.082), rpy=(0.0, 0.0, 0.0)),
        material=accent_grey,
        name="swivel_seat",
    )
    body.visual(
        Box((0.022, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_black,
        name="shoe_pedestal",
    )
    body.visual(
        Box((0.014, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=body_black,
        name="shoe_stem",
    )
    body.visual(
        Box((0.018, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=matte_dark,
        name="shoe_plate",
    )
    body.visual(
        Box((0.018, 0.002, 0.002)),
        origin=Origin(xyz=(0.0, 0.008, -0.004)),
        material=matte_dark,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.018, 0.002, 0.002)),
        origin=Origin(xyz=(0.0, -0.008, -0.004)),
        material=matte_dark,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.0025, 0.014, 0.010)),
        origin=Origin(xyz=(0.01675, 0.0, 0.055)),
        material=matte_dark,
        name="sensor_window",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.013, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=accent_grey,
        name="swivel_collar",
    )
    cradle.visual(
        Box((0.010, 0.024, 0.009)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0095)),
        material=body_black,
        name="cradle_stem",
    )
    cradle.visual(
        Box((0.012, 0.072, 0.008)),
        origin=Origin(xyz=(-0.010, 0.0, 0.013)),
        material=body_black,
        name="cradle_bridge",
    )
    cradle.visual(
        Box((0.014, 0.004, 0.014)),
        origin=Origin(xyz=(-0.003, 0.036, 0.016)),
        material=body_black,
        name="cheek_0",
    )
    cradle.visual(
        Box((0.014, 0.004, 0.014)),
        origin=Origin(xyz=(-0.003, -0.036, 0.016)),
        material=body_black,
        name="cheek_1",
    )
    cradle.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(xyz=(0.004, 0.039, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_grey,
        name="hinge_stub_0",
    )
    cradle.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(xyz=(0.004, -0.039, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_grey,
        name="hinge_stub_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shell(), "flash_head_shell"),
        material=body_black,
        name="head_shell",
    )
    head.visual(
        Box((0.003, 0.054, 0.028)),
        origin=Origin(xyz=(0.0265, 0.0, 0.016)),
        material=diffuser_white,
        name="flash_window",
    )
    head.visual(
        Box((0.0025, 0.046, 0.024)),
        origin=Origin(xyz=(-0.00475, 0.0, 0.017)),
        material=matte_dark,
        name="rear_panel",
    )

    bounce_card = model.part("bounce_card")
    bounce_card.visual(
        Box((0.0014, 0.041, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=card_white,
        name="card_blade",
    )
    bounce_card.visual(
        Box((0.0022, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=matte_dark,
        name="card_tab",
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "cradle_to_head",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=head,
        origin=Origin(xyz=(0.004, 0.0, 0.014)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "head_to_bounce_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=bounce_card,
        origin=Origin(xyz=(0.001, 0.0, 0.031)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head = object_model.get_part("head")
    bounce_card = object_model.get_part("bounce_card")
    swivel = object_model.get_articulation("body_to_cradle")
    tilt = object_model.get_articulation("cradle_to_head")
    slide = object_model.get_articulation("head_to_bounce_card")

    swivel_limits = swivel.motion_limits
    tilt_limits = tilt.motion_limits
    slide_limits = slide.motion_limits

    rest_window_center = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    rest_card_pos = ctx.part_world_position(bounce_card)

    tilt_upper = math.radians(70.0)
    if tilt_limits is not None and tilt_limits.upper is not None:
        tilt_upper = min(tilt_upper, tilt_limits.upper)
    swivel_probe = math.radians(90.0)
    if swivel_limits is not None and swivel_limits.upper is not None:
        swivel_probe = min(swivel_probe, swivel_limits.upper)
    slide_upper = 0.0 if slide_limits is None or slide_limits.upper is None else slide_limits.upper

    with ctx.pose({tilt: tilt_upper}):
        tilted_window_center = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))

    ctx.check(
        "head tilts upward",
        rest_window_center is not None
        and tilted_window_center is not None
        and tilted_window_center[2] > rest_window_center[2] + 0.012,
        details=f"rest={rest_window_center}, tilted={tilted_window_center}",
    )

    with ctx.pose({swivel: swivel_probe}):
        swiveled_window_center = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))

    ctx.check(
        "head swivels sideways",
        rest_window_center is not None
        and swiveled_window_center is not None
        and swiveled_window_center[1] > rest_window_center[1] + 0.015
        and swiveled_window_center[0] < rest_window_center[0] - 0.010,
        details=f"rest={rest_window_center}, swiveled={swiveled_window_center}",
    )

    with ctx.pose({slide: slide_upper}):
        ctx.expect_overlap(
            bounce_card,
            head,
            axes="xy",
            elem_a="card_blade",
            elem_b="head_shell",
            min_overlap=0.001,
            name="bounce card stays centered in the head slot",
        )
        ctx.expect_overlap(
            bounce_card,
            head,
            axes="z",
            elem_a="card_blade",
            elem_b="head_shell",
            min_overlap=0.010,
            name="bounce card keeps retained insertion",
        )
        extended_card_pos = ctx.part_world_position(bounce_card)

    ctx.check(
        "bounce card extends upward",
        rest_card_pos is not None
        and extended_card_pos is not None
        and extended_card_pos[2] > rest_card_pos[2] + 0.015,
        details=f"rest={rest_card_pos}, extended={extended_card_pos}",
    )

    return ctx.report()


object_model = build_object_model()
