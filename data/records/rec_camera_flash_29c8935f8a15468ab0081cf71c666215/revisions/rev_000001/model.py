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


def _slotted_head_shell():
    """Boxy flash head housing with a real top slot for the bounce card."""
    outer = cq.Workplane("XY").box(0.066, 0.066, 0.042).translate((0.024, 0.0, 0.026))
    # A narrow vertical channel opens through the top and clears the card in X/Y.
    slot = cq.Workplane("XY").box(0.007, 0.056, 0.060).translate((-0.004, 0.0, 0.026))
    shell = outer.cut(slot)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_shoe_flash")

    black = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.055, 0.058, 0.064, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    lens = model.material("milky_flash_lens", rgba=(0.86, 0.90, 0.92, 0.62))
    lcd = model.material("blue_black_lcd", rgba=(0.07, 0.16, 0.20, 0.85))
    metal = model.material("brushed_shoe_metal", rgba=(0.70, 0.71, 0.70, 1.0))
    card_mat = model.material("white_bounce_card", rgba=(0.94, 0.95, 0.91, 0.78))
    ink = model.material("white_marking", rgba=(0.95, 0.96, 0.94, 1.0))
    red = model.material("ready_lamp_red", rgba=(0.9, 0.06, 0.02, 1.0))

    body = model.part("battery_body")
    body.visual(
        Box((0.042, 0.058, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=black,
        name="battery_shell",
    )
    body.visual(
        Box((0.002, 0.040, 0.022)),
        origin=Origin(xyz=(-0.0220, 0.0, -0.026)),
        material=lcd,
        name="rear_display",
    )
    body.visual(
        Box((0.002, 0.046, 0.040)),
        origin=Origin(xyz=(-0.0220, 0.0, -0.066)),
        material=charcoal,
        name="rear_control_panel",
    )
    body.visual(
        Box((0.0015, 0.007, 0.007)),
        origin=Origin(xyz=(-0.0232, 0.017, -0.046)),
        material=red,
        name="ready_lamp",
    )
    body.visual(
        Box((0.030, 0.042, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.099)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.030, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.019, -0.105)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.030, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.019, -0.105)),
        material=metal,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.0015, 0.044, 0.0015)),
        origin=Origin(xyz=(-0.0231, 0.0, -0.046)),
        material=ink,
        name="display_label_line",
    )

    neck = model.part("swivel_neck")
    neck.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(),
        material=charcoal,
        name="swivel_base",
    )
    neck.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="neck_post",
    )
    neck.visual(
        Box((0.018, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=charcoal,
        name="yoke_bridge",
    )
    neck.visual(
        Box((0.014, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, 0.039, 0.036)),
        material=charcoal,
        name="yoke_arm_0",
    )
    neck.visual(
        Box((0.014, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, -0.039, 0.036)),
        material=charcoal,
        name="yoke_arm_1",
    )

    head = model.part("flash_head")
    head.visual(
        mesh_from_cadquery(_slotted_head_shell(), "slotted_head_shell", tolerance=0.0007),
        origin=Origin(),
        material=black,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="tilt_barrel",
    )
    head.visual(
        Box((0.004, 0.050, 0.028)),
        origin=Origin(xyz=(0.059, 0.0, 0.026)),
        material=lens,
        name="front_lens",
    )
    head.visual(
        Box((0.002, 0.060, 0.002)),
        origin=Origin(xyz=(-0.008, 0.0, 0.048)),
        material=charcoal,
        name="top_slot_lip_0",
    )
    head.visual(
        Box((0.002, 0.060, 0.002)),
        origin=Origin(xyz=(0.001, 0.0, 0.048)),
        material=charcoal,
        name="top_slot_lip_1",
    )

    bounce_card = model.part("bounce_card")
    bounce_card.visual(
        Box((0.003, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=card_mat,
        name="card_sheet",
    )
    bounce_card.visual(
        Box((0.004, 0.052, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=card_mat,
        name="pull_tab",
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="dial_cap",
    )
    selector_dial.visual(
        Box((0.0008, 0.0016, 0.014)),
        origin=Origin(xyz=(-0.0024, 0.0, 0.0)),
        material=ink,
        name="dial_index",
    )

    for i, y in enumerate((-0.012, 0.012)):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.0042, length=0.003),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.0245, y, -0.083)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=0.05, lower=0.0, upper=0.002),
        )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        # The head shell extends forward along local +X; -Y makes positive q bounce upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.5, lower=-0.35, upper=1.25),
    )
    model.articulation(
        "head_to_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=bounce_card,
        origin=Origin(xyz=(-0.004, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=0.08, lower=0.0, upper=0.032),
    )
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(-0.025, 0.0, -0.063)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=4.0, lower=-2.2, upper=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("battery_body")
    neck = object_model.get_part("swivel_neck")
    head = object_model.get_part("flash_head")
    card = object_model.get_part("bounce_card")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    slide = object_model.get_articulation("head_to_card")

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        name="swivel base sits on body top",
    )
    ctx.expect_contact(
        head,
        neck,
        elem_a="tilt_barrel",
        elem_b="yoke_arm_0",
        contact_tol=0.001,
        name="head barrel is captured by yoke",
    )
    ctx.expect_within(
        card,
        head,
        axes="xy",
        inner_elem="card_sheet",
        outer_elem="head_shell",
        margin=0.0,
        name="bounce card fits inside top slot footprint",
    )
    ctx.expect_overlap(
        card,
        head,
        axes="z",
        elem_a="card_sheet",
        elem_b="head_shell",
        min_overlap=0.035,
        name="collapsed bounce card remains inserted",
    )

    def _coords(v):
        if hasattr(v, "x"):
            return (v.x, v.y, v.z)
        return tuple(v)

    def _aabb_center(aabb):
        lo, hi = aabb
        lo = _coords(lo)
        hi = _coords(hi)
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_card_center = _aabb_center(ctx.part_element_world_aabb(card, elem="card_sheet"))
    with ctx.pose({slide: 0.032}):
        extended_card_center = _aabb_center(ctx.part_element_world_aabb(card, elem="card_sheet"))
        ctx.expect_overlap(
            card,
            head,
            axes="z",
            elem_a="card_sheet",
            elem_b="head_shell",
            min_overlap=0.010,
            name="extended bounce card retains insertion",
        )
    ctx.check(
        "bounce card slides upward",
        extended_card_center[2] > rest_card_center[2] + 0.025,
        details=f"rest={rest_card_center}, extended={extended_card_center}",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({tilt: 0.85}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "head tilts upward on horizontal joint",
        tilted_lens_center[2] > rest_lens_center[2] + 0.020,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    with ctx.pose({swivel: 0.85}):
        swiveled_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "neck swivels head around vertical axis",
        abs(swiveled_lens_center[1] - rest_lens_center[1]) > 0.020,
        details=f"rest={rest_lens_center}, swiveled={swiveled_lens_center}",
    )

    return ctx.report()


object_model = build_object_model()
