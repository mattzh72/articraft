from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speedlight_flash")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.055, 0.052, 1.0))
    rubber = model.material("rubber_grip", rgba=(0.020, 0.020, 0.018, 1.0))
    metal = model.material("brushed_metal", rgba=(0.55, 0.54, 0.50, 1.0))
    warm_lens = model.material("warm_diffuser", rgba=(1.0, 0.86, 0.48, 0.78))
    translucent = model.material("milky_plastic", rgba=(0.92, 0.94, 0.90, 0.86))
    red_window = model.material("red_window", rgba=(0.75, 0.05, 0.025, 0.82))

    body = model.part("body")
    body.visual(
        Box((0.048, 0.058, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=matte_black,
        name="battery_body",
    )
    body.visual(
        Box((0.054, 0.064, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_plastic,
        name="lower_battery_step",
    )
    body.visual(
        Box((0.030, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.1215)),
        material=dark_plastic,
        name="top_turntable_seat",
    )
    body.visual(
        Box((0.003, 0.044, 0.073)),
        origin=Origin(xyz=(0.0235, 0.0, 0.069)),
        material=rubber,
        name="front_battery_door",
    )
    body.visual(
        Box((0.003, 0.022, 0.012)),
        origin=Origin(xyz=(0.024, 0.0, 0.109)),
        material=red_window,
        name="focus_assist_window",
    )
    body.visual(
        Box((0.040, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.046, 0.007, 0.008)),
        origin=Origin(xyz=(0.0, 0.0215, -0.009)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.046, 0.007, 0.008)),
        origin=Origin(xyz=(0.0, -0.0215, -0.009)),
        material=metal,
        name="shoe_rail_1",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_plastic,
        name="turntable_collar",
    )
    swivel.visual(
        Box((0.022, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_plastic,
        name="neck_stem",
    )
    swivel.visual(
        Box((0.024, 0.092, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=dark_plastic,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.020, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, 0.0425, 0.064)),
        material=dark_plastic,
        name="yoke_arm_0",
    )
    swivel.visual(
        Box((0.020, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.0425, 0.064)),
        material=dark_plastic,
        name="yoke_arm_1",
    )
    swivel.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.040, 0.070), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_plastic,
        name="tilt_socket_0",
    )
    swivel.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, -0.040, 0.070), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_plastic,
        name="tilt_socket_1",
    )

    head = model.part("head")
    head.visual(
        Box((0.074, 0.074, 0.006)),
        origin=Origin(xyz=(0.037, 0.0, -0.011)),
        material=matte_black,
        name="bottom_shell",
    )
    head.visual(
        Box((0.074, 0.006, 0.050)),
        origin=Origin(xyz=(0.037, 0.034, 0.011)),
        material=matte_black,
        name="side_shell_0",
    )
    head.visual(
        Box((0.074, 0.006, 0.050)),
        origin=Origin(xyz=(0.037, -0.034, 0.011)),
        material=matte_black,
        name="side_shell_1",
    )
    head.visual(
        Box((0.006, 0.074, 0.050)),
        origin=Origin(xyz=(0.003, 0.0, 0.011)),
        material=matte_black,
        name="rear_shell",
    )
    head.visual(
        Box((0.003, 0.074, 0.004)),
        origin=Origin(xyz=(0.0135, 0.0, 0.038)),
        material=matte_black,
        name="rear_slot_lip",
    )
    head.visual(
        Box((0.003, 0.074, 0.004)),
        origin=Origin(xyz=(0.0265, 0.0, 0.038)),
        material=matte_black,
        name="front_slot_lip",
    )
    head.visual(
        Box((0.004, 0.064, 0.006)),
        origin=Origin(xyz=(0.076, 0.0, 0.027)),
        material=matte_black,
        name="front_bezel_top",
    )
    head.visual(
        Box((0.004, 0.064, 0.006)),
        origin=Origin(xyz=(0.076, 0.0, -0.011)),
        material=matte_black,
        name="front_bezel_bottom",
    )
    head.visual(
        Box((0.004, 0.006, 0.044)),
        origin=Origin(xyz=(0.076, 0.031, 0.008)),
        material=matte_black,
        name="front_bezel_0",
    )
    head.visual(
        Box((0.004, 0.006, 0.044)),
        origin=Origin(xyz=(0.076, -0.031, 0.008)),
        material=matte_black,
        name="front_bezel_1",
    )
    head.visual(
        Box((0.004, 0.058, 0.034)),
        origin=Origin(xyz=(0.078, 0.0, 0.008)),
        material=warm_lens,
        name="lamp_diffuser",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_plastic,
        name="tilt_barrel",
    )

    bounce_card = model.part("bounce_card")
    bounce_card.visual(
        Box((0.003, 0.062, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=translucent,
        name="white_card",
    )
    bounce_card.visual(
        Box((0.005, 0.062, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=translucent,
        name="pull_tab",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.1255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-0.35, upper=1.45),
    )
    model.articulation(
        "head_to_bounce_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=bounce_card,
        origin=Origin(xyz=(0.020, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=0.18, lower=0.0, upper=0.040),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    card = object_model.get_part("bounce_card")
    lower_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")
    card_slide = object_model.get_articulation("head_to_bounce_card")

    ctx.check(
        "lower joint is vertical swivel",
        lower_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper joint is horizontal tilt",
        tilt_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "bounce card slides upward",
        card_slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={card_slide.axis}",
    )

    ctx.expect_gap(
        swivel,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_collar",
        negative_elem="top_turntable_seat",
        name="swivel collar seats on body",
    )
    ctx.expect_overlap(
        swivel,
        body,
        axes="xy",
        min_overlap=0.020,
        elem_a="turntable_collar",
        elem_b="top_turntable_seat",
        name="swivel footprint is over the top seat",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.045,
        name="head is visibly raised above body by neck",
    )
    ctx.expect_within(
        card,
        head,
        axes="xy",
        margin=0.001,
        inner_elem="white_card",
        outer_elem="bottom_shell",
        name="card stays inside head slot footprint",
    )
    ctx.expect_overlap(
        card,
        head,
        axes="z",
        min_overlap=0.025,
        elem_a="white_card",
        elem_b="side_shell_0",
        name="stored card remains inserted in head",
    )

    rest_card_pos = ctx.part_world_position(card)
    with ctx.pose({card_slide: 0.040}):
        ctx.expect_overlap(
            card,
            head,
            axes="z",
            min_overlap=0.003,
            elem_a="white_card",
            elem_b="side_shell_0",
            name="extended card retains insertion in slot",
        )
        extended_card_pos = ctx.part_world_position(card)
    ctx.check(
        "card extension moves out of head top",
        rest_card_pos is not None
        and extended_card_pos is not None
        and extended_card_pos[2] > rest_card_pos[2] + 0.030,
        details=f"rest={rest_card_pos}, extended={extended_card_pos}",
    )

    return ctx.report()


object_model = build_object_model()
