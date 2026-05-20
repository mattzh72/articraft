from __future__ import annotations

from math import pi

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

    shell = model.material("shell", color=(0.12, 0.12, 0.13))
    trim = model.material("trim", color=(0.22, 0.22, 0.24))
    neck_finish = model.material("neck_finish", color=(0.18, 0.18, 0.19))
    display_frame = model.material("display_frame", color=(0.08, 0.08, 0.09))
    screen = model.material("screen", color=(0.18, 0.34, 0.42))
    button = model.material("button", color=(0.10, 0.10, 0.11))
    diffuser = model.material("diffuser", rgba=(0.93, 0.94, 0.96, 0.90))
    card = model.material("card", color=(0.97, 0.97, 0.93))
    metal = model.material("metal", color=(0.70, 0.72, 0.74))

    body = model.part("body")
    body.visual(
        Box((0.062, 0.038, 0.069)),
        origin=Origin(xyz=(0.0, 0.0, 0.0495)),
        material=shell,
        name="main_shell",
    )
    body.visual(
        Box((0.044, 0.031, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=trim,
        name="shoulder",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=trim,
        name="swivel_base",
    )
    body.visual(
        Box((0.028, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim,
        name="hotshoe_stem",
    )
    body.visual(
        Box((0.018, 0.021, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="hotshoe_plate",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=metal,
        name="lock_ring",
    )
    body.visual(
        Box((0.040, 0.003, 0.023)),
        origin=Origin(xyz=(0.0, -0.0185, 0.061)),
        material=display_frame,
        name="display_frame",
    )
    body.visual(
        Box((0.034, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, -0.0190, 0.061)),
        material=screen,
        name="display_screen",
    )
    body.visual(
        Box((0.046, 0.003, 0.031)),
        origin=Origin(xyz=(0.0, -0.0185, 0.034)),
        material=display_frame,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.0115, length=0.0035),
        origin=Origin(xyz=(0.0, -0.0178, 0.037), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=button,
        name="control_dial",
    )
    for name, xyz in (
        ("button_up", (0.0, -0.0174, 0.051)),
        ("button_down", (0.0, -0.0174, 0.023)),
        ("button_left", (-0.015, -0.0174, 0.037)),
        ("button_right", (0.015, -0.0174, 0.037)),
        ("button_menu", (-0.016, -0.0174, 0.017)),
        ("button_back", (0.016, -0.0174, 0.017)),
    ):
        body.visual(
            Box((0.010, 0.0035, 0.0055)),
            origin=Origin(xyz=xyz),
            material=button,
            name=name,
        )
    body.visual(
        Box((0.018, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, 0.0185, 0.049)),
        material=display_frame,
        name="af_window",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=neck_finish,
        name="lower_collar",
    )
    neck.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, 0.016)),
        material=neck_finish,
        name="upright",
    )
    neck.visual(
        Box((0.050, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.011, 0.021)),
        material=trim,
        name="upper_bridge",
    )
    for i, x in enumerate((-0.028, 0.028)):
        neck.visual(
            Box((0.012, 0.010, 0.018)),
            origin=Origin(xyz=(x, -0.011, 0.014)),
            material=trim,
            name=f"arm_{i}",
        )
    for i, x in enumerate((-0.034, 0.034)):
        neck.visual(
            Cylinder(radius=0.005, length=0.002),
            origin=Origin(xyz=(x, -0.007, 0.026), rpy=(0.0, pi / 2.0, 0.0)),
            material=trim,
            name=f"pivot_{i}",
        )

    head = model.part("head")
    head.visual(
        Box((0.072, 0.045, 0.004)),
        origin=Origin(xyz=(0.0, 0.0295, -0.002)),
        material=shell,
        name="bottom_wall",
    )
    for i, x in enumerate((-0.034, 0.034)):
        head.visual(
            Box((0.004, 0.045, 0.044)),
            origin=Origin(xyz=(x, 0.0295, 0.018)),
            material=shell,
            name=f"side_{i}",
        )
    head.visual(
        Box((0.068, 0.004, 0.044)),
        origin=Origin(xyz=(0.0, 0.009, 0.018)),
        material=shell,
        name="rear_wall",
    )
    head.visual(
        Box((0.072, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.010, 0.042)),
        material=shell,
        name="top_rear",
    )
    head.visual(
        Box((0.072, 0.031, 0.004)),
        origin=Origin(xyz=(0.0, 0.0365, 0.042)),
        material=shell,
        name="top_front",
    )
    head.visual(
        Box((0.064, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, 0.0505, 0.019)),
        material=diffuser,
        name="diffuser",
    )
    for i, x in enumerate((-0.037, 0.037)):
        head.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=trim,
            name=f"boss_{i}",
        )
    for i, x in enumerate((-0.034, 0.034)):
        head.visual(
            Box((0.006, 0.008, 0.010)),
            origin=Origin(xyz=(x, 0.006, 0.001)),
            material=trim,
            name=f"boss_bridge_{i}",
        )
    for i, x in enumerate((-0.015, 0.015)):
        head.visual(
            Box((0.002, 0.012, 0.040)),
            origin=Origin(xyz=(x, 0.013, 0.020)),
            material=trim,
            name=f"card_guide_{i}",
        )

    bounce_card = model.part("bounce_card")
    bounce_card.visual(
        Box((0.028, 0.0016, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=card,
        name="card_panel",
    )
    bounce_card.visual(
        Box((0.012, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=card,
        name="pull_tab",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, -0.007, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.5, lower=-0.15, upper=1.45),
    )
    model.articulation(
        "head_to_bounce_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=bounce_card,
        origin=Origin(xyz=(0.0, 0.017, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.024),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    bounce_card = object_model.get_part("bounce_card")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    slider = object_model.get_articulation("head_to_bounce_card")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    with ctx.pose({swivel: 0.0, tilt: 0.0, slider: 0.0}):
        ctx.expect_gap(
            neck,
            body,
            axis="z",
            positive_elem="lower_collar",
            negative_elem="swivel_base",
            max_gap=0.0005,
            max_penetration=0.0,
            name="neck seats on the swivel base",
        )
        ctx.expect_gap(
            head,
            neck,
            axis="y",
            positive_elem="rear_wall",
            negative_elem="upper_bridge",
            min_gap=0.004,
            max_gap=0.010,
            name="head stays visibly forward of the neck yoke",
        )
        ctx.expect_within(
            bounce_card,
            head,
            axes="xy",
            inner_elem="card_panel",
            margin=0.001,
            name="bounce card stays laterally within the head shell",
        )

    swivel_limits = swivel.motion_limits
    if swivel_limits is not None and swivel_limits.upper is not None:
        rest_center = aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
        with ctx.pose({swivel: swivel_limits.upper, tilt: 0.0, slider: 0.0}):
            swiveled_center = aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
        ctx.check(
            "neck swivel turns the lamp head sideways",
            rest_center is not None
            and swiveled_center is not None
            and swiveled_center[0] < rest_center[0] - 0.030,
            details=f"rest={rest_center}, swiveled={swiveled_center}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_center = aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
        with ctx.pose({tilt: tilt_limits.upper, slider: 0.0}):
            tilted_center = aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
        ctx.check(
            "head tilt raises the diffuser upward",
            rest_center is not None
            and tilted_center is not None
            and tilted_center[2] > rest_center[2] + 0.025,
            details=f"rest={rest_center}, tilted={tilted_center}",
        )

    slider_limits = slider.motion_limits
    if slider_limits is not None and slider_limits.upper is not None:
        rest_pos = ctx.part_world_position(bounce_card)
        with ctx.pose({slider: slider_limits.upper}):
            ctx.expect_within(
                bounce_card,
                head,
                axes="xy",
                inner_elem="card_panel",
                margin=0.001,
                name="extended bounce card stays centered in the slot",
            )
            ctx.expect_overlap(
                bounce_card,
                head,
                axes="z",
                elem_a="card_panel",
                min_overlap=0.006,
                name="bounce card remains retained in the head at full extension",
            )
            extended_pos = ctx.part_world_position(bounce_card)
        ctx.check(
            "bounce card slides upward from the head top",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.020,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
