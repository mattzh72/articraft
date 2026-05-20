from __future__ import annotations

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
    model = ArticulatedObject(name="speedlight_flash")

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_black = model.material("trim_black", rgba=(0.16, 0.16, 0.17, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.34, 0.35, 0.37, 1.0))
    lens_white = model.material("lens_white", rgba=(0.88, 0.90, 0.93, 1.0))
    card_white = model.material("card_white", rgba=(0.97, 0.97, 0.95, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.066, 0.040, 0.058)),
        origin=Origin(xyz=(0.004, 0.0, -0.038)),
        material=body_black,
        name="upper_shell",
    )
    body.visual(
        Box((0.046, 0.032, 0.050)),
        origin=Origin(xyz=(-0.002, 0.0, -0.091)),
        material=body_black,
        name="lower_shell",
    )
    body.visual(
        Box((0.046, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_black,
        name="shoulder_top",
    )
    body.visual(
        Box((0.016, 0.028, 0.018)),
        origin=Origin(xyz=(0.029, 0.0, -0.042)),
        material=trim_black,
        name="front_panel",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
        material=metal_dark,
        name="lock_ring",
    )
    body.visual(
        Box((0.022, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=metal_dark,
        name="shoe_stem",
    )
    body.visual(
        Box((0.022, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.127)),
        material=metal_dark,
        name="shoe_tongue",
    )
    body.visual(
        Box((0.032, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.008, -0.130)),
        material=metal_dark,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.032, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -0.008, -0.130)),
        material=metal_dark,
        name="shoe_rail_1",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_black,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.010, 0.046, 0.026)),
        origin=Origin(xyz=(-0.005, 0.0, 0.020)),
        material=trim_black,
        name="rear_web",
    )
    neck.visual(
        Box((0.010, 0.006, 0.044)),
        origin=Origin(xyz=(0.002, 0.026, 0.023)),
        material=trim_black,
        name="left_arm",
    )
    neck.visual(
        Box((0.010, 0.006, 0.044)),
        origin=Origin(xyz=(0.002, -0.026, 0.023)),
        material=trim_black,
        name="right_arm",
    )

    head = model.part("head")
    head.visual(
        Box((0.054, 0.049, 0.004)),
        origin=Origin(xyz=(0.030, 0.0, -0.012)),
        material=body_black,
        name="bottom_shell",
    )
    head.visual(
        Box((0.054, 0.003, 0.042)),
        origin=Origin(xyz=(0.030, 0.026, 0.007)),
        material=body_black,
        name="left_wall",
    )
    head.visual(
        Box((0.054, 0.003, 0.042)),
        origin=Origin(xyz=(0.030, -0.026, 0.007)),
        material=body_black,
        name="right_wall",
    )
    head.visual(
        Box((0.006, 0.052, 0.026)),
        origin=Origin(xyz=(0.004, 0.0, 0.003)),
        material=body_black,
        name="rear_wall",
    )
    head.visual(
        Box((0.008, 0.052, 0.004)),
        origin=Origin(xyz=(0.008, 0.0, 0.028)),
        material=body_black,
        name="rear_roof",
    )
    head.visual(
        Box((0.026, 0.052, 0.004)),
        origin=Origin(xyz=(0.044, 0.0, 0.028)),
        material=body_black,
        name="front_roof",
    )
    head.visual(
        Box((0.004, 0.052, 0.044)),
        origin=Origin(xyz=(0.057, 0.0, 0.008)),
        material=trim_black,
        name="front_bezel",
    )
    head.visual(
        Box((0.002, 0.046, 0.032)),
        origin=Origin(xyz=(0.059, 0.0, 0.010)),
        material=lens_white,
        name="lens_panel",
    )
    head.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.006, 0.031, 0.000)),
        material=trim_black,
        name="left_pivot_boss",
    )
    head.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.006, -0.031, 0.000)),
        material=trim_black,
        name="right_pivot_boss",
    )
    head.visual(
        Box((0.004, 0.0015, 0.012)),
        origin=Origin(xyz=(0.006, 0.02825, 0.000)),
        material=trim_black,
        name="left_pivot_bridge",
    )
    head.visual(
        Box((0.004, 0.0015, 0.012)),
        origin=Origin(xyz=(0.006, -0.02825, 0.000)),
        material=trim_black,
        name="right_pivot_bridge",
    )
    head.visual(
        Box((0.016, 0.004, 0.020)),
        origin=Origin(xyz=(0.012, 0.009, 0.010)),
        material=trim_black,
        name="slot_rail_0",
    )
    head.visual(
        Box((0.016, 0.004, 0.020)),
        origin=Origin(xyz=(0.012, -0.009, 0.010)),
        material=trim_black,
        name="slot_rail_1",
    )

    bounce_card = model.part("bounce_card")
    bounce_card.visual(
        Box((0.0015, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=card_white,
        name="card_plate",
    )
    bounce_card.visual(
        Box((0.004, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=trim_black,
        name="card_slider",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.57,
            upper=1.57,
        ),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.35,
            upper=1.45,
        ),
    )
    model.articulation(
        "head_to_bounce_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=bounce_card,
        origin=Origin(xyz=(0.020, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.2,
            lower=0.0,
            upper=0.028,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    bounce_card = object_model.get_part("bounce_card")

    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    slide = object_model.get_articulation("head_to_bounce_card")

    ctx.expect_contact(
        neck,
        body,
        elem_a="swivel_collar",
        elem_b="shoulder_top",
        name="swivel collar seats on body shoulder",
    )
    ctx.expect_contact(
        neck,
        head,
        elem_a="left_arm",
        elem_b="left_pivot_boss",
        name="left yoke arm supports head",
    )
    ctx.expect_contact(
        neck,
        head,
        elem_a="right_arm",
        elem_b="right_pivot_boss",
        name="right yoke arm supports head",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.025,
        name="head remains clearly above the battery body",
    )
    ctx.expect_within(
        bounce_card,
        head,
        axes="xy",
        elem_a="card_plate",
        margin=0.004,
        name="bounce card stays centered in the head slot",
    )

    lens_rest = _aabb_center(ctx.part_element_world_aabb(head, elem="lens_panel"))
    card_rest = _aabb_center(ctx.part_element_world_aabb(bounce_card, elem="card_plate"))

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            lens_tilted = _aabb_center(ctx.part_element_world_aabb(head, elem="lens_panel"))
        ctx.check(
            "positive tilt raises the flash head",
            lens_rest is not None
            and lens_tilted is not None
            and lens_tilted[2] > lens_rest[2] + 0.020,
            details=f"rest={lens_rest}, tilted={lens_tilted}",
        )

    with ctx.pose({swivel: 0.9}):
        lens_swiveled = _aabb_center(ctx.part_element_world_aabb(head, elem="lens_panel"))
    ctx.check(
        "positive swivel turns the head leftward",
        lens_rest is not None
        and lens_swiveled is not None
        and lens_swiveled[1] > lens_rest[1] + 0.030,
        details=f"rest={lens_rest}, swiveled={lens_swiveled}",
    )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide: slide_limits.upper}):
            card_extended = _aabb_center(
                ctx.part_element_world_aabb(bounce_card, elem="card_plate")
            )
            ctx.expect_within(
                bounce_card,
                head,
                axes="xy",
                elem_a="card_plate",
                margin=0.004,
                name="extended bounce card stays aligned to the slot",
            )
        ctx.check(
            "bounce card slides upward from the head top",
            card_rest is not None
            and card_extended is not None
            and card_extended[2] > card_rest[2] + 0.020,
            details=f"rest={card_rest}, extended={card_extended}",
        )

    return ctx.report()


object_model = build_object_model()
