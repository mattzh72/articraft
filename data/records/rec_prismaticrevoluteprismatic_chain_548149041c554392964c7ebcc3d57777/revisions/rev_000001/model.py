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
    model = ArticulatedObject(name="compact_slide_link_slide_module")

    dark_steel = model.material("dark_hard_anodized_steel", color=(0.12, 0.13, 0.14, 1.0))
    rail_steel = model.material("brushed_rail_steel", color=(0.48, 0.50, 0.50, 1.0))
    carriage_blue = model.material("service_blue_carriage", color=(0.07, 0.18, 0.34, 1.0))
    pivot_gray = model.material("matte_pivot_frame_gray", color=(0.30, 0.31, 0.32, 1.0))
    slider_orange = model.material("orange_terminal_slider", color=(0.95, 0.42, 0.08, 1.0))
    stop_black = model.material("black_stop_blocks", color=(0.04, 0.04, 0.04, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.90, 0.36, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="ground_plate",
    )
    body.visual(
        Box((0.78, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, -0.125, 0.085)),
        material=rail_steel,
        name="rail_0",
    )
    body.visual(
        Box((0.78, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, 0.125, 0.085)),
        material=rail_steel,
        name="rail_1",
    )
    body.visual(
        Box((0.055, 0.31, 0.055)),
        origin=Origin(xyz=(-0.405, 0.0, 0.0875)),
        material=stop_black,
        name="rear_stop",
    )
    body.visual(
        Box((0.055, 0.31, 0.055)),
        origin=Origin(xyz=(0.405, 0.0, 0.0875)),
        material=stop_black,
        name="front_stop",
    )
    body.visual(
        Box((0.34, 0.11, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=stop_black,
        name="center_wear_strip",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, -0.125, 0.013)),
        material=carriage_blue,
        name="runner_0",
    )
    carriage.visual(
        Box((0.22, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, 0.125, 0.013)),
        material=carriage_blue,
        name="runner_1",
    )
    carriage.visual(
        Box((0.24, 0.31, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material=carriage_blue,
        name="deck",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0785)),
        material=rail_steel,
        name="pivot_boss",
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=pivot_gray,
        name="pivot_pin",
    )
    pivot_frame.visual(
        Box((0.09, 0.13, 0.045)),
        origin=Origin(xyz=(0.045, 0.0, 0.0445)),
        material=pivot_gray,
        name="hub_block",
    )
    pivot_frame.visual(
        Box((0.34, 0.070, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, 0.016)),
        material=pivot_gray,
        name="floor_strip",
    )
    for index, y in enumerate((-0.055, 0.055)):
        pivot_frame.visual(
            Box((0.32, 0.018, 0.035)),
            origin=Origin(xyz=(0.215, y, 0.0395)),
            material=pivot_gray,
            name=f"side_rail_{index}",
        )
    pivot_frame.visual(
        Box((0.028, 0.13, 0.050)),
        origin=Origin(xyz=(0.375, 0.0, 0.047)),
        material=pivot_gray,
        name="open_end_lip",
    )

    terminal_slider = model.part("terminal_slider")
    terminal_slider.visual(
        Box((0.160, 0.055, 0.028)),
        origin=Origin(xyz=(0.080, 0.0, 0.014)),
        material=slider_orange,
        name="slider_block",
    )
    terminal_slider.visual(
        Box((0.040, 0.085, 0.036)),
        origin=Origin(xyz=(0.178, 0.0, 0.018)),
        material=slider_orange,
        name="square_nose",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.22, 0.0, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.36),
    )
    model.articulation(
        "carriage_to_pivot_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "pivot_frame_to_terminal_slider",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=terminal_slider,
        origin=Origin(xyz=(0.115, 0.0, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=0.0, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    pivot_frame = object_model.get_part("pivot_frame")
    terminal_slider = object_model.get_part("terminal_slider")
    carriage_slide = object_model.get_articulation("body_to_carriage")
    pivot = object_model.get_articulation("carriage_to_pivot_frame")
    terminal_slide = object_model.get_articulation("pivot_frame_to_terminal_slider")

    ctx.check(
        "slide-link-slide joint sequence",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and pivot.articulation_type == ArticulationType.REVOLUTE
        and terminal_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Expected prismatic carriage, revolute pivot frame, and prismatic terminal slider.",
    )
    ctx.expect_gap(
        carriage,
        body,
        axis="z",
        positive_elem="runner_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.000001,
        name="carriage runner sits on body rail",
    )
    ctx.expect_gap(
        pivot_frame,
        carriage,
        axis="z",
        positive_elem="pivot_pin",
        negative_elem="pivot_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="pivot frame is seated on carriage boss",
    )
    ctx.expect_gap(
        terminal_slider,
        pivot_frame,
        axis="z",
        positive_elem="slider_block",
        negative_elem="floor_strip",
        max_gap=0.001,
        max_penetration=0.000001,
        name="terminal slider rides on frame floor",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.36}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            body,
            axis="z",
            positive_elem="runner_0",
            negative_elem="rail_0",
            max_gap=0.001,
            max_penetration=0.000001,
            name="extended carriage remains on rail",
        )
    ctx.check(
        "carriage translates along body",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.30,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    slider_rest = ctx.part_world_position(terminal_slider)
    with ctx.pose({terminal_slide: 0.12}):
        slider_extended = ctx.part_world_position(terminal_slider)
        ctx.expect_gap(
            terminal_slider,
            pivot_frame,
            axis="z",
            positive_elem="slider_block",
            negative_elem="floor_strip",
            max_gap=0.001,
            max_penetration=0.000001,
            name="extended terminal slider stays supported",
        )
    ctx.check(
        "terminal slider translates along pivot frame",
        slider_rest is not None
        and slider_extended is not None
        and slider_extended[0] > slider_rest[0] + 0.10,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    terminal_rest_aabb = ctx.part_world_aabb(terminal_slider)
    with ctx.pose({pivot: 0.85}):
        terminal_rotated_aabb = ctx.part_world_aabb(terminal_slider)
    if terminal_rest_aabb is None or terminal_rotated_aabb is None:
        ctx.fail("pivot frame rotates terminal assembly", "Missing terminal-slider AABB.")
    else:
        rest_y = 0.5 * (terminal_rest_aabb[0][1] + terminal_rest_aabb[1][1])
        rotated_y = 0.5 * (terminal_rotated_aabb[0][1] + terminal_rotated_aabb[1][1])
        ctx.check(
            "pivot frame rotates terminal assembly",
            rotated_y > rest_y + 0.08,
            details=f"rest_y={rest_y}, rotated_y={rotated_y}",
        )

    return ctx.report()


object_model = build_object_model()
