from __future__ import annotations

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
    model = ArticulatedObject(name="low_profile_slide_link_slide")

    steel = Material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark = Material("dark_anodized_base", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("black_guide_insert", rgba=(0.015, 0.016, 0.018, 1.0))
    blue = Material("blue_carriage", rgba=(0.05, 0.20, 0.55, 1.0))
    orange = Material("orange_link_frame", rgba=(0.95, 0.45, 0.08, 1.0))
    red = Material("red_terminal_slider", rgba=(0.75, 0.06, 0.04, 1.0))

    guide = model.part("guide")
    guide.visual(
        Box((0.84, 0.19, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark,
        name="ground_plate",
    )
    for rail_name, y in (("linear_rail_0", -0.066), ("linear_rail_1", 0.066)):
        guide.visual(
            Box((0.74, 0.024, 0.037)),
            origin=Origin(xyz=(0.0, y, 0.038)),
            material=steel,
            name=rail_name,
        )
    guide.visual(
        Box((0.70, 0.078, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=black,
        name="center_way",
    )
    for index, x in enumerate((-0.395, 0.395)):
        guide.visual(
            Box((0.030, 0.170, 0.052)),
            origin=Origin(xyz=(x, 0.0, 0.045)),
            material=dark,
            name=f"end_stop_{index}",
        )

    block = model.part("block")
    block.visual(
        Box((0.160, 0.140, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=blue,
        name="saddle",
    )
    for shoe_name, y in (("guide_shoe_0", -0.066), ("guide_shoe_1", 0.066)):
        block.visual(
            Box((0.142, 0.022, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.012)),
            material=black,
            name=shoe_name,
        )
    block.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=steel,
        name="pivot_post",
    )
    block.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel,
        name="turntable_washer",
    )

    link_frame = model.part("link_frame")
    link_frame.visual(
        Cylinder(radius=0.043, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel,
        name="hinge_hub",
    )
    link_frame.visual(
        Box((0.270, 0.054, 0.004)),
        origin=Origin(xyz=(0.185, 0.0, 0.002)),
        material=orange,
        name="guide_floor",
    )
    link_frame.visual(
        Box((0.056, 0.092, 0.020)),
        origin=Origin(xyz=(0.052, 0.0, 0.010)),
        material=orange,
        name="near_bridge",
    )
    for index, y in enumerate((-0.036, 0.036)):
        link_frame.visual(
            Box((0.286, 0.014, 0.020)),
            origin=Origin(xyz=(0.188, y, 0.010)),
            material=orange,
            name=f"side_rail_{index}",
        )
    link_frame.visual(
        Box((0.028, 0.092, 0.020)),
        origin=Origin(xyz=(0.326, 0.0, 0.010)),
        material=orange,
        name="far_crossbar",
    )

    terminal_slider = model.part("terminal_slider")
    terminal_slider.visual(
        Box((0.064, 0.042, 0.014)),
        origin=Origin(),
        material=red,
        name="slider_block",
    )
    terminal_slider.visual(
        Box((0.026, 0.030, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.020)),
        material=red,
        name="output_tab",
    )
    terminal_slider.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.022, 0.0, 0.042)),
        material=steel,
        name="terminal_pin",
    )

    model.articulation(
        "guide_to_block",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=block,
        origin=Origin(xyz=(-0.220, 0.0, 0.0745)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.55, lower=0.0, upper=0.320),
    )
    model.articulation(
        "block_to_link_frame",
        ArticulationType.REVOLUTE,
        parent=block,
        child=link_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "link_frame_to_terminal_slider",
        ArticulationType.PRISMATIC,
        parent=link_frame,
        child=terminal_slider,
        origin=Origin(xyz=(0.150, 0.0, 0.011)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.45, lower=0.0, upper=0.120),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide")
    block = object_model.get_part("block")
    link_frame = object_model.get_part("link_frame")
    terminal_slider = object_model.get_part("terminal_slider")

    first_slide = object_model.get_articulation("guide_to_block")
    hinge = object_model.get_articulation("block_to_link_frame")
    terminal_slide = object_model.get_articulation("link_frame_to_terminal_slider")

    ctx.check(
        "slide-link-slide joint chain",
        len(object_model.articulations) == 3,
        details="The mechanism should expose one guide slide, one hinge, and one terminal slide.",
    )

    ctx.expect_gap(
        block,
        guide,
        axis="z",
        positive_elem="guide_shoe_0",
        negative_elem="linear_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="block shoe rides on guide rail",
    )
    ctx.expect_within(
        terminal_slider,
        link_frame,
        axes="yz",
        inner_elem="slider_block",
        outer_elem="guide_floor",
        margin=0.035,
        name="terminal slider remains inside link guide section",
    )
    ctx.expect_overlap(
        terminal_slider,
        link_frame,
        axes="x",
        elem_a="slider_block",
        elem_b="guide_floor",
        min_overlap=0.060,
        name="terminal slider retained in guide at rest",
    )

    rest_block = ctx.part_world_position(block)
    with ctx.pose({first_slide: 0.320}):
        extended_block = ctx.part_world_position(block)
        ctx.expect_gap(
            block,
            guide,
            axis="z",
            positive_elem="guide_shoe_1",
            negative_elem="linear_rail_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended block remains seated on second rail",
        )

    ctx.check(
        "first stage translates along guide",
        rest_block is not None
        and extended_block is not None
        and extended_block[0] > rest_block[0] + 0.30,
        details=f"rest={rest_block}, extended={extended_block}",
    )

    rest_crossbar = _aabb_center(ctx.part_element_world_aabb(link_frame, elem="far_crossbar"))
    with ctx.pose({hinge: 0.80}):
        turned_crossbar = _aabb_center(ctx.part_element_world_aabb(link_frame, elem="far_crossbar"))
    ctx.check(
        "middle stage hinges about vertical pivot",
        rest_crossbar is not None
        and turned_crossbar is not None
        and turned_crossbar[1] > rest_crossbar[1] + 0.18,
        details=f"rest={rest_crossbar}, turned={turned_crossbar}",
    )

    rest_terminal = ctx.part_world_position(terminal_slider)
    with ctx.pose({terminal_slide: 0.120}):
        extended_terminal = ctx.part_world_position(terminal_slider)
        ctx.expect_within(
            terminal_slider,
            link_frame,
            axes="yz",
            inner_elem="slider_block",
            outer_elem="guide_floor",
            margin=0.035,
            name="extended terminal slider stays captured by guide",
        )
        ctx.expect_overlap(
            terminal_slider,
            link_frame,
            axes="x",
            elem_a="slider_block",
            elem_b="guide_floor",
            min_overlap=0.060,
            name="extended terminal slider keeps insertion",
        )

    ctx.check(
        "terminal stage translates along link frame",
        rest_terminal is not None
        and extended_terminal is not None
        and extended_terminal[0] > rest_terminal[0] + 0.11,
        details=f"rest={rest_terminal}, extended={extended_terminal}",
    )

    return ctx.report()


object_model = build_object_model()
