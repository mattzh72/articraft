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
    model = ArticulatedObject(name="slide_hinge_slide_chain")

    dark_steel = model.material("dark_steel", color=(0.12, 0.13, 0.14, 1.0))
    brushed_rail = model.material("brushed_rail", color=(0.72, 0.74, 0.72, 1.0))
    carriage_blue = model.material("carriage_blue", color=(0.08, 0.22, 0.55, 1.0))
    pivot_orange = model.material("pivot_orange", color=(0.95, 0.46, 0.12, 1.0))
    slider_red = model.material("slider_red", color=(0.86, 0.10, 0.08, 1.0))
    black_rubber = model.material("black_rubber", color=(0.02, 0.02, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.22, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    for rail_name, y in (("rail_0", -0.055), ("rail_1", 0.055)):
        base.visual(
            Box((0.82, 0.018, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.046)),
            material=brushed_rail,
            name=rail_name,
        )
    for index, x in enumerate((-0.43, 0.43)):
        base.visual(
            Box((0.035, 0.18, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=dark_steel,
            name=f"end_stop_{index}",
        )
    for index, x in enumerate((-0.32, -0.10, 0.10, 0.32)):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.028)),
            material=brushed_rail,
            name=f"deck_screw_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.16, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="crosshead_plate",
    )
    for shoe_name, y in (("slide_shoe_0", -0.055), ("slide_shoe_1", 0.055)):
        carriage.visual(
            Box((0.16, 0.030, 0.017)),
            origin=Origin(xyz=(0.0, y, -0.0155)),
            material=black_rubber,
            name=shoe_name,
        )
    carriage.visual(
        Cylinder(radius=0.066, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=brushed_rail,
        name="turntable_pad",
    )
    carriage.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_steel,
        name="pivot_stud",
    )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=pivot_orange,
        name="pivot_disk",
    )
    pivot_bracket.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=brushed_rail,
        name="hub_cap",
    )
    pivot_bracket.visual(
        Box((0.31, 0.070, 0.020)),
        origin=Origin(xyz=(0.185, 0.0, 0.010)),
        material=pivot_orange,
        name="arm_plate",
    )
    for wall_name, y in (("guide_wall_0", -0.034), ("guide_wall_1", 0.034)):
        pivot_bracket.visual(
            Box((0.30, 0.012, 0.052)),
            origin=Origin(xyz=(0.300, y, 0.046)),
            material=pivot_orange,
            name=wall_name,
        )
    pivot_bracket.visual(
        Box((0.30, 0.080, 0.012)),
        origin=Origin(xyz=(0.300, 0.0, 0.078)),
        material=pivot_orange,
        name="guide_roof",
    )

    terminal_slider = model.part("terminal_slider")
    terminal_slider.visual(
        Box((0.38, 0.037, 0.026)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=brushed_rail,
        name="slider_bar",
    )
    terminal_slider.visual(
        Box((0.060, 0.062, 0.046)),
        origin=Origin(xyz=(0.410, 0.0, 0.0)),
        material=slider_red,
        name="end_grip",
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.28, 0.0, 0.0925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.56),
    )
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "terminal_slide",
        ArticulationType.PRISMATIC,
        parent=pivot_bracket,
        child=terminal_slider,
        origin=Origin(xyz=(0.190, 0.0, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.17),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    pivot_bracket = object_model.get_part("pivot_bracket")
    terminal_slider = object_model.get_part("terminal_slider")

    base_slide = object_model.get_articulation("base_slide")
    pivot = object_model.get_articulation("pivot")
    terminal_slide = object_model.get_articulation("terminal_slide")

    ctx.allow_overlap(
        carriage,
        pivot_bracket,
        elem_a="pivot_stud",
        elem_b="hub_cap",
        reason="The dark pivot stud is intentionally captured inside the bracket hub bearing.",
    )
    ctx.allow_overlap(
        carriage,
        pivot_bracket,
        elem_a="pivot_stud",
        elem_b="pivot_disk",
        reason="The simplified solid pivot disk represents a bored turntable plate around the pivot stud.",
    )

    ctx.check(
        "slide hinge slide joint order",
        (
            base_slide.articulation_type == ArticulationType.PRISMATIC
            and pivot.articulation_type == ArticulationType.REVOLUTE
            and terminal_slide.articulation_type == ArticulationType.PRISMATIC
            and base_slide.parent == "base"
            and base_slide.child == "carriage"
            and pivot.parent == "carriage"
            and pivot.child == "pivot_bracket"
            and terminal_slide.parent == "pivot_bracket"
            and terminal_slide.child == "terminal_slider"
        ),
        details="Expected base prismatic, middle revolute, terminal prismatic chain.",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="slide_shoe_0",
        negative_elem="rail_0",
        min_gap=0.0,
        max_gap=0.001,
        name="carriage shoe rides on rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="slide_shoe_0",
        elem_b="rail_0",
        min_overlap=0.12,
        name="base slide has rail engagement",
    )
    ctx.expect_gap(
        terminal_slider,
        pivot_bracket,
        axis="z",
        positive_elem="slider_bar",
        negative_elem="arm_plate",
        max_gap=0.001,
        max_penetration=0.000001,
        name="terminal bar rides on guide floor",
    )
    ctx.expect_within(
        carriage,
        pivot_bracket,
        axes="xy",
        inner_elem="pivot_stud",
        outer_elem="hub_cap",
        margin=0.0,
        name="pivot stud is centered in hub",
    )
    ctx.expect_overlap(
        carriage,
        pivot_bracket,
        axes="z",
        elem_a="pivot_stud",
        elem_b="hub_cap",
        min_overlap=0.015,
        name="pivot stud remains captured axially",
    )
    ctx.expect_within(
        carriage,
        pivot_bracket,
        axes="xy",
        inner_elem="pivot_stud",
        outer_elem="pivot_disk",
        margin=0.0,
        name="pivot stud passes through disk center",
    )
    ctx.expect_overlap(
        carriage,
        pivot_bracket,
        axes="z",
        elem_a="pivot_stud",
        elem_b="pivot_disk",
        min_overlap=0.010,
        name="pivot disk surrounds stud bore",
    )
    ctx.expect_overlap(
        terminal_slider,
        pivot_bracket,
        axes="x",
        elem_a="slider_bar",
        elem_b="guide_wall_0",
        min_overlap=0.20,
        name="terminal slider is retained in its guide",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: 0.40}):
        moved_carriage = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem="slide_shoe_0",
            outer_elem="rail_0",
            margin=0.005,
            name="carriage remains on rail at travel",
        )
    ctx.check(
        "base slide moves along x",
        rest_carriage is not None
        and moved_carriage is not None
        and moved_carriage[0] > rest_carriage[0] + 0.35,
        details=f"rest={rest_carriage}, moved={moved_carriage}",
    )

    with ctx.pose({pivot: 0.70}):
        rotated_terminal = ctx.part_world_position(terminal_slider)
    neutral_terminal = ctx.part_world_position(terminal_slider)
    ctx.check(
        "pivot turns terminal output direction",
        neutral_terminal is not None
        and rotated_terminal is not None
        and rotated_terminal[1] > neutral_terminal[1] + 0.10,
        details=f"neutral={neutral_terminal}, rotated={rotated_terminal}",
    )

    terminal_rest = ctx.part_world_position(terminal_slider)
    with ctx.pose({terminal_slide: 0.17}):
        terminal_extended = ctx.part_world_position(terminal_slider)
        ctx.expect_overlap(
            terminal_slider,
            pivot_bracket,
            axes="x",
            elem_a="slider_bar",
            elem_b="guide_wall_0",
            min_overlap=0.06,
            name="terminal slider remains inserted at extension",
        )
    ctx.check(
        "terminal slide moves along bracket output",
        terminal_rest is not None
        and terminal_extended is not None
        and terminal_extended[0] > terminal_rest[0] + 0.15,
        details=f"rest={terminal_rest}, extended={terminal_extended}",
    )

    return ctx.report()


object_model = build_object_model()
