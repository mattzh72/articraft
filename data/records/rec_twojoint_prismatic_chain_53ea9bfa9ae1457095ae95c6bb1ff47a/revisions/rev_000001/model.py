from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_sample_tray_carrier")

    dark_powder = Material("dark_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_black = Material("black_acetal_slide", rgba=(0.015, 0.015, 0.014, 1.0))
    brushed = Material("brushed_stainless", rgba=(0.62, 0.64, 0.62, 1.0))
    blue = Material("anodized_middle_tray", rgba=(0.18, 0.38, 0.56, 1.0))
    light = Material("ivory_inner_tray", rgba=(0.86, 0.84, 0.75, 1.0))
    safety = Material("red_stop_tabs", rgba=(0.75, 0.06, 0.035, 1.0))

    model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("black_acetal_slide", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("brushed_stainless", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("anodized_middle_tray", rgba=(0.18, 0.38, 0.56, 1.0))
    model.material("ivory_inner_tray", rgba=(0.86, 0.84, 0.75, 1.0))
    model.material("red_stop_tabs", rgba=(0.75, 0.06, 0.035, 1.0))

    outer = model.part("outer_body")
    outer.visual(
        Box((0.780, 0.460, 0.025)),
        origin=Origin(xyz=(0.000, 0.000, 0.0125)),
        material=dark_powder,
        name="outer_floor",
    )
    for y, suffix in ((0.225, "0"), (-0.225, "1")):
        outer.visual(
            Box((0.780, 0.030, 0.160)),
            origin=Origin(xyz=(0.000, y, 0.080)),
            material=dark_powder,
            name=f"outer_side_wall_{suffix}",
        )
        # Folded top lips make the fixed drawer body read as formed sheet metal.
        outer.visual(
            Box((0.765, 0.050, 0.018)),
            origin=Origin(xyz=(0.000, y * 0.855, 0.161)),
            material=dark_powder,
            name=f"outer_top_lip_{suffix}",
        )
        # Lower ledges carry the middle tray runners; upper rails capture lift.
        outer.visual(
            Box((0.675, 0.024, 0.022)),
            origin=Origin(xyz=(0.010, y * 0.880, 0.081)),
            material=rail_black,
            name=f"outer_capture_{suffix}",
        )
        outer.visual(
            Box((0.026, 0.026, 0.034)),
            origin=Origin(xyz=(0.360, y, 0.120)),
            material=safety,
            name=f"outer_stop_{suffix}",
        )
    outer.visual(
        Box((0.675, 0.024, 0.012)),
        origin=Origin(xyz=(0.010, 0.198, 0.033)),
        material=rail_black,
        name="outer_support_0",
    )
    outer.visual(
        Box((0.675, 0.024, 0.012)),
        origin=Origin(xyz=(0.010, -0.198, 0.033)),
        material=rail_black,
        name="outer_support_1",
    )
    outer.visual(
        Box((0.035, 0.460, 0.160)),
        origin=Origin(xyz=(-0.3725, 0.000, 0.080)),
        material=dark_powder,
        name="outer_rear_wall",
    )
    outer.visual(
        Box((0.035, 0.460, 0.030)),
        origin=Origin(xyz=(0.3725, 0.000, 0.015)),
        material=dark_powder,
        name="outer_front_sill",
    )
    outer.visual(
        Box((0.040, 0.110, 0.018)),
        origin=Origin(xyz=(0.385, 0.000, 0.024)),
        material=brushed,
        name="outer_front_badge",
    )

    middle = model.part("middle_tray")
    middle.visual(
        Box((0.680, 0.320, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=blue,
        name="middle_floor",
    )
    for y, suffix in ((0.162, "0"), (-0.162, "1")):
        middle.visual(
            Box((0.680, 0.020, 0.095)),
            origin=Origin(xyz=(0.000, y, 0.0475)),
            material=blue,
            name=f"middle_side_wall_{suffix}",
        )
        middle.visual(
            Box((0.665, 0.030, 0.014)),
            origin=Origin(xyz=(0.000, y * 1.160, 0.102)),
            material=blue,
            name=f"middle_lip_{suffix}",
        )
        middle.visual(
            Box((0.038, 0.026, 0.020)),
            origin=Origin(xyz=(-0.310, y * 0.895, 0.105)),
            material=safety,
            name=f"middle_rear_stop_{suffix}",
        )
    middle.visual(
        Box((0.610, 0.014, 0.024)),
        origin=Origin(xyz=(-0.010, 0.179, 0.016)),
        material=rail_black,
        name="middle_runner_0",
    )
    middle.visual(
        Box((0.610, 0.014, 0.024)),
        origin=Origin(xyz=(-0.010, -0.179, 0.016)),
        material=rail_black,
        name="middle_runner_1",
    )
    middle.visual(
        Box((0.455, 0.014, 0.010)),
        origin=Origin(xyz=(0.085, 0.145, 0.054)),
        material=rail_black,
        name="middle_inner_support_0",
    )
    middle.visual(
        Box((0.455, 0.014, 0.010)),
        origin=Origin(xyz=(0.085, -0.145, 0.054)),
        material=rail_black,
        name="middle_inner_support_1",
    )
    middle.visual(
        Box((0.022, 0.360, 0.095)),
        origin=Origin(xyz=(-0.329, 0.000, 0.0475)),
        material=blue,
        name="middle_rear_wall",
    )
    middle.visual(
        Box((0.026, 0.300, 0.046)),
        origin=Origin(xyz=(0.337, 0.000, 0.023)),
        material=blue,
        name="middle_front_lip",
    )
    middle.visual(
        Box((0.030, 0.165, 0.034)),
        origin=Origin(xyz=(0.365, 0.000, 0.040)),
        material=brushed,
        name="middle_pull",
    )

    inner = model.part("inner_tray")
    inner.visual(
        Box((0.440, 0.230, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=light,
        name="inner_floor",
    )
    for y, suffix in ((0.124, "0"), (-0.124, "1")):
        inner.visual(
            Box((0.440, 0.018, 0.075)),
            origin=Origin(xyz=(0.000, y, 0.0375)),
            material=light,
            name=f"inner_side_wall_{suffix}",
        )
        inner.visual(
            Box((0.420, 0.030, 0.012)),
            origin=Origin(xyz=(0.000, y * 0.895, 0.080)),
            material=light,
            name=f"inner_lip_{suffix}",
        )
    inner.visual(
        Box((0.380, 0.010, 0.014)),
        origin=Origin(xyz=(-0.015, 0.133, 0.008)),
        material=rail_black,
        name="inner_runner_0",
    )
    inner.visual(
        Box((0.380, 0.010, 0.014)),
        origin=Origin(xyz=(-0.015, -0.133, 0.008)),
        material=rail_black,
        name="inner_runner_1",
    )
    inner.visual(
        Box((0.018, 0.260, 0.075)),
        origin=Origin(xyz=(-0.216, 0.000, 0.0375)),
        material=light,
        name="inner_rear_wall",
    )
    inner.visual(
        Box((0.018, 0.260, 0.075)),
        origin=Origin(xyz=(0.216, 0.000, 0.0375)),
        material=light,
        name="inner_front_wall",
    )
    inner.visual(
        Box((0.024, 0.105, 0.030)),
        origin=Origin(xyz=(0.237, 0.000, 0.041)),
        material=brushed,
        name="inner_pull",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.000, 0.000, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.22, lower=0.0, upper=0.260),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.120, 0.000, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.20, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_body")
    middle = object_model.get_part("middle_tray")
    inner = object_model.get_part("inner_tray")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="y",
        margin=0.0,
        name="middle tray is laterally nested in outer body",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.570,
        elem_a="middle_runner_0",
        elem_b="outer_support_0",
        name="middle runners have long retained insertion when stowed",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="middle_runner_0",
        negative_elem="outer_support_0",
        name="middle runner bears on outer ledge",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="y",
        margin=0.0,
        name="inner tray is laterally nested in middle tray",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.330,
        elem_a="inner_runner_0",
        elem_b="middle_inner_support_0",
        name="inner runners have retained insertion when stowed",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="inner_runner_0",
        negative_elem="middle_inner_support_0",
        name="inner runner bears on middle ledge",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="z",
        min_gap=0.006,
        positive_elem="inner_floor",
        negative_elem="middle_front_lip",
        name="inner tray clears the middle front lip",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({outer_to_middle: 0.260, middle_to_inner: 0.180}):
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            margin=0.0,
            name="extended middle tray remains centered in outer rails",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.310,
            elem_a="middle_runner_0",
            elem_b="outer_support_0",
            name="extended middle tray stays supported in outer body",
        )
        ctx.expect_gap(
            middle,
            outer,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="middle_runner_0",
            negative_elem="outer_support_0",
            name="extended middle runner stays on outer ledge",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            margin=0.0,
            name="extended inner tray remains centered in middle rails",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.160,
            elem_a="inner_runner_0",
            elem_b="middle_inner_support_0",
            name="extended inner tray stays supported in middle tray",
        )
        ctx.expect_gap(
            inner,
            middle,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="inner_runner_0",
            negative_elem="middle_inner_support_0",
            name="extended inner runner stays on middle ledge",
        )
        ctx.expect_gap(
            middle,
            outer,
            axis="z",
            min_gap=0.004,
            positive_elem="middle_floor",
            negative_elem="outer_front_sill",
            name="middle tray clears the outer front sill while extending",
        )
        ctx.expect_gap(
            inner,
            middle,
            axis="z",
            min_gap=0.0005,
            positive_elem="inner_floor",
            negative_elem="middle_pull",
            name="inner tray clears the middle pull while extending",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "middle stage extends forward",
        rest_middle is not None
        and extended_middle is not None
        and extended_middle[0] > rest_middle[0] + 0.240,
        details=f"rest={rest_middle}, extended={extended_middle}",
    )
    ctx.check(
        "serial inner stage extends farther forward",
        rest_inner is not None
        and extended_inner is not None
        and extended_inner[0] > rest_inner[0] + 0.420,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
