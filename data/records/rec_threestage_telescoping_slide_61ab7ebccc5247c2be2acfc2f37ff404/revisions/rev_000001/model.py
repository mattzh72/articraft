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
    model = ArticulatedObject(name="under_slung_three_stage_runner")

    dark_coat = Material("black_powder_coated_steel", rgba=(0.03, 0.035, 0.035, 1.0))
    zinc = Material("zinc_plated_outer_sleeve", rgba=(0.62, 0.65, 0.63, 1.0))
    brushed = Material("brushed_middle_runner", rgba=(0.78, 0.80, 0.78, 1.0))
    bright = Material("bright_inner_runner", rgba=(0.88, 0.89, 0.86, 1.0))
    fastener = Material("dark_fastener_heads", rgba=(0.01, 0.011, 0.012, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.82, 0.18, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=dark_coat,
        name="mounting_plate",
    )
    for flange_name, y in (("hanger_flange_0", -0.058), ("hanger_flange_1", 0.058)):
        top_support.visual(
            Box((0.76, 0.014, 0.072)),
            origin=Origin(xyz=(0.0, y, 0.1935)),
            material=dark_coat,
            name=flange_name,
        )
    for index, (x, y) in enumerate(((-0.30, -0.055), (-0.30, 0.055), (0.30, -0.055), (0.30, 0.055))):
        top_support.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(x, y, 0.244)),
            material=fastener,
            name=f"bolt_head_{index}",
        )

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        Box((0.72, 0.11, 0.0115)),
        origin=Origin(xyz=(0.0, 0.0, 0.15175)),
        material=zinc,
        name="outer_top_web",
    )
    for index, y in enumerate((-0.050, 0.050)):
        outer_sleeve.visual(
            Box((0.72, 0.010, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.1275)),
            material=zinc,
            name=f"outer_side_wall_{index}",
        )
    for lip_name, y in (("outer_lip_0", -0.0365), ("outer_lip_1", 0.0365)):
        outer_sleeve.visual(
            Box((0.72, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.1015)),
            material=zinc,
            name=lip_name,
        )

    middle_runner = model.part("middle_runner")
    middle_runner.visual(
        Box((0.62, 0.072, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=brushed,
        name="middle_top_slide",
    )
    middle_runner.visual(
        Box((0.62, 0.022, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=brushed,
        name="middle_drop_web",
    )
    middle_runner.visual(
        Box((0.62, 0.070, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=brushed,
        name="middle_lower_top",
    )
    for index, y in enumerate((-0.031, 0.031)):
        middle_runner.visual(
            Box((0.62, 0.008, 0.039)),
            origin=Origin(xyz=(0.0, y, 0.0415)),
            material=brushed,
            name=f"middle_lower_wall_{index}",
        )
    for lip_name, y in (("middle_lip_0", -0.0225), ("middle_lip_1", 0.0225)):
        middle_runner.visual(
            Box((0.62, 0.014, 0.007)),
            origin=Origin(xyz=(0.0, y, 0.0255)),
            material=brushed,
            name=lip_name,
        )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((0.52, 0.044, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=bright,
        name="inner_top_slide",
    )
    inner_runner.visual(
        Box((0.52, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=bright,
        name="inner_drop_web",
    )
    inner_runner.visual(
        Box((0.52, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=bright,
        name="inner_lower_blade",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=top_support,
        child=outer_sleeve,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=middle_runner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    outer_sleeve = object_model.get_part("outer_sleeve")
    middle_runner = object_model.get_part("middle_runner")
    inner_runner = object_model.get_part("inner_runner")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_contact(
        top_support,
        outer_sleeve,
        elem_a="hanger_flange_0",
        elem_b="outer_top_web",
        name="support flange bears on outer sleeve",
    )

    ctx.expect_within(
        middle_runner,
        outer_sleeve,
        axes="y",
        inner_elem="middle_top_slide",
        outer_elem="outer_top_web",
        margin=0.0,
        name="middle slide is laterally captured inside outer sleeve",
    )
    ctx.expect_gap(
        middle_runner,
        outer_sleeve,
        axis="z",
        positive_elem="middle_top_slide",
        negative_elem="outer_lip_0",
        max_gap=0.0005,
        max_penetration=0.00002,
        name="middle slide bears on outer retaining lip",
    )
    ctx.expect_overlap(
        middle_runner,
        outer_sleeve,
        axes="x",
        elem_a="middle_top_slide",
        elem_b="outer_top_web",
        min_overlap=0.50,
        name="middle runner is deeply inserted when collapsed",
    )

    ctx.expect_within(
        inner_runner,
        middle_runner,
        axes="y",
        inner_elem="inner_top_slide",
        outer_elem="middle_lower_top",
        margin=0.0,
        name="inner slide is laterally captured inside middle runner",
    )
    ctx.expect_gap(
        inner_runner,
        middle_runner,
        axis="z",
        positive_elem="inner_top_slide",
        negative_elem="middle_lip_0",
        max_gap=0.0005,
        max_penetration=0.00002,
        name="inner slide bears on middle retaining lip",
    )
    ctx.expect_overlap(
        inner_runner,
        middle_runner,
        axes="x",
        elem_a="inner_top_slide",
        elem_b="middle_lower_top",
        min_overlap=0.45,
        name="inner runner is deeply inserted when collapsed",
    )

    rest_middle = ctx.part_world_position(middle_runner)
    rest_inner = ctx.part_world_position(inner_runner)
    with ctx.pose({outer_slide: 0.24, inner_slide: 0.20}):
        ctx.expect_overlap(
            middle_runner,
            outer_sleeve,
            axes="x",
            elem_a="middle_top_slide",
            elem_b="outer_top_web",
            min_overlap=0.30,
            name="extended middle runner remains retained in outer sleeve",
        )
        ctx.expect_overlap(
            inner_runner,
            middle_runner,
            axes="x",
            elem_a="inner_top_slide",
            elem_b="middle_lower_top",
            min_overlap=0.30,
            name="extended inner runner remains retained in middle runner",
        )
        extended_middle = ctx.part_world_position(middle_runner)
        extended_inner = ctx.part_world_position(inner_runner)

    ctx.check(
        "serial runners extend along positive x",
        rest_middle is not None
        and rest_inner is not None
        and extended_middle is not None
        and extended_inner is not None
        and extended_middle[0] > rest_middle[0] + 0.20
        and extended_inner[0] > rest_inner[0] + 0.40,
        details=(
            f"rest_middle={rest_middle}, extended_middle={extended_middle}, "
            f"rest_inner={rest_inner}, extended_inner={extended_inner}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
