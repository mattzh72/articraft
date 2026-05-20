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
    model = ArticulatedObject(name="three_stage_slide_module")

    dark_anodized = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.62, 0.65, 0.66, 1.0))
    satin_graphite = Material("satin_graphite", color=(0.20, 0.22, 0.24, 1.0))
    nylon = Material("black_nylon_guides", color=(0.02, 0.02, 0.025, 1.0))
    stop_rubber = Material("soft_end_stops", color=(0.01, 0.01, 0.012, 1.0))

    outer = model.part("outer_stage")
    outer.visual(
        Box((0.620, 0.170, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_anodized,
        name="outer_base",
    )
    for y in (-0.079, 0.079):
        outer.visual(
            Box((0.620, 0.012, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.021)),
            material=dark_anodized,
            name=f"outer_side_rail_{0 if y < 0 else 1}",
        )
        for x in (-0.300, 0.300):
            outer.visual(
                Box((0.024, 0.014, 0.036)),
                origin=Origin(xyz=(x, y, 0.024)),
                material=stop_rubber,
                name=f"outer_stop_{0 if y < 0 else 1}_{0 if x < 0 else 1}",
            )
    outer.visual(
        Box((0.560, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.040, 0.021)),
        material=nylon,
        name="outer_runner_0",
    )
    outer.visual(
        Box((0.560, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.040, 0.021)),
        material=nylon,
        name="outer_runner_1",
    )

    middle = model.part("middle_stage")
    middle.visual(
        Box((0.420, 0.100, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_steel,
        name="middle_web",
    )
    for y in (-0.046, 0.046):
        middle.visual(
            Box((0.420, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.014)),
            material=brushed_steel,
            name=f"middle_side_lip_{0 if y < 0 else 1}",
        )
    middle.visual(
        Box((0.360, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, 0.015)),
        material=nylon,
        name="middle_runner_0",
    )
    middle.visual(
        Box((0.360, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, 0.015)),
        material=nylon,
        name="middle_runner_1",
    )

    inner = model.part("inner_stage")
    inner.visual(
        Box((0.260, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_graphite,
        name="inner_web",
    )
    for y in (-0.022, 0.022):
        inner.visual(
            Box((0.260, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.012)),
            material=satin_graphite,
            name=f"inner_edge_rib_{0 if y < 0 else 1}",
        )
    inner.visual(
        Box((0.040, 0.048, 0.016)),
        origin=Origin(xyz=(0.110, 0.0, 0.013)),
        material=nylon,
        name="inner_pull_pad",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.140),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "serial prismatic slide joints",
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(inner_slide.axis) == (1.0, 0.0, 0.0),
        details=f"outer axis={outer_slide.axis}, inner axis={inner_slide.axis}",
    )

    with ctx.pose({outer_slide: 0.0, inner_slide: 0.0}):
        ctx.expect_gap(
            middle,
            outer,
            axis="z",
            positive_elem="middle_web",
            negative_elem="outer_runner_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="middle stage rides on outer runners",
        )
        ctx.expect_gap(
            inner,
            middle,
            axis="z",
            positive_elem="inner_web",
            negative_elem="middle_runner_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="inner stage rides on middle runners",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            margin=0.002,
            name="middle stage is guided within outer width",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            margin=0.002,
            name="inner stage is guided within middle width",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.400,
            name="nested middle has long engagement at rest",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.240,
            name="nested inner has long engagement at rest",
        )
        rest_middle = ctx.part_world_position(middle)
        rest_inner = ctx.part_world_position(inner)

    with ctx.pose({outer_slide: 0.140, inner_slide: 0.120}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.250,
            name="extended middle remains captured by outer",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.150,
            name="extended inner remains captured by middle",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "moving stages extend along shared guide axis",
        rest_middle is not None
        and rest_inner is not None
        and extended_middle is not None
        and extended_inner is not None
        and extended_middle[0] > rest_middle[0] + 0.13
        and extended_inner[0] > rest_inner[0] + 0.25,
        details=(
            f"middle rest={rest_middle}, middle extended={extended_middle}, "
            f"inner rest={rest_inner}, inner extended={extended_inner}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
