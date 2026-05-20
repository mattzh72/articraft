from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_two_stage_slide")

    anodized = Material("black_anodized_aluminum", rgba=(0.02, 0.023, 0.026, 1.0))
    dark_steel = Material("dark_zinc_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    bright_steel = Material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    polymer = Material("low_friction_wear_strip", rgba=(0.06, 0.06, 0.055, 1.0))
    rubber = Material("black_rubber_stop", rgba=(0.01, 0.01, 0.01, 1.0))

    for material in (anodized, dark_steel, bright_steel, polymer, rubber):
        model.materials.append(material)

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.90, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=anodized,
        name="mounting_plate",
    )
    top_support.visual(
        Box((0.82, 0.012, 0.078)),
        origin=Origin(xyz=(0.0, 0.075, 0.132)),
        material=anodized,
        name="support_web_0",
    )
    top_support.visual(
        Box((0.82, 0.012, 0.078)),
        origin=Origin(xyz=(0.0, -0.075, 0.132)),
        material=anodized,
        name="support_web_1",
    )
    top_support.visual(
        Box((0.80, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.095)),
        material=anodized,
        name="support_lip_0",
    )
    top_support.visual(
        Box((0.80, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.060, 0.095)),
        material=anodized,
        name="support_lip_1",
    )
    top_support.visual(
        Box((0.74, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.047, 0.103)),
        material=polymer,
        name="fixed_wear_strip_0",
    )
    top_support.visual(
        Box((0.74, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.047, 0.103)),
        material=polymer,
        name="fixed_wear_strip_1",
    )
    top_support.visual(
        Box((0.022, 0.160, 0.035)),
        origin=Origin(xyz=(-0.405, 0.0, 0.145)),
        material=rubber,
        name="rear_stop",
    )
    top_support.visual(
        Box((0.022, 0.160, 0.035)),
        origin=Origin(xyz=(0.405, 0.0, 0.145)),
        material=rubber,
        name="front_stop",
    )

    outer_slider = model.part("outer_slider")
    outer_slider.visual(
        Box((0.60, 0.078, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=dark_steel,
        name="outer_web",
    )
    outer_slider.visual(
        Box((0.60, 0.009, 0.086)),
        origin=Origin(xyz=(0.0, 0.0395, 0.073)),
        material=dark_steel,
        name="outer_cheek_0",
    )
    outer_slider.visual(
        Box((0.60, 0.009, 0.086)),
        origin=Origin(xyz=(0.0, -0.0395, 0.073)),
        material=dark_steel,
        name="outer_cheek_1",
    )
    outer_slider.visual(
        Box((0.60, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.035, 0.113)),
        material=dark_steel,
        name="outer_runner_0",
    )
    outer_slider.visual(
        Box((0.60, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.035, 0.113)),
        material=dark_steel,
        name="outer_runner_1",
    )
    outer_slider.visual(
        Box((0.60, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.031, 0.029)),
        material=dark_steel,
        name="outer_lip_0",
    )
    outer_slider.visual(
        Box((0.60, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.031, 0.029)),
        material=dark_steel,
        name="outer_lip_1",
    )
    outer_slider.visual(
        Box((0.040, 0.074, 0.018)),
        origin=Origin(xyz=(-0.300, 0.0, 0.074)),
        material=dark_steel,
        name="outer_end_tie",
    )

    inner_slider = model.part("inner_slider")
    inner_slider.visual(
        Box((0.54, 0.056, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=bright_steel,
        name="inner_flange",
    )
    inner_slider.visual(
        Box((0.54, 0.026, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=bright_steel,
        name="inner_stem",
    )
    inner_slider.visual(
        Box((0.54, 0.042, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=bright_steel,
        name="lower_blade",
    )
    inner_slider.visual(
        Box((0.020, 0.040, 0.040)),
        origin=Origin(xyz=(0.270, 0.0, 0.012)),
        material=bright_steel,
        name="inner_end_tie",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=outer_slider,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.24),
        motion_properties=MotionProperties(damping=4.0, friction=1.5),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_slider,
        child=inner_slider,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.22),
        motion_properties=MotionProperties(damping=3.0, friction=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    outer_slider = object_model.get_part("outer_slider")
    inner_slider = object_model.get_part("inner_slider")
    support_to_outer = object_model.get_articulation("support_to_outer")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

    ctx.check(
        "serial prismatic slide chain",
        support_to_outer.parent == "top_support"
        and support_to_outer.child == "outer_slider"
        and outer_to_inner.parent == "outer_slider"
        and outer_to_inner.child == "inner_slider",
        details="Expected top_support -> outer_slider -> inner_slider serial joint chain.",
    )
    ctx.check(
        "both slide axes run fore-aft",
        support_to_outer.axis == (1.0, 0.0, 0.0) and outer_to_inner.axis == (1.0, 0.0, 0.0),
        details=f"axes={support_to_outer.axis}, {outer_to_inner.axis}",
    )

    ctx.expect_gap(
        outer_slider,
        top_support,
        axis="z",
        positive_elem="outer_runner_0",
        negative_elem="support_lip_0",
        max_gap=0.007,
        max_penetration=0.0,
        name="outer runner rides just above fixed lip",
    )
    ctx.expect_gap(
        inner_slider,
        outer_slider,
        axis="z",
        positive_elem="inner_flange",
        negative_elem="outer_lip_0",
        max_gap=0.006,
        max_penetration=0.0,
        name="inner flange is captured above outer lip",
    )
    ctx.expect_within(
        outer_slider,
        top_support,
        axes="y",
        margin=0.002,
        name="outer slider is laterally inside support channel",
    )
    ctx.expect_within(
        inner_slider,
        outer_slider,
        axes="y",
        margin=0.002,
        name="inner slider is laterally nested inside outer slider",
    )
    ctx.expect_overlap(
        outer_slider,
        top_support,
        axes="x",
        min_overlap=0.55,
        name="retracted outer stage has long bearing engagement",
    )
    ctx.expect_overlap(
        inner_slider,
        outer_slider,
        axes="x",
        min_overlap=0.50,
        name="retracted inner stage has long bearing engagement",
    )

    rest_inner = ctx.part_world_position(inner_slider)
    with ctx.pose({support_to_outer: 0.24, outer_to_inner: 0.22}):
        ctx.expect_overlap(
            outer_slider,
            top_support,
            axes="x",
            min_overlap=0.34,
            name="extended outer stage remains retained in support",
        )
        ctx.expect_overlap(
            inner_slider,
            outer_slider,
            axes="x",
            min_overlap=0.30,
            name="extended inner stage remains retained in outer slider",
        )
        extended_inner = ctx.part_world_position(inner_slider)

    ctx.check(
        "two stages extend in the same direction",
        rest_inner is not None
        and extended_inner is not None
        and extended_inner[0] > rest_inner[0] + 0.43,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
