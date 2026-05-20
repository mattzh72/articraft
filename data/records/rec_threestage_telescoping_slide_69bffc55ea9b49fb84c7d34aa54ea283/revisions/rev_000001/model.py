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
    model = ArticulatedObject(name="long_reach_extension_rail")

    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.09, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.015, 0.015, 0.018, 1.0))
    zinc_middle = model.material("zinc_middle", rgba=(0.50, 0.56, 0.62, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.78, 0.82, 0.84, 1.0))
    carriage_orange = model.material("carriage_orange", rgba=(0.90, 0.35, 0.08, 1.0))

    outer = model.part("outer_member")
    # Fixed outer rail: a straight rectilinear U-channel with mounting plate.
    outer.visual(
        Box((1.26, 0.22, 0.012)),
        origin=Origin(xyz=(0.60, 0.0, -0.005)),
        material=black_oxide,
        name="mounting_plate",
    )
    outer.visual(
        Box((1.20, 0.16, 0.018)),
        origin=Origin(xyz=(0.60, 0.0, 0.009)),
        material=dark_anodized,
        name="outer_web",
    )
    outer.visual(
        Box((1.20, 0.018, 0.090)),
        origin=Origin(xyz=(0.60, 0.071, 0.054)),
        material=dark_anodized,
        name="outer_side_0",
    )
    outer.visual(
        Box((1.20, 0.018, 0.090)),
        origin=Origin(xyz=(0.60, -0.071, 0.054)),
        material=dark_anodized,
        name="outer_side_1",
    )
    outer.visual(
        Box((1.20, 0.044, 0.012)),
        origin=Origin(xyz=(0.60, 0.049, 0.099)),
        material=dark_anodized,
        name="outer_lip_0",
    )
    outer.visual(
        Box((1.20, 0.044, 0.012)),
        origin=Origin(xyz=(0.60, -0.049, 0.099)),
        material=dark_anodized,
        name="outer_lip_1",
    )
    for i, x in enumerate((0.14, 0.42, 0.78, 1.06)):
        outer.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(x, 0.092, 0.004)),
            material=black_oxide,
            name=f"fastener_top_{i}",
        )
        outer.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(x, -0.092, 0.004)),
            material=black_oxide,
            name=f"fastener_bottom_{i}",
        )

    middle = model.part("middle_rail")
    # The middle stage is another straight channel nested inside the outer member.
    middle.visual(
        Box((1.10, 0.088, 0.018)),
        origin=Origin(xyz=(0.55, 0.0, -0.022)),
        material=zinc_middle,
        name="middle_web",
    )
    middle.visual(
        Box((1.10, 0.014, 0.050)),
        origin=Origin(xyz=(0.55, 0.037, 0.000)),
        material=zinc_middle,
        name="middle_side_0",
    )
    middle.visual(
        Box((1.10, 0.014, 0.050)),
        origin=Origin(xyz=(0.55, -0.037, 0.000)),
        material=zinc_middle,
        name="middle_side_1",
    )
    middle.visual(
        Box((1.10, 0.028, 0.010)),
        origin=Origin(xyz=(0.55, 0.020, 0.025)),
        material=zinc_middle,
        name="middle_lip_0",
    )
    middle.visual(
        Box((1.10, 0.028, 0.010)),
        origin=Origin(xyz=(0.55, -0.020, 0.025)),
        material=zinc_middle,
        name="middle_lip_1",
    )

    inner = model.part("inner_rail")
    # A narrow rectilinear blade and top strip run through the middle channel.
    inner.visual(
        Box((1.05, 0.012, 0.048)),
        origin=Origin(xyz=(0.525, 0.0, -0.002)),
        material=bright_steel,
        name="inner_blade",
    )
    inner.visual(
        Box((1.05, 0.050, 0.012)),
        origin=Origin(xyz=(0.525, 0.0, 0.0275)),
        material=bright_steel,
        name="inner_top_strip",
    )
    inner.visual(
        Box((1.05, 0.020, 0.010)),
        origin=Origin(xyz=(0.525, 0.0, -0.0305)),
        material=bright_steel,
        name="inner_guide_shoe",
    )

    carriage = model.part("top_carriage")
    carriage.visual(
        Box((0.160, 0.085, 0.032)),
        origin=Origin(xyz=(0.070, 0.0, 0.016)),
        material=carriage_orange,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.090, 0.050, 0.020)),
        origin=Origin(xyz=(0.085, 0.0, 0.0415)),
        material=carriage_orange,
        name="top_saddle",
    )
    carriage.visual(
        Box((0.018, 0.095, 0.046)),
        origin=Origin(xyz=(0.151, 0.0, 0.023)),
        material=carriage_orange,
        name="end_stop",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.20, 0.0, 0.049)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=160.0, velocity=0.35),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.35, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.50, effort=120.0, velocity=0.35),
    )
    model.articulation(
        "inner_to_carriage",
        ArticulationType.FIXED,
        parent=inner,
        child=carriage,
        origin=Origin(xyz=(1.05, 0.0, 0.0335)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_rail")
    inner = object_model.get_part("inner_rail")
    carriage = object_model.get_part("top_carriage")
    mid_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "serial prismatic stages share the rail axis",
        mid_slide.axis == (1.0, 0.0, 0.0) and inner_slide.axis == (1.0, 0.0, 0.0),
        details=f"middle axis={mid_slide.axis}, inner axis={inner_slide.axis}",
    )

    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.90,
        name="collapsed middle remains deeply inserted in outer member",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.70,
        name="collapsed inner remains deeply inserted in middle rail",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="y",
        margin=0.0,
        name="middle rail stays laterally inside outer member",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="y",
        margin=0.0,
        name="inner rail stays laterally inside middle channel",
    )
    ctx.expect_contact(
        carriage,
        inner,
        elem_a="carriage_block",
        elem_b="inner_top_strip",
        contact_tol=0.001,
        name="carriage is seated on inner rail tip",
    )

    rest_carriage_position = ctx.part_world_position(carriage)
    with ctx.pose({mid_slide: 0.55, inner_slide: 0.50}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.40,
            name="extended middle retains overlap with outer member",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.20,
            name="extended inner retains overlap with middle rail",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            margin=0.0,
            name="extended middle remains on the same straight axis",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            margin=0.0,
            name="extended inner remains on the same straight axis",
        )
        extended_carriage_position = ctx.part_world_position(carriage)

    ctx.check(
        "top carriage extends with both serial stages",
        rest_carriage_position is not None
        and extended_carriage_position is not None
        and extended_carriage_position[0] > rest_carriage_position[0] + 0.95,
        details=f"rest={rest_carriage_position}, extended={extended_carriage_position}",
    )

    return ctx.report()


object_model = build_object_model()
