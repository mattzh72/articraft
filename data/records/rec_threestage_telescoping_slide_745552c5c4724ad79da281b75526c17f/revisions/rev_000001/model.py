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
    model = ArticulatedObject(name="compact_three_stage_drawer_slide")

    dark_steel = Material("dark_burnished_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    zinc = Material("zinc_plated_runner", rgba=(0.62, 0.64, 0.62, 1.0))
    bright_steel = Material("polished_inner_rail", rgba=(0.78, 0.80, 0.78, 1.0))
    black_plastic = Material("black_plastic_stops", rgba=(0.015, 0.015, 0.014, 1.0))
    shadow = Material("dark_recess_shadow", rgba=(0.02, 0.022, 0.025, 1.0))

    outer = model.part("outer_channel")
    outer.visual(
        Box((0.360, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_steel,
        name="outer_web",
    )
    for side, y in (("side_0", -0.037), ("side_1", 0.037)):
        outer.visual(
            Box((0.360, 0.006, 0.054)),
            origin=Origin(xyz=(0.0, y, 0.033)),
            material=dark_steel,
            name=f"outer_{side}_wall",
        )
    for side, y in (("side_0", -0.027), ("side_1", 0.027)):
        outer.visual(
            Box((0.360, 0.014, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.058)),
            material=dark_steel,
            name=f"outer_{side}_rolled_lip",
        )
    # Darkened end stop and elongated mounting-slot decals make the fixed member
    # read as a stamped drawer-slide channel rather than a plain rectangular rail.
    outer.visual(
        Box((0.006, 0.060, 0.026)),
        origin=Origin(xyz=(-0.177, 0.0, 0.019)),
        material=black_plastic,
        name="outer_rear_stop",
    )
    for x in (-0.105, 0.080):
        outer.visual(
            Box((0.046, 0.0015, 0.014)),
            origin=Origin(xyz=(x, -0.0400, 0.036)),
            material=shadow,
            name=f"outer_slot_{x:+.3f}".replace("+", "p").replace("-", "m").replace(".", "_"),
        )

    middle = model.part("middle_runner")
    middle.visual(
        Box((0.310, 0.068, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=zinc,
        name="middle_web",
    )
    for side, y in (("side_0", -0.0295), ("side_1", 0.0295)):
        middle.visual(
            Box((0.310, 0.009, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.026)),
            material=zinc,
            name=f"middle_{side}_wall",
        )
    for side, y in (("side_0", -0.020), ("side_1", 0.020)):
        middle.visual(
            Box((0.310, 0.010, 0.0035)),
            origin=Origin(xyz=(0.0, y, 0.04025)),
            material=zinc,
            name=f"middle_{side}_inward_lip",
        )
    middle.visual(
        Box((0.008, 0.050, 0.020)),
        origin=Origin(xyz=(0.151, 0.0, 0.022)),
        material=black_plastic,
        name="middle_front_stop",
    )
    middle.visual(
        Box((0.008, 0.050, 0.020)),
        origin=Origin(xyz=(-0.151, 0.0, 0.022)),
        material=black_plastic,
        name="middle_rear_stop",
    )

    inner = model.part("inner_runner")
    inner.visual(
        Box((0.260, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bright_steel,
        name="inner_bar",
    )
    inner.visual(
        Box((0.260, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=bright_steel,
        name="inner_top_rib",
    )
    inner.visual(
        Box((0.010, 0.044, 0.022)),
        origin=Origin(xyz=(0.125, 0.0, 0.022)),
        material=black_plastic,
        name="inner_front_stop",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_contact(
        middle,
        outer,
        elem_a="middle_web",
        elem_b="outer_web",
        name="middle runner rides on the fixed channel web",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        max_penetration=0.0,
        positive_elem="middle_web",
        negative_elem="outer_web",
        name="middle web touches outer web without penetration",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_bar",
        elem_b="middle_web",
        name="inner runner rides on the middle channel web",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="z",
        max_penetration=0.0,
        positive_elem="inner_bar",
        negative_elem="middle_web",
        name="inner bar touches middle web without penetration",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.001,
        name="middle cross section remains nested in outer channel",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.001,
        name="inner cross section remains nested in middle runner",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.25,
        name="middle and outer have long overlap when collapsed",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.22,
        name="inner and middle have long overlap when collapsed",
    )

    rest_middle_pos = ctx.part_world_position(middle)
    rest_inner_pos = ctx.part_world_position(inner)
    with ctx.pose({outer_to_middle: 0.120, middle_to_inner: 0.100}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.001,
            name="extended middle stays captured in outer channel",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.001,
            name="extended inner stays captured in middle runner",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.18,
            name="extended middle retains visible overlap with outer",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.16,
            name="extended inner retains visible overlap with middle",
        )
        extended_middle_pos = ctx.part_world_position(middle)
        extended_inner_pos = ctx.part_world_position(inner)

    ctx.check(
        "serial slides extend in the same drawer direction",
        rest_middle_pos is not None
        and rest_inner_pos is not None
        and extended_middle_pos is not None
        and extended_inner_pos is not None
        and extended_middle_pos[0] > rest_middle_pos[0] + 0.10
        and extended_inner_pos[0] > rest_inner_pos[0] + 0.20
        and extended_inner_pos[0] > extended_middle_pos[0] + 0.09,
        details=(
            f"rest_middle={rest_middle_pos}, extended_middle={extended_middle_pos}, "
            f"rest_inner={rest_inner_pos}, extended_inner={extended_inner_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
