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
    model = ArticulatedObject(name="three_stage_service_slide")

    painted_steel = model.material("painted_steel", rgba=(0.10, 0.13, 0.16, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_wear = model.material("dark_wear", rgba=(0.02, 0.02, 0.018, 1.0))
    stop_color = model.material("front_stop", rgba=(0.82, 0.48, 0.10, 1.0))

    outer = model.part("outer_guide")
    outer.visual(
        Box((0.94, 0.22, 0.025)),
        origin=Origin(xyz=(-0.35, 0.0, 0.0125)),
        material=painted_steel,
        name="ground_foot",
    )
    outer.visual(
        Box((0.82, 0.150, 0.016)),
        origin=Origin(xyz=(-0.35, 0.0, 0.045)),
        material=painted_steel,
        name="outer_floor",
    )
    for idx, y in enumerate((-0.048, 0.048)):
        outer.visual(
            Box((0.82, 0.020, 0.016)),
            origin=Origin(xyz=(-0.35, y, 0.030)),
            material=painted_steel,
            name=f"floor_riser_{idx}",
        )
    for idx, y in enumerate((-0.069, 0.069)):
        outer.visual(
            Box((0.82, 0.014, 0.100)),
            origin=Origin(xyz=(-0.35, y, 0.090)),
            material=painted_steel,
            name=f"outer_side_{idx}",
        )
        outer.visual(
            Box((0.82, 0.032, 0.012)),
            origin=Origin(xyz=(-0.35, 0.045 if y > 0 else -0.045, 0.134)),
            material=painted_steel,
            name=f"outer_lip_{idx}",
        )

    outer.visual(
        Box((0.030, 0.150, 0.105)),
        origin=Origin(xyz=(-0.775, 0.0, 0.0875)),
        material=painted_steel,
        name="rear_stop",
    )
    for idx, (x, y) in enumerate(
        ((-0.72, -0.085), (-0.72, 0.085), (0.02, -0.085), (0.02, 0.085))
    ):
        outer.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, y, 0.027)),
            material=dark_wear,
            name=f"mount_screw_{idx}",
        )

    middle = model.part("middle_runner")
    middle.visual(
        Box((0.660, 0.086, 0.012)),
        origin=Origin(xyz=(-0.300, 0.0, 0.059)),
        material=zinc_steel,
        name="middle_floor",
    )
    for idx, y in enumerate((-0.046, 0.046)):
        middle.visual(
            Box((0.660, 0.012, 0.054)),
            origin=Origin(xyz=(-0.300, y, 0.090)),
            material=zinc_steel,
            name=f"middle_side_{idx}",
        )
        middle.visual(
            Box((0.022, 0.014, 0.060)),
            origin=Origin(xyz=(0.041, y, 0.088)),
            material=stop_color,
            name=f"middle_front_cheek_{idx}",
        )
    middle.visual(
        Box((0.022, 0.108, 0.010)),
        origin=Origin(xyz=(0.041, 0.0, 0.058)),
        material=stop_color,
        name="middle_front_sill",
    )

    inner = model.part("inner_runner")
    inner.visual(
        Box((0.580, 0.044, 0.028)),
        origin=Origin(xyz=(-0.210, 0.0, 0.079)),
        material=bright_steel,
        name="inner_bar",
    )
    inner.visual(
        Box((0.030, 0.064, 0.048)),
        origin=Origin(xyz=(0.085, 0.0, 0.079)),
        material=stop_color,
        name="inner_front_face",
    )
    inner.visual(
        Box((0.120, 0.048, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.096)),
        material=dark_wear,
        name="pull_wear_pad",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.30, lower=0.0, upper=0.43),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.35, lower=0.0, upper=0.34),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.002,
        inner_elem="middle_floor",
        name="middle runner stays between outer guide walls",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="middle_floor",
        negative_elem="outer_floor",
        name="middle runner rides on outer bearing floor",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.002,
        inner_elem="inner_bar",
        name="inner runner remains centered in middle channel",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="inner_bar",
        negative_elem="middle_floor",
        name="inner runner rides on middle bearing floor",
    )

    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.55,
        elem_a="middle_floor",
        elem_b="outer_floor",
        name="collapsed middle runner has long retained insertion",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.45,
        elem_a="inner_bar",
        elem_b="middle_floor",
        name="collapsed inner runner has long retained insertion",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.43, inner_slide: 0.34}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.18,
            elem_a="middle_floor",
            elem_b="outer_floor",
            name="extended middle runner keeps practical overlap",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.12,
            elem_a="inner_bar",
            elem_b="middle_floor",
            name="extended inner runner keeps practical overlap",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.002,
            inner_elem="middle_floor",
            name="extended middle remains guided laterally",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.002,
            inner_elem="inner_bar",
            name="extended inner remains guided laterally",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "serial slide extends outward",
        rest_middle is not None
        and rest_inner is not None
        and extended_middle is not None
        and extended_inner is not None
        and extended_middle[0] > rest_middle[0] + 0.40
        and extended_inner[0] > rest_inner[0] + 0.75,
        details=f"middle {rest_middle}->{extended_middle}, inner {rest_inner}->{extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
