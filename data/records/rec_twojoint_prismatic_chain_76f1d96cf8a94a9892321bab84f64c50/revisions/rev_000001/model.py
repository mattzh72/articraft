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
    model = ArticulatedObject(name="compact_nested_two_stage_slide")

    anodized_black = Material("anodized_black", rgba=(0.04, 0.045, 0.05, 1.0))
    blue_aluminum = Material("blue_aluminum", rgba=(0.08, 0.22, 0.46, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    low_friction = Material("low_friction_liner", rgba=(0.01, 0.012, 0.014, 1.0))
    stop_rubber = Material("stop_rubber", rgba=(0.02, 0.018, 0.016, 1.0))

    outer = model.part("outer_guide")
    outer.visual(
        Box((0.42, 0.12, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=anodized_black,
        name="base_plate",
    )
    outer.visual(
        Box((0.42, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, 0.053, 0.0275)),
        material=anodized_black,
        name="side_rail_pos",
    )
    outer.visual(
        Box((0.42, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, -0.053, 0.0275)),
        material=anodized_black,
        name="side_rail_neg",
    )
    outer.visual(
        Box((0.016, 0.12, 0.055)),
        origin=Origin(xyz=(-0.202, 0.0, 0.0275)),
        material=anodized_black,
        name="rear_stop",
    )
    outer.visual(
        Box((0.006, 0.082, 0.020)),
        origin=Origin(xyz=(-0.191, 0.0, 0.027)),
        material=stop_rubber,
        name="rear_bumper",
    )

    middle = model.part("middle_carriage")
    middle.visual(
        Box((0.28, 0.078, 0.018)),
        origin=Origin(),
        material=blue_aluminum,
        name="middle_body",
    )
    middle.visual(
        Box((0.24, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.042, 0.0)),
        material=low_friction,
        name="outer_shoe_pos",
    )
    middle.visual(
        Box((0.24, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.042, 0.0)),
        material=low_friction,
        name="outer_shoe_neg",
    )
    middle.visual(
        Box((0.27, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.030, 0.0175)),
        material=blue_aluminum,
        name="inner_rail_pos",
    )
    middle.visual(
        Box((0.27, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, 0.0175)),
        material=blue_aluminum,
        name="inner_rail_neg",
    )
    middle.visual(
        Box((0.010, 0.070, 0.010)),
        origin=Origin(xyz=(-0.135, 0.0, 0.017)),
        material=stop_rubber,
        name="middle_bumper",
    )

    inner = model.part("inner_carriage")
    inner.visual(
        Box((0.17, 0.041, 0.016)),
        origin=Origin(),
        material=satin_steel,
        name="inner_body",
    )
    inner.visual(
        Box((0.15, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=low_friction,
        name="inner_shoe_pos",
    )
    inner.visual(
        Box((0.15, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=low_friction,
        name="inner_shoe_neg",
    )
    inner.visual(
        Box((0.030, 0.030, 0.006)),
        origin=Origin(xyz=(0.075, 0.0, 0.011)),
        material=satin_steel,
        name="front_pull_tab",
    )

    model.articulation(
        "outer_to_middle_slide",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(-0.050, 0.0, 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.160),
    )
    model.articulation(
        "middle_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(-0.045, 0.0, 0.017)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.30, lower=0.0, upper=0.110),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_carriage")
    inner = object_model.get_part("inner_carriage")
    outer_slide = object_model.get_articulation("outer_to_middle_slide")
    inner_slide = object_model.get_articulation("middle_to_inner_slide")

    ctx.check(
        "two serial prismatic joints",
        len(object_model.articulations) == 2
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="middle_body",
        elem_b="base_plate",
        name="middle carriage is seated on the outer guide floor",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_body",
        elem_b="middle_body",
        name="inner carriage is seated on the middle carriage floor",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="outer_shoe_pos",
        elem_b="side_rail_pos",
        name="middle side shoe bears against outer rail",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_shoe_pos",
        elem_b="inner_rail_pos",
        name="inner side shoe bears against middle rail",
    )

    with ctx.pose({outer_slide: 0.160, inner_slide: 0.110}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.18,
            elem_a="middle_body",
            elem_b="base_plate",
            name="extended middle carriage remains captured in the outer guide",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.12,
            elem_a="inner_body",
            elem_b="middle_body",
            name="extended inner carriage remains captured in the middle carriage",
        )

    return ctx.report()


object_model = build_object_model()
