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


def _add_box_visuals(part, specs, *, material) -> None:
    for visual_name, size, center in specs:
        part.visual(
            Box(size),
            origin=Origin(xyz=center),
            material=material,
            name=visual_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_carriage_slide_chain")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_rail = model.material("dark_hardcoat", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite_slider", rgba=(0.02, 0.025, 0.03, 1.0))
    blue = model.material("blue_anodized_stage", rgba=(0.10, 0.22, 0.42, 1.0))
    red = model.material("red_tip_pad", rgba=(0.75, 0.08, 0.04, 1.0))

    outer_guide = model.part("outer_guide")
    _add_box_visuals(
        outer_guide,
        (
            ("guide_0", (0.90, 0.36, 0.016), (0.45, 0.0, 0.008)),
            ("guide_1", (0.90, 0.28, 0.034), (0.45, 0.0, 0.032)),
            ("guide_2", (0.90, 0.036, 0.096), (0.45, 0.122, 0.082)),
            ("guide_3", (0.90, 0.036, 0.096), (0.45, -0.122, 0.082)),
            ("guide_4", (0.90, 0.038, 0.014), (0.45, 0.087, 0.123)),
            ("guide_5", (0.90, 0.038, 0.014), (0.45, -0.087, 0.123)),
            ("guide_6", (0.035, 0.20, 0.050), (0.018, 0.0, 0.061)),
        ),
        material=aluminum,
    )
    for idx, (x, y) in enumerate(((0.12, 0.15), (0.78, 0.15), (0.12, -0.15), (0.78, -0.15))):
        outer_guide.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.018)),
            material=dark_rail,
            name=f"screw_head_{idx}",
        )

    middle_stage = model.part("middle_stage")
    _add_box_visuals(
        middle_stage,
        (
            ("middle_carriage_0", (0.66, 0.146, 0.034), (0.31, 0.0, 0.002)),
            ("middle_carriage_1", (0.66, 0.062, 0.078), (0.31, 0.0, 0.058)),
            ("middle_carriage_2", (0.66, 0.182, 0.024), (0.31, 0.0, 0.108)),
            ("middle_carriage_3", (0.66, 0.026, 0.020), (0.31, 0.075, 0.128)),
            ("middle_carriage_4", (0.66, 0.026, 0.020), (0.31, -0.075, 0.128)),
            ("middle_carriage_5", (0.035, 0.146, 0.048), (-0.002, 0.0, 0.010)),
        ),
        material=blue,
    )
    middle_stage.visual(
        Box((0.62, 0.010, 0.020)),
        origin=Origin(xyz=(0.31, 0.078, 0.000)),
        material=graphite,
        name="outer_bearing_pad_0",
    )
    middle_stage.visual(
        Box((0.62, 0.010, 0.020)),
        origin=Origin(xyz=(0.31, -0.078, 0.000)),
        material=graphite,
        name="outer_bearing_pad_1",
    )

    inner_stage = model.part("inner_stage")
    _add_box_visuals(
        inner_stage,
        (
            ("inner_carriage_0", (0.48, 0.080, 0.020), (0.225, 0.0, 0.004)),
            ("inner_carriage_1", (0.48, 0.106, 0.030), (0.225, 0.0, 0.031)),
            ("inner_carriage_2", (0.48, 0.016, 0.022), (0.225, 0.043, 0.056)),
            ("inner_carriage_3", (0.48, 0.016, 0.022), (0.225, -0.043, 0.056)),
            ("inner_carriage_4", (0.030, 0.096, 0.036), (-0.002, 0.0, 0.025)),
        ),
        material=aluminum,
    )
    inner_stage.visual(
        Box((0.44, 0.012, 0.012)),
        origin=Origin(xyz=(0.225, 0.056, 0.030)),
        material=graphite,
        name="middle_bearing_pad_0",
    )
    inner_stage.visual(
        Box((0.44, 0.012, 0.012)),
        origin=Origin(xyz=(0.225, -0.056, 0.030)),
        material=graphite,
        name="middle_bearing_pad_1",
    )

    tip_stage = model.part("tip_stage")
    _add_box_visuals(
        tip_stage,
        (
            ("tip_carriage_0", (0.22, 0.048, 0.014), (0.105, 0.0, -0.004)),
            ("tip_carriage_1", (0.22, 0.066, 0.040), (0.105, 0.0, 0.020)),
            ("tip_carriage_2", (0.036, 0.060, 0.048), (0.215, 0.0, 0.022)),
        ),
        material=dark_rail,
    )
    tip_stage.visual(
        Box((0.012, 0.060, 0.046)),
        origin=Origin(xyz=(0.228, 0.0, 0.022)),
        material=red,
        name="tip_pad",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=middle_stage,
        origin=Origin(xyz=(0.11, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.36, effort=180.0, velocity=0.45),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.08, 0.0, 0.126)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.28, effort=120.0, velocity=0.40),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner_stage,
        child=tip_stage,
        origin=Origin(xyz=(0.06, 0.0, 0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.16, effort=70.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    tip = object_model.get_part("tip_stage")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_tip = object_model.get_articulation("inner_to_tip")
    joints = (outer_to_middle, middle_to_inner, inner_to_tip)

    ctx.check(
        "three serial prismatic joints",
        len(object_model.articulations) == 3
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in joints)
        and outer_to_middle.parent == "outer_guide"
        and outer_to_middle.child == "middle_stage"
        and middle_to_inner.parent == "middle_stage"
        and middle_to_inner.child == "inner_stage"
        and inner_to_tip.parent == "inner_stage"
        and inner_to_tip.child == "tip_stage",
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "all stages slide forward on x",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    ctx.expect_contact(
        middle,
        outer,
        elem_a="middle_carriage_0",
        elem_b="guide_1",
        contact_tol=1e-5,
        name="middle shoe is seated on the outer guide floor",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_carriage_0",
        elem_b="middle_carriage_2",
        contact_tol=1e-5,
        name="inner stage is seated on the middle guide",
    )
    ctx.expect_contact(
        tip,
        inner,
        elem_a="tip_carriage_0",
        elem_b="inner_carriage_1",
        contact_tol=1e-5,
        name="tip stage is seated on the inner guide",
    )

    rest_tip_position = ctx.part_world_position(tip)
    with ctx.pose(
        {
            outer_to_middle: 0.36,
            middle_to_inner: 0.28,
            inner_to_tip: 0.16,
        }
    ):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.30,
            name="extended middle stage remains retained in the outer guide",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.18,
            name="extended inner stage remains retained in the middle stage",
        )
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            min_overlap=0.10,
            name="extended tip stage remains retained in the inner stage",
        )
        extended_tip_position = ctx.part_world_position(tip)

    ctx.check(
        "three-stage chain produces cumulative extension",
        rest_tip_position is not None
        and extended_tip_position is not None
        and extended_tip_position[0] > rest_tip_position[0] + 0.75,
        details=f"rest={rest_tip_position}, extended={extended_tip_position}",
    )

    return ctx.report()


object_model = build_object_model()
