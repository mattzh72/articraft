from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_three_stage_slide")

    dark_steel = model.material("dark_steel", color=(0.18, 0.19, 0.19, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.62, 0.64, 0.63, 1.0))
    satin_steel = model.material("satin_steel", color=(0.44, 0.47, 0.48, 1.0))
    inner_alloy = model.material("inner_alloy", color=(0.76, 0.68, 0.50, 1.0))
    polymer = model.material("black_polymer", color=(0.025, 0.025, 0.025, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.78, 0.22, 0.035)),
        origin=Origin(xyz=(0.39, 0.0, 0.0175)),
        material=dark_steel,
        name="mount_plate",
    )
    support.visual(
        Box((0.64, 0.060, 0.095)),
        origin=Origin(xyz=(0.34, 0.0, -0.0475)),
        material=dark_steel,
        name="center_hanger",
    )
    support.visual(
        Box((0.74, 0.026, 0.085)),
        origin=Origin(xyz=(0.38, 0.094, -0.039)),
        material=dark_steel,
        name="side_cheek_pos",
    )
    support.visual(
        Box((0.74, 0.026, 0.085)),
        origin=Origin(xyz=(0.38, -0.094, -0.039)),
        material=dark_steel,
        name="side_cheek_neg",
    )

    outer = model.part("outer_stage")
    outer.visual(
        Box((0.61, 0.150, 0.018)),
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        material=brushed_steel,
        name="outer_web",
    )
    outer.visual(
        Box((0.61, 0.020, 0.058)),
        origin=Origin(xyz=(0.305, 0.065, -0.027)),
        material=brushed_steel,
        name="outer_wall_pos",
    )
    outer.visual(
        Box((0.61, 0.020, 0.058)),
        origin=Origin(xyz=(0.305, -0.065, -0.027)),
        material=brushed_steel,
        name="outer_wall_neg",
    )
    outer.visual(
        Box((0.61, 0.026, 0.014)),
        origin=Origin(xyz=(0.305, 0.049, -0.060)),
        material=brushed_steel,
        name="outer_lip_pos",
    )
    outer.visual(
        Box((0.61, 0.026, 0.014)),
        origin=Origin(xyz=(0.305, -0.049, -0.060)),
        material=brushed_steel,
        name="outer_lip_neg",
    )
    outer.visual(
        Box((0.030, 0.145, 0.045)),
        origin=Origin(xyz=(0.020, 0.0, -0.030)),
        material=polymer,
        name="outer_stop",
    )

    middle = model.part("middle_stage")
    middle.visual(
        Box((0.56, 0.110, 0.016)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=satin_steel,
        name="middle_web",
    )
    middle.visual(
        Box((0.56, 0.018, 0.046)),
        origin=Origin(xyz=(0.280, 0.047, -0.022)),
        material=satin_steel,
        name="middle_wall_pos",
    )
    middle.visual(
        Box((0.56, 0.018, 0.046)),
        origin=Origin(xyz=(0.280, -0.047, -0.022)),
        material=satin_steel,
        name="middle_wall_neg",
    )
    middle.visual(
        Box((0.56, 0.020, 0.012)),
        origin=Origin(xyz=(0.280, 0.036, -0.051)),
        material=satin_steel,
        name="middle_lip_pos",
    )
    middle.visual(
        Box((0.56, 0.020, 0.012)),
        origin=Origin(xyz=(0.280, -0.036, -0.051)),
        material=satin_steel,
        name="middle_lip_neg",
    )
    middle.visual(
        Box((0.026, 0.105, 0.036)),
        origin=Origin(xyz=(0.018, 0.0, -0.024)),
        material=polymer,
        name="middle_stop",
    )

    inner = model.part("inner_stage")
    inner.visual(
        Box((0.50, 0.074, 0.014)),
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
        material=inner_alloy,
        name="inner_web",
    )
    inner.visual(
        Box((0.50, 0.015, 0.034)),
        origin=Origin(xyz=(0.250, 0.030, -0.016)),
        material=inner_alloy,
        name="inner_wall_pos",
    )
    inner.visual(
        Box((0.50, 0.015, 0.034)),
        origin=Origin(xyz=(0.250, -0.030, -0.016)),
        material=inner_alloy,
        name="inner_wall_neg",
    )
    inner.visual(
        Box((0.080, 0.090, 0.020)),
        origin=Origin(xyz=(0.510, 0.0, -0.018)),
        material=inner_alloy,
        name="load_tab",
    )
    inner.visual(
        Box((0.020, 0.084, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, -0.016)),
        material=polymer,
        name="inner_stop",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.PRISMATIC,
        parent=support,
        child=outer,
        origin=Origin(xyz=(0.050, 0.0, -0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.040, 0.0, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.28),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.035, 0.0, -0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.24),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    support_to_outer = object_model.get_articulation("support_to_outer")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    slide_joints = (support_to_outer, outer_to_middle, middle_to_inner)
    ctx.check(
        "three serial prismatic joints",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in slide_joints)
        and [joint.parent for joint in slide_joints] == ["top_support", "outer_stage", "middle_stage"]
        and [joint.child for joint in slide_joints] == ["outer_stage", "middle_stage", "inner_stage"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in slide_joints]}",
    )

    ctx.expect_gap(
        support,
        outer,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="center_hanger",
        negative_elem="outer_web",
        name="outer stage hangs from support",
    )
    ctx.expect_gap(
        outer,
        middle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="outer_lip_pos",
        negative_elem="middle_web",
        name="middle stage rides under outer lip",
    )
    ctx.expect_gap(
        middle,
        inner,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="middle_lip_pos",
        negative_elem="inner_web",
        name="inner stage rides under middle lip",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="y",
        margin=0.001,
        name="middle stage nests inside outer width",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="y",
        margin=0.001,
        name="inner stage nests inside middle width",
    )

    rest_inner_pos = ctx.part_world_position(inner)
    with ctx.pose({support_to_outer: 0.32, outer_to_middle: 0.28, middle_to_inner: 0.24}):
        ctx.expect_overlap(
            outer,
            support,
            axes="x",
            min_overlap=0.25,
            elem_a="outer_web",
            elem_b="center_hanger",
            name="outer stage remains captured at extension",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.25,
            elem_a="middle_web",
            elem_b="outer_web",
            name="middle stage remains captured at extension",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.22,
            elem_a="inner_web",
            elem_b="middle_web",
            name="inner stage remains captured at extension",
        )
        extended_inner_pos = ctx.part_world_position(inner)

    ctx.check(
        "tip extends along root to tip axis",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[0] > rest_inner_pos[0] + 0.80,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )

    return ctx.report()


object_model = build_object_model()
