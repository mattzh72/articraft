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
    model = ArticulatedObject(name="three_joint_prismatic_chain")

    graphite = Material("graphite_powder_coat", rgba=(0.06, 0.07, 0.08, 1.0))
    steel = Material("brushed_steel", rgba=(0.65, 0.67, 0.68, 1.0))
    blue = Material("blue_anodized_carriage", rgba=(0.05, 0.22, 0.62, 1.0))
    dark_blue = Material("dark_blue_bearing", rgba=(0.03, 0.10, 0.32, 1.0))
    warning = Material("yellow_tip_marker", rgba=(1.0, 0.74, 0.08, 1.0))

    # Grounded X-axis guide: a machine base plate with two raised rectangular ways.
    base_guide = model.part("base_guide")
    base_guide.visual(
        Box((1.20, 0.30, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=graphite,
        name="base_plate",
    )
    for rib_name, rail_name, y in (
        ("rail_rib_0", "base_rail_0", -0.080),
        ("rail_rib_1", "base_rail_1", 0.080),
    ):
        base_guide.visual(
            Box((1.08, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.070)),
            material=graphite,
            name=rib_name,
        )
        base_guide.visual(
            Box((1.08, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.1025)),
            material=steel,
            name=rail_name,
        )
    for name, x in (("low_stop", -0.555), ("high_stop", 0.555)):
        base_guide.visual(
            Box((0.035, 0.25, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=graphite,
            name=name,
        )

    # First moving member: an X carriage riding on the base ways and carrying a
    # Y-axis guide on its upper deck.
    first_carriage = model.part("first_carriage")
    for shoe_name, y in (("base_shoe_0", -0.080), ("base_shoe_1", 0.080)):
        first_carriage.visual(
            Box((0.18, 0.055, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.135)),
            material=dark_blue,
            name=shoe_name,
        )
    first_carriage.visual(
        Box((0.22, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1675)),
        material=blue,
        name="lower_saddle",
    )
    first_carriage.visual(
        Box((0.18, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1975)),
        material=blue,
        name="raised_deck",
    )
    first_carriage.visual(
        Box((0.20, 0.54, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.220)),
        material=blue,
        name="cross_bed",
    )
    for rail_name, x in (("cross_rail_0", -0.060), ("cross_rail_1", 0.060)):
        first_carriage.visual(
            Box((0.030, 0.50, 0.030)),
            origin=Origin(xyz=(x, 0.020, 0.245)),
            material=steel,
            name=rail_name,
        )
    for name, y in (("near_y_stop", -0.250), ("far_y_stop", 0.290)):
        first_carriage.visual(
            Box((0.18, 0.025, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.245)),
            material=blue,
            name=name,
        )

    # Second moving member: a Y slide whose table carries the vertical Z guide.
    second_stage = model.part("second_stage")
    for shoe_name, x in (("cross_shoe_0", -0.060), ("cross_shoe_1", 0.060)):
        second_stage.visual(
            Box((0.055, 0.14, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.2725)),
            material=dark_blue,
            name=shoe_name,
        )
    second_stage.visual(
        Box((0.20, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.3025)),
        material=blue,
        name="moving_table",
    )
    second_stage.visual(
        Box((0.18, 0.08, 0.035)),
        origin=Origin(xyz=(0.0, 0.065, 0.3375)),
        material=blue,
        name="upright_foot",
    )
    for rail_name, x in (("vertical_rail_0", -0.045), ("vertical_rail_1", 0.045)):
        second_stage.visual(
            Box((0.025, 0.025, 0.340)),
            origin=Origin(xyz=(x, 0.065, 0.525)),
            material=steel,
            name=rail_name,
        )
    second_stage.visual(
        Box((0.13, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.065, 0.7125)),
        material=blue,
        name="top_bridge",
    )

    # Third moving member: a small Z slide at the tip of the Y table.  Its back
    # face bears on the vertical rails while the yellow nose marks the output tip.
    third_stage = model.part("third_stage")
    third_stage.visual(
        Box((0.15, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, 0.030, 0.045)),
        material=dark_blue,
        name="vertical_shoe",
    )
    third_stage.visual(
        Box((0.080, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.0775, 0.010)),
        material=blue,
        name="ram_bridge",
    )
    third_stage.visual(
        Box((0.070, 0.060, 0.160)),
        origin=Origin(xyz=(0.0, 0.135, -0.055)),
        material=blue,
        name="tip_ram",
    )
    third_stage.visual(
        Box((0.10, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.182, -0.110)),
        material=warning,
        name="tip_pad",
    )

    model.articulation(
        "base_to_first",
        ArticulationType.PRISMATIC,
        parent=base_guide,
        child=first_carriage,
        origin=Origin(xyz=(-0.34, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.55),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_stage,
        origin=Origin(xyz=(0.0, -0.14, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=0.32),
    )
    model.articulation(
        "second_to_third",
        ArticulationType.PRISMATIC,
        parent=second_stage,
        child=third_stage,
        origin=Origin(xyz=(0.0, 0.065, 0.390)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_stage")
    third = object_model.get_part("third_stage")
    j0 = object_model.get_articulation("base_to_first")
    j1 = object_model.get_articulation("first_to_second")
    j2 = object_model.get_articulation("second_to_third")

    joints = (j0, j1, j2)
    ctx.check(
        "three serial prismatic joints",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in joints)
        and [j.parent for j in joints] == ["base_guide", "first_carriage", "second_stage"]
        and [j.child for j in joints] == ["first_carriage", "second_stage", "third_stage"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in joints]}",
    )

    # The bearing shoes are seated on their respective guide rails at the
    # collapsed pose, so each moving member has a visible supported axis.
    ctx.expect_contact(
        first,
        base,
        elem_a="base_shoe_0",
        elem_b="base_rail_0",
        contact_tol=0.001,
        name="first carriage shoe contacts base rail",
    )
    ctx.expect_contact(
        second,
        first,
        elem_a="cross_shoe_0",
        elem_b="cross_rail_0",
        contact_tol=0.001,
        name="second stage shoe contacts cross rail",
    )
    ctx.expect_contact(
        third,
        second,
        elem_a="vertical_shoe",
        elem_b="vertical_rail_0",
        contact_tol=0.001,
        name="third stage shoe contacts vertical rail",
    )

    # At maximum travel the stages are still retained on their guide axes.
    with ctx.pose({j0: 0.55, j1: 0.32, j2: 0.22}):
        ctx.expect_overlap(
            first,
            base,
            axes="x",
            elem_a="base_shoe_0",
            elem_b="base_rail_0",
            min_overlap=0.12,
            name="first carriage remains on x guide",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="y",
            elem_a="cross_shoe_0",
            elem_b="cross_rail_0",
            min_overlap=0.10,
            name="second stage remains on y guide",
        )
        ctx.expect_overlap(
            third,
            second,
            axes="z",
            elem_a="vertical_shoe",
            elem_b="vertical_rail_0",
            min_overlap=0.08,
            name="third stage remains on z guide",
        )

    return ctx.report()


object_model = build_object_model()
