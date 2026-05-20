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
    model = ArticulatedObject(name="two_stage_linear_stack")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_rail = model.material("brushed_rail", rgba=(0.62, 0.64, 0.66, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.12, 0.24, 0.42, 1.0))
    carriage_orange = model.material("carriage_orange", rgba=(0.90, 0.42, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.02, 0.02, 0.018, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        Box((0.82, 0.24, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    for y in (-0.086, 0.086):
        outer_guide.visual(
            Box((0.76, 0.028, 0.101)),
            origin=Origin(xyz=(0.0, y, 0.0745)),
            material=brushed_rail,
            name=f"side_rail_{'neg' if y < 0 else 'pos'}",
        )
    for y in (-0.053, 0.053):
        outer_guide.visual(
            Box((0.76, 0.040, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.116)),
            material=brushed_rail,
            name=f"capture_lip_{'neg' if y < 0 else 'pos'}",
        )
    outer_guide.visual(
        Box((0.040, 0.22, 0.105)),
        origin=Origin(xyz=(-0.390, 0.0, 0.0775)),
        material=dark_steel,
        name="rear_stop",
    )
    for x in (-0.34, 0.34):
        for y in (-0.095, 0.095):
            outer_guide.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, y, 0.0285)),
                material=rubber_black,
                name=f"anchor_head_{'rear' if x < 0 else 'front'}_{'neg' if y < 0 else 'pos'}",
            )

    middle_carriage = model.part("middle_carriage")
    middle_carriage.visual(
        Box((0.56, 0.118, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="lower_shoe",
    )
    middle_carriage.visual(
        Box((0.56, 0.052, 0.077)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=carriage_blue,
        name="center_neck",
    )
    middle_carriage.visual(
        Box((0.56, 0.116, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=carriage_blue,
        name="upper_deck",
    )
    for y in (-0.049, 0.049):
        middle_carriage.visual(
            Box((0.54, 0.018, 0.041)),
            origin=Origin(xyz=(0.0, y, 0.1325)),
            material=brushed_rail,
            name=f"inner_rail_{'neg' if y < 0 else 'pos'}",
        )

    inner_carriage = model.part("inner_carriage")
    inner_carriage.visual(
        Box((0.38, 0.064, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_orange,
        name="lower_slide",
    )
    inner_carriage.visual(
        Box((0.34, 0.040, 0.027)),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=carriage_orange,
        name="raised_tongue",
    )
    inner_carriage.visual(
        Box((0.34, 0.096, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=carriage_orange,
        name="top_plate",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=middle_carriage,
        origin=Origin(xyz=(-0.080, 0.0, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.35, lower=0.0, upper=0.30),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_carriage,
        child=inner_carriage,
        origin=Origin(xyz=(-0.060, 0.0, 0.126)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_carriage")
    inner = object_model.get_part("inner_carriage")
    outer_joint = object_model.get_articulation("outer_to_middle")
    inner_joint = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "two serial prismatic joints",
        len(object_model.articulations) == 2
        and outer_joint.articulation_type == ArticulationType.PRISMATIC
        and inner_joint.articulation_type == ArticulationType.PRISMATIC
        and outer_joint.parent == "outer_guide"
        and outer_joint.child == "middle_carriage"
        and inner_joint.parent == "middle_carriage"
        and inner_joint.child == "inner_carriage",
        details="Expected outer guide -> middle carriage -> inner carriage prismatic chain.",
    )

    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        positive_elem="lower_shoe",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=1e-6,
        name="middle shoe rides on grounded base",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="z",
        positive_elem="lower_slide",
        negative_elem="upper_deck",
        max_gap=0.001,
        max_penetration=1e-6,
        name="inner slide rides on middle deck",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="y",
        inner_elem="lower_shoe",
        outer_elem="base_plate",
        margin=0.0,
        name="middle carriage is laterally inside outer guide",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="y",
        inner_elem="lower_slide",
        outer_elem="upper_deck",
        margin=0.0,
        name="inner carriage is laterally inside middle guide",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="lower_shoe",
        elem_b="base_plate",
        min_overlap=0.40,
        name="middle carriage retained in outer guide at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="lower_slide",
        elem_b="upper_deck",
        min_overlap=0.25,
        name="inner carriage retained in middle guide at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({outer_joint: 0.30, inner_joint: 0.20}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="lower_shoe",
            elem_b="base_plate",
            min_overlap=0.30,
            name="middle carriage remains inserted at full travel",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="lower_slide",
            elem_b="upper_deck",
            min_overlap=0.20,
            name="inner carriage remains inserted at full travel",
        )
        middle_extended = ctx.part_world_position(middle)
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "both carriages extend along positive x",
        middle_rest is not None
        and inner_rest is not None
        and middle_extended is not None
        and inner_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.25
        and inner_extended[0] > inner_rest[0] + 0.45,
        details=f"middle_rest={middle_rest}, middle_extended={middle_extended}, "
        f"inner_rest={inner_rest}, inner_extended={inner_extended}",
    )

    return ctx.report()


object_model = build_object_model()
