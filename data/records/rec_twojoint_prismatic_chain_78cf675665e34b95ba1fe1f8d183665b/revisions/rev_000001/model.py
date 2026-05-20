from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bridge_backed_double_slide")

    powder = model.material("black_powder_coat", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_rail = model.material("dark_burnished_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    bright_rail = model.material("polished_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    carriage_finish = model.material("satin_aluminum", rgba=(0.56, 0.58, 0.56, 1.0))
    slide_finish = model.material("blue_anodized_slide", rgba=(0.06, 0.24, 0.58, 1.0))
    rubber = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))

    support = model.part("rear_support")
    support.visual(
        Box((1.16, 0.54, 0.04)),
        origin=Origin(xyz=(0.46, 0.0, 0.02)),
        material=powder,
        name="base_plate",
    )
    # Portal-style rear bridge: two uprights and a top lintel back the slides.
    for index, y in enumerate((-0.225, 0.225)):
        support.visual(
            Box((0.10, 0.07, 0.60)),
            origin=Origin(xyz=(-0.055, y, 0.34)),
            material=powder,
            name=f"bridge_post_{index}",
        )
    support.visual(
        Box((0.12, 0.53, 0.08)),
        origin=Origin(xyz=(-0.055, 0.0, 0.68)),
        material=powder,
        name="bridge_lintel",
    )
    support.visual(
        Box((0.11, 0.53, 0.10)),
        origin=Origin(xyz=(-0.055, 0.0, 0.115)),
        material=powder,
        name="lower_tie",
    )
    support.visual(
        Box((0.08, 0.22, 0.18)),
        origin=Origin(xyz=(-0.005, 0.0, 0.225)),
        material=powder,
        name="center_web",
    )
    support.visual(
        Box((0.98, 0.32, 0.035)),
        origin=Origin(xyz=(0.475, 0.0, 0.3125)),
        material=dark_rail,
        name="guide_bed",
    )
    for index, y in enumerate((-0.14, 0.14)):
        support.visual(
            Box((0.98, 0.035, 0.065)),
            origin=Origin(xyz=(0.475, y, 0.3625)),
            material=bright_rail,
            name=f"guide_side_{index}",
        )

    long_carriage = model.part("long_carriage")
    long_carriage.visual(
        Box((0.84, 0.18, 0.055)),
        origin=Origin(xyz=(0.42, 0.0, 0.0275)),
        material=carriage_finish,
        name="main_beam",
    )
    long_carriage.visual(
        Box((0.030, 0.20, 0.055)),
        origin=Origin(xyz=(0.015, 0.0, 0.0275)),
        material=carriage_finish,
        name="rear_cap",
    )
    long_carriage.visual(
        Box((0.045, 0.20, 0.055)),
        origin=Origin(xyz=(0.845, 0.0, 0.0275)),
        material=carriage_finish,
        name="front_cap",
    )
    for rail_name, y in (("top_rail_0", -0.065), ("top_rail_1", 0.065)):
        long_carriage.visual(
            Box((0.76, 0.030, 0.035)),
            origin=Origin(xyz=(0.45, y, 0.0725)),
            material=bright_rail,
            name=rail_name,
        )
    long_carriage.visual(
        Box((0.18, 0.11, 0.018)),
        origin=Origin(xyz=(0.17, 0.0, 0.064)),
        material=dark_rail,
        name="index_plate",
    )

    short_slide = model.part("short_slide")
    short_slide.visual(
        Box((0.44, 0.20, 0.05)),
        origin=Origin(xyz=(0.22, 0.0, 0.025)),
        material=slide_finish,
        name="sled_plate",
    )
    for index, y in enumerate((-0.115, 0.115)):
        short_slide.visual(
            Box((0.40, 0.035, 0.045)),
            origin=Origin(xyz=(0.22, y, 0.0075)),
            material=slide_finish,
            name=f"capture_lip_{index}",
        )
    short_slide.visual(
        Box((0.24, 0.12, 0.035)),
        origin=Origin(xyz=(0.28, 0.0, 0.0675)),
        material=slide_finish,
        name="raised_pad",
    )
    short_slide.visual(
        Box((0.06, 0.07, 0.05)),
        origin=Origin(xyz=(0.465, 0.0, 0.050)),
        material=slide_finish,
        name="handle_stem",
    )
    short_slide.visual(
        Cylinder(radius=0.018, length=0.23),
        origin=Origin(xyz=(0.500, 0.0, 0.068), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="handle_bar",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support,
        child=long_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=long_carriage,
        child=short_slide,
        origin=Origin(xyz=(0.16, 0.0, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.30, lower=0.0, upper=0.27),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    long_carriage = object_model.get_part("long_carriage")
    short_slide = object_model.get_part("short_slide")
    support_joint = object_model.get_articulation("support_to_carriage")
    carried_joint = object_model.get_articulation("carriage_to_slide")

    ctx.check(
        "two serial prismatic slides",
        len(object_model.articulations) == 2
        and support_joint.articulation_type == ArticulationType.PRISMATIC
        and carried_joint.articulation_type == ArticulationType.PRISMATIC
        and support_joint.parent == "rear_support"
        and support_joint.child == "long_carriage"
        and carried_joint.parent == "long_carriage"
        and carried_joint.child == "short_slide",
        details=(
            f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in object_model.articulations]}"
        ),
    )

    ctx.expect_gap(
        long_carriage,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="main_beam",
        negative_elem="guide_bed",
        name="long carriage rides on support bed",
    )
    ctx.expect_gap(
        short_slide,
        long_carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="sled_plate",
        negative_elem="top_rail_0",
        name="carried slide rides on carriage rail",
    )
    ctx.expect_overlap(
        long_carriage,
        support,
        axes="x",
        min_overlap=0.70,
        elem_a="main_beam",
        elem_b="guide_bed",
        name="collapsed long carriage remains deeply supported",
    )
    ctx.expect_overlap(
        short_slide,
        long_carriage,
        axes="x",
        min_overlap=0.35,
        elem_a="sled_plate",
        elem_b="top_rail_0",
        name="collapsed carried slide remains on its rail",
    )

    rest_long = ctx.part_world_position(long_carriage)
    rest_short = ctx.part_world_position(short_slide)
    with ctx.pose({support_joint: 0.32, carried_joint: 0.27}):
        extended_long = ctx.part_world_position(long_carriage)
        extended_short = ctx.part_world_position(short_slide)
        ctx.expect_overlap(
            long_carriage,
            support,
            axes="x",
            min_overlap=0.55,
            elem_a="main_beam",
            elem_b="guide_bed",
            name="extended long carriage keeps retained insertion",
        )
        ctx.expect_overlap(
            short_slide,
            long_carriage,
            axes="x",
            min_overlap=0.30,
            elem_a="sled_plate",
            elem_b="top_rail_0",
            name="extended carried slide keeps retained insertion",
        )

    ctx.check(
        "serial slides extend forward",
        rest_long is not None
        and rest_short is not None
        and extended_long is not None
        and extended_short is not None
        and extended_long[0] > rest_long[0] + 0.30
        and extended_short[0] > rest_short[0] + 0.58,
        details=f"long {rest_long}->{extended_long}, short {rest_short}->{extended_short}",
    )

    return ctx.report()


object_model = build_object_model()
