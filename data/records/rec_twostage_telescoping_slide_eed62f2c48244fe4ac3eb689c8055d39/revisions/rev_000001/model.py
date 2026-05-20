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
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.77, 1.0))
    blackened = model.material("blackened_slot", rgba=(0.015, 0.015, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.86, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="mounting_plate",
    )
    base.visual(
        Box((0.76, 0.088, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brushed_steel,
        name="outer_bottom_web",
    )
    base.visual(
        Box((0.76, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.044, 0.042)),
        material=brushed_steel,
        name="outer_wall_0",
    )
    base.visual(
        Box((0.76, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, -0.044, 0.042)),
        material=brushed_steel,
        name="outer_wall_1",
    )
    base.visual(
        Box((0.76, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.034, 0.068)),
        material=brushed_steel,
        name="outer_lip_0",
    )
    base.visual(
        Box((0.76, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.034, 0.068)),
        material=brushed_steel,
        name="outer_lip_1",
    )
    base.visual(
        Box((0.70, 0.011, 0.008)),
        origin=Origin(xyz=(0.0, 0.0355, 0.040)),
        material=polished_steel,
        name="outer_race_0",
    )
    base.visual(
        Box((0.70, 0.011, 0.008)),
        origin=Origin(xyz=(0.0, -0.0355, 0.040)),
        material=polished_steel,
        name="outer_race_1",
    )

    for index, (x, y) in enumerate(
        ((-0.31, 0.060), (0.31, 0.060), (-0.31, -0.060), (0.31, -0.060))
    ):
        base.visual(
            Box((0.070, 0.020, 0.0012)),
            origin=Origin(xyz=(x, y, 0.0125)),
            material=blackened,
            name=f"mount_slot_{index}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, y, 0.014)),
            material=polished_steel,
            name=f"screw_head_{index}",
        )
        base.visual(
            Cylinder(radius=0.004, length=0.001),
            origin=Origin(xyz=(x, y, 0.0165)),
            material=blackened,
            name=f"screw_recess_{index}",
        )

    slide = model.part("slide")
    slide.visual(
        Box((0.72, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=brushed_steel,
        name="inner_spine",
    )
    slide.visual(
        Box((0.72, 0.052, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brushed_steel,
        name="inner_top_web",
    )
    slide.visual(
        Box((0.72, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, 0.003)),
        material=brushed_steel,
        name="inner_web_0",
    )
    slide.visual(
        Box((0.72, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.023, 0.003)),
        material=brushed_steel,
        name="inner_web_1",
    )
    slide.visual(
        Cylinder(radius=0.004, length=0.69),
        origin=Origin(xyz=(0.0, 0.026, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="inner_race_0",
    )
    slide.visual(
        Cylinder(radius=0.004, length=0.69),
        origin=Origin(xyz=(0.0, -0.026, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="inner_race_1",
    )
    slide.visual(
        Box((0.035, 0.044, 0.032)),
        origin=Origin(xyz=(0.365, 0.0, -0.001)),
        material=dark_steel,
        name="front_stop",
    )
    slide.visual(
        Box((0.060, 0.012, 0.001)),
        origin=Origin(xyz=(-0.16, 0.0, 0.0155)),
        material=blackened,
        name="inner_slot_0",
    )
    slide.visual(
        Box((0.060, 0.012, 0.001)),
        origin=Origin(xyz=(0.16, 0.0, 0.0155)),
        material=blackened,
        name="inner_slot_1",
    )

    model.articulation(
        "base_to_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    slide = object_model.get_part("slide")
    extension = object_model.get_articulation("base_to_slide")

    ctx.check(
        "single prismatic extension path",
        len(object_model.articulations) == 1
        and extension.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_overlap(
        slide,
        base,
        axes="x",
        elem_a="inner_spine",
        elem_b="outer_bottom_web",
        min_overlap=0.68,
        name="nested rails overlap when retracted",
    )
    ctx.expect_within(
        slide,
        base,
        axes="y",
        inner_elem="inner_top_web",
        outer_elem="outer_bottom_web",
        margin=0.0,
        name="inner rail sits inside outer channel width",
    )
    ctx.expect_gap(
        slide,
        base,
        axis="z",
        positive_elem="inner_spine",
        negative_elem="outer_bottom_web",
        min_gap=0.010,
        max_gap=0.025,
        name="inner rail clears lower web",
    )
    ctx.expect_gap(
        base,
        slide,
        axis="z",
        positive_elem="outer_lip_0",
        negative_elem="inner_top_web",
        min_gap=0.003,
        max_gap=0.010,
        name="inner rail clears top lip",
    )
    ctx.expect_contact(
        base,
        slide,
        elem_a="outer_race_0",
        elem_b="inner_race_0",
        contact_tol=0.0001,
        name="nested race supports slide",
    )

    rest_position = ctx.part_world_position(slide)
    with ctx.pose({extension: 0.32}):
        ctx.expect_overlap(
            slide,
            base,
            axes="x",
            elem_a="inner_spine",
            elem_b="outer_bottom_web",
            min_overlap=0.35,
            name="extended rail remains retained",
        )
        ctx.expect_within(
            slide,
            base,
            axes="y",
            inner_elem="inner_top_web",
            outer_elem="outer_bottom_web",
            margin=0.0,
            name="extended rail remains centered in channel",
        )
        extended_position = ctx.part_world_position(slide)

    ctx.check(
        "slide extends along positive x",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.30,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
