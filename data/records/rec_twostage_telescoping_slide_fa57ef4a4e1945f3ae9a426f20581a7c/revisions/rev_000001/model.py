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
    model = ArticulatedObject(name="bridge_backed_extension_slide")

    powder_coat = model.material("powder_coat_black", rgba=(0.055, 0.058, 0.062, 1.0))
    rail_steel = model.material("brushed_rail_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    shadow = model.material("shadowed_channel", rgba=(0.015, 0.016, 0.018, 1.0))
    wear_strip = model.material("acetal_wear_strip", rgba=(0.10, 0.105, 0.11, 1.0))
    stop_red = model.material("red_stop_cap", rgba=(0.70, 0.08, 0.045, 1.0))

    outer_slide = model.part("outer_slide")

    # Rear support: a bridge/yoke frame with an open center, sized like a compact
    # industrial extension-slide bracket.  The outer channel is mechanically tied
    # back to the bridge by side webs and cradles rather than floating in the gap.
    outer_slide.visual(
        Box((0.060, 0.050, 0.290)),
        origin=Origin(xyz=(-0.030, -0.160, 0.145)),
        material=powder_coat,
        name="bridge_post_0",
    )
    outer_slide.visual(
        Box((0.060, 0.050, 0.290)),
        origin=Origin(xyz=(-0.030, 0.160, 0.145)),
        material=powder_coat,
        name="bridge_post_1",
    )
    outer_slide.visual(
        Box((0.060, 0.370, 0.050)),
        origin=Origin(xyz=(-0.030, 0.0, 0.025)),
        material=powder_coat,
        name="bridge_foot",
    )
    outer_slide.visual(
        Box((0.060, 0.370, 0.050)),
        origin=Origin(xyz=(-0.030, 0.0, 0.265)),
        material=powder_coat,
        name="bridge_crown",
    )

    # Short rear webs visibly carry the channel from the support bridge.
    for y, name in ((-0.112, "rear_web_0"), (0.112, "rear_web_1")):
        outer_slide.visual(
            Box((0.090, 0.060, 0.132)),
            origin=Origin(xyz=(0.002, y, 0.140)),
            material=powder_coat,
            name=name,
        )
    outer_slide.visual(
        Box((0.090, 0.145, 0.040)),
        origin=Origin(xyz=(0.002, 0.0, 0.062)),
        material=powder_coat,
        name="lower_cradle",
    )
    outer_slide.visual(
        Box((0.090, 0.145, 0.040)),
        origin=Origin(xyz=(0.002, 0.0, 0.218)),
        material=powder_coat,
        name="upper_cradle",
    )

    # Outer slide body: a rectangular captured channel extruded on the same X
    # axis as the moving stage.  It is open through the middle with top, bottom,
    # and side rails, plus a darker mouth lip and wear strips to make the hollow
    # slide path legible.
    outer_slide.visual(
        Box((0.560, 0.170, 0.035)),
        origin=Origin(xyz=(0.275, 0.0, 0.095)),
        material=rail_steel,
        name="outer_bottom_rail",
    )
    outer_slide.visual(
        Box((0.560, 0.170, 0.035)),
        origin=Origin(xyz=(0.275, 0.0, 0.185)),
        material=rail_steel,
        name="outer_top_rail",
    )
    outer_slide.visual(
        Box((0.560, 0.026, 0.126)),
        origin=Origin(xyz=(0.275, -0.085, 0.140)),
        material=rail_steel,
        name="outer_side_0",
    )
    outer_slide.visual(
        Box((0.560, 0.026, 0.126)),
        origin=Origin(xyz=(0.275, 0.085, 0.140)),
        material=rail_steel,
        name="outer_side_1",
    )
    outer_slide.visual(
        Box((0.520, 0.105, 0.0125)),
        origin=Origin(xyz=(0.295, 0.0, 0.11875)),
        material=wear_strip,
        name="lower_liner",
    )
    outer_slide.visual(
        Box((0.520, 0.105, 0.0125)),
        origin=Origin(xyz=(0.295, 0.0, 0.16125)),
        material=wear_strip,
        name="upper_liner",
    )
    outer_slide.visual(
        Box((0.030, 0.205, 0.030)),
        origin=Origin(xyz=(0.555, 0.0, 0.095)),
        material=shadow,
        name="mouth_lower_lip",
    )
    outer_slide.visual(
        Box((0.030, 0.205, 0.030)),
        origin=Origin(xyz=(0.555, 0.0, 0.185)),
        material=shadow,
        name="mouth_upper_lip",
    )
    outer_slide.visual(
        Box((0.030, 0.030, 0.122)),
        origin=Origin(xyz=(0.555, -0.101, 0.140)),
        material=shadow,
        name="mouth_side_0",
    )
    outer_slide.visual(
        Box((0.030, 0.030, 0.122)),
        origin=Origin(xyz=(0.555, 0.101, 0.140)),
        material=shadow,
        name="mouth_side_1",
    )

    # Bolt heads on the rear bridge front face make the bracket read as a real
    # mounted support.  They are slightly embedded into the bridge pieces.
    for index, (y, z) in enumerate(((-0.160, 0.045), (0.160, 0.045), (-0.160, 0.245), (0.160, 0.245))):
        outer_slide.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.006, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=shadow,
            name=f"bridge_bolt_{index}",
        )

    inner_stage = model.part("inner_stage")
    # The carried stage is long enough to remain retained inside the outer
    # channel at full travel, while already projecting forward at zero travel.
    inner_stage.visual(
        Box((0.620, 0.070, 0.030)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=rail_steel,
        name="inner_beam",
    )
    inner_stage.visual(
        Box((0.320, 0.010, 0.024)),
        origin=Origin(xyz=(-0.135, -0.039, 0.0)),
        material=wear_strip,
        name="stage_pad_0",
    )
    inner_stage.visual(
        Box((0.320, 0.010, 0.024)),
        origin=Origin(xyz=(-0.135, 0.039, 0.0)),
        material=wear_strip,
        name="stage_pad_1",
    )
    inner_stage.visual(
        Box((0.036, 0.130, 0.075)),
        origin=Origin(xyz=(0.287, 0.0, 0.0)),
        material=stop_red,
        name="front_stop",
    )
    inner_stage.visual(
        Box((0.018, 0.085, 0.050)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=shadow,
        name="stop_neck",
    )

    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=outer_slide,
        child=inner_stage,
        origin=Origin(xyz=(0.555, 0.0, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_slide = object_model.get_part("outer_slide")
    inner_stage = object_model.get_part("inner_stage")
    slide = object_model.get_articulation("stage_slide")

    ctx.check(
        "single prismatic slide joint",
        len(object_model.articulations) == 1 and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations!r}",
    )
    ctx.check(
        "slide axis is shared x",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis!r}",
    )

    ctx.expect_within(
        inner_stage,
        outer_slide,
        axes="yz",
        inner_elem="inner_beam",
        margin=0.0,
        name="inner beam sits within channel envelope",
    )
    ctx.expect_gap(
        inner_stage,
        outer_slide,
        axis="z",
        positive_elem="inner_beam",
        negative_elem="outer_bottom_rail",
        min_gap=0.006,
        max_gap=0.020,
        name="inner beam clears lower rail",
    )
    ctx.expect_gap(
        outer_slide,
        inner_stage,
        axis="z",
        positive_elem="outer_top_rail",
        negative_elem="inner_beam",
        min_gap=0.006,
        max_gap=0.020,
        name="inner beam clears upper rail",
    )
    ctx.expect_overlap(
        inner_stage,
        outer_slide,
        axes="x",
        elem_a="inner_beam",
        elem_b="outer_bottom_rail",
        min_overlap=0.300,
        name="collapsed stage retains long insertion",
    )

    rest_position = ctx.part_world_position(inner_stage)
    with ctx.pose({slide: 0.220}):
        ctx.expect_overlap(
            inner_stage,
            outer_slide,
            axes="x",
            elem_a="inner_beam",
            elem_b="outer_bottom_rail",
            min_overlap=0.110,
            name="extended stage remains inserted",
        )
        ctx.expect_within(
            inner_stage,
            outer_slide,
            axes="yz",
            inner_elem="inner_beam",
            margin=0.0,
            name="extended stage stays on shared axis",
        )
        extended_position = ctx.part_world_position(inner_stage)

    ctx.check(
        "stage extends forward",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.200,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
