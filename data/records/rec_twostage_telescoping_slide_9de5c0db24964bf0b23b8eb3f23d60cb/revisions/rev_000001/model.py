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
    model = ArticulatedObject(name="nested_slide_pair_with_end_plate")

    dark_rail = model.material("dark_anodized_rail", color=(0.08, 0.09, 0.10, 1.0))
    blue_slide = model.material("blue_middle_slide", color=(0.08, 0.24, 0.55, 1.0))
    silver = model.material("brushed_bearing_metal", color=(0.72, 0.74, 0.72, 1.0))
    red_plate = model.material("red_output_end_plate", color=(0.72, 0.05, 0.035, 1.0))
    rubber = model.material("black_rubber_face", color=(0.015, 0.014, 0.013, 1.0))

    # Rooted outer slide guide: a grounded C-channel with top lips and a rear stop.
    outer = model.part("outer_guide")
    outer.visual(
        Box((0.76, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_rail,
        name="base_plate",
    )
    outer.visual(
        Box((0.76, 0.025, 0.120)),
        origin=Origin(xyz=(0.0, 0.1075, 0.095)),
        material=dark_rail,
        name="side_rail_0",
    )
    outer.visual(
        Box((0.76, 0.025, 0.120)),
        origin=Origin(xyz=(0.0, -0.1075, 0.095)),
        material=dark_rail,
        name="side_rail_1",
    )
    outer.visual(
        Box((0.76, 0.055, 0.022)),
        origin=Origin(xyz=(0.0, 0.0775, 0.162)),
        material=dark_rail,
        name="top_lip_0",
    )
    outer.visual(
        Box((0.76, 0.055, 0.022)),
        origin=Origin(xyz=(0.0, -0.0775, 0.162)),
        material=dark_rail,
        name="top_lip_1",
    )
    outer.visual(
        Box((0.035, 0.24, 0.155)),
        origin=Origin(xyz=(-0.3625, 0.0, 0.1125)),
        material=dark_rail,
        name="rear_stop",
    )
    outer.visual(
        Box((0.055, 0.16, 0.006)),
        origin=Origin(xyz=(-0.27, 0.0, 0.032)),
        material=silver,
        name="wear_strip_0",
    )
    outer.visual(
        Box((0.055, 0.16, 0.006)),
        origin=Origin(xyz=(0.19, 0.0, 0.032)),
        material=silver,
        name="wear_strip_1",
    )
    for i, (x, y) in enumerate(((-0.30, -0.0775), (-0.30, 0.0775), (0.30, -0.0775), (0.30, 0.0775))):
        outer.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(x, y, 0.176)),
            material=silver,
            name=f"mount_screw_{i}",
        )

    # Middle stage: a long hidden sliding member with a raised top track for the output stage.
    middle = model.part("middle_stage")
    middle.visual(
        Box((0.60, 0.160, 0.030)),
        origin=Origin(xyz=(-0.270, 0.0, 0.015)),
        material=blue_slide,
        name="lower_shoe",
    )
    middle.visual(
        Box((0.60, 0.100, 0.055)),
        origin=Origin(xyz=(-0.270, 0.0, 0.0575)),
        material=blue_slide,
        name="center_web",
    )
    middle.visual(
        Box((0.60, 0.135, 0.028)),
        origin=Origin(xyz=(-0.270, 0.0, 0.099)),
        material=blue_slide,
        name="upper_track",
    )
    middle.visual(
        Box((0.040, 0.168, 0.050)),
        origin=Origin(xyz=(0.040, 0.0, 0.088)),
        material=blue_slide,
        name="front_stop_block",
    )
    middle.visual(
        Box((0.030, 0.150, 0.045)),
        origin=Origin(xyz=(-0.560, 0.0, 0.086)),
        material=blue_slide,
        name="rear_retainer",
    )

    # Compact final output stage riding on the middle track, with a visible end plate.
    output = model.part("output_stage")
    output.visual(
        Box((0.365, 0.090, 0.035)),
        origin=Origin(xyz=(-0.1375, 0.0, 0.0175)),
        material=silver,
        name="output_runner",
    )
    output.visual(
        Box((0.025, 0.160, 0.140)),
        origin=Origin(xyz=(0.052, 0.0, 0.070)),
        material=red_plate,
        name="end_plate",
    )
    output.visual(
        Box((0.008, 0.130, 0.100)),
        origin=Origin(xyz=(0.068, 0.0, 0.072)),
        material=rubber,
        name="end_pad",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        # The child frame sits at the front mouth and top of the outer base plate.
        origin=Origin(xyz=(0.320, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.30),
    )

    model.articulation(
        "middle_to_output",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=output,
        # The output runner rides on top of the middle stage's upper track.
        origin=Origin(xyz=(0.030, 0.0, 0.113)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_stage")
    output = object_model.get_part("output_stage")
    outer_slide = object_model.get_articulation("outer_to_middle")
    output_slide = object_model.get_articulation("middle_to_output")

    ctx.expect_within(
        middle,
        outer,
        axes="y",
        inner_elem="lower_shoe",
        outer_elem="base_plate",
        margin=0.0,
        name="middle shoe is laterally inside outer guide",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        positive_elem="lower_shoe",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="middle shoe rests on outer base",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="lower_shoe",
        elem_b="base_plate",
        min_overlap=0.25,
        name="collapsed middle stage remains retained in outer guide",
    )
    ctx.expect_gap(
        outer,
        middle,
        axis="z",
        positive_elem="top_lip_0",
        negative_elem="upper_track",
        min_gap=0.002,
        max_gap=0.006,
        name="outer top lip clears middle upper track",
    )

    ctx.expect_within(
        output,
        outer,
        axes="y",
        inner_elem="output_runner",
        outer_elem="base_plate",
        margin=0.0,
        name="compact output runner stays inside guide width",
    )
    ctx.expect_gap(
        output,
        middle,
        axis="z",
        positive_elem="output_runner",
        negative_elem="upper_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="output runner rides on middle track",
    )
    ctx.expect_overlap(
        output,
        middle,
        axes="x",
        elem_a="output_runner",
        elem_b="upper_track",
        min_overlap=0.12,
        name="collapsed output stage remains retained on middle track",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_output = ctx.part_world_position(output)
    with ctx.pose({outer_slide: 0.30, output_slide: 0.18}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="lower_shoe",
            elem_b="base_plate",
            min_overlap=0.25,
            name="extended middle stage still has retained insertion",
        )
        ctx.expect_overlap(
            output,
            middle,
            axes="x",
            elem_a="output_runner",
            elem_b="upper_track",
            min_overlap=0.12,
            name="extended output stage still has retained insertion",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_output = ctx.part_world_position(output)

    ctx.check(
        "two serial prismatic joints extend in positive x",
        rest_middle is not None
        and rest_output is not None
        and extended_middle is not None
        and extended_output is not None
        and extended_middle[0] > rest_middle[0] + 0.29
        and extended_output[0] > rest_output[0] + 0.47,
        details=f"middle {rest_middle}->{extended_middle}, output {rest_output}->{extended_output}",
    )

    return ctx.report()


object_model = build_object_model()
