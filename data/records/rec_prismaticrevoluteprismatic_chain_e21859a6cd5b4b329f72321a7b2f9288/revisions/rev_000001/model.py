from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_prismatic_chain")

    dark_rail = Material("black_anodized_rail", color=(0.04, 0.045, 0.05, 1.0))
    carriage_blue = Material("blue_carriage", color=(0.08, 0.22, 0.62, 1.0))
    bracket_orange = Material("orange_pivot_bracket", color=(0.90, 0.42, 0.08, 1.0))
    bright_steel = Material("brushed_steel", color=(0.72, 0.74, 0.73, 1.0))
    stop_red = Material("red_end_stops", color=(0.65, 0.05, 0.04, 1.0))

    base_track = model.part("base_track")
    base_track.visual(
        Box((0.36, 0.12, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_rail,
        name="track_bed",
    )
    base_track.visual(
        Box((0.34, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, -0.043, 0.0375)),
        material=bright_steel,
        name="slide_rail_0",
    )
    base_track.visual(
        Box((0.34, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, 0.043, 0.0375)),
        material=bright_steel,
        name="slide_rail_1",
    )
    for index, x in enumerate((-0.171, 0.171)):
        base_track.visual(
            Box((0.018, 0.125, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.035)),
            material=stop_red,
            name=f"end_stop_{index}",
        )

    base_slide = model.part("base_slide")
    base_slide.visual(
        Box((0.090, 0.105, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.0575)),
        material=carriage_blue,
        name="carriage_plate",
    )
    base_slide.visual(
        Box((0.050, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=carriage_blue,
        name="pivot_plinth",
    )
    for index, y in enumerate((-0.041, 0.041)):
        base_slide.visual(
            Box((0.034, 0.010, 0.065)),
            origin=Origin(xyz=(0.0, y, -0.0325)),
            material=carriage_blue,
            name=f"fork_cheek_{index}",
        )
        base_slide.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.0, y * 1.29, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"pivot_boss_{index}",
        )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bracket_orange,
        name="center_hub",
    )
    pivot_bracket.visual(
        Box((0.065, 0.052, 0.047)),
        origin=Origin(xyz=(0.0325, 0.0, 0.0)),
        material=bracket_orange,
        name="hub_neck",
    )
    pivot_bracket.visual(
        Box((0.190, 0.066, 0.009)),
        origin=Origin(xyz=(0.115, 0.0, 0.0235)),
        material=bracket_orange,
        name="sleeve_top",
    )
    pivot_bracket.visual(
        Box((0.190, 0.066, 0.009)),
        origin=Origin(xyz=(0.115, 0.0, -0.0235)),
        material=bracket_orange,
        name="sleeve_bottom",
    )
    for index, y in enumerate((-0.033, 0.033)):
        pivot_bracket.visual(
            Box((0.190, 0.008, 0.042)),
            origin=Origin(xyz=(0.115, y, 0.0)),
            material=bracket_orange,
            name=f"sleeve_side_{index}",
        )

    distal_stage = model.part("distal_stage")
    distal_stage.visual(
        Box((0.220, 0.044, 0.026)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=bright_steel,
        name="slider_bar",
    )
    distal_stage.visual(
        Box((0.028, 0.062, 0.044)),
        origin=Origin(xyz=(0.199, 0.0, 0.0)),
        material=carriage_blue,
        name="pull_block",
    )

    model.articulation(
        "base_slide_joint",
        ArticulationType.PRISMATIC,
        parent=base_track,
        child=base_slide,
        origin=Origin(xyz=(-0.075, 0.0, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.40, lower=0.0, upper=0.150),
    )
    model.articulation(
        "pivot_joint",
        ArticulationType.REVOLUTE,
        parent=base_slide,
        child=pivot_bracket,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "distal_slide_joint",
        ArticulationType.PRISMATIC,
        parent=pivot_bracket,
        child=distal_stage,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.30, lower=0.0, upper=0.080),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_track = object_model.get_part("base_track")
    base_slide = object_model.get_part("base_slide")
    pivot_bracket = object_model.get_part("pivot_bracket")
    distal_stage = object_model.get_part("distal_stage")
    base_joint = object_model.get_articulation("base_slide_joint")
    pivot_joint = object_model.get_articulation("pivot_joint")
    distal_joint = object_model.get_articulation("distal_slide_joint")

    ctx.check(
        "base slide travel is 150 mm",
        abs(base_joint.motion_limits.upper - 0.150) < 1e-6
        and abs(base_joint.motion_limits.lower) < 1e-6,
        details=str(base_joint.motion_limits),
    )
    ctx.check(
        "pivot rotates 90 degrees each way",
        abs(pivot_joint.motion_limits.lower + math.pi / 2.0) < 1e-6
        and abs(pivot_joint.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=str(pivot_joint.motion_limits),
    )
    ctx.check(
        "distal slide travel is 80 mm",
        abs(distal_joint.motion_limits.upper - 0.080) < 1e-6
        and abs(distal_joint.motion_limits.lower) < 1e-6,
        details=str(distal_joint.motion_limits),
    )

    ctx.expect_contact(
        base_slide,
        base_track,
        elem_a="carriage_plate",
        elem_b="slide_rail_0",
        contact_tol=0.001,
        name="carriage rides on the rail",
    )
    ctx.expect_within(
        distal_stage,
        pivot_bracket,
        axes="yz",
        inner_elem="slider_bar",
        margin=0.001,
        name="distal bar is centered in sleeve opening",
    )
    ctx.expect_gap(
        pivot_bracket,
        distal_stage,
        axis="z",
        min_gap=0.004,
        max_gap=0.008,
        positive_elem="sleeve_top",
        negative_elem="slider_bar",
        name="slider clears the sleeve roof",
    )
    ctx.expect_gap(
        distal_stage,
        pivot_bracket,
        axis="z",
        min_gap=0.004,
        max_gap=0.008,
        positive_elem="slider_bar",
        negative_elem="sleeve_bottom",
        name="slider clears the sleeve floor",
    )
    ctx.expect_overlap(
        distal_stage,
        pivot_bracket,
        axes="x",
        elem_a="slider_bar",
        elem_b="sleeve_top",
        min_overlap=0.10,
        name="retracted distal bar remains captured",
    )

    rest_base = ctx.part_world_position(base_slide)
    with ctx.pose({base_joint: 0.150}):
        extended_base = ctx.part_world_position(base_slide)
    ctx.check(
        "base carriage moves along positive rail axis",
        rest_base is not None
        and extended_base is not None
        and extended_base[0] > rest_base[0] + 0.145,
        details=f"rest={rest_base}, extended={extended_base}",
    )

    with ctx.pose({distal_joint: 0.080}):
        ctx.expect_overlap(
            distal_stage,
            pivot_bracket,
            axes="x",
            elem_a="slider_bar",
            elem_b="sleeve_top",
            min_overlap=0.050,
            name="extended distal bar remains inserted",
        )

    with ctx.pose({pivot_joint: math.pi / 2.0, distal_joint: 0.0}):
        yawed_rest = ctx.part_world_position(distal_stage)
    with ctx.pose({pivot_joint: math.pi / 2.0, distal_joint: 0.080}):
        yawed_extended = ctx.part_world_position(distal_stage)
    ctx.check(
        "distal slide follows rotated bracket axis",
        yawed_rest is not None
        and yawed_extended is not None
        and yawed_extended[1] > yawed_rest[1] + 0.075,
        details=f"rest={yawed_rest}, extended={yawed_extended}",
    )

    return ctx.report()


object_model = build_object_model()
