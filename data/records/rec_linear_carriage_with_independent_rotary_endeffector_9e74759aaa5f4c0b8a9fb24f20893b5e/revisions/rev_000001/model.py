from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_transfer_axis_tool_nose")

    dark_cast = model.material("dark_cast_iron", color=(0.08, 0.09, 0.10, 1.0))
    blued_steel = model.material("blued_guide_steel", color=(0.20, 0.27, 0.34, 1.0))
    carriage_paint = model.material("saddle_blue_gray", color=(0.24, 0.32, 0.40, 1.0))
    bright_steel = model.material("brushed_output_steel", color=(0.72, 0.72, 0.68, 1.0))
    cutting_steel = model.material("dark_tool_steel", color=(0.12, 0.12, 0.11, 1.0))
    marker = model.material("rotary_index_mark", color=(0.02, 0.05, 0.07, 1.0))

    fixed_rail = model.part("fixed_rail")
    fixed_rail.visual(
        Box((1.28, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_cast,
        name="bench_plate",
    )
    fixed_rail.visual(
        Box((1.14, 0.080, 0.071)),
        origin=Origin(xyz=(0.0, 0.0, 0.0745)),
        material=blued_steel,
        name="guide_rail",
    )
    fixed_rail.visual(
        Box((0.030, 0.190, 0.090)),
        origin=Origin(xyz=(-0.595, 0.0, 0.085)),
        material=dark_cast,
        name="end_stop_0",
    )
    fixed_rail.visual(
        Box((0.030, 0.190, 0.090)),
        origin=Origin(xyz=(0.595, 0.0, 0.085)),
        material=dark_cast,
        name="end_stop_1",
    )
    fixed_rail.visual(
        Box((1.05, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.128, 0.048)),
        material=blued_steel,
        name="front_way_strip",
    )
    fixed_rail.visual(
        Box((1.05, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.128, 0.048)),
        material=blued_steel,
        name="rear_way_strip",
    )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.205, 0.182, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=carriage_paint,
        name="saddle_plate",
    )
    saddle.visual(
        Box((0.205, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, -0.076, -0.015)),
        material=carriage_paint,
        name="front_gib",
    )
    saddle.visual(
        Box((0.205, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, 0.076, -0.015)),
        material=carriage_paint,
        name="rear_gib",
    )
    saddle.visual(
        Box((0.110, 0.070, 0.025)),
        origin=Origin(xyz=(0.0, 0.072, 0.0575)),
        material=carriage_paint,
        name="bearing_foot",
    )
    saddle.visual(
        Box((0.076, 0.046, 0.037)),
        origin=Origin(xyz=(0.0, 0.108, 0.0655)),
        material=carriage_paint,
        name="bearing_post",
    )
    saddle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.046, tube=0.012, radial_segments=40, tubular_segments=14), "bearing_collar"),
        origin=Origin(xyz=(0.0, 0.115, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=carriage_paint,
        name="bearing_collar",
    )

    tool_nose = model.part("tool_nose")
    tool_nose.visual(
        Cylinder(radius=0.031, length=0.125),
        origin=Origin(xyz=(0.0, 0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="spindle_body",
    )
    tool_nose.visual(
        Box((0.012, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, 0.060, 0.0325)),
        material=marker,
        name="index_mark",
    )
    tool_nose.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(xyz=(0.0, 0.152, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="collet_nose",
    )
    tool_nose.visual(
        Cylinder(radius=0.0075, length=0.070),
        origin=Origin(xyz=(0.0, 0.212, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cutting_steel,
        name="tool_bit",
    )

    model.articulation(
        "rail_to_saddle",
        ArticulationType.PRISMATIC,
        parent=fixed_rail,
        child=saddle,
        origin=Origin(xyz=(-0.340, 0.0, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.680),
    )
    model.articulation(
        "saddle_to_tool_nose",
        ArticulationType.REVOLUTE,
        parent=saddle,
        child=tool_nose,
        origin=Origin(xyz=(0.0, 0.115, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_rail = object_model.get_part("fixed_rail")
    saddle = object_model.get_part("saddle")
    tool_nose = object_model.get_part("tool_nose")
    slide = object_model.get_articulation("rail_to_saddle")
    spin = object_model.get_articulation("saddle_to_tool_nose")

    ctx.check(
        "one slide and one spin joint",
        len(object_model.articulations) == 2
        and slide.articulation_type == ArticulationType.PRISMATIC
        and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_gap(
        saddle,
        fixed_rail,
        axis="z",
        positive_elem="saddle_plate",
        negative_elem="guide_rail",
        min_gap=0.0,
        max_gap=0.002,
        name="saddle rides just above guide rail",
    )
    ctx.expect_overlap(
        saddle,
        fixed_rail,
        axes="xy",
        elem_a="saddle_plate",
        elem_b="guide_rail",
        min_overlap=0.070,
        name="saddle footprint remains on rail at rest",
    )
    ctx.expect_within(
        tool_nose,
        saddle,
        axes="xz",
        inner_elem="spindle_body",
        outer_elem="bearing_collar",
        margin=0.003,
        name="spindle is centered inside bearing collar envelope",
    )
    ctx.expect_overlap(
        tool_nose,
        saddle,
        axes="y",
        elem_a="spindle_body",
        elem_b="bearing_collar",
        min_overlap=0.010,
        name="spindle passes through local saddle support",
    )

    rest_saddle_pos = ctx.part_world_position(saddle)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            saddle,
            fixed_rail,
            axis="z",
            positive_elem="saddle_plate",
            negative_elem="guide_rail",
            min_gap=0.0,
            max_gap=0.002,
            name="extended saddle stays supported on rail",
        )
        ctx.expect_overlap(
            saddle,
            fixed_rail,
            axes="xy",
            elem_a="saddle_plate",
            elem_b="guide_rail",
            min_overlap=0.070,
            name="extended saddle footprint stays on rail",
        )
        extended_saddle_pos = ctx.part_world_position(saddle)

    ctx.check(
        "prismatic joint moves saddle along bench axis",
        rest_saddle_pos is not None
        and extended_saddle_pos is not None
        and extended_saddle_pos[0] > rest_saddle_pos[0] + 0.60,
        details=f"rest={rest_saddle_pos}, extended={extended_saddle_pos}",
    )

    rest_mark_aabb = ctx.part_element_world_aabb(tool_nose, elem="index_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_mark_aabb = ctx.part_element_world_aabb(tool_nose, elem="index_mark")

    if rest_mark_aabb is not None and spun_mark_aabb is not None:
        rest_mark_x = (rest_mark_aabb[0][0] + rest_mark_aabb[1][0]) / 2.0
        spun_mark_x = (spun_mark_aabb[0][0] + spun_mark_aabb[1][0]) / 2.0
        rest_mark_z = (rest_mark_aabb[0][2] + rest_mark_aabb[1][2]) / 2.0
        spun_mark_z = (spun_mark_aabb[0][2] + spun_mark_aabb[1][2]) / 2.0
    else:
        rest_mark_x = spun_mark_x = rest_mark_z = spun_mark_z = None

    ctx.check(
        "revolute joint visibly spins index mark",
        rest_mark_x is not None
        and spun_mark_x is not None
        and spun_mark_z is not None
        and rest_mark_z is not None
        and spun_mark_x > rest_mark_x + 0.020
        and spun_mark_z < rest_mark_z - 0.020,
        details=f"rest_aabb={rest_mark_aabb}, spun_aabb={spun_mark_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
