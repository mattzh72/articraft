from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="tower_offset_rotary_unit")

    cast_iron = Material("dark_cast_iron", color=(0.08, 0.09, 0.10, 1.0))
    tower_paint = Material("blue_gray_paint", color=(0.22, 0.30, 0.38, 1.0))
    bearing_metal = Material("brushed_bearing_metal", color=(0.62, 0.64, 0.62, 1.0))
    lower_stage_paint = Material("lower_stage_orange", color=(0.93, 0.42, 0.12, 1.0))
    upper_stage_paint = Material("upper_stage_green", color=(0.12, 0.58, 0.34, 1.0))
    black_rubber = Material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))
    brass_marker = Material("brass_marker", color=(0.90, 0.68, 0.25, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.00, 0.52, 0.06)),
        origin=Origin(xyz=(0.05, 0.0, 0.03)),
        material=cast_iron,
        name="ground_plate",
    )
    frame.visual(
        Cylinder(radius=0.135, length=0.050),
        origin=Origin(xyz=(-0.24, 0.0, 0.085)),
        material=bearing_metal,
        name="lower_bearing",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.020),
        origin=Origin(xyz=(-0.24, 0.0, 0.120)),
        material=black_rubber,
        name="lower_thrust_washer",
    )
    frame.visual(
        Box((0.130, 0.170, 0.680)),
        origin=Origin(xyz=(0.35, 0.0, 0.400)),
        material=tower_paint,
        name="tower_column",
    )
    frame.visual(
        Box((0.245, 0.235, 0.100)),
        origin=Origin(xyz=(0.35, 0.0, 0.790)),
        material=tower_paint,
        name="upper_head_block",
    )
    frame.visual(
        Cylinder(radius=0.088, length=0.042),
        origin=Origin(xyz=(0.35, 0.0, 0.861)),
        material=bearing_metal,
        name="upper_bearing",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.185, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=lower_stage_paint,
        name="lower_table",
    )
    lower_stage.visual(
        Cylinder(radius=0.072, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=bearing_metal,
        name="lower_hub",
    )
    lower_stage.visual(
        Box((0.160, 0.030, 0.014)),
        origin=Origin(xyz=(0.080, 0.0, 0.057)),
        material=brass_marker,
        name="lower_index",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.122, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=upper_stage_paint,
        name="upper_table",
    )
    upper_stage.visual(
        Cylinder(radius=0.052, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=bearing_metal,
        name="upper_hub",
    )
    upper_stage.visual(
        Box((0.105, 0.023, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, 0.046)),
        material=brass_marker,
        name="upper_index",
    )

    model.articulation(
        "lower_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_stage,
        origin=Origin(xyz=(-0.24, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "upper_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_stage,
        origin=Origin(xyz=(0.35, 0.0, 0.882)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_axis = object_model.get_articulation("lower_axis")
    upper_axis = object_model.get_articulation("upper_axis")

    ctx.expect_contact(
        lower_stage,
        frame,
        elem_a="lower_table",
        elem_b="lower_thrust_washer",
        name="lower rotary table is seated on its bearing washer",
    )
    ctx.expect_contact(
        upper_stage,
        frame,
        elem_a="upper_table",
        elem_b="upper_bearing",
        name="upper rotary table is seated on its tower bearing",
    )
    ctx.expect_origin_distance(
        lower_stage,
        upper_stage,
        axes="xy",
        min_dist=0.55,
        max_dist=0.60,
        name="rotary shaft lines are horizontally offset",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="z",
        min_gap=0.70,
        max_gap=0.80,
        name="upper rotary stage is carried above lower stage",
    )

    ctx.check(
        "stage joints are separate parallel revolute axes",
        lower_axis.articulation_type == ArticulationType.REVOLUTE
        and upper_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_axis.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_axis.axis) == (0.0, 0.0, 1.0),
        details=f"lower={lower_axis}, upper={upper_axis}",
    )

    return ctx.report()


object_model = build_object_model()
