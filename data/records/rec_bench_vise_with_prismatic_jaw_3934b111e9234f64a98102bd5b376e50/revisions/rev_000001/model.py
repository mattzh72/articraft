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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_milling_vise")

    cast_iron = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    machined = model.material("machined_steel", rgba=(0.63, 0.64, 0.61, 1.0))
    hardened = model.material("hardened_jaw_steel", rgba=(0.46, 0.48, 0.49, 1.0))
    black = model.material("black_oxide", rgba=(0.02, 0.022, 0.025, 1.0))
    oiled = model.material("oiled_slot", rgba=(0.005, 0.006, 0.007, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_block",
    )
    base.visual(
        Box((0.54, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.135, 0.087)),
        material=machined,
        name="rear_x_way",
    )
    base.visual(
        Box((0.54, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.135, 0.087)),
        material=machined,
        name="front_x_way",
    )
    base.visual(
        Box((0.50, 0.034, 0.004)),
        origin=Origin(xyz=(0.01, 0.0, 0.078)),
        material=oiled,
        name="recessed_t_slot",
    )
    base.visual(
        Box((0.055, 0.25, 0.105)),
        origin=Origin(xyz=(-0.245, 0.0, 0.1305)),
        material=cast_iron,
        name="fixed_lower_jaw",
    )
    base.visual(
        Box((0.006, 0.220, 0.072)),
        origin=Origin(xyz=(-0.2145, 0.0, 0.136)),
        material=hardened,
        name="fixed_lower_plate",
    )
    for idx, z in enumerate((0.112, 0.136, 0.160)):
        base.visual(
            Box((0.004, 0.205, 0.004)),
            origin=Origin(xyz=(-0.210, 0.0, z)),
            material=machined,
            name=f"fixed_lower_tooth_{idx}",
        )
    base.visual(
        Box((0.042, 0.040, 0.055)),
        origin=Origin(xyz=(-0.285, -0.171, 0.082)),
        material=cast_iron,
        name="x_screw_left_bearing",
    )
    base.visual(
        Box((0.042, 0.040, 0.055)),
        origin=Origin(xyz=(0.285, -0.171, 0.082)),
        material=cast_iron,
        name="x_screw_right_bearing",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.70),
        origin=Origin(xyz=(0.0, -0.198, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="x_lead_screw",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.338, -0.198, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="x_handwheel",
    )
    base.visual(
        Box((0.010, 0.007, 0.060)),
        origin=Origin(xyz=(0.346, -0.198, 0.119)),
        material=black,
        name="x_wheel_spoke",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(0.358, -0.198, 0.149), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="x_crank_knob",
    )

    lower_slide = model.part("lower_jaw")
    lower_slide.visual(
        Box((0.32, 0.20, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="lower_saddle",
    )
    lower_slide.visual(
        Box((0.048, 0.220, 0.090)),
        origin=Origin(xyz=(-0.135, 0.0, 0.088)),
        material=cast_iron,
        name="moving_lower_jaw",
    )
    lower_slide.visual(
        Box((0.006, 0.200, 0.066)),
        origin=Origin(xyz=(-0.162, 0.0, 0.092)),
        material=hardened,
        name="moving_lower_plate",
    )
    for idx, z in enumerate((0.070, 0.092, 0.114)):
        lower_slide.visual(
            Box((0.004, 0.188, 0.004)),
            origin=Origin(xyz=(-0.166, 0.0, z)),
            material=machined,
            name=f"moving_lower_tooth_{idx}",
        )
    lower_slide.visual(
        Box((0.028, 0.190, 0.018)),
        origin=Origin(xyz=(-0.115, 0.0, 0.052)),
        material=machined,
        name="left_y_way",
    )
    lower_slide.visual(
        Box((0.028, 0.190, 0.018)),
        origin=Origin(xyz=(0.115, 0.0, 0.052)),
        material=machined,
        name="right_y_way",
    )
    lower_slide.visual(
        Box((0.160, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, -0.085, 0.078)),
        material=cast_iron,
        name="fixed_upper_jaw",
    )
    lower_slide.visual(
        Box((0.145, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, -0.066, 0.080)),
        material=hardened,
        name="fixed_upper_plate",
    )
    lower_slide.visual(
        Box((0.045, 0.060, 0.075)),
        origin=Origin(xyz=(0.167, 0.0, 0.080)),
        material=cast_iron,
        name="y_screw_bearing",
    )
    lower_slide.visual(
        Cylinder(radius=0.006, length=0.37),
        origin=Origin(xyz=(0.187, 0.0, 0.110), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="y_lead_screw",
    )
    lower_slide.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.187, 0.178, 0.110), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="y_handwheel",
    )
    lower_slide.visual(
        Box((0.006, 0.010, 0.050)),
        origin=Origin(xyz=(0.187, 0.184, 0.130)),
        material=black,
        name="y_wheel_spoke",
    )

    upper_jaw = model.part("upper_jaw")
    upper_jaw.visual(
        Box((0.180, 0.150, 0.035)),
        origin=Origin(xyz=(0.0, 0.055, 0.0175)),
        material=cast_iron,
        name="upper_table",
    )
    upper_jaw.visual(
        Box((0.160, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.035, 0.068)),
        material=cast_iron,
        name="moving_upper_jaw",
    )
    upper_jaw.visual(
        Box((0.145, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, 0.012, 0.070)),
        material=hardened,
        name="moving_upper_plate",
    )
    for idx, x in enumerate((-0.048, 0.0, 0.048)):
        upper_jaw.visual(
            Box((0.005, 0.004, 0.046)),
            origin=Origin(xyz=(x, 0.008, 0.071)),
            material=machined,
            name=f"upper_tooth_{idx}",
        )

    model.articulation(
        "x_cross_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.10, lower=0.0, upper=0.12),
    )
    model.articulation(
        "y_upper_slide",
        ArticulationType.PRISMATIC,
        parent=lower_slide,
        child=upper_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.08, lower=-0.075, upper=0.06),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_jaw = object_model.get_part("lower_jaw")
    upper_jaw = object_model.get_part("upper_jaw")
    x_slide = object_model.get_articulation("x_cross_slide")
    y_slide = object_model.get_articulation("y_upper_slide")

    ctx.check(
        "lower jaw uses transverse x prismatic joint",
        x_slide.articulation_type == ArticulationType.PRISMATIC and x_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={x_slide.articulation_type}, axis={x_slide.axis}",
    )
    ctx.check(
        "upper jaw uses perpendicular y prismatic joint",
        y_slide.articulation_type == ArticulationType.PRISMATIC and y_slide.axis == (0.0, 1.0, 0.0),
        details=f"type={y_slide.articulation_type}, axis={y_slide.axis}",
    )

    ctx.expect_gap(
        lower_jaw,
        base,
        axis="z",
        positive_elem="lower_saddle",
        negative_elem="base_block",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower slide rests on base ways height",
    )
    ctx.expect_overlap(
        lower_jaw,
        base,
        axes="xy",
        elem_a="lower_saddle",
        elem_b="base_block",
        min_overlap=0.16,
        name="lower slide is carried by the rectangular base",
    )
    ctx.expect_gap(
        upper_jaw,
        lower_jaw,
        axis="z",
        positive_elem="upper_table",
        negative_elem="lower_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper slide sits above the lower jaw saddle",
    )
    ctx.expect_overlap(
        upper_jaw,
        lower_jaw,
        axes="xy",
        elem_a="upper_table",
        elem_b="lower_saddle",
        min_overlap=0.12,
        name="upper slide is nested on the lower saddle footprint",
    )

    lower_rest = ctx.part_world_position(lower_jaw)
    with ctx.pose({x_slide: 0.10}):
        lower_extended = ctx.part_world_position(lower_jaw)
    ctx.check(
        "lower jaw translates along positive x",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.095
        and abs(lower_extended[1] - lower_rest[1]) < 0.001,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_jaw)
    with ctx.pose({y_slide: 0.05}):
        upper_open = ctx.part_world_position(upper_jaw)
    with ctx.pose({y_slide: -0.07}):
        upper_closed = ctx.part_world_position(upper_jaw)
        ctx.expect_gap(
            upper_jaw,
            lower_jaw,
            axis="y",
            positive_elem="moving_upper_plate",
            negative_elem="fixed_upper_plate",
            min_gap=0.0,
            max_gap=0.010,
            name="upper jaw closes toward its fixed plate",
        )
    ctx.check(
        "upper jaw translates along y above lower slide",
        upper_rest is not None
        and upper_open is not None
        and upper_closed is not None
        and upper_open[1] > upper_rest[1] + 0.045
        and upper_closed[1] < upper_rest[1] - 0.065
        and abs(upper_open[0] - upper_rest[0]) < 0.001,
        details=f"rest={upper_rest}, open={upper_open}, closed={upper_closed}",
    )

    return ctx.report()


object_model = build_object_model()
