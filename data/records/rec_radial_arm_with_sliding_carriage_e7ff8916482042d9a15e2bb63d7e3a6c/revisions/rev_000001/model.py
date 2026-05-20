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
    model = ArticulatedObject(name="radial_arm_sliding_carriage")

    painted_cast = Material("warm_grey_painted_cast_iron", color=(0.48, 0.50, 0.48, 1.0))
    dark_steel = Material("dark_blued_steel", color=(0.08, 0.09, 0.10, 1.0))
    oiled_steel = Material("oiled_bright_steel", color=(0.62, 0.64, 0.62, 1.0))
    black = Material("black_plastic_handles", color=(0.015, 0.014, 0.012, 1.0))
    safety = Material("muted_safety_orange", color=(0.95, 0.42, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.48, 0.36, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=painted_cast,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=painted_cast,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=dark_steel,
        name="slew_bearing",
    )
    base.visual(
        Box((0.13, 0.025, 0.16)),
        origin=Origin(xyz=(0.0, 0.045, 0.12)),
        material=painted_cast,
        name="front_web",
    )
    base.visual(
        Box((0.13, 0.025, 0.16)),
        origin=Origin(xyz=(0.0, -0.045, 0.12)),
        material=painted_cast,
        name="rear_web",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.087, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="rotary_table",
    )
    arm.visual(
        Cylinder(radius=0.066, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=painted_cast,
        name="pivot_boss",
    )
    arm.visual(
        Box((0.78, 0.080, 0.070)),
        origin=Origin(xyz=(0.455, 0.0, 0.095)),
        material=painted_cast,
        name="main_beam",
    )
    arm.visual(
        Box((0.62, 0.018, 0.012)),
        origin=Origin(xyz=(0.455, 0.049, 0.095)),
        material=oiled_steel,
        name="guide_rail_0",
    )
    arm.visual(
        Box((0.62, 0.018, 0.012)),
        origin=Origin(xyz=(0.455, -0.049, 0.095)),
        material=oiled_steel,
        name="guide_rail_1",
    )
    arm.visual(
        Box((0.036, 0.118, 0.090)),
        origin=Origin(xyz=(0.115, 0.0, 0.105)),
        material=dark_steel,
        name="travel_stop_0",
    )
    arm.visual(
        Box((0.036, 0.118, 0.090)),
        origin=Origin(xyz=(0.515, 0.0, 0.105)),
        material=dark_steel,
        name="travel_stop_1",
    )
    arm.visual(
        Box((0.42, 0.018, 0.004)),
        origin=Origin(xyz=(0.315, -0.018, 0.132)),
        material=safety,
        name="travel_scale",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.170, 0.170, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_steel,
        name="saddle_top",
    )
    carriage.visual(
        Box((0.155, 0.024, 0.096)),
        origin=Origin(xyz=(0.0, 0.070, 0.087)),
        material=dark_steel,
        name="side_cheek_0",
    )
    carriage.visual(
        Box((0.155, 0.024, 0.096)),
        origin=Origin(xyz=(0.0, -0.070, 0.087)),
        material=dark_steel,
        name="side_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.190),
        origin=Origin(xyz=(-0.045, 0.0, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="roller_pin_0",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.190),
        origin=Origin(xyz=(0.045, 0.0, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="roller_pin_1",
    )
    carriage.visual(
        Cylinder(radius=0.007, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=oiled_steel,
        name="clamp_stem",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=black,
        name="clamp_knob",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-2.094, upper=2.094),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    sweep = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")

    ctx.check(
        "arm sweep is about 120 degrees each way",
        sweep.motion_limits is not None
        and abs(sweep.motion_limits.lower + 2.094) < 0.01
        and abs(sweep.motion_limits.upper - 2.094) < 0.01,
        details=f"limits={sweep.motion_limits}",
    )
    ctx.check(
        "carriage travel is 180 mm",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and abs(slide.motion_limits.upper - 0.180) < 0.002,
        details=f"limits={slide.motion_limits}",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="z",
        positive_elem="saddle_top",
        negative_elem="main_beam",
        min_gap=0.004,
        max_gap=0.010,
        name="saddle clears top of arm beam",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="x",
        elem_a="saddle_top",
        elem_b="main_beam",
        min_overlap=0.14,
        name="carriage sits over the arm rail",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.180}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="saddle_top",
            elem_b="main_beam",
            min_overlap=0.14,
            name="extended carriage remains on the arm",
        )
    ctx.check(
        "positive slide moves carriage outward along the arm",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.17,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({sweep: 2.094}):
        swept_pos = ctx.part_world_position(carriage)
    ctx.check(
        "positive sweep rotates carriage in plan",
        rest_pos is not None
        and swept_pos is not None
        and swept_pos[1] > rest_pos[1] + 0.15
        and abs(swept_pos[2] - rest_pos[2]) < 0.005,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
