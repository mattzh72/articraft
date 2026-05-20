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
    model = ArticulatedObject(name="wall_slide_swing_fixture")

    powder_coat = Material("warm_white_powder_coat", color=(0.78, 0.76, 0.70, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    carriage_paint = Material("carriage_blue", color=(0.08, 0.18, 0.32, 1.0))
    safety_yellow = Material("safety_yellow", color=(0.94, 0.68, 0.12, 1.0))
    black_rubber = Material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    dark_fastener = Material("dark_fastener", color=(0.05, 0.052, 0.055, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((1.20, 0.035, 0.42)),
        origin=Origin(),
        material=powder_coat,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.95, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.035, 0.075)),
        material=brushed_steel,
        name="upper_rail",
    )
    backplate.visual(
        Box((0.95, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.035, -0.075)),
        material=brushed_steel,
        name="lower_rail",
    )
    for x in (-0.50, 0.50):
        backplate.visual(
            Box((0.035, 0.060, 0.25)),
            origin=Origin(xyz=(x, 0.0475, 0.0)),
            material=dark_fastener,
            name=f"end_stop_{0 if x < 0 else 1}",
        )
    for i, (x, z) in enumerate(((-0.52, 0.155), (0.52, 0.155), (-0.52, -0.155), (0.52, -0.155))):
        backplate.visual(
            Cylinder(radius=0.024, length=0.008),
            origin=Origin(xyz=(x, 0.0215, z), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_fastener,
            name=f"screw_head_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.15, 0.060, 0.23)),
        origin=Origin(),
        material=carriage_paint,
        name="slider_block",
    )
    carriage.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="front_bushing",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.32, 0.028, 0.045)),
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        material=safety_yellow,
        name="arm_bar",
    )
    arm.visual(
        Box((0.095, 0.035, 0.10)),
        origin=Origin(xyz=(0.4075, 0.0, 0.0)),
        material=black_rubber,
        name="end_pad",
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(-0.30, 0.0825, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.60),
    )
    model.articulation(
        "arm_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-1.10, upper=1.10),
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

    backplate = object_model.get_part("backplate")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("guide_slide")
    pivot = object_model.get_articulation("arm_pivot")

    ctx.expect_contact(
        carriage,
        backplate,
        elem_a="slider_block",
        elem_b="upper_rail",
        name="carriage rides upper guide rail",
    )
    ctx.expect_contact(
        carriage,
        backplate,
        elem_a="slider_block",
        elem_b="lower_rail",
        name="carriage rides lower guide rail",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        positive_elem="pivot_hub",
        negative_elem="slider_block",
        max_gap=0.001,
        max_penetration=0.0,
        name="swing hub is seated on carriage face",
    )
    ctx.expect_overlap(
        arm,
        carriage,
        axes="xz",
        elem_a="pivot_hub",
        elem_b="slider_block",
        min_overlap=0.05,
        name="pivot hub is centered over carriage",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_pad_aabb = ctx.part_element_world_aabb(arm, elem="end_pad")
    rest_pad_z = None
    if rest_pad_aabb is not None:
        rest_pad_z = 0.5 * (rest_pad_aabb[0][2] + rest_pad_aabb[1][2])

    with ctx.pose({slide: 0.60}):
        ctx.expect_within(
            carriage,
            backplate,
            axes="x",
            inner_elem="slider_block",
            outer_elem="upper_rail",
            margin=0.0,
            name="extended carriage remains within guide rail",
        )
        extended_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "guide stage translates horizontally",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.55
        and abs(extended_carriage[2] - rest_carriage[2]) < 1e-6,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    with ctx.pose({pivot: 0.75}):
        swung_pad_aabb = ctx.part_element_world_aabb(arm, elem="end_pad")
        swung_pad_z = None
        if swung_pad_aabb is not None:
            swung_pad_z = 0.5 * (swung_pad_aabb[0][2] + swung_pad_aabb[1][2])

    ctx.check(
        "arm swings about carriage pivot",
        rest_pad_z is not None and swung_pad_z is not None and swung_pad_z < rest_pad_z - 0.20,
        details=f"rest_pad_z={rest_pad_z}, swung_pad_z={swung_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()
