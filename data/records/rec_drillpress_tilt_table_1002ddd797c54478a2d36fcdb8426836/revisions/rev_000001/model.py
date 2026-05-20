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
    model = ArticulatedObject(name="woodworking_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    table_iron = model.material("table_iron", rgba=(0.30, 0.31, 0.33, 1.0))
    fence_finish = model.material("fence_finish", rgba=(0.74, 0.58, 0.35, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.58, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base",
    )
    frame.visual(
        Cylinder(radius=0.045, length=1.24),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=machined_steel,
        name="column",
    )
    frame.visual(
        Box((0.18, 0.18, 0.26)),
        origin=Origin(xyz=(0.0, 0.09, 1.31)),
        material=cast_iron,
        name="column_head",
    )
    frame.visual(
        Box((0.42, 0.26, 0.28)),
        origin=Origin(xyz=(0.0, 0.26, 1.40)),
        material=cast_iron,
        name="head",
    )
    frame.visual(
        Cylinder(radius=0.085, length=0.34),
        origin=Origin(xyz=(0.0, 0.03, 1.42), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="motor",
    )
    frame.visual(
        Box((0.34, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.20, 1.60)),
        material=cast_iron,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(0.0, 0.35, 1.28)),
        material=machined_steel,
        name="quill_housing",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.35, 1.11)),
        material=machined_steel,
        name="spindle",
    )
    frame.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(0.0, 0.35, 0.96)),
        material=machined_steel,
        name="chuck",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.06),
        origin=Origin(xyz=(0.0, 0.35, 0.87)),
        material=machined_steel,
        name="chuck_tip",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.14, 0.06, 0.18)),
        material=machined_steel,
        name="saddle",
    )
    table_carriage.visual(
        Box((0.08, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.09, 0.03)),
        material=cast_iron,
        name="support_arm",
    )
    table_carriage.visual(
        Box((0.26, 0.04, 0.08)),
        origin=Origin(xyz=(0.0, 0.15, 0.04)),
        material=cast_iron,
        name="tilt_yoke",
    )

    table = model.part("table")
    table.visual(
        Box((0.40, 0.32, 0.035)),
        origin=Origin(xyz=(0.0, 0.16, 0.0175)),
        material=table_iron,
        name="top",
    )
    table.visual(
        Box((0.12, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.14, -0.015)),
        material=cast_iron,
        name="rib",
    )
    table.visual(
        Box((0.24, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.02, -0.002)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.46, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.335, 0.0)),
        material=machined_steel,
        name="front_guide",
    )

    fence = model.part("fence")
    fence.visual(
        Box((0.50, 0.025, 0.08)),
        origin=Origin(xyz=(0.0, 0.04, 0.07)),
        material=fence_finish,
        name="beam",
    )
    fence.visual(
        Box((0.20, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=machined_steel,
        name="slider_carriage",
    )
    for x_pos, strut_name in ((-0.07, "strut_0"), (0.07, "strut_1")):
        fence.visual(
            Box((0.03, 0.02, 0.09)),
            origin=Origin(xyz=(x_pos, 0.025, 0.02)),
            material=machined_steel,
            name=strut_name,
        )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.075, 0.57)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.20,
            lower=0.0,
            upper=0.16,
        ),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.17, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "fence_slide",
        ArticulationType.PRISMATIC,
        parent=table,
        child=fence,
        origin=Origin(xyz=(0.0, 0.335, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.40,
            lower=-0.12,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    fence = object_model.get_part("fence")

    column_slide = object_model.get_articulation("column_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    fence_slide = object_model.get_articulation("fence_slide")

    slide_upper = column_slide.motion_limits.upper or 0.0
    tilt_check = min(table_tilt.motion_limits.upper or 0.45, 0.45)
    fence_lower = fence_slide.motion_limits.lower or 0.0
    fence_upper = fence_slide.motion_limits.upper or 0.0

    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="chuck_tip",
        negative_elem="top",
        min_gap=0.16,
        max_gap=0.24,
        name="rest table sits below the spindle nose",
    )

    rest_carriage_pos = ctx.part_world_position(table_carriage)
    with ctx.pose({column_slide: slide_upper}):
        ctx.expect_gap(
            frame,
            table,
            axis="z",
            positive_elem="chuck_tip",
            negative_elem="top",
            min_gap=0.02,
            name="raised table still clears the chuck tip",
        )
        raised_carriage_pos = ctx.part_world_position(table_carriage)

    ctx.check(
        "table carriage climbs the column",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.12,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_front_guide_aabb = ctx.part_element_world_aabb(table, elem="front_guide")
    with ctx.pose({table_tilt: tilt_check}):
        tilted_front_guide_aabb = ctx.part_element_world_aabb(table, elem="front_guide")

    ctx.check(
        "positive table tilt raises the front guide",
        rest_front_guide_aabb is not None
        and tilted_front_guide_aabb is not None
        and tilted_front_guide_aabb[1][2] > rest_front_guide_aabb[1][2] + 0.08,
        details=f"rest={rest_front_guide_aabb}, tilted={tilted_front_guide_aabb}",
    )

    ctx.expect_within(
        fence,
        table,
        axes="y",
        inner_elem="slider_carriage",
        outer_elem="front_guide",
        margin=0.001,
        name="fence carriage stays aligned with the front guide depth",
    )
    ctx.expect_gap(
        table,
        fence,
        axis="z",
        positive_elem="front_guide",
        negative_elem="slider_carriage",
        max_gap=0.001,
        max_penetration=0.0,
        name="fence carriage rides directly under the front guide",
    )
    ctx.expect_overlap(
        fence,
        table,
        axes="x",
        elem_a="slider_carriage",
        elem_b="front_guide",
        min_overlap=0.19,
        name="centered fence remains engaged on the front guide",
    )

    with ctx.pose({fence_slide: fence_lower}):
        lower_fence_pos = ctx.part_world_position(fence)
        ctx.expect_overlap(
            fence,
            table,
            axes="x",
            elem_a="slider_carriage",
            elem_b="front_guide",
            min_overlap=0.18,
            name="fence stays retained at the left travel stop",
        )

    with ctx.pose({fence_slide: fence_upper}):
        upper_fence_pos = ctx.part_world_position(fence)
        ctx.expect_overlap(
            fence,
            table,
            axes="x",
            elem_a="slider_carriage",
            elem_b="front_guide",
            min_overlap=0.18,
            name="fence stays retained at the right travel stop",
        )

    ctx.check(
        "fence traverses across the table front edge",
        lower_fence_pos is not None
        and upper_fence_pos is not None
        and upper_fence_pos[0] > lower_fence_pos[0] + 0.20,
        details=f"lower={lower_fence_pos}, upper={upper_fence_pos}",
    )

    return ctx.report()


object_model = build_object_model()
