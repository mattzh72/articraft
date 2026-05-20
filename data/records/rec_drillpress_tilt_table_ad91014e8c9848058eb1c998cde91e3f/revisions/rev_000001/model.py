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

    machine_orange = model.material("machine_orange", rgba=(0.85, 0.43, 0.16, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.24, 0.26, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_motor = model.material("dark_motor", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.46, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=machine_orange,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.045, length=1.20),
        origin=Origin(xyz=(0.0, -0.10, 0.69)),
        material=steel,
        name="column",
    )
    base.visual(
        Box((0.32, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, -0.03, 1.31)),
        material=machine_orange,
        name="head",
    )
    base.visual(
        Box((0.38, 0.30, 0.14)),
        origin=Origin(xyz=(0.02, -0.05, 1.50)),
        material=machine_orange,
        name="belt_cover",
    )
    base.visual(
        Box((0.12, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.18, 1.48)),
        material=machine_orange,
        name="motor_mount",
    )
    base.visual(
        Cylinder(radius=0.09, length=0.20),
        origin=Origin(xyz=(0.0, -0.30, 1.48), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_motor,
        name="motor",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.22),
        origin=Origin(xyz=(0.0, 0.02, 1.12)),
        material=steel,
        name="quill",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.09),
        origin=Origin(xyz=(0.0, 0.03, 0.965)),
        material=steel,
        name="chuck",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.08),
        origin=Origin(xyz=(0.0, 0.035, 0.88)),
        material=steel,
        name="spindle",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.14, 0.06, 0.11)),
        origin=Origin(xyz=(0.0, 0.075, 0.0)),
        material=machine_orange,
        name="clamp_body",
    )
    carriage.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=steel,
        name="guide_0",
    )
    carriage.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=steel,
        name="guide_1",
    )
    carriage.visual(
        Box((0.018, 0.075, 0.09)),
        origin=Origin(xyz=(-0.065, 0.038, 0.0)),
        material=machine_orange,
        name="link_0",
    )
    carriage.visual(
        Box((0.018, 0.075, 0.09)),
        origin=Origin(xyz=(0.065, 0.038, 0.0)),
        material=machine_orange,
        name="link_1",
    )
    carriage.visual(
        Box((0.10, 0.13, 0.045)),
        origin=Origin(xyz=(0.0, 0.145, -0.050)),
        material=machine_orange,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.14, 0.07, 0.03)),
        origin=Origin(xyz=(0.0, 0.195, -0.056)),
        material=machine_orange,
        name="brace",
    )
    carriage.visual(
        Box((0.20, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.225, -0.048)),
        material=machine_orange,
        name="trunnion_saddle",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.023, length=0.22),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.18, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.095, 0.055)),
        material=cast_iron,
        name="front_bridge",
    )
    table.visual(
        Box((0.46, 0.34, 0.038)),
        origin=Origin(xyz=(0.0, 0.040, 0.078)),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Box((0.42, 0.028, 0.060)),
        origin=Origin(xyz=(0.0, 0.170, 0.049)),
        material=cast_iron,
        name="front_lip",
    )
    table.visual(
        Box((0.032, 0.140, 0.075)),
        origin=Origin(xyz=(-0.102, 0.065, 0.043)),
        material=cast_iron,
        name="rib_0",
    )
    table.visual(
        Box((0.032, 0.140, 0.075)),
        origin=Origin(xyz=(0.102, 0.065, 0.043)),
        material=cast_iron,
        name="rib_1",
    )

    fence = model.part("fence")
    fence.visual(
        Box((0.42, 0.028, 0.06)),
        material=cast_iron,
        name="fence_body",
    )
    fence.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=(0.0, 0.050, 0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="front_rail",
    )
    fence.visual(
        Box((0.020, 0.036, 0.030)),
        origin=Origin(xyz=(-0.170, 0.032, 0.007)),
        material=steel,
        name="rail_bracket_0",
    )
    fence.visual(
        Box((0.020, 0.036, 0.030)),
        origin=Origin(xyz=(0.170, 0.032, 0.007)),
        material=steel,
        name="rail_bracket_1",
    )

    stop_block = model.part("stop_block")
    stop_block.visual(
        Box((0.055, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=cast_iron,
        name="stop_body",
    )
    stop_block.visual(
        Box((0.055, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=cast_iron,
        name="stop_jaw",
    )
    stop_block.visual(
        Box((0.055, 0.016, 0.068)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=cast_iron,
        name="stop_bridge",
    )
    stop_block.visual(
        Box((0.012, 0.020, 0.070)),
        origin=Origin(xyz=(0.022, 0.024, 0.0)),
        material=steel,
        name="stop_flag",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.10, 0.67)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.18, upper=0.13, effort=220.0, velocity=0.16),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.225, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.78, upper=0.78, effort=90.0, velocity=0.8),
    )
    model.articulation(
        "fence_mount",
        ArticulationType.FIXED,
        parent=table,
        child=fence,
        origin=Origin(xyz=(0.0, -0.085, 0.127)),
    )
    model.articulation(
        "stop_slide",
        ArticulationType.PRISMATIC,
        parent=fence,
        child=stop_block,
        origin=Origin(xyz=(0.0, 0.050, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.15, upper=0.15, effort=20.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    fence = object_model.get_part("fence")
    stop_block = object_model.get_part("stop_block")

    carriage_slide = object_model.get_articulation("carriage_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    stop_slide = object_model.get_articulation("stop_slide")

    ctx.expect_gap(
        table,
        base,
        axis="z",
        positive_elem="table_top",
        negative_elem="base_plate",
        min_gap=0.55,
        name="table sits well above the floor base",
    )
    ctx.expect_gap(
        fence,
        table,
        axis="z",
        positive_elem="fence_body",
        negative_elem="table_top",
        max_gap=0.001,
        max_penetration=0.0,
        name="fence rests on the table surface",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="z",
        elem_a="guide_0",
        elem_b="column",
        min_overlap=0.09,
        name="carriage remains engaged on the column",
    )

    rest_table_pos = ctx.part_world_position(table)
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        raised_table_pos = ctx.part_world_position(table)
        ctx.expect_overlap(
            carriage,
            base,
            axes="z",
            elem_a="guide_0",
            elem_b="column",
            min_overlap=0.09,
            name="raised carriage still wraps the column",
        )

    ctx.check(
        "table slides upward on the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.10,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_top_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: 0.55}):
        tilted_top_aabb = ctx.part_element_world_aabb(table, elem="table_top")

    rest_height = None if rest_top_aabb is None else rest_top_aabb[1][2] - rest_top_aabb[0][2]
    tilted_height = None if tilted_top_aabb is None else tilted_top_aabb[1][2] - tilted_top_aabb[0][2]
    ctx.check(
        "table tilt changes the tabletop pitch",
        rest_height is not None and tilted_height is not None and tilted_height > rest_height + 0.10,
        details=f"rest_height={rest_height}, tilted_height={tilted_height}",
    )

    with ctx.pose({stop_slide: stop_slide.motion_limits.lower}):
        left_stop_pos = ctx.part_world_position(stop_block)
        ctx.expect_overlap(
            stop_block,
            fence,
            axes="x",
            elem_a="stop_body",
            elem_b="front_rail",
            min_overlap=0.02,
            name="stop block stays captured at the left rail limit",
        )
    with ctx.pose({stop_slide: stop_slide.motion_limits.upper}):
        right_stop_pos = ctx.part_world_position(stop_block)
        ctx.expect_overlap(
            stop_block,
            fence,
            axes="x",
            elem_a="stop_body",
            elem_b="front_rail",
            min_overlap=0.02,
            name="stop block stays captured at the right rail limit",
        )

    ctx.check(
        "stop block traverses the fence rail",
        left_stop_pos is not None
        and right_stop_pos is not None
        and right_stop_pos[0] > left_stop_pos[0] + 0.25,
        details=f"left={left_stop_pos}, right={right_stop_pos}",
    )

    return ctx.report()


object_model = build_object_model()
