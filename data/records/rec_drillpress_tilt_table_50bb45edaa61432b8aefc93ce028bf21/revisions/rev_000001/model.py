from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _hollow_collar_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """A simple centered vertical bearing collar with a real clearance bore."""
    collar = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )
    return mesh_from_cadquery(collar, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press")

    cast_green = Material("cast_green", rgba=(0.12, 0.27, 0.20, 1.0))
    dark_cast = Material("dark_cast", rgba=(0.08, 0.09, 0.09, 1.0))
    blued_cast = Material("blued_cast", rgba=(0.08, 0.16, 0.24, 1.0))
    steel = Material("polished_steel", rgba=(0.65, 0.68, 0.66, 1.0))
    black = Material("black_oxide", rgba=(0.01, 0.012, 0.012, 1.0))
    safety_red = Material("safety_red", rgba=(0.65, 0.05, 0.03, 1.0))

    base = model.part("base_column")
    base.visual(
        Box((0.92, 0.56, 0.08)),
        origin=Origin(xyz=(0.31, 0.0, 0.04)),
        material=cast_green,
        name="floor_base",
    )
    base.visual(
        Cylinder(radius=0.055, length=1.49),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=steel,
        name="round_column",
    )
    base.visual(
        Cylinder(radius=0.086, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.415)),
        material=black,
        name="top_thrust_washer",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=cast_green,
        name="table_collar",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.285),
        origin=Origin(xyz=(0.175, 0.0, 0.535), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_green,
        name="table_support_arm",
    )
    base.visual(
        Box((0.095, 0.185, 0.035)),
        origin=Origin(xyz=(0.315, 0.0, 0.480)),
        material=cast_green,
        name="yoke_bridge",
    )
    base.visual(
        Box((0.055, 0.055, 0.090)),
        origin=Origin(xyz=(0.285, 0.0, 0.515)),
        material=cast_green,
        name="yoke_web",
    )
    for y, visual_name in ((0.084, "yoke_cheek_0"), (-0.084, "yoke_cheek_1")):
        base.visual(
            Box((0.080, 0.026, 0.100)),
            origin=Origin(xyz=(0.345, y, 0.535)),
            material=cast_green,
            name=visual_name,
        )
        base.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(0.345, y * 1.19, 0.535), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"hinge_boss_{0 if y > 0 else 1}",
        )

    arm = model.part("radial_arm")
    arm.visual(
        _hollow_collar_mesh(0.087, 0.063, 0.150, "swing_collar"),
        material=cast_green,
        name="swing_collar",
    )
    arm.visual(
        Box((0.760, 0.120, 0.105)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=cast_green,
        name="arm_beam",
    )
    arm.visual(
        Box((0.730, 0.018, 0.020)),
        origin=Origin(xyz=(0.470, 0.043, -0.066)),
        material=steel,
        name="slide_way_0",
    )
    arm.visual(
        Box((0.730, 0.018, 0.020)),
        origin=Origin(xyz=(0.470, -0.043, -0.066)),
        material=steel,
        name="slide_way_1",
    )
    arm.visual(
        Box((0.100, 0.145, 0.130)),
        origin=Origin(xyz=(0.860, 0.0, 0.0)),
        material=dark_cast,
        name="arm_end_stop",
    )
    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.7, lower=-1.75, upper=1.75),
    )

    head = model.part("drill_head")
    head.visual(
        Box((0.220, 0.185, 0.035)),
        origin=Origin(xyz=(0.000, 0.0, 0.088)),
        material=blued_cast,
        name="saddle_top",
    )
    for y, visual_name in ((0.095, "saddle_side_0"), (-0.095, "saddle_side_1")):
        head.visual(
            Box((0.220, 0.026, 0.180)),
            origin=Origin(xyz=(0.000, y, 0.000)),
            material=blued_cast,
            name=visual_name,
        )
    head.visual(
        Box((0.170, 0.070, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, 0.0615)),
        material=steel,
        name="top_slide_shoe",
    )
    head.visual(
        Box((0.220, 0.185, 0.035)),
        origin=Origin(xyz=(0.000, 0.0, -0.105)),
        material=blued_cast,
        name="saddle_bottom",
    )
    head.visual(
        Box((0.230, 0.205, 0.300)),
        origin=Origin(xyz=(0.020, 0.0, -0.270)),
        material=blued_cast,
        name="gearbox_casting",
    )
    head.visual(
        Cylinder(radius=0.070, length=0.180),
        origin=Origin(xyz=(-0.030, 0.175, -0.235), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="side_motor",
    )
    head.visual(
        Cylinder(radius=0.037, length=0.025),
        origin=Origin(xyz=(-0.020, -0.1125, -0.215), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blued_cast,
        name="feed_boss",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.170),
        origin=Origin(xyz=(0.030, 0.0, -0.470)),
        material=steel,
        name="quill",
    )
    head.visual(
        Cylinder(radius=0.025, length=0.075),
        origin=Origin(xyz=(0.030, 0.0, -0.590)),
        material=black,
        name="chuck",
    )
    head.visual(
        Cylinder(radius=0.0065, length=0.205),
        origin=Origin(xyz=(0.030, 0.0, -0.730)),
        material=steel,
        name="drill_bit",
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.450, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.20, lower=-0.220, upper=0.260),
    )

    feed = model.part("feed_handle")
    feed.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="feed_hub",
    )
    feed.visual(
        Cylinder(radius=0.008, length=0.165),
        origin=Origin(xyz=(0.0825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="feed_spoke",
    )
    feed.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(xyz=(0.172, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_red,
        name="feed_grip",
    )
    model.articulation(
        "head_to_feed",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed,
        origin=Origin(xyz=(-0.020, -0.145, -0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-1.0, upper=0.9),
    )

    table = model.part("round_table")
    table.visual(
        Cylinder(radius=0.230, length=0.040),
        origin=Origin(xyz=(0.130, 0.0, 0.085)),
        material=dark_cast,
        name="table_disk",
    )
    for y, visual_name in ((0.070, "table_slot_0"), (-0.070, "table_slot_1")):
        table.visual(
            Box((0.320, 0.022, 0.005)),
            origin=Origin(xyz=(0.130, y, 0.104)),
            material=black,
            name=visual_name,
        )
    table.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.130, 0.0, 0.104)),
        material=black,
        name="center_hole",
    )
    table.visual(
        Cylinder(radius=0.027, length=0.142),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_barrel",
    )
    table.visual(
        Box((0.145, 0.075, 0.055)),
        origin=Origin(xyz=(0.075, 0.0, 0.045)),
        material=dark_cast,
        name="trunnion_web",
    )
    model.articulation(
        "column_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.345, 0.0, 0.535)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.7, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    arm = object_model.get_part("radial_arm")
    head = object_model.get_part("drill_head")
    table = object_model.get_part("round_table")
    feed = object_model.get_part("feed_handle")
    arm_swing = object_model.get_articulation("column_to_arm")
    head_slide = object_model.get_articulation("arm_to_head")
    table_tilt = object_model.get_articulation("column_to_table")
    feed_swing = object_model.get_articulation("head_to_feed")

    ctx.expect_overlap(
        head,
        arm,
        axes="x",
        elem_a="saddle_top",
        elem_b="arm_beam",
        min_overlap=0.12,
        name="head saddle is retained on the radial arm at center travel",
    )
    ctx.expect_overlap(
        head,
        table,
        axes="xy",
        elem_a="drill_bit",
        elem_b="table_disk",
        min_overlap=0.004,
        name="drill bit is above the round work table at the center position",
    )
    ctx.expect_gap(
        head,
        table,
        axis="z",
        positive_elem="drill_bit",
        negative_elem="table_disk",
        min_gap=0.025,
        max_gap=0.090,
        name="drill bit clears the table surface",
    )
    ctx.expect_contact(
        feed,
        head,
        elem_a="feed_hub",
        elem_b="feed_boss",
        contact_tol=0.002,
        name="feed handle hub seats on the head boss",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.240}):
        extended_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            arm,
            axes="x",
            elem_a="saddle_top",
            elem_b="arm_beam",
            min_overlap=0.12,
            name="head remains on the arm at outward slide limit",
        )
    ctx.check(
        "head slides outward along the horizontal arm",
        rest_head is not None
        and extended_head is not None
        and extended_head[0] > rest_head[0] + 0.20,
        details=f"rest={rest_head}, extended={extended_head}",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({arm_swing: 0.70}):
        swung_head = ctx.part_world_position(head)
    ctx.check(
        "radial arm swings around the round column",
        rest_head is not None
        and swung_head is not None
        and abs(swung_head[1] - rest_head[1]) > 0.20,
        details=f"rest={rest_head}, swung={swung_head}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_disk")
    with ctx.pose({table_tilt: 0.55}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_disk")
    ctx.check(
        "round table tilts on its hinge bracket",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and (tilted_table_aabb[1][2] - tilted_table_aabb[0][2])
        > (rest_table_aabb[1][2] - rest_table_aabb[0][2]) + 0.06,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_feed_aabb = ctx.part_world_aabb(feed)
    with ctx.pose({feed_swing: 0.65}):
        moved_feed_aabb = ctx.part_world_aabb(feed)
    ctx.check(
        "feed handle rotates on the side of the drill head",
        rest_feed_aabb is not None
        and moved_feed_aabb is not None
        and abs(moved_feed_aabb[0][2] - rest_feed_aabb[0][2]) > 0.05,
        details=f"rest={rest_feed_aabb}, moved={moved_feed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
