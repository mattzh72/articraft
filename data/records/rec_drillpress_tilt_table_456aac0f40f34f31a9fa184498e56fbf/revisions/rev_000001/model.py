from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _head_casting_mesh():
    body = cq.Workplane("XY").box(0.14, 0.105, 0.11).translate((0.100, 0.0, 0.015))
    rear_saddle = cq.Workplane("XY").box(0.04, 0.11, 0.13).translate((0.045, 0.0, 0.0))
    top_cover = cq.Workplane("XY").box(0.15, 0.10, 0.04).translate((0.100, 0.0, 0.085))
    casting = rear_saddle.union(body).union(top_cover)
    depth_slot = cq.Workplane("XY").box(0.020, 0.018, 0.080).translate((0.160, 0.0, 0.055))

    return casting.cut(depth_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_drill_press")

    cast_gray = model.material("cast_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    table_gray = model.material("table_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.30, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_gray,
        name="base",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.56),
        origin=Origin(xyz=(-0.09, 0.0, 0.31)),
        material=steel,
        name="column",
    )
    frame.visual(
        Box((0.06, 0.03, 0.18)),
        origin=Origin(xyz=(-0.07, 0.0, 0.12)),
        material=cast_gray,
        name="rear_web",
    )
    frame.visual(
        Box((0.09, 0.05, 0.03)),
        origin=Origin(xyz=(-0.045, 0.0, 0.24)),
        material=cast_gray,
        name="table_arm",
    )
    frame.visual(
        Box((0.030, 0.14, 0.018)),
        origin=Origin(xyz=(0.009, 0.0, 0.229)),
        material=cast_gray,
        name="yoke_bridge",
    )
    frame.visual(
        Box((0.016, 0.012, 0.028)),
        origin=Origin(xyz=(0.024, 0.067, 0.250)),
        material=cast_gray,
        name="yoke_ear_0",
    )
    frame.visual(
        Box((0.016, 0.012, 0.028)),
        origin=Origin(xyz=(0.024, -0.067, 0.250)),
        material=cast_gray,
        name="yoke_ear_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_casting_mesh(), "head_casting_v4"),
        material=cast_gray,
        name="casting",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.150, 0.0, -0.020)),
        material=cast_gray,
        name="quill_housing",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.055),
        origin=Origin(xyz=(0.150, 0.0, -0.072)),
        material=steel,
        name="spindle_nose",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.036),
        origin=Origin(xyz=(0.125, 0.070, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_gray,
        name="feed_boss",
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="hub",
    )
    arm_length = 0.072
    knob_offset = 0.078
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        dx = math.cos(angle)
        dz = math.sin(angle)
        feed_handle.visual(
            Cylinder(radius=0.004, length=arm_length),
            origin=Origin(
                xyz=(dx * arm_length / 2.0, 0.0, dz * arm_length / 2.0),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material=handle_black,
            name=f"spoke_{index}",
        )
        feed_handle.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(
                xyz=(dx * knob_offset, 0.0, dz * knob_offset),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"grip_{index}",
        )

    depth_stop = model.part("depth_stop")
    depth_stop.visual(
        Box((0.020, 0.018, 0.050)),
        material=steel,
        name="slider",
    )
    depth_stop.visual(
        Box((0.012, 0.028, 0.014)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=handle_black,
        name="thumb_pad",
    )

    table = model.part("table")
    table.visual(
        Box((0.16, 0.16, 0.012)),
        origin=Origin(xyz=(0.05, 0.0, 0.028)),
        material=table_gray,
        name="table_top",
    )
    table.visual(
        Box((0.05, 0.08, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=cast_gray,
        name="table_rib",
    )
    table.visual(
        Cylinder(radius=0.010, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion",
    )

    model.articulation(
        "frame_to_head",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head,
        origin=Origin(xyz=(-0.09, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=0.0, upper=0.08),
    )
    model.articulation(
        "frame_to_table",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(0.03, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "head_to_feed_handle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=feed_handle,
        origin=Origin(xyz=(0.125, 0.098, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )
    model.articulation(
        "head_to_depth_stop",
        ArticulationType.PRISMATIC,
        parent=head,
        child=depth_stop,
        origin=Origin(xyz=(0.160, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=-0.01, upper=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    table = object_model.get_part("table")
    feed_handle = object_model.get_part("feed_handle")
    depth_stop = object_model.get_part("depth_stop")
    head_slide = object_model.get_articulation("frame_to_head")
    table_tilt = object_model.get_articulation("frame_to_table")
    handle_spin = object_model.get_articulation("head_to_feed_handle")
    stop_slide = object_model.get_articulation("head_to_depth_stop")

    ctx.allow_overlap(
        depth_stop,
        head,
        elem_a="slider",
        elem_b="casting",
        reason="The depth-stop rail is intentionally represented as sliding inside the head casting's front slot proxy.",
    )
    ctx.allow_overlap(
        depth_stop,
        head,
        elem_a="thumb_pad",
        elem_b="casting",
        reason="The depth-stop thumb pad emerges from the same simplified front-slot proxy on the head casting.",
    )

    ctx.expect_gap(
        head,
        table,
        axis="z",
        min_gap=0.015,
        name="head clears the table in the rest pose",
    )
    ctx.expect_overlap(
        table,
        frame,
        axes="x",
        min_overlap=0.02,
        name="table stays in front of the column support",
    )

    head_rest = ctx.part_world_position(head)
    with ctx.pose({head_slide: head_slide.motion_limits.upper}):
        head_raised = ctx.part_world_position(head)
    ctx.check(
        "head slides upward on the column",
        head_rest is not None and head_raised is not None and head_raised[2] > head_rest[2] + 0.05,
        details=f"rest={head_rest}, raised={head_raised}",
    )

    table_flat = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: math.radians(25.0)}):
        table_tipped = ctx.part_element_world_aabb(table, elem="table_top")

    flat_front_height = table_flat[1][2] if table_flat is not None else None
    tipped_front_height = table_tipped[1][2] if table_tipped is not None else None
    ctx.check(
        "table tilts about its horizontal hinge",
        flat_front_height is not None
        and tipped_front_height is not None
        and tipped_front_height > flat_front_height + 0.02,
        details=f"flat={table_flat}, tipped={table_tipped}",
    )

    handle_rest = ctx.part_element_world_aabb(feed_handle, elem="spoke_0")
    with ctx.pose({handle_spin: math.pi / 2.0}):
        handle_turned = ctx.part_element_world_aabb(feed_handle, elem="spoke_0")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    rest_center = aabb_center(handle_rest)
    turned_center = aabb_center(handle_turned)
    ctx.check(
        "feed handle rotates around the quill-feed axis",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.02
        and abs(rest_center[2] - turned_center[2]) > 0.02,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    stop_rest = ctx.part_world_position(depth_stop)
    with ctx.pose({stop_slide: stop_slide.motion_limits.upper}):
        stop_high = ctx.part_world_position(depth_stop)
    ctx.check(
        "depth stop slider moves vertically in the front slot",
        stop_rest is not None and stop_high is not None and stop_high[2] > stop_rest[2] + 0.015,
        details=f"rest={stop_rest}, high={stop_high}",
    )
    ctx.expect_within(
        depth_stop,
        head,
        axes="xy",
        elem_a="slider",
        elem_b="casting",
        margin=0.005,
        name="depth stop stays captured on the head front",
    )

    return ctx.report()


object_model = build_object_model()
