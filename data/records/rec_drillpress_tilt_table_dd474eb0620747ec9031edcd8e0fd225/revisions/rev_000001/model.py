from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.25, 0.28, 0.30, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.43, 0.47, 0.49, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.62, 0.46, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base",
    )
    frame.visual(
        Box((0.22, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.07, 0.105)),
        material=machine_gray,
        name="base_riser",
    )
    frame.visual(
        Cylinder(radius=0.080, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=machine_gray,
        name="column_foot",
    )
    frame.visual(
        Cylinder(radius=0.044, length=1.38),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=polished_steel,
        name="column",
    )
    frame.visual(
        Box((0.14, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, 0.00, 1.28)),
        material=machine_gray,
        name="head_clamp",
    )
    frame.visual(
        Box((0.24, 0.30, 0.30)),
        origin=Origin(xyz=(0.0, 0.03, 1.31)),
        material=machine_gray,
        name="head",
    )
    frame.visual(
        Box((0.34, 0.26, 0.13)),
        origin=Origin(xyz=(0.0, -0.01, 1.50)),
        material=cast_iron,
        name="belt_cover",
    )
    frame.visual(
        Box((0.12, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, -0.10, 1.40)),
        material=machine_gray,
        name="motor_mount",
    )
    frame.visual(
        Cylinder(radius=0.085, length=0.22),
        origin=Origin(xyz=(0.0, -0.22, 1.53), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="motor",
    )
    frame.visual(
        Box((0.14, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.15, 1.28)),
        material=machine_gray,
        name="nose_block",
    )
    frame.visual(
        Cylinder(radius=0.062, length=0.30),
        origin=Origin(xyz=(0.0, 0.22, 1.29)),
        material=polished_steel,
        name="quill_housing",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.10),
        origin=Origin(xyz=(0.0, 0.22, 1.09)),
        material=polished_steel,
        name="chuck",
    )

    table_support = model.part("table_support")
    collar_half_span_x = 0.060
    collar_half_span_y = 0.060
    collar_thickness = 0.032
    collar_height = 0.115
    table_support.visual(
        Box((collar_thickness, 0.140, collar_height)),
        origin=Origin(xyz=(collar_half_span_x, 0.0, 0.0)),
        material=cast_iron,
        name="collar_right",
    )
    table_support.visual(
        Box((collar_thickness, 0.140, collar_height)),
        origin=Origin(xyz=(-collar_half_span_x, 0.0, 0.0)),
        material=cast_iron,
        name="collar_left",
    )
    table_support.visual(
        Box((0.096, collar_thickness, collar_height)),
        origin=Origin(xyz=(0.0, collar_half_span_y, 0.0)),
        material=cast_iron,
        name="collar_front",
    )
    table_support.visual(
        Box((0.096, collar_thickness, collar_height)),
        origin=Origin(xyz=(0.0, -collar_half_span_y, 0.0)),
        material=cast_iron,
        name="collar_rear",
    )
    table_support.visual(
        Box((0.11, 0.136, 0.07)),
        origin=Origin(xyz=(0.0, 0.125, -0.004)),
        material=machine_gray,
        name="support_arm",
    )
    table_support.visual(
        Box((0.12, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.11, -0.045)),
        material=machine_gray,
        name="gusset",
    )
    table_support.visual(
        Box((0.028, 0.065, 0.095)),
        origin=Origin(xyz=(0.045, 0.220, -0.045)),
        material=cast_iron,
        name="yoke_right",
    )
    table_support.visual(
        Box((0.028, 0.065, 0.095)),
        origin=Origin(xyz=(-0.045, 0.220, -0.045)),
        material=cast_iron,
        name="yoke_left",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.016, length=0.062),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="hinge_barrel",
    )
    table.visual(
        Box((0.05, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=machine_gray,
        name="trunnion_block",
    )
    table.visual(
        Box((0.09, 0.12, 0.045)),
        origin=Origin(xyz=(0.0, 0.105, 0.017)),
        material=machine_gray,
        name="support_web",
    )
    table.visual(
        Cylinder(radius=0.18, length=0.034),
        origin=Origin(xyz=(0.0, 0.155, 0.034)),
        material=cast_iron,
        name="table_disk",
    )
    table.visual(
        Box((0.038, 0.12, 0.010)),
        origin=Origin(xyz=(0.0, 0.150, 0.056)),
        material=machine_gray,
        name="center_slot_rib",
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.018, length=0.05),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="hub",
    )
    feed_handle.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="shaft",
    )
    spoke_length = 0.17
    spoke_radius = 0.008
    knob_radius = 0.018
    spoke_x = 0.128
    for spoke_index in range(3):
        angle = 2.0 * pi * spoke_index / 3.0
        spoke_center = (
            spoke_x,
            0.5 * spoke_length * sin(angle),
            0.5 * spoke_length * cos(angle),
        )
        spoke_tip = (
            spoke_x,
            spoke_length * sin(angle),
            spoke_length * cos(angle),
        )
        feed_handle.visual(
            Cylinder(radius=spoke_radius, length=spoke_length),
            origin=Origin(xyz=spoke_center, rpy=(-angle, 0.0, 0.0)),
            material=polished_steel,
            name=f"spoke_{spoke_index}",
        )
        feed_handle.visual(
            Sphere(radius=knob_radius),
            origin=Origin(xyz=spoke_tip),
            material=handle_black,
            name=f"knob_{spoke_index}",
        )

    model.articulation(
        "table_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_support,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.20, lower=0.0, upper=0.24),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_support,
        child=table,
        origin=Origin(xyz=(0.0, 0.220, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "feed_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=feed_handle,
        origin=Origin(xyz=(0.130, 0.045, 1.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    table_support = object_model.get_part("table_support")
    table = object_model.get_part("table")
    feed_handle = object_model.get_part("feed_handle")

    table_slide = object_model.get_articulation("table_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    feed_spin = object_model.get_articulation("feed_spin")

    ctx.expect_origin_distance(
        table_support,
        frame,
        axes="xy",
        max_dist=0.001,
        name="table support stays centered on the column",
    )
    ctx.expect_gap(
        table,
        frame,
        axis="z",
        positive_elem="table_disk",
        negative_elem="base",
        min_gap=0.55,
        name="work table sits well above the floor base",
    )
    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="chuck",
        negative_elem="table_disk",
        min_gap=0.22,
        name="chuck clears the table at the rest height",
    )

    rest_support_pos = ctx.part_world_position(table_support)
    with ctx.pose({table_slide: 0.18}):
        raised_support_pos = ctx.part_world_position(table_support)
        ctx.expect_origin_distance(
            table_support,
            frame,
            axes="xy",
            max_dist=0.001,
            name="raised table support remains concentric with the column",
        )
        ctx.expect_gap(
            frame,
            table,
            axis="z",
            positive_elem="chuck",
            negative_elem="table_disk",
            min_gap=0.04,
            name="raised table still clears the chuck",
        )

    ctx.check(
        "table support moves upward along the column",
        rest_support_pos is not None
        and raised_support_pos is not None
        and raised_support_pos[2] > rest_support_pos[2] + 0.15,
        details=f"rest={rest_support_pos}, raised={raised_support_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_disk")
    with ctx.pose({table_tilt: 0.60}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_disk")

    rest_table_span = None if rest_table_aabb is None else rest_table_aabb[1][2] - rest_table_aabb[0][2]
    tilted_table_span = (
        None if tilted_table_aabb is None else tilted_table_aabb[1][2] - tilted_table_aabb[0][2]
    )
    ctx.check(
        "table tilt articulation changes the table pitch",
        rest_table_span is not None
        and tilted_table_span is not None
        and tilted_table_span > rest_table_span + 0.12,
        details=f"rest_span={rest_table_span}, tilted_span={tilted_table_span}",
    )

    rest_knob_aabb = ctx.part_element_world_aabb(feed_handle, elem="knob_0")
    with ctx.pose({feed_spin: pi / 2.0}):
        turned_knob_aabb = ctx.part_element_world_aabb(feed_handle, elem="knob_0")

    rest_knob_center = _center_from_aabb(rest_knob_aabb)
    turned_knob_center = _center_from_aabb(turned_knob_aabb)
    ctx.check(
        "feed handle rotates about the quill feed axis",
        rest_knob_center is not None
        and turned_knob_center is not None
        and abs(turned_knob_center[0] - rest_knob_center[0]) < 0.01
        and abs(turned_knob_center[1] - rest_knob_center[1]) > 0.06
        and abs(turned_knob_center[2] - rest_knob_center[2]) > 0.06,
        details=f"rest={rest_knob_center}, turned={turned_knob_center}",
    )

    return ctx.report()


object_model = build_object_model()
