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


def _head_carriage_mesh():
    """Casting for the sliding drill head with a clear rectangular arm tunnel."""
    sleeve = cq.Workplane("XY").box(0.30, 0.24, 0.24)
    arm_clearance = cq.Workplane("XY").box(0.36, 0.16, 0.17)
    lower_casting = cq.Workplane("XY").box(0.22, 0.20, 0.30).translate((0.04, 0.0, -0.25))
    nose = cq.Workplane("XY").box(0.15, 0.16, 0.11).translate((0.055, 0.0, -0.335))
    return sleeve.cut(arm_clearance).union(lower_casting).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press")

    dark_cast = model.material("dark_cast_iron", rgba=(0.10, 0.12, 0.13, 1.0))
    blue_cast = model.material("blue_gray_casting", rgba=(0.22, 0.30, 0.36, 1.0))
    bare_steel = model.material("bare_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    polished = model.material("polished_spindle", rgba=(0.78, 0.78, 0.74, 1.0))
    black = model.material("blackened_steel", rgba=(0.02, 0.02, 0.02, 1.0))

    base_column = model.part("base_column")
    base_column.visual(
        Cylinder(radius=0.38, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_cast,
        name="round_base",
    )
    base_column.visual(
        Cylinder(radius=0.085, length=1.42),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=blue_cast,
        name="round_column",
    )
    base_column.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=blue_cast,
        name="top_collar",
    )
    base_column.visual(
        Box((0.55, 0.10, 0.06)),
        origin=Origin(xyz=(0.28, 0.0, 0.705)),
        material=blue_cast,
        name="table_support",
    )
    base_column.visual(
        Cylinder(radius=0.26, length=0.05),
        origin=Origin(xyz=(0.52, 0.0, 0.72)),
        material=dark_cast,
        name="round_table",
    )
    for slot_index, y_offset in enumerate((-0.09, 0.0, 0.09)):
        base_column.visual(
            Box((0.42, 0.018, 0.008)),
            origin=Origin(xyz=(0.52, y_offset, 0.748)),
            material=black,
            name=f"table_slot_{slot_index}",
        )
    base_column.visual(
        Box((0.06, 0.34, 0.20)),
        origin=Origin(xyz=(-0.10, 0.0, 1.62)),
        material=blue_cast,
        name="yoke_back",
    )
    for lug_index, y_offset in enumerate((-0.125, 0.125)):
        base_column.visual(
            Box((0.16, 0.05, 0.18)),
            origin=Origin(xyz=(0.0, y_offset, 1.62)),
            material=blue_cast,
            name=f"yoke_lug_{lug_index}",
        )
    base_column.visual(
        Cylinder(radius=0.024, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 1.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="pivot_pin",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.065, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="pivot_hub",
    )
    arm.visual(
        Box((1.10, 0.09, 0.09)),
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        material=blue_cast,
        name="main_beam",
    )
    arm.visual(
        Box((1.02, 0.04, 0.035)),
        origin=Origin(xyz=(0.62, 0.0, 0.060)),
        material=bare_steel,
        name="top_slide_rail",
    )
    arm.visual(
        Box((1.02, 0.04, 0.035)),
        origin=Origin(xyz=(0.62, 0.0, -0.060)),
        material=bare_steel,
        name="lower_slide_rail",
    )
    arm.visual(
        Box((0.055, 0.16, 0.16)),
        origin=Origin(xyz=(1.175, 0.0, 0.0)),
        material=dark_cast,
        name="end_stop",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_carriage_mesh(), "sliding_head_casting", tolerance=0.0015),
        material=blue_cast,
        name="slide_sleeve",
    )
    head.visual(
        Box((0.24, 0.035, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=polished,
        name="upper_wear_pad",
    )
    head.visual(
        Box((0.24, 0.035, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
        material=polished,
        name="lower_wear_pad",
    )
    head.visual(
        Cylinder(radius=0.075, length=0.20),
        origin=Origin(xyz=(-0.06, -0.16, -0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="side_motor",
    )
    feed_lever = model.part("feed_lever")
    feed_lever.visual(
        Cylinder(radius=0.024, length=0.085),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="feed_hub",
    )
    feed_lever.visual(
        Cylinder(radius=0.012, length=0.22),
        origin=Origin(xyz=(-0.080, 0.0, -0.120)),
        material=black,
        name="feed_spoke",
    )
    feed_lever.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(xyz=(-0.080, 0.0, -0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="feed_grip",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.028, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=polished,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=bare_steel,
        name="chuck_collar",
    )
    spindle.visual(
        Cylinder(radius=0.058, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=dark_cast,
        name="drill_chuck",
    )
    spindle.visual(
        Box((0.055, 0.012, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, -0.190)),
        material=bare_steel,
        name="chuck_key",
    )
    spindle.visual(
        Cylinder(radius=0.011, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.325)),
        material=black,
        name="drill_bit",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=base_column,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.5, lower=0.0, upper=0.45),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.55),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.055, 0.0, -0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=20.0),
    )
    model.articulation(
        "head_to_feed_lever",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_lever,
        origin=Origin(xyz=(-0.070, 0.0, -0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.9, upper=0.9),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_column = object_model.get_part("base_column")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    feed_lever = object_model.get_part("feed_lever")
    spindle = object_model.get_part("spindle")
    arm_swing = object_model.get_articulation("column_to_arm")
    head_slide = object_model.get_articulation("arm_to_head")
    spindle_spin = object_model.get_articulation("head_to_spindle")
    feed_joint = object_model.get_articulation("head_to_feed_lever")

    ctx.allow_overlap(
        base_column,
        arm,
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        reason="The steel pivot pin is intentionally captured inside the radial arm hinge hub.",
    )
    ctx.allow_overlap(
        head,
        spindle,
        elem_a="slide_sleeve",
        elem_b="spindle_shaft",
        reason="The spindle shaft is intentionally seated into the head's lower bearing bore.",
    )
    ctx.allow_overlap(
        head,
        feed_lever,
        elem_a="slide_sleeve",
        elem_b="feed_hub",
        reason="The feed lever hub is intentionally seated into the head casting as a control shaft.",
    )
    ctx.expect_within(
        base_column,
        arm,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_hub",
        margin=0.001,
        name="pivot pin is centered in hinge hub",
    )
    ctx.expect_overlap(
        base_column,
        arm,
        axes="y",
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        min_overlap=0.13,
        name="hinge pin passes through the hub",
    )
    ctx.expect_within(
        spindle,
        head,
        axes="xy",
        inner_elem="spindle_shaft",
        outer_elem="slide_sleeve",
        margin=0.0,
        name="spindle shaft is centered in the head bearing",
    )
    ctx.expect_overlap(
        spindle,
        head,
        axes="z",
        elem_a="spindle_shaft",
        elem_b="slide_sleeve",
        min_overlap=0.01,
        name="spindle shaft has retained insertion in the head",
    )
    ctx.expect_overlap(
        feed_lever,
        head,
        axes="x",
        elem_a="feed_hub",
        elem_b="slide_sleeve",
        min_overlap=0.025,
        name="feed lever hub is inserted into head casting",
    )

    ctx.expect_within(
        arm,
        head,
        axes="yz",
        inner_elem="main_beam",
        outer_elem="slide_sleeve",
        margin=0.0,
        name="head sleeve surrounds the radial arm",
    )
    ctx.expect_overlap(
        head,
        arm,
        axes="x",
        elem_a="slide_sleeve",
        elem_b="main_beam",
        min_overlap=0.22,
        name="head carriage remains on the arm at rest",
    )
    head_rest = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.55}):
        ctx.expect_overlap(
            head,
            arm,
            axes="x",
            elem_a="slide_sleeve",
            elem_b="main_beam",
            min_overlap=0.22,
            name="extended head carriage remains on the arm",
        )
        head_extended = ctx.part_world_position(head)
    ctx.check(
        "head slide moves outward along arm",
        head_rest is not None
        and head_extended is not None
        and head_extended[0] > head_rest[0] + 0.50,
        details=f"rest={head_rest}, extended={head_extended}",
    )

    rest_end_stop = ctx.part_element_world_aabb(arm, elem="end_stop")
    with ctx.pose({arm_swing: 0.45}):
        raised_end_stop = ctx.part_element_world_aabb(arm, elem="end_stop")
    ctx.check(
        "radial arm swings about horizontal axle",
        rest_end_stop is not None
        and raised_end_stop is not None
        and raised_end_stop[1][2] > rest_end_stop[1][2] + 0.35,
        details=f"rest={rest_end_stop}, raised={raised_end_stop}",
    )

    key_rest = ctx.part_element_world_aabb(spindle, elem="chuck_key")
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        key_rotated = ctx.part_element_world_aabb(spindle, elem="chuck_key")
    if key_rest is not None and key_rotated is not None:
        key_rest_center = (
            (key_rest[0][0] + key_rest[1][0]) / 2.0,
            (key_rest[0][1] + key_rest[1][1]) / 2.0,
        )
        key_rotated_center = (
            (key_rotated[0][0] + key_rotated[1][0]) / 2.0,
            (key_rotated[0][1] + key_rotated[1][1]) / 2.0,
        )
    else:
        key_rest_center = None
        key_rotated_center = None
    ctx.check(
        "spindle rotates visible chuck key",
        key_rest_center is not None
        and key_rotated_center is not None
        and key_rotated_center[1] > key_rest_center[1] + 0.04,
        details=f"rest={key_rest_center}, rotated={key_rotated_center}",
    )

    grip_rest = ctx.part_element_world_aabb(feed_lever, elem="feed_grip")
    with ctx.pose({feed_joint: 0.8}):
        grip_moved = ctx.part_element_world_aabb(feed_lever, elem="feed_grip")
    ctx.check(
        "feed lever rotates on head",
        grip_rest is not None
        and grip_moved is not None
        and grip_moved[0][1] > grip_rest[0][1] + 0.10,
        details=f"rest={grip_rest}, moved={grip_moved}",
    )

    return ctx.report()


object_model = build_object_model()
