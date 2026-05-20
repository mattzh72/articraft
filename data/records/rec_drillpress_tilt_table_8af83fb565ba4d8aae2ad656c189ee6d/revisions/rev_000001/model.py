from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mill_drill_combo_press")

    cast = model.material("dark_machine_green", rgba=(0.12, 0.24, 0.19, 1.0))
    darker_cast = model.material("dark_cast_iron", rgba=(0.08, 0.09, 0.085, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_oxide", rgba=(0.01, 0.012, 0.012, 1.0))
    slot_black = model.material("shadowed_t_slots", rgba=(0.0, 0.0, 0.0, 1.0))
    red = model.material("red_pointer", rgba=(0.75, 0.05, 0.03, 1.0))

    # Fixed base assembly: heavy floor casting, round column, table ways, and
    # the trunnion yoke that carries the tilting head.
    base = model.part("base")
    base.visual(
        Box((0.85, 0.70, 0.12)),
        origin=Origin(xyz=(0.0, -0.10, 0.06)),
        material=darker_cast,
        name="base_casting",
    )
    base.visual(
        Box((0.58, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, -0.20, 0.20)),
        material=cast,
        name="front_knee",
    )
    for y, name in [(-0.285, "front_way"), (-0.115, "rear_way")]:
        base.visual(
            Box((0.56, 0.055, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.2975)),
            material=steel,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.075, length=1.25),
        origin=Origin(xyz=(0.0, 0.19, 0.725)),
        material=cast,
        name="round_column",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.035),
        origin=Origin(xyz=(0.0, 0.19, 0.1375)),
        material=darker_cast,
        name="column_flange",
    )
    base.visual(
        Box((0.38, 0.12, 0.20)),
        origin=Origin(xyz=(0.0, 0.13, 1.02)),
        material=cast,
        name="head_clamp_block",
    )
    for x, name in [(-0.17, "yoke_lug_0"), (0.17, "yoke_lug_1")]:
        base.visual(
            Box((0.06, 0.16, 0.22)),
            origin=Origin(xyz=(x, 0.04, 1.02)),
            material=cast,
            name=name,
        )

    # Y saddle: the lower cross-slide carriage riding on the fixed base ways.
    saddle = model.part("saddle")
    saddle.visual(
        Box((0.54, 0.26, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast,
        name="saddle_body",
    )
    for x, name in [(-0.20, "x_way_0"), (0.20, "x_way_1")]:
        saddle.visual(
            Box((0.055, 0.24, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0825)),
            material=steel,
            name=name,
        )
    saddle.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=(0.0, -0.15, 0.035), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=black,
        name="y_leadscrew",
    )

    model.articulation(
        "base_to_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(0.0, -0.20, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.18, lower=-0.07, upper=0.09),
    )

    # X table: a long T-slotted work table sliding on top of the saddle.
    table = model.part("table")
    table.visual(
        Box((0.80, 0.22, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=cast,
        name="table_slab",
    )
    for y, name in [(-0.070, "t_slot_0"), (0.0, "t_slot_1"), (0.070, "t_slot_2")]:
        table.visual(
            Box((0.74, 0.018, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.075)),
            material=slot_black,
            name=name,
        )
    table.visual(
        Cylinder(radius=0.011, length=0.84),
        origin=Origin(xyz=(0.0, -0.135, 0.027), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="x_leadscrew",
    )
    for x, name in [(-0.30, "screw_boss_0"), (0.30, "screw_boss_1")]:
        table.visual(
            Box((0.050, 0.030, 0.075)),
            origin=Origin(xyz=(x, -0.122, 0.0375)),
            material=cast,
            name=name,
        )

    model.articulation(
        "saddle_to_table",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.18, lower=-0.12, upper=0.12),
    )

    # Head with a real trunnion pin between the yoke lugs and a hollow quill
    # sleeve, so the quill can pass through without a fake solid overlap.
    head = model.part("head")
    head.visual(
        Cylinder(radius=0.040, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="trunnion_pin",
    )
    head.visual(
        Box((0.25, 0.30, 0.17)),
        origin=Origin(xyz=(0.0, -0.11, 0.0)),
        material=cast,
        name="head_casting",
    )
    head.visual(
        Box((0.22, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, -0.12, 0.175)),
        material=cast,
        name="motor_housing",
    )
    for x, name in [(-0.075, "sleeve_side_0"), (0.075, "sleeve_side_1")]:
        head.visual(
            Box((0.025, 0.160, 0.220)),
            origin=Origin(xyz=(x, -0.31, -0.05)),
            material=cast,
            name=name,
        )
    for y, name in [(-0.382, "sleeve_jaw_0"), (-0.238, "sleeve_jaw_1")]:
        head.visual(
            Box((0.125, 0.025, 0.220)),
            origin=Origin(xyz=(0.0, y, -0.05)),
            material=cast,
            name=name,
        )
    head.visual(
        Box((0.15, 0.045, 0.025)),
        origin=Origin(xyz=(0.0, -0.245, 0.085)),
        material=black,
        name="speed_plate",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.03, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=-0.35, upper=0.35),
    )

    # Dropping quill, chuck, and drill bit.
    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.048, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        material=steel,
        name="quill_tube",
    )
    quill.visual(
        Box((0.015, 0.018, 0.18)),
        origin=Origin(xyz=(0.0555, 0.0, -0.10)),
        material=steel,
        name="guide_key",
    )
    quill.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        material=black,
        name="drill_chuck",
    )
    quill.visual(
        Cylinder(radius=0.006, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.43)),
        material=steel,
        name="drill_bit",
    )

    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, -0.31, 0.06)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.16, lower=0.0, upper=0.12),
    )

    # Articulated handwheels and the quill-feed lever are distinct user controls.
    x_handwheel = model.part("x_handwheel")
    x_handwheel.visual(
        mesh_from_geometry(TorusGeometry(0.055, 0.006, radial_segments=16, tubular_segments=40).rotate_y(math.pi / 2), "x_handwheel_ring_mesh"),
        material=black,
        name="wheel_ring",
    )
    x_handwheel.visual(Box((0.008, 0.108, 0.008)), material=black, name="spoke_y")
    x_handwheel.visual(Box((0.008, 0.008, 0.108)), material=black, name="spoke_z")
    x_handwheel.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="wheel_hub",
    )
    x_handwheel.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="crank_knob",
    )
    model.articulation(
        "table_to_x_handwheel",
        ArticulationType.REVOLUTE,
        parent=table,
        child=x_handwheel,
        origin=Origin(xyz=(0.435, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    y_handwheel = model.part("y_handwheel")
    y_handwheel.visual(
        mesh_from_geometry(TorusGeometry(0.050, 0.006, radial_segments=16, tubular_segments=40).rotate_x(-math.pi / 2), "y_handwheel_ring_mesh"),
        material=black,
        name="wheel_ring",
    )
    y_handwheel.visual(Box((0.042, 0.008, 0.008)), origin=Origin(xyz=(0.037, 0.0, 0.0)), material=black, name="spoke_x_0")
    y_handwheel.visual(Box((0.042, 0.008, 0.008)), origin=Origin(xyz=(-0.037, 0.0, 0.0)), material=black, name="spoke_x_1")
    y_handwheel.visual(Box((0.008, 0.008, 0.042)), origin=Origin(xyz=(0.0, 0.0, 0.037)), material=black, name="spoke_z_0")
    y_handwheel.visual(Box((0.008, 0.008, 0.042)), origin=Origin(xyz=(0.0, 0.0, -0.037)), material=black, name="spoke_z_1")
    y_handwheel.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )
    y_handwheel.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="crank_knob",
    )
    model.articulation(
        "saddle_to_y_handwheel",
        ArticulationType.REVOLUTE,
        parent=saddle,
        child=y_handwheel,
        origin=Origin(xyz=(0.0, -0.170, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    feed_lever = model.part("feed_lever")
    feed_lever.visual(
        Cylinder(radius=0.025, length=0.040),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="feed_hub",
    )
    feed_lever.visual(
        Box((0.018, 0.018, 0.255)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=steel,
        name="feed_arm",
    )
    feed_lever.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.280)),
        material=red,
        name="feed_knob",
    )
    model.articulation(
        "head_to_feed_lever",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_lever,
        origin=Origin(xyz=(0.150, -0.20, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=-0.9, upper=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    saddle = object_model.get_part("saddle")
    table = object_model.get_part("table")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")

    y_slide = object_model.get_articulation("base_to_saddle")
    x_slide = object_model.get_articulation("saddle_to_table")
    tilt = object_model.get_articulation("base_to_head")
    quill_drop = object_model.get_articulation("head_to_quill")

    ctx.expect_gap(
        saddle,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="saddle_body",
        negative_elem="front_way",
        name="saddle rides on base ways",
    )
    ctx.expect_gap(
        table,
        saddle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="table_slab",
        negative_elem="x_way_0",
        name="table rides on saddle ways",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="trunnion_pin",
        elem_b="yoke_lug_0",
        contact_tol=0.001,
        name="head trunnion seated in yoke",
    )
    ctx.allow_overlap(
        saddle,
        object_model.get_part("y_handwheel"),
        elem_a="y_leadscrew",
        elem_b="wheel_hub",
        reason="The Y-feed handwheel hub is intentionally seated around the leadscrew shaft.",
    )
    ctx.expect_overlap(
        saddle,
        object_model.get_part("y_handwheel"),
        axes="y",
        min_overlap=0.025,
        elem_a="y_leadscrew",
        elem_b="wheel_hub",
        name="Y handwheel hub remains captured on the leadscrew",
    )

    rest_saddle = ctx.part_world_position(saddle)
    rest_table = ctx.part_world_position(table)
    rest_quill = ctx.part_world_position(quill)
    with ctx.pose({y_slide: 0.08, x_slide: 0.10, quill_drop: 0.10}):
        moved_saddle = ctx.part_world_position(saddle)
        moved_table = ctx.part_world_position(table)
        dropped_quill = ctx.part_world_position(quill)
    ctx.check(
        "Y slide moves the saddle toward the operator",
        rest_saddle is not None and moved_saddle is not None and moved_saddle[1] < rest_saddle[1] - 0.05,
        details=f"rest={rest_saddle}, moved={moved_saddle}",
    )
    ctx.check(
        "X slide moves the table laterally",
        rest_table is not None and moved_table is not None and moved_table[0] > rest_table[0] + 0.08,
        details=f"rest={rest_table}, moved={moved_table}",
    )
    ctx.check(
        "quill drops below the head",
        rest_quill is not None and dropped_quill is not None and dropped_quill[2] < rest_quill[2] - 0.08,
        details=f"rest={rest_quill}, dropped={dropped_quill}",
    )

    rest_quill_at_head = ctx.part_world_position(quill)
    with ctx.pose({tilt: 0.30}):
        tilted_quill = ctx.part_world_position(quill)
    ctx.check(
        "head tilt rotates the spindle axis about the column yoke",
        rest_quill_at_head is not None
        and tilted_quill is not None
        and abs(tilted_quill[1] - rest_quill_at_head[1]) > 0.002
        and abs(tilted_quill[2] - rest_quill_at_head[2]) > 0.05,
        details=f"rest={rest_quill_at_head}, tilted={tilted_quill}",
    )

    return ctx.report()


object_model = build_object_model()
