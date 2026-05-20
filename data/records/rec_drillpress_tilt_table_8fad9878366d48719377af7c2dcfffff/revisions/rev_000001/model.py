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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drill_press")

    cast_iron = Material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    column_steel = Material("brushed_column_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    machine_green = Material("machine_green", rgba=(0.18, 0.32, 0.25, 1.0))
    black = Material("black_knurled_plastic", rgba=(0.01, 0.01, 0.01, 1.0))
    bright_steel = Material("bright_steel", rgba=(0.80, 0.82, 0.82, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))

    frame = model.part("frame")

    # Heavy floor base and round upright column.
    frame.visual(
        Box((0.62, 0.44, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="floor_base",
    )
    frame.visual(
        Box((0.50, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.13, 0.064)),
        material=dark_steel,
        name="front_floor_slot",
    )
    frame.visual(
        Box((0.50, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.13, 0.064)),
        material=dark_steel,
        name="rear_floor_slot",
    )
    frame.visual(
        Cylinder(radius=0.035, length=1.360),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=column_steel,
        name="round_column",
    )

    # Upper head casting, front quill boss, and motor/pulley mass.
    frame.visual(
        Box((0.42, 0.38, 0.200)),
        origin=Origin(xyz=(0.0, -0.20, 1.430)),
        material=machine_green,
        name="head_casting",
    )
    frame.visual(
        Box((0.34, 0.18, 0.070)),
        origin=Origin(xyz=(0.0, -0.12, 1.555)),
        material=machine_green,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.230),
        origin=Origin(xyz=(0.0, 0.040, 1.440), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_green,
        name="rear_motor",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.100),
        origin=Origin(xyz=(0.0, -0.300, 1.330)),
        material=machine_green,
        name="quill_boss",
    )
    frame.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.0, -0.300, 1.284)),
        material=dark_steel,
        name="lower_bearing_face",
    )

    # Column clamp and clevis bracket for the tilting table.
    frame.visual(
        Cylinder(radius=0.066, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=cast_iron,
        name="column_clamp",
    )
    for x in (-0.075, 0.075):
        frame.visual(
            Box((0.025, 0.075, 0.072)),
            origin=Origin(xyz=(x, -0.0475, 0.820)),
            material=cast_iron,
            name=f"table_bracket_arm_{0 if x < 0 else 1}",
        )
    frame.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(-0.083, -0.095, 0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="table_hinge_lug_0",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.083, -0.095, 0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="table_hinge_lug_1",
    )

    # User-facing depth-stop detail on the side of the head.
    frame.visual(
        Box((0.010, 0.020, 0.155)),
        origin=Origin(xyz=(-0.216, -0.210, 1.380)),
        material=dark_steel,
        name="depth_stop_bar",
    )
    frame.visual(
        Box((0.022, 0.030, 0.018)),
        origin=Origin(xyz=(-0.218, -0.210, 1.335)),
        material=black,
        name="depth_stop_knob",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.016, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=bright_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_steel,
        name="quill_collar",
    )
    spindle.visual(
        Cylinder(radius=0.036, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=dark_steel,
        name="chuck_upper",
    )
    spindle.visual(
        Box((0.026, 0.012, 0.014)),
        origin=Origin(xyz=(0.043, 0.0, -0.145)),
        material=black,
        name="chuck_key_socket",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.196)),
        material=dark_steel,
        name="chuck_jaws",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=bright_steel,
        name="drill_bit",
    )
    spindle.visual(
        Cylinder(radius=0.0025, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.355)),
        material=bright_steel,
        name="drill_point",
    )

    handle = model.part("feed_handle")
    handle.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="handle_axle",
    )
    handle.visual(
        Cylinder(radius=0.034, length=0.038),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    spoke_len = 0.145
    for i, theta in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        y = math.sin(theta)
        z = math.cos(theta)
        handle.visual(
            Cylinder(radius=0.006, length=spoke_len),
            origin=Origin(
                xyz=(0.108, 0.5 * spoke_len * y, 0.5 * spoke_len * z),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=bright_steel,
            name=f"feed_spoke_{i}",
        )
        handle.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=(0.108, spoke_len * y, spoke_len * z)),
            material=black,
            name=f"feed_knob_{i}",
        )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.021, length=0.111),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tilt_barrel",
    )
    table.visual(
        Box((0.070, 0.205, 0.035)),
        origin=Origin(xyz=(0.0, -0.1025, 0.025)),
        material=cast_iron,
        name="table_support_arm",
    )
    table.visual(
        Cylinder(radius=0.220, length=0.045),
        origin=Origin(xyz=(0.0, -0.205, 0.058)),
        material=cast_iron,
        name="round_table_top",
    )
    table.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, -0.205, 0.0805)),
        material=black,
        name="center_hole_shadow",
    )
    table.visual(
        Box((0.280, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -0.205, 0.0805)),
        material=black,
        name="cross_slot",
    )
    table.visual(
        Box((0.026, 0.280, 0.004)),
        origin=Origin(xyz=(0.0, -0.205, 0.0805)),
        material=black,
        name="long_slot",
    )

    model.articulation(
        "spindle_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, -0.300, 1.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=80.0),
    )
    model.articulation(
        "quill_feed",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.210, -0.205, 1.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(0.0, -0.095, 0.820)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    handle = object_model.get_part("feed_handle")
    table = object_model.get_part("table")
    spindle_axle = object_model.get_articulation("spindle_axle")
    quill_feed = object_model.get_articulation("quill_feed")
    table_tilt = object_model.get_articulation("table_tilt")

    ctx.check(
        "requested major parts are present",
        all(
            part is not None
            for part in (
                frame,
                spindle,
                handle,
                table,
            )
        ),
    )
    ctx.check(
        "spindle is a continuous rotary axle",
        spindle_axle.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spindle_axle.axis) == (0.0, 0.0, 1.0),
        details=f"type={spindle_axle.articulation_type}, axis={spindle_axle.axis}",
    )
    ctx.check(
        "feed handle is revolute",
        quill_feed.articulation_type == ArticulationType.REVOLUTE
        and tuple(quill_feed.axis) == (1.0, 0.0, 0.0)
        and quill_feed.motion_limits is not None
        and quill_feed.motion_limits.lower < 0.0
        and quill_feed.motion_limits.upper > 0.0,
        details=f"type={quill_feed.articulation_type}, axis={quill_feed.axis}, limits={quill_feed.motion_limits}",
    )
    ctx.check(
        "round table tilts on the column bracket hinge",
        table_tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(table_tilt.axis) == (1.0, 0.0, 0.0)
        and table_tilt.motion_limits is not None
        and table_tilt.motion_limits.lower < -0.5
        and table_tilt.motion_limits.upper > 0.5,
        details=f"type={table_tilt.articulation_type}, axis={table_tilt.axis}, limits={table_tilt.motion_limits}",
    )

    ctx.expect_contact(
        table,
        frame,
        elem_a="tilt_barrel",
        elem_b="table_hinge_lug_0",
        contact_tol=0.002,
        name="table barrel is captured by the bracket lug",
    )
    ctx.expect_contact(
        handle,
        frame,
        elem_a="handle_axle",
        elem_b="head_casting",
        contact_tol=0.002,
        name="feed handle axle mounts on the head",
    )
    ctx.expect_overlap(
        spindle,
        table,
        axes="xy",
        elem_a="drill_bit",
        elem_b="round_table_top",
        min_overlap=0.010,
        name="drill bit is centered over the round table",
    )
    ctx.expect_gap(
        spindle,
        table,
        axis="z",
        positive_elem="drill_point",
        negative_elem="round_table_top",
        min_gap=0.005,
        max_gap=0.040,
        name="drill point clears the work table",
    )

    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: table_tilt.motion_limits.upper}):
        tilted_table_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "table tilt changes the table plane height",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and abs(float(tilted_table_aabb[0][2]) - float(rest_table_aabb[0][2])) > 0.020,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_spindle_aabb = ctx.part_world_aabb(spindle)
    with ctx.pose({spindle_axle: math.pi / 2.0}):
        spun_spindle_aabb = ctx.part_world_aabb(spindle)
    ctx.check(
        "chuck key socket visibly follows spindle rotation",
        rest_spindle_aabb is not None
        and spun_spindle_aabb is not None
        and abs(float(spun_spindle_aabb[1][1]) - float(rest_spindle_aabb[1][1])) > 0.020,
        details=f"rest={rest_spindle_aabb}, spun={spun_spindle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
