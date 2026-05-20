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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_drill_press")

    paint = model.material("deep_blue_machine_paint", rgba=(0.03, 0.10, 0.22, 1.0))
    black = model.material("black_anodized_steel", rgba=(0.01, 0.012, 0.014, 1.0))
    dark_steel = model.material("dark_magnetic_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.015, 0.015, 1.0))
    red = model.material("red_stop_button", rgba=(0.85, 0.03, 0.02, 1.0))
    green = model.material("green_start_button", rgba=(0.02, 0.55, 0.10, 1.0))
    label = model.material("silver_label_plate", rgba=(0.75, 0.75, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.22, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="magnet_block",
    )
    base.visual(
        Box((0.30, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=paint,
        name="base_top_cover",
    )
    base.visual(
        Box((0.14, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, -0.108, 0.086)),
        material=black,
        name="front_control_panel",
    )
    base.visual(
        Box((0.12, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.112, 0.048)),
        material=label,
        name="magnet_label_plate",
    )

    # A real magnetic drill has a square slide sleeve rising from the magnet.
    # It is built here from four walls so the sliding column has visible
    # clearance and does not pass through a solid proxy.
    sleeve_center_y = 0.060
    sleeve_outer = 0.090
    sleeve_wall = 0.012
    sleeve_height = 0.244
    sleeve_bottom = 0.106
    sleeve_z = sleeve_bottom + sleeve_height / 2.0
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(-(sleeve_outer - sleeve_wall) / 2.0, sleeve_center_y, sleeve_z)),
        material=paint,
        name="sleeve_side_0",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=((sleeve_outer - sleeve_wall) / 2.0, sleeve_center_y, sleeve_z)),
        material=paint,
        name="sleeve_side_1",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, sleeve_center_y - (sleeve_outer - sleeve_wall) / 2.0, sleeve_z)),
        material=paint,
        name="sleeve_front_wall",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, sleeve_center_y + (sleeve_outer - sleeve_wall) / 2.0, sleeve_z)),
        material=paint,
        name="sleeve_rear_wall",
    )

    column = model.part("column")
    column.visual(
        Box((0.066, 0.066, 0.700)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=brushed,
        name="sliding_column",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, sleeve_center_y, sleeve_bottom + sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.12, lower=0.0, upper=0.18),
    )

    head = model.part("head")
    head.visual(
        Box((0.130, 0.022, 0.205)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=paint,
        name="rear_mount_plate",
    )
    head.visual(
        Box((0.035, 0.185, 0.205)),
        origin=Origin(xyz=(-0.0525, -0.102, 0.0)),
        material=paint,
        name="head_cheek_0",
    )
    head.visual(
        Box((0.035, 0.185, 0.205)),
        origin=Origin(xyz=(0.0525, -0.102, 0.0)),
        material=paint,
        name="head_cheek_1",
    )
    head.visual(
        Box((0.140, 0.185, 0.044)),
        origin=Origin(xyz=(0.0, -0.102, 0.081)),
        material=paint,
        name="top_bridge",
    )
    head.visual(
        Cylinder(radius=0.070, length=0.185),
        origin=Origin(xyz=(0.0, -0.102, 0.190)),
        material=black,
        name="motor_barrel",
    )
    head.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.0, -0.102, 0.296)),
        material=black,
        name="motor_cap",
    )
    head.visual(
        Box((0.150, 0.010, 0.075)),
        origin=Origin(xyz=(0.0, -0.198, -0.030)),
        material=black,
        name="front_nameplate",
    )

    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, -0.033, 0.300)),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.035, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=brushed,
        name="quill_sleeve",
    )
    quill.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        material=dark_steel,
        name="drill_chuck",
    )
    quill.visual(
        Cylinder(radius=0.008, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        material=brushed,
        name="drill_bit",
    )

    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, -0.102, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.12),
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial_y = -math.sin(angle)
        radial_z = math.cos(angle)
        feed_handle.visual(
            Box((0.012, 0.012, 0.100)),
            origin=Origin(
                xyz=(0.060, 0.050 * radial_y, 0.050 * radial_z),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"handle_spoke_{index}",
        )
        feed_handle.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.060, 0.105 * radial_y, 0.105 * radial_z)),
            material=rubber,
            name=f"handle_knob_{index}",
        )

    model.articulation(
        "head_to_feed_handle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=feed_handle,
        origin=Origin(xyz=(0.070, -0.102, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )

    magnet_button = model.part("magnet_button")
    magnet_button.visual(
        Box((0.034, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=green,
        name="button_cap",
    )
    model.articulation(
        "base_to_magnet_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=magnet_button,
        origin=Origin(xyz=(-0.035, -0.114, 0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.03, lower=0.0, upper=0.006),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.034, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=red,
        name="button_cap",
    )
    model.articulation(
        "base_to_power_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=power_button,
        origin=Origin(xyz=(0.035, -0.114, 0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.03, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")
    magnet_button = object_model.get_part("magnet_button")
    power_button = object_model.get_part("power_button")

    column_slide = object_model.get_articulation("base_to_column")
    quill_slide = object_model.get_articulation("head_to_quill")

    ctx.expect_contact(
        column,
        base,
        elem_a="sliding_column",
        elem_b="sleeve_side_0",
        name="sliding column bears on the square sleeve",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        min_overlap=0.20,
        elem_a="sliding_column",
        elem_b="sleeve_rear_wall",
        name="column remains deeply inserted at rest",
    )

    column_rest = ctx.part_world_position(column)
    with ctx.pose({column_slide: 0.18}):
        ctx.expect_contact(
            column,
            base,
            elem_a="sliding_column",
            elem_b="sleeve_side_0",
            name="raised column still rides in sleeve",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            min_overlap=0.055,
            elem_a="sliding_column",
            elem_b="sleeve_rear_wall",
            name="raised column retains sleeve insertion",
        )
        column_raised = ctx.part_world_position(column)
    ctx.check(
        "column prismatic joint lifts the head",
        column_rest is not None
        and column_raised is not None
        and column_raised[2] > column_rest[2] + 0.15,
        details=f"rest={column_rest}, raised={column_raised}",
    )

    ctx.expect_contact(
        quill,
        head,
        elem_a="quill_sleeve",
        elem_b="head_cheek_0",
        name="quill is guided by the drill head cheek",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="z",
        min_overlap=0.12,
        elem_a="quill_sleeve",
        elem_b="head_cheek_0",
        name="quill is seated inside the head at rest",
    )

    quill_rest = ctx.part_world_position(quill)
    with ctx.pose({quill_slide: 0.12}):
        ctx.expect_contact(
            quill,
            head,
            elem_a="quill_sleeve",
            elem_b="head_cheek_0",
            name="lowered quill still rides in head guide",
        )
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            min_overlap=0.030,
            elem_a="quill_sleeve",
            elem_b="head_cheek_0",
            name="lowered quill retains insertion in head",
        )
        quill_lowered = ctx.part_world_position(quill)
    ctx.check(
        "quill prismatic joint drops the drill",
        quill_rest is not None
        and quill_lowered is not None
        and quill_lowered[2] < quill_rest[2] - 0.10,
        details=f"rest={quill_rest}, lowered={quill_lowered}",
    )

    ctx.expect_contact(
        magnet_button,
        base,
        elem_a="button_cap",
        elem_b="front_control_panel",
        name="magnet pushbutton is mounted on the panel",
    )
    ctx.expect_contact(
        power_button,
        base,
        elem_a="button_cap",
        elem_b="front_control_panel",
        name="power pushbutton is mounted on the panel",
    )

    return ctx.report()


object_model = build_object_model()
