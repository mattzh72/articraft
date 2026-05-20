from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_drill_stand")

    model.material("magnet_blue", color=(0.03, 0.08, 0.16, 1.0))
    model.material("dark_cast", color=(0.025, 0.028, 0.032, 1.0))
    model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    model.material("black_oxide", color=(0.005, 0.005, 0.006, 1.0))
    model.material("rack_black", color=(0.015, 0.015, 0.015, 1.0))
    model.material("warning_yellow", color=(1.0, 0.72, 0.05, 1.0))
    model.material("red_switch", color=(0.75, 0.02, 0.015, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.32, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="brushed_steel",
        name="magnet_sole",
    )
    base.visual(
        Box((0.44, 0.30, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material="magnet_blue",
        name="magnet_case",
    )
    base.visual(
        Box((0.22, 0.005, 0.035)),
        origin=Origin(xyz=(0.0, -0.1525, 0.080)),
        material="warning_yellow",
        name="front_label",
    )
    # Four wall pieces leave a real clear rectangular socket for the sliding
    # column instead of modeling the sleeve as a solid block.
    base.visual(
        Box((0.018, 0.095, 0.260)),
        origin=Origin(xyz=(-0.055, 0.080, 0.235)),
        material="dark_cast",
        name="sleeve_side_0",
    )
    base.visual(
        Box((0.018, 0.095, 0.260)),
        origin=Origin(xyz=(0.055, 0.080, 0.235)),
        material="dark_cast",
        name="sleeve_side_1",
    )
    base.visual(
        Box((0.092, 0.014, 0.260)),
        origin=Origin(xyz=(0.0, 0.032, 0.235)),
        material="dark_cast",
        name="sleeve_front",
    )
    base.visual(
        Box((0.092, 0.014, 0.260)),
        origin=Origin(xyz=(0.0, 0.128, 0.235)),
        material="dark_cast",
        name="sleeve_rear",
    )
    base.visual(
        Box((0.034, 0.150, 0.145)),
        origin=Origin(xyz=(-0.140, 0.070, 0.180)),
        material="dark_cast",
        name="left_gusset",
    )
    base.visual(
        Box((0.034, 0.150, 0.145)),
        origin=Origin(xyz=(0.140, 0.070, 0.180)),
        material="dark_cast",
        name="right_gusset",
    )

    magnet_switch = model.part("magnet_switch")
    magnet_switch.visual(
        Box((0.035, 0.006, 0.026)),
        material="red_switch",
        name="switch_rocker",
    )

    column = model.part("column")
    column.visual(
        Box((0.065, 0.055, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material="brushed_steel",
        name="column_tube",
    )
    column.visual(
        Box((0.095, 0.075, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.5375)),
        material="dark_cast",
        name="top_stop",
    )
    column.visual(
        Box((0.0135, 0.040, 0.240)),
        origin=Origin(xyz=(-0.03925, 0.0, -0.080)),
        material="brushed_steel",
        name="lower_slide_pad_0",
    )
    column.visual(
        Box((0.0135, 0.040, 0.240)),
        origin=Origin(xyz=(0.03925, 0.0, -0.080)),
        material="brushed_steel",
        name="lower_slide_pad_1",
    )
    column.visual(
        Box((0.082, 0.038, 0.035)),
        origin=Origin(xyz=(0.068, -0.0465, 0.090)),
        material="brushed_steel",
        name="rack_web",
    )
    column.visual(
        Box((0.018, 0.012, 0.600)),
        origin=Origin(xyz=(0.105, -0.066, 0.220)),
        material="rack_black",
        name="rack_strip",
    )
    for i in range(24):
        column.visual(
            Box((0.020, 0.010, 0.008)),
            origin=Origin(xyz=(0.105, -0.077, -0.050 + i * 0.024)),
            material="brushed_steel",
            name=f"rack_tooth_{i}",
        )
    column.visual(
        Box((0.006, 0.004, 0.500)),
        origin=Origin(xyz=(-0.0355, -0.0295, 0.175)),
        material="warning_yellow",
        name="height_scale",
    )

    spindle_head = model.part("spindle_head")
    spindle_head.visual(
        Box((0.170, 0.110, 0.220)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material="dark_cast",
        name="slide_carriage",
    )
    spindle_head.visual(
        Box((0.034, 0.007, 0.205)),
        origin=Origin(xyz=(-0.040, 0.0835, 0.0)),
        material="brushed_steel",
        name="guide_shoe_0",
    )
    spindle_head.visual(
        Box((0.034, 0.007, 0.205)),
        origin=Origin(xyz=(0.040, 0.0835, 0.0)),
        material="brushed_steel",
        name="guide_shoe_1",
    )
    spindle_head.visual(
        Box((0.125, 0.086, 0.125)),
        origin=Origin(xyz=(0.0, -0.070, 0.005)),
        material="dark_cast",
        name="motor_mount",
    )
    spindle_head.visual(
        Cylinder(radius=0.055, length=0.310),
        origin=Origin(xyz=(0.0, -0.145, 0.020)),
        material="magnet_blue",
        name="motor_case",
    )
    spindle_head.visual(
        Cylinder(radius=0.031, length=0.075),
        origin=Origin(xyz=(0.0, -0.145, 0.200)),
        material="dark_cast",
        name="motor_cap",
    )
    spindle_head.visual(
        Cylinder(radius=0.024, length=0.195),
        origin=Origin(xyz=(0.0, -0.145, -0.205)),
        material="black_oxide",
        name="quill",
    )
    chuck_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, -0.044),
                (0.014, -0.044),
                (0.028, 0.036),
                (0.020, 0.044),
                (0.0, 0.044),
            ],
            segments=40,
        ),
        "tapered_chuck",
    )
    spindle_head.visual(
        chuck_mesh,
        origin=Origin(xyz=(0.0, -0.145, -0.325)),
        material="brushed_steel",
        name="chuck",
    )
    spindle_head.visual(
        Cylinder(radius=0.0055, length=0.190),
        origin=Origin(xyz=(0.0, -0.145, -0.462)),
        material="black_oxide",
        name="drill_bit",
    )
    spindle_head.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.100, 0.010, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="shaft_bushing",
    )

    handwheel = model.part("handwheel")
    pinion_mesh = mesh_from_cadquery(
        SpurGear(module=0.0035, teeth_number=18, width=0.012).build(
            bore_d=0.010,
            hub_d=0.026,
            hub_length=0.016,
            n_spokes=3,
            spoke_width=0.004,
            spokes_id=0.014,
            spokes_od=0.050,
            chamfer=0.0005,
        ),
        "pinion_gear",
        tolerance=0.00045,
        angular_tolerance=0.08,
    )
    handwheel.visual(
        pinion_mesh,
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="pinion",
    )
    handwheel.visual(
        Cylinder(radius=0.009, length=0.074),
        origin=Origin(xyz=(0.037, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_oxide",
        name="feed_shaft",
    )
    handwheel.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="wheel_hub",
    )
    handwheel.visual(
        Box((0.009, 0.134, 0.012)),
        origin=Origin(xyz=(0.083, 0.0, 0.0)),
        material="brushed_steel",
        name="wheel_spoke_0",
    )
    handwheel.visual(
        Box((0.009, 0.012, 0.134)),
        origin=Origin(xyz=(0.083, 0.0, 0.0)),
        material="brushed_steel",
        name="wheel_spoke_1",
    )
    handwheel.visual(
        Box((0.009, 0.012, 0.134)),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(pi / 4.0, 0.0, 0.0)),
        material="brushed_steel",
        name="wheel_spoke_2",
    )
    handwheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.067, tube=0.006, radial_segments=20, tubular_segments=56), "handwheel_rim"),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=0.0055, length=0.044),
        origin=Origin(xyz=(0.111, 0.0, 0.067), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_oxide",
        name="crank_stem",
    )
    handwheel.visual(
        Cylinder(radius=0.012, length=0.045),
        origin=Origin(xyz=(0.148, 0.0, 0.067), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_cast",
        name="crank_grip",
    )

    model.articulation(
        "base_to_magnet_switch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magnet_switch,
        origin=Origin(xyz=(-0.155, -0.153, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.25, upper=0.25),
    )
    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.080, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.10, lower=0.0, upper=0.180),
    )
    model.articulation(
        "column_to_spindle_head",
        ArticulationType.PRISMATIC,
        parent=column,
        child=spindle_head,
        origin=Origin(xyz=(0.0, -0.115, 0.420)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    model.articulation(
        "spindle_head_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=spindle_head,
        child=handwheel,
        origin=Origin(xyz=(0.115, 0.010, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    spindle_head = object_model.get_part("spindle_head")
    handwheel = object_model.get_part("handwheel")
    magnet_switch = object_model.get_part("magnet_switch")

    switch_joint = object_model.get_articulation("base_to_magnet_switch")
    column_slide = object_model.get_articulation("base_to_column")
    feed_slide = object_model.get_articulation("column_to_spindle_head")
    wheel_spin = object_model.get_articulation("spindle_head_to_handwheel")

    ctx.check(
        "electromagnet switch is a small rocker",
        switch_joint.articulation_type == ArticulationType.REVOLUTE
        and switch_joint.axis == (1.0, 0.0, 0.0)
        and switch_joint.motion_limits is not None
        and switch_joint.motion_limits.lower == -0.25
        and switch_joint.motion_limits.upper == 0.25,
    )
    ctx.check(
        "column is a vertical prismatic slide",
        column_slide.articulation_type == ArticulationType.PRISMATIC
        and column_slide.axis == (0.0, 0.0, 1.0)
        and column_slide.motion_limits is not None
        and column_slide.motion_limits.upper == 0.180,
    )
    ctx.check(
        "spindle head feeds downward on a prismatic rack slide",
        feed_slide.articulation_type == ArticulationType.PRISMATIC
        and feed_slide.axis == (0.0, 0.0, -1.0)
        and feed_slide.motion_limits is not None
        and feed_slide.motion_limits.upper == 0.160,
    )
    ctx.check(
        "side handwheel is a free rotary control",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (1.0, 0.0, 0.0),
    )

    ctx.expect_gap(
        base,
        magnet_switch,
        axis="y",
        positive_elem="magnet_case",
        negative_elem="switch_rocker",
        max_gap=0.001,
        max_penetration=0.0,
        name="rocker switch is seated on the magnetic base face",
    )
    ctx.expect_gap(
        base,
        column,
        axis="x",
        positive_elem="sleeve_side_1",
        negative_elem="lower_slide_pad_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="telescoping column bears against sleeve guide pad",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="lower_slide_pad_1",
        elem_b="sleeve_side_1",
        min_overlap=0.20,
        name="collapsed column keeps long sleeve engagement",
    )

    column_rest = ctx.part_world_position(column)
    with ctx.pose({column_slide: column_slide.motion_limits.upper}):
        column_extended = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="lower_slide_pad_1",
            elem_b="sleeve_side_1",
            min_overlap=0.055,
            name="extended column retains guide pad insertion",
        )
    ctx.check(
        "column extends upward from magnetic base",
        column_rest is not None
        and column_extended is not None
        and column_extended[2] > column_rest[2] + 0.17,
        details=f"rest={column_rest}, extended={column_extended}",
    )

    ctx.expect_gap(
        column,
        spindle_head,
        axis="y",
        positive_elem="column_tube",
        negative_elem="guide_shoe_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="spindle carriage rides just in front of column",
    )
    ctx.expect_gap(
        handwheel,
        spindle_head,
        axis="x",
        positive_elem="feed_shaft",
        negative_elem="shaft_bushing",
        max_gap=0.001,
        max_penetration=0.000001,
        name="handwheel shaft seats against side bushing",
    )
    ctx.expect_gap(
        handwheel,
        column,
        axis="x",
        positive_elem="pinion",
        negative_elem="rack_strip",
        min_gap=0.004,
        max_gap=0.020,
        name="pinion is visibly set beside the rack",
    )
    ctx.expect_overlap(
        handwheel,
        column,
        axes="z",
        elem_a="pinion",
        elem_b="rack_strip",
        min_overlap=0.050,
        name="pinion overlaps the rack height",
    )

    head_rest = ctx.part_world_position(spindle_head)
    with ctx.pose({feed_slide: feed_slide.motion_limits.upper}):
        head_lowered = ctx.part_world_position(spindle_head)
        ctx.expect_overlap(
            spindle_head,
            column,
            axes="z",
            elem_a="guide_shoe_1",
            elem_b="rack_strip",
            min_overlap=0.16,
            name="lowered spindle head remains on the rack",
        )
        ctx.expect_gap(
            column,
            spindle_head,
            axis="y",
            positive_elem="column_tube",
            negative_elem="guide_shoe_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="lowered spindle carriage stays on guide face",
        )
    ctx.check(
        "rack feed moves spindle head downward",
        head_rest is not None
        and head_lowered is not None
        and head_lowered[2] < head_rest[2] - 0.15,
        details=f"rest={head_rest}, lowered={head_lowered}",
    )

    return ctx.report()


object_model = build_object_model()
