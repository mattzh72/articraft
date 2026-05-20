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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pickup_tailgate")

    paint = Material("deep_blue_paint", rgba=(0.06, 0.16, 0.28, 1.0))
    inner_paint = Material("scuffed_inner_blue", rgba=(0.09, 0.20, 0.32, 1.0))
    dark_trim = Material("black_textured_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    hinge_steel = Material("dark_zinc_hardware", rgba=(0.35, 0.36, 0.34, 1.0))
    bed_liner = Material("matte_bed_liner", rgba=(0.025, 0.028, 0.026, 1.0))

    cyl_y = (math.pi / 2.0, 0.0, 0.0)
    cyl_x = (0.0, math.pi / 2.0, 0.0)

    # Root: a short rear section of pickup bed, with the tailgate hinge line as
    # the object frame origin.  +X points into the bed, Y is width, and +Z is up.
    bed = model.part("bed")
    bed.visual(
        Box((0.86, 1.56, 0.055)),
        origin=Origin(xyz=(0.45, 0.0, 0.020)),
        material=bed_liner,
        name="bed_floor",
    )
    bed.visual(
        Box((0.075, 1.58, 0.090)),
        origin=Origin(xyz=(0.055, 0.0, 0.000)),
        material=hinge_steel,
        name="lower_sill",
    )
    bed.visual(
        Box((0.82, 0.070, 0.430)),
        origin=Origin(xyz=(0.46, -0.785, 0.225)),
        material=paint,
        name="side_wall_0",
    )
    bed.visual(
        Box((0.82, 0.070, 0.430)),
        origin=Origin(xyz=(0.46, 0.785, 0.225)),
        material=paint,
        name="side_wall_1",
    )
    bed.visual(
        Box((0.080, 0.060, 0.410)),
        origin=Origin(xyz=(0.035, -0.805, 0.235)),
        material=paint,
        name="rear_jamb_0",
    )
    bed.visual(
        Box((0.080, 0.060, 0.410)),
        origin=Origin(xyz=(0.035, 0.805, 0.235)),
        material=paint,
        name="rear_jamb_1",
    )
    for i, y in ((0, -0.675), (2, 0.675)):
        bed.visual(
            Cylinder(radius=0.026, length=0.220),
            origin=Origin(xyz=(-0.032, y, 0.000), rpy=cyl_y),
            material=hinge_steel,
            name=f"bed_hinge_{i}",
        )
        bed.visual(
            Box((0.050, 0.220, 0.030)),
            origin=Origin(xyz=(0.000, y, -0.028)),
            material=hinge_steel,
            name=f"bed_hinge_mount_{i}",
        )
    bed.visual(
        Cylinder(radius=0.026, length=0.440),
        origin=Origin(xyz=(-0.032, 0.0, 0.000), rpy=cyl_y),
        material=hinge_steel,
        name="bed_hinge_1",
    )
    bed.visual(
        Box((0.050, 0.440, 0.030)),
        origin=Origin(xyz=(0.000, 0.0, -0.028)),
        material=hinge_steel,
        name="bed_hinge_mount_1",
    )

    # The gate child frame sits on the lower hinge axis.  Its sheet metal rises
    # along local +Z from that axis; positive motion opens the gate outward and
    # down about the lower horizontal hinge.
    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.056, 1.500, 0.500)),
        origin=Origin(xyz=(-0.030, 0.0, 0.285)),
        material=paint,
        name="outer_skin",
    )
    tailgate.visual(
        Box((0.018, 1.360, 0.355)),
        origin=Origin(xyz=(0.003, 0.0, 0.295)),
        material=inner_paint,
        name="inner_panel",
    )
    tailgate.visual(
        Box((0.032, 1.520, 0.055)),
        origin=Origin(xyz=(-0.018, 0.0, 0.540)),
        material=paint,
        name="top_cap",
    )
    tailgate.visual(
        Box((0.058, 1.520, 0.065)),
        origin=Origin(xyz=(-0.030, 0.0, 0.076)),
        material=paint,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((0.024, 0.060, 0.430)),
        origin=Origin(xyz=(0.010, -0.690, 0.285)),
        material=inner_paint,
        name="side_rib_0",
    )
    tailgate.visual(
        Box((0.024, 0.060, 0.430)),
        origin=Origin(xyz=(0.010, 0.690, 0.285)),
        material=inner_paint,
        name="side_rib_1",
    )
    tailgate.visual(
        Box((0.024, 1.180, 0.045)),
        origin=Origin(xyz=(0.012, 0.0, 0.165)),
        material=inner_paint,
        name="lower_stamp_rib",
    )
    tailgate.visual(
        Box((0.024, 1.180, 0.045)),
        origin=Origin(xyz=(0.012, 0.0, 0.390)),
        material=inner_paint,
        name="upper_stamp_rib",
    )
    tailgate.visual(
        Cylinder(radius=0.026, length=0.347),
        origin=Origin(xyz=(-0.032, -0.3925, 0.000), rpy=cyl_y),
        material=hinge_steel,
        name="tailgate_hinge_0",
    )
    tailgate.visual(
        Box((0.052, 0.347, 0.036)),
        origin=Origin(xyz=(-0.032, -0.3925, 0.031)),
        material=hinge_steel,
        name="tailgate_hinge_mount_0",
    )
    tailgate.visual(
        Cylinder(radius=0.026, length=0.347),
        origin=Origin(xyz=(-0.032, 0.3925, 0.000), rpy=cyl_y),
        material=hinge_steel,
        name="tailgate_hinge_1",
    )
    tailgate.visual(
        Box((0.052, 0.347, 0.036)),
        origin=Origin(xyz=(-0.032, 0.3925, 0.031)),
        material=hinge_steel,
        name="tailgate_hinge_mount_1",
    )
    for i, y in enumerate((-0.515, 0.515)):
        tailgate.visual(
            Box((0.030, 0.080, 0.040)),
            origin=Origin(xyz=(0.018, y, 0.493)),
            material=hinge_steel,
            name=f"stop_hinge_mount_{i}",
        )
    tailgate.visual(
        Box((0.024, 1.050, 0.108)),
        origin=Origin(xyz=(0.020, 0.0, 0.453)),
        material=hinge_steel,
        name="stop_hinge_plate",
    )
    tailgate.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.014, -0.635, 0.475), rpy=cyl_x),
        material=hinge_steel,
        name="latch_boss_0",
    )
    tailgate.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.014, 0.635, 0.475), rpy=cyl_x),
        material=hinge_steel,
        name="latch_boss_1",
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.57),
    )

    # Two separate rotating latch knobs sit on short shafts through the top of
    # the inner panel.  The long oval grip makes their rotation readable.
    for i, y in enumerate((-0.635, 0.635)):
        knob = model.part(f"latch_knob_{i}")
        knob.visual(
            Cylinder(radius=0.011, length=0.044),
            origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=cyl_x),
            material=hinge_steel,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.046, length=0.018),
            origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=cyl_x),
            material=dark_trim,
            name="round_cap",
        )
        knob.visual(
            Box((0.014, 0.120, 0.024)),
            origin=Origin(xyz=(0.061, 0.0, 0.0)),
            material=dark_trim,
            name="grip_tab",
        )
        model.articulation(
            f"latch_knob_{i}_shaft",
            ArticulationType.REVOLUTE,
            parent=tailgate,
            child=knob,
            origin=Origin(xyz=(0.012, y, 0.475)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=0.0, upper=1.57),
        )

    # Fold-flat work stop: closed against the inner skin at rest, and able to
    # rotate out from the top edge so it becomes a raised ledge when the gate is
    # lying flat as a compact work surface.
    work_stop = model.part("work_stop")
    work_stop.visual(
        Cylinder(radius=0.014, length=1.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=hinge_steel,
        name="hinge_pin",
    )
    work_stop.visual(
        Box((0.032, 1.120, 0.050)),
        origin=Origin(xyz=(0.016, 0.0, -0.055)),
        material=dark_trim,
        name="stop_bar",
    )
    work_stop.visual(
        Box((0.025, 0.070, 0.070)),
        origin=Origin(xyz=(0.012, -0.545, -0.030)),
        material=dark_trim,
        name="end_foot_0",
    )
    work_stop.visual(
        Box((0.025, 0.070, 0.070)),
        origin=Origin(xyz=(0.012, 0.545, -0.030)),
        material=dark_trim,
        name="end_foot_1",
    )
    model.articulation(
        "work_stop_hinge",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=work_stop,
        origin=Origin(xyz=(0.034, 0.0, 0.525)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bed = object_model.get_part("bed")
    tailgate = object_model.get_part("tailgate")
    knob_0 = object_model.get_part("latch_knob_0")
    knob_1 = object_model.get_part("latch_knob_1")
    work_stop = object_model.get_part("work_stop")

    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    knob_0_shaft = object_model.get_articulation("latch_knob_0_shaft")
    knob_1_shaft = object_model.get_articulation("latch_knob_1_shaft")
    work_stop_hinge = object_model.get_articulation("work_stop_hinge")

    def center_axis(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return (float(aabb[0][axis_index]) + float(aabb[1][axis_index])) * 0.5

    def span_axis(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return float(aabb[1][axis_index]) - float(aabb[0][axis_index])

    # Prompt-specific mounting proof: the alternating lower hinge barrels are
    # captured with a very small insertion at the bed sill instead of floating.
    ctx.expect_gap(
        bed,
        tailgate,
        axis="y",
        positive_elem="bed_hinge_1",
        negative_elem="tailgate_hinge_0",
        max_gap=0.0015,
        max_penetration=0.004,
        name="lower hinge barrels are captured at the bed edge",
    )
    ctx.expect_gap(
        knob_0,
        tailgate,
        axis="x",
        positive_elem="shaft",
        negative_elem="latch_boss_0",
        max_gap=0.0025,
        max_penetration=0.004,
        name="first latch knob shaft seats in its boss",
    )
    ctx.expect_gap(
        knob_1,
        tailgate,
        axis="x",
        positive_elem="shaft",
        negative_elem="latch_boss_1",
        max_gap=0.0025,
        max_penetration=0.004,
        name="second latch knob shaft seats in its boss",
    )
    ctx.expect_origin_distance(
        knob_0,
        knob_1,
        axes="y",
        min_dist=1.20,
        max_dist=1.35,
        name="twin latch knobs are spaced across the top edge",
    )

    closed_top = ctx.part_element_world_aabb(tailgate, elem="top_cap")
    closed_top_x = center_axis(closed_top, 0)
    closed_top_z = center_axis(closed_top, 2)
    with ctx.pose({tailgate_hinge: 1.45}):
        lowered_top = ctx.part_element_world_aabb(tailgate, elem="top_cap")
        lowered_top_x = center_axis(lowered_top, 0)
        lowered_top_z = center_axis(lowered_top, 2)
    ctx.check(
        "tailgate opens outward and downward",
        None not in (closed_top_x, closed_top_z, lowered_top_x, lowered_top_z)
        and lowered_top_x < closed_top_x - 0.35
        and lowered_top_z < closed_top_z - 0.30,
        details=f"closed=({closed_top_x}, {closed_top_z}), opened=({lowered_top_x}, {lowered_top_z})",
    )

    closed_stop = ctx.part_element_world_aabb(work_stop, elem="stop_bar")
    closed_stop_x = center_axis(closed_stop, 0)
    with ctx.pose({work_stop_hinge: 1.20}):
        raised_stop = ctx.part_element_world_aabb(work_stop, elem="stop_bar")
        raised_stop_x = center_axis(raised_stop, 0)
    ctx.check(
        "work stop folds out from the inner top edge",
        None not in (closed_stop_x, raised_stop_x) and raised_stop_x > closed_stop_x + 0.035,
        details=f"folded_x={closed_stop_x}, raised_x={raised_stop_x}",
    )

    closed_tab = ctx.part_element_world_aabb(knob_0, elem="grip_tab")
    closed_tab_z = span_axis(closed_tab, 2)
    with ctx.pose({knob_0_shaft: 1.57, knob_1_shaft: 1.57}):
        rotated_tab = ctx.part_element_world_aabb(knob_0, elem="grip_tab")
        rotated_tab_z = span_axis(rotated_tab, 2)
    ctx.check(
        "latch knobs rotate on short shafts",
        None not in (closed_tab_z, rotated_tab_z) and rotated_tab_z > closed_tab_z + 0.050,
        details=f"closed_z_span={closed_tab_z}, rotated_z_span={rotated_tab_z}",
    )

    return ctx.report()


object_model = build_object_model()
