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
    model = ArticulatedObject(name="adjustable_weight_bench")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.09, 0.09, 0.10, 1.0))
    board_gray = model.material("board_gray", rgba=(0.21, 0.21, 0.22, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.18, 0.64, 0.08)),
        origin=Origin(xyz=(-0.56, 0.0, 0.04)),
        material=powder_black,
        name="rear_base",
    )
    frame.visual(
        Box((0.22, 0.12, 0.08)),
        origin=Origin(xyz=(-0.49, 0.0, 0.12)),
        material=powder_black,
        name="rear_spine",
    )
    frame.visual(
        Box((0.92, 0.10, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, 0.13)),
        material=powder_black,
        name="main_spine",
    )
    frame.visual(
        Box((0.24, 0.34, 0.08)),
        origin=Origin(xyz=(0.46, 0.0, 0.04)),
        material=powder_black,
        name="front_base",
    )
    frame.visual(
        Box((0.08, 0.08, 0.24)),
        origin=Origin(xyz=(0.40, 0.0, 0.20)),
        material=powder_black,
        name="front_post",
    )
    frame.visual(
        Box((0.10, 0.12, 0.30)),
        origin=Origin(xyz=(0.16, 0.0, 0.24)),
        material=powder_black,
        name="seat_tower",
    )
    frame.visual(
        Box((0.24, 0.10, 0.06)),
        origin=Origin(xyz=(0.28, 0.0, 0.34)),
        material=powder_black,
        name="seat_top_bar",
    )
    frame.visual(
        Box((0.06, 0.24, 0.04)),
        origin=Origin(xyz=(0.31, 0.0, 0.38)),
        material=powder_black,
        name="seat_hinge_bridge",
    )
    frame.visual(
        Box((0.10, 0.18, 0.06)),
        origin=Origin(xyz=(-0.27, 0.0, 0.14)),
        material=powder_black,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.40, 0.04, 0.05)),
        origin=Origin(xyz=(-0.09, 0.085, 0.30)),
        material=powder_black,
        name="top_rail_0",
    )
    frame.visual(
        Box((0.40, 0.04, 0.05)),
        origin=Origin(xyz=(-0.09, -0.085, 0.30)),
        material=powder_black,
        name="top_rail_1",
    )
    frame.visual(
        Box((0.06, 0.05, 0.24)),
        origin=Origin(xyz=(-0.24, 0.085, 0.25)),
        material=powder_black,
        name="rear_post_0",
    )
    frame.visual(
        Box((0.06, 0.05, 0.24)),
        origin=Origin(xyz=(-0.24, -0.085, 0.25)),
        material=powder_black,
        name="rear_post_1",
    )
    frame.visual(
        Box((0.08, 0.26, 0.08)),
        origin=Origin(xyz=(-0.01, 0.0, 0.365)),
        material=powder_black,
        name="back_hinge_bridge",
    )
    frame.visual(
        Box((0.08, 0.03, 0.12)),
        origin=Origin(xyz=(0.56, 0.1725, 0.10)),
        material=dark_steel,
        name="wheel_bracket_0",
    )
    frame.visual(
        Box((0.08, 0.03, 0.12)),
        origin=Origin(xyz=(0.56, -0.1725, 0.10)),
        material=dark_steel,
        name="wheel_bracket_1",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.30),
        origin=Origin(xyz=(0.56, 0.0, 0.092), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_axle",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        Box((0.90, 0.28, 0.012)),
        origin=Origin(xyz=(-0.45, 0.0, 0.006)),
        material=board_gray,
        name="back_board",
    )
    back_pad.visual(
        Box((0.92, 0.30, 0.05)),
        origin=Origin(xyz=(-0.46, 0.0, 0.037)),
        material=vinyl_black,
        name="back_cushion",
    )
    back_pad.visual(
        Box((0.56, 0.10, 0.04)),
        origin=Origin(xyz=(-0.32, 0.0, 0.020)),
        material=powder_black,
        name="back_rail",
    )
    back_pad.visual(
        Cylinder(radius=0.018, length=0.22),
        origin=Origin(xyz=(-0.01, 0.0, 0.018), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="back_hinge_tube",
    )
    back_pad.visual(
        Box((0.08, 0.16, 0.035)),
        origin=Origin(xyz=(-0.24, 0.0, 0.010)),
        material=dark_steel,
        name="back_latch_bar",
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        Box((0.31, 0.27, 0.012)),
        origin=Origin(xyz=(-0.155, 0.0, 0.006)),
        material=board_gray,
        name="seat_board",
    )
    seat_pad.visual(
        Box((0.31, 0.29, 0.05)),
        origin=Origin(xyz=(-0.155, 0.0, 0.037)),
        material=vinyl_black,
        name="seat_cushion",
    )
    seat_pad.visual(
        Box((0.18, 0.10, 0.035)),
        origin=Origin(xyz=(-0.16, 0.0, 0.019)),
        material=powder_black,
        name="seat_rail",
    )
    seat_pad.visual(
        Cylinder(radius=0.016, length=0.20),
        origin=Origin(xyz=(-0.01, 0.0, 0.016), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="seat_hinge_tube",
    )
    seat_pad.visual(
        Box((0.05, 0.14, 0.025)),
        origin=Origin(xyz=(-0.25, 0.0, 0.018)),
        material=dark_steel,
        name="seat_rear_block",
    )

    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="brace_pivot_tube",
    )
    rear_brace.visual(
        Box((0.40, 0.045, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, 0.040), rpy=(0.0, -0.08, 0.0)),
        material=dark_steel,
        name="brace_beam",
    )
    rear_brace.visual(
        Box((0.06, 0.045, 0.020)),
        origin=Origin(xyz=(0.16, 0.0, 0.065)),
        material=dark_steel,
        name="brace_step_0",
    )
    rear_brace.visual(
        Box((0.06, 0.045, 0.020)),
        origin=Origin(xyz=(0.20, 0.0, 0.095)),
        material=dark_steel,
        name="brace_step_1",
    )
    rear_brace.visual(
        Box((0.08, 0.045, 0.024)),
        origin=Origin(xyz=(0.24, 0.0, 0.035)),
        material=dark_steel,
        name="brace_step_2",
    )
    rear_brace.visual(
        Box((0.08, 0.045, 0.024)),
        origin=Origin(xyz=(0.28, 0.0, 0.055)),
        material=dark_steel,
        name="brace_step_3",
    )
    rear_brace.visual(
        Box((0.16, 0.08, 0.03)),
        origin=Origin(xyz=(0.20, 0.0, 0.085)),
        material=dark_steel,
        name="brace_step_web",
    )
    rear_brace.visual(
        Box((0.10, 0.08, 0.04)),
        origin=Origin(xyz=(0.26, 0.0, 0.045)),
        material=hub_gray,
        name="brace_top_step",
    )

    for wheel_name in ("front_wheel_0", "front_wheel_1"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.055, length=0.035),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.036, length=0.041),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=0.045),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=hub_gray,
            name="hub_cap",
        )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(0.32, 0.0, 0.405)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=0.40),
    )
    model.articulation(
        "brace_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_brace,
        origin=Origin(xyz=(-0.36, 0.0, 0.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.5, lower=0.0, upper=0.72),
    )
    model.articulation(
        "front_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child="front_wheel_0",
        origin=Origin(xyz=(0.56, 0.21, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child="front_wheel_1",
        origin=Origin(xyz=(0.56, -0.21, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "frame",
        "rear_brace",
        elem_a="main_spine",
        elem_b="brace_pivot_tube",
        reason="The rear brace pivots through the lower frame mount; the centered pivot tube stands in for a pin passing through the spine bracket.",
    )

    back_pad = object_model.get_part("back_pad")
    seat_pad = object_model.get_part("seat_pad")
    rear_brace = object_model.get_part("rear_brace")
    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    brace_hinge = object_model.get_articulation("brace_hinge")
    wheel_0_spin = object_model.get_articulation("front_wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("front_wheel_1_spin")

    with ctx.pose({back_hinge: 0.0, seat_hinge: 0.0, brace_hinge: 0.0}):
        ctx.expect_gap(
            seat_pad,
            back_pad,
            axis="x",
            positive_elem="seat_cushion",
            negative_elem="back_cushion",
            min_gap=0.008,
            max_gap=0.030,
            name="seat pad sits just ahead of the back pad",
        )
        ctx.expect_overlap(
            seat_pad,
            back_pad,
            axes="y",
            elem_a="seat_cushion",
            elem_b="back_cushion",
            min_overlap=0.26,
            name="seat and back pads stay aligned across the width",
        )

    back_lower_aabb = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    with ctx.pose({back_hinge: 1.10}):
        back_upper_aabb = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    ctx.check(
        "back pad raises to a real incline",
        back_lower_aabb is not None
        and back_upper_aabb is not None
        and back_upper_aabb[1][2] > back_lower_aabb[1][2] + 0.30,
        details=f"flat={back_lower_aabb}, raised={back_upper_aabb}",
    )

    seat_lower_aabb = ctx.part_element_world_aabb(seat_pad, elem="seat_cushion")
    with ctx.pose({seat_hinge: 0.32}):
        seat_upper_aabb = ctx.part_element_world_aabb(seat_pad, elem="seat_cushion")
    ctx.check(
        "seat pad lifts at the rear support angle",
        seat_lower_aabb is not None
        and seat_upper_aabb is not None
        and seat_upper_aabb[1][2] > seat_lower_aabb[1][2] + 0.06,
        details=f"flat={seat_lower_aabb}, raised={seat_upper_aabb}",
    )

    brace_lower_aabb = ctx.part_element_world_aabb(rear_brace, elem="brace_top_step")
    with ctx.pose({brace_hinge: 0.62}):
        brace_upper_aabb = ctx.part_element_world_aabb(rear_brace, elem="brace_top_step")
    ctx.check(
        "rear brace swings upward into support position",
        brace_lower_aabb is not None
        and brace_upper_aabb is not None
        and brace_upper_aabb[1][2] > brace_lower_aabb[1][2] + 0.12
        and brace_upper_aabb[0][0] < brace_lower_aabb[0][0] - 0.06,
        details=f"folded={brace_lower_aabb}, deployed={brace_upper_aabb}",
    )

    with ctx.pose({back_hinge: 0.50, brace_hinge: 0.62}):
        ctx.expect_gap(
            back_pad,
            rear_brace,
            axis="z",
            positive_elem="back_latch_bar",
            negative_elem="brace_top_step",
            min_gap=0.0,
            max_gap=0.09,
            name="brace reaches the underside of the raised back pad",
        )
        ctx.expect_overlap(
            back_pad,
            rear_brace,
            axes="x",
            elem_a="back_latch_bar",
            elem_b="brace_top_step",
            min_overlap=0.02,
            name="brace stays positioned under the backrest latch bar",
        )

    for wheel_joint in (wheel_0_spin, wheel_1_spin):
        ctx.check(
            f"{wheel_joint.name} uses continuous wheel motion",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
