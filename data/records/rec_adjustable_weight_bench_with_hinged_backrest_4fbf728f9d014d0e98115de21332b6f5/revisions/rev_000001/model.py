from __future__ import annotations

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
    model = ArticulatedObject(name="apartment_gym_bench")

    frame_mat = model.material("frame_mat", color=(0.18, 0.19, 0.21))
    pad_mat = model.material("pad_mat", color=(0.12, 0.12, 0.13))
    wheel_mat = model.material("wheel_mat", color=(0.09, 0.09, 0.10))
    trim_mat = model.material("trim_mat", color=(0.44, 0.46, 0.48))

    frame = model.part("frame")
    frame.visual(
        Box((0.88, 0.03, 0.04)),
        origin=Origin(xyz=(-0.02, -0.105, 0.315)),
        material=frame_mat,
        name="rail_0",
    )
    frame.visual(
        Box((0.88, 0.03, 0.04)),
        origin=Origin(xyz=(-0.02, 0.105, 0.315)),
        material=frame_mat,
        name="rail_1",
    )
    frame.visual(
        Box((0.08, 0.22, 0.04)),
        origin=Origin(xyz=(0.36, 0.0, 0.315)),
        material=frame_mat,
        name="front_cross",
    )
    frame.visual(
        Box((0.08, 0.22, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.315)),
        material=frame_mat,
        name="seat_cross",
    )
    frame.visual(
        Box((0.10, 0.22, 0.04)),
        origin=Origin(xyz=(-0.34, 0.0, 0.315)),
        material=frame_mat,
        name="rear_cross",
    )
    frame.visual(
        Box((0.06, 0.02, 0.15)),
        origin=Origin(xyz=(-0.28, -0.09, 0.24)),
        material=frame_mat,
        name="ladder_ear_0",
    )
    frame.visual(
        Box((0.06, 0.02, 0.15)),
        origin=Origin(xyz=(-0.28, 0.09, 0.24)),
        material=frame_mat,
        name="ladder_ear_1",
    )
    frame.visual(
        Box((0.04, 0.03, 0.22)),
        origin=Origin(xyz=(-0.43, -0.105, 0.195)),
        material=frame_mat,
        name="wheel_bracket_0",
    )
    frame.visual(
        Box((0.04, 0.03, 0.22)),
        origin=Origin(xyz=(-0.43, 0.105, 0.195)),
        material=frame_mat,
        name="wheel_bracket_1",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.26),
        origin=Origin(xyz=(-0.43, 0.0, 0.085), rpy=(1.5708, 0.0, 0.0)),
        material=trim_mat,
        name="rear_axle",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        Box((0.64, 0.23, 0.016)),
        origin=Origin(xyz=(-0.332, 0.0, 0.008)),
        material=trim_mat,
        name="back_board",
    )
    back_pad.visual(
        Box((0.66, 0.25, 0.056)),
        origin=Origin(xyz=(-0.342, 0.0, 0.043)),
        material=pad_mat,
        name="back_cushion",
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        Box((0.27, 0.23, 0.016)),
        origin=Origin(xyz=(0.147, 0.0, 0.008)),
        material=trim_mat,
        name="seat_board",
    )
    seat_pad.visual(
        Box((0.29, 0.25, 0.056)),
        origin=Origin(xyz=(0.157, 0.0, 0.043)),
        material=pad_mat,
        name="seat_cushion",
    )

    ladder = model.part("ladder_support")
    ladder.visual(
        Box((0.028, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=frame_mat,
        name="ladder_base",
    )
    ladder.visual(
        Box((0.022, 0.02, 0.165)),
        origin=Origin(xyz=(0.0, -0.07, 0.0825)),
        material=frame_mat,
        name="ladder_strut_0",
    )
    ladder.visual(
        Box((0.022, 0.02, 0.165)),
        origin=Origin(xyz=(0.0, 0.07, 0.0825)),
        material=frame_mat,
        name="ladder_strut_1",
    )
    ladder.visual(
        Box((0.018, 0.16, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=trim_mat,
        name="ladder_rung_0",
    )
    ladder.visual(
        Box((0.018, 0.16, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=trim_mat,
        name="ladder_rung_1",
    )
    ladder.visual(
        Box((0.03, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=trim_mat,
        name="ladder_top",
    )

    front_foot = model.part("front_foot")
    front_foot.visual(
        Box((0.04, 0.18, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=frame_mat,
        name="top_block",
    )
    front_foot.visual(
        Box((0.018, 0.018, 0.22)),
        origin=Origin(xyz=(0.0, -0.075, -0.12)),
        material=frame_mat,
        name="foot_leg_0",
    )
    front_foot.visual(
        Box((0.018, 0.018, 0.22)),
        origin=Origin(xyz=(0.0, 0.075, -0.12)),
        material=frame_mat,
        name="foot_leg_1",
    )
    front_foot.visual(
        Box((0.028, 0.20, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, -0.235)),
        material=trim_mat,
        name="foot_bar",
    )

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(rpy=(1.5708, 0.0, 0.0)),
        material=wheel_mat,
        name="wheel",
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(rpy=(1.5708, 0.0, 0.0)),
        material=wheel_mat,
        name="wheel",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(-0.015, 0.0, 0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8, lower=0.0, upper=1.15),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(0.015, 0.0, 0.335)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=0.38),
    )
    model.articulation(
        "rear_support_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ladder,
        origin=Origin(xyz=(-0.28, 0.0, 0.18), rpy=(0.0, 0.5, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=-0.45, upper=0.35),
    )
    model.articulation(
        "front_foot_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_foot,
        origin=Origin(xyz=(0.36, 0.0, 0.295)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=0.0, upper=1.1),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.43, -0.144, 0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.43, 0.144, 0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat_pad = object_model.get_part("seat_pad")
    ladder = object_model.get_part("ladder_support")
    front_foot = object_model.get_part("front_foot")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    rear_support_hinge = object_model.get_articulation("rear_support_hinge")
    front_foot_hinge = object_model.get_articulation("front_foot_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.expect_gap(
        seat_pad,
        back_pad,
        axis="x",
        positive_elem="seat_board",
        negative_elem="back_board",
        min_gap=0.03,
        max_gap=0.07,
        name="seat pad leaves a compact hinge gap ahead of the back pad",
    )
    ctx.expect_overlap(
        seat_pad,
        back_pad,
        axes="y",
        elem_a="seat_cushion",
        elem_b="back_cushion",
        min_overlap=0.20,
        name="seat and back pads stay aligned across the narrow bench width",
    )
    ctx.expect_gap(
        frame,
        front_foot,
        axis="z",
        positive_elem="front_cross",
        negative_elem="top_block",
        min_gap=0.0,
        max_gap=0.004,
        name="front foot nests directly under the front crosspiece",
    )
    ctx.expect_gap(
        back_pad,
        ladder,
        axis="z",
        positive_elem="back_board",
        negative_elem="ladder_top",
        min_gap=0.0,
        max_gap=0.02,
        name="ladder support sits just below the back pad in the flat bench pose",
    )
    ctx.expect_within(
        ladder,
        back_pad,
        axes="y",
        inner_elem="ladder_top",
        outer_elem="back_cushion",
        margin=0.02,
        name="rear ladder support stays centered under the back pad",
    )

    back_limits = back_hinge.motion_limits
    seat_limits = seat_hinge.motion_limits
    support_limits = rear_support_hinge.motion_limits
    foot_limits = front_foot_hinge.motion_limits
    wheel_0_limits = wheel_0_spin.motion_limits
    wheel_1_limits = wheel_1_spin.motion_limits

    back_rest = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    seat_rest = ctx.part_element_world_aabb(seat_pad, elem="seat_cushion")
    foot_rest = ctx.part_element_world_aabb(front_foot, elem="foot_bar")

    back_open = None
    if back_limits is not None and back_limits.upper is not None:
        with ctx.pose({back_hinge: back_limits.upper}):
            back_open = ctx.part_element_world_aabb(back_pad, elem="back_cushion")

    seat_open = None
    if seat_limits is not None and seat_limits.upper is not None:
        with ctx.pose({seat_hinge: seat_limits.upper}):
            seat_open = ctx.part_element_world_aabb(seat_pad, elem="seat_cushion")

    support_low = None
    support_high = None
    if (
        support_limits is not None
        and support_limits.lower is not None
        and support_limits.upper is not None
    ):
        with ctx.pose({rear_support_hinge: support_limits.lower}):
            support_low = ctx.part_element_world_aabb(ladder, elem="ladder_top")
        with ctx.pose({rear_support_hinge: support_limits.upper}):
            support_high = ctx.part_element_world_aabb(ladder, elem="ladder_top")

    foot_folded = None
    if foot_limits is not None and foot_limits.upper is not None:
        with ctx.pose({front_foot_hinge: foot_limits.upper}):
            foot_folded = ctx.part_element_world_aabb(front_foot, elem="foot_bar")

    ctx.check(
        "back pad raises clearly above the frame when opened",
        back_rest is not None
        and back_open is not None
        and back_open[1][2] > back_rest[1][2] + 0.20,
        details=f"rest={back_rest}, open={back_open}",
    )
    ctx.check(
        "seat pad front edge lifts upward on its own hinge",
        seat_rest is not None
        and seat_open is not None
        and seat_open[1][2] > seat_rest[1][2] + 0.04,
        details=f"rest={seat_rest}, open={seat_open}",
    )
    ctx.check(
        "rear ladder support rotates through a visible adjustment sweep",
        support_low is not None
        and support_high is not None
        and support_high[1][0] > support_low[1][0] + 0.06,
        details=f"low={support_low}, high={support_high}",
    )
    ctx.check(
        "front support foot folds rearward and upward",
        foot_rest is not None
        and foot_folded is not None
        and foot_folded[0][2] > foot_rest[0][2] + 0.10
        and foot_folded[1][0] < foot_rest[1][0] - 0.08,
        details=f"rest={foot_rest}, folded={foot_folded}",
    )
    ctx.check(
        "transport wheels are continuous joints without angle stops",
        wheel_0_limits is not None
        and wheel_1_limits is not None
        and wheel_0_limits.lower is None
        and wheel_0_limits.upper is None
        and wheel_1_limits.lower is None
        and wheel_1_limits.upper is None,
        details=f"wheel_0={wheel_0_limits}, wheel_1={wheel_1_limits}",
    )
    ctx.expect_gap(
        back_pad,
        frame,
        axis="z",
        positive_elem="back_board",
        negative_elem="rear_cross",
        min_gap=0.0,
        max_gap=0.05,
        name="back pad stays above the rear frame crossmember",
    )
    ctx.expect_gap(
        seat_pad,
        frame,
        axis="z",
        positive_elem="seat_board",
        negative_elem="seat_cross",
        min_gap=0.0,
        max_gap=0.05,
        name="seat pad stays above the seat crossmember",
    )

    return ctx.report()


object_model = build_object_model()
