from __future__ import annotations

import math

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
    model = ArticulatedObject(name="decline_bench")

    frame_mat = model.material("frame_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    pad_mat = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    board_mat = model.material("board", rgba=(0.24, 0.21, 0.18, 1.0))

    frame = model.part("frame")
    for name, size, xyz in (
        ("rail_0", (1.00, 0.04, 0.06), (0.03, -0.18, 0.30)),
        ("rail_1", (1.00, 0.04, 0.06), (0.03, 0.18, 0.30)),
        ("mid_crossmember", (0.10, 0.32, 0.04), (0.12, 0.00, 0.30)),
        ("front_foot", (0.10, 0.50, 0.06), (0.52, 0.00, 0.03)),
        ("front_upright_0", (0.08, 0.04, 0.39), (0.47, -0.18, 0.225)),
        ("front_upright_1", (0.08, 0.04, 0.39), (0.47, 0.18, 0.225)),
        ("rear_axle_beam", (0.10, 0.44, 0.10), (-0.47, 0.00, 0.05)),
        ("rear_post_0", (0.10, 0.04, 0.25), (-0.39, -0.18, 0.175)),
        ("rear_post_1", (0.10, 0.04, 0.25), (-0.39, 0.18, 0.175)),
        ("back_support_0", (0.08, 0.04, 0.08), (0.00, -0.18, 0.30)),
        ("back_support_1", (0.08, 0.04, 0.08), (0.00, 0.18, 0.30)),
        ("back_hinge_bridge", (0.08, 0.36, 0.04), (0.00, 0.00, 0.34)),
        ("seat_support_0", (0.20, 0.04, 0.05), (0.39, -0.18, 0.34)),
        ("seat_support_1", (0.20, 0.04, 0.05), (0.39, 0.18, 0.34)),
        ("seat_hinge_bridge", (0.08, 0.36, 0.04), (0.31, 0.00, 0.34)),
        ("brace_hanger_0", (0.03, 0.03, 0.09), (0.065, -0.155, 0.255)),
        ("brace_hanger_1", (0.03, 0.03, 0.09), (0.065, 0.155, 0.255)),
        ("brace_mount_bar", (0.03, 0.34, 0.03), (0.065, 0.00, 0.22)),
        ("leg_post", (0.08, 0.08, 0.26), (0.52, 0.00, 0.19)),
        ("leg_neck", (0.12, 0.08, 0.04), (0.57, 0.00, 0.34)),
        ("roller_head", (0.08, 0.08, 0.12), (0.578, 0.00, 0.38)),
        ("wheel_hub_0", (0.03, 0.03, 0.03), (-0.47, -0.2225, 0.06)),
        ("wheel_hub_1", (0.03, 0.03, 0.03), (-0.47, 0.2225, 0.06)),
    ):
        frame.visual(Box(size), origin=Origin(xyz=xyz), material=frame_mat, name=name)
    frame.visual(
        Cylinder(radius=0.012, length=0.08),
        origin=Origin(xyz=(0.63, 0.00, 0.38), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="roller_axle",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        Box((0.90, 0.28, 0.02)),
        origin=Origin(xyz=(-0.42, 0.00, 0.01)),
        material=board_mat,
        name="back_board",
    )
    back_pad.visual(
        Box((0.92, 0.30, 0.06)),
        origin=Origin(xyz=(-0.42, 0.00, 0.05)),
        material=pad_mat,
        name="back_cushion",
    )
    back_pad.visual(
        Box((0.04, 0.03, 0.04)),
        origin=Origin(xyz=(0.01, -0.10, 0.01)),
        material=frame_mat,
        name="back_hinge_block_0",
    )
    back_pad.visual(
        Box((0.04, 0.03, 0.04)),
        origin=Origin(xyz=(0.01, 0.10, 0.01)),
        material=frame_mat,
        name="back_hinge_block_1",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.30, 0.28, 0.02)),
        origin=Origin(xyz=(-0.14, 0.00, 0.01)),
        material=board_mat,
        name="seat_board",
    )
    seat.visual(
        Box((0.32, 0.30, 0.06)),
        origin=Origin(xyz=(-0.14, 0.00, 0.05)),
        material=pad_mat,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.04, 0.03, 0.04)),
        origin=Origin(xyz=(0.01, -0.10, 0.01)),
        material=frame_mat,
        name="seat_hinge_block_0",
    )
    seat.visual(
        Box((0.04, 0.03, 0.04)),
        origin=Origin(xyz=(0.01, 0.10, 0.01)),
        material=frame_mat,
        name="seat_hinge_block_1",
    )

    brace = model.part("brace")
    brace.visual(
        Box((0.03, 0.34, 0.03)),
        origin=Origin(xyz=(0.015, 0.00, 0.00)),
        material=frame_mat,
        name="brace_rear",
    )
    brace.visual(
        Box((0.32, 0.03, 0.03)),
        origin=Origin(xyz=(0.17, -0.155, 0.00)),
        material=frame_mat,
        name="brace_side_0",
    )
    brace.visual(
        Box((0.32, 0.03, 0.03)),
        origin=Origin(xyz=(0.17, 0.155, 0.00)),
        material=frame_mat,
        name="brace_side_1",
    )
    brace.visual(
        Box((0.03, 0.34, 0.03)),
        origin=Origin(xyz=(0.325, 0.00, 0.00)),
        material=frame_mat,
        name="brace_front",
    )
    roller_0 = model.part("roller_0")
    roller_0.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pad_mat,
        name="roller_body",
    )
    roller_0.visual(
        Box((0.02, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=frame_mat,
        name="roller_marker",
    )

    roller_1 = model.part("roller_1")
    roller_1.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pad_mat,
        name="roller_body",
    )
    roller_1.visual(
        Box((0.02, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=frame_mat,
        name="roller_marker",
    )

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.06, length=0.035),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="tire",
    )
    wheel_0.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=board_mat,
        name="hub",
    )
    wheel_0.visual(
        Box((0.01, 0.008, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=board_mat,
        name="wheel_marker",
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.06, length=0.035),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="tire",
    )
    wheel_1.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=board_mat,
        name="hub",
    )
    wheel_1.visual(
        Box((0.01, 0.008, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=board_mat,
        name="wheel_marker",
    )

    model.articulation(
        "frame_to_back_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.00, 0.00, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.35, upper=1.20),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.34, 0.00, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=0.55),
    )
    model.articulation(
        "frame_to_brace",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace,
        origin=Origin(xyz=(0.08, 0.00, 0.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-1.05, upper=0.35),
    )
    model.articulation(
        "frame_to_roller_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=roller_0,
        origin=Origin(xyz=(0.63, -0.10, 0.38)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_roller_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=roller_1,
        origin=Origin(xyz=(0.63, 0.10, 0.38)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.47, -0.2575, 0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.47, 0.2575, 0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat = object_model.get_part("seat")
    brace = object_model.get_part("brace")
    roller_0 = object_model.get_part("roller_0")
    wheel_0 = object_model.get_part("wheel_0")

    back_joint = object_model.get_articulation("frame_to_back_pad")
    seat_joint = object_model.get_articulation("frame_to_seat")
    brace_joint = object_model.get_articulation("frame_to_brace")
    roller_joint = object_model.get_articulation("frame_to_roller_0")
    wheel_joint = object_model.get_articulation("frame_to_wheel_0")

    ctx.expect_gap(
        back_pad,
        frame,
        axis="z",
        positive_elem="back_board",
        negative_elem="back_hinge_bridge",
        min_gap=0.005,
        max_gap=0.03,
        name="back pad rides just above its hinge bridge",
    )
    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        positive_elem="seat_board",
        negative_elem="seat_hinge_bridge",
        min_gap=0.005,
        max_gap=0.03,
        name="seat rides just above its hinge bridge",
    )
    ctx.expect_gap(
        seat,
        back_pad,
        axis="x",
        positive_elem="seat_board",
        negative_elem="back_board",
        max_gap=0.025,
        max_penetration=0.0,
        name="seat meets the back pad with only a narrow hinge gap",
    )

    back_limits = back_joint.motion_limits
    seat_limits = seat_joint.motion_limits
    brace_limits = brace_joint.motion_limits

    rest_back = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    rest_seat = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    rest_brace = ctx.part_element_world_aabb(brace, elem="brace_front")

    if back_limits is not None and back_limits.upper is not None and rest_back is not None:
        with ctx.pose({back_joint: back_limits.upper}):
            raised_back = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
        ctx.check(
            "back pad raises into a tall incline",
            raised_back is not None and raised_back[1][2] > rest_back[1][2] + 0.22,
            details=f"rest={rest_back}, raised={raised_back}",
        )

    if seat_limits is not None and seat_limits.upper is not None and rest_seat is not None:
        with ctx.pose({seat_joint: seat_limits.upper}):
            raised_seat = ctx.part_element_world_aabb(seat, elem="seat_cushion")
        ctx.check(
            "seat tips upward at the rear",
            raised_seat is not None and raised_seat[1][2] > rest_seat[1][2] + 0.07,
            details=f"rest={rest_seat}, raised={raised_seat}",
        )

    if brace_limits is not None and brace_limits.lower is not None and rest_brace is not None:
        with ctx.pose({brace_joint: brace_limits.lower}):
            tucked_brace = ctx.part_element_world_aabb(brace, elem="brace_front")
        ctx.check(
            "brace tucks upward under the bench",
            tucked_brace is not None
            and tucked_brace[1][2] > rest_brace[1][2] + 0.10
            and tucked_brace[1][0] < rest_brace[1][0] - 0.08,
            details=f"rest={rest_brace}, tucked={tucked_brace}",
        )

    roller_rest = ctx.part_element_world_aabb(roller_0, elem="roller_marker")
    if roller_rest is not None:
        with ctx.pose({roller_joint: 1.3}):
            roller_turned = ctx.part_element_world_aabb(roller_0, elem="roller_marker")
        ctx.check(
            "leg roller spins around its axle",
            roller_turned is not None and roller_turned != roller_rest,
            details=f"rest={roller_rest}, turned={roller_turned}",
        )

    wheel_rest = ctx.part_element_world_aabb(wheel_0, elem="wheel_marker")
    if wheel_rest is not None:
        with ctx.pose({wheel_joint: 1.2}):
            wheel_turned = ctx.part_element_world_aabb(wheel_0, elem="wheel_marker")
        ctx.check(
            "transport wheel spins on the rear axle",
            wheel_turned is not None and wheel_turned != wheel_rest,
            details=f"rest={wheel_rest}, turned={wheel_turned}",
        )

    return ctx.report()


object_model = build_object_model()
