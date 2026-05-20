from __future__ import annotations

from math import pi

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


def _upholstery_mesh(name: str, *, width: float, length: float, height: float, fillet: float):
    shape = (
        cq.Workplane("XY")
        .box(width, length, height)
        .edges()
        .fillet(fillet)
        .translate((0.0, length * 0.5, height * 0.5))
    )
    return mesh_from_cadquery(shape, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_gym_bench")

    frame_black = model.material("frame_black", rgba=(0.13, 0.13, 0.14, 1.0))
    frame_gloss = model.material("frame_gloss", rgba=(0.19, 0.20, 0.21, 1.0))
    upholstery = model.material("upholstery", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    hardware = model.material("hardware", rgba=(0.48, 0.49, 0.50, 1.0))

    main_frame = model.part("main_frame")
    main_frame.visual(
        Box((0.66, 0.09, 0.055)),
        origin=Origin(xyz=(0.0, -0.52, 0.0275)),
        material=frame_black,
        name="front_stabilizer",
    )
    main_frame.visual(
        Box((0.74, 0.10, 0.055)),
        origin=Origin(xyz=(0.0, 0.52, 0.0275)),
        material=frame_black,
        name="rear_stabilizer",
    )
    for side in (-1.0, 1.0):
        x_pos = 0.205 * side
        main_frame.visual(
            Box((0.055, 0.42, 0.055)),
            origin=Origin(xyz=(x_pos, -0.35, 0.20), rpy=(0.79, 0.0, 0.0)),
            material=frame_black,
            name=f"front_leg_{int((side + 1.0) * 0.5)}",
        )
        main_frame.visual(
            Box((0.09, 0.48, 0.06)),
            origin=Origin(xyz=(0.17 * side, 0.06, 0.35)),
            material=frame_black,
            name=f"upper_rail_{int((side + 1.0) * 0.5)}",
        )
        main_frame.visual(
            Box((0.07, 0.18, 0.31)),
            origin=Origin(xyz=(0.18 * side, 0.44, 0.185)),
            material=frame_black,
            name=f"rear_post_{int((side + 1.0) * 0.5)}",
        )
        main_frame.visual(
            Box((0.08, 0.08, 0.12)),
            origin=Origin(xyz=(0.15 * side, 0.18, 0.32)),
            material=frame_black,
            name=f"hinge_upright_{int((side + 1.0) * 0.5)}",
        )
        main_frame.visual(
            Box((0.06, 0.09, 0.26)),
            origin=Origin(xyz=(0.16 * side, 0.32, 0.25)),
            material=frame_black,
            name=f"support_cheek_{int((side + 1.0) * 0.5)}",
        )
    main_frame.visual(
        Box((0.04, 0.05, 0.09)),
        origin=Origin(xyz=(-0.175, -0.601, 0.10)),
        material=frame_black,
        name="wheel_mount_0",
    )
    main_frame.visual(
        Box((0.04, 0.08, 0.09)),
        origin=Origin(xyz=(-0.175, -0.551, 0.055)),
        material=frame_black,
        name="wheel_brace_0",
    )
    main_frame.visual(
        Box((0.04, 0.05, 0.09)),
        origin=Origin(xyz=(0.175, -0.601, 0.10)),
        material=frame_black,
        name="wheel_mount_1",
    )
    main_frame.visual(
        Box((0.04, 0.08, 0.09)),
        origin=Origin(xyz=(0.175, -0.551, 0.055)),
        material=frame_black,
        name="wheel_brace_1",
    )
    main_frame.visual(
        Box((0.46, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, -0.18, 0.35)),
        material=frame_black,
        name="seat_crossmember",
    )
    main_frame.visual(
        Box((0.30, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.36, 0.11)),
        material=frame_black,
        name="support_bridge",
    )
    main_frame.visual(
        Box((0.34, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.18, 0.36)),
        material=frame_gloss,
        name="hinge_bridge",
    )
    main_frame.visual(
        Box((0.34, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.27, 0.34)),
        material=frame_gloss,
        name="backrest_stop",
    )

    seat = model.part("seat")
    seat.visual(
        _upholstery_mesh(
            "seat_pad",
            width=0.31,
            length=0.33,
            height=0.08,
            fillet=0.013,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=upholstery,
        name="seat_pad",
    )
    seat.visual(
        Box((0.23, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.19, 0.0)),
        material=frame_gloss,
        name="seat_plate",
    )
    seat.visual(
        Box((0.17, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.28, -0.01)),
        material=frame_black,
        name="seat_brace",
    )

    backrest = model.part("backrest")
    backrest.visual(
        _upholstery_mesh(
            "backrest_pad",
            width=0.31,
            length=0.92,
            height=0.075,
            fillet=0.013,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=upholstery,
        name="back_pad",
    )
    backrest.visual(
        Box((0.24, 0.74, 0.035)),
        origin=Origin(xyz=(0.0, 0.39, 0.0025)),
        material=frame_gloss,
        name="back_plate",
    )
    backrest.visual(
        Box((0.20, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.095, -0.005)),
        material=frame_black,
        name="hinge_block",
    )

    support_arm = model.part("support_arm")
    support_arm.visual(
        Cylinder(radius=0.022, length=0.24),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="pivot_tube",
    )
    support_arm.visual(
        Box((0.04, 0.31, 0.025)),
        origin=Origin(xyz=(-0.11, 0.122, 0.095), rpy=(0.66, 0.0, 0.0)),
        material=frame_black,
        name="arm_side_0",
    )
    support_arm.visual(
        Box((0.04, 0.31, 0.025)),
        origin=Origin(xyz=(0.11, 0.122, 0.095), rpy=(0.66, 0.0, 0.0)),
        material=frame_black,
        name="arm_side_1",
    )
    support_arm.visual(
        Box((0.18, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.142, 0.11), rpy=(0.66, 0.0, 0.0)),
        material=frame_gloss,
        name="arm_web",
    )
    support_arm.visual(
        Cylinder(radius=0.012, length=0.26),
        origin=Origin(xyz=(0.0, 0.236, 0.182), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_rubber,
        name="support_roller",
    )

    front_handle = model.part("front_handle")
    front_handle.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(-0.09, -0.04, 0.045)),
        material=frame_gloss,
        name="handle_bracket_0",
    )
    front_handle.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(0.09, -0.04, 0.045)),
        material=frame_gloss,
        name="handle_bracket_1",
    )
    front_handle.visual(
        Box((0.03, 0.09, 0.03)),
        origin=Origin(xyz=(-0.09, -0.065, 0.06)),
        material=frame_gloss,
        name="handle_strut_0",
    )
    front_handle.visual(
        Box((0.03, 0.09, 0.03)),
        origin=Origin(xyz=(0.09, -0.065, 0.06)),
        material=frame_gloss,
        name="handle_strut_1",
    )
    front_handle.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(0.0, -0.06, 0.06), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="handle_bar",
    )

    for side in (-1.0, 1.0):
        wheel = model.part(f"wheel_{int((side + 1.0) * 0.5)}")
        wheel.visual(
            Cylinder(radius=0.06, length=0.03),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.032, length=0.038),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.05),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_gloss,
            name="axle_cap",
        )

    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=seat,
        origin=Origin(xyz=(0.0, -0.18, 0.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.08, upper=0.26),
    )
    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.18, 0.40)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=1.18),
    )
    model.articulation(
        "frame_to_support_arm",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=support_arm,
        origin=Origin(xyz=(0.0, 0.32, 0.19)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.22, upper=0.72),
    )
    model.articulation(
        "frame_to_front_handle",
        ArticulationType.FIXED,
        parent=main_frame,
        child=front_handle,
        origin=Origin(xyz=(0.0, -0.55, 0.0)),
    )
    for side in (-1.0, 1.0):
        wheel_index = int((side + 1.0) * 0.5)
        model.articulation(
            f"frame_to_wheel_{wheel_index}",
            ArticulationType.CONTINUOUS,
            parent=main_frame,
            child=f"wheel_{wheel_index}",
            origin=Origin(xyz=(0.22 * side, -0.64, 0.06)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    support_arm = object_model.get_part("support_arm")
    main_frame = object_model.get_part("main_frame")
    front_handle = object_model.get_part("front_handle")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    backrest_hinge = object_model.get_articulation("frame_to_backrest")
    support_hinge = object_model.get_articulation("frame_to_support_arm")
    wheel_joint_0 = object_model.get_articulation("frame_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("frame_to_wheel_1")

    ctx.expect_overlap(
        seat,
        backrest,
        axes="x",
        min_overlap=0.28,
        elem_a="seat_pad",
        elem_b="back_pad",
        name="seat and backrest share full bench width",
    )
    ctx.expect_gap(
        backrest,
        seat,
        axis="y",
        positive_elem="back_pad",
        negative_elem="seat_pad",
        min_gap=0.015,
        max_gap=0.045,
        name="split pads leave a realistic hinge gap",
    )
    ctx.expect_gap(
        seat,
        main_frame,
        axis="z",
        positive_elem="seat_pad",
        negative_elem="seat_crossmember",
        min_gap=0.0,
        max_gap=0.03,
        name="seat pad sits above the welded frame",
    )
    ctx.expect_gap(
        backrest,
        main_frame,
        axis="z",
        positive_elem="back_plate",
        negative_elem="hinge_bridge",
        min_gap=0.0,
        max_gap=0.02,
        name="backrest hinge sits just above the main frame bridge",
    )
    ctx.expect_gap(
        backrest,
        support_arm,
        axis="z",
        positive_elem="back_plate",
        negative_elem="support_roller",
        min_gap=0.0,
        max_gap=0.02,
        name="support arm roller stays just under the backrest plate",
    )
    ctx.expect_contact(
        front_handle,
        main_frame,
        elem_a="handle_bracket_0",
        elem_b="front_stabilizer",
        name="front handle is mounted to the front crossmember",
    )
    ctx.expect_gap(
        main_frame,
        front_handle,
        axis="y",
        positive_elem="front_stabilizer",
        negative_elem="handle_bar",
        min_gap=0.02,
        max_gap=0.06,
        name="front carry handle projects ahead of the front crossmember",
    )
    ctx.expect_contact(
        wheel_0,
        main_frame,
        elem_a="axle_cap",
        elem_b="wheel_mount_0",
        name="wheel 0 rides on its axle mount",
    )
    ctx.expect_contact(
        wheel_1,
        main_frame,
        elem_a="axle_cap",
        elem_b="wheel_mount_1",
        name="wheel 1 rides on its axle mount",
    )
    ctx.check(
        "transport wheels use continuous spin joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_0={wheel_joint_0.articulation_type}, joint_1={wheel_joint_1.articulation_type}",
    )

    backrest_limits = backrest_hinge.motion_limits
    support_limits = support_hinge.motion_limits
    seat_limits = seat_hinge.motion_limits
    if backrest_limits is not None and backrest_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
        with ctx.pose({backrest_hinge: backrest_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
        ctx.check(
            "backrest raises at maximum incline",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.25
            and open_aabb[1][1] < closed_aabb[1][1] - 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
    if support_limits is not None and support_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(support_arm, elem="support_roller")
        with ctx.pose({support_hinge: support_limits.upper}):
            raised_aabb = ctx.part_element_world_aabb(support_arm, elem="support_roller")
        ctx.check(
            "support arm swings upward",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.10,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )
    if seat_limits is not None and seat_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
        with ctx.pose({seat_hinge: seat_limits.upper}):
            raised_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
        ctx.check(
            "seat rear edge lifts on its forward hinge",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.03,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
