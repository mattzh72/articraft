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


def _along_y_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decline_bench")

    frame_gray = model.material("frame_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    pad_black = model.material("pad_black", rgba=(0.08, 0.08, 0.09, 1.0))
    foam_black = model.material("foam_black", rgba=(0.12, 0.12, 0.12, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.05, 0.05, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.24, 0.08, 0.06)),
        origin=Origin(xyz=(0.12, 0.0, 0.18)),
        material=frame_gray,
        name="spine",
    )
    frame.visual(
        Box((0.10, 0.55, 0.05)),
        origin=Origin(xyz=(-0.52, 0.0, 0.025)),
        material=frame_gray,
        name="rear_foot",
    )
    frame.visual(
        Box((0.08, 0.08, 0.28)),
        origin=Origin(xyz=(-0.52, 0.0, 0.18)),
        material=frame_gray,
        name="rear_post",
    )
    frame.visual(
        Box((0.22, 0.18, 0.12)),
        origin=Origin(xyz=(0.16, 0.0, 0.27)),
        material=frame_gray,
        name="pivot_tower",
    )
    frame.visual(
        Box((0.08, 0.08, 0.52)),
        origin=Origin(xyz=(0.82, 0.0, 0.44)),
        material=frame_gray,
        name="front_upright",
    )
    frame.visual(
        Box((0.42, 0.06, 0.06)),
        origin=Origin(xyz=(0.64, 0.0, 0.37), rpy=(0.0, -0.90, 0.0)),
        material=frame_gray,
        name="front_brace",
    )
    frame.visual(
        Box((0.10, 0.50, 0.08)),
        origin=Origin(xyz=(0.84, 0.0, 0.78)),
        material=frame_gray,
        name="yoke_cross",
    )
    frame.visual(
        Box((0.12, 0.46, 0.06)),
        origin=Origin(xyz=(0.84, 0.0, 0.49)),
        material=frame_gray,
        name="yoke_base",
    )
    frame.visual(
        Box((0.12, 0.04, 0.34)),
        origin=Origin(xyz=(0.84, -0.22, 0.61)),
        material=frame_gray,
        name="yoke_side_0",
    )
    frame.visual(
        Box((0.12, 0.04, 0.34)),
        origin=Origin(xyz=(0.84, 0.22, 0.61)),
        material=frame_gray,
        name="yoke_side_1",
    )
    frame.visual(
        Box((0.34, 0.04, 0.08)),
        origin=Origin(xyz=(1.01, -0.22, 0.56)),
        material=frame_gray,
        name="foot_arm_0",
    )
    frame.visual(
        Box((0.34, 0.04, 0.08)),
        origin=Origin(xyz=(1.01, 0.22, 0.56)),
        material=frame_gray,
        name="foot_arm_1",
    )
    frame.visual(
        Box((0.06, 0.40, 0.05)),
        origin=Origin(xyz=(1.20, 0.0, 0.56)),
        material=frame_gray,
        name="front_foot_bar",
    )
    frame.visual(
        Box((0.06, 0.06, 0.19)),
        origin=Origin(xyz=(0.84, 0.0, 0.595)),
        material=frame_gray,
        name="yoke_center_post",
    )
    frame.visual(
        Box((0.16, 0.06, 0.05)),
        origin=Origin(xyz=(1.13, 0.0, 0.56)),
        material=frame_gray,
        name="foot_center_bridge",
    )
    frame.visual(
        Box((0.06, 0.10, 0.05)),
        origin=Origin(xyz=(-0.52, 0.0, 0.055)),
        material=frame_gray,
        name="wheel_mount",
    )
    upper_axle_geometry, upper_axle_origin = _along_y_cylinder(radius=0.014, length=0.08)
    frame.visual(
        upper_axle_geometry,
        origin=Origin(xyz=(0.84, 0.0, 0.66), rpy=upper_axle_origin.rpy),
        material=frame_gray,
        name="upper_axle",
    )
    lower_axle_geometry, lower_axle_origin = _along_y_cylinder(radius=0.014, length=0.08)
    frame.visual(
        lower_axle_geometry,
        origin=Origin(xyz=(1.06, 0.0, 0.56), rpy=lower_axle_origin.rpy),
        material=frame_gray,
        name="lower_axle",
    )
    wheel_axle_geometry, wheel_axle_origin = _along_y_cylinder(radius=0.010, length=0.55)
    frame.visual(
        wheel_axle_geometry,
        origin=Origin(xyz=(-0.52, 0.0, 0.07), rpy=wheel_axle_origin.rpy),
        material=frame_gray,
        name="wheel_axle",
    )

    back = model.part("back_pad")
    back.visual(
        Box((0.92, 0.30, 0.08)),
        origin=Origin(xyz=(-0.46, 0.0, 0.04)),
        material=pad_black,
        name="back_cushion",
    )
    back.visual(
        Box((0.72, 0.08, 0.05)),
        origin=Origin(xyz=(-0.36, 0.0, -0.02)),
        material=frame_gray,
        name="back_support",
    )
    back.visual(
        Box((0.08, 0.14, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, -0.03)),
        material=frame_gray,
        name="back_hinge_block",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.34, 0.28, 0.075)),
        origin=Origin(xyz=(0.17, 0.0, 0.0375)),
        material=pad_black,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.26, 0.08, 0.05)),
        origin=Origin(xyz=(0.13, 0.0, -0.02)),
        material=frame_gray,
        name="seat_support",
    )
    seat.visual(
        Box((0.09, 0.14, 0.06)),
        origin=Origin(xyz=(0.03, 0.0, -0.03)),
        material=frame_gray,
        name="seat_hinge_block",
    )

    leg_roller_0 = model.part("leg_roller_0")
    leg_roller_geometry, leg_roller_origin = _along_y_cylinder(radius=0.065, length=0.16)
    leg_roller_0.visual(
        leg_roller_geometry,
        origin=leg_roller_origin,
        material=foam_black,
        name="roller",
    )
    leg_roller_1 = model.part("leg_roller_1")
    leg_roller_1.visual(
        leg_roller_geometry,
        origin=leg_roller_origin,
        material=foam_black,
        name="roller",
    )

    foot_roller_0 = model.part("foot_roller_0")
    foot_roller_geometry, foot_roller_origin = _along_y_cylinder(radius=0.055, length=0.16)
    foot_roller_0.visual(
        foot_roller_geometry,
        origin=foot_roller_origin,
        material=foam_black,
        name="roller",
    )
    foot_roller_1 = model.part("foot_roller_1")
    foot_roller_1.visual(
        foot_roller_geometry,
        origin=foot_roller_origin,
        material=foam_black,
        name="roller",
    )

    wheel_0 = model.part("wheel_0")
    wheel_geometry, wheel_origin = _along_y_cylinder(radius=0.055, length=0.035)
    wheel_0.visual(
        wheel_geometry,
        origin=wheel_origin,
        material=plastic_black,
        name="wheel_tire",
    )
    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        wheel_geometry,
        origin=wheel_origin,
        material=plastic_black,
        name="wheel_tire",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back,
        origin=Origin(xyz=(0.14, 0.0, 0.39)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.30, upper=1.10),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.24, 0.0, 0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.55),
    )
    model.articulation(
        "leg_roller_joint_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=leg_roller_0,
        origin=Origin(xyz=(0.84, -0.12, 0.66)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "leg_roller_joint_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=leg_roller_1,
        origin=Origin(xyz=(0.84, 0.12, 0.66)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "foot_roller_joint_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=foot_roller_0,
        origin=Origin(xyz=(1.06, -0.12, 0.56)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "foot_roller_joint_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=foot_roller_1,
        origin=Origin(xyz=(1.06, 0.12, 0.56)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "wheel_joint_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.52, -0.2925, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )
    model.articulation(
        "wheel_joint_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.52, 0.2925, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    back = object_model.get_part("back_pad")
    seat = object_model.get_part("seat")
    leg_roller_0 = object_model.get_part("leg_roller_0")
    leg_roller_1 = object_model.get_part("leg_roller_1")
    foot_roller_0 = object_model.get_part("foot_roller_0")
    foot_roller_1 = object_model.get_part("foot_roller_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    leg_roller_joint_0 = object_model.get_articulation("leg_roller_joint_0")
    leg_roller_joint_1 = object_model.get_articulation("leg_roller_joint_1")
    foot_roller_joint_0 = object_model.get_articulation("foot_roller_joint_0")
    foot_roller_joint_1 = object_model.get_articulation("foot_roller_joint_1")
    wheel_joint_0 = object_model.get_articulation("wheel_joint_0")
    wheel_joint_1 = object_model.get_articulation("wheel_joint_1")

    ctx.check(
        "back hinge is revolute",
        back_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={back_hinge.articulation_type}",
    )
    ctx.check(
        "seat hinge is revolute",
        seat_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={seat_hinge.articulation_type}",
    )
    ctx.check(
        "rollers and wheels use continuous joints",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            for joint in (
                leg_roller_joint_0,
                leg_roller_joint_1,
                foot_roller_joint_0,
                foot_roller_joint_1,
                wheel_joint_0,
                wheel_joint_1,
            )
        ),
        details=(
            f"types={[joint.articulation_type for joint in (leg_roller_joint_0, leg_roller_joint_1, foot_roller_joint_0, foot_roller_joint_1, wheel_joint_0, wheel_joint_1)]}"
        ),
    )
    with ctx.pose({back_hinge: 0.0, seat_hinge: 0.0}):
        ctx.expect_gap(
            seat,
            back,
            axis="x",
            min_gap=0.02,
            max_gap=0.10,
            name="seat sits just ahead of back pad",
        )
        ctx.expect_overlap(
            back,
            seat,
            axes="y",
            min_overlap=0.24,
            name="seat and back align across bench width",
        )
        ctx.expect_gap(
            back,
            frame,
            axis="z",
            min_gap=0.02,
            max_gap=0.25,
            positive_elem="back_cushion",
            negative_elem="pivot_tower",
            name="back pad clears the frame at rest",
        )
        ctx.expect_contact(
            leg_roller_0,
            frame,
            elem_a="roller",
            elem_b="yoke_side_0",
            name="first leg roller is carried by the front yoke",
        )
        ctx.expect_contact(
            foot_roller_0,
            frame,
            elem_a="roller",
            elem_b="foot_arm_0",
            name="first foot roller is carried by the front support arm",
        )
        ctx.expect_contact(
            wheel_0,
            frame,
            elem_a="wheel_tire",
            elem_b="rear_foot",
            name="first transport wheel mounts at the rear foot",
        )
        ctx.expect_gap(
            leg_roller_0,
            seat,
            axis="z",
            min_gap=0.10,
            elem_a="roller",
            elem_b="seat_cushion",
            name="leg rollers stay above the seat cushion",
        )
        ctx.expect_origin_gap(
            foot_roller_0,
            leg_roller_0,
            axis="x",
            min_gap=0.18,
            name="front foot rollers sit ahead of the leg rollers",
        )
        ctx.expect_origin_gap(
            seat,
            wheel_0,
            axis="x",
            min_gap=0.60,
            name="wheel set stays at the rear end of the bench",
        )
        rest_back_aabb = ctx.part_element_world_aabb(back, elem="back_cushion")
        rest_seat_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")

    wheel_0_pos = ctx.part_world_position(wheel_0)
    wheel_1_pos = ctx.part_world_position(wheel_1)
    leg_roller_0_pos = ctx.part_world_position(leg_roller_0)
    leg_roller_1_pos = ctx.part_world_position(leg_roller_1)
    foot_roller_0_pos = ctx.part_world_position(foot_roller_0)
    foot_roller_1_pos = ctx.part_world_position(foot_roller_1)
    ctx.check(
        "roller and wheel pairs are mirrored across the centerline",
        wheel_0_pos is not None
        and wheel_1_pos is not None
        and leg_roller_0_pos is not None
        and leg_roller_1_pos is not None
        and foot_roller_0_pos is not None
        and foot_roller_1_pos is not None
        and abs(wheel_0_pos[1] + wheel_1_pos[1]) < 1e-6
        and abs(leg_roller_0_pos[1] + leg_roller_1_pos[1]) < 1e-6
        and abs(foot_roller_0_pos[1] + foot_roller_1_pos[1]) < 1e-6,
        details=(
            f"wheel={wheel_0_pos},{wheel_1_pos} leg={leg_roller_0_pos},{leg_roller_1_pos} "
            f"foot={foot_roller_0_pos},{foot_roller_1_pos}"
        ),
    )

    limits = back_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({back_hinge: limits.upper, seat_hinge: 0.0}):
            raised_back_aabb = ctx.part_element_world_aabb(back, elem="back_cushion")
        ctx.check(
            "back pad raises at upper limit",
            rest_back_aabb is not None
            and raised_back_aabb is not None
            and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.18,
            details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
        )

    seat_limits = seat_hinge.motion_limits
    if seat_limits is not None and seat_limits.upper is not None:
        with ctx.pose({back_hinge: 0.0, seat_hinge: seat_limits.upper}):
            raised_seat_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
        ctx.check(
            "seat front lifts at upper limit",
            rest_seat_aabb is not None
            and raised_seat_aabb is not None
            and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.05,
            details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
