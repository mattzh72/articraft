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


def _add_wheel(part, *, rubber, hub):
    part.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.046),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=hub,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.052),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=hub,
        name="cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_weight_bench")

    frame_paint = model.material("frame_paint", rgba=(0.15, 0.16, 0.17, 1.0))
    vinyl = model.material("vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    underside = model.material("underside", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip = model.material("grip", rgba=(0.08, 0.08, 0.09, 1.0))
    hub = model.material("hub", rgba=(0.34, 0.36, 0.39, 1.0))

    frame = model.part("frame")
    for y_pos in (-0.205, 0.205):
        frame.visual(
            Box((1.24, 0.050, 0.050)),
            origin=Origin(xyz=(0.0, y_pos, 0.025)),
            material=frame_paint,
            name=f"base_rail_{0 if y_pos < 0.0 else 1}",
        )
    for index, x_pos in enumerate((-0.56, -0.20, 0.10, 0.34, 0.57)):
        frame.visual(
            Box((0.050, 0.360, 0.050)),
            origin=Origin(xyz=(x_pos, 0.0, 0.025)),
            material=frame_paint,
            name=f"crossbar_{index}",
        )

    frame.visual(
        Box((0.88, 0.080, 0.040)),
        origin=Origin(xyz=(-0.02, 0.0, 0.070)),
        material=frame_paint,
        name="center_beam",
    )
    frame.visual(
        Box((0.100, 0.120, 0.240)),
        origin=Origin(xyz=(0.05, 0.0, 0.210)),
        material=frame_paint,
        name="rear_post",
    )
    frame.visual(
        Box((0.060, 0.100, 0.240)),
        origin=Origin(xyz=(0.39, 0.0, 0.210)),
        material=frame_paint,
        name="front_post",
    )
    frame.visual(
        Box((0.500, 0.100, 0.040)),
        origin=Origin(xyz=(0.22, 0.0, 0.350)),
        material=frame_paint,
        name="upper_deck",
    )
    frame.visual(
        Box((0.120, 0.100, 0.068)),
        origin=Origin(xyz=(-0.19, 0.0, 0.058)),
        material=frame_paint,
        name="arm_pivot_block",
    )

    for y_pos in (-0.203, 0.203):
        wheel_name = "wheel_mount_0" if y_pos < 0.0 else "wheel_mount_1"
        frame.visual(
            Box((0.080, 0.070, 0.080)),
            origin=Origin(xyz=(0.570, y_pos, 0.085)),
            material=frame_paint,
            name=wheel_name,
        )

    for y_pos in (-0.085, 0.085):
        frame.visual(
            Box((0.080, 0.030, 0.030)),
            origin=Origin(xyz=(0.550, y_pos, 0.065)),
            material=frame_paint,
            name=f"handle_brace_{0 if y_pos < 0.0 else 1}",
        )

    frame.visual(
        Cylinder(radius=0.017, length=0.200),
        origin=Origin(xyz=(0.510, 0.0, 0.065), rpy=(1.57079632679, 0.0, 0.0)),
        material=grip,
        name="handle",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=underside,
        name="hinge_sleeve",
    )
    backrest.visual(
        Box((0.100, 0.180, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, 0.022)),
        material=underside,
        name="hinge_block",
    )
    backrest.visual(
        Box((0.740, 0.250, 0.014)),
        origin=Origin(xyz=(-0.370, 0.0, 0.021)),
        material=underside,
        name="back_plate",
    )
    backrest.visual(
        Box((0.540, 0.085, 0.040)),
        origin=Origin(xyz=(-0.280, 0.0, 0.032)),
        material=underside,
        name="brace",
    )
    backrest.visual(
        Box((0.760, 0.290, 0.072)),
        origin=Origin(xyz=(-0.380, 0.0, 0.063)),
        material=vinyl,
        name="pad",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=underside,
        name="hinge_sleeve",
    )
    seat.visual(
        Box((0.090, 0.160, 0.040)),
        origin=Origin(xyz=(-0.030, 0.0, 0.022)),
        material=underside,
        name="hinge_block",
    )
    seat.visual(
        Box((0.320, 0.260, 0.014)),
        origin=Origin(xyz=(-0.160, 0.0, 0.021)),
        material=underside,
        name="seat_plate",
    )
    seat.visual(
        Box((0.220, 0.090, 0.040)),
        origin=Origin(xyz=(-0.140, 0.0, 0.032)),
        material=underside,
        name="brace",
    )
    seat.visual(
        Box((0.340, 0.300, 0.072)),
        origin=Origin(xyz=(-0.170, 0.0, 0.063)),
        material=vinyl,
        name="pad",
    )

    support_arm = model.part("support_arm")
    support_arm.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=underside,
        name="pivot_sleeve",
    )
    for y_pos in (-0.040, 0.040):
        support_arm.visual(
            Box((0.300, 0.032, 0.032)),
            origin=Origin(xyz=(-0.150, y_pos, 0.018)),
            material=underside,
            name=f"rail_{0 if y_pos < 0.0 else 1}",
        )
    support_arm.visual(
        Box((0.050, 0.140, 0.036)),
        origin=Origin(xyz=(-0.285, 0.0, 0.036)),
        material=underside,
        name="saddle",
    )

    wheel_0 = model.part("wheel_0")
    _add_wheel(wheel_0, rubber=rubber, hub=hub)

    wheel_1 = model.part("wheel_1")
    _add_wheel(wheel_1, rubber=rubber, hub=hub)

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.05, 0.0, 0.388)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.18),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.39, 0.0, 0.388)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.30),
    )
    model.articulation(
        "support_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.19, 0.0, 0.108)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.4, lower=0.0, upper=1.10),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(0.610, -0.264, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(0.610, 0.264, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_arm = object_model.get_part("support_arm")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    backrest_hinge = object_model.get_articulation("backrest_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    support_arm_hinge = object_model.get_articulation("support_arm_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        positive_elem="hinge_sleeve",
        negative_elem="upper_deck",
        min_gap=-1e-6,
        max_gap=0.020,
        name="seat hinge stays seated on the upper deck",
    )
    ctx.expect_gap(
        backrest,
        frame,
        axis="z",
        positive_elem="hinge_sleeve",
        negative_elem="upper_deck",
        min_gap=-1e-6,
        max_gap=0.020,
        name="backrest hinge stays seated on the upper deck",
    )
    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        positive_elem="pad",
        negative_elem="pad",
        min_gap=None,
        max_gap=0.040,
        max_penetration=0.0,
        name="split pads meet closely at the junction",
    )

    handle_aabb = ctx.part_element_world_aabb(frame, elem="handle")
    wheel_0_pos = ctx.part_world_position(wheel_0)
    wheel_1_pos = ctx.part_world_position(wheel_1)
    handle_between_wheels = (
        handle_aabb is not None
        and wheel_0_pos is not None
        and wheel_1_pos is not None
        and handle_aabb[0][1] > min(wheel_0_pos[1], wheel_1_pos[1])
        and handle_aabb[1][1] < max(wheel_0_pos[1], wheel_1_pos[1])
        and handle_aabb[0][0] < wheel_0_pos[0]
        and handle_aabb[0][0] < wheel_1_pos[0]
    )
    ctx.check(
        "front carry handle sits between the transport wheels",
        handle_between_wheels,
        details=f"handle_aabb={handle_aabb}, wheel_0={wheel_0_pos}, wheel_1={wheel_1_pos}",
    )

    backrest_rest_aabb = ctx.part_world_aabb(backrest)
    seat_rest_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({backrest_hinge: 0.75, seat_hinge: 0.18, support_arm_hinge: 0.95}):
        backrest_incline_aabb = ctx.part_world_aabb(backrest)
        seat_incline_aabb = ctx.part_world_aabb(seat)
        ctx.expect_gap(
            backrest,
            support_arm,
            axis="z",
            positive_elem="brace",
            negative_elem="saddle",
            min_gap=0.0,
            max_gap=0.030,
            name="rear support arm rises beneath the backrest brace",
        )
        ctx.expect_overlap(
            backrest,
            support_arm,
            axes="x",
            elem_a="brace",
            elem_b="saddle",
            min_overlap=0.040,
            name="support arm remains under the backrest support zone",
        )

    backrest_lifts = (
        backrest_rest_aabb is not None
        and backrest_incline_aabb is not None
        and backrest_incline_aabb[1][2] > backrest_rest_aabb[1][2] + 0.22
    )
    ctx.check(
        "backrest lifts into an incline position",
        backrest_lifts,
        details=f"rest={backrest_rest_aabb}, incline={backrest_incline_aabb}",
    )

    seat_lifts = (
        seat_rest_aabb is not None
        and seat_incline_aabb is not None
        and seat_incline_aabb[1][2] > seat_rest_aabb[1][2] + 0.04
    )
    ctx.check(
        "seat pivots upward at the front support",
        seat_lifts,
        details=f"rest={seat_rest_aabb}, incline={seat_incline_aabb}",
    )

    wheel_0_rest = ctx.part_world_position(wheel_0)
    wheel_1_rest = ctx.part_world_position(wheel_1)
    with ctx.pose({wheel_0_spin: 1.7, wheel_1_spin: -2.1}):
        wheel_0_turn = ctx.part_world_position(wheel_0)
        wheel_1_turn = ctx.part_world_position(wheel_1)

    wheels_spin_in_place = (
        wheel_0_rest is not None
        and wheel_0_turn is not None
        and wheel_1_rest is not None
        and wheel_1_turn is not None
        and max(abs(a - b) for a, b in zip(wheel_0_rest, wheel_0_turn)) < 1e-6
        and max(abs(a - b) for a, b in zip(wheel_1_rest, wheel_1_turn)) < 1e-6
    )
    ctx.check(
        "transport wheels rotate about fixed axle centers",
        wheels_spin_in_place,
        details=(
            f"wheel_0_rest={wheel_0_rest}, wheel_0_turn={wheel_0_turn}, "
            f"wheel_1_rest={wheel_1_rest}, wheel_1_turn={wheel_1_turn}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
