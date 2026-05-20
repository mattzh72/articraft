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
    model = ArticulatedObject(name="decline_bench")

    model.material("frame_steel", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("upholstery", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("board", rgba=(0.19, 0.19, 0.20, 1.0))
    model.material("roller_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("wheel_rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.20, 0.54, 0.06)),
        origin=Origin(xyz=(0.60, 0.0, 0.03)),
        material="frame_steel",
        name="rear_foot",
    )
    frame.visual(
        Box((0.24, 0.48, 0.06)),
        origin=Origin(xyz=(-0.49, 0.0, 0.03)),
        material="frame_steel",
        name="front_foot",
    )
    frame.visual(
        Box((0.96, 0.10, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, 0.28)),
        material="frame_steel",
        name="main_spine",
    )
    frame.visual(
        Box((0.12, 0.10, 0.28)),
        origin=Origin(xyz=(-0.39, 0.0, 0.17)),
        material="frame_steel",
        name="front_post",
    )
    frame.visual(
        Box((0.12, 0.10, 0.28)),
        origin=Origin(xyz=(0.55, 0.0, 0.17)),
        material="frame_steel",
        name="rear_post",
    )
    frame.visual(
        Box((0.08, 0.14, 0.08)),
        origin=Origin(xyz=(-0.28, 0.0, 0.35)),
        material="frame_steel",
        name="seat_hinge_block",
    )
    frame.visual(
        Box((0.08, 0.14, 0.08)),
        origin=Origin(xyz=(0.00, 0.0, 0.35)),
        material="frame_steel",
        name="backrest_hinge_block",
    )
    frame.visual(
        Box((0.10, 0.05, 0.31)),
        origin=Origin(xyz=(-0.56, 0.0, 0.185)),
        material="frame_steel",
        name="front_roller_post",
    )
    frame.visual(
        Box((0.08, 0.08, 0.08)),
        origin=Origin(xyz=(-0.56, 0.0, 0.38)),
        material="frame_steel",
        name="front_roller_axle",
    )
    frame.visual(
        Box((0.08, 0.12, 0.10)),
        origin=Origin(xyz=(0.71, 0.0, 0.08)),
        material="frame_steel",
        name="rear_wheel_bracket",
    )
    frame.visual(
        Box((0.04, 0.445, 0.04)),
        origin=Origin(xyz=(0.75, 0.0, 0.07)),
        material="frame_steel",
        name="rear_axle",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.27, 0.28, 0.028)),
        origin=Origin(xyz=(0.135, 0.0, 0.014)),
        material="board",
        name="seat_board",
    )
    seat.visual(
        Box((0.28, 0.30, 0.062)),
        origin=Origin(xyz=(0.14, 0.0, 0.059)),
        material="upholstery",
        name="seat_cushion",
    )
    seat.visual(
        Box((0.14, 0.08, 0.04)),
        origin=Origin(xyz=(0.14, 0.0, -0.02)),
        material="frame_steel",
        name="seat_support",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.84, 0.29, 0.028)),
        origin=Origin(xyz=(0.42, 0.0, 0.014)),
        material="board",
        name="backrest_board",
    )
    backrest.visual(
        Box((0.85, 0.31, 0.062)),
        origin=Origin(xyz=(0.425, 0.0, 0.059)),
        material="upholstery",
        name="backrest_cushion",
    )
    backrest.visual(
        Box((0.28, 0.08, 0.04)),
        origin=Origin(xyz=(0.19, 0.0, -0.02)),
        material="frame_steel",
        name="backrest_support",
    )

    for name in ("leg_roller_0", "leg_roller_1"):
        roller = model.part(name)
        roller.visual(
            Cylinder(radius=0.055, length=0.14),
            origin=Origin(rpy=(-1.5707963267948966, 0.0, 0.0)),
            material="roller_vinyl",
            name="roller_body",
        )

    for name in ("wheel_0", "wheel_1"):
        wheel = model.part(name)
        wheel.visual(
            Cylinder(radius=0.045, length=0.035),
            origin=Origin(rpy=(-1.5707963267948966, 0.0, 0.0)),
            material="wheel_rubber",
            name="wheel_body",
        )

    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.28, 0.0, 0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=120.0, velocity=1.5),
    )
    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=180.0, velocity=1.2),
    )
    model.articulation(
        "leg_roller_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=model.get_part("leg_roller_0"),
        origin=Origin(xyz=(-0.56, 0.11, 0.38)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )
    model.articulation(
        "leg_roller_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=model.get_part("leg_roller_1"),
        origin=Origin(xyz=(-0.56, -0.11, 0.38)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=model.get_part("wheel_0"),
        origin=Origin(xyz=(0.75, 0.24, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=12.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=model.get_part("wheel_1"),
        origin=Origin(xyz=(0.75, -0.24, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    leg_roller_0 = object_model.get_part("leg_roller_0")
    leg_roller_1 = object_model.get_part("leg_roller_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    seat_hinge = object_model.get_articulation("seat_hinge")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    leg_roller_0_spin = object_model.get_articulation("leg_roller_0_spin")
    leg_roller_1_spin = object_model.get_articulation("leg_roller_1_spin")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        positive_elem="backrest_cushion",
        negative_elem="seat_cushion",
        max_gap=0.002,
        max_penetration=0.0,
        name="seat and backrest meet cleanly at the hinge line",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="y",
        elem_a="backrest_cushion",
        elem_b="seat_cushion",
        min_overlap=0.29,
        name="seat and backrest share the same centered bench width",
    )

    ctx.expect_gap(
        leg_roller_0,
        frame,
        axis="y",
        positive_elem="roller_body",
        negative_elem="front_roller_axle",
        max_gap=0.001,
        max_penetration=1e-6,
        name="outer leg roller seats against the support axle",
    )
    ctx.expect_gap(
        frame,
        leg_roller_1,
        axis="y",
        positive_elem="front_roller_axle",
        negative_elem="roller_body",
        max_gap=0.001,
        max_penetration=1e-6,
        name="inner leg roller seats against the support axle",
    )
    ctx.expect_gap(
        wheel_0,
        frame,
        axis="y",
        positive_elem="wheel_body",
        negative_elem="rear_axle",
        max_gap=0.001,
        max_penetration=1e-6,
        name="outer transport wheel seats against the rear axle",
    )
    ctx.expect_gap(
        frame,
        wheel_1,
        axis="y",
        positive_elem="rear_axle",
        negative_elem="wheel_body",
        max_gap=0.001,
        max_penetration=1e-6,
        name="inner transport wheel seats against the rear axle",
    )

    ctx.check(
        "continuous rolling joints use horizontal axle axes",
        leg_roller_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and leg_roller_1_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_1_spin.articulation_type == ArticulationType.CONTINUOUS
        and leg_roller_0_spin.axis == (0.0, 1.0, 0.0)
        and leg_roller_1_spin.axis == (0.0, 1.0, 0.0)
        and wheel_0_spin.axis == (0.0, 1.0, 0.0)
        and wheel_1_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"roller_axes={[leg_roller_0_spin.axis, leg_roller_1_spin.axis]}, "
            f"wheel_axes={[wheel_0_spin.axis, wheel_1_spin.axis]}"
        ),
    )

    seat_upper = seat_hinge.motion_limits.upper if seat_hinge.motion_limits is not None else None
    back_upper = backrest_hinge.motion_limits.upper if backrest_hinge.motion_limits is not None else None

    rest_seat_aabb = ctx.part_world_aabb(seat)
    rest_backrest_aabb = ctx.part_world_aabb(backrest)

    if seat_upper is not None:
        with ctx.pose({seat_hinge: seat_upper}):
            raised_seat_aabb = ctx.part_world_aabb(seat)
            ctx.expect_gap(
                seat,
                frame,
                axis="z",
                positive_elem="seat_board",
                negative_elem="main_spine",
                min_gap=0.03,
                name="raised seat clears the center spine",
            )
        ctx.check(
            "seat raises at its rear edge",
            rest_seat_aabb is not None
            and raised_seat_aabb is not None
            and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.04,
            details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
        )

    if back_upper is not None:
        with ctx.pose({backrest_hinge: back_upper}):
            raised_backrest_aabb = ctx.part_world_aabb(backrest)
            ctx.expect_gap(
                backrest,
                frame,
                axis="z",
                positive_elem="backrest_board",
                negative_elem="main_spine",
                min_gap=0.03,
                name="raised backrest clears the center spine",
            )
        ctx.check(
            "backrest opens upward",
            rest_backrest_aabb is not None
            and raised_backrest_aabb is not None
            and raised_backrest_aabb[1][2] > rest_backrest_aabb[1][2] + 0.18,
            details=f"rest={rest_backrest_aabb}, raised={raised_backrest_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
