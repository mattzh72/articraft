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


FRAME_TUBE = 0.05
FRAME_RAIL_Y = 0.17
PAD_WIDTH = 0.30
PAD_THICK = 0.06
HINGE_HEIGHT = 0.40

SEAT_HINGE_X = -0.22
BACK_HINGE_X = 0.14
REAR_PIVOT_X = 0.57
ROLLER_X = -0.49
ROLLER_Z = 0.16

SEAT_LEN = 0.36
BACK_LEN = 0.74
LADDER_LEN = 0.24
LADDER_REST_PITCH = 0.65


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_incline_utility_bench")

    model.material("frame", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("upholstery", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("accent", rgba=(0.78, 0.11, 0.11, 1.0))
    model.material("roller_foam", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))

    base = model.part("base")

    # Rectangular floor frame.
    base.visual(
        Box((1.10, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.07, -FRAME_RAIL_Y, FRAME_TUBE / 2.0)),
        material="frame",
        name="side_rail_0",
    )
    base.visual(
        Box((1.10, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.07, FRAME_RAIL_Y, FRAME_TUBE / 2.0)),
        material="frame",
        name="side_rail_1",
    )
    base.visual(
        Box((0.06, 0.39, FRAME_TUBE)),
        origin=Origin(xyz=(-0.44, 0.0, FRAME_TUBE / 2.0)),
        material="frame",
        name="front_cross_rail",
    )
    base.visual(
        Box((0.06, 0.39, FRAME_TUBE)),
        origin=Origin(xyz=(0.04, 0.0, FRAME_TUBE / 2.0)),
        material="frame",
        name="middle_cross_rail",
    )
    base.visual(
        Box((0.06, 0.39, FRAME_TUBE)),
        origin=Origin(xyz=(0.58, 0.0, FRAME_TUBE / 2.0)),
        material="frame",
        name="rear_cross_rail",
    )

    # Seat pedestal and rigid front yoke.
    for idx, y in enumerate((-FRAME_RAIL_Y, FRAME_RAIL_Y)):
        base.visual(
            Box((FRAME_TUBE, FRAME_TUBE, 0.32)),
            origin=Origin(xyz=(SEAT_HINGE_X - 0.03, y, 0.21)),
            material="frame",
            name=f"seat_post_{idx}",
        )
        base.visual(
            Box((0.24, 0.03, 0.03)),
            origin=Origin(xyz=(-0.37, y, 0.25)),
            material="frame",
            name=f"yoke_strut_{idx}",
        )
        base.visual(
            Box((FRAME_TUBE, FRAME_TUBE, 0.35)),
            origin=Origin(xyz=(BACK_HINGE_X, y, 0.225)),
            material="frame",
            name=f"back_post_{idx}",
        )
        base.visual(
            Box((0.39, 0.03, 0.03)),
            origin=Origin(xyz=(-0.055, y, 0.32)),
            material="frame",
            name=f"upper_side_brace_{idx}",
        )
        base.visual(
            Box((FRAME_TUBE, FRAME_TUBE, 0.18)),
            origin=Origin(xyz=(REAR_PIVOT_X, y, 0.14)),
            material="frame",
            name=f"rear_post_{idx}",
        )
        base.visual(
            Box((REAR_PIVOT_X - BACK_HINGE_X, 0.03, 0.03)),
            origin=Origin(xyz=((REAR_PIVOT_X + BACK_HINGE_X) / 2.0, y, 0.23)),
            material="frame",
            name=f"rear_side_brace_{idx}",
        )

    base.visual(
        Box((0.06, 0.34, 0.03)),
        origin=Origin(xyz=(SEAT_HINGE_X, 0.0, HINGE_HEIGHT - 0.015)),
        material="frame",
        name="seat_hinge_bridge",
    )
    base.visual(
        Box((0.06, 0.34, 0.03)),
        origin=Origin(xyz=(BACK_HINGE_X, 0.0, HINGE_HEIGHT - 0.015)),
        material="frame",
        name="back_hinge_bridge",
    )
    base.visual(
        Box((0.06, 0.03, 0.03)),
        origin=Origin(xyz=(REAR_PIVOT_X, -0.14, 0.215)),
        material="frame",
        name="rear_pivot_tab_0",
    )
    base.visual(
        Box((0.06, 0.03, 0.03)),
        origin=Origin(xyz=(REAR_PIVOT_X, 0.14, 0.215)),
        material="frame",
        name="rear_pivot_tab_1",
    )
    base.visual(
        Box((0.04, 0.34, 0.03)),
        origin=Origin(xyz=(ROLLER_X, 0.0, 0.25)),
        material="frame",
        name="yoke_top_bridge",
    )
    base.visual(
        Box((0.04, 0.34, 0.03)),
        origin=Origin(xyz=(ROLLER_X, 0.0, 0.07)),
        material="frame",
        name="yoke_bottom_bridge",
    )
    base.visual(
        Box((0.04, 0.03, 0.18)),
        origin=Origin(xyz=(ROLLER_X, -FRAME_RAIL_Y, 0.16)),
        material="frame",
        name="yoke_arm_0",
    )
    base.visual(
        Box((0.04, 0.03, 0.18)),
        origin=Origin(xyz=(ROLLER_X, FRAME_RAIL_Y, 0.16)),
        material="frame",
        name="yoke_arm_1",
    )
    base.visual(
        Box((0.02, 0.04, 0.18)),
        origin=Origin(xyz=(ROLLER_X, 0.0, 0.16)),
        material="frame",
        name="yoke_center_plate",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.07),
        origin=Origin(xyz=(ROLLER_X, 0.0, ROLLER_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="roller_axle_center",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.01),
        origin=Origin(xyz=(ROLLER_X, -0.15, ROLLER_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="roller_axle_stub_0",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.01),
        origin=Origin(xyz=(ROLLER_X, 0.15, ROLLER_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="roller_axle_stub_1",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.30, 0.22, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, 0.02)),
        material="steel",
        name="seat_tray",
    )
    seat.visual(
        Box((SEAT_LEN, PAD_WIDTH, PAD_THICK)),
        origin=Origin(xyz=(SEAT_LEN / 2.0, 0.0, 0.07)),
        material="upholstery",
        name="seat_pad",
    )
    seat.visual(
        Cylinder(radius=0.018, length=0.24),
        origin=Origin(xyz=(0.03, 0.0, 0.02), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="accent",
        name="seat_hinge_tube",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.62, 0.22, 0.04)),
        origin=Origin(xyz=(0.31, 0.0, 0.02)),
        material="steel",
        name="back_tray",
    )
    backrest.visual(
        Box((BACK_LEN, PAD_WIDTH, PAD_THICK)),
        origin=Origin(xyz=(BACK_LEN / 2.0, 0.0, 0.07)),
        material="upholstery",
        name="back_pad",
    )
    backrest.visual(
        Cylinder(radius=0.018, length=0.26),
        origin=Origin(xyz=(0.03, 0.0, 0.02), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="accent",
        name="back_hinge_tube",
    )
    backrest.visual(
        Box((0.10, 0.02, 0.20)),
        origin=Origin(xyz=(0.14, -0.10, -0.08)),
        material="frame",
        name="support_plate_0",
    )
    backrest.visual(
        Box((0.10, 0.02, 0.20)),
        origin=Origin(xyz=(0.14, 0.10, -0.08)),
        material="frame",
        name="support_plate_1",
    )
    backrest.visual(
        Box((0.08, 0.22, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, -0.15)),
        material="frame",
        name="support_bracket",
    )

    ladder = model.part("ladder")
    ladder.visual(
        Box((LADDER_LEN, 0.03, 0.04)),
        origin=Origin(xyz=(-LADDER_LEN / 2.0, -0.11, 0.0)),
        material="frame",
        name="ladder_rail_0",
    )
    ladder.visual(
        Box((LADDER_LEN, 0.03, 0.04)),
        origin=Origin(xyz=(-LADDER_LEN / 2.0, 0.11, 0.0)),
        material="frame",
        name="ladder_rail_1",
    )
    for idx, rung_x in enumerate((-0.04, -0.09, -0.14, -0.19)):
        ladder.visual(
            Box((0.03, 0.25, 0.03)),
            origin=Origin(xyz=(rung_x, 0.0, 0.0)),
            material="steel",
            name=f"ladder_rung_{idx}",
        )

    roller_0 = model.part("roller_0")
    roller_0.visual(
        Cylinder(radius=0.055, length=0.11),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="roller_foam",
        name="roller_pad",
    )

    roller_1 = model.part("roller_1")
    roller_1.visual(
        Cylinder(radius=0.055, length=0.11),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="roller_foam",
        name="roller_pad",
    )

    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(SEAT_HINGE_X, 0.0, HINGE_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.28, effort=140.0, velocity=1.8),
    )
    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(BACK_HINGE_X, 0.0, HINGE_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=180.0, velocity=1.8),
    )
    model.articulation(
        "ladder_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=ladder,
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, 0.23), rpy=(0.0, LADDER_REST_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=120.0, velocity=1.5),
    )
    model.articulation(
        "roller_spin_0",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=roller_0,
        origin=Origin(xyz=(ROLLER_X, -0.09, ROLLER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=10.0),
    )
    model.articulation(
        "roller_spin_1",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=roller_1,
        origin=Origin(xyz=(ROLLER_X, 0.09, ROLLER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    ladder = object_model.get_part("ladder")
    roller_0 = object_model.get_part("roller_0")
    roller_1 = object_model.get_part("roller_1")

    seat_hinge = object_model.get_articulation("seat_hinge")
    back_hinge = object_model.get_articulation("back_hinge")
    ladder_pivot = object_model.get_articulation("ladder_pivot")

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        positive_elem="back_pad",
        negative_elem="seat_pad",
        max_gap=0.005,
        max_penetration=0.0,
        name="seat and backrest meet cleanly at the hinge line",
    )
    ctx.expect_gap(
        ladder,
        backrest,
        axis="x",
        positive_elem="ladder_rail_0",
        negative_elem="support_plate_0",
        min_gap=0.015,
        max_gap=0.08,
        name="rear ladder stays behind the backrest bracket",
    )
    ctx.expect_overlap(
        ladder,
        backrest,
        axes="z",
        elem_a="ladder_rail_0",
        elem_b="support_plate_0",
        min_overlap=0.10,
        name="ladder spans the bracket support height",
    )
    ctx.expect_gap(
        roller_1,
        roller_0,
        axis="y",
        positive_elem="roller_pad",
        negative_elem="roller_pad",
        min_gap=0.04,
        max_gap=0.07,
        name="front rollers keep the central ankle gap",
    )

    back_limits = back_hinge.motion_limits
    if back_limits is not None and back_limits.upper is not None:
        rest_back = ctx.part_element_world_aabb(backrest, elem="back_pad")
        with ctx.pose({back_hinge: back_limits.upper}):
            raised_back = ctx.part_element_world_aabb(backrest, elem="back_pad")
        ctx.check(
            "backrest rotates upward into incline positions",
            rest_back is not None
            and raised_back is not None
            and raised_back[1][2] > rest_back[1][2] + 0.22,
            details=f"rest={rest_back}, raised={raised_back}",
        )

    seat_limits = seat_hinge.motion_limits
    if seat_limits is not None and seat_limits.upper is not None:
        rest_seat = ctx.part_element_world_aabb(seat, elem="seat_pad")
        with ctx.pose({seat_hinge: seat_limits.upper}):
            tipped_seat = ctx.part_element_world_aabb(seat, elem="seat_pad")
        ctx.check(
            "seat pivots upward from its front hinge",
            rest_seat is not None
            and tipped_seat is not None
            and tipped_seat[1][2] > rest_seat[1][2] + 0.05,
            details=f"rest={rest_seat}, tipped={tipped_seat}",
        )

    ladder_limits = ladder_pivot.motion_limits
    if ladder_limits is not None and ladder_limits.upper is not None:
        rest_rung = ctx.part_element_world_aabb(ladder, elem="ladder_rung_3")
        with ctx.pose({ladder_pivot: ladder_limits.upper}):
            raised_rung = ctx.part_element_world_aabb(ladder, elem="ladder_rung_3")
        ctx.check(
            "ladder rotates upward around the rear lower pivot",
            rest_rung is not None
            and raised_rung is not None
            and raised_rung[1][2] > rest_rung[1][2] + 0.045,
            details=f"rest={rest_rung}, raised={raised_rung}",
        )

    roller_pos_0 = ctx.part_world_position(roller_0)
    roller_pos_1 = ctx.part_world_position(roller_1)
    ctx.check(
        "front rollers share a common axle line on the yoke",
        roller_pos_0 is not None
        and roller_pos_1 is not None
        and abs(roller_pos_0[0] - roller_pos_1[0]) < 1e-6
        and abs(roller_pos_0[2] - roller_pos_1[2]) < 1e-6
        and abs(roller_pos_0[1] + roller_pos_1[1]) < 0.01,
        details=f"roller_0={roller_pos_0}, roller_1={roller_pos_1}",
    )

    return ctx.report()


object_model = build_object_model()
