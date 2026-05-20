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
    model = ArticulatedObject(name="flat_incline_utility_bench")

    steel = model.material("steel", rgba=(0.18, 0.19, 0.21, 1.0))
    upholstery = model.material("upholstery", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    accent = model.material("accent", rgba=(0.72, 0.11, 0.10, 1.0))

    base = model.part("base")
    # Floor feet and main frame.
    base.visual(
        Box((0.24, 0.56, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, 0.025)),
        material=steel,
        name="front_crossmember",
    )
    base.visual(
        Box((0.12, 0.66, 0.05)),
        origin=Origin(xyz=(1.11, 0.0, 0.025)),
        material=steel,
        name="rear_crossmember",
    )
    for index, y in enumerate((-0.22, 0.22)):
        base.visual(
            Box((1.00, 0.05, 0.05)),
            origin=Origin(xyz=(0.58, y, 0.25)),
            material=steel,
            name=f"side_rail_{index}",
        )
        base.visual(
            Box((0.05, 0.05, 0.45)),
            origin=Origin(xyz=(0.16, y, 0.225)),
            material=steel,
            name=f"front_upright_{index}",
        )
        base.visual(
            Box((0.05, 0.05, 0.27)),
            origin=Origin(xyz=(0.96, y, 0.135)),
            material=steel,
            name=f"rear_upright_{index}",
        )
        base.visual(
            Box((0.08, 0.05, 0.10)),
            origin=Origin(xyz=(0.28, y, 0.325)),
            material=steel,
            name=f"seat_hinge_tab_{index}",
        )
        base.visual(
            Box((0.08, 0.05, 0.06)),
            origin=Origin(xyz=(0.97, y, 0.18)),
            material=steel,
            name=f"ladder_pivot_tab_{index}",
        )
    base.visual(
        Box((0.08, 0.44, 0.05)),
        origin=Origin(xyz=(0.28, 0.0, 0.325)),
        material=steel,
        name="seat_hinge_crossmember",
    )
    base.visual(
        Box((0.08, 0.40, 0.05)),
        origin=Origin(xyz=(0.58, 0.0, 0.275)),
        material=steel,
        name="center_crossmember",
    )
    base.visual(
        Box((0.04, 0.40, 0.05)),
        origin=Origin(xyz=(0.95, 0.0, 0.18)),
        material=steel,
        name="ladder_pivot_crossmember",
    )
    # Wheel axle and transport wheels.
    base.visual(
        Cylinder(radius=0.012, length=0.62),
        origin=Origin(xyz=(0.05, 0.0, 0.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_axle",
    )
    for index, y in enumerate((-0.30, 0.30)):
        base.visual(
            Box((0.05, 0.04, 0.09)),
            origin=Origin(xyz=(0.05, y * 0.68, 0.075)),
            material=steel,
            name=f"wheel_bracket_{index}",
        )
        base.visual(
            Cylinder(radius=0.07, length=0.04),
            origin=Origin(xyz=(0.05, y, 0.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"wheel_{index}",
        )
    # Front carry handle welded to the front crossmember.
    base.visual(
        Cylinder(radius=0.015, length=0.24),
        origin=Origin(xyz=(-0.08, 0.0, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="front_handle_grip",
    )
    for index, y in enumerate((-0.12, 0.12)):
        base.visual(
            Cylinder(radius=0.015, length=0.13),
            origin=Origin(xyz=(-0.015, y, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=accent,
            name=f"front_handle_leg_{index}",
        )
    # Small rear gussets to visually support the upper frame.
    for index, y in enumerate((-0.19, 0.19)):
        base.visual(
            Box((0.12, 0.05, 0.16)),
            origin=Origin(xyz=(1.05, y, 0.10)),
            material=steel,
            name=f"rear_gusset_{index}",
        )

    seat = model.part("seat")
    seat.visual(
        Box((0.34, 0.30, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.06)),
        material=upholstery,
        name="seat_pad",
    )
    seat.visual(
        Box((0.35, 0.27, 0.02)),
        origin=Origin(xyz=(0.18, 0.0, 0.02)),
        material=steel,
        name="seat_pan",
    )
    for index, y in enumerate((-0.11, 0.11)):
        seat.visual(
            Box((0.30, 0.03, 0.05)),
            origin=Origin(xyz=(0.19, y, 0.0)),
            material=steel,
            name=f"seat_stringer_{index}",
        )
    seat.visual(
        Cylinder(radius=0.018, length=0.24),
        origin=Origin(xyz=(0.02, 0.0, 0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="seat_hinge_barrel",
    )
    seat.visual(
        Box((0.05, 0.18, 0.05)),
        origin=Origin(xyz=(0.04, 0.0, 0.02)),
        material=steel,
        name="seat_hinge_bridge",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.80, 0.28, 0.06)),
        origin=Origin(xyz=(0.42, 0.0, 0.06)),
        material=upholstery,
        name="back_pad",
    )
    backrest.visual(
        Box((0.81, 0.24, 0.02)),
        origin=Origin(xyz=(0.42, 0.0, 0.02)),
        material=steel,
        name="back_pan",
    )
    for index, y in enumerate((-0.105, 0.105)):
        backrest.visual(
            Box((0.26, 0.03, 0.12)),
            origin=Origin(xyz=(0.17, y, -0.03)),
            material=steel,
            name=f"support_arm_{index}",
        )
    backrest.visual(
        Box((0.08, 0.20, 0.04)),
        origin=Origin(xyz=(0.30, 0.0, -0.03)),
        material=steel,
        name="support_catch",
    )
    backrest.visual(
        Box((0.085, 0.20, 0.05)),
        origin=Origin(xyz=(0.0375, 0.0, 0.015)),
        material=steel,
        name="back_hinge_bridge",
    )

    support_ladder = model.part("support_ladder")
    for index, y in enumerate((-0.11, 0.11)):
        support_ladder.visual(
            Box((0.04, 0.03, 0.24)),
            origin=Origin(xyz=(0.0, y, 0.12)),
            material=steel,
            name=f"ladder_rail_{index}",
        )
    rung_z = (0.10, 0.15, 0.20)
    for index, z in enumerate(rung_z):
        support_ladder.visual(
            Box((0.05, 0.24, 0.03)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=f"rung_{index}",
        )
    support_ladder.visual(
        Box((0.06, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=steel,
        name="top_rung",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.28, 0.0, 0.355)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.5,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_support_ladder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support_ladder,
        origin=Origin(xyz=(1.01, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.4,
            lower=-0.25,
            upper=0.60,
        ),
    )

    return model


def _aabb_extents(aabb):
    if aabb is None:
        return None
    return aabb[0], aabb[1]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    support_ladder = object_model.get_part("support_ladder")
    seat_hinge = object_model.get_articulation("base_to_seat")
    backrest_hinge = object_model.get_articulation("seat_to_backrest")
    ladder_hinge = object_model.get_articulation("base_to_support_ladder")

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        positive_elem="back_pad",
        negative_elem="seat_pad",
        min_gap=0.0,
        max_gap=0.05,
        name="backrest pad clears the seat pad seam",
    )
    ctx.expect_gap(
        support_ladder,
        backrest,
        axis="x",
        positive_elem="top_rung",
        negative_elem="support_catch",
        min_gap=0.0,
        max_gap=0.025,
        name="ladder stays just behind the backrest catch",
    )

    front_crossmember_aabb = ctx.part_element_world_aabb(base, elem="front_crossmember")
    handle_aabb = ctx.part_element_world_aabb(base, elem="front_handle_grip")
    wheel_0_aabb = ctx.part_element_world_aabb(base, elem="wheel_0")
    wheel_1_aabb = ctx.part_element_world_aabb(base, elem="wheel_1")

    handle_ok = False
    if (
        front_crossmember_aabb is not None
        and handle_aabb is not None
        and wheel_0_aabb is not None
        and wheel_1_aabb is not None
    ):
        front_crossmember_min, _ = _aabb_extents(front_crossmember_aabb)
        handle_min, handle_max = _aabb_extents(handle_aabb)
        wheel_0_min, wheel_0_max = _aabb_extents(wheel_0_aabb)
        wheel_1_min, wheel_1_max = _aabb_extents(wheel_1_aabb)
        inner_negative_y = wheel_0_max[1] if wheel_0_max[1] < 0.0 else wheel_1_max[1]
        inner_positive_y = wheel_0_min[1] if wheel_0_min[1] > 0.0 else wheel_1_min[1]
        handle_ok = (
            handle_max[0] < front_crossmember_min[0] - 0.01
            and handle_min[1] > inner_negative_y
            and handle_max[1] < inner_positive_y
        )
    ctx.check(
        "front handle projects ahead of the frame between the wheels",
        handle_ok,
        details=(
            f"front_crossmember={front_crossmember_aabb}, "
            f"handle={handle_aabb}, wheel_0={wheel_0_aabb}, wheel_1={wheel_1_aabb}"
        ),
    )

    backrest_rest = ctx.part_element_world_aabb(backrest, elem="back_pad")
    backrest_raised = None
    with ctx.pose({backrest_hinge: 1.0}):
        backrest_raised = ctx.part_element_world_aabb(backrest, elem="back_pad")
    backrest_motion_ok = (
        backrest_rest is not None
        and backrest_raised is not None
        and backrest_raised[1][2] > backrest_rest[1][2] + 0.20
    )
    ctx.check(
        "backrest rotates upward from the seat junction",
        backrest_motion_ok,
        details=f"rest={backrest_rest}, raised={backrest_raised}",
    )

    seat_rest = ctx.part_element_world_aabb(seat, elem="seat_pad")
    seat_tilted = None
    with ctx.pose({seat_hinge: 0.20}):
        seat_tilted = ctx.part_element_world_aabb(seat, elem="seat_pad")
    seat_motion_ok = (
        seat_rest is not None
        and seat_tilted is not None
        and seat_tilted[1][2] > seat_rest[1][2] + 0.03
    )
    ctx.check(
        "seat pivots on its separate front hinge",
        seat_motion_ok,
        details=f"rest={seat_rest}, tilted={seat_tilted}",
    )

    ladder_rest = ctx.part_element_world_aabb(support_ladder, elem="top_rung")
    ladder_swung = None
    with ctx.pose({ladder_hinge: 0.45}):
        ladder_swung = ctx.part_element_world_aabb(support_ladder, elem="top_rung")
    ladder_motion_ok = (
        ladder_rest is not None
        and ladder_swung is not None
        and ladder_swung[0][0] < ladder_rest[0][0] - 0.08
    )
    ctx.check(
        "rear ladder support swings forward from its lower rear pivot",
        ladder_motion_ok,
        details=f"rest={ladder_rest}, swung={ladder_swung}",
    )

    return ctx.report()


object_model = build_object_model()
