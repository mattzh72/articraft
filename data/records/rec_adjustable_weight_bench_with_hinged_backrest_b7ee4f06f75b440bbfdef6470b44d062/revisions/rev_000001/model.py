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


def _x_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _y_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    frame_paint = model.material("frame_paint", rgba=(0.15, 0.16, 0.17, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.09, 0.09, 0.10, 1.0))
    pad_base = model.material("pad_base", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    roller_foam = model.material("roller_foam", rgba=(0.16, 0.16, 0.18, 1.0))

    center_frame = model.part("center_frame")
    center_frame.visual(
        Cylinder(radius=0.023, length=0.98),
        origin=_x_origin((0.03, 0.19, 0.09)),
        material=frame_paint,
        name="rail_0",
    )
    center_frame.visual(
        Cylinder(radius=0.023, length=0.98),
        origin=_x_origin((0.03, -0.19, 0.09)),
        material=frame_paint,
        name="rail_1",
    )
    center_frame.visual(
        Cylinder(radius=0.023, length=0.40),
        origin=_y_origin((0.50, 0.0, 0.09)),
        material=frame_paint,
        name="front_crossbar",
    )
    center_frame.visual(
        Cylinder(radius=0.023, length=0.40),
        origin=_y_origin((-0.40, 0.0, 0.09)),
        material=frame_paint,
        name="rear_crossbar",
    )
    center_frame.visual(
        Cylinder(radius=0.023, length=0.40),
        origin=_y_origin((-0.02, 0.0, 0.09)),
        material=frame_paint,
        name="center_crossbar",
    )
    center_frame.visual(
        Box((0.20, 0.26, 0.08)),
        origin=Origin(xyz=(-0.02, 0.0, 0.13)),
        material=frame_paint,
        name="hinge_beam",
    )
    center_frame.visual(
        Box((0.08, 0.22, 0.04)),
        origin=Origin(xyz=(-0.43, 0.0, 0.11)),
        material=frame_paint,
        name="rear_pivot_bridge",
    )
    center_frame.visual(
        Box((0.22, 0.12, 0.206)),
        origin=Origin(xyz=(0.14, 0.0, 0.223)),
        material=frame_paint,
        name="seat_support_pedestal",
    )
    center_frame.visual(
        Box((0.10, 0.18, 0.03)),
        origin=Origin(xyz=(0.20, 0.0, 0.338)),
        material=frame_paint,
        name="seat_stop",
    )
    center_frame.visual(
        Box((0.05, 0.02, 0.20)),
        origin=Origin(xyz=(0.05, 0.113, 0.26)),
        material=frame_paint,
        name="seat_hinge_post_0",
    )
    center_frame.visual(
        Box((0.05, 0.02, 0.20)),
        origin=Origin(xyz=(0.05, -0.113, 0.26)),
        material=frame_paint,
        name="seat_hinge_post_1",
    )
    center_frame.visual(
        Box((0.05, 0.02, 0.22)),
        origin=Origin(xyz=(0.00, 0.113, 0.26)),
        material=frame_paint,
        name="back_hinge_post_0",
    )
    center_frame.visual(
        Box((0.05, 0.02, 0.22)),
        origin=Origin(xyz=(0.00, -0.113, 0.26)),
        material=frame_paint,
        name="back_hinge_post_1",
    )
    center_frame.visual(
        Box((0.05, 0.026, 0.14)),
        origin=Origin(xyz=(-0.43, 0.113, 0.16)),
        material=frame_paint,
        name="rear_link_post_0",
    )
    center_frame.visual(
        Box((0.05, 0.026, 0.14)),
        origin=Origin(xyz=(-0.43, -0.113, 0.16)),
        material=frame_paint,
        name="rear_link_post_1",
    )
    center_frame.visual(
        Cylinder(radius=0.022, length=0.46),
        origin=_y_origin((-0.46, 0.0, 0.03)),
        material=frame_paint,
        name="rear_foot",
    )
    center_frame.visual(
        Box((0.05, 0.05, 0.12)),
        origin=Origin(xyz=(-0.46, 0.19, 0.06)),
        material=frame_paint,
        name="rear_leg_0",
    )
    center_frame.visual(
        Box((0.05, 0.05, 0.12)),
        origin=Origin(xyz=(-0.46, -0.19, 0.06)),
        material=frame_paint,
        name="rear_leg_1",
    )
    center_frame.visual(
        Box((0.08, 0.12, 0.06)),
        origin=Origin(xyz=(0.48, 0.0, 0.13)),
        material=frame_paint,
        name="front_yoke_body",
    )
    center_frame.visual(
        Box((0.04, 0.03, 0.17)),
        origin=Origin(xyz=(0.50, 0.15, 0.172)),
        material=frame_paint,
        name="yoke_upright_0",
    )
    center_frame.visual(
        Box((0.04, 0.03, 0.17)),
        origin=Origin(xyz=(0.50, -0.15, 0.172)),
        material=frame_paint,
        name="yoke_upright_1",
    )
    center_frame.visual(
        Cylinder(radius=0.009, length=0.54),
        origin=_y_origin((0.52, 0.0, 0.055)),
        material=steel,
        name="wheel_axle",
    )
    center_frame.visual(
        Cylinder(radius=0.009, length=0.30),
        origin=_y_origin((0.50, 0.0, 0.255)),
        material=steel,
        name="roller_axle",
    )
    center_frame.visual(
        Box((0.05, 0.03, 0.05)),
        origin=Origin(xyz=(0.50, 0.215, 0.055)),
        material=frame_paint,
        name="wheel_mount_0",
    )
    center_frame.visual(
        Box((0.05, 0.03, 0.05)),
        origin=Origin(xyz=(0.50, -0.215, 0.055)),
        material=frame_paint,
        name="wheel_mount_1",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.82, 0.29, 0.024)),
        origin=Origin(xyz=(-0.395, 0.0, 0.042)),
        material=pad_base,
        name="back_panel",
    )
    backrest.visual(
        Box((0.80, 0.28, 0.052)),
        origin=Origin(xyz=(-0.395, 0.0, 0.068)),
        material=pad_vinyl,
        name="back_cushion",
    )
    backrest.visual(
        Box((0.46, 0.03, 0.026)),
        origin=Origin(xyz=(-0.23, 0.09, 0.018)),
        material=frame_paint,
        name="back_support_0",
    )
    backrest.visual(
        Box((0.46, 0.03, 0.026)),
        origin=Origin(xyz=(-0.23, -0.09, 0.018)),
        material=frame_paint,
        name="back_support_1",
    )
    backrest.visual(
        Box((0.03, 0.026, 0.06)),
        origin=Origin(xyz=(-0.005, 0.09, 0.012)),
        material=frame_paint,
        name="back_hinge_lug_0",
    )
    backrest.visual(
        Box((0.03, 0.026, 0.06)),
        origin=Origin(xyz=(-0.005, -0.09, 0.012)),
        material=frame_paint,
        name="back_hinge_lug_1",
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.22),
        origin=_y_origin((-0.275, 0.0, 0.03)),
        material=steel,
        name="support_bar",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.36, 0.28, 0.024)),
        origin=Origin(xyz=(0.165, 0.0, 0.042)),
        material=pad_base,
        name="seat_panel",
    )
    seat.visual(
        Box((0.35, 0.27, 0.050)),
        origin=Origin(xyz=(0.165, 0.0, 0.067)),
        material=pad_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.22, 0.03, 0.026)),
        origin=Origin(xyz=(0.12, 0.085, 0.018)),
        material=frame_paint,
        name="seat_support_0",
    )
    seat.visual(
        Box((0.22, 0.03, 0.026)),
        origin=Origin(xyz=(0.12, -0.085, 0.018)),
        material=frame_paint,
        name="seat_support_1",
    )
    seat.visual(
        Box((0.03, 0.026, 0.06)),
        origin=Origin(xyz=(0.005, 0.085, 0.012)),
        material=frame_paint,
        name="seat_hinge_lug_0",
    )
    seat.visual(
        Box((0.03, 0.026, 0.06)),
        origin=Origin(xyz=(0.005, -0.085, 0.012)),
        material=frame_paint,
        name="seat_hinge_lug_1",
    )
    seat.visual(
        Cylinder(radius=0.013, length=0.20),
        origin=_y_origin((0.21, 0.0, 0.006)),
        material=steel,
        name="seat_front_tube",
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        Cylinder(radius=0.014, length=0.20),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    rear_support.visual(
        Box((0.26, 0.018, 0.045)),
        origin=Origin(xyz=(0.085, 0.085, 0.10), rpy=(0.0, -0.88, 0.0)),
        material=frame_paint,
        name="rear_strut_0",
    )
    rear_support.visual(
        Box((0.26, 0.018, 0.045)),
        origin=Origin(xyz=(0.085, -0.085, 0.10), rpy=(0.0, -0.88, 0.0)),
        material=frame_paint,
        name="rear_strut_1",
    )
    rear_support.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=_y_origin((0.18, 0.0, 0.208)),
        material=steel,
        name="rear_saddle",
    )
    rear_support.visual(
        Box((0.05, 0.14, 0.012)),
        origin=Origin(xyz=(0.195, 0.0, 0.220)),
        material=frame_paint,
        name="rear_plate",
    )

    front_wheel_0 = model.part("front_wheel_0")
    front_wheel_0.visual(
        Cylinder(radius=0.058, length=0.036),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    front_wheel_0.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )

    front_wheel_1 = model.part("front_wheel_1")
    front_wheel_1.visual(
        Cylinder(radius=0.058, length=0.036),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    front_wheel_1.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )

    foot_roller_0 = model.part("foot_roller_0")
    foot_roller_0.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=roller_foam,
        name="roller_body",
    )
    foot_roller_0.visual(
        Cylinder(radius=0.018, length=0.108),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="roller_core",
    )

    foot_roller_1 = model.part("foot_roller_1")
    foot_roller_1.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=roller_foam,
        name="roller_body",
    )
    foot_roller_1.visual(
        Cylinder(radius=0.018, length=0.108),
        origin=_y_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="roller_core",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=seat,
        origin=Origin(xyz=(0.04, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.6, lower=0.0, upper=0.42),
    )
    model.articulation(
        "rear_support_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=rear_support,
        origin=Origin(xyz=(-0.43, 0.0, 0.15)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.6, lower=-0.35, upper=0.45),
    )
    model.articulation(
        "front_wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=front_wheel_0,
        origin=Origin(xyz=(0.52, 0.25, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "front_wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=front_wheel_1,
        origin=Origin(xyz=(0.52, -0.25, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "foot_roller_spin_0",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=foot_roller_0,
        origin=Origin(xyz=(0.50, 0.085, 0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )
    model.articulation(
        "foot_roller_spin_1",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=foot_roller_1,
        origin=Origin(xyz=(0.50, -0.085, 0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_frame = object_model.get_part("center_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    rear_support = object_model.get_part("rear_support")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    foot_roller_0 = object_model.get_part("foot_roller_0")
    foot_roller_1 = object_model.get_part("foot_roller_1")

    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    rear_support_hinge = object_model.get_articulation("rear_support_hinge")

    ctx.allow_overlap(
        center_frame,
        front_wheel_0,
        elem_a="wheel_axle",
        elem_b="wheel_tire",
        reason="The transport wheel is simplified as a solid wheel proxy spinning around the fixed axle.",
    )
    ctx.allow_overlap(
        center_frame,
        front_wheel_0,
        elem_a="wheel_axle",
        elem_b="wheel_hub",
        reason="The wheel hub is intentionally centered on the fixed transport axle.",
    )
    ctx.allow_overlap(
        center_frame,
        front_wheel_1,
        elem_a="wheel_axle",
        elem_b="wheel_tire",
        reason="The transport wheel is simplified as a solid wheel proxy spinning around the fixed axle.",
    )
    ctx.allow_overlap(
        center_frame,
        front_wheel_1,
        elem_a="wheel_axle",
        elem_b="wheel_hub",
        reason="The wheel hub is intentionally centered on the fixed transport axle.",
    )
    ctx.allow_overlap(
        center_frame,
        foot_roller_0,
        elem_a="roller_axle",
        elem_b="roller_body",
        reason="The foam ankle roller is represented as a solid sleeve proxy spinning around the shared axle.",
    )
    ctx.allow_overlap(
        center_frame,
        foot_roller_0,
        elem_a="roller_axle",
        elem_b="roller_core",
        reason="The roller core is intentionally centered on the shared front axle.",
    )
    ctx.allow_overlap(
        center_frame,
        foot_roller_1,
        elem_a="roller_axle",
        elem_b="roller_body",
        reason="The foam ankle roller is represented as a solid sleeve proxy spinning around the shared axle.",
    )
    ctx.allow_overlap(
        center_frame,
        foot_roller_1,
        elem_a="roller_axle",
        elem_b="roller_core",
        reason="The roller core is intentionally centered on the shared front axle.",
    )

    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        positive_elem="seat_cushion",
        negative_elem="back_cushion",
        min_gap=0.006,
        max_gap=0.028,
        name="split pads keep a narrow center gap",
    )
    ctx.expect_gap(
        backrest,
        rear_support,
        axis="z",
        positive_elem="support_bar",
        negative_elem="rear_saddle",
        min_gap=0.0,
        max_gap=0.02,
        name="rear support sits just below the backrest catch bar",
    )
    ctx.expect_origin_gap(
        foot_roller_0,
        seat,
        axis="x",
        min_gap=0.25,
        name="foot rollers stay clearly ahead of the seat pad",
    )
    ctx.expect_origin_gap(
        front_wheel_0,
        seat,
        axis="x",
        min_gap=0.30,
        name="front transport wheels stay out at the front of the bench",
    )

    back_limits = back_hinge.motion_limits
    seat_limits = seat_hinge.motion_limits
    rear_limits = rear_support_hinge.motion_limits

    rest_back_aabb = ctx.part_world_aabb(backrest)
    rest_seat_aabb = ctx.part_world_aabb(seat)
    rest_support_aabb = ctx.part_world_aabb(rear_support)

    if back_limits is not None and back_limits.upper is not None and rest_back_aabb is not None:
        with ctx.pose({back_hinge: back_limits.upper}):
            raised_back_aabb = ctx.part_world_aabb(backrest)
        ctx.check(
            "backrest raises toward an incline position",
            raised_back_aabb is not None and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.24,
            details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
        )

    if seat_limits is not None and seat_limits.upper is not None and rest_seat_aabb is not None:
        with ctx.pose({seat_hinge: seat_limits.upper}):
            raised_seat_aabb = ctx.part_world_aabb(seat)
        ctx.check(
            "seat front lifts with positive adjustment",
            raised_seat_aabb is not None and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.05,
            details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
        )

    if rear_limits is not None and rear_limits.upper is not None and rest_support_aabb is not None:
        with ctx.pose({rear_support_hinge: rear_limits.upper}):
            raised_support_aabb = ctx.part_world_aabb(rear_support)
        ctx.check(
            "rear support link rotates upward with positive motion",
            raised_support_aabb is not None and raised_support_aabb[1][2] > rest_support_aabb[1][2] + 0.04,
            details=f"rest={rest_support_aabb}, raised={raised_support_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
