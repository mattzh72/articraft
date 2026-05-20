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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rotate_y(x: float, z: float, angle: float) -> tuple[float, float]:
    return (
        x * math.cos(angle) + z * math.sin(angle),
        -x * math.sin(angle) + z * math.cos(angle),
    )


def _pitched_origin(x: float, y: float, z: float, angle: float) -> Origin:
    rx, rz = _rotate_y(x, z, angle)
    return Origin(xyz=(rx, y, rz), rpy=(0.0, angle, 0.0))


def _add_front_wheel(part, *, tire_material, hub_material, metal_material) -> None:
    wheel_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.040, length=0.034),
        origin=wheel_axis,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.038),
        origin=wheel_axis,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=wheel_axis,
        material=metal_material,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    frame_paint = model.material("frame_paint", rgba=(0.13, 0.13, 0.14, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.08, 0.08, 0.09, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.34, 0.35, 0.37, 1.0))
    selector_metal = model.material("selector_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    frame = model.part("frame")
    tube_r = 0.015

    _add_tube(
        frame,
        (-0.42, -0.15, 0.03),
        (-0.42, 0.15, 0.03),
        radius=tube_r,
        material=frame_paint,
        name="rear_cross",
    )
    _add_tube(
        frame,
        (0.40, -0.12, 0.05),
        (0.40, 0.12, 0.05),
        radius=tube_r,
        material=frame_paint,
        name="front_cross",
    )
    _add_tube(
        frame,
        (-0.42, 0.15, 0.03),
        (0.26, 0.15, 0.05),
        radius=tube_r,
        material=frame_paint,
        name="side_rail_0",
    )
    _add_tube(
        frame,
        (-0.42, -0.15, 0.03),
        (0.26, -0.15, 0.05),
        radius=tube_r,
        material=frame_paint,
        name="side_rail_1",
    )
    _add_tube(
        frame,
        (-0.30, 0.15, 0.035),
        (-0.145, 0.05, 0.075),
        radius=0.013,
        material=frame_paint,
        name="support_brace_0",
    )
    _add_tube(
        frame,
        (-0.30, -0.15, 0.035),
        (-0.145, -0.05, 0.075),
        radius=0.013,
        material=frame_paint,
        name="support_brace_1",
    )
    _add_tube(
        frame,
        (-0.145, -0.05, 0.075),
        (-0.145, 0.05, 0.075),
        radius=0.013,
        material=frame_paint,
        name="support_bridge",
    )
    _add_tube(
        frame,
        (-0.05, 0.12, 0.33),
        (-0.05, -0.12, 0.33),
        radius=0.013,
        material=frame_paint,
        name="back_hinge_brace",
    )
    _add_tube(
        frame,
        (0.10, 0.10, 0.290),
        (0.10, -0.10, 0.290),
        radius=0.013,
        material=frame_paint,
        name="seat_cross",
    )
    _add_tube(
        frame,
        (-0.05, 0.12, 0.33),
        (-0.22, 0.15, 0.05),
        radius=0.013,
        material=frame_paint,
        name="back_upright_0",
    )
    _add_tube(
        frame,
        (-0.05, -0.12, 0.33),
        (-0.22, -0.15, 0.05),
        radius=0.013,
        material=frame_paint,
        name="back_upright_1",
    )
    _add_tube(
        frame,
        (0.10, 0.10, 0.290),
        (0.26, 0.15, 0.05),
        radius=0.013,
        material=frame_paint,
        name="front_rise_0",
    )
    _add_tube(
        frame,
        (0.10, -0.10, 0.290),
        (0.26, -0.15, 0.05),
        radius=0.013,
        material=frame_paint,
        name="front_rise_1",
    )
    _add_tube(
        frame,
        (0.10, 0.10, 0.290),
        (0.40, 0.11, 0.11),
        radius=0.012,
        material=frame_paint,
        name="front_reach_0",
    )
    _add_tube(
        frame,
        (0.10, -0.10, 0.290),
        (0.40, -0.11, 0.11),
        radius=0.012,
        material=frame_paint,
        name="front_reach_1",
    )

    frame.visual(
        Box((0.060, 0.020, 0.090)),
        origin=Origin(xyz=(-0.122, 0.048, 0.090)),
        material=bracket_steel,
        name="support_clevis_0",
    )
    frame.visual(
        Box((0.060, 0.020, 0.090)),
        origin=Origin(xyz=(-0.122, -0.048, 0.090)),
        material=bracket_steel,
        name="support_clevis_1",
    )
    frame.visual(
        Box((0.055, 0.020, 0.080)),
        origin=Origin(xyz=(-0.022, 0.110, 0.360)),
        material=bracket_steel,
        name="back_hinge_plate_0",
    )
    frame.visual(
        Box((0.055, 0.020, 0.080)),
        origin=Origin(xyz=(-0.022, -0.110, 0.360)),
        material=bracket_steel,
        name="back_hinge_plate_1",
    )
    frame.visual(
        Box((0.032, 0.020, 0.065)),
        origin=Origin(xyz=(0.082, 0.100, 0.305)),
        material=bracket_steel,
        name="seat_hinge_plate_0",
    )
    frame.visual(
        Box((0.032, 0.020, 0.065)),
        origin=Origin(xyz=(0.082, -0.100, 0.305)),
        material=bracket_steel,
        name="seat_hinge_plate_1",
    )
    frame.visual(
        Box((0.050, 0.038, 0.080)),
        origin=Origin(xyz=(0.420, 0.105, 0.090)),
        material=bracket_steel,
        name="front_fork_0",
    )
    frame.visual(
        Box((0.050, 0.038, 0.080)),
        origin=Origin(xyz=(0.420, -0.105, 0.090)),
        material=bracket_steel,
        name="front_fork_1",
    )
    frame.visual(
        Box((0.060, 0.050, 0.016)),
        origin=Origin(xyz=(-0.420, 0.120, 0.008)),
        material=foot_rubber,
        name="rear_foot_0",
    )
    frame.visual(
        Box((0.060, 0.050, 0.016)),
        origin=Origin(xyz=(-0.420, -0.120, 0.008)),
        material=foot_rubber,
        name="rear_foot_1",
    )

    backrest = model.part("backrest")
    back_angle = 0.34
    backrest.visual(
        Cylinder(radius=0.017, length=0.200),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_steel,
        name="back_hinge_barrel",
    )
    backrest.visual(
        Box((0.140, 0.180, 0.018)),
        origin=_pitched_origin(-0.075, 0.0, 0.000, back_angle),
        material=bracket_steel,
        name="back_hinge_plate",
    )
    backrest.visual(
        Box((0.460, 0.030, 0.024)),
        origin=_pitched_origin(-0.285, 0.090, -0.011, back_angle),
        material=bracket_steel,
        name="back_rail_0",
    )
    backrest.visual(
        Box((0.460, 0.030, 0.024)),
        origin=_pitched_origin(-0.285, -0.090, -0.011, back_angle),
        material=bracket_steel,
        name="back_rail_1",
    )
    backrest.visual(
        Box((0.300, 0.030, 0.030)),
        origin=_pitched_origin(-0.270, 0.090, 0.010, back_angle),
        material=bracket_steel,
        name="back_mount_0",
    )
    backrest.visual(
        Box((0.300, 0.030, 0.030)),
        origin=_pitched_origin(-0.270, -0.090, 0.010, back_angle),
        material=bracket_steel,
        name="back_mount_1",
    )
    backrest.visual(
        Box((0.190, 0.060, 0.012)),
        origin=_pitched_origin(-0.240, 0.0, -0.022, back_angle),
        material=bracket_steel,
        name="selector_rack",
    )
    backrest.visual(
        Box((0.170, 0.060, 0.016)),
        origin=_pitched_origin(-0.145, 0.0, -0.015, back_angle),
        material=bracket_steel,
        name="selector_spine",
    )
    for index, x_local in enumerate((-0.180, -0.240, -0.300)):
        backrest.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(
                xyz=(
                    _rotate_y(x_local, -0.022, back_angle)[0],
                    0.0,
                    _rotate_y(x_local, -0.022, back_angle)[1],
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=bracket_steel,
            name=f"selector_boss_{index}",
        )
    backrest.visual(
        Box((0.630, 0.290, 0.014)),
        origin=_pitched_origin(-0.375, 0.0, 0.026, back_angle),
        material=bracket_steel,
        name="back_board",
    )
    backrest.visual(
        Box((0.610, 0.270, 0.050)),
        origin=_pitched_origin(-0.375, 0.0, 0.058, back_angle),
        material=pad_vinyl,
        name="back_pad",
    )

    seat = model.part("seat")
    seat_angle = -0.10
    seat.visual(
        Cylinder(radius=0.016, length=0.194),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_steel,
        name="seat_hinge_barrel",
    )
    seat.visual(
        Box((0.120, 0.164, 0.018)),
        origin=_pitched_origin(0.040, 0.0, -0.008, seat_angle),
        material=bracket_steel,
        name="seat_hinge_plate",
    )
    seat.visual(
        Box((0.240, 0.028, 0.022)),
        origin=_pitched_origin(0.090, 0.075, -0.011, seat_angle),
        material=bracket_steel,
        name="seat_rail_0",
    )
    seat.visual(
        Box((0.240, 0.028, 0.022)),
        origin=_pitched_origin(0.090, -0.075, -0.011, seat_angle),
        material=bracket_steel,
        name="seat_rail_1",
    )
    seat.visual(
        Box((0.340, 0.280, 0.014)),
        origin=_pitched_origin(0.135, 0.0, 0.018, seat_angle),
        material=bracket_steel,
        name="seat_board",
    )
    seat.visual(
        Box((0.320, 0.260, 0.048)),
        origin=_pitched_origin(0.135, 0.0, 0.048, seat_angle),
        material=pad_vinyl,
        name="seat_pad",
    )

    support_link = model.part("support_link")
    support_angle = 1.15
    support_link.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_steel,
        name="lower_pivot",
    )
    support_link.visual(
        Box((0.310, 0.032, 0.018)),
        origin=_pitched_origin(-0.155, 0.0, 0.0, support_angle),
        material=bracket_steel,
        name="support_arm",
    )
    support_link.visual(
        Box((0.026, 0.082, 0.018)),
        origin=_pitched_origin(-0.285, 0.0, -0.004, support_angle),
        material=bracket_steel,
        name="support_cap",
    )
    guide_x, guide_z = _rotate_y(-0.120, 0.0, support_angle)
    support_link.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(
            xyz=(guide_x, 0.0, guide_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=selector_metal,
        name="selector_guide",
    )

    wheel_0 = model.part("wheel_0")
    _add_front_wheel(
        wheel_0,
        tire_material=wheel_rubber,
        hub_material=wheel_hub,
        metal_material=selector_metal,
    )

    wheel_1 = model.part("wheel_1")
    _add_front_wheel(
        wheel_1,
        tire_material=wheel_rubber,
        hub_material=wheel_hub,
        metal_material=selector_metal,
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=selector_metal,
        name="selector_pin",
    )
    selector_knob.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=selector_metal,
        name="knob_head",
    )
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="knob_stem",
    )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.4,
            lower=-0.32,
            upper=0.88,
        ),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.060, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-0.12,
            upper=0.35,
        ),
    )
    model.articulation(
        "frame_to_support_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_link,
        origin=Origin(xyz=(-0.100, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=-0.55,
            upper=0.12,
        ),
    )
    model.articulation(
        "frame_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(0.440, 0.141, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(0.440, -0.141, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=20.0),
    )
    model.articulation(
        "support_link_to_selector_knob",
        ArticulationType.PRISMATIC,
        parent=support_link,
        child=selector_knob,
        origin=Origin(xyz=(guide_x, 0.0, guide_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_link = object_model.get_part("support_link")
    selector_knob = object_model.get_part("selector_knob")

    back_joint = object_model.get_articulation("frame_to_backrest")
    seat_joint = object_model.get_articulation("frame_to_seat")
    support_joint = object_model.get_articulation("frame_to_support_link")
    wheel_joint_0 = object_model.get_articulation("frame_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("frame_to_wheel_1")
    knob_joint = object_model.get_articulation("support_link_to_selector_knob")

    ctx.allow_overlap(
        support_link,
        selector_knob,
        elem_a="selector_guide",
        elem_b="selector_pin",
        reason="The pop-pin shaft is intentionally represented as sliding inside the selector guide sleeve.",
    )
    ctx.allow_overlap(
        support_link,
        selector_knob,
        elem_a="support_arm",
        elem_b="selector_pin",
        reason="The pop-pin shaft passes through a simplified support-arm proxy before entering the guide sleeve.",
    )
    ctx.allow_overlap(
        frame,
        support_link,
        elem_a="support_clevis_0",
        elem_b="lower_pivot",
        reason="The support-link pivot barrel is represented as passing through the simplified clevis lug without modeling the bore hole.",
    )
    ctx.allow_overlap(
        frame,
        support_link,
        elem_a="support_clevis_1",
        elem_b="lower_pivot",
        reason="The support-link pivot barrel is represented as passing through the simplified clevis lug without modeling the bore hole.",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_plate_0",
        elem_b="seat_hinge_barrel",
        reason="The seat hinge barrel is represented as running through a simplified frame-side hinge cheek without modeling the bore hole.",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_plate_1",
        elem_b="seat_hinge_barrel",
        reason="The seat hinge barrel is represented as running through a simplified frame-side hinge cheek without modeling the bore hole.",
    )

    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        positive_elem="seat_pad",
        negative_elem="back_pad",
        min_gap=0.020,
        max_gap=0.070,
        name="split pads stay visibly separated",
    )
    ctx.expect_gap(
        backrest,
        support_link,
        axis="z",
        positive_elem="selector_rack",
        negative_elem="support_cap",
        min_gap=0.0,
        max_gap=0.020,
        name="support link sits just below the backrest selector rack",
    )
    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        positive_elem="seat_rail_0",
        negative_elem="seat_cross",
        min_gap=0.010,
        max_gap=0.060,
        name="seat support bracket clears the center frame",
    )

    def _max_z(part, *, elem: str | None = None) -> float | None:
        if elem is None:
            aabb = ctx.part_world_aabb(part)
        else:
            aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return aabb[1][2]

    rest_back_z = _max_z(backrest, elem="back_pad")
    back_limits = back_joint.motion_limits
    if back_limits is not None and back_limits.upper is not None:
        with ctx.pose({back_joint: back_limits.upper}):
            raised_back_z = _max_z(backrest, elem="back_pad")
        ctx.check(
            "backrest raises upward",
            rest_back_z is not None
            and raised_back_z is not None
            and raised_back_z > rest_back_z + 0.14,
            details=f"rest_back_z={rest_back_z}, raised_back_z={raised_back_z}",
        )

    rest_seat_z = _max_z(seat, elem="seat_pad")
    seat_limits = seat_joint.motion_limits
    if seat_limits is not None and seat_limits.upper is not None:
        with ctx.pose({seat_joint: seat_limits.upper}):
            raised_seat_z = _max_z(seat, elem="seat_pad")
        ctx.check(
            "seat front lifts at upper limit",
            rest_seat_z is not None
            and raised_seat_z is not None
            and raised_seat_z > rest_seat_z + 0.035,
            details=f"rest_seat_z={rest_seat_z}, raised_seat_z={raised_seat_z}",
        )

    support_limits = support_joint.motion_limits
    if (
        support_limits is not None
        and support_limits.lower is not None
        and support_limits.upper is not None
    ):
        with ctx.pose({support_joint: support_limits.lower}):
            lowered_support_z = _max_z(support_link, elem="support_cap")
        with ctx.pose({support_joint: support_limits.upper}):
            raised_support_z = _max_z(support_link, elem="support_cap")
        ctx.check(
            "rear support link swings upward",
            lowered_support_z is not None
            and raised_support_z is not None
            and raised_support_z > lowered_support_z + 0.10,
            details=f"lowered_support_z={lowered_support_z}, raised_support_z={raised_support_z}",
        )

    knob_limits = knob_joint.motion_limits
    rest_knob_pos = ctx.part_world_position(selector_knob)
    if knob_limits is not None and knob_limits.upper is not None:
        with ctx.pose({knob_joint: knob_limits.upper}):
            extended_knob_pos = ctx.part_world_position(selector_knob)
        ctx.check(
            "selector knob pulls outward",
            rest_knob_pos is not None
            and extended_knob_pos is not None
            and extended_knob_pos[1] > rest_knob_pos[1] + 0.012,
            details=f"rest_knob_pos={rest_knob_pos}, extended_knob_pos={extended_knob_pos}",
        )

    continuous_wheels = (
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_joint_0.axis) == (0.0, 1.0, 0.0)
        and tuple(wheel_joint_1.axis) == (0.0, 1.0, 0.0)
        and wheel_joint_0.motion_limits is not None
        and wheel_joint_1.motion_limits is not None
        and wheel_joint_0.motion_limits.lower is None
        and wheel_joint_0.motion_limits.upper is None
        and wheel_joint_1.motion_limits.lower is None
        and wheel_joint_1.motion_limits.upper is None
    )
    ctx.check(
        "front transport wheels use continuous spin joints",
        continuous_wheels,
        details=(
            f"wheel_0=({wheel_joint_0.articulation_type}, {wheel_joint_0.axis}, {wheel_joint_0.motion_limits}), "
            f"wheel_1=({wheel_joint_1.articulation_type}, {wheel_joint_1.axis}, {wheel_joint_1.motion_limits})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
