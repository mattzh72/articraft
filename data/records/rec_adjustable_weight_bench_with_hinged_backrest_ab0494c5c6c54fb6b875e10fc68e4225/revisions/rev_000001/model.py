from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

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


def _rotate_y(x: float, z: float, angle: float) -> tuple[float, float]:
    return (x * cos(angle) + z * sin(angle), -x * sin(angle) + z * cos(angle))


def _beam_pose(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(-dz, sqrt(dx * dx + dy * dy))
    return (
        length,
        Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
    )


def _add_beam(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    thickness: float,
    depth: float | None = None,
    material=None,
    name: str | None = None,
):
    length, origin = _beam_pose(start, end)
    part.visual(
        Box((length, thickness, thickness if depth is None else depth)),
        origin=origin,
        material=material,
        name=name,
    )


def _add_pivoted_pad(
    part,
    *,
    pad_name: str,
    pad_length: float,
    pad_width: float,
    pad_thickness: float,
    board_thickness: float,
    bridge_length: float,
    angle: float,
    pad_material,
    board_material,
):
    pad_x, pad_z = _rotate_y(-pad_length * 0.5, pad_thickness * 0.5, angle)
    part.visual(
        Box((pad_length, pad_width, pad_thickness)),
        origin=Origin(xyz=(pad_x, 0.0, pad_z), rpy=(0.0, angle, 0.0)),
        material=pad_material,
        name=pad_name,
    )

    board_x, board_z = _rotate_y(-pad_length * 0.48, board_thickness * 0.5, angle)
    part.visual(
        Box((pad_length * 0.92, pad_width * 0.88, board_thickness)),
        origin=Origin(xyz=(board_x, 0.0, board_z), rpy=(0.0, angle, 0.0)),
        material=board_material,
        name=f"{pad_name}_board",
    )

    bridge_x, bridge_z = _rotate_y(-bridge_length * 0.5, board_thickness * 0.7, angle)
    part.visual(
        Box((bridge_length, pad_width * 0.56, board_thickness * 1.5)),
        origin=Origin(xyz=(bridge_x, 0.0, bridge_z), rpy=(0.0, angle, 0.0)),
        material=board_material,
        name=f"{pad_name}_bridge",
    )

    part.visual(
        Cylinder(radius=board_thickness * 0.95, length=pad_width * 0.62),
        origin=Origin(xyz=(0.0, 0.0, board_thickness * 0.8), rpy=(pi / 2.0, 0.0, 0.0)),
        material=board_material,
        name=f"{pad_name}_hinge",
    )


def _add_wheel(part, *, radius: float, width: float, tire_material, hub_material, name: str):
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name=name,
    )
    part.visual(
        Cylinder(radius=radius * 0.52, length=width * 1.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name=f"{name}_hub",
    )
    part.visual(
        Cylinder(radius=radius * 0.25, length=width * 1.20),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name=f"{name}_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_adjustable_bench")

    frame_paint = model.material("frame_paint", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.21, 0.22, 0.24, 1.0))
    upholstery = model.material("upholstery", rgba=(0.09, 0.09, 0.10, 1.0))
    backing = model.material("backing", rgba=(0.19, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    roller_vinyl = model.material("roller_vinyl", rgba=(0.13, 0.13, 0.14, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.55, 0.57, 0.60, 1.0))

    frame = model.part("frame")
    heavy = 0.08
    medium = 0.06
    light = 0.05

    rear_stabilizer_length, rear_stabilizer_origin = _beam_pose((-0.62, -0.28, 0.04), (-0.62, 0.28, 0.04))
    frame.visual(
        Box((rear_stabilizer_length, heavy, heavy)),
        origin=rear_stabilizer_origin,
        material=frame_paint,
        name="rear_stabilizer",
    )
    _add_beam(frame, (0.60, -0.26, 0.04), (0.60, 0.26, 0.04), thickness=heavy, material=frame_paint, name="front_stabilizer")
    _add_beam(frame, (-0.62, -0.14, 0.08), (0.36, -0.14, 0.12), thickness=medium, material=frame_paint, name="side_rail_0")
    _add_beam(frame, (-0.62, 0.14, 0.08), (0.36, 0.14, 0.12), thickness=medium, material=frame_paint, name="side_rail_1")
    _add_beam(frame, (-0.62, -0.14, 0.08), (-0.27, -0.15, 0.18), thickness=light, material=frame_paint, name="rear_brace_0")
    _add_beam(frame, (-0.62, 0.14, 0.08), (-0.27, 0.15, 0.18), thickness=light, material=frame_paint, name="rear_brace_1")
    _add_beam(frame, (-0.27, -0.15, 0.18), (-0.27, 0.15, 0.18), thickness=light, material=frame_paint, name="support_pivot_tube")
    _add_beam(frame, (-0.27, -0.15, 0.18), (-0.05, -0.15, 0.41), thickness=light, material=frame_paint, name="tower_brace_0")
    _add_beam(frame, (-0.27, 0.15, 0.18), (-0.05, 0.15, 0.41), thickness=light, material=frame_paint, name="tower_brace_1")
    _add_beam(frame, (-0.05, -0.15, 0.41), (-0.05, 0.15, 0.41), thickness=light, material=frame_paint, name="back_hinge_tube")
    _add_beam(frame, (0.18, -0.14, 0.14), (0.34, -0.15, 0.31), thickness=light, material=frame_paint, name="seat_upright_0")
    _add_beam(frame, (0.18, 0.14, 0.14), (0.34, 0.15, 0.31), thickness=light, material=frame_paint, name="seat_upright_1")
    _add_beam(frame, (0.34, -0.15, 0.31), (0.34, 0.15, 0.31), thickness=light, material=frame_paint, name="seat_hinge_tube")
    _add_beam(frame, (0.36, -0.14, 0.12), (0.60, -0.17, 0.16), thickness=light, material=frame_paint, name="front_brace_0")
    _add_beam(frame, (0.36, 0.14, 0.12), (0.60, 0.17, 0.16), thickness=light, material=frame_paint, name="front_brace_1")
    _add_beam(frame, (0.60, -0.17, 0.04), (0.60, -0.17, 0.24), thickness=light, material=frame_paint, name="yoke_post_0")
    _add_beam(frame, (0.60, 0.17, 0.04), (0.60, 0.17, 0.24), thickness=light, material=frame_paint, name="yoke_post_1")
    _add_beam(frame, (0.64, -0.17, 0.18), (0.64, 0.17, 0.18), thickness=light, material=frame_paint, name="roller_axle_tube")
    _add_beam(frame, (-0.60, -0.20, 0.08), (-0.60, -0.20, 0.14), thickness=light, material=frame_paint, name="wheel_bracket_0")
    _add_beam(frame, (-0.60, 0.20, 0.08), (-0.60, 0.20, 0.14), thickness=light, material=frame_paint, name="wheel_bracket_1")
    _add_beam(frame, (-0.60, -0.20, 0.11), (-0.60, -0.315, 0.11), thickness=0.035, material=dark_steel, name="wheel_axle_0")
    _add_beam(frame, (-0.60, 0.20, 0.11), (-0.60, 0.315, 0.11), thickness=0.035, material=dark_steel, name="wheel_axle_1")
    frame.visual(
        Box((0.08, 0.36, 0.05)),
        origin=Origin(xyz=(0.60, 0.0, 0.18)),
        material=frame_paint,
        name="front_yoke_bridge",
    )
    frame.visual(
        Box((0.20, 0.26, 0.14)),
        origin=Origin(xyz=(0.22, 0.0, 0.20)),
        material=dark_steel,
        name="center_gusset",
    )

    backrest = model.part("backrest")
    backrest_angle = 0.34
    backrest_pad_x, backrest_pad_z = _rotate_y(-0.82 * 0.5, 0.09 * 0.5, backrest_angle)
    backrest.visual(
        Box((0.82, 0.31, 0.09)),
        origin=Origin(xyz=(backrest_pad_x, 0.0, backrest_pad_z), rpy=(0.0, backrest_angle, 0.0)),
        material=upholstery,
        name="backrest_pad",
    )
    backrest_board_x, backrest_board_z = _rotate_y(-0.82 * 0.48, 0.025 * 0.5, backrest_angle)
    backrest.visual(
        Box((0.82 * 0.92, 0.31 * 0.88, 0.025)),
        origin=Origin(xyz=(backrest_board_x, 0.0, backrest_board_z), rpy=(0.0, backrest_angle, 0.0)),
        material=backing,
        name="backrest_pad_board",
    )
    backrest_bridge_x, backrest_bridge_z = _rotate_y(-0.11 * 0.5, 0.025 * 0.7, backrest_angle)
    backrest.visual(
        Box((0.11, 0.31 * 0.56, 0.025 * 1.5)),
        origin=Origin(xyz=(backrest_bridge_x, 0.0, backrest_bridge_z), rpy=(0.0, backrest_angle, 0.0)),
        material=backing,
        name="backrest_pad_bridge",
    )
    backrest.visual(
        Cylinder(radius=0.025 * 0.95, length=0.31 * 0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.025 * 0.8), rpy=(pi / 2.0, 0.0, 0.0)),
        material=backing,
        name="backrest_pad_hinge",
    )

    seat = model.part("seat")
    seat_angle = 0.05
    seat_pad_x, seat_pad_z = _rotate_y(-0.30 * 0.5, 0.085 * 0.5, seat_angle)
    seat.visual(
        Box((0.30, 0.34, 0.085)),
        origin=Origin(xyz=(seat_pad_x, 0.0, seat_pad_z), rpy=(0.0, seat_angle, 0.0)),
        material=upholstery,
        name="seat_pad",
    )
    seat_board_x, seat_board_z = _rotate_y(-0.30 * 0.48, 0.025 * 0.5, seat_angle)
    seat.visual(
        Box((0.30 * 0.92, 0.34 * 0.88, 0.025)),
        origin=Origin(xyz=(seat_board_x, 0.0, seat_board_z), rpy=(0.0, seat_angle, 0.0)),
        material=backing,
        name="seat_pad_board",
    )
    seat_bridge_x, seat_bridge_z = _rotate_y(-0.09 * 0.5, 0.025 * 0.7, seat_angle)
    seat.visual(
        Box((0.09, 0.34 * 0.56, 0.025 * 1.5)),
        origin=Origin(xyz=(seat_bridge_x, 0.0, seat_bridge_z), rpy=(0.0, seat_angle, 0.0)),
        material=backing,
        name="seat_pad_bridge",
    )
    seat.visual(
        Cylinder(radius=0.025 * 0.95, length=0.34 * 0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.025 * 0.8), rpy=(pi / 2.0, 0.0, 0.0)),
        material=backing,
        name="seat_pad_hinge",
    )

    support_arm = model.part("support_arm")
    _add_beam(support_arm, (0.0, -0.12, 0.0), (0.0, 0.12, 0.0), thickness=0.05, material=frame_paint, name="support_axle")
    _add_beam(support_arm, (0.0, -0.12, 0.0), (-0.16, -0.12, 0.23), thickness=0.045, material=frame_paint, name="support_leg_0")
    _add_beam(support_arm, (0.0, 0.12, 0.0), (-0.16, 0.12, 0.23), thickness=0.045, material=frame_paint, name="support_leg_1")
    _add_beam(support_arm, (-0.16, -0.12, 0.23), (-0.16, 0.12, 0.23), thickness=0.045, material=frame_paint, name="support_cross")
    _add_beam(support_arm, (-0.06, -0.10, 0.10), (-0.16, 0.00, 0.23), thickness=0.035, material=frame_paint, name="support_web_0")
    _add_beam(support_arm, (-0.06, 0.10, 0.10), (-0.16, 0.00, 0.23), thickness=0.035, material=frame_paint, name="support_web_1")
    saddle_x, saddle_z = _rotate_y(-0.11, 0.19, 0.12)
    support_arm.visual(
        Box((0.18, 0.24, 0.025)),
        origin=Origin(xyz=(saddle_x, 0.0, saddle_z), rpy=(0.0, 0.12, 0.0)),
        material=backing,
        name="support_saddle",
    )

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    wheel_0.visual(
        Cylinder(radius=0.055 * 0.52, length=0.028 * 1.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="wheel_tire_hub",
    )
    wheel_0.visual(
        Cylinder(radius=0.055 * 0.25, length=0.028 * 1.20),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="wheel_tire_cap",
    )
    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    wheel_1.visual(
        Cylinder(radius=0.055 * 0.52, length=0.028 * 1.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="wheel_tire_hub",
    )
    wheel_1.visual(
        Cylinder(radius=0.055 * 0.25, length=0.028 * 1.20),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="wheel_tire_cap",
    )

    roller_0 = model.part("roller_0")
    roller_0.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=roller_vinyl,
        name="roller_body",
    )
    roller_0.visual(
        Cylinder(radius=0.055 * 0.52, length=0.12 * 1.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="roller_body_hub",
    )
    roller_0.visual(
        Cylinder(radius=0.055 * 0.25, length=0.12 * 1.20),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="roller_body_cap",
    )
    roller_1 = model.part("roller_1")
    roller_1.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=roller_vinyl,
        name="roller_body",
    )
    roller_1.visual(
        Cylinder(radius=0.055 * 0.52, length=0.12 * 1.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="roller_body_hub",
    )
    roller_1.visual(
        Cylinder(radius=0.055 * 0.25, length=0.12 * 1.20),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="roller_body_cap",
    )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.05, 0.0, 0.41)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-0.34, upper=1.02),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.34, 0.0, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.10, upper=0.34),
    )
    model.articulation(
        "frame_to_support_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.27, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-0.55, upper=0.45),
    )
    model.articulation(
        "frame_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.60, -0.314, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.60, 0.314, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_roller_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=roller_0,
        origin=Origin(xyz=(0.64, -0.085, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_roller_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=roller_1,
        origin=Origin(xyz=(0.64, 0.085, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_arm = object_model.get_part("support_arm")
    backrest_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    support_joint = object_model.get_articulation("frame_to_support_arm")

    ctx.allow_overlap(
        frame,
        backrest,
        reason="The backrest hinge barrel is intentionally modeled as nesting on the frame hinge tube.",
    )
    ctx.allow_overlap(
        frame,
        support_arm,
        reason="The wide rear support arm rotates on the lower frame pivot tube.",
    )
    ctx.allow_overlap(
        frame,
        seat,
        reason="The seat hinge barrel is intentionally modeled as nesting on the forward frame hinge tube.",
    )
    ctx.allow_overlap(
        frame,
        "wheel_0",
        reason="The left transport wheel hub is intentionally carried on the rear axle stub.",
    )
    ctx.allow_overlap(
        frame,
        "wheel_1",
        reason="The right transport wheel hub is intentionally carried on the rear axle stub.",
    )
    ctx.allow_overlap(
        frame,
        "roller_0",
        reason="The left front roller rotates around the shared front axle tube.",
    )
    ctx.allow_overlap(
        frame,
        "roller_1",
        reason="The right front roller rotates around the shared front axle tube.",
    )

    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        positive_elem="seat_pad",
        negative_elem="backrest_pad",
        min_gap=0.03,
        max_gap=0.12,
        name="split pad gap stays visible",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="y",
        elem_a="backrest_pad",
        elem_b="seat_pad",
        min_overlap=0.24,
        name="backrest and seat share commercial bench width",
    )
    ctx.expect_gap(
        backrest,
        support_arm,
        axis="z",
        positive_elem="backrest_pad_board",
        negative_elem="support_saddle",
        min_gap=0.002,
        max_gap=0.05,
        name="support arm sits just under the backrest",
    )
    ctx.expect_overlap(
        support_arm,
        backrest,
        axes="xy",
        elem_a="support_saddle",
        elem_b="backrest_pad_board",
        min_overlap=0.08,
        name="support arm stays centered under the backrest",
    )
    ctx.expect_gap(
        "roller_1",
        "roller_0",
        axis="y",
        positive_elem="roller_body",
        negative_elem="roller_body",
        min_gap=0.02,
        max_gap=0.08,
        name="front rollers keep a center gap on the shared axle",
    )
    ctx.expect_gap(
        "wheel_1",
        frame,
        axis="y",
        positive_elem="wheel_tire",
        negative_elem="rear_stabilizer",
        min_gap=0.018,
        name="right transport wheel sits outboard of the frame",
    )
    ctx.expect_gap(
        frame,
        "wheel_0",
        axis="y",
        positive_elem="rear_stabilizer",
        negative_elem="wheel_tire",
        min_gap=0.018,
        name="left transport wheel sits outboard of the frame",
    )

    backrest_limits = backrest_hinge.motion_limits
    if backrest_limits is not None and backrest_limits.upper is not None:
        rest_backrest = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
        with ctx.pose({backrest_hinge: backrest_limits.upper}):
            raised_backrest = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
        ctx.check(
            "backrest raises at upper limit",
            rest_backrest is not None
            and raised_backrest is not None
            and raised_backrest[1][2] > rest_backrest[1][2] + 0.18,
            details=f"rest={rest_backrest}, raised={raised_backrest}",
        )

    seat_limits = seat_hinge.motion_limits
    if seat_limits is not None and seat_limits.upper is not None:
        rest_seat = ctx.part_element_world_aabb(seat, elem="seat_pad")
        with ctx.pose({seat_hinge: seat_limits.upper}):
            raised_seat = ctx.part_element_world_aabb(seat, elem="seat_pad")
        ctx.check(
            "seat tilts upward at upper limit",
            rest_seat is not None and raised_seat is not None and raised_seat[1][2] > rest_seat[1][2] + 0.04,
            details=f"rest={rest_seat}, raised={raised_seat}",
        )

    support_limits = support_joint.motion_limits
    if support_limits is not None and support_limits.upper is not None:
        rest_support = ctx.part_element_world_aabb(support_arm, elem="support_saddle")
        with ctx.pose({support_joint: support_limits.upper}):
            raised_support = ctx.part_element_world_aabb(support_arm, elem="support_saddle")
        ctx.check(
            "support arm rotates upward",
            rest_support is not None and raised_support is not None and raised_support[1][2] > rest_support[1][2] + 0.02,
            details=f"rest={rest_support}, raised={raised_support}",
        )

    wheel_axis_ok = all(
        object_model.get_articulation(name).axis == (0.0, 1.0, 0.0)
        for name in ("frame_to_wheel_0", "frame_to_wheel_1", "frame_to_roller_0", "frame_to_roller_1")
    )
    ctx.check("wheel and roller spin axes are transverse", wheel_axis_ok, details="All spin joints should rotate around the shared axle direction.")

    return ctx.report()


object_model = build_object_model()
