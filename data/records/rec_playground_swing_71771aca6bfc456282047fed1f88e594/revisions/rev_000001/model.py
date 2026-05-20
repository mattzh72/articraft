from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TOP_Z = 2.25
PIVOT_Z = 2.08
SEAT_DROP = 1.33
SEAT_SPACING = 1.30
HANGER_HALF_SPAN = 0.28


def _tube_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(horizontal, dz)
    yaw = math.atan2(dy, dx) if horizontal > 1.0e-9 else 0.0
    origin = Origin(
        xyz=(
            (start[0] + end[0]) * 0.5,
            (start[1] + end[1]) * 0.5,
            (start[2] + end[2]) * 0.5,
        ),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_tube(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    origin, length = _tube_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_pivot_bracket(frame, *, x: float, hardware, index: int) -> None:
    frame.visual(
        Box((0.12, 0.17, 0.035)),
        origin=Origin(xyz=(x, 0.0, PIVOT_Z + 0.0435)),
        material=hardware,
        name=f"clevis_cross_{index}",
    )
    frame.visual(
        Box((0.070, 0.042, 0.115)),
        origin=Origin(xyz=(x, 0.0, PIVOT_Z + 0.095)),
        material=hardware,
        name=f"hanger_stem_{index}",
    )
    for side, y in enumerate((-0.038, 0.038)):
        frame.visual(
            Box((0.12, 0.018, 0.13)),
            origin=Origin(xyz=(x, y, PIVOT_Z - 0.010)),
            material=hardware,
            name=f"clevis_cheek_{index}_{side}",
        )


def _add_swing_seat(model, frame, *, seat_index: int, center_x: float, rubber, hardware) -> None:
    seat = model.part(f"seat_{seat_index}")

    seat.visual(
        Box((0.70, 0.34, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -SEAT_DROP)),
        material=rubber,
        name="seat_board",
    )
    seat.visual(
        Box((0.62, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.155, -SEAT_DROP + 0.045)),
        material=hardware,
        name="front_edge_band",
    )
    seat.visual(
        Box((0.62, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.155, -SEAT_DROP + 0.045)),
        material=hardware,
        name="rear_edge_band",
    )

    for hanger_index, x in enumerate((-HANGER_HALF_SPAN, HANGER_HALF_SPAN)):
        seat.visual(
            Cylinder(radius=0.026, length=0.085),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"pivot_bushing_{hanger_index}",
        )
        seat.visual(
            Cylinder(radius=0.016, length=SEAT_DROP - 0.030),
            origin=Origin(xyz=(x, 0.0, -(SEAT_DROP - 0.030) * 0.5)),
            material=hardware,
            name=f"side_hanger_{hanger_index}",
        )
        seat.visual(
            Box((0.110, 0.070, 0.030)),
            origin=Origin(xyz=(x, 0.0, -SEAT_DROP + 0.060)),
            material=hardware,
            name=f"lower_anchor_{hanger_index}",
        )

    seat.inertial = Inertial.from_geometry(
        Box((0.72, 0.36, SEAT_DROP + 0.08)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, -SEAT_DROP * 0.55)),
    )

    model.articulation(
        f"frame_to_seat_{seat_index}",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(center_x, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.2, lower=-0.75, upper=0.75),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_seat_swing_set")

    frame_paint = model.material("green_powder_coat", rgba=(0.05, 0.34, 0.18, 1.0))
    hardware = model.material("galvanized_hardware", rgba=(0.62, 0.65, 0.66, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.026, 0.024, 1.0))
    foot_plastic = model.material("dark_foot_caps", rgba=(0.04, 0.045, 0.04, 1.0))

    frame = model.part("frame")

    frame.visual(
        Cylinder(radius=0.075, length=3.95),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="top_beam",
    )

    for end_index, x in enumerate((-1.72, 1.72)):
        top = (x, 0.0, TOP_Z - 0.030)
        for side_index, y in enumerate((-0.88, 0.88)):
            foot = (x, y, 0.075)
            _add_tube(
                frame,
                start=foot,
                end=top,
                radius=0.055,
                material=frame_paint,
                name=f"a_leg_{end_index}_{side_index}",
            )
            frame.visual(
                Box((0.42, 0.20, 0.060)),
                origin=Origin(xyz=(x, y, 0.030)),
                material=foot_plastic,
                name=f"foot_cap_{end_index}_{side_index}",
            )
        _add_tube(
            frame,
            start=(x, -0.66, 0.82),
            end=(x, 0.66, 0.82),
            radius=0.030,
            material=frame_paint,
            name=f"a_frame_spreader_{end_index}",
        )
        frame.visual(
            Box((0.22, 0.18, 0.12)),
            origin=Origin(xyz=(x, 0.0, TOP_Z - 0.025)),
            material=hardware,
            name=f"top_gusset_{end_index}",
        )

    _add_tube(
        frame,
        start=(-1.72, -0.88, 0.100),
        end=(1.72, -0.88, 0.100),
        radius=0.032,
        material=frame_paint,
        name="rear_ground_rail",
    )
    _add_tube(
        frame,
        start=(-1.72, 0.88, 0.100),
        end=(1.72, 0.88, 0.100),
        radius=0.032,
        material=frame_paint,
        name="front_ground_rail",
    )
    _add_tube(
        frame,
        start=(-1.72, -0.44, 1.18),
        end=(1.72, -0.44, 1.18),
        radius=0.026,
        material=frame_paint,
        name="rear_mid_rail",
    )
    _add_tube(
        frame,
        start=(-1.72, 0.44, 1.18),
        end=(1.72, 0.44, 1.18),
        radius=0.026,
        material=frame_paint,
        name="front_mid_rail",
    )

    bracket_index = 0
    for center_x in (-SEAT_SPACING * 0.5, SEAT_SPACING * 0.5):
        for local_x in (-HANGER_HALF_SPAN, HANGER_HALF_SPAN):
            _add_pivot_bracket(frame, x=center_x + local_x, hardware=hardware, index=bracket_index)
            bracket_index += 1

    frame.inertial = Inertial.from_geometry(
        Box((4.00, 1.90, 2.35)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )

    _add_swing_seat(
        model,
        frame,
        seat_index=0,
        center_x=-SEAT_SPACING * 0.5,
        rubber=rubber,
        hardware=hardware,
    )
    _add_swing_seat(
        model,
        frame,
        seat_index=1,
        center_x=SEAT_SPACING * 0.5,
        rubber=rubber,
        hardware=hardware,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat_0 = object_model.get_part("seat_0")
    seat_1 = object_model.get_part("seat_1")
    joint_0 = object_model.get_articulation("frame_to_seat_0")
    joint_1 = object_model.get_articulation("frame_to_seat_1")

    ctx.check("two_independent_swing_joints", joint_0 is not None and joint_1 is not None)
    if frame is None or seat_0 is None or seat_1 is None or joint_0 is None or joint_1 is None:
        return ctx.report()

    ctx.expect_gap(
        frame,
        seat_0,
        axis="z",
        positive_elem="top_beam",
        negative_elem="seat_board",
        min_gap=1.25,
        name="seat_0_hangs_below_beam",
    )
    ctx.expect_gap(
        frame,
        seat_1,
        axis="z",
        positive_elem="top_beam",
        negative_elem="seat_board",
        min_gap=1.25,
        name="seat_1_hangs_below_beam",
    )
    ctx.expect_gap(
        seat_1,
        seat_0,
        axis="x",
        positive_elem="seat_board",
        negative_elem="seat_board",
        min_gap=0.45,
        name="seats_are_separate_along_beam",
    )

    def _board_y_center(part) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem="seat_board")
        if aabb is None:
            return None
        mins, maxs = aabb
        return float((mins[1] + maxs[1]) * 0.5)

    rest_0_y = _board_y_center(seat_0)
    rest_1_y = _board_y_center(seat_1)
    with ctx.pose({joint_0: 0.55, joint_1: 0.0}):
        moved_0_y = _board_y_center(seat_0)
        still_1_y = _board_y_center(seat_1)

    ctx.check(
        "seat_0_swings_forward",
        rest_0_y is not None and moved_0_y is not None and moved_0_y > rest_0_y + 0.45,
        details=f"rest={rest_0_y}, moved={moved_0_y}",
    )
    ctx.check(
        "seat_1_stays_put_when_seat_0_moves",
        rest_1_y is not None and still_1_y is not None and abs(still_1_y - rest_1_y) < 0.010,
        details=f"rest={rest_1_y}, posed={still_1_y}",
    )

    with ctx.pose({joint_0: 0.55, joint_1: -0.45}):
        pose_0_y = _board_y_center(seat_0)
        pose_1_y = _board_y_center(seat_1)
    ctx.check(
        "seats_can_pose_independently",
        pose_0_y is not None and pose_1_y is not None and pose_0_y > 0.40 and pose_1_y < -0.35,
        details=f"seat_0_y={pose_0_y}, seat_1_y={pose_1_y}",
    )

    return ctx.report()


object_model = build_object_model()
