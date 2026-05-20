from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Mimic,
    Origin,
    TestContext,
    TestReport,
)


WHEEL_CENTER_Z = 1.90
RIM_RADIUS = 0.92
PIVOT_RADIUS = 1.40
SEAT_COUNT = 6
WHEEL_HALF_WIDTH = 0.12


def _origin_for_y_cylinder(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _origin_for_xz_cylinder(
    xyz: tuple[float, float, float],
    *,
    angle: float,
) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, angle, 0.0))


def _add_y_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin_for_y_cylinder(xyz),
        material=material,
        name=name,
    )


def _add_xz_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    angle: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin_for_xz_cylinder(xyz, angle=angle),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_wheel")

    frame_mat = model.material("frame_steel", rgba=(0.20, 0.23, 0.27, 1.0))
    frame_trim = model.material("frame_trim", rgba=(0.66, 0.69, 0.72, 1.0))
    wheel_mat = model.material("wheel_red", rgba=(0.76, 0.18, 0.13, 1.0))
    hub_mat = model.material("hub_gray", rgba=(0.54, 0.56, 0.60, 1.0))
    gondola_mat = model.material("gondola_cream", rgba=(0.89, 0.85, 0.74, 1.0))
    gondola_frame = model.material("gondola_frame", rgba=(0.14, 0.15, 0.16, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.96, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.22, 0.04)),
        material=frame_mat,
        name="foot_beam_0",
    )
    frame.visual(
        Box((0.96, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.22, 0.04)),
        material=frame_mat,
        name="foot_beam_1",
    )
    _add_y_cylinder(
        frame,
        name="tie_beam_0",
        radius=0.03,
        length=0.44,
        xyz=(-0.42, 0.0, 0.08),
        material=frame_mat,
    )
    _add_y_cylinder(
        frame,
        name="tie_beam_1",
        radius=0.03,
        length=0.44,
        xyz=(0.42, 0.0, 0.08),
        material=frame_mat,
    )
    frame.visual(
        Box((0.50, 0.44, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_trim,
        name="operator_platform",
    )

    for index, y in enumerate((-0.22, 0.22)):
        for side, x0 in enumerate((-0.46, 0.46)):
            top = (0.0, y, WHEEL_CENTER_Z)
            dx = top[0] - x0
            dz = top[2] - 0.08
            angle = math.atan2(dx, dz)
            length = math.hypot(dx, dz)
            _add_xz_cylinder(
                frame,
                name=f"leg_{index}_{side}",
                radius=0.03,
                length=length,
                xyz=((x0 + top[0]) / 2.0, y, (0.08 + top[2]) / 2.0),
                angle=angle,
                material=frame_mat,
            )
        frame.visual(
            Box((0.18, 0.08, 0.12)),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z)),
            material=frame_trim,
            name=f"bearing_block_{index}",
        )

    wheel = model.part("wheel")
    _add_y_cylinder(
        wheel,
        name="axle",
        radius=0.03,
        length=0.42,
        xyz=(0.0, 0.0, 0.0),
        material=hub_mat,
    )
    _add_y_cylinder(
        wheel,
        name="hub_drum",
        radius=0.12,
        length=0.20,
        xyz=(0.0, 0.0, 0.0),
        material=hub_mat,
    )
    _add_y_cylinder(
        wheel,
        name="hub_flange_0",
        radius=0.17,
        length=0.08,
        xyz=(0.0, -0.10, 0.0),
        material=wheel_mat,
    )
    _add_y_cylinder(
        wheel,
        name="hub_flange_1",
        radius=0.17,
        length=0.08,
        xyz=(0.0, 0.10, 0.0),
        material=wheel_mat,
    )

    seat_angles = [math.pi + index * (2.0 * math.pi / SEAT_COUNT) for index in range(SEAT_COUNT)]
    for index, angle in enumerate(seat_angles):
        spoke_inner_radius = 0.12
        spoke_outer_radius = PIVOT_RADIUS
        spoke_length = spoke_outer_radius - spoke_inner_radius
        spoke_center_radius = spoke_inner_radius + spoke_length / 2.0
        for side_index, y in enumerate((-WHEEL_HALF_WIDTH, WHEEL_HALF_WIDTH)):
            arm_name = "arm_0" if index == 0 and side_index == 0 else f"arm_{index}_{side_index}"
            _add_xz_cylinder(
                wheel,
                name=arm_name,
                radius=0.018,
                length=spoke_length,
                xyz=(
                    math.sin(angle) * spoke_center_radius,
                    y,
                    math.cos(angle) * spoke_center_radius,
                ),
                angle=angle,
                material=wheel_mat,
            )
        lug_center = (
            math.sin(angle) * PIVOT_RADIUS,
            0.0,
            math.cos(angle) * PIVOT_RADIUS,
        )
        _add_y_cylinder(
            wheel,
            name=f"lug_{index}_0",
            radius=0.02,
            length=0.10,
            xyz=(lug_center[0], -WHEEL_HALF_WIDTH, lug_center[2]),
            material=wheel_mat,
        )
        _add_y_cylinder(
            wheel,
            name=f"lug_{index}_1",
            radius=0.02,
            length=0.10,
            xyz=(lug_center[0], WHEEL_HALF_WIDTH, lug_center[2]),
            material=wheel_mat,
        )

    rim_segment_count = 12
    rim_segment_length = 2.0 * RIM_RADIUS * math.sin(math.pi / rim_segment_count) + 0.03
    for index in range(rim_segment_count):
        midpoint_angle = (index + 0.5) * (2.0 * math.pi / rim_segment_count)
        tangent_angle = midpoint_angle + math.pi / 2.0
        for side_index, y in enumerate((-WHEEL_HALF_WIDTH, WHEEL_HALF_WIDTH)):
            rim_name = f"rim_{index}" if side_index == 0 else f"rim_{index}_{side_index}"
            _add_xz_cylinder(
                wheel,
                name=rim_name,
                radius=0.028,
                length=rim_segment_length,
                xyz=(
                    math.sin(midpoint_angle) * RIM_RADIUS,
                    y,
                    math.cos(midpoint_angle) * RIM_RADIUS,
                ),
                angle=tangent_angle,
                material=wheel_mat,
            )

    wheel_spin = model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8),
    )

    for index, angle in enumerate(seat_angles):
        seat = model.part(f"seat_{index}")
        _add_y_cylinder(
            seat,
            name="pivot_tube",
            radius=0.01,
            length=0.14,
            xyz=(0.0, 0.0, 0.0),
            material=gondola_frame,
        )
        for hanger_index, y in enumerate((-0.045, 0.045)):
            seat.visual(
                Cylinder(radius=0.01, length=0.38),
                origin=Origin(xyz=(0.0, y, -0.19)),
                material=gondola_frame,
                name=f"hanger_{hanger_index}",
            )
        seat.visual(
            Box((0.12, 0.16, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, -0.39)),
            material=gondola_mat,
            name="seat_pan",
        )
        seat.visual(
            Box((0.02, 0.16, 0.16)),
            origin=Origin(xyz=(-0.05, 0.0, -0.30)),
            material=gondola_mat,
            name="backrest",
        )

        model.articulation(
            f"seat_pivot_{index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=seat,
            origin=Origin(
                xyz=(
                    math.sin(angle) * PIVOT_RADIUS,
                    0.0,
                    math.cos(angle) * PIVOT_RADIUS,
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=4.0),
            mimic=Mimic(joint=wheel_spin.name, multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel = object_model.get_part("wheel")
    seat_0 = object_model.get_part("seat_0")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.allow_overlap(
        "frame",
        "wheel",
        reason="The axle is intentionally represented as running through the bearing housings on the support frame.",
    )

    ctx.expect_gap(
        wheel,
        seat_0,
        axis="z",
        positive_elem="arm_0",
        negative_elem="seat_pan",
        min_gap=0.08,
        name="bottom gondola seat hangs below its radial arm",
    )

    rest_pos = ctx.part_world_position(seat_0)
    quarter_pos = None
    pan_extents = None
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        quarter_pos = ctx.part_world_position(seat_0)
        pan_aabb = ctx.part_element_world_aabb(seat_0, elem="seat_pan")
        if pan_aabb is not None:
            lower, upper = pan_aabb
            pan_extents = (
                upper[0] - lower[0],
                upper[1] - lower[1],
                upper[2] - lower[2],
            )

    ctx.check(
        "wheel carries a gondola around the axle",
        rest_pos is not None
        and quarter_pos is not None
        and quarter_pos[0] < rest_pos[0] - 1.1
        and quarter_pos[2] > rest_pos[2] + 1.1,
        details=f"rest={rest_pos}, quarter_turn={quarter_pos}",
    )
    ctx.check(
        "gondola stays level at quarter turn",
        pan_extents is not None and pan_extents[0] > 0.08 and pan_extents[2] < 0.04,
        details=f"seat_pan_extents={pan_extents}",
    )

    return ctx.report()


object_model = build_object_model()
