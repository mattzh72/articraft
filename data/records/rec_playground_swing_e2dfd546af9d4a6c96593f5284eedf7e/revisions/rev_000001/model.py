from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(
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


def _build_basket_mesh() -> object:
    basket = TorusGeometry(radius=0.46, tube=0.036, radial_segments=18, tubular_segments=72)
    basket.translate(0.0, 0.0, -0.18)
    basket.merge(
        TorusGeometry(radius=0.21, tube=0.013, radial_segments=14, tubular_segments=48).translate(
            0.0, 0.0, -0.23
        )
    )
    for index in range(8):
        angle = (2.0 * pi * index) / 8.0
        outer = (cos(angle) * 0.42, sin(angle) * 0.42, -0.18)
        mid = (cos(angle) * 0.29, sin(angle) * 0.29, -0.24)
        inner = (cos(angle) * 0.18, sin(angle) * 0.18, -0.23)
        basket.merge(
            tube_from_spline_points(
                [outer, mid, inner],
                radius=0.006,
                samples_per_segment=10,
                radial_segments=12,
                cap_ends=True,
            )
        )
    basket.merge(
        tube_from_spline_points(
            [(0.36, 0.0, -0.19), (0.12, 0.0, -0.235), (-0.12, 0.0, -0.235), (-0.36, 0.0, -0.19)],
            radius=0.0055,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        )
    )
    basket.merge(
        tube_from_spline_points(
            [(0.0, 0.36, -0.19), (0.0, 0.12, -0.235), (0.0, -0.12, -0.235), (0.0, -0.36, -0.19)],
            radius=0.0055,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        )
    )
    return _save_mesh("basket_rim_and_web", basket)


def _build_clip_mesh(side_y: float, mesh_name: str):
    clip = wire_from_points(
        [
            (0.060, side_y, 0.004),
            (0.086, side_y, -0.030),
            (0.086, side_y, -0.102),
            (0.0, side_y, -0.136),
            (-0.086, side_y, -0.102),
            (-0.086, side_y, -0.030),
            (-0.060, side_y, 0.004),
        ],
        radius=0.010,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.022,
        corner_segments=8,
    )
    clip.merge(
        tube_from_spline_points(
            [(0.060, side_y, 0.004), (0.22, side_y, -0.095), (0.355, side_y, -0.175)],
            radius=0.008,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        )
    )
    clip.merge(
        tube_from_spline_points(
            [(-0.060, side_y, 0.004), (-0.22, side_y, -0.095), (-0.355, side_y, -0.175)],
            radius=0.008,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        )
    )
    return _save_mesh(mesh_name, clip)


def _build_capture_fork_mesh():
    fork = wire_from_points(
        [
            (0.056, 0.0, -0.020),
            (0.056, 0.0, -0.090),
            (0.0, 0.0, -0.148),
            (-0.056, 0.0, -0.090),
            (-0.056, 0.0, -0.020),
        ],
        radius=0.011,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.024,
        corner_segments=8,
    )
    fork.merge(
        tube_from_spline_points(
            [(0.0, 0.0, -0.010), (0.0, 0.0, -0.075), (0.0, 0.0, -0.115)],
            radius=0.010,
            samples_per_segment=6,
            radial_segments=12,
            cap_ends=True,
        )
    )
    return _save_mesh("right_capture_fork", fork)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nest_playground_swing")

    powder_coat = model.material("powder_coat", rgba=(0.36, 0.42, 0.46, 1.0))
    hanger_steel = model.material("hanger_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    clamp_steel = model.material("clamp_steel", rgba=(0.56, 0.59, 0.62, 1.0))
    rim_green = model.material("rim_green", rgba=(0.22, 0.42, 0.20, 1.0))
    woven_black = model.material("woven_black", rgba=(0.10, 0.10, 0.11, 1.0))

    basket_mesh = _build_basket_mesh()

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((1.70, 2.38, 2.30)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )

    crossbeam_z = 2.22
    pivot_z = 2.155
    side_y = 0.96
    front_foot = 0.82
    rear_foot = -0.82
    support_frame.visual(
        Cylinder(radius=0.065, length=2.24),
        origin=Origin(xyz=(0.0, 0.0, crossbeam_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="crossbeam",
    )

    for y_sign in (-1.0, 1.0):
        side = y_sign * side_y
        apex_point = (0.0, side, pivot_z)
        front_point = (front_foot, side, 0.0)
        rear_point = (rear_foot, side, 0.0)
        spreader_front = (0.758, side, 0.17)
        spreader_rear = (-0.758, side, 0.17)
        _add_member(
            support_frame,
            front_point,
            apex_point,
            radius=0.050,
            material=powder_coat,
            name=f"front_leg_{'right' if y_sign > 0 else 'left'}",
        )
        _add_member(
            support_frame,
            rear_point,
            apex_point,
            radius=0.050,
            material=powder_coat,
            name=f"rear_leg_{'right' if y_sign > 0 else 'left'}",
        )
        _add_member(
            support_frame,
            spreader_rear,
            spreader_front,
            radius=0.040,
            material=powder_coat,
            name=f"base_spreader_{'right' if y_sign > 0 else 'left'}",
        )
        support_frame.visual(
            Box((0.12, 0.11, 0.10)),
            origin=Origin(xyz=(0.0, side, pivot_z + 0.01)),
            material=clamp_steel,
            name=f"apex_head_{'right' if y_sign > 0 else 'left'}",
        )
    support_frame.visual(
        Box((0.08, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.45, 2.095)),
        material=clamp_steel,
        name="left_top_lug",
    )
    support_frame.visual(
        Box((0.08, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.45, 2.095)),
        material=clamp_steel,
        name="right_top_lug",
    )

    swing_cradle = model.part("swing_cradle")
    swing_cradle.inertial = Inertial.from_geometry(
        Box((0.48, 0.96, 1.26)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.45, -0.64)),
    )
    swing_cradle.visual(
        Box((0.06, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=clamp_steel,
        name="left_top_receiver",
    )
    swing_cradle.visual(
        Box((0.04, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, 0.90, -0.065)),
        material=clamp_steel,
        name="right_top_receiver",
    )
    _add_member(
        swing_cradle,
        (0.0, 0.02, -0.105),
        (0.20, 0.23, -1.16),
        radius=0.022,
        material=hanger_steel,
        name="left_front_link",
    )
    _add_member(
        swing_cradle,
        (0.0, 0.02, -0.105),
        (-0.20, 0.23, -1.16),
        radius=0.022,
        material=hanger_steel,
        name="left_rear_link",
    )
    _add_member(
        swing_cradle,
        (0.0, 0.88, -0.105),
        (0.20, 0.67, -1.16),
        radius=0.022,
        material=hanger_steel,
        name="right_front_link",
    )
    _add_member(
        swing_cradle,
        (0.0, 0.88, -0.105),
        (-0.20, 0.67, -1.16),
        radius=0.022,
        material=hanger_steel,
        name="right_rear_link",
    )
    swing_cradle.visual(
        Cylinder(radius=0.012, length=0.92),
        origin=Origin(xyz=(0.0, 0.45, -0.105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hanger_steel,
        name="top_spreader",
    )
    swing_cradle.visual(
        Cylinder(radius=0.028, length=0.12),
        origin=Origin(xyz=(0.0, 0.23, -1.16), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clamp_steel,
        name="left_lower_pivot",
    )
    swing_cradle.visual(
        Cylinder(radius=0.028, length=0.12),
        origin=Origin(xyz=(0.0, 0.67, -1.16), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clamp_steel,
        name="right_lower_pivot",
    )
    _add_member(
        swing_cradle,
        (0.024, 0.23, -1.16),
        (0.20, 0.23, -1.16),
        radius=0.018,
        material=clamp_steel,
        name="left_front_clevis_arm",
    )
    _add_member(
        swing_cradle,
        (-0.024, 0.23, -1.16),
        (-0.20, 0.23, -1.16),
        radius=0.018,
        material=clamp_steel,
        name="left_rear_clevis_arm",
    )
    _add_member(
        swing_cradle,
        (0.024, 0.67, -1.16),
        (0.20, 0.67, -1.16),
        radius=0.018,
        material=clamp_steel,
        name="right_front_clevis_arm",
    )
    _add_member(
        swing_cradle,
        (-0.024, 0.67, -1.16),
        (-0.20, 0.67, -1.16),
        radius=0.018,
        material=clamp_steel,
        name="right_rear_clevis_arm",
    )
    _add_member(
        swing_cradle,
        (0.20, 0.23, -1.16),
        (0.20, 0.67, -1.16),
        radius=0.016,
        material=clamp_steel,
        name="front_tie_bar",
    )
    _add_member(
        swing_cradle,
        (-0.20, 0.23, -1.16),
        (-0.20, 0.67, -1.16),
        radius=0.016,
        material=clamp_steel,
        name="rear_tie_bar",
    )

    right_top_shackle = model.part("right_top_shackle")
    right_top_shackle.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.22)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )
    right_top_shackle.visual(
        Box((0.10, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=clamp_steel,
        name="upper_bridge",
    )
    right_top_shackle.visual(
        Box((0.018, 0.08, 0.14)),
        origin=Origin(xyz=(-0.040, 0.0, -0.075)),
        material=hanger_steel,
        name="left_fork_plate",
    )
    right_top_shackle.visual(
        Box((0.018, 0.08, 0.14)),
        origin=Origin(xyz=(0.040, 0.0, -0.075)),
        material=hanger_steel,
        name="right_fork_plate",
    )

    basket_seat = model.part("basket_seat")
    basket_seat.inertial = Inertial.from_geometry(
        Box((0.96, 0.96, 0.32)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )
    basket_seat.visual(
        basket_mesh,
        material=rim_green,
        name="basket_rim",
    )
    basket_seat.visual(
        Cylinder(radius=0.13, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        material=woven_black,
        name="center_pad",
    )
    basket_seat.visual(
        Cylinder(radius=0.031, length=0.14),
        origin=Origin(xyz=(0.0, -0.22, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clamp_steel,
        name="left_pivot_sleeve",
    )
    basket_seat.visual(
        Cylinder(radius=0.031, length=0.14),
        origin=Origin(xyz=(0.0, 0.22, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clamp_steel,
        name="right_pivot_sleeve",
    )
    for side_y_clip in (-0.22, 0.22):
        _add_member(
            basket_seat,
            (0.0, side_y_clip, 0.0),
            (0.41, side_y_clip, -0.18),
            radius=0.010,
            material=clamp_steel,
            name=f"{'left' if side_y_clip < 0 else 'right'}_front_basket_brace",
        )
        _add_member(
            basket_seat,
            (0.0, side_y_clip, 0.0),
            (-0.41, side_y_clip, -0.18),
            radius=0.010,
            material=clamp_steel,
            name=f"{'left' if side_y_clip < 0 else 'right'}_rear_basket_brace",
        )

    model.articulation(
        "left_top_swing",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=swing_cradle,
        origin=Origin(xyz=(0.0, -0.45, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "right_top_swing",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=right_top_shackle,
        origin=Origin(xyz=(0.0, 0.45, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "basket_level",
        ArticulationType.REVOLUTE,
        parent=swing_cradle,
        child=basket_seat,
        origin=Origin(xyz=(0.0, 0.45, -1.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.22, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    swing_cradle = object_model.get_part("swing_cradle")
    right_top_shackle = object_model.get_part("right_top_shackle")
    basket_seat = object_model.get_part("basket_seat")

    left_top_swing = object_model.get_articulation("left_top_swing")
    right_top_swing = object_model.get_articulation("right_top_swing")
    basket_level = object_model.get_articulation("basket_level")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="left_top_lug",
        elem_b="left_top_receiver",
        reason="Solid box proxies approximate a pinned left clevis at the crossbeam.",
    )
    ctx.allow_overlap(
        support_frame,
        right_top_shackle,
        elem_a="right_top_lug",
        elem_b="upper_bridge",
        reason="Solid box proxies approximate the right beam-side hanger clevis.",
    )
    ctx.allow_overlap(
        right_top_shackle,
        swing_cradle,
        elem_a="upper_bridge",
        elem_b="right_top_receiver",
        reason="The right shackle nests around the receiver as a simplified captive hanger.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_pivot_sleeve",
        elem_b="left_lower_pivot",
        reason="The left basket clip is modeled as a solid sleeve proxy around the lower pivot.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_pivot_sleeve",
        elem_b="left_rear_clevis_arm",
        reason="The left rear clevis arm is simplified as sharing the pivot sleeve volume near the pin boss.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_pivot_sleeve",
        elem_b="left_front_clevis_arm",
        reason="The left front clevis arm is simplified as sharing the pivot sleeve volume near the pin boss.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_pivot_sleeve",
        elem_b="right_lower_pivot",
        reason="The right basket clip is modeled as a solid sleeve proxy around the lower pivot.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_pivot_sleeve",
        elem_b="right_rear_clevis_arm",
        reason="The right rear clevis arm is simplified as sharing the pivot sleeve volume near the pin boss.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_pivot_sleeve",
        elem_b="right_front_clevis_arm",
        reason="The right front clevis arm is simplified as sharing the pivot sleeve volume near the pin boss.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_pivot_sleeve",
        elem_b="left_lower_yoke",
        reason="The left pivot sleeve sits inside the simplified lower yoke bracket geometry.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_pivot_sleeve",
        elem_b="right_lower_yoke",
        reason="The right pivot sleeve sits inside the simplified lower yoke bracket geometry.",
    )
    ctx.allow_overlap(
        right_top_shackle,
        support_frame,
        elem_a="left_fork_plate",
        elem_b="right_top_lug",
        reason="The captive right hanger plate wraps around the beam-side lug in this simplified clevis.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="right_top_lug",
        elem_b="right_top_receiver",
        reason="The right receiver nests around the beam lug as a simplified captive pivot proxy.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="left_top_lug",
        elem_b="top_spreader",
        reason="The top spreader tube passes through the simplified left clevis proxy at the beam.",
    )
    ctx.allow_overlap(
        right_top_shackle,
        support_frame,
        elem_a="right_fork_plate",
        elem_b="right_top_lug",
        reason="The right fork plate wraps the beam-side lug in the simplified captive hanger.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="left_top_lug",
        elem_b="left_rear_link",
        reason="The rear left hanger rod enters the simplified beam-side lug volume at the top pivot.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_rear_basket_brace",
        elem_b="left_rear_clevis_arm",
        reason="The rear basket brace is modeled as sharing the left pivot clevis centerline in a simplified proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_front_basket_brace",
        elem_b="left_front_clevis_arm",
        reason="The front left basket brace is modeled as sharing the left pivot clevis centerline in a simplified proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_front_basket_brace",
        elem_b="left_lower_pivot",
        reason="The front left basket brace meets the same simplified lower pivot pin centerline as the sleeve proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="left_rear_basket_brace",
        elem_b="left_lower_pivot",
        reason="The rear left basket brace meets the same simplified lower pivot pin centerline as the sleeve proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_rear_basket_brace",
        elem_b="right_rear_clevis_arm",
        reason="The rear basket brace is modeled as sharing the right pivot clevis centerline in a simplified proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_front_basket_brace",
        elem_b="right_front_clevis_arm",
        reason="The front right basket brace is modeled as sharing the right pivot clevis centerline in a simplified proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_front_basket_brace",
        elem_b="right_lower_pivot",
        reason="The front right basket brace meets the same simplified lower pivot pin centerline as the sleeve proxy.",
    )
    ctx.allow_overlap(
        basket_seat,
        swing_cradle,
        elem_a="right_rear_basket_brace",
        elem_b="right_lower_pivot",
        reason="The rear right basket brace meets the same simplified lower pivot pin centerline as the sleeve proxy.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="right_top_lug",
        elem_b="right_front_link",
        reason="The front right hanger rod enters the simplified beam-side lug volume at the top pivot.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="left_top_lug",
        elem_b="left_front_link",
        reason="The front left hanger rod enters the simplified beam-side lug volume at the top pivot.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="right_top_lug",
        elem_b="right_rear_link",
        reason="The rear right hanger rod enters the simplified beam-side lug volume at the top pivot.",
    )
    ctx.allow_overlap(
        support_frame,
        swing_cradle,
        elem_a="right_top_lug",
        elem_b="top_spreader",
        reason="The top spreader tube passes through the simplified right clevis proxy at the beam.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "left_top_joint_axis_is_beam_parallel",
        tuple(left_top_swing.axis) == (0.0, 1.0, 0.0),
        f"Unexpected left top axis: {left_top_swing.axis!r}",
    )
    ctx.check(
        "right_top_joint_axis_is_beam_parallel",
        tuple(right_top_swing.axis) == (0.0, 1.0, 0.0),
        f"Unexpected right top axis: {right_top_swing.axis!r}",
    )
    ctx.check(
        "basket_level_joint_axis_is_beam_parallel",
        tuple(basket_level.axis) == (0.0, 1.0, 0.0),
        f"Unexpected basket level axis: {basket_level.axis!r}",
    )

    ctx.expect_overlap(
        swing_cradle,
        support_frame,
        axes="xz",
        elem_a="left_top_receiver",
        elem_b="left_top_lug",
        min_overlap=0.050,
        name="left_top_receiver_reaches_beam_lug",
    )
    ctx.expect_overlap(
        right_top_shackle,
        support_frame,
        axes="xz",
        elem_a="upper_bridge",
        elem_b="right_top_lug",
        min_overlap=0.015,
        name="right_top_shackle_reaches_beam_lug",
    )
    ctx.expect_overlap(
        right_top_shackle,
        swing_cradle,
        axes="y",
        elem_a="upper_bridge",
        elem_b="right_top_receiver",
        min_overlap=0.040,
        name="right_top_capture_surrounds_receiver",
    )
    ctx.expect_overlap(
        basket_seat,
        swing_cradle,
        axes="xz",
        elem_a="left_pivot_sleeve",
        elem_b="left_lower_pivot",
        min_overlap=0.050,
        name="left_clip_stays_wrapped_around_lower_axle",
    )
    ctx.expect_overlap(
        basket_seat,
        swing_cradle,
        axes="xz",
        elem_a="right_pivot_sleeve",
        elem_b="right_lower_pivot",
        min_overlap=0.050,
        name="right_clip_stays_wrapped_around_lower_axle",
    )

    rim_rest_aabb = ctx.part_element_world_aabb(basket_seat, elem="basket_rim")
    assert rim_rest_aabb is not None
    ctx.check(
        "basket_hangs_well_above_ground_plane",
        rim_rest_aabb[0][2] > 0.70,
        f"Expected basket rim min z > 0.70 m, got {rim_rest_aabb[0][2]:.3f}",
    )
    basket_rest_pos = ctx.part_world_position(basket_seat)
    assert basket_rest_pos is not None
    with ctx.pose({left_top_swing: 0.42, right_top_swing: 0.42, basket_level: 0.10}):
        basket_swung_pos = ctx.part_world_position(basket_seat)
        assert basket_swung_pos is not None
        ctx.check(
            "swing_assembly_moves_forward_as_unit",
            abs(basket_swung_pos[0] - basket_rest_pos[0]) > 0.30,
            f"Expected basket x to advance by >0.30 m, got rest={basket_rest_pos}, swung={basket_swung_pos}",
        )
        swung_rim_aabb = ctx.part_element_world_aabb(basket_seat, elem="basket_rim")
        assert swung_rim_aabb is not None
        ctx.check(
            "basket_keeps_ground_clearance_when_swung",
            swung_rim_aabb[0][2] > 0.60,
            f"Expected swung basket rim min z > 0.60 m, got {swung_rim_aabb[0][2]:.3f}",
        )
        ctx.expect_overlap(
            right_top_shackle,
            swing_cradle,
            axes="y",
            elem_a="upper_bridge",
            elem_b="right_top_receiver",
            min_overlap=0.040,
            name="right_top_capture_stays_aligned_in_swung_pose",
        )
        ctx.expect_overlap(
            basket_seat,
            swing_cradle,
            axes="xz",
            elem_a="left_pivot_sleeve",
            elem_b="left_lower_pivot",
            min_overlap=0.050,
            name="left_clip_tracks_axle_in_swung_pose",
        )
        ctx.expect_overlap(
            basket_seat,
            swing_cradle,
            axes="xz",
            elem_a="right_pivot_sleeve",
            elem_b="right_lower_pivot",
            min_overlap=0.050,
            name="right_clip_tracks_axle_in_swung_pose",
        )

    rim_rest_height = rim_rest_aabb[1][2] - rim_rest_aabb[0][2]
    with ctx.pose({basket_level: 0.18}):
        rim_tilted_aabb = ctx.part_element_world_aabb(basket_seat, elem="basket_rim")
        assert rim_tilted_aabb is not None
        rim_tilted_height = rim_tilted_aabb[1][2] - rim_tilted_aabb[0][2]
        ctx.check(
            "basket_can_pitch_on_lower_pivots",
            rim_tilted_height > rim_rest_height + 0.10,
            f"Expected tilted rim height to grow by >0.10 m, got rest={rim_rest_height:.3f}, tilted={rim_tilted_height:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
