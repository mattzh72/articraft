from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

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


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    mid = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _tube_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
):
    origin, length = _segment_pose(start, end)
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _build_wing_frame(
    part,
    *,
    span: float,
    depth: float,
    frame_radius: float,
    rod_radius: float,
    frame_material,
    rod_material,
) -> None:
    half_depth = depth * 0.5
    hinge_gap = 0.24
    inner_drop = 0.024
    _tube_between(
        part,
        (0.0, -hinge_gap * 0.5, 0.0),
        (0.0, hinge_gap * 0.5, 0.0),
        radius=frame_radius,
        material=frame_material,
        name="hinge_rail",
    )
    _tube_between(
        part,
        (0.0, -half_depth, -inner_drop),
        (0.0, half_depth, -inner_drop),
        radius=frame_radius,
        material=frame_material,
        name="inner_rail",
    )
    _tube_between(
        part,
        (0.0, -half_depth, -span),
        (0.0, half_depth, -span),
        radius=frame_radius,
        material=frame_material,
        name="outer_rail",
    )
    _tube_between(
        part,
        (0.0, -half_depth, -inner_drop),
        (0.0, -half_depth, -span),
        radius=frame_radius,
        material=frame_material,
        name="rear_end_rail",
    )
    _tube_between(
        part,
        (0.0, half_depth, -inner_drop),
        (0.0, half_depth, -span),
        radius=frame_radius,
        material=frame_material,
        name="front_end_rail",
    )
    for index, y in enumerate((-0.10, 0.10)):
        _tube_between(
            part,
            (0.0, y, 0.0),
            (0.0, y, -inner_drop),
            radius=rod_radius,
            material=frame_material,
            name=f"hinge_bracket_{index}",
        )
    for index, y in enumerate((-0.18, -0.06, 0.06, 0.18)):
        _tube_between(
            part,
            (0.0, y, -0.012),
            (0.0, y, -span + 0.012),
            radius=rod_radius,
            material=rod_material,
            name=f"wing_rod_{index}",
        )


def _build_lower_support_frame(
    part,
    *,
    width: float,
    depth: float,
    frame_radius: float,
    rod_radius: float,
    frame_material,
    rod_material,
) -> None:
    half_width = width * 0.5
    hinge_span = 0.395
    inner_drop = 0.022
    _tube_between(
        part,
        (-hinge_span * 0.5, 0.0, 0.0),
        (hinge_span * 0.5, 0.0, 0.0),
        radius=frame_radius,
        material=frame_material,
        name="hinge_rail",
    )
    _tube_between(
        part,
        (-half_width, 0.0, -inner_drop),
        (half_width, 0.0, -inner_drop),
        radius=frame_radius,
        material=frame_material,
        name="inner_rail",
    )
    _tube_between(
        part,
        (-half_width, 0.0, -depth),
        (half_width, 0.0, -depth),
        radius=frame_radius,
        material=frame_material,
        name="outer_rail",
    )
    _tube_between(
        part,
        (-half_width, 0.0, -inner_drop),
        (-half_width, 0.0, -depth),
        radius=frame_radius,
        material=frame_material,
        name="left_side_rail",
    )
    _tube_between(
        part,
        (half_width, 0.0, -inner_drop),
        (half_width, 0.0, -depth),
        radius=frame_radius,
        material=frame_material,
        name="right_side_rail",
    )
    for index, x in enumerate((-0.18, 0.18)):
        _tube_between(
            part,
            (x, 0.0, 0.0),
            (x, 0.0, -inner_drop),
            radius=rod_radius,
            material=frame_material,
            name=f"hinge_bracket_{index}",
        )
    for index, z in enumerate((-0.06, -0.12, -0.18, -0.24)):
        _tube_between(
            part,
            (-half_width, 0.0, z),
            (half_width, 0.0, z),
            radius=rod_radius,
            material=rod_material,
            name=f"support_rod_{index}",
        )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    frame_white = model.material("frame_white", rgba=(0.93, 0.94, 0.95, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.54, 0.56, 0.60, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.23, 0.24, 0.25, 1.0))

    top_width = 0.68
    top_depth = 0.56
    top_z = 0.86
    tube_r = 0.008
    rod_r = 0.0055
    leg_r = 0.009

    half_w = top_width * 0.5
    half_d = top_depth * 0.5

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.76, 0.92)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    # Upper fixed drying deck.
    _tube_between(
        main_frame,
        (-half_w, -half_d, top_z),
        (-half_w, half_d, top_z),
        radius=tube_r,
        material=frame_white,
        name="left_top_side",
    )
    _tube_between(
        main_frame,
        (half_w, -half_d, top_z),
        (half_w, half_d, top_z),
        radius=tube_r,
        material=frame_white,
        name="right_top_side",
    )
    _tube_between(
        main_frame,
        (-half_w, -half_d, top_z),
        (half_w, -half_d, top_z),
        radius=tube_r,
        material=frame_white,
        name="rear_top_cross",
    )
    _tube_between(
        main_frame,
        (-half_w, half_d, top_z),
        (half_w, half_d, top_z),
        radius=tube_r,
        material=frame_white,
        name="front_top_cross",
    )

    for index, y in enumerate((-0.20, -0.12, -0.04, 0.04, 0.12, 0.20)):
        _tube_between(
            main_frame,
            (-half_w - 0.010, y, top_z - 0.012),
            (half_w + 0.010, y, top_z - 0.012),
            radius=rod_r,
            material=steel_gray,
            name=f"center_rod_{index}",
        )

    # Legs and lower braces for a stable floor stand.
    front_left_foot = (-0.31, 0.36, 0.03)
    rear_left_foot = (-0.31, -0.36, 0.03)
    front_right_foot = (0.31, 0.36, 0.03)
    rear_right_foot = (0.31, -0.36, 0.03)

    _tube_between(
        main_frame,
        (-half_w, half_d, top_z + 0.010),
        front_left_foot,
        radius=leg_r,
        material=frame_white,
        name="front_left_leg",
    )
    _tube_between(
        main_frame,
        (-half_w, -half_d, top_z + 0.010),
        rear_left_foot,
        radius=leg_r,
        material=frame_white,
        name="rear_left_leg",
    )
    _tube_between(
        main_frame,
        (half_w, half_d, top_z + 0.010),
        front_right_foot,
        radius=leg_r,
        material=frame_white,
        name="front_right_leg",
    )
    _tube_between(
        main_frame,
        (half_w, -half_d, top_z + 0.010),
        rear_right_foot,
        radius=leg_r,
        material=frame_white,
        name="rear_right_leg",
    )

    _tube_between(
        main_frame,
        front_left_foot,
        front_right_foot,
        radius=tube_r,
        material=frame_white,
        name="front_floor_bar",
    )
    _tube_between(
        main_frame,
        rear_left_foot,
        rear_right_foot,
        radius=tube_r,
        material=frame_white,
        name="rear_floor_bar",
    )
    _tube_between(
        main_frame,
        (-0.326, -0.318, 0.48),
        (-0.326, 0.318, 0.48),
        radius=tube_r,
        material=frame_white,
        name="left_mid_side",
    )
    _tube_between(
        main_frame,
        (0.326, -0.318, 0.48),
        (0.326, 0.318, 0.48),
        radius=tube_r,
        material=frame_white,
        name="right_mid_side",
    )
    _tube_between(
        main_frame,
        (-0.326, -0.23, 0.48),
        (0.326, -0.23, 0.48),
        radius=tube_r,
        material=frame_white,
        name="rear_support_cross",
    )

    # Hinge barrels and feet.
    for sign, x in ((-1.0, -half_w - 0.014), (1.0, half_w + 0.014)):
        for idx, y in enumerate((-0.16, 0.16)):
            main_frame.visual(
                Cylinder(radius=0.0065, length=0.080),
                origin=Origin(xyz=(x, y, top_z - 0.002), rpy=(1.5707963267948966, 0.0, 0.0)),
                material=hinge_gray,
                name=f"{'left' if sign < 0.0 else 'right'}_wing_hinge_barrel_{idx}",
            )
            bridge_x = x + 0.008 * (-sign)
            main_frame.visual(
                Box((0.016, 0.022, 0.012)),
                origin=Origin(xyz=((x + bridge_x) * 0.5, y, top_z - 0.002)),
                material=hinge_gray,
                name=f"{'left' if sign < 0.0 else 'right'}_wing_hinge_bridge_{idx}",
            )

    for idx, x in enumerate((-0.23, 0.23)):
        main_frame.visual(
            Cylinder(radius=0.007, length=0.065),
            origin=Origin(
                xyz=(x, -0.23, 0.48),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=hinge_gray,
            name=f"lower_hinge_barrel_{idx}",
        )

    for name, xyz in (
        ("front_left_foot_pad", front_left_foot),
        ("rear_left_foot_pad", rear_left_foot),
        ("front_right_foot_pad", front_right_foot),
        ("rear_right_foot_pad", rear_right_foot),
    ):
        main_frame.visual(
            Box((0.030, 0.020, 0.016)),
            origin=Origin(xyz=(xyz[0], xyz[1], 0.030)),
            material=foot_gray,
            name=name,
        )

    wing_span = 0.29
    wing_depth = top_depth
    wing_joint_x = half_w + 0.022

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((0.06, wing_depth, wing_span)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -wing_span * 0.5)),
    )
    _build_wing_frame(
        left_wing,
        span=wing_span,
        depth=wing_depth,
        frame_radius=0.007,
        rod_radius=0.005,
        frame_material=frame_white,
        rod_material=steel_gray,
    )

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((0.06, wing_depth, wing_span)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -wing_span * 0.5)),
    )
    _build_wing_frame(
        right_wing,
        span=wing_span,
        depth=wing_depth,
        frame_radius=0.007,
        rod_radius=0.005,
        frame_material=frame_white,
        rod_material=steel_gray,
    )

    lower_support = model.part("lower_support")
    lower_support.inertial = Inertial.from_geometry(
        Box((0.62, 0.06, 0.30)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
    )
    _build_lower_support_frame(
        lower_support,
        width=0.62,
        depth=0.30,
        frame_radius=0.007,
        rod_radius=0.005,
        frame_material=frame_white,
        rod_material=steel_gray,
    )

    model.articulation(
        "main_to_left_wing",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=left_wing,
        origin=Origin(xyz=(-wing_joint_x, 0.0, top_z - 0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "main_to_right_wing",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=right_wing,
        origin=Origin(xyz=(wing_joint_x, 0.0, top_z - 0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "main_to_lower_support",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=lower_support,
        origin=Origin(xyz=(0.0, -0.216, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=1.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    lower_support = object_model.get_part("lower_support")
    left_joint = object_model.get_articulation("main_to_left_wing")
    right_joint = object_model.get_articulation("main_to_right_wing")
    lower_joint = object_model.get_articulation("main_to_lower_support")

    ctx.expect_gap(
        main_frame,
        left_wing,
        axis="x",
        positive_elem="left_top_side",
        negative_elem="hinge_rail",
        min_gap=0.002,
        max_gap=0.020,
        name="left wing folds close beside the fixed deck",
    )
    ctx.expect_gap(
        right_wing,
        main_frame,
        axis="x",
        positive_elem="hinge_rail",
        negative_elem="right_top_side",
        min_gap=0.002,
        max_gap=0.020,
        name="right wing folds close beside the fixed deck",
    )
    ctx.expect_gap(
        lower_support,
        main_frame,
        axis="y",
        positive_elem="hinge_rail",
        negative_elem="rear_support_cross",
        max_penetration=0.002,
        max_gap=0.015,
        name="lower support folds close to the rear hinge rail",
    )

    left_outer_rest = ctx.part_element_world_aabb(left_wing, elem="outer_rail")
    right_outer_rest = ctx.part_element_world_aabb(right_wing, elem="outer_rail")
    lower_outer_rest = ctx.part_element_world_aabb(lower_support, elem="outer_rail")

    with ctx.pose(
        {
            left_joint: left_joint.motion_limits.upper,
            right_joint: right_joint.motion_limits.upper,
            lower_joint: lower_joint.motion_limits.upper,
        }
    ):
        ctx.expect_gap(
            main_frame,
            left_wing,
            axis="x",
            positive_elem="left_top_side",
            negative_elem="outer_rail",
            min_gap=0.22,
            name="left wing opens outward from the main frame",
        )
        ctx.expect_gap(
            right_wing,
            main_frame,
            axis="x",
            positive_elem="outer_rail",
            negative_elem="right_top_side",
            min_gap=0.22,
            name="right wing opens outward from the main frame",
        )
        ctx.expect_gap(
            lower_support,
            main_frame,
            axis="y",
            positive_elem="outer_rail",
            negative_elem="rear_support_cross",
            min_gap=0.24,
            name="lower support swings forward into use",
        )

        left_outer_open = ctx.part_element_world_aabb(left_wing, elem="outer_rail")
        right_outer_open = ctx.part_element_world_aabb(right_wing, elem="outer_rail")
        lower_outer_open = ctx.part_element_world_aabb(lower_support, elem="outer_rail")

    left_rest_center = _aabb_center(left_outer_rest)
    left_open_center = _aabb_center(left_outer_open)
    right_rest_center = _aabb_center(right_outer_rest)
    right_open_center = _aabb_center(right_outer_open)
    lower_rest_center = _aabb_center(lower_outer_rest)
    lower_open_center = _aabb_center(lower_outer_open)

    ctx.check(
        "left wing outer rail rises and shifts left when opened",
        left_rest_center is not None
        and left_open_center is not None
        and left_open_center[0] < left_rest_center[0] - 0.18
        and left_open_center[2] > left_rest_center[2] + 0.22,
        details=f"rest={left_rest_center}, open={left_open_center}",
    )
    ctx.check(
        "right wing outer rail rises and shifts right when opened",
        right_rest_center is not None
        and right_open_center is not None
        and right_open_center[0] > right_rest_center[0] + 0.18
        and right_open_center[2] > right_rest_center[2] + 0.22,
        details=f"rest={right_rest_center}, open={right_open_center}",
    )
    ctx.check(
        "lower support outer rail moves forward when opened",
        lower_rest_center is not None
        and lower_open_center is not None
        and lower_open_center[1] > lower_rest_center[1] + 0.22
        and lower_open_center[2] > lower_rest_center[2] + 0.24,
        details=f"rest={lower_rest_center}, open={lower_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
