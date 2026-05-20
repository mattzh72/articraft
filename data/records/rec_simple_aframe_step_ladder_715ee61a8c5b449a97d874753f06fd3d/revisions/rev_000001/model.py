from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, sqrt

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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _lerp_point(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size: tuple[float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((size[0], size[1], _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _mirror_x(point: tuple[float, float, float]) -> tuple[float, float, float]:
    return (-point[0], point[1], point[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.82, 1.0))
    tread_gray = model.material("tread_gray", rgba=(0.63, 0.64, 0.66, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    rubber = model.material("rubber", rgba=(0.13, 0.13, 0.13, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.34, 0.98)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.12, -0.49)),
    )

    front_left_top = (-0.17, 0.04, -0.11)
    front_left_bottom = (-0.23, 0.22, -0.92)
    front_right_top = _mirror_x(front_left_top)
    front_right_bottom = _mirror_x(front_left_bottom)

    for a, b, name in [
        (front_left_top, front_left_bottom, "front_left_rail"),
        (front_right_top, front_right_bottom, "front_right_rail"),
    ]:
        _add_box_member(
            front_frame,
            a,
            b,
            size=(0.056, 0.030),
            material=aluminum,
            name=name,
        )

    step_specs = [
        (0.22, "upper_tread"),
        (0.48, "middle_tread"),
        (0.74, "lower_tread"),
    ]
    for t, name in step_specs:
        left_pt = _lerp_point(front_left_top, front_left_bottom, t)
        right_pt = _lerp_point(front_right_top, front_right_bottom, t)
        tread_center = _midpoint(left_pt, right_pt)
        front_frame.visual(
            Box((0.39, 0.105, 0.028)),
            origin=Origin(xyz=(0.0, tread_center[1], tread_center[2])),
            material=tread_gray,
            name=name,
        )
        for x_sign in (-1.0, 1.0):
            front_frame.visual(
                Box((0.024, 0.085, 0.060)),
                origin=Origin(
                    xyz=(
                        x_sign * 0.175,
                        tread_center[1],
                        tread_center[2] - 0.018,
                    )
                ),
                material=bracket_gray,
            )

    front_frame.visual(
        Box((0.38, 0.15, 0.09)),
        origin=Origin(xyz=(0.0, 0.11, -0.085)),
        material=bracket_gray,
        name="top_cap_body",
    )
    front_frame.visual(
        Box((0.30, 0.05, 0.07)),
        origin=Origin(xyz=(0.0, 0.03, -0.075)),
        material=bracket_gray,
    )
    front_frame.visual(
        Box((0.27, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, 0.11, -0.033)),
        material=tread_gray,
        name="top_platform",
    )
    front_frame.visual(
        Box((0.24, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.166, -0.034)),
        material=bracket_gray,
        name="top_front_lip",
    )
    for x_sign in (-1.0, 1.0):
        front_frame.visual(
            Box((0.064, 0.100, 0.060)),
            origin=Origin(xyz=(x_sign * 0.126, 0.055, -0.032)),
            material=bracket_gray,
            name="front_left_hinge_block" if x_sign < 0.0 else "front_right_hinge_block",
        )
        front_frame.visual(
            Cylinder(radius=0.022, length=0.060),
            origin=Origin(
                xyz=(x_sign * 0.126, 0.0, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=bracket_gray,
            name="front_left_hinge_lug" if x_sign < 0.0 else "front_right_hinge_lug",
        )

    for x_sign, foot_name in [(-1.0, "front_left_foot"), (1.0, "front_right_foot")]:
        front_frame.visual(
            Box((0.115, 0.060, 0.032)),
            origin=Origin(xyz=(x_sign * 0.23, 0.22, -0.936)),
            material=rubber,
            name=foot_name,
        )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.54, 0.96)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.20, -0.48)),
    )

    rear_left_top = (-0.20, -0.045, -0.10)
    rear_left_bottom = (-0.27, -0.40, -0.90)
    rear_right_top = _mirror_x(rear_left_top)
    rear_right_bottom = _mirror_x(rear_left_bottom)

    for a, b, name in [
        (rear_left_top, rear_left_bottom, "rear_left_rail"),
        (rear_right_top, rear_right_bottom, "rear_right_rail"),
    ]:
        _add_box_member(
            rear_frame,
            a,
            b,
            size=(0.050, 0.028),
            material=aluminum,
            name=name,
        )

    rear_frame.visual(
        Cylinder(radius=0.019, length=0.192),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=bracket_gray,
        name="rear_hinge_barrel",
    )
    rear_frame.visual(
        Box((0.29, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.075, -0.120)),
        material=bracket_gray,
        name="rear_top_brace",
    )

    rear_cross_levels = [
        (0.40, "rear_mid_brace"),
        (0.72, "rear_lower_brace"),
    ]
    for t, name in rear_cross_levels:
        left_pt = _lerp_point(rear_left_top, rear_left_bottom, t)
        right_pt = _lerp_point(rear_right_top, rear_right_bottom, t)
        brace_center = _midpoint(left_pt, right_pt)
        rear_frame.visual(
            Box((0.45, 0.032, 0.022)),
            origin=Origin(xyz=(0.0, brace_center[1], brace_center[2])),
            material=aluminum,
            name=name,
        )

    _add_box_member(
        rear_frame,
        (-0.165, -0.070, -0.135),
        (0.195, -0.300, -0.680),
        size=(0.018, 0.012),
        material=bracket_gray,
    )
    _add_box_member(
        rear_frame,
        (0.165, -0.070, -0.135),
        (-0.195, -0.300, -0.680),
        size=(0.018, 0.012),
        material=bracket_gray,
    )
    rear_frame.visual(
        Box((0.180, 0.036, 0.038)),
        origin=Origin(xyz=(0.0, -0.028, -0.031)),
        material=bracket_gray,
        name="rear_hinge_knuckle_block",
    )
    _add_box_member(
        rear_frame,
        (0.0, -0.042, -0.050),
        (0.0, -0.074, -0.118),
        size=(0.104, 0.024),
        material=bracket_gray,
        name="rear_hinge_web",
    )
    for x_sign in (-1.0, 1.0):
        rear_frame.visual(
            Box((0.100, 0.040, 0.115)),
            origin=Origin(xyz=(x_sign * 0.140, -0.055, -0.085)),
            material=bracket_gray,
            name="rear_left_hinge_cheek" if x_sign < 0.0 else "rear_right_hinge_cheek",
        )

    for x_sign, foot_name in [(-1.0, "rear_left_foot"), (1.0, "rear_right_foot")]:
        rear_frame.visual(
            Box((0.120, 0.060, 0.032)),
            origin=Origin(xyz=(x_sign * 0.27, -0.40, -0.916)),
            material=rubber,
            name=foot_name,
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    rear_hinge = object_model.get_articulation("rear_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="front_left_foot",
        negative_elem="rear_left_foot",
        min_gap=0.45,
        name="rear feet sit well behind the front feet in the open stance",
    )
    front_foot_aabb = ctx.part_element_world_aabb(front_frame, elem="front_left_foot")
    rear_foot_aabb = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    rest_center_y = None if rear_foot_aabb is None else (rear_foot_aabb[0][1] + rear_foot_aabb[1][1]) * 0.5
    ctx.check(
        "front and rear feet land at nearly the same ground height",
        front_foot_aabb is not None
        and rear_foot_aabb is not None
        and abs(front_foot_aabb[0][2] - rear_foot_aabb[0][2]) <= 0.03,
        details=f"front_foot_aabb={front_foot_aabb}, rear_foot_aabb={rear_foot_aabb}",
    )

    with ctx.pose({rear_hinge: 0.42}):
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="y",
            positive_elem="front_left_foot",
            negative_elem="rear_left_foot",
            min_gap=0.12,
            max_gap=0.28,
            name="folded pose pulls the rear support toward the front frame",
        )
        folded_foot_aabb = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
        folded_center_y = (
            None if folded_foot_aabb is None else (folded_foot_aabb[0][1] + folded_foot_aabb[1][1]) * 0.5
        )
        ctx.check(
            "rear foot swings forward when folding",
            rest_center_y is not None
            and folded_center_y is not None
            and folded_center_y > rest_center_y + 0.30,
            details=f"rest_center_y={rest_center_y}, folded_center_y={folded_center_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
