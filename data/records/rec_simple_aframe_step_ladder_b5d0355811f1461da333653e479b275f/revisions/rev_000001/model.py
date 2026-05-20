from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
    return hypot(hypot(b[0] - a[0], b[1] - a[1]), b[2] - a[2])


def _rpy_for_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.18, 0.18, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.18, 1.06)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.02, 0.53)),
    )

    _add_box_member(
        front_frame,
        (-0.175, -0.004, 1.00),
        (-0.215, -0.060, 0.030),
        width=0.036,
        depth=0.028,
        material=aluminum,
        name="left_front_rail",
    )
    _add_box_member(
        front_frame,
        (0.175, -0.004, 1.00),
        (0.215, -0.060, 0.030),
        width=0.036,
        depth=0.028,
        material=aluminum,
        name="right_front_rail",
    )

    front_frame.visual(
        Box((0.360, 0.110, 0.060)),
        origin=Origin(xyz=(0.0, 0.015, 1.020)),
        material=dark_plastic,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.080, 0.010, 0.024)),
        origin=Origin(xyz=(-0.160, 0.067, 1.014), rpy=(0.42, 0.0, 0.0)),
        material=steel,
        name="left_hinge_bracket",
    )
    front_frame.visual(
        Box((0.080, 0.010, 0.024)),
        origin=Origin(xyz=(0.160, 0.067, 1.014), rpy=(0.42, 0.0, 0.0)),
        material=steel,
        name="right_hinge_bracket",
    )
    front_frame.visual(
        Box((0.395, 0.105, 0.028)),
        origin=Origin(xyz=(0.0, -0.048, 0.230)),
        material=aluminum,
        name="step_1",
    )
    front_frame.visual(
        Box((0.385, 0.105, 0.028)),
        origin=Origin(xyz=(0.0, -0.038, 0.455)),
        material=aluminum,
        name="step_2",
    )
    front_frame.visual(
        Box((0.375, 0.102, 0.028)),
        origin=Origin(xyz=(0.0, -0.027, 0.680)),
        material=aluminum,
        name="step_3",
    )
    front_frame.visual(
        Box((0.365, 0.100, 0.028)),
        origin=Origin(xyz=(0.0, -0.016, 0.900)),
        material=aluminum,
        name="step_4",
    )
    front_frame.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(-0.215, -0.060, 0.021)),
        material=rubber,
        name="front_foot_left",
    )
    front_frame.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.215, -0.060, 0.021)),
        material=rubber,
        name="front_foot_right",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 1.00)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.16, -0.50)),
    )

    _add_box_member(
        rear_frame,
        (-0.160, 0.000, -0.012),
        (-0.205, 0.000, -0.965),
        width=0.032,
        depth=0.025,
        material=aluminum,
        name="left_rear_rail",
    )
    _add_box_member(
        rear_frame,
        (0.160, 0.000, -0.012),
        (0.205, 0.000, -0.965),
        width=0.032,
        depth=0.025,
        material=aluminum,
        name="right_rear_rail",
    )
    rear_frame.visual(
        Box((0.338, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.000, -0.075)),
        material=steel,
        name="rear_top_crossbar",
    )
    rear_frame.visual(
        Box((0.355, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.000, -0.430)),
        material=steel,
        name="rear_mid_crossbar",
    )
    rear_frame.visual(
        Box((0.390, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.000, -0.760)),
        material=steel,
        name="rear_lower_crossbar",
    )
    rear_frame.visual(
        Box((0.050, 0.032, 0.016)),
        origin=Origin(xyz=(-0.205, 0.000, -0.972)),
        material=rubber,
        name="rear_foot_left",
    )
    rear_frame.visual(
        Box((0.050, 0.032, 0.016)),
        origin=Origin(xyz=(0.205, 0.000, -0.972)),
        material=rubber,
        name="rear_foot_right",
    )
    rear_frame.visual(
        Box((0.070, 0.022, 0.018)),
        origin=Origin(xyz=(-0.160, 0.000, 0.000)),
        material=steel,
        name="left_hinge_tab",
    )
    rear_frame.visual(
        Box((0.070, 0.022, 0.018)),
        origin=Origin(xyz=(0.160, 0.000, 0.000)),
        material=steel,
        name="right_hinge_tab",
    )
    _add_box_member(
        rear_frame,
        (-0.160, 0.000, -0.004),
        (-0.182, 0.000, -0.090),
        width=0.018,
        depth=0.018,
        material=steel,
        name="left_hinge_gusset",
    )
    _add_box_member(
        rear_frame,
        (0.160, 0.000, -0.004),
        (0.182, 0.000, -0.090),
        width=0.018,
        depth=0.018,
        material=steel,
        name="right_hinge_gusset",
    )

    model.articulation(
        "rear_frame_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.082, 1.020), rpy=(0.42, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.36, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_frame_fold")
    open_q = hinge.motion_limits.upper if hinge.motion_limits is not None else 0.0
    folded_q = hinge.motion_limits.lower if hinge.motion_limits is not None else -0.36

    with ctx.pose({hinge: open_q}):
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="rear_foot_left",
            negative_elem="front_foot_left",
            min_gap=0.45,
            name="rear left foot stands well behind front left foot when open",
        )
        open_front_aabb = ctx.part_world_aabb(front_frame)
        open_rear_aabb = ctx.part_world_aabb(rear_frame)

    with ctx.pose({hinge: folded_q}):
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="rear_foot_left",
            negative_elem="front_foot_left",
            min_gap=0.14,
            max_gap=0.22,
            name="rear left foot tucks close behind front left foot when folded",
        )
        folded_front_aabb = ctx.part_world_aabb(front_frame)
        folded_rear_aabb = ctx.part_world_aabb(rear_frame)

    open_depth = None
    folded_depth = None
    if open_front_aabb is not None and open_rear_aabb is not None:
        open_depth = max(open_front_aabb[1][1], open_rear_aabb[1][1]) - min(
            open_front_aabb[0][1], open_rear_aabb[0][1]
        )
    if folded_front_aabb is not None and folded_rear_aabb is not None:
        folded_depth = max(folded_front_aabb[1][1], folded_rear_aabb[1][1]) - min(
            folded_front_aabb[0][1], folded_rear_aabb[0][1]
        )

    ctx.check(
        "folded ladder stores in a much narrower depth envelope",
        open_depth is not None
        and folded_depth is not None
        and folded_depth < 0.30
        and folded_depth < open_depth - 0.25,
        details=f"open_depth={open_depth}, folded_depth={folded_depth}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
