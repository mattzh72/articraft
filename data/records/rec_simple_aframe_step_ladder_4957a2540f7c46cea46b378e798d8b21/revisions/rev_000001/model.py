from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

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


def _rpy_for_z_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
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
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_member(a, b)),
        material=material,
        name=name,
    )


def _point_at_z(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    z: float,
) -> tuple[float, float, float]:
    t = (z - a[2]) / (b[2] - a[2])
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        z,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    plastic = model.material("plastic", rgba=(0.86, 0.33, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))

    rail_width = 0.040
    rail_depth = 0.022

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.28, 1.12)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.06, 0.56)),
    )

    front_left_foot = (-0.275, 0.150, 0.026)
    front_right_foot = (0.275, 0.150, 0.026)
    front_left_top = (-0.170, 0.000, 1.040)
    front_right_top = (0.170, 0.000, 1.040)

    _add_box_member(
        front_frame,
        front_left_foot,
        front_left_top,
        width=rail_width,
        depth=rail_depth,
        material=aluminum,
        name="left_front_rail",
    )
    _add_box_member(
        front_frame,
        front_right_foot,
        front_right_top,
        width=rail_width,
        depth=rail_depth,
        material=aluminum,
        name="right_front_rail",
    )

    front_frame.visual(
        Box((0.400, 0.120, 0.058)),
        origin=Origin(xyz=(0.0, 0.050, 1.068)),
        material=plastic,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.356, 0.026, 0.072)),
        origin=Origin(xyz=(0.0, 0.088, 1.016)),
        material=plastic,
        name="top_cap_front_apron",
    )
    front_frame.visual(
        Box((0.034, 0.022, 0.072)),
        origin=Origin(xyz=(-0.205, -0.021, 1.040)),
        material=steel,
        name="left_hinge_cheek",
    )
    front_frame.visual(
        Box((0.034, 0.022, 0.072)),
        origin=Origin(xyz=(0.205, -0.021, 1.040)),
        material=steel,
        name="right_hinge_cheek",
    )

    for index, step_z in enumerate((0.250, 0.475, 0.700, 0.865), start=1):
        left_step_anchor = _point_at_z(front_left_foot, front_left_top, step_z)
        right_step_anchor = _point_at_z(front_right_foot, front_right_top, step_z)
        tread_width = (right_step_anchor[0] - left_step_anchor[0]) - 0.030
        tread_y = ((left_step_anchor[1] + right_step_anchor[1]) * 0.5) + 0.020
        front_frame.visual(
            Box((tread_width, 0.105, 0.030)),
            origin=Origin(xyz=(0.0, tread_y, step_z)),
            material=aluminum,
            name=f"tread_{index}",
        )
        front_frame.visual(
            Box((tread_width * 0.92, 0.020, 0.032)),
            origin=Origin(xyz=(0.0, tread_y + 0.043, step_z - 0.001)),
            material=steel,
            name=f"tread_lip_{index}",
        )

    front_frame.visual(
        Box((0.505, 0.020, 0.034)),
        origin=Origin(xyz=(0.0, 0.096, 0.205)),
        material=steel,
        name="lower_front_brace",
    )
    front_frame.visual(
        Box((0.075, 0.082, 0.032)),
        origin=Origin(xyz=(-0.275, 0.155, 0.016)),
        material=rubber,
        name="left_front_foot_pad",
    )
    front_frame.visual(
        Box((0.075, 0.082, 0.032)),
        origin=Origin(xyz=(0.275, 0.155, 0.016)),
        material=rubber,
        name="right_front_foot_pad",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.12, 1.05)),
        mass=4.5,
        origin=Origin(xyz=(0.0, -0.015, -0.52)),
    )

    rear_left_top = (-0.170, -0.045, -0.028)
    rear_right_top = (0.170, -0.045, -0.028)
    rear_left_foot = (-0.245, -0.037, -1.005)
    rear_right_foot = (0.245, -0.037, -1.005)

    _add_box_member(
        rear_frame,
        rear_left_top,
        rear_left_foot,
        width=0.034,
        depth=0.018,
        material=aluminum,
        name="left_rear_rail",
    )
    _add_box_member(
        rear_frame,
        rear_right_top,
        rear_right_foot,
        width=0.034,
        depth=0.018,
        material=aluminum,
        name="right_rear_rail",
    )

    rear_frame.visual(
        Box((0.344, 0.024, 0.052)),
        origin=Origin(xyz=(0.0, -0.050, -0.028)),
        material=steel,
        name="rear_upper_brace",
    )
    rear_frame.visual(
        Box((0.395, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, -0.047, -0.440)),
        material=steel,
        name="rear_mid_brace",
    )
    rear_frame.visual(
        Box((0.450, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.041, -0.790)),
        material=steel,
        name="rear_lower_brace",
    )
    rear_frame.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(-0.176, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_pivot_boss",
    )
    rear_frame.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(0.176, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_pivot_boss",
    )
    rear_frame.visual(
        Box((0.020, 0.038, 0.038)),
        origin=Origin(xyz=(-0.164, -0.030, -0.018)),
        material=steel,
        name="left_hinge_web",
    )
    rear_frame.visual(
        Box((0.020, 0.038, 0.038)),
        origin=Origin(xyz=(0.164, -0.030, -0.018)),
        material=steel,
        name="right_hinge_web",
    )
    rear_frame.visual(
        Box((0.070, 0.072, 0.028)),
        origin=Origin(xyz=(-0.245, -0.037, -1.018)),
        material=rubber,
        name="left_rear_foot_pad",
    )
    rear_frame.visual(
        Box((0.070, 0.072, 0.028)),
        origin=Origin(xyz=(0.245, -0.037, -1.018)),
        material=rubber,
        name="right_rear_foot_pad",
    )

    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.036, 1.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_frame_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            front_frame,
            rear_frame,
            elem_a="left_hinge_cheek",
            elem_b="left_pivot_boss",
            contact_tol=1e-6,
            name="left hinge barrel stays seated between the cap cheeks",
        )
        ctx.expect_contact(
            front_frame,
            rear_frame,
            elem_a="right_hinge_cheek",
            elem_b="right_pivot_boss",
            contact_tol=1e-6,
            name="right hinge barrel stays seated between the cap cheeks",
        )

    rear_rest_aabb = ctx.part_world_aabb(rear_frame)
    with ctx.pose({hinge: 0.50}):
        rear_open_aabb = ctx.part_world_aabb(rear_frame)
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="y",
            positive_elem="lower_front_brace",
            negative_elem="rear_lower_brace",
            min_gap=0.10,
            name="rear support swings backward when opened",
        )

    ctx.check(
        "rear frame stays close in folded silhouette",
        rear_rest_aabb is not None
        and rear_rest_aabb[1][1] < 0.05,
        details=f"rear_rest_aabb={rear_rest_aabb}",
    )
    ctx.check(
        "rear frame opens farther backward than folded pose",
        rear_rest_aabb is not None
        and rear_open_aabb is not None
        and rear_open_aabb[0][1] < rear_rest_aabb[0][1] - 0.20,
        details=f"rest={rear_rest_aabb}, open={rear_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
