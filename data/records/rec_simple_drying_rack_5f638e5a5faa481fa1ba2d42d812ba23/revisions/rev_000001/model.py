from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi

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


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
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


def _build_wing(part, *, side_sign: float, steel, zinc, polymer, name_prefix: str) -> None:
    sx = lambda value: side_sign * value

    front_leaf_end = (sx(0.10), -0.07, -0.02)
    rear_leaf_end = (sx(0.10), 0.07, -0.02)
    top_inner_front = (sx(0.10), -0.22, -0.04)
    top_inner_rear = (sx(0.10), 0.22, -0.04)
    outer_top_front = (sx(0.34), -0.22, -0.06)
    outer_top_rear = (sx(0.34), 0.22, -0.06)
    outer_mid_front = (sx(0.34), -0.22, -0.31)
    outer_mid_rear = (sx(0.34), 0.22, -0.31)
    outer_bottom_front = (sx(0.34), -0.22, -0.56)
    outer_bottom_rear = (sx(0.34), 0.22, -0.56)
    inner_bottom_front = (sx(0.10), -0.22, -0.50)
    inner_bottom_rear = (sx(0.10), 0.22, -0.50)

    part.visual(
        Cylinder(radius=0.015, length=0.160),
        origin=Origin(rpy=(0.0, pi / 2.0, pi / 2.0)),
        material=zinc,
        name=f"{name_prefix}_wing_hinge_barrel",
    )
    part.visual(
        Box((0.12, 0.03, 0.035)),
        origin=Origin(xyz=(sx(0.060), -0.055, -0.010)),
        material=steel,
        name=f"{name_prefix}_front_leaf_plate",
    )
    part.visual(
        Box((0.12, 0.03, 0.035)),
        origin=Origin(xyz=(sx(0.060), 0.055, -0.010)),
        material=steel,
        name=f"{name_prefix}_rear_leaf_plate",
    )

    for a, b, radius, material, name in (
        (front_leaf_end, top_inner_front, 0.008, steel, None),
        (rear_leaf_end, top_inner_rear, 0.008, steel, None),
        (top_inner_front, outer_top_front, 0.011, steel, None),
        (top_inner_rear, outer_top_rear, 0.011, steel, None),
        (outer_top_front, outer_bottom_front, 0.013, steel, None),
        (outer_top_rear, outer_bottom_rear, 0.013, steel, None),
        (outer_bottom_front, inner_bottom_front, 0.011, steel, None),
        (outer_bottom_rear, inner_bottom_rear, 0.011, steel, None),
        (inner_bottom_front, top_inner_front, 0.009, steel, None),
        (inner_bottom_rear, top_inner_rear, 0.009, steel, None),
        (top_inner_front, top_inner_rear, 0.011, steel, None),
        (outer_top_front, outer_top_rear, 0.011, steel, None),
        (outer_bottom_front, outer_bottom_rear, 0.011, steel, None),
        (inner_bottom_front, inner_bottom_rear, 0.011, steel, None),
        (outer_mid_front, outer_mid_rear, 0.012, steel, f"{name_prefix}_outer_side_rail"),
        ((sx(0.20), -0.22, -0.26), (sx(0.20), 0.22, -0.26), 0.007, zinc, None),
        ((sx(0.27), -0.22, -0.38), (sx(0.27), 0.22, -0.38), 0.007, zinc, None),
        ((sx(0.32), -0.22, -0.50), (sx(0.32), 0.22, -0.50), 0.007, zinc, None),
        ((sx(0.20), -0.22, -0.26), top_inner_front, 0.005, zinc, None),
        ((sx(0.20), 0.22, -0.26), top_inner_rear, 0.005, zinc, None),
        ((sx(0.27), -0.22, -0.38), outer_mid_front, 0.005, zinc, None),
        ((sx(0.27), 0.22, -0.38), outer_mid_rear, 0.005, zinc, None),
        ((sx(0.32), -0.22, -0.50), outer_bottom_front, 0.005, zinc, None),
        ((sx(0.32), 0.22, -0.50), outer_bottom_rear, 0.005, zinc, None),
    ):
        _add_member(part, a, b, radius=radius, material=material, name=name)

    part.visual(
        Box((0.070, 0.12, 0.032)),
        origin=Origin(xyz=(sx(0.030), 0.0, -0.018)),
        material=polymer,
        name=f"{name_prefix}_hinge_stop_pad",
    )
    part.visual(
        Box((0.050, 0.14, 0.020)),
        origin=Origin(xyz=(sx(0.315), 0.0, -0.575)),
        material=polymer,
        name=f"{name_prefix}_wear_shoe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_drying_rack")

    steel = model.material("powder_coat_steel", rgba=(0.44, 0.48, 0.53, 1.0))
    zinc = model.material("zinc_hardware", rgba=(0.72, 0.75, 0.78, 1.0))
    polymer = model.material("service_polymer", rgba=(0.17, 0.18, 0.19, 1.0))
    bumper = model.material("maintenance_bumper", rgba=(0.56, 0.58, 0.60, 1.0))

    center_frame = model.part("center_frame")
    center_frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.56, 1.06)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    front_y = -0.24
    rear_y = 0.24
    left_x = -0.31
    right_x = 0.31
    hinge_z = 1.01

    for a, b, radius, material, name in (
        ((-0.36, front_y, 0.03), (0.36, front_y, 0.03), 0.014, steel, "front_foot_rail"),
        ((-0.36, rear_y, 0.03), (0.36, rear_y, 0.03), 0.014, steel, "rear_foot_rail"),
        ((left_x, front_y, 0.05), (left_x, rear_y, 0.05), 0.014, steel, None),
        ((right_x, front_y, 0.05), (right_x, rear_y, 0.05), 0.014, steel, None),
        ((left_x, front_y, 0.05), (left_x, front_y, 0.98), 0.014, steel, None),
        ((right_x, front_y, 0.05), (right_x, front_y, 0.98), 0.014, steel, None),
        ((left_x, rear_y, 0.05), (left_x, rear_y, 0.98), 0.014, steel, None),
        ((right_x, rear_y, 0.05), (right_x, rear_y, 0.98), 0.014, steel, None),
        ((-0.28, front_y, 0.98), (0.28, front_y, 0.98), 0.014, steel, None),
        ((-0.28, rear_y, 0.98), (0.28, rear_y, 0.98), 0.014, steel, None),
        ((left_x, front_y, 0.18), (left_x, rear_y, 0.18), 0.012, steel, None),
        ((right_x, front_y, 0.18), (right_x, rear_y, 0.18), 0.012, steel, None),
        ((left_x, front_y, 0.84), (left_x, rear_y, 0.84), 0.012, steel, None),
        ((right_x, front_y, 0.84), (right_x, rear_y, 0.84), 0.012, steel, None),
        ((-0.22, front_y, 0.72), (0.22, front_y, 0.72), 0.010, steel, None),
        ((-0.22, rear_y, 0.72), (0.22, rear_y, 0.72), 0.010, steel, None),
        ((-0.16, front_y, 0.50), (0.16, front_y, 0.50), 0.010, steel, None),
        ((-0.16, rear_y, 0.50), (0.16, rear_y, 0.50), 0.010, steel, None),
        ((-0.18, front_y, 0.72), (-0.18, rear_y, 0.72), 0.007, zinc, None),
        ((-0.06, front_y, 0.72), (-0.06, rear_y, 0.72), 0.007, zinc, None),
        ((0.06, front_y, 0.72), (0.06, rear_y, 0.72), 0.007, zinc, None),
        ((0.18, front_y, 0.72), (0.18, rear_y, 0.72), 0.007, zinc, None),
        ((-0.11, front_y, 0.50), (-0.11, rear_y, 0.50), 0.007, zinc, None),
        ((0.00, front_y, 0.50), (0.00, rear_y, 0.50), 0.007, zinc, None),
        ((0.11, front_y, 0.50), (0.11, rear_y, 0.50), 0.007, zinc, None),
        ((left_x, front_y, 0.20), (left_x, rear_y, 0.78), 0.007, zinc, None),
        ((left_x, front_y, 0.78), (left_x, rear_y, 0.20), 0.007, zinc, None),
        ((right_x, front_y, 0.20), (right_x, rear_y, 0.78), 0.007, zinc, None),
        ((right_x, front_y, 0.78), (right_x, rear_y, 0.20), 0.007, zinc, None),
        ((left_x, -0.22, hinge_z), (left_x, -0.08, hinge_z), 0.017, zinc, "left_center_hinge_barrel_front"),
        ((left_x, 0.08, hinge_z), (left_x, 0.22, hinge_z), 0.017, zinc, "left_center_hinge_barrel_rear"),
        ((right_x, -0.22, hinge_z), (right_x, -0.08, hinge_z), 0.017, zinc, "right_center_hinge_barrel_front"),
        ((right_x, 0.08, hinge_z), (right_x, 0.22, hinge_z), 0.017, zinc, "right_center_hinge_barrel_rear"),
        ((left_x, -0.15, 0.96), (left_x, -0.15, hinge_z), 0.010, zinc, None),
        ((left_x, 0.15, 0.96), (left_x, 0.15, hinge_z), 0.010, zinc, None),
        ((right_x, -0.15, 0.96), (right_x, -0.15, hinge_z), 0.010, zinc, None),
        ((right_x, 0.15, 0.96), (right_x, 0.15, hinge_z), 0.010, zinc, None),
        ((-0.31, front_y, 0.72), (-0.22, front_y, 0.72), 0.009, zinc, None),
        ((0.22, front_y, 0.72), (0.31, front_y, 0.72), 0.009, zinc, None),
        ((-0.31, rear_y, 0.72), (-0.22, rear_y, 0.72), 0.009, zinc, None),
        ((0.22, rear_y, 0.72), (0.31, rear_y, 0.72), 0.009, zinc, None),
        ((-0.31, front_y, 0.50), (-0.16, front_y, 0.50), 0.009, zinc, None),
        ((0.16, front_y, 0.50), (0.31, front_y, 0.50), 0.009, zinc, None),
        ((-0.31, rear_y, 0.50), (-0.16, rear_y, 0.50), 0.009, zinc, None),
        ((0.16, rear_y, 0.50), (0.31, rear_y, 0.50), 0.009, zinc, None),
        ((-0.31, front_y, 0.98), (-0.28, front_y, 0.98), 0.010, zinc, None),
        ((0.28, front_y, 0.98), (0.31, front_y, 0.98), 0.010, zinc, None),
        ((-0.31, rear_y, 0.98), (-0.28, rear_y, 0.98), 0.010, zinc, None),
        ((0.28, rear_y, 0.98), (0.31, rear_y, 0.98), 0.010, zinc, None),
    ):
        _add_member(center_frame, a, b, radius=radius, material=material, name=name)

    for x in (-0.36, 0.36):
        for y in (front_y, rear_y):
            center_frame.visual(
                Box((0.040, 0.040, 0.020)),
                origin=Origin(xyz=(x, y, 0.010)),
                material=polymer,
            )

    for side_sign, label in ((-1.0, "left"), (1.0, "right")):
        _add_member(
            center_frame,
            (side_sign * 0.31, -0.08, hinge_z),
            (side_sign * 0.28, front_y, 0.98),
            radius=0.009,
            material=steel,
            name=f"{label}_front_hinge_brace",
        )
        _add_member(
            center_frame,
            (side_sign * 0.31, 0.08, hinge_z),
            (side_sign * 0.28, rear_y, 0.98),
            radius=0.009,
            material=steel,
            name=f"{label}_rear_hinge_brace",
        )
        _add_member(
            center_frame,
            (side_sign * 0.28, -0.12, 0.962),
            (side_sign * 0.28, 0.12, 0.962),
            radius=0.010,
            material=zinc,
            name=f"{label}_stop_spine",
        )
        _add_member(
            center_frame,
            (side_sign * 0.28, -0.12, 0.962),
            (side_sign * 0.28, front_y, 0.98),
            radius=0.006,
            material=steel,
            name=f"{label}_front_stop_gusset",
        )
        _add_member(
            center_frame,
            (side_sign * 0.28, 0.12, 0.962),
            (side_sign * 0.28, rear_y, 0.98),
            radius=0.006,
            material=steel,
            name=f"{label}_rear_stop_gusset",
        )
        center_frame.visual(
            Box((0.018, 0.12, 0.048)),
            origin=Origin(xyz=(side_sign * 0.27, 0.0, 0.948)),
            material=bumper,
            name=f"{label}_stop_block",
        )

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((0.36, 0.50, 0.60)),
        mass=3.2,
        origin=Origin(xyz=(-0.18, 0.0, -0.30)),
    )
    _build_wing(left_wing, side_sign=-1.0, steel=steel, zinc=zinc, polymer=polymer, name_prefix="left")

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((0.36, 0.50, 0.60)),
        mass=3.2,
        origin=Origin(xyz=(0.18, 0.0, -0.30)),
    )
    _build_wing(right_wing, side_sign=1.0, steel=steel, zinc=zinc, polymer=polymer, name_prefix="right")

    model.articulation(
        "center_to_left_wing",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=left_wing,
        origin=Origin(xyz=(left_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.22),
    )
    model.articulation(
        "center_to_right_wing",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=right_wing,
        origin=Origin(xyz=(right_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_frame = object_model.get_part("center_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_hinge = object_model.get_articulation("center_to_left_wing")
    right_hinge = object_model.get_articulation("center_to_right_wing")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wing hinge axes open upward",
        left_hinge.axis == (0.0, 1.0, 0.0) and right_hinge.axis == (0.0, -1.0, 0.0),
        details=f"left axis={left_hinge.axis}, right axis={right_hinge.axis}",
    )
    ctx.check(
        "wing stop range is realistic",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and right_hinge.motion_limits.lower == 0.0
        and 1.0 <= left_hinge.motion_limits.upper <= 1.35
        and 1.0 <= right_hinge.motion_limits.upper <= 1.35,
        details="expected field-service wing stops near seventy degrees",
    )

    ctx.expect_contact(
        center_frame,
        left_wing,
        elem_a="left_center_hinge_barrel_front",
        elem_b="left_wing_hinge_barrel",
        name="left front hinge barrels stay in running contact",
    )
    ctx.expect_contact(
        center_frame,
        left_wing,
        elem_a="left_center_hinge_barrel_rear",
        elem_b="left_wing_hinge_barrel",
        name="left rear hinge barrels stay in running contact",
    )
    ctx.expect_contact(
        center_frame,
        right_wing,
        elem_a="right_center_hinge_barrel_front",
        elem_b="right_wing_hinge_barrel",
        name="right front hinge barrels stay in running contact",
    )
    ctx.expect_contact(
        center_frame,
        right_wing,
        elem_a="right_center_hinge_barrel_rear",
        elem_b="right_wing_hinge_barrel",
        name="right rear hinge barrels stay in running contact",
    )

    left_upper = left_hinge.motion_limits.upper if left_hinge.motion_limits is not None else 1.22
    right_upper = right_hinge.motion_limits.upper if right_hinge.motion_limits is not None else 1.22

    with ctx.pose({left_hinge: 0.0}):
        left_closed = ctx.part_element_world_aabb(left_wing, elem="left_outer_side_rail")
    with ctx.pose({right_hinge: 0.0}):
        right_closed = ctx.part_element_world_aabb(right_wing, elem="right_outer_side_rail")
    with ctx.pose({left_hinge: left_upper, right_hinge: right_upper}):
        left_open = ctx.part_element_world_aabb(left_wing, elem="left_outer_side_rail")
        right_open = ctx.part_element_world_aabb(right_wing, elem="right_outer_side_rail")
        ctx.fail_if_parts_overlap_in_current_pose(name="wings deployed without interference")
        ctx.expect_contact(
            center_frame,
            left_wing,
            elem_a="left_center_hinge_barrel_front",
            elem_b="left_wing_hinge_barrel",
            name="left hinge remains engaged when deployed",
        )
        ctx.expect_contact(
            center_frame,
            right_wing,
            elem_a="right_center_hinge_barrel_front",
            elem_b="right_wing_hinge_barrel",
            name="right hinge remains engaged when deployed",
        )

    left_ok = (
        left_closed is not None
        and left_open is not None
        and left_open[1][2] > left_closed[1][2] + 0.22
        and left_open[1][2] > 1.14
    )
    right_ok = (
        right_closed is not None
        and right_open is not None
        and right_open[1][2] > right_closed[1][2] + 0.22
        and right_open[1][2] > 1.14
    )
    ctx.check(
        "left wing reaches service drying position",
        left_ok,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right wing reaches service drying position",
        right_ok,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
