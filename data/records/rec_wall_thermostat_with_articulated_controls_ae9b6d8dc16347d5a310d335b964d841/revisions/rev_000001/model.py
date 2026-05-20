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


HOUSING_WIDTH = 0.086
HOUSING_HEIGHT = 0.126
HOUSING_DEPTH = 0.024
BACK_THICKNESS = 0.002
FACE_THICKNESS = 0.003
WALL_THICKNESS = 0.004
INNER_DEPTH = HOUSING_DEPTH - BACK_THICKNESS - FACE_THICKNESS

ROCKER_WIDTH = 0.030
ROCKER_HEIGHT = 0.058
ROCKER_OPENING_WIDTH = 0.032
ROCKER_OPENING_HEIGHT = 0.062
ROCKER_AXIS_Z = 0.018

DRAWER_CENTER_Y = -0.028
DRAWER_HEIGHT = 0.050
DRAWER_OPENING_HEIGHT = 0.052
DRAWER_DEPTH = 0.018
DRAWER_LENGTH = 0.026
DRAWER_CAP_THICKNESS = 0.0024
DRAWER_WALL = 0.0014
DRAWER_TRAVEL = 0.016
DRAWER_CENTER_Z = BACK_THICKNESS + INNER_DEPTH / 2.0


def _add_housing_shell(model: ArticulatedObject, housing, shell, trim) -> None:
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_HEIGHT, BACK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BACK_THICKNESS / 2.0)),
        material=shell,
        name="back_plate",
    )

    wall_center_z = BACK_THICKNESS + INNER_DEPTH / 2.0
    inner_height = HOUSING_HEIGHT - 2.0 * WALL_THICKNESS

    housing.visual(
        Box((HOUSING_WIDTH, WALL_THICKNESS, INNER_DEPTH)),
        origin=Origin(
            xyz=(0.0, HOUSING_HEIGHT / 2.0 - WALL_THICKNESS / 2.0, wall_center_z)
        ),
        material=shell,
        name="top_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH, WALL_THICKNESS, INNER_DEPTH)),
        origin=Origin(
            xyz=(0.0, -HOUSING_HEIGHT / 2.0 + WALL_THICKNESS / 2.0, wall_center_z)
        ),
        material=shell,
        name="bottom_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, inner_height, INNER_DEPTH)),
        origin=Origin(
            xyz=(-HOUSING_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, wall_center_z)
        ),
        material=shell,
        name="left_wall",
    )

    drawer_open_bottom = DRAWER_CENTER_Y - DRAWER_OPENING_HEIGHT / 2.0
    drawer_open_top = DRAWER_CENTER_Y + DRAWER_OPENING_HEIGHT / 2.0
    right_lower_height = drawer_open_bottom - (-HOUSING_HEIGHT / 2.0 + WALL_THICKNESS)
    right_upper_height = (HOUSING_HEIGHT / 2.0 - WALL_THICKNESS) - drawer_open_top

    housing.visual(
        Box((WALL_THICKNESS, right_upper_height, INNER_DEPTH)),
        origin=Origin(
            xyz=(
                HOUSING_WIDTH / 2.0 - WALL_THICKNESS / 2.0,
                drawer_open_top + right_upper_height / 2.0,
                wall_center_z,
            )
        ),
        material=shell,
        name="right_wall_upper",
    )
    housing.visual(
        Box((WALL_THICKNESS, right_lower_height, INNER_DEPTH)),
        origin=Origin(
            xyz=(
                HOUSING_WIDTH / 2.0 - WALL_THICKNESS / 2.0,
                -HOUSING_HEIGHT / 2.0
                + WALL_THICKNESS
                + right_lower_height / 2.0,
                wall_center_z,
            )
        ),
        material=shell,
        name="right_wall_lower",
    )

    front_center_z = HOUSING_DEPTH - FACE_THICKNESS / 2.0
    rocker_side_width = (HOUSING_WIDTH - ROCKER_OPENING_WIDTH) / 2.0
    rocker_strip_height = (HOUSING_HEIGHT - ROCKER_OPENING_HEIGHT) / 2.0

    housing.visual(
        Box((HOUSING_WIDTH, rocker_strip_height, FACE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, HOUSING_HEIGHT / 2.0 - rocker_strip_height / 2.0, front_center_z)
        ),
        material=trim,
        name="face_top",
    )
    housing.visual(
        Box((HOUSING_WIDTH, rocker_strip_height, FACE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -HOUSING_HEIGHT / 2.0 + rocker_strip_height / 2.0, front_center_z)
        ),
        material=trim,
        name="face_bottom",
    )
    housing.visual(
        Box((rocker_side_width, ROCKER_OPENING_HEIGHT, FACE_THICKNESS)),
        origin=Origin(
            xyz=(
                -HOUSING_WIDTH / 2.0 + rocker_side_width / 2.0,
                0.0,
                front_center_z,
            )
        ),
        material=trim,
        name="face_left",
    )
    housing.visual(
        Box((rocker_side_width, ROCKER_OPENING_HEIGHT, FACE_THICKNESS)),
        origin=Origin(
            xyz=(
                HOUSING_WIDTH / 2.0 - rocker_side_width / 2.0,
                0.0,
                front_center_z,
            )
        ),
        material=trim,
        name="face_right",
    )

    boss_depth = 0.006
    boss_width = 0.004
    boss_height = 0.010
    boss_center_z = HOUSING_DEPTH - FACE_THICKNESS - boss_depth / 2.0
    boss_center_x = ROCKER_OPENING_WIDTH / 2.0 + boss_width / 2.0
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((boss_width, boss_height, boss_depth)),
            origin=Origin(xyz=(x_sign * boss_center_x, 0.0, boss_center_z)),
            material=shell,
            name=f"rocker_boss_{side}",
        )


def _add_rocker_part(model: ArticulatedObject, housing, rocker_material, accent_material) -> None:
    rocker = model.part("rocker")
    axle_length = ROCKER_OPENING_WIDTH
    rocker.visual(
        Cylinder(radius=0.0017, length=axle_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="axle",
    )
    rocker.visual(
        Box((ROCKER_WIDTH, ROCKER_HEIGHT, 0.0036)),
        origin=Origin(xyz=(0.0, 0.0, 0.0028)),
        material=rocker_material,
        name="paddle",
    )
    rocker.visual(
        Box((ROCKER_WIDTH - 0.004, ROCKER_HEIGHT / 2.0 - 0.003, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0145, 0.0049)),
        material=accent_material,
        name="upper_pad",
    )
    rocker.visual(
        Box((ROCKER_WIDTH - 0.004, ROCKER_HEIGHT / 2.0 - 0.003, 0.0008)),
        origin=Origin(xyz=(0.0, -0.0145, 0.0049)),
        material=accent_material,
        name="lower_pad",
    )

    model.articulation(
        "rocker_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, ROCKER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-0.20,
            upper=0.20,
        ),
    )


def _add_battery_drawer(
    model: ArticulatedObject,
    housing,
    drawer_material,
    pull_material,
) -> None:
    drawer = model.part("battery_drawer")
    drawer.visual(
        Box((DRAWER_CAP_THICKNESS, DRAWER_HEIGHT, DRAWER_DEPTH)),
        origin=Origin(xyz=(DRAWER_CAP_THICKNESS / 2.0, 0.0, 0.0)),
        material=drawer_material,
        name="drawer_cap",
    )
    drawer.visual(
        Cylinder(radius=0.0028, length=DRAWER_HEIGHT * 0.42),
        origin=Origin(
            xyz=(DRAWER_CAP_THICKNESS + 0.0014, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=pull_material,
        name="drawer_pull",
    )
    drawer.visual(
        Box((DRAWER_LENGTH, DRAWER_WALL, DRAWER_DEPTH)),
        origin=Origin(
            xyz=(-DRAWER_LENGTH / 2.0, DRAWER_HEIGHT / 2.0 - DRAWER_WALL / 2.0, 0.0)
        ),
        material=drawer_material,
        name="tray_top",
    )
    drawer.visual(
        Box((DRAWER_LENGTH, DRAWER_WALL, DRAWER_DEPTH)),
        origin=Origin(
            xyz=(-DRAWER_LENGTH / 2.0, -DRAWER_HEIGHT / 2.0 + DRAWER_WALL / 2.0, 0.0)
        ),
        material=drawer_material,
        name="tray_bottom",
    )
    drawer.visual(
        Box((DRAWER_LENGTH, DRAWER_HEIGHT - 2.0 * DRAWER_WALL, DRAWER_WALL)),
        origin=Origin(
            xyz=(-DRAWER_LENGTH / 2.0, 0.0, DRAWER_DEPTH / 2.0 - DRAWER_WALL / 2.0)
        ),
        material=drawer_material,
        name="tray_front",
    )
    drawer.visual(
        Box((DRAWER_LENGTH, DRAWER_HEIGHT - 2.0 * DRAWER_WALL, DRAWER_WALL)),
        origin=Origin(
            xyz=(-DRAWER_LENGTH / 2.0, 0.0, -DRAWER_DEPTH / 2.0 + DRAWER_WALL / 2.0)
        ),
        material=drawer_material,
        name="tray_back",
    )

    model.articulation(
        "battery_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(
            xyz=(HOUSING_WIDTH / 2.0, DRAWER_CENTER_Y, DRAWER_CENTER_Z),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    shell = model.material("shell", rgba=(0.93, 0.93, 0.91, 1.0))
    trim = model.material("trim", rgba=(0.96, 0.96, 0.95, 1.0))
    rocker_material = model.material("rocker", rgba=(0.80, 0.81, 0.82, 1.0))
    rocker_accent = model.material("rocker_accent", rgba=(0.67, 0.69, 0.71, 1.0))
    drawer_material = model.material("drawer", rgba=(0.88, 0.88, 0.87, 1.0))
    pull_material = model.material("pull", rgba=(0.55, 0.57, 0.60, 1.0))

    housing = model.part("housing")
    _add_housing_shell(model, housing, shell, trim)
    _add_rocker_part(model, housing, rocker_material, rocker_accent)
    _add_battery_drawer(model, housing, drawer_material, pull_material)

    return model


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[index] - mins[index]) for index in range(3))


def _z_max(aabb) -> float | None:
    if aabb is None:
        return None
    _, maxs = aabb
    return float(maxs[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    rocker = object_model.get_part("rocker")
    drawer = object_model.get_part("battery_drawer")
    rocker_hinge = object_model.get_articulation("rocker_hinge")
    drawer_slide = object_model.get_articulation("battery_drawer_slide")

    housing_aabb = ctx.part_world_aabb(housing)
    housing_size = _aabb_size(housing_aabb)
    ctx.check("housing_aabb_present", housing_aabb is not None, "Expected housing AABB.")
    if housing_size is not None:
        ctx.check("housing_width_household_scale", 0.080 <= housing_size[0] <= 0.090, f"size={housing_size!r}")
        ctx.check("housing_height_household_scale", 0.120 <= housing_size[1] <= 0.130, f"size={housing_size!r}")
        ctx.check("housing_depth_household_scale", 0.022 <= housing_size[2] <= 0.026, f"size={housing_size!r}")

    rocker_upper_aabb = ctx.part_element_world_aabb(rocker, elem="upper_pad")
    rocker_lower_aabb = ctx.part_element_world_aabb(rocker, elem="lower_pad")
    housing_front_z = None if housing_aabb is None else float(housing_aabb[1][2])
    upper_z = _z_max(rocker_upper_aabb)
    lower_z = _z_max(rocker_lower_aabb)
    ctx.check(
        "rocker_rest_nearly_flush",
        housing_front_z is not None
        and upper_z is not None
        and lower_z is not None
        and 0.0002 <= housing_front_z - upper_z <= 0.0020
        and 0.0002 <= housing_front_z - lower_z <= 0.0020,
        details=f"housing_front_z={housing_front_z}, upper_z={upper_z}, lower_z={lower_z}",
    )
    ctx.check(
        "rocker_rest_balanced",
        upper_z is not None and lower_z is not None and abs(upper_z - lower_z) <= 0.0006,
        details=f"upper_z={upper_z}, lower_z={lower_z}",
    )

    rocker_upper_limit = rocker_hinge.motion_limits.upper if rocker_hinge.motion_limits is not None else None
    rocker_lower_limit = rocker_hinge.motion_limits.lower if rocker_hinge.motion_limits is not None else None
    if rocker_upper_limit is not None:
        with ctx.pose({rocker_hinge: rocker_upper_limit}):
            upper_pressed_aabb = ctx.part_element_world_aabb(rocker, elem="upper_pad")
            lower_pressed_aabb = ctx.part_element_world_aabb(rocker, elem="lower_pad")
            ctx.check(
                "rocker_positive_pose_tilts_upper_outward",
                _z_max(upper_pressed_aabb) is not None
                and _z_max(lower_pressed_aabb) is not None
                and _z_max(upper_pressed_aabb) > _z_max(lower_pressed_aabb) + 0.0015,
                details=f"upper={upper_pressed_aabb}, lower={lower_pressed_aabb}",
            )
    if rocker_lower_limit is not None:
        with ctx.pose({rocker_hinge: rocker_lower_limit}):
            upper_pressed_aabb = ctx.part_element_world_aabb(rocker, elem="upper_pad")
            lower_pressed_aabb = ctx.part_element_world_aabb(rocker, elem="lower_pad")
            ctx.check(
                "rocker_negative_pose_tilts_lower_outward",
                _z_max(upper_pressed_aabb) is not None
                and _z_max(lower_pressed_aabb) is not None
                and _z_max(lower_pressed_aabb) > _z_max(upper_pressed_aabb) + 0.0015,
                details=f"upper={upper_pressed_aabb}, lower={lower_pressed_aabb}",
            )

    drawer_cap_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_cap")
    drawer_aabb = ctx.part_world_aabb(drawer)
    drawer_rest_position = ctx.part_world_position(drawer)
    housing_max_x = None if housing_aabb is None else float(housing_aabb[1][0])
    cap_min_x = None if drawer_cap_aabb is None else float(drawer_cap_aabb[0][0])
    ctx.check(
        "drawer_rest_seats_at_side",
        housing_max_x is not None
        and cap_min_x is not None
        and abs(cap_min_x - housing_max_x) <= 0.0010,
        details=f"housing_max_x={housing_max_x}, cap_min_x={cap_min_x}",
    )

    drawer_upper_limit = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if drawer_upper_limit is not None:
        with ctx.pose({drawer_slide: drawer_upper_limit}):
            drawer_extended_position = ctx.part_world_position(drawer)
            drawer_extended_aabb = ctx.part_world_aabb(drawer)
            drawer_extended_cap = ctx.part_element_world_aabb(drawer, elem="drawer_cap")
            extended_min_x = None if drawer_extended_aabb is None else float(drawer_extended_aabb[0][0])
            extended_cap_min_x = None if drawer_extended_cap is None else float(drawer_extended_cap[0][0])
            ctx.check(
                "drawer_extends_outward",
                drawer_rest_position is not None
                and drawer_extended_position is not None
                and drawer_extended_position[0] > drawer_rest_position[0] + 0.012,
                details=f"rest={drawer_rest_position}, extended={drawer_extended_position}",
            )
            ctx.check(
                "drawer_retains_insertion_when_extended",
                housing_max_x is not None
                and extended_min_x is not None
                and extended_min_x < housing_max_x - 0.006,
                details=f"housing_max_x={housing_max_x}, extended_min_x={extended_min_x}",
            )
            ctx.check(
                "drawer_cap_clearly_protrudes_when_extended",
                housing_max_x is not None
                and extended_cap_min_x is not None
                and extended_cap_min_x > housing_max_x + 0.012,
                details=f"housing_max_x={housing_max_x}, extended_cap_min_x={extended_cap_min_x}",
            )

    return ctx.report()


object_model = build_object_model()
