from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


BODY_W = 0.290
BODY_D = 0.330
BODY_FRONT_H = 0.096
BODY_REAR_H = 0.155
SLOPE_START_Y = -0.015
SLOPE_END_Y = 0.095

DRAWER_FRONT_W = 0.242
DRAWER_FRONT_T = 0.012
DRAWER_FRONT_H = 0.062
DRAWER_TRAY_W = 0.226
DRAWER_TRAY_D = 0.230
DRAWER_WALL_T = 0.004
DRAWER_SIDE_H = 0.045
DRAWER_BOTTOM_T = 0.004
DRAWER_DIVIDER_T = 0.003
DRAWER_TRAVEL = 0.112
DRAWER_FLOOR_Z = 0.014

SCREEN_PIVOT_X = 0.104
SCREEN_PIVOT_Y = 0.122
SCREEN_MOUNT_H = 0.006


def _slope_z(y_pos: float) -> float:
    slope = (BODY_REAR_H - BODY_FRONT_H) / (SLOPE_END_Y - SLOPE_START_Y)
    return BODY_FRONT_H + (y_pos - SLOPE_START_Y) * slope


def _build_register_shell() -> cq.Workplane:
    side_profile = [
        (-BODY_D / 2.0, 0.0),
        (-BODY_D / 2.0, BODY_FRONT_H),
        (SLOPE_START_Y, BODY_FRONT_H),
        (SLOPE_END_Y, BODY_REAR_H),
        (BODY_D / 2.0, BODY_REAR_H),
        (BODY_D / 2.0, 0.0),
    ]
    shell = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    drawer_bay = cq.Workplane("XY").box(0.248, 0.252, 0.066).translate(
        (0.0, -0.039, 0.046)
    )
    return shell.cut(drawer_bay)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_cash_register")

    body_color = model.material("body_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    panel_color = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    key_color = model.material("key_grey", rgba=(0.52, 0.55, 0.58, 1.0))
    accent_color = model.material("accent_blue", rgba=(0.14, 0.31, 0.45, 1.0))
    rubber_color = model.material("rubber_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    drawer_color = model.material("drawer_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    handle_color = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    screen_body_color = model.material("screen_body", rgba=(0.17, 0.18, 0.20, 1.0))
    glass_color = model.material("glass_teal", rgba=(0.20, 0.49, 0.55, 0.45))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_register_shell(), "register_shell"),
        material=body_color,
        name="shell",
    )

    slope_angle = math.atan2(
        BODY_REAR_H - BODY_FRONT_H,
        SLOPE_END_Y - SLOPE_START_Y,
    )

    body.visual(
        Box((0.178, 0.096, 0.003)),
        origin=Origin(
            xyz=(0.0, 0.040, _slope_z(0.040) + 0.0008),
            rpy=(slope_angle, 0.0, 0.0),
        ),
        material=panel_color,
        name="keypad_plate",
    )

    key_centers_x = (-0.054, -0.018, 0.018, 0.054)
    key_centers_y = (-0.002, 0.024, 0.050, 0.076)
    for row_index, center_y in enumerate(key_centers_y):
        for col_index, center_x in enumerate(key_centers_x):
            body.visual(
                Box((0.024, 0.019, 0.006)),
                origin=Origin(
                    xyz=(center_x, center_y, _slope_z(center_y) + 0.0015),
                    rpy=(slope_angle, 0.0, 0.0),
                ),
                material=key_color,
                name=f"key_{row_index}_{col_index}",
            )

    for index, x_pos in enumerate((-0.082, -0.044, 0.044, 0.082)):
        body.visual(
            Box((0.022, 0.026, 0.006)),
            origin=Origin(
                xyz=(x_pos, 0.106, BODY_REAR_H + 0.0015),
            ),
            material=accent_color if index < 2 else key_color,
            name=f"rear_button_{index}",
        )

    body.visual(
        Box((0.118, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.121, BODY_REAR_H + 0.0015)),
        material=panel_color,
        name="receipt_slot",
    )

    body.visual(
        Box((0.248, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.005, 0.049)),
        material=panel_color,
        name="drawer_frame",
    )
    body.visual(
        Cylinder(radius=0.012, length=SCREEN_MOUNT_H),
        origin=Origin(
            xyz=(SCREEN_PIVOT_X, SCREEN_PIVOT_Y, BODY_REAR_H + SCREEN_MOUNT_H / 2.0),
        ),
        material=panel_color,
        name="screen_mount",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.108, -0.122),
            (0.108, -0.122),
            (-0.108, 0.122),
            (0.108, 0.122),
        )
    ):
        body.visual(
            Box((0.030, 0.018, 0.006)),
            origin=Origin(xyz=(x_pos, y_pos, -0.003)),
            material=rubber_color,
            name=f"foot_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_REAR_H + SCREEN_MOUNT_H)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, (BODY_REAR_H + SCREEN_MOUNT_H) / 2.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_FRONT_H)),
        origin=Origin(
            xyz=(0.0, -DRAWER_FRONT_T / 2.0, DRAWER_FRONT_H / 2.0),
        ),
        material=drawer_color,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.106, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.031)),
        material=handle_color,
        name="drawer_pull",
    )
    drawer.visual(
        Box((DRAWER_TRAY_W, DRAWER_TRAY_D, DRAWER_BOTTOM_T)),
        origin=Origin(
            xyz=(0.0, DRAWER_TRAY_D / 2.0, DRAWER_BOTTOM_T / 2.0),
        ),
        material=drawer_color,
        name="tray_bottom",
    )
    side_wall_x = DRAWER_TRAY_W / 2.0 - DRAWER_WALL_T / 2.0
    side_wall_y = DRAWER_TRAY_D / 2.0
    side_wall_z = DRAWER_SIDE_H / 2.0 + DRAWER_BOTTOM_T
    drawer.visual(
        Box((DRAWER_WALL_T, DRAWER_TRAY_D, DRAWER_SIDE_H)),
        origin=Origin(xyz=(-side_wall_x, side_wall_y, side_wall_z)),
        material=drawer_color,
        name="side_wall_0",
    )
    drawer.visual(
        Box((DRAWER_WALL_T, DRAWER_TRAY_D, DRAWER_SIDE_H)),
        origin=Origin(xyz=(side_wall_x, side_wall_y, side_wall_z)),
        material=drawer_color,
        name="side_wall_1",
    )
    drawer.visual(
        Box((DRAWER_TRAY_W, DRAWER_WALL_T, DRAWER_SIDE_H)),
        origin=Origin(
            xyz=(0.0, DRAWER_TRAY_D - DRAWER_WALL_T / 2.0, side_wall_z),
        ),
        material=drawer_color,
        name="back_wall",
    )
    for index, x_pos in enumerate((-0.055, 0.0, 0.055)):
        drawer.visual(
            Box((DRAWER_DIVIDER_T, 0.152, 0.029)),
            origin=Origin(xyz=(x_pos, 0.138, 0.0185)),
            material=drawer_color,
            name=f"divider_{index}",
        )

    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_W, DRAWER_TRAY_D + 0.010, DRAWER_FRONT_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.110, 0.031)),
    )

    screen = model.part("screen")
    screen.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=screen_body_color,
        name="post",
    )
    screen.visual(
        Box((0.044, 0.012, 0.030)),
        origin=Origin(xyz=(-0.022, -0.006, 0.075)),
        material=screen_body_color,
        name="neck",
    )
    screen.visual(
        Box((0.078, 0.016, 0.058)),
        origin=Origin(xyz=(-0.058, -0.008, 0.112)),
        material=screen_body_color,
        name="screen_panel",
    )
    screen.visual(
        Box((0.066, 0.004, 0.046)),
        origin=Origin(xyz=(-0.058, -0.018, 0.112)),
        material=glass_color,
        name="screen_glass",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.090, 0.020, 0.150)),
        mass=0.18,
        origin=Origin(xyz=(-0.040, -0.008, 0.080)),
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, DRAWER_FLOOR_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(
            xyz=(
                SCREEN_PIVOT_X,
                SCREEN_PIVOT_Y,
                BODY_REAR_H + SCREEN_MOUNT_H,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    screen = object_model.get_part("screen")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    screen_swivel = object_model.get_articulation("body_to_screen")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            (lower[0] + upper[0]) / 2.0,
            (lower[1] + upper[1]) / 2.0,
            (lower[2] + upper[2]) / 2.0,
        )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_gap(
            body,
            drawer,
            axis="y",
            positive_elem="shell",
            negative_elem="drawer_front",
            max_gap=0.001,
            max_penetration=0.001,
            name="drawer front closes flush with the body",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="xz",
            elem_a="drawer_front",
            elem_b="drawer_frame",
            min_overlap=0.050,
            name="closed drawer aligns with the front opening",
        )
        ctx.expect_gap(
            screen,
            body,
            axis="z",
            positive_elem="post",
            negative_elem="screen_mount",
            max_gap=0.001,
            max_penetration=0.0,
            name="screen post seats on the rear swivel mount",
        )

    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_gap(
            body,
            drawer,
            axis="y",
            positive_elem="shell",
            negative_elem="drawer_front",
            min_gap=0.105,
            max_gap=0.119,
            name="drawer extends forward on its slide axis",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_bottom",
            elem_b="drawer_frame",
            min_overlap=0.200,
            name="drawer remains laterally guided when extended",
        )

    lower_limit = screen_swivel.motion_limits.lower if screen_swivel.motion_limits is not None else None
    upper_limit = screen_swivel.motion_limits.upper if screen_swivel.motion_limits is not None else None
    lower_center = None
    upper_center = None
    if lower_limit is not None and upper_limit is not None:
        with ctx.pose({screen_swivel: lower_limit}):
            lower_center = _aabb_center(
                ctx.part_element_world_aabb(screen, elem="screen_panel")
            )
        with ctx.pose({screen_swivel: upper_limit}):
            upper_center = _aabb_center(
                ctx.part_element_world_aabb(screen, elem="screen_panel")
            )

    ctx.check(
        "screen swivels across the operator corner",
        lower_center is not None
        and upper_center is not None
        and math.hypot(
            upper_center[0] - lower_center[0],
            upper_center[1] - lower_center[1],
        )
        > 0.090,
        details=f"lower={lower_center}, upper={upper_center}",
    )

    return ctx.report()


object_model = build_object_model()
