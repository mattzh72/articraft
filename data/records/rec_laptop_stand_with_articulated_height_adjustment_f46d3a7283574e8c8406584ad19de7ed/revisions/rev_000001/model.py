from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_W = 0.34
BASE_D = 0.24
BASE_T = 0.010

POST_X = 0.115
POST_Y = -0.082

SLEEVE_INNER_W = 0.024
SLEEVE_INNER_D = 0.032
SLEEVE_WALL = 0.004
SLEEVE_OUTER_D = SLEEVE_INNER_D + (2.0 * SLEEVE_WALL)
SLEEVE_H = 0.055

POST_W = 0.020
POST_D = 0.028
POST_LEN = 0.135
POST_CENTER_Z = 0.0125

HEAD_BLOCK_X = 0.018
HEAD_BLOCK_Y = 0.016
HEAD_BLOCK_Z = 0.012
HEAD_BLOCK_CENTER_Z = 0.078

CHEEK_X = 0.018
CHEEK_Y = 0.004
CHEEK_Z = 0.030
CHEEK_CENTER_Z = 0.086
CHEEK_OFFSET_Y = 0.009

HINGE_Z = 0.094
LIFT_TRAVEL = 0.035
TRAY_BASE_ANGLE = 0.22
TRAY_TRAVEL = 0.52

HINGE_SPAN = 0.230
HINGE_RADIUS = 0.005

TRAY_W = 0.300
TRAY_D = 0.250
TRAY_T = 0.008
TRAY_PANEL_CENTER_X = HINGE_SPAN / 2.0
TRAY_PANEL_CENTER_Y = 0.139
TRAY_PANEL_CENTER_Z = 0.022

REAR_WEB_Y = 0.014
REAR_WEB_Z = 0.018
REAR_WEB_CENTER_X = HINGE_SPAN / 2.0
REAR_WEB_CENTER_Y = REAR_WEB_Y / 2.0
REAR_WEB_CENTER_Z = REAR_WEB_Z / 2.0

FRONT_LIP_Y = 0.016
FRONT_LIP_Z = 0.015
FRONT_LIP_CENTER_X = HINGE_SPAN / 2.0
FRONT_LIP_CENTER_Y = 0.257
FRONT_LIP_CENTER_Z = 0.0335


def _add_sleeve(model_part, x_pos: float, y_pos: float, material: str, stem: str) -> None:
    side_x = (SLEEVE_INNER_W / 2.0) + (SLEEVE_WALL / 2.0)
    side_z = BASE_T + (SLEEVE_H / 2.0)
    front_y = (SLEEVE_INNER_D / 2.0) + (SLEEVE_WALL / 2.0)

    model_part.visual(
        Box((SLEEVE_WALL, SLEEVE_OUTER_D, SLEEVE_H)),
        origin=Origin(xyz=(x_pos - side_x, y_pos, side_z)),
        material=material,
        name=f"{stem}_side_outer",
    )
    model_part.visual(
        Box((SLEEVE_WALL, SLEEVE_OUTER_D, SLEEVE_H)),
        origin=Origin(xyz=(x_pos + side_x, y_pos, side_z)),
        material=material,
        name=f"{stem}_side_inner",
    )
    model_part.visual(
        Box((SLEEVE_INNER_W, SLEEVE_WALL, SLEEVE_H)),
        origin=Origin(xyz=(x_pos, y_pos - front_y, side_z)),
        material=material,
        name=f"{stem}_rear_wall",
    )
    model_part.visual(
        Box((SLEEVE_INNER_W, SLEEVE_WALL, SLEEVE_H)),
        origin=Origin(xyz=(x_pos, y_pos + front_y, side_z)),
        material=material,
        name=f"{stem}_front_wall",
    )


def _add_post_geometry(model_part, side: int, post_material: str, cap_material: str) -> None:
    x_offset = side * 0.009

    model_part.visual(
        Box((POST_W, POST_D, POST_LEN)),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=post_material,
        name="mast",
    )
    model_part.visual(
        Box((HEAD_BLOCK_X, HEAD_BLOCK_Y, HEAD_BLOCK_Z)),
        origin=Origin(xyz=(x_offset, 0.0, HEAD_BLOCK_CENTER_Z)),
        material=cap_material,
        name="head_block",
    )
    model_part.visual(
        Box((CHEEK_X, CHEEK_Y, CHEEK_Z)),
        origin=Origin(xyz=(x_offset, -CHEEK_OFFSET_Y, CHEEK_CENTER_Z)),
        material=cap_material,
        name="rear_cheek",
    )
    model_part.visual(
        Box((CHEEK_X, CHEEK_Y, CHEEK_Z)),
        origin=Origin(xyz=(x_offset, CHEEK_OFFSET_Y, CHEEK_CENTER_Z)),
        material=cap_material,
        name="front_cheek",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_laptop_stand")

    model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("silver", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="graphite",
        name="base_plate",
    )
    _add_sleeve(base, -POST_X, POST_Y, "graphite", "left_sleeve")
    _add_sleeve(base, POST_X, POST_Y, "graphite", "right_sleeve")

    left_post = model.part("left_post")
    _add_post_geometry(left_post, side=-1, post_material="silver", cap_material="graphite")

    right_post = model.part("right_post")
    _add_post_geometry(right_post, side=1, post_material="silver", cap_material="graphite")

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SPAN),
        origin=Origin(xyz=(HINGE_SPAN / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="silver",
        name="hinge_rod",
    )
    tray.visual(
        Box((HINGE_SPAN, REAR_WEB_Y, REAR_WEB_Z)),
        origin=Origin(xyz=(REAR_WEB_CENTER_X, REAR_WEB_CENTER_Y, REAR_WEB_CENTER_Z)),
        material="silver",
        name="rear_web",
    )
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_T)),
        origin=Origin(xyz=(TRAY_PANEL_CENTER_X, TRAY_PANEL_CENTER_Y, TRAY_PANEL_CENTER_Z)),
        material="silver",
        name="tray_panel",
    )
    tray.visual(
        Box((TRAY_W, FRONT_LIP_Y, FRONT_LIP_Z)),
        origin=Origin(xyz=(FRONT_LIP_CENTER_X, FRONT_LIP_CENTER_Y, FRONT_LIP_CENTER_Z)),
        material="rubber",
        name="front_lip",
    )

    base_to_left = model.articulation(
        "left_post_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_post,
        origin=Origin(xyz=(-POST_X, POST_Y, BASE_T + SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=30.0, velocity=0.12),
    )
    model.articulation(
        "right_post_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_post,
        origin=Origin(xyz=(POST_X, POST_Y, BASE_T + SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=30.0, velocity=0.12),
        mimic=Mimic("left_post_lift"),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=left_post,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(TRAY_BASE_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=12.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    left_post = object_model.get_part("left_post")
    right_post = object_model.get_part("right_post")
    tray = object_model.get_part("tray")

    left_post_lift = object_model.get_articulation("left_post_lift")
    tray_tilt = object_model.get_articulation("tray_tilt")

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.08,
        name="tray clears the base at rest",
    )

    left_rest = ctx.part_world_position(left_post)
    right_rest = ctx.part_world_position(right_post)
    rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")

    with ctx.pose({left_post_lift: LIFT_TRAVEL}):
        left_raised = ctx.part_world_position(left_post)
        right_raised = ctx.part_world_position(right_post)
        ctx.check(
            "paired posts lift together",
            left_rest is not None
            and right_rest is not None
            and left_raised is not None
            and right_raised is not None
            and left_raised[2] > left_rest[2] + 0.03
            and right_raised[2] > right_rest[2] + 0.03
            and abs(left_raised[2] - right_raised[2]) < 1e-6
            and abs(left_raised[1] - right_raised[1]) < 1e-6,
            details=(
                f"left_rest={left_rest}, right_rest={right_rest}, "
                f"left_raised={left_raised}, right_raised={right_raised}"
            ),
        )
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.12,
            name="raised tray still clears the base",
        )

    with ctx.pose({tray_tilt: TRAY_TRAVEL}):
        tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.check(
            "tray front rises when tilted",
            rest_front is not None
            and tilted_front is not None
            and tilted_front[0][2] > rest_front[0][2] + 0.05,
            details=f"rest_front={rest_front}, tilted_front={tilted_front}",
        )

    return ctx.report()


object_model = build_object_model()
