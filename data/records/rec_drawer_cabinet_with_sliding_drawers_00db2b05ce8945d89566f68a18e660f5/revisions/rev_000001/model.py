from __future__ import annotations

from math import pi

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


BODY_W = 0.46
BODY_D = 0.40
BODY_TOP_Z = 0.598
TOP_T = 0.022
TOP_W = 0.50
TOP_D = 0.43
TOP_Z = BODY_TOP_Z + TOP_T / 2.0

PLINTH_H = 0.050
PLINTH_W = 0.39
PLINTH_D = 0.31
PLINTH_Y = -0.020

SIDE_T = 0.018
BACK_T = 0.006
BOTTOM_T = 0.018
SHELF_T = 0.014

INNER_W = BODY_W - 2.0 * SIDE_T
INNER_FACE_X = BODY_W / 2.0 - SIDE_T
FRONT_PLANE_Y = BODY_D / 2.0
BACK_PANEL_CENTER_Y = -BODY_D / 2.0 + BACK_T / 2.0
INTERIOR_REAR_FACE_Y = BACK_PANEL_CENTER_Y + BACK_T / 2.0
INTERIOR_FRONT_SUPPORT_Y = BODY_D / 2.0 - 0.020

LOWER_SHELF_Z = 0.285
UPPER_SHELF_Z = 0.488

LOWER_DRAWER_Z = 0.173
UPPER_DRAWER_Z = 0.3865
TRAY_Z = 0.541

RUNNER_T = 0.010
RUNNER_H = 0.022
RUNNER_LEN = 0.260
RUNNER_CENTER_Y = -0.010
LOWER_RUNNER_Z = 0.163
UPPER_RUNNER_Z = 0.378

GUIDE_T = 0.008
GUIDE_H = 0.010
GUIDE_LEN = 0.240
GUIDE_CENTER_Y = -0.020
TRAY_GUIDE_Z = 0.558

DRAWER_FRONT_T = 0.018
DRAWER_BODY_W = 0.388
DRAWER_BODY_D = 0.330
DRAWER_SIDE_T = 0.012
DRAWER_BACK_T = 0.012
DRAWER_BASE_T = 0.006
SLIDE_T = 0.008
SLIDE_H = 0.022
SLIDE_LEN = 0.200
SLIDE_CENTER_Y = -0.200

LOWER_FRONT_W = 0.410
LOWER_FRONT_H = 0.202
LOWER_BOX_H = 0.160

UPPER_FRONT_W = 0.410
UPPER_FRONT_H = 0.181
UPPER_BOX_H = 0.145

TRAY_FRONT_W = 0.406
TRAY_FRONT_H = 0.062
TRAY_FRONT_T = 0.016
TRAY_BOARD_W = 0.360
TRAY_BOARD_D = 0.220
TRAY_BOARD_T = 0.012
TRAY_SIDE_T = 0.006
TRAY_SIDE_H = 0.014
TRAY_REAR_T = 0.008
TRAY_GUIDE_RAIL_T = 0.008
TRAY_GUIDE_RAIL_H = 0.008
TRAY_GUIDE_RAIL_LEN = 0.180

LOWER_DRAWER_TRAVEL = 0.180
UPPER_DRAWER_TRAVEL = 0.155
TRAY_TRAVEL = 0.120


def _drawer_box_center_z(front_h: float, box_h: float, bottom_margin: float = 0.018) -> float:
    return -front_h / 2.0 + bottom_margin + box_h / 2.0


def _drawer_base_center_z(front_h: float, bottom_margin: float = 0.018) -> float:
    return -front_h / 2.0 + bottom_margin + DRAWER_BASE_T / 2.0


def _add_bar_pull(part, prefix: str, width: float, z: float = 0.0) -> None:
    rod_length = min(0.120, width * 0.33)
    rod_radius = 0.006
    post_radius = 0.003
    post_length = 0.018
    post_x = rod_length * 0.32

    part.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(xyz=(0.0, 0.024, z), rpy=(0.0, pi / 2.0, 0.0)),
        material="pull_metal",
        name=f"{prefix}_pull",
    )
    part.visual(
        Cylinder(radius=post_radius, length=post_length),
        origin=Origin(xyz=(-post_x, 0.009, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pull_metal",
        name=f"{prefix}_post_0",
    )
    part.visual(
        Cylinder(radius=post_radius, length=post_length),
        origin=Origin(xyz=(post_x, 0.009, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pull_metal",
        name=f"{prefix}_post_1",
    )


def _build_drawer(
    part,
    *,
    prefix: str,
    front_w: float,
    front_h: float,
    box_h: float,
    slide_left_name: str,
    slide_right_name: str,
) -> None:
    box_center_z = _drawer_box_center_z(front_h, box_h)
    base_center_z = _drawer_base_center_z(front_h)
    side_x = DRAWER_BODY_W / 2.0 - DRAWER_SIDE_T / 2.0
    back_center_y = -DRAWER_FRONT_T - DRAWER_BODY_D + DRAWER_BACK_T / 2.0

    part.visual(
        Box((front_w, DRAWER_FRONT_T, front_h)),
        origin=Origin(xyz=(0.0, -DRAWER_FRONT_T / 2.0, 0.0)),
        material="case_oak",
        name=f"{prefix}_front",
    )
    part.visual(
        Box((DRAWER_SIDE_T, DRAWER_BODY_D, box_h)),
        origin=Origin(xyz=(-side_x, -DRAWER_FRONT_T - DRAWER_BODY_D / 2.0, box_center_z)),
        material="drawer_oak",
        name=f"{prefix}_side_left",
    )
    part.visual(
        Box((DRAWER_SIDE_T, DRAWER_BODY_D, box_h)),
        origin=Origin(xyz=(side_x, -DRAWER_FRONT_T - DRAWER_BODY_D / 2.0, box_center_z)),
        material="drawer_oak",
        name=f"{prefix}_side_right",
    )
    part.visual(
        Box((DRAWER_BODY_W - 2.0 * DRAWER_SIDE_T, DRAWER_BODY_D, DRAWER_BASE_T)),
        origin=Origin(xyz=(0.0, -DRAWER_FRONT_T - DRAWER_BODY_D / 2.0, base_center_z)),
        material="drawer_oak",
        name=f"{prefix}_base",
    )
    part.visual(
        Box((DRAWER_BODY_W, DRAWER_BACK_T, box_h)),
        origin=Origin(xyz=(0.0, back_center_y, box_center_z)),
        material="drawer_oak",
        name=f"{prefix}_back",
    )

    slide_x = DRAWER_BODY_W / 2.0 + SLIDE_T / 2.0
    slide_z = box_center_z - 0.006
    part.visual(
        Box((SLIDE_T, SLIDE_LEN, SLIDE_H)),
        origin=Origin(xyz=(-slide_x, SLIDE_CENTER_Y, slide_z)),
        material="runner_metal",
        name=slide_left_name,
    )
    part.visual(
        Box((SLIDE_T, SLIDE_LEN, SLIDE_H)),
        origin=Origin(xyz=(slide_x, SLIDE_CENTER_Y, slide_z)),
        material="runner_metal",
        name=slide_right_name,
    )

    _add_bar_pull(part, prefix, front_w)


def _build_tray(part) -> None:
    board_center_y = -TRAY_FRONT_T - TRAY_BOARD_D / 2.0
    side_z = TRAY_BOARD_T / 2.0 + TRAY_SIDE_H / 2.0
    guide_x = INNER_FACE_X - GUIDE_T - TRAY_GUIDE_RAIL_T / 2.0
    bracket_w = 0.016
    bracket_len = 0.040
    bracket_x = TRAY_BOARD_W / 2.0 + bracket_w / 2.0

    part.visual(
        Box((TRAY_FRONT_W, TRAY_FRONT_T, TRAY_FRONT_H)),
        origin=Origin(xyz=(0.0, -TRAY_FRONT_T / 2.0, 0.0)),
        material="case_oak",
        name="tray_front",
    )
    part.visual(
        Box((TRAY_BOARD_W, TRAY_BOARD_D, TRAY_BOARD_T)),
        origin=Origin(xyz=(0.0, board_center_y, TRAY_BOARD_T / 2.0)),
        material="drawer_oak",
        name="tray_board",
    )
    part.visual(
        Box((TRAY_SIDE_T, TRAY_BOARD_D, TRAY_SIDE_H)),
        origin=Origin(
            xyz=(-TRAY_BOARD_W / 2.0 + TRAY_SIDE_T / 2.0, board_center_y, side_z),
        ),
        material="drawer_oak",
        name="tray_side_left",
    )
    part.visual(
        Box((TRAY_SIDE_T, TRAY_BOARD_D, TRAY_SIDE_H)),
        origin=Origin(
            xyz=(TRAY_BOARD_W / 2.0 - TRAY_SIDE_T / 2.0, board_center_y, side_z),
        ),
        material="drawer_oak",
        name="tray_side_right",
    )
    part.visual(
        Box((TRAY_BOARD_W - 2.0 * TRAY_SIDE_T, TRAY_REAR_T, TRAY_SIDE_H)),
        origin=Origin(
            xyz=(
                0.0,
                -TRAY_FRONT_T - TRAY_BOARD_D + TRAY_REAR_T / 2.0,
                side_z,
            ),
        ),
        material="drawer_oak",
        name="tray_back",
    )

    part.visual(
        Box((TRAY_GUIDE_RAIL_T, TRAY_GUIDE_RAIL_LEN, TRAY_GUIDE_RAIL_H)),
        origin=Origin(xyz=(-guide_x, -0.170, 0.018)),
        material="runner_metal",
        name="tray_slide_left",
    )
    part.visual(
        Box((TRAY_GUIDE_RAIL_T, TRAY_GUIDE_RAIL_LEN, TRAY_GUIDE_RAIL_H)),
        origin=Origin(xyz=(guide_x, -0.170, 0.018)),
        material="runner_metal",
        name="tray_slide_right",
    )
    part.visual(
        Box((bracket_w, bracket_len, TRAY_GUIDE_RAIL_H)),
        origin=Origin(xyz=(-bracket_x, -0.170, 0.018)),
        material="runner_metal",
        name="tray_bracket_left",
    )
    part.visual(
        Box((bracket_w, bracket_len, TRAY_GUIDE_RAIL_H)),
        origin=Origin(xyz=(bracket_x, -0.170, 0.018)),
        material="runner_metal",
        name="tray_bracket_right",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nightstand_drawer_cabinet")

    model.material("case_oak", rgba=(0.68, 0.55, 0.39, 1.0))
    model.material("drawer_oak", rgba=(0.62, 0.50, 0.35, 1.0))
    model.material("top_oak", rgba=(0.55, 0.42, 0.28, 1.0))
    model.material("runner_metal", rgba=(0.65, 0.66, 0.68, 1.0))
    model.material("pull_metal", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("plinth_dark", rgba=(0.25, 0.18, 0.12, 1.0))

    carcass = model.part("carcass")

    side_h = BODY_TOP_Z - PLINTH_H
    side_z = PLINTH_H + side_h / 2.0
    panel_depth = INTERIOR_FRONT_SUPPORT_Y - INTERIOR_REAR_FACE_Y
    panel_center_y = (INTERIOR_FRONT_SUPPORT_Y + INTERIOR_REAR_FACE_Y) / 2.0

    carcass.visual(
        Box((TOP_W, TOP_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        material="top_oak",
        name="top_panel",
    )
    carcass.visual(
        Box((PLINTH_W, PLINTH_D, PLINTH_H)),
        origin=Origin(xyz=(0.0, PLINTH_Y, PLINTH_H / 2.0)),
        material="plinth_dark",
        name="plinth",
    )
    carcass.visual(
        Box((SIDE_T, BODY_D, side_h)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SIDE_T / 2.0, 0.0, side_z)),
        material="case_oak",
        name="side_left",
    )
    carcass.visual(
        Box((SIDE_T, BODY_D, side_h)),
        origin=Origin(xyz=(BODY_W / 2.0 - SIDE_T / 2.0, 0.0, side_z)),
        material="case_oak",
        name="side_right",
    )
    carcass.visual(
        Box((INNER_W, panel_depth, BOTTOM_T)),
        origin=Origin(xyz=(0.0, panel_center_y, PLINTH_H + BOTTOM_T / 2.0)),
        material="case_oak",
        name="bottom_panel",
    )
    carcass.visual(
        Box((INNER_W, panel_depth, SHELF_T)),
        origin=Origin(xyz=(0.0, panel_center_y, LOWER_SHELF_Z)),
        material="case_oak",
        name="drawer_divider",
    )
    carcass.visual(
        Box((INNER_W, panel_depth, SHELF_T)),
        origin=Origin(xyz=(0.0, panel_center_y, UPPER_SHELF_Z)),
        material="case_oak",
        name="tray_divider",
    )
    carcass.visual(
        Box((INNER_W, BACK_T, side_h)),
        origin=Origin(xyz=(0.0, BACK_PANEL_CENTER_Y, side_z)),
        material="case_oak",
        name="back_panel",
    )

    runner_x = INNER_FACE_X - RUNNER_T / 2.0
    carcass.visual(
        Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(-runner_x, RUNNER_CENTER_Y, LOWER_RUNNER_Z)),
        material="runner_metal",
        name="lower_runner_left",
    )
    carcass.visual(
        Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(runner_x, RUNNER_CENTER_Y, LOWER_RUNNER_Z)),
        material="runner_metal",
        name="lower_runner_right",
    )
    carcass.visual(
        Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(-runner_x, RUNNER_CENTER_Y, UPPER_RUNNER_Z)),
        material="runner_metal",
        name="upper_runner_left",
    )
    carcass.visual(
        Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(runner_x, RUNNER_CENTER_Y, UPPER_RUNNER_Z)),
        material="runner_metal",
        name="upper_runner_right",
    )

    guide_x = INNER_FACE_X - GUIDE_T / 2.0
    carcass.visual(
        Box((GUIDE_T, GUIDE_LEN, GUIDE_H)),
        origin=Origin(xyz=(-guide_x, GUIDE_CENTER_Y, TRAY_GUIDE_Z)),
        material="runner_metal",
        name="tray_guide_left",
    )
    carcass.visual(
        Box((GUIDE_T, GUIDE_LEN, GUIDE_H)),
        origin=Origin(xyz=(guide_x, GUIDE_CENTER_Y, TRAY_GUIDE_Z)),
        material="runner_metal",
        name="tray_guide_right",
    )

    lower_drawer = model.part("lower_drawer")
    _build_drawer(
        lower_drawer,
        prefix="lower",
        front_w=LOWER_FRONT_W,
        front_h=LOWER_FRONT_H,
        box_h=LOWER_BOX_H,
        slide_left_name="lower_slide_left",
        slide_right_name="lower_slide_right",
    )

    upper_drawer = model.part("upper_drawer")
    _build_drawer(
        upper_drawer,
        prefix="upper",
        front_w=UPPER_FRONT_W,
        front_h=UPPER_FRONT_H,
        box_h=UPPER_BOX_H,
        slide_left_name="upper_slide_left",
        slide_right_name="upper_slide_right",
    )

    tray = model.part("tray")
    _build_tray(tray)

    model.articulation(
        "lower_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=lower_drawer,
        origin=Origin(xyz=(0.0, FRONT_PLANE_Y, LOWER_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=LOWER_DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "upper_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=upper_drawer,
        origin=Origin(xyz=(0.0, FRONT_PLANE_Y, UPPER_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=UPPER_DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tray,
        origin=Origin(xyz=(0.0, FRONT_PLANE_Y, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_drawer = object_model.get_part("upper_drawer")
    tray = object_model.get_part("tray")

    lower_slide = object_model.get_articulation("lower_drawer_slide")
    upper_slide = object_model.get_articulation("upper_drawer_slide")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.expect_gap(
        carcass,
        tray,
        axis="z",
        positive_elem="top_panel",
        negative_elem="tray_board",
        min_gap=0.025,
        max_gap=0.055,
        name="tray board stays tucked below the top panel",
    )

    ctx.expect_overlap(
        lower_drawer,
        carcass,
        axes="y",
        elem_a="lower_slide_right",
        elem_b="lower_runner_right",
        min_overlap=0.065,
        name="lower drawer keeps runner engagement when closed",
    )
    ctx.expect_overlap(
        upper_drawer,
        carcass,
        axes="y",
        elem_a="upper_slide_right",
        elem_b="upper_runner_right",
        min_overlap=0.080,
        name="upper drawer keeps runner engagement when closed",
    )
    ctx.expect_overlap(
        tray,
        carcass,
        axes="y",
        elem_a="tray_slide_right",
        elem_b="tray_guide_right",
        min_overlap=0.060,
        name="tray stays engaged on its side guide when closed",
    )

    lower_rest = ctx.part_world_position(lower_drawer)
    upper_rest = ctx.part_world_position(upper_drawer)
    tray_rest = ctx.part_world_position(tray)

    with ctx.pose({lower_slide: LOWER_DRAWER_TRAVEL}):
        ctx.expect_overlap(
            lower_drawer,
            carcass,
            axes="y",
            elem_a="lower_slide_right",
            elem_b="lower_runner_right",
            min_overlap=0.035,
            name="lower drawer keeps runner engagement when extended",
        )
        lower_extended = ctx.part_world_position(lower_drawer)

    with ctx.pose({upper_slide: UPPER_DRAWER_TRAVEL}):
        ctx.expect_overlap(
            upper_drawer,
            carcass,
            axes="y",
            elem_a="upper_slide_right",
            elem_b="upper_runner_right",
            min_overlap=0.040,
            name="upper drawer keeps runner engagement when extended",
        )
        upper_extended = ctx.part_world_position(upper_drawer)

    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_overlap(
            tray,
            carcass,
            axes="y",
            elem_a="tray_slide_right",
            elem_b="tray_guide_right",
            min_overlap=0.035,
            name="tray keeps guide engagement when extended",
        )
        tray_extended = ctx.part_world_position(tray)

    ctx.check(
        "lower drawer extends outward",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[1] > lower_rest[1] + 0.15,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "upper drawer extends outward",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] > upper_rest[1] + 0.12,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )
    ctx.check(
        "tray extends outward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] > tray_rest[1] + 0.10,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
