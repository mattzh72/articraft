from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Some harness stages appear to retain a deleted working directory. Guard both
# os.getcwd() and the process cwd itself so any later pathlib absolute/resolve
# call can still succeed.
_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        return "/tmp"


os.getcwd = _safe_getcwd
try:
    _ORIG_GETCWD()
except FileNotFoundError:
    os.chdir("/tmp")

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

CASE_W = 0.220
CASE_D = 0.440
CASE_H = 0.450
SHEET = 0.002
BEZEL_T = 0.006

SIDE_PANEL_T = 0.0012
SIDE_PANEL_DEPTH = CASE_D - SHEET - BEZEL_T
SIDE_PANEL_H = CASE_H - 2.0 * SHEET
SIDE_PANEL_LIP_W = 0.006
SIDE_PANEL_LIP_D = 0.018
SIDE_PANEL_LIP_H = 0.414
SIDE_PANEL_OPEN = 1.92

HINGE_R = 0.004
HINGE_LOWER_LEN = 0.090
HINGE_MID_LEN = 0.090
HINGE_UPPER_LEN = 0.070
HINGE_PANEL_LEN = 0.090
HINGE_LOWER_Z = -0.170
HINGE_PANEL_LOWER_Z = -0.080
HINGE_MID_Z = 0.010
HINGE_PANEL_UPPER_Z = 0.100
HINGE_UPPER_Z = 0.180

DRIVE_FACE_W = 0.148
DRIVE_FACE_H = 0.042
DRIVE_CENTER_Z = 0.390

TRAY_BODY_W = 0.138
TRAY_BODY_H = 0.014
TRAY_BODY_D = 0.155
TRAY_FACE_T = 0.004
TRAY_RAIL_T = 0.003
TRAY_RAIL_H = 0.008
TRAY_OPEN = 0.110

GUIDE_T = 0.003
GUIDE_H = 0.028
BAY_DEPTH = 0.180
BAY_PLATE_T = 0.003
BAY_BACKSTOP_T = 0.020

OPENING_SIDE_W = (CASE_W - DRIVE_FACE_W) / 2.0
LOWER_BEZEL_H = DRIVE_CENTER_Z - DRIVE_FACE_H / 2.0
UPPER_BEZEL_H = CASE_H - (DRIVE_CENTER_Z + DRIVE_FACE_H / 2.0)
TRAY_HOME_Y = -CASE_D / 2.0 + TRAY_BODY_D / 2.0 + TRAY_FACE_T
TRAY_RAIL_CENTER_X = TRAY_BODY_W / 2.0 + TRAY_RAIL_T / 2.0
GUIDE_CENTER_X = TRAY_BODY_W / 2.0 + TRAY_RAIL_T + GUIDE_T / 2.0
BAY_Y_CENTER = -CASE_D / 2.0 + SHEET + BAY_DEPTH / 2.0
BAY_PLATE_W = TRAY_BODY_W + 2.0 * (TRAY_RAIL_T + GUIDE_T)
BAY_ROOF_Z = DRIVE_CENTER_Z + GUIDE_H / 2.0 + BAY_PLATE_T / 2.0
BAY_FLOOR_Z = DRIVE_CENTER_Z - GUIDE_H / 2.0 - BAY_PLATE_T / 2.0
BAY_BACKSTOP_W = TRAY_BODY_W + 2.0 * TRAY_RAIL_T
BAY_BACKSTOP_Y = TRAY_HOME_Y + TRAY_BODY_D / 2.0 + BAY_BACKSTOP_T / 2.0

SIDE_PANEL_STOP_W = 0.006
SIDE_PANEL_STOP_D = 0.032
SIDE_PANEL_STOP_H = 0.360


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mid_tower_pc", assets=ASSETS)

    steel = model.material("steel", rgba=(0.24, 0.25, 0.27, 1.0))
    steel_panel = model.material("steel_panel", rgba=(0.20, 0.21, 0.23, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    drive_gray = model.material("drive_gray", rgba=(0.74, 0.76, 0.79, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((CASE_W, CASE_D, SHEET)),
        origin=Origin(xyz=(0.0, 0.0, SHEET / 2.0)),
        material=steel,
        name="bottom_shell",
    )
    chassis.visual(
        Box((CASE_W, CASE_D, SHEET)),
        origin=Origin(xyz=(0.0, 0.0, CASE_H - SHEET / 2.0)),
        material=steel,
        name="top_shell",
    )
    chassis.visual(
        Box((SHEET, CASE_D, CASE_H - 2.0 * SHEET)),
        origin=Origin(xyz=(-CASE_W / 2.0 + SHEET / 2.0, 0.0, CASE_H / 2.0)),
        material=steel,
        name="left_shell",
    )
    chassis.visual(
        Box((CASE_W, SHEET, CASE_H)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - SHEET / 2.0, CASE_H / 2.0)),
        material=steel,
        name="rear_shell",
    )
    chassis.visual(
        Box((OPENING_SIDE_W, BEZEL_T, CASE_H)),
        origin=Origin(
            xyz=(
                -CASE_W / 2.0 + OPENING_SIDE_W / 2.0,
                -CASE_D / 2.0 + BEZEL_T / 2.0,
                CASE_H / 2.0,
            )
        ),
        material=bezel_black,
        name="front_left_bezel",
    )
    chassis.visual(
        Box((OPENING_SIDE_W, BEZEL_T, CASE_H)),
        origin=Origin(
            xyz=(
                CASE_W / 2.0 - OPENING_SIDE_W / 2.0,
                -CASE_D / 2.0 + BEZEL_T / 2.0,
                CASE_H / 2.0,
            )
        ),
        material=bezel_black,
        name="front_right_bezel",
    )
    chassis.visual(
        Box((DRIVE_FACE_W, BEZEL_T, LOWER_BEZEL_H)),
        origin=Origin(
            xyz=(
                0.0,
                -CASE_D / 2.0 + BEZEL_T / 2.0,
                LOWER_BEZEL_H / 2.0,
            )
        ),
        material=bezel_black,
        name="front_lower_bezel",
    )
    chassis.visual(
        Box((DRIVE_FACE_W, BEZEL_T, UPPER_BEZEL_H)),
        origin=Origin(
            xyz=(
                0.0,
                -CASE_D / 2.0 + BEZEL_T / 2.0,
                DRIVE_CENTER_Z + DRIVE_FACE_H / 2.0 + UPPER_BEZEL_H / 2.0,
            )
        ),
        material=bezel_black,
        name="front_upper_bezel",
    )
    chassis.visual(
        Box((SIDE_PANEL_STOP_W, SIDE_PANEL_STOP_D, SIDE_PANEL_STOP_H)),
        origin=Origin(
            xyz=(
                CASE_W / 2.0 - SIDE_PANEL_LIP_W - SIDE_PANEL_STOP_W / 2.0,
                -CASE_D / 2.0 + SHEET + SIDE_PANEL_STOP_D / 2.0,
                CASE_H / 2.0,
            )
        ),
        material=steel,
        name="side_panel_stop",
    )

    chassis.visual(
        Box((BAY_PLATE_W, BAY_DEPTH, BAY_PLATE_T)),
        origin=Origin(xyz=(0.0, BAY_Y_CENTER, BAY_ROOF_Z)),
        material=steel,
        name="bay_roof",
    )
    chassis.visual(
        Box((BAY_PLATE_W, BAY_DEPTH, BAY_PLATE_T)),
        origin=Origin(xyz=(0.0, BAY_Y_CENTER, BAY_FLOOR_Z)),
        material=steel,
        name="bay_floor",
    )
    chassis.visual(
        Box((GUIDE_T, BAY_DEPTH, GUIDE_H)),
        origin=Origin(xyz=(-GUIDE_CENTER_X, BAY_Y_CENTER, DRIVE_CENTER_Z)),
        material=steel,
        name="bay_left_guide",
    )
    chassis.visual(
        Box((GUIDE_T, BAY_DEPTH, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_CENTER_X, BAY_Y_CENTER, DRIVE_CENTER_Z)),
        material=steel,
        name="bay_right_guide",
    )
    chassis.visual(
        Box((BAY_BACKSTOP_W, BAY_BACKSTOP_T, GUIDE_H)),
        origin=Origin(xyz=(0.0, BAY_BACKSTOP_Y, DRIVE_CENTER_Z)),
        material=steel,
        name="bay_backstop",
    )
    chassis.visual(
        Cylinder(radius=HINGE_R, length=HINGE_LOWER_LEN),
        origin=Origin(xyz=(CASE_W / 2.0, CASE_D / 2.0 - SHEET, CASE_H / 2.0 + HINGE_LOWER_Z)),
        material=steel,
        name="chassis_hinge_lower",
    )
    chassis.visual(
        Cylinder(radius=HINGE_R, length=HINGE_MID_LEN),
        origin=Origin(xyz=(CASE_W / 2.0, CASE_D / 2.0 - SHEET, CASE_H / 2.0 + HINGE_MID_Z)),
        material=steel,
        name="chassis_hinge_middle",
    )
    chassis.visual(
        Cylinder(radius=HINGE_R, length=HINGE_UPPER_LEN),
        origin=Origin(xyz=(CASE_W / 2.0, CASE_D / 2.0 - SHEET, CASE_H / 2.0 + HINGE_UPPER_Z)),
        material=steel,
        name="chassis_hinge_upper",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, CASE_H / 2.0)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((SIDE_PANEL_T, SIDE_PANEL_DEPTH, SIDE_PANEL_H)),
        origin=Origin(
            xyz=(
                -SIDE_PANEL_T / 2.0,
                -SIDE_PANEL_DEPTH / 2.0,
                0.0,
            )
        ),
        material=steel_panel,
        name="panel_sheet",
    )
    side_panel.visual(
        Box((SIDE_PANEL_LIP_W, SIDE_PANEL_LIP_D, SIDE_PANEL_LIP_H)),
        origin=Origin(
            xyz=(
                -SIDE_PANEL_LIP_W / 2.0,
                -SIDE_PANEL_DEPTH + SIDE_PANEL_LIP_D / 2.0,
                0.0,
            )
        ),
        material=steel_panel,
        name="front_lip",
    )
    side_panel.visual(
        Cylinder(radius=HINGE_R, length=HINGE_PANEL_LEN),
        origin=Origin(xyz=(0.0, 0.0, HINGE_PANEL_LOWER_Z)),
        material=steel_panel,
        name="panel_hinge_lower",
    )
    side_panel.visual(
        Cylinder(radius=HINGE_R, length=HINGE_PANEL_LEN),
        origin=Origin(xyz=(0.0, 0.0, HINGE_PANEL_UPPER_Z)),
        material=steel_panel,
        name="panel_hinge_upper",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((SIDE_PANEL_LIP_W, SIDE_PANEL_DEPTH, SIDE_PANEL_H)),
        mass=1.8,
        origin=Origin(xyz=(-SIDE_PANEL_LIP_W / 2.0, -SIDE_PANEL_DEPTH / 2.0, 0.0)),
    )

    optical_tray = model.part("optical_tray")
    optical_tray.visual(
        Box((TRAY_BODY_W, TRAY_BODY_D, TRAY_BODY_H)),
        material=drive_gray,
        name="tray_body",
    )
    optical_tray.visual(
        Box((DRIVE_FACE_W, TRAY_FACE_T, DRIVE_FACE_H)),
        origin=Origin(xyz=(0.0, -TRAY_BODY_D / 2.0 - TRAY_FACE_T / 2.0, 0.0)),
        material=bezel_black,
        name="tray_face",
    )
    optical_tray.visual(
        Box((TRAY_RAIL_T, TRAY_BODY_D, TRAY_RAIL_H)),
        origin=Origin(xyz=(-TRAY_RAIL_CENTER_X, 0.0, 0.0)),
        material=drive_gray,
        name="tray_left_rail",
    )
    optical_tray.visual(
        Box((TRAY_RAIL_T, TRAY_BODY_D, TRAY_RAIL_H)),
        origin=Origin(xyz=(TRAY_RAIL_CENTER_X, 0.0, 0.0)),
        material=drive_gray,
        name="tray_right_rail",
    )
    optical_tray.inertial = Inertial.from_geometry(
        Box((DRIVE_FACE_W, TRAY_BODY_D + TRAY_FACE_T, DRIVE_FACE_H)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -TRAY_FACE_T / 2.0, 0.0)),
    )

    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(CASE_W / 2.0, CASE_D / 2.0 - SHEET, CASE_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=SIDE_PANEL_OPEN),
    )
    model.articulation(
        "optical_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=optical_tray,
        origin=Origin(xyz=(0.0, TRAY_HOME_Y, DRIVE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.16, lower=0.0, upper=TRAY_OPEN),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    optical_tray = object_model.get_part("optical_tray")
    side_panel_hinge = object_model.get_articulation("side_panel_hinge")
    optical_tray_slide = object_model.get_articulation("optical_tray_slide")

    side_panel_stop = chassis.get_visual("side_panel_stop")
    bay_left_guide = chassis.get_visual("bay_left_guide")
    bay_right_guide = chassis.get_visual("bay_right_guide")
    bay_backstop = chassis.get_visual("bay_backstop")
    front_lower_bezel = chassis.get_visual("front_lower_bezel")
    chassis_hinge_lower = chassis.get_visual("chassis_hinge_lower")
    chassis_hinge_upper = chassis.get_visual("chassis_hinge_upper")
    panel_sheet = side_panel.get_visual("panel_sheet")
    front_lip = side_panel.get_visual("front_lip")
    panel_hinge_lower = side_panel.get_visual("panel_hinge_lower")
    panel_hinge_upper = side_panel.get_visual("panel_hinge_upper")
    tray_body = optical_tray.get_visual("tray_body")
    tray_face = optical_tray.get_visual("tray_face")
    tray_left_rail = optical_tray.get_visual("tray_left_rail")
    tray_right_rail = optical_tray.get_visual("tray_right_rail")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_no_floating_parts")
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.42, elem_a=panel_sheet)
    ctx.expect_contact(side_panel, chassis, elem_a=front_lip, elem_b=side_panel_stop)
    ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_lower, elem_b=chassis_hinge_lower)
    ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_upper, elem_b=chassis_hinge_upper)

    ctx.expect_contact(optical_tray, chassis, elem_a=tray_left_rail, elem_b=bay_left_guide)
    ctx.expect_contact(optical_tray, chassis, elem_a=tray_right_rail, elem_b=bay_right_guide)
    ctx.expect_contact(optical_tray, chassis, elem_a=tray_body, elem_b=bay_backstop)
    ctx.expect_overlap(optical_tray, chassis, axes="x", min_overlap=0.14, elem_a=tray_face)
    ctx.expect_overlap(optical_tray, chassis, axes="z", min_overlap=0.04, elem_a=tray_face)

    side_limits = side_panel_hinge.motion_limits
    tray_limits = optical_tray_slide.motion_limits

    if side_limits is not None and side_limits.lower is not None and side_limits.upper is not None:
        with ctx.pose({side_panel_hinge: side_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="side_panel_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="side_panel_closed_no_floating")
        with ctx.pose({side_panel_hinge: side_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="side_panel_open_no_overlap")
            ctx.fail_if_isolated_parts(name="side_panel_open_no_floating")
            ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_lower, elem_b=chassis_hinge_lower)
            ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_upper, elem_b=chassis_hinge_upper)
            ctx.expect_gap(
                side_panel,
                chassis,
                axis="x",
                min_gap=0.15,
                positive_elem=front_lip,
                negative_elem=side_panel_stop,
            )

    if tray_limits is not None and tray_limits.lower is not None and tray_limits.upper is not None:
        with ctx.pose({optical_tray_slide: tray_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tray_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="tray_closed_no_floating")
        with ctx.pose({optical_tray_slide: tray_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tray_open_no_overlap")
            ctx.fail_if_isolated_parts(name="tray_open_no_floating")
            ctx.expect_contact(optical_tray, chassis, elem_a=tray_left_rail, elem_b=bay_left_guide)
            ctx.expect_contact(optical_tray, chassis, elem_a=tray_right_rail, elem_b=bay_right_guide)
            ctx.expect_gap(
                chassis,
                optical_tray,
                axis="y",
                min_gap=0.10,
                positive_elem=front_lower_bezel,
                negative_elem=tray_face,
            )
            ctx.expect_overlap(optical_tray, chassis, axes="x", min_overlap=0.13, elem_a=tray_body)
            ctx.expect_overlap(optical_tray, chassis, axes="z", min_overlap=0.01, elem_a=tray_body)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
