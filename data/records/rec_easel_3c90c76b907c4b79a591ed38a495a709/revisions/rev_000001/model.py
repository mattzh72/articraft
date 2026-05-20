from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.82
BASE_DEPTH = 0.66
BASE_THICKNESS = 0.09
POST_WIDTH = 0.075
POST_DEPTH = 0.05
POST_HEIGHT = 1.74
POST_X = 0.28
TOP_BEAM_WIDTH = 0.64
TOP_BEAM_HEIGHT = 0.08
LOWER_BEAM_HEIGHT = 0.07
LOWER_BEAM_Z = 0.29
MAST_WIDTH = 0.09
MAST_DEPTH = 0.055
MAST_HEIGHT = 1.66
MAST_Z = 0.92

SLIDE_ORIGIN_Z = 0.52
SLIDE_TRAVEL = 0.70

COLLAR_WIDTH = 0.156
COLLAR_DEPTH = 0.112
COLLAR_HEIGHT = 0.28
COLLAR_RAIL_THICKNESS = 0.03
COLLAR_PLATE_THICKNESS = 0.024

TRAY_TILT_FORWARD = -0.35
TRAY_TILT_BACK = 0.45


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_h_frame_easel")

    oak = model.material("oak", rgba=(0.56, 0.40, 0.25, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.42, 0.28, 0.16, 1.0))
    frame = model.part("frame")
    frame.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2)),
        material=dark_oak,
        name="base_plinth",
    )
    frame.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(-POST_X, 0.0, BASE_THICKNESS + POST_HEIGHT / 2)),
        material=oak,
        name="side_post_0",
    )
    frame.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(POST_X, 0.0, BASE_THICKNESS + POST_HEIGHT / 2)),
        material=oak,
        name="side_post_1",
    )
    frame.visual(
        Box((TOP_BEAM_WIDTH, POST_DEPTH, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT - TOP_BEAM_HEIGHT / 2)),
        material=oak,
        name="top_beam",
    )
    frame.visual(
        Box((TOP_BEAM_WIDTH, POST_DEPTH, LOWER_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_BEAM_Z)),
        material=oak,
        name="lower_beam",
    )
    frame.visual(
        Box((MAST_WIDTH, MAST_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_Z)),
        material=oak,
        name="mast",
    )
    frame.visual(
        Box((0.19, 0.12, 0.20)),
        origin=Origin(xyz=(-0.25, 0.0, 0.19)),
        material=dark_oak,
        name="base_block_0",
    )
    frame.visual(
        Box((0.19, 0.12, 0.20)),
        origin=Origin(xyz=(0.25, 0.0, 0.19)),
        material=dark_oak,
        name="base_block_1",
    )

    carriage = model.part("carriage")
    rail_x = MAST_WIDTH / 2 + COLLAR_RAIL_THICKNESS / 2
    plate_y = COLLAR_DEPTH / 2 - COLLAR_PLATE_THICKNESS / 2
    carriage.visual(
        Box((COLLAR_RAIL_THICKNESS, COLLAR_DEPTH, COLLAR_HEIGHT)),
        origin=Origin(xyz=(-rail_x, 0.0, 0.0)),
        material=dark_oak,
        name="sleeve_left",
    )
    carriage.visual(
        Box((COLLAR_RAIL_THICKNESS, COLLAR_DEPTH, COLLAR_HEIGHT)),
        origin=Origin(xyz=(rail_x, 0.0, 0.0)),
        material=dark_oak,
        name="sleeve_right",
    )
    carriage.visual(
        Box((COLLAR_WIDTH, COLLAR_PLATE_THICKNESS, COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, -plate_y, 0.0)),
        material=dark_oak,
        name="sleeve_back",
    )
    carriage.visual(
        Box((COLLAR_WIDTH, COLLAR_PLATE_THICKNESS, COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, plate_y, 0.0)),
        material=dark_oak,
        name="sleeve_front",
    )
    carriage.visual(
        Box((0.14, 0.03, 0.75)),
        origin=Origin(xyz=(0.0, 0.071, 0.18)),
        material=oak,
        name="carriage_spine",
    )
    carriage.visual(
        Box((0.26, 0.038, 0.09)),
        origin=Origin(xyz=(0.0, 0.075, 0.60)),
        material=oak,
        name="top_holder",
    )
    carriage.visual(
        Box((0.28, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.072, -0.215)),
        material=oak,
        name="lower_carriage",
    )
    carriage.visual(
        Box((0.24, 0.01, 0.05)),
        origin=Origin(xyz=(0.0, 0.102, -0.17)),
        material=oak,
        name="hinge_mount",
    )
    tray = model.part("tray")
    tray.visual(
        Box((0.50, 0.03, 0.44)),
        origin=Origin(xyz=(0.0, 0.015, 0.22)),
        material=oak,
        name="tray_panel",
    )
    tray.visual(
        Box((0.54, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.12, 0.015)),
        material=oak,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.54, 0.035, 0.06)),
        origin=Origin(xyz=(0.0, 0.2225, 0.06)),
        material=oak,
        name="tray_lip",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.0, 0.107, -0.195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=TRAY_TILT_FORWARD,
            upper=TRAY_TILT_BACK,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    mast_slide = object_model.get_articulation("mast_slide")
    tray_tilt = object_model.get_articulation("tray_tilt")

    ctx.expect_origin_distance(
        carriage,
        frame,
        axes="xy",
        max_dist=0.001,
        name="carriage stays centered on the mast axis",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="sleeve_front",
        negative_elem="base_plinth",
        min_gap=0.25,
        name="lower carriage clears the heavy base",
    )
    ctx.expect_gap(
        tray,
        carriage,
        axis="y",
        positive_elem="tray_lip",
        negative_elem="carriage_spine",
        min_gap=0.11,
        name="tray projects forward of the carriage",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({mast_slide: SLIDE_TRAVEL}):
        raised_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides upward along the mast",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.60,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")
    with ctx.pose({tray_tilt: 0.35}):
        tilted_panel_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")
    ctx.check(
        "tray tilts backward around the lower hinge",
        rest_panel_aabb is not None
        and tilted_panel_aabb is not None
        and tilted_panel_aabb[0][1] < rest_panel_aabb[0][1] - 0.10,
        details=f"rest={rest_panel_aabb}, tilted={tilted_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
