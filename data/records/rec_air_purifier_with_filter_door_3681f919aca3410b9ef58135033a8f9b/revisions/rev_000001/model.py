from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BODY_W = 0.40
BODY_H = 0.68
BODY_D = 0.14
BACK_T = 0.018
SHELL_T = 0.020
FRAME_T = 0.014
OPEN_W = 0.324
OPEN_H = 0.576

DOOR_T = 0.004
DOOR_W = 0.336
DOOR_H = 0.588
DOOR_OPEN_ANGLE = math.radians(72.0)

FILTER_W = 0.302
FILTER_H = 0.552
FILTER_D = 0.080
FILTER_TRAVEL = 0.060

SLIDER_TRAVEL = 0.055


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.94, 1.0))
    frame_white = model.material("frame_white", rgba=(0.90, 0.91, 0.89, 1.0))
    soft_grey = model.material("soft_grey", rgba=(0.75, 0.77, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.28, 0.31, 0.30, 1.0))
    indicator_blue = model.material("indicator_blue", rgba=(0.40, 0.62, 0.74, 0.85))
    slider_black = model.material("slider_black", rgba=(0.10, 0.11, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((BACK_T, BODY_W, BODY_H)),
        origin=Origin(xyz=(-BODY_D / 2.0 + BACK_T / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="back_panel",
    )
    housing.visual(
        Box((BODY_D, SHELL_T, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + SHELL_T / 2.0, 0.0)),
        material=shell_white,
        name="left_wall",
    )
    housing.visual(
        Box((BODY_D, SHELL_T, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - SHELL_T / 2.0, 0.0)),
        material=shell_white,
        name="right_wall",
    )
    housing.visual(
        Box((BODY_D, BODY_W - 2.0 * SHELL_T, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 - SHELL_T / 2.0)),
        material=shell_white,
        name="top_wall",
    )
    housing.visual(
        Box((BODY_D, BODY_W - 2.0 * SHELL_T, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_H / 2.0 + SHELL_T / 2.0)),
        material=shell_white,
        name="bottom_wall",
    )
    housing.visual(
        Box((FRAME_T, (BODY_W - OPEN_W) / 2.0, OPEN_H)),
        origin=Origin(
            xyz=(
                BODY_D / 2.0 - FRAME_T / 2.0,
                -(OPEN_W / 2.0 + (BODY_W - OPEN_W) / 4.0),
                0.0,
            )
        ),
        material=frame_white,
        name="front_frame_0",
    )
    housing.visual(
        Box((FRAME_T, (BODY_W - OPEN_W) / 2.0, OPEN_H)),
        origin=Origin(
            xyz=(
                BODY_D / 2.0 - FRAME_T / 2.0,
                OPEN_W / 2.0 + (BODY_W - OPEN_W) / 4.0,
                0.0,
            )
        ),
        material=frame_white,
        name="front_frame_1",
    )
    housing.visual(
        Box((FRAME_T, OPEN_W, (BODY_H - OPEN_H) / 2.0)),
        origin=Origin(
            xyz=(
                BODY_D / 2.0 - FRAME_T / 2.0,
                0.0,
                OPEN_H / 2.0 + (BODY_H - OPEN_H) / 4.0,
            )
        ),
        material=frame_white,
        name="front_frame_2",
    )
    housing.visual(
        Box((FRAME_T, OPEN_W, (BODY_H - OPEN_H) / 2.0)),
        origin=Origin(
            xyz=(
                BODY_D / 2.0 - FRAME_T / 2.0,
                0.0,
                -(OPEN_H / 2.0 + (BODY_H - OPEN_H) / 4.0),
            )
        ),
        material=frame_white,
        name="front_frame_3",
    )
    housing.visual(
        Box((0.004, 0.020, DOOR_H)),
        origin=Origin(xyz=(BODY_D / 2.0, -DOOR_W / 2.0 + 0.010, 0.0)),
        material=soft_grey,
        name="hinge_mount",
    )
    housing.visual(
        Box((0.060, 0.029, 0.520)),
        origin=Origin(xyz=(0.010, -(BODY_W / 2.0 - 0.0345), 0.0)),
        material=soft_grey,
        name="filter_guide_0",
    )
    housing.visual(
        Box((0.060, 0.029, 0.520)),
        origin=Origin(xyz=(0.010, BODY_W / 2.0 - 0.0345, 0.0)),
        material=soft_grey,
        name="filter_guide_1",
    )
    housing.visual(
        Box((0.012, 0.002, 0.110)),
        origin=Origin(xyz=(0.034, BODY_W / 2.0 - 0.001, -0.065)),
        material=soft_grey,
        name="slider_guide",
    )
    housing.visual(
        Box((0.050, 0.080, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, BODY_H / 2.0 - 0.005)),
        material=soft_grey,
        name="top_cap",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_T / 2.0, DOOR_W / 2.0, DOOR_H / 2.0)),
        material=frame_white,
        name="door_panel",
    )
    door.visual(
        Box((0.010, 0.300, 0.548)),
        origin=Origin(xyz=(-0.003, DOOR_W / 2.0, DOOR_H / 2.0)),
        material=soft_grey,
        name="door_liner",
    )
    door.visual(
        Box((0.008, 0.018, DOOR_H)),
        origin=Origin(xyz=(0.004, 0.009, DOOR_H / 2.0)),
        material=shell_white,
        name="hinge_spine",
    )
    door.visual(
        Box((0.016, 0.018, 0.240)),
        origin=Origin(xyz=(0.010, DOOR_W - 0.022, DOOR_H * 0.54)),
        material=soft_grey,
        name="pull_rail",
    )
    door.visual(
        Box((0.002, 0.090, 0.010)),
        origin=Origin(xyz=(0.002, DOOR_W / 2.0, 0.090)),
        material=indicator_blue,
        name="status_lens",
    )

    filter_cassette = model.part("filter_cassette")
    frame_band = 0.016
    filter_cassette.visual(
        Box((FILTER_D, frame_band, FILTER_H)),
        origin=Origin(xyz=(-FILTER_D / 2.0, -(FILTER_W / 2.0 - frame_band / 2.0), 0.0)),
        material=charcoal,
        name="filter_frame_0",
    )
    filter_cassette.visual(
        Box((FILTER_D, frame_band, FILTER_H)),
        origin=Origin(xyz=(-FILTER_D / 2.0, FILTER_W / 2.0 - frame_band / 2.0, 0.0)),
        material=charcoal,
        name="filter_frame_1",
    )
    filter_cassette.visual(
        Box((FILTER_D, FILTER_W - 2.0 * frame_band, frame_band)),
        origin=Origin(xyz=(-FILTER_D / 2.0, 0.0, FILTER_H / 2.0 - frame_band / 2.0)),
        material=charcoal,
        name="filter_frame_2",
    )
    filter_cassette.visual(
        Box((FILTER_D, FILTER_W - 2.0 * frame_band, frame_band)),
        origin=Origin(xyz=(-FILTER_D / 2.0, 0.0, -(FILTER_H / 2.0 - frame_band / 2.0))),
        material=charcoal,
        name="filter_frame_3",
    )
    filter_cassette.visual(
        Box((0.062, FILTER_W - 2.0 * frame_band + 0.001, FILTER_H - 2.0 * frame_band + 0.001)),
        origin=Origin(xyz=(-FILTER_D / 2.0, 0.0, 0.0)),
        material=filter_dark,
        name="filter_media",
    )
    filter_cassette.visual(
        Box((0.018, 0.086, 0.028)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=soft_grey,
        name="filter_tab",
    )

    side_slider = model.part("side_slider")
    side_slider.visual(
        Box((0.006, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.004, 0.030)),
        material=slider_black,
        name="slider_stem",
    )
    side_slider.visual(
        Box((0.016, 0.022, 0.022)),
        origin=Origin(xyz=(0.003, 0.011, 0.012)),
        material=slider_black,
        name="thumb_tab",
    )
    side_slider.visual(
        Box((0.018, 0.010, 0.003)),
        origin=Origin(xyz=(0.006, 0.022, 0.012)),
        material=soft_grey,
        name="thumb_ridge",
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(BODY_D / 2.0 + 0.002, -DOOR_W / 2.0, -DOOR_H / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_cassette,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )
    model.articulation(
        "housing_to_side_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=side_slider,
        origin=Origin(xyz=(0.034, BODY_W / 2.0, -0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    filter_cassette = object_model.get_part("filter_cassette")
    side_slider = object_model.get_part("side_slider")

    door_hinge = object_model.get_articulation("housing_to_door")
    filter_slide = object_model.get_articulation("housing_to_filter")
    speed_slide = object_model.get_articulation("housing_to_side_slider")

    ctx.expect_gap(
        door,
        housing,
        axis="x",
        positive_elem="door_panel",
        max_gap=0.010,
        max_penetration=0.0,
        name="closed door sits slightly proud of the purifier frame",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="yz",
        elem_a="door_panel",
        min_overlap=0.30,
        name="door covers the purifier front footprint",
    )
    ctx.expect_gap(
        door,
        filter_cassette,
        axis="x",
        positive_elem="door_panel",
        max_gap=0.030,
        max_penetration=0.0,
        name="filter cassette rests behind the closed door",
    )
    ctx.expect_within(
        filter_cassette,
        housing,
        axes="yz",
        margin=0.0,
        name="filter cassette fits inside the purifier bay",
    )
    ctx.expect_gap(
        side_slider,
        housing,
        axis="y",
        positive_elem="thumb_tab",
        max_gap=0.002,
        max_penetration=0.0,
        name="side slider stays mounted on the right wall",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "door swings outward on a vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and float(open_door_aabb[1][0]) > float(closed_door_aabb[1][0]) + 0.15,
        details=f"closed={closed_door_aabb!r}, open={open_door_aabb!r}",
    )

    filter_rest_pos = ctx.part_world_position(filter_cassette)
    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE, filter_slide: FILTER_TRAVEL}):
        filter_extended_pos = ctx.part_world_position(filter_cassette)
        ctx.expect_within(
            filter_cassette,
            housing,
            axes="yz",
            margin=0.0,
            name="extended filter stays aligned with the purifier opening",
        )
        ctx.expect_overlap(
            filter_cassette,
            housing,
            axes="x",
            min_overlap=0.020,
            name="extended filter retains insertion in the purifier body",
        )

    ctx.check(
        "filter cassette slides out from the front opening",
        filter_rest_pos is not None
        and filter_extended_pos is not None
        and float(filter_extended_pos[0]) > float(filter_rest_pos[0]) + 0.050,
        details=f"rest={filter_rest_pos!r}, extended={filter_extended_pos!r}",
    )

    slider_rest_pos = ctx.part_world_position(side_slider)
    with ctx.pose({speed_slide: SLIDER_TRAVEL}):
        slider_high_pos = ctx.part_world_position(side_slider)
        ctx.expect_gap(
            side_slider,
            housing,
            axis="y",
            positive_elem="thumb_tab",
            max_gap=0.003,
            max_penetration=0.0,
            name="slider remains close to its guide at high speed setting",
        )

    ctx.check(
        "side slider moves upward along its short guide",
        slider_rest_pos is not None
        and slider_high_pos is not None
        and float(slider_high_pos[2]) > float(slider_rest_pos[2]) + 0.045,
        details=f"rest={slider_rest_pos!r}, high={slider_high_pos!r}",
    )

    return ctx.report()


object_model = build_object_model()
