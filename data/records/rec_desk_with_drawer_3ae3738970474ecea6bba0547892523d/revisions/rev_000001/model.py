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


DESK_WIDTH = 1.05
DESK_DEPTH = 0.55
DESK_HEIGHT = 0.75
TOP_THICKNESS = 0.022
UNDERSIDE_Z = DESK_HEIGHT - TOP_THICKNESS

FRAME_HEIGHT = 0.070
FRAME_THICKNESS = 0.020
LEG_SIZE = 0.035

PEDESTAL_WIDTH = 0.36
PEDESTAL_DEPTH = 0.50
PEDESTAL_CENTER_X = DESK_DEPTH / 2.0 - PEDESTAL_DEPTH / 2.0
PEDESTAL_CENTER_Y = DESK_WIDTH / 2.0 - 0.015 - PEDESTAL_WIDTH / 2.0
PEDESTAL_HEIGHT = UNDERSIDE_Z
PEDESTAL_SIDE_THICKNESS = 0.018
PEDESTAL_BACK_THICKNESS = 0.012
PEDESTAL_SHELF_THICKNESS = 0.018

TOP_DRAWER_TRAVEL = 0.20
LOWER_DRAWER_TRAVEL = 0.24


def _front_flush_with_desktop(ctx: TestContext, drawer, elem_name: str, check_name: str) -> bool:
    drawer_aabb = ctx.part_element_world_aabb(drawer, elem=elem_name)
    desktop_aabb = ctx.part_element_world_aabb("desk", elem="desktop")
    if drawer_aabb is None or desktop_aabb is None:
        return ctx.fail(check_name, f"missing aabb: drawer={drawer_aabb}, desktop={desktop_aabb}")
    front_delta = abs(drawer_aabb[1][0] - desktop_aabb[1][0])
    return ctx.check(check_name, front_delta <= 0.0015, details=f"front_delta={front_delta:.6f}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="study_desk_right_pedestal")

    wood = model.material("wood", rgba=(0.71, 0.57, 0.42, 1.0))
    frame = model.material("frame", rgba=(0.90, 0.90, 0.88, 1.0))
    drawer_box = model.material("drawer_box", rgba=(0.83, 0.84, 0.85, 1.0))
    metal = model.material("metal", rgba=(0.23, 0.24, 0.27, 1.0))

    desk = model.part("desk")

    desk.visual(
        Box((DESK_DEPTH, DESK_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0)),
        material=wood,
        name="desktop",
    )

    left_leg_y = -DESK_WIDTH / 2.0 + 0.040 + LEG_SIZE / 2.0
    front_leg_x = DESK_DEPTH / 2.0 - 0.040 - LEG_SIZE / 2.0
    rear_leg_x = -DESK_DEPTH / 2.0 + 0.040 + LEG_SIZE / 2.0
    leg_z = UNDERSIDE_Z / 2.0

    desk.visual(
        Box((LEG_SIZE, LEG_SIZE, UNDERSIDE_Z)),
        origin=Origin(xyz=(front_leg_x, left_leg_y, leg_z)),
        material=frame,
        name="front_leg",
    )
    desk.visual(
        Box((LEG_SIZE, LEG_SIZE, UNDERSIDE_Z)),
        origin=Origin(xyz=(rear_leg_x, left_leg_y, leg_z)),
        material=frame,
        name="rear_leg",
    )

    knee_apron_width = (PEDESTAL_CENTER_Y - PEDESTAL_WIDTH / 2.0) - (left_leg_y + LEG_SIZE / 2.0)
    knee_apron_center_y = (PEDESTAL_CENTER_Y - PEDESTAL_WIDTH / 2.0 + left_leg_y + LEG_SIZE / 2.0) / 2.0
    apron_z = UNDERSIDE_Z - FRAME_HEIGHT / 2.0

    desk.visual(
        Box((FRAME_THICKNESS, knee_apron_width, FRAME_HEIGHT)),
        origin=Origin(xyz=(DESK_DEPTH / 2.0 - FRAME_THICKNESS / 2.0, knee_apron_center_y, apron_z)),
        material=frame,
        name="front_apron",
    )

    rear_span_left = left_leg_y + LEG_SIZE / 2.0
    rear_span_right = PEDESTAL_CENTER_Y + PEDESTAL_WIDTH / 2.0
    rear_apron_width = rear_span_right - rear_span_left
    rear_apron_center_y = (rear_span_right + rear_span_left) / 2.0
    desk.visual(
        Box((FRAME_THICKNESS, rear_apron_width, FRAME_HEIGHT)),
        origin=Origin(xyz=(-DESK_DEPTH / 2.0 + FRAME_THICKNESS / 2.0, rear_apron_center_y, apron_z)),
        material=frame,
        name="rear_apron",
    )

    left_side_rail_length = 2.0 * (front_leg_x - LEG_SIZE / 2.0)
    desk.visual(
        Box((left_side_rail_length, FRAME_THICKNESS, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, left_leg_y, apron_z)),
        material=frame,
        name="left_side_rail",
    )

    modesty_panel_width = (PEDESTAL_CENTER_Y - PEDESTAL_WIDTH / 2.0) - (left_leg_y + LEG_SIZE / 2.0)
    desk.visual(
        Box((0.012, modesty_panel_width, 0.24)),
        origin=Origin(
            xyz=(
                -DESK_DEPTH / 2.0 + FRAME_THICKNESS + 0.006,
                knee_apron_center_y,
                apron_z - FRAME_HEIGHT / 2.0 - 0.12,
            )
        ),
        material=frame,
        name="modesty_panel",
    )

    side_panel_z = PEDESTAL_HEIGHT / 2.0
    desk.visual(
        Box((PEDESTAL_DEPTH, PEDESTAL_SIDE_THICKNESS, PEDESTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                PEDESTAL_CENTER_X,
                PEDESTAL_CENTER_Y - PEDESTAL_WIDTH / 2.0 + PEDESTAL_SIDE_THICKNESS / 2.0,
                side_panel_z,
            )
        ),
        material=frame,
        name="left_pedestal_side",
    )
    desk.visual(
        Box((PEDESTAL_DEPTH, PEDESTAL_SIDE_THICKNESS, PEDESTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                PEDESTAL_CENTER_X,
                PEDESTAL_CENTER_Y + PEDESTAL_WIDTH / 2.0 - PEDESTAL_SIDE_THICKNESS / 2.0,
                side_panel_z,
            )
        ),
        material=frame,
        name="right_pedestal_side",
    )

    desk.visual(
        Box((PEDESTAL_BACK_THICKNESS, PEDESTAL_WIDTH - 2.0 * PEDESTAL_SIDE_THICKNESS, PEDESTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                PEDESTAL_CENTER_X - PEDESTAL_DEPTH / 2.0 + PEDESTAL_BACK_THICKNESS / 2.0,
                PEDESTAL_CENTER_Y,
                side_panel_z,
            )
        ),
        material=frame,
        name="pedestal_back",
    )

    bottom_and_divider_depth = PEDESTAL_DEPTH - PEDESTAL_BACK_THICKNESS - 0.040
    bottom_and_divider_center_x = (
        PEDESTAL_CENTER_X - PEDESTAL_DEPTH / 2.0 + PEDESTAL_BACK_THICKNESS + bottom_and_divider_depth / 2.0
    )
    interior_width = PEDESTAL_WIDTH - 2.0 * PEDESTAL_SIDE_THICKNESS

    desk.visual(
        Box((bottom_and_divider_depth, interior_width, PEDESTAL_SHELF_THICKNESS)),
        origin=Origin(xyz=(bottom_and_divider_center_x, PEDESTAL_CENTER_Y, PEDESTAL_SHELF_THICKNESS / 2.0)),
        material=frame,
        name="pedestal_floor",
    )

    divider_z = 0.530
    desk.visual(
        Box((bottom_and_divider_depth, interior_width, PEDESTAL_SHELF_THICKNESS)),
        origin=Origin(xyz=(bottom_and_divider_center_x, PEDESTAL_CENTER_Y, divider_z)),
        material=frame,
        name="drawer_divider",
    )

    rail_width = interior_width
    desk.visual(
        Box((FRAME_THICKNESS, rail_width, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                DESK_DEPTH / 2.0 - FRAME_THICKNESS / 2.0,
                PEDESTAL_CENTER_Y,
                apron_z,
            )
        ),
        material=frame,
        name="top_rail",
    )

    guide_thickness = 0.008
    guide_height = 0.018
    guide_half_offset = interior_width / 2.0 - guide_thickness / 2.0
    top_guide_z = 0.589
    lower_guide_z = 0.266
    guide_length_top = 0.420
    guide_length_lower = 0.430
    guide_center_x_top = DESK_DEPTH / 2.0 - 0.020 - guide_length_top / 2.0
    guide_center_x_lower = DESK_DEPTH / 2.0 - 0.020 - guide_length_lower / 2.0

    for y_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        desk.visual(
            Box((guide_length_top, guide_thickness, guide_height)),
            origin=Origin(
                xyz=(
                    guide_center_x_top,
                    PEDESTAL_CENTER_Y + y_sign * guide_half_offset,
                    top_guide_z,
                )
            ),
            material=metal,
            name=f"top_guide_{side_name}",
        )
        desk.visual(
            Box((guide_length_lower, guide_thickness, guide_height)),
            origin=Origin(
                xyz=(
                    guide_center_x_lower,
                    PEDESTAL_CENTER_Y + y_sign * guide_half_offset,
                    lower_guide_z,
                )
            ),
            material=metal,
            name=f"lower_guide_{side_name}",
        )

    top_drawer = model.part("top_drawer")
    top_front_height = 0.110
    top_front_width = interior_width - 0.008
    top_front_center_z = 0.599
    top_drawer.visual(
        Box((0.018, top_front_width, top_front_height)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=wood,
        name="top_front",
    )
    top_drawer.visual(
        Box((0.350, 0.010, 0.080)),
        origin=Origin(xyz=(-0.193, -0.145, -0.008)),
        material=drawer_box,
        name="left_side",
    )
    top_drawer.visual(
        Box((0.350, 0.010, 0.080)),
        origin=Origin(xyz=(-0.193, 0.145, -0.008)),
        material=drawer_box,
        name="right_side",
    )
    top_drawer.visual(
        Box((0.350, 0.290, 0.010)),
        origin=Origin(xyz=(-0.193, 0.0, -0.043)),
        material=drawer_box,
        name="bottom",
    )
    top_drawer.visual(
        Box((0.010, 0.290, 0.080)),
        origin=Origin(xyz=(-0.373, 0.0, -0.008)),
        material=drawer_box,
        name="back",
    )
    top_drawer.visual(
        Box((0.340, 0.006, 0.016)),
        origin=Origin(xyz=(-0.188, -0.151, -0.010)),
        material=metal,
        name="left_runner",
    )
    top_drawer.visual(
        Box((0.340, 0.006, 0.016)),
        origin=Origin(xyz=(-0.188, 0.151, -0.010)),
        material=metal,
        name="right_runner",
    )
    top_drawer.visual(
        Box((0.010, 0.130, 0.010)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material=metal,
        name="handle_bar",
    )
    top_drawer.visual(
        Box((0.026, 0.012, 0.024)),
        origin=Origin(xyz=(0.013, -0.042, 0.0)),
        material=metal,
        name="handle_post_0",
    )
    top_drawer.visual(
        Box((0.026, 0.012, 0.024)),
        origin=Origin(xyz=(0.013, 0.042, 0.0)),
        material=metal,
        name="handle_post_1",
    )

    lower_drawer = model.part("lower_drawer")
    lower_front_height = 0.460
    lower_front_width = interior_width - 0.008
    lower_front_center_z = 0.306
    lower_drawer.visual(
        Box((0.018, lower_front_width, lower_front_height)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=wood,
        name="lower_front",
    )
    lower_drawer.visual(
        Box((0.390, 0.010, 0.420)),
        origin=Origin(xyz=(-0.213, -0.145, -0.010)),
        material=drawer_box,
        name="left_side",
    )
    lower_drawer.visual(
        Box((0.390, 0.010, 0.420)),
        origin=Origin(xyz=(-0.213, 0.145, -0.010)),
        material=drawer_box,
        name="right_side",
    )
    lower_drawer.visual(
        Box((0.390, 0.290, 0.010)),
        origin=Origin(xyz=(-0.213, 0.0, -0.215)),
        material=drawer_box,
        name="bottom",
    )
    lower_drawer.visual(
        Box((0.010, 0.290, 0.420)),
        origin=Origin(xyz=(-0.413, 0.0, -0.010)),
        material=drawer_box,
        name="back",
    )
    lower_drawer.visual(
        Box((0.360, 0.006, 0.018)),
        origin=Origin(xyz=(-0.198, -0.151, -0.040)),
        material=metal,
        name="left_runner",
    )
    lower_drawer.visual(
        Box((0.360, 0.006, 0.018)),
        origin=Origin(xyz=(-0.198, 0.151, -0.040)),
        material=metal,
        name="right_runner",
    )
    lower_drawer.visual(
        Box((0.010, 0.140, 0.010)),
        origin=Origin(xyz=(0.021, 0.0, 0.105)),
        material=metal,
        name="handle_bar",
    )
    lower_drawer.visual(
        Box((0.026, 0.012, 0.024)),
        origin=Origin(xyz=(0.013, -0.046, 0.105)),
        material=metal,
        name="handle_post_0",
    )
    lower_drawer.visual(
        Box((0.026, 0.012, 0.024)),
        origin=Origin(xyz=(0.013, 0.046, 0.105)),
        material=metal,
        name="handle_post_1",
    )

    model.articulation(
        "desk_to_top_drawer",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=top_drawer,
        origin=Origin(xyz=(DESK_DEPTH / 2.0, PEDESTAL_CENTER_Y, top_front_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=TOP_DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "desk_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=lower_drawer,
        origin=Origin(xyz=(DESK_DEPTH / 2.0, PEDESTAL_CENTER_Y, lower_front_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=LOWER_DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk")
    top_drawer = object_model.get_part("top_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    top_joint = object_model.get_articulation("desk_to_top_drawer")
    lower_joint = object_model.get_articulation("desk_to_lower_drawer")

    ctx.expect_gap(
        desk,
        top_drawer,
        axis="z",
        positive_elem="top_rail",
        negative_elem="top_front",
        min_gap=0.003,
        max_gap=0.005,
        name="top drawer sits just below the rail",
    )
    ctx.expect_gap(
        top_drawer,
        lower_drawer,
        axis="z",
        positive_elem="top_front",
        negative_elem="lower_front",
        min_gap=0.007,
        max_gap=0.009,
        name="drawer fronts keep an even reveal",
    )
    ctx.expect_overlap(
        top_drawer,
        desk,
        axes="x",
        elem_a="left_runner",
        elem_b="top_guide_left",
        min_overlap=0.33,
        name="top drawer runner stays engaged at rest",
    )
    ctx.expect_overlap(
        lower_drawer,
        desk,
        axes="x",
        elem_a="left_runner",
        elem_b="lower_guide_left",
        min_overlap=0.35,
        name="lower drawer runner stays engaged at rest",
    )
    _front_flush_with_desktop(ctx, top_drawer, "top_front", "top drawer front is flush with the desktop front")
    _front_flush_with_desktop(ctx, lower_drawer, "lower_front", "lower drawer front is flush with the desktop front")

    top_rest = ctx.part_world_position(top_drawer)
    lower_rest = ctx.part_world_position(lower_drawer)
    with ctx.pose({top_joint: TOP_DRAWER_TRAVEL, lower_joint: LOWER_DRAWER_TRAVEL}):
        ctx.expect_overlap(
            top_drawer,
            desk,
            axes="x",
            elem_a="left_runner",
            elem_b="top_guide_left",
            min_overlap=0.13,
            name="top drawer keeps guide overlap extended",
        )
        ctx.expect_overlap(
            lower_drawer,
            desk,
            axes="x",
            elem_a="left_runner",
            elem_b="lower_guide_left",
            min_overlap=0.11,
            name="lower drawer keeps guide overlap extended",
        )
        top_extended = ctx.part_world_position(top_drawer)
        lower_extended = ctx.part_world_position(lower_drawer)

    ctx.check(
        "top drawer extends forward",
        top_rest is not None and top_extended is not None and top_extended[0] > top_rest[0] + 0.18,
        details=f"rest={top_rest}, extended={top_extended}",
    )
    ctx.check(
        "lower drawer extends forward",
        lower_rest is not None and lower_extended is not None and lower_extended[0] > lower_rest[0] + 0.22,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    return ctx.report()


object_model = build_object_model()
