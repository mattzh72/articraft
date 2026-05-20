from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_DEPTH = 0.18
BODY_WIDTH = 0.22
BODY_HEIGHT = 0.42
BODY_WALL = 0.004
BODY_CORNER_RADIUS = 0.018

FRONT_OPENING_WIDTH = 0.182
FRONT_OPENING_HEIGHT = 0.158
FRONT_OPENING_BOTTOM = 0.040
PANEL_GAP = 0.002
PANEL_THICKNESS = 0.012
PANEL_WIDTH = FRONT_OPENING_WIDTH - (2.0 * PANEL_GAP)
PANEL_HEIGHT = FRONT_OPENING_HEIGHT - (2.0 * PANEL_GAP)

FILTER_WIDTH = 0.168
FILTER_HEIGHT = 0.148
FILTER_DEPTH = 0.140
FILTER_REST_RECESS = 0.016
FILTER_TRAVEL = 0.095

SWITCH_LENGTH = 0.024
SWITCH_WIDTH = 0.018
SWITCH_HEIGHT = 0.006
SWITCH_X = -0.028
SWITCH_Y = 0.0


def _build_body_shape() -> cq.Workplane:
    front_x = BODY_DEPTH / 2.0
    opening_center_z = FRONT_OPENING_BOTTOM + (FRONT_OPENING_HEIGHT / 2.0)

    body = cq.Workplane("XY").box(
        BODY_DEPTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        centered=(True, True, False),
    )
    body = body.edges("|Z").fillet(BODY_CORNER_RADIUS)

    inner_cavity = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - (2.0 * BODY_WALL),
            BODY_WIDTH - (2.0 * BODY_WALL),
            BODY_HEIGHT - (2.0 * BODY_WALL),
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_WALL))
    )
    body = body.cut(inner_cavity)

    lower_opening = (
        cq.Workplane("XY")
        .box(
            0.028,
            FRONT_OPENING_WIDTH,
            FRONT_OPENING_HEIGHT,
            centered=(True, True, True),
        )
        .translate(
            (
                front_x - 0.010,
                0.0,
                FRONT_OPENING_BOTTOM + (FRONT_OPENING_HEIGHT / 2.0),
            )
        )
    )
    body = body.cut(lower_opening)

    switch_pocket = (
        cq.Workplane("XY")
        .box(0.030, 0.024, 0.014, centered=(True, True, True))
        .translate((SWITCH_X, SWITCH_Y, BODY_HEIGHT - 0.007))
    )
    body = body.cut(switch_pocket)

    for slot_y in (-0.056, -0.034, -0.012, 0.012, 0.034, 0.056):
        top_slot = (
            cq.Workplane("XY")
            .box(0.090, 0.008, 0.012, centered=(True, True, True))
            .translate((-0.008, slot_y, BODY_HEIGHT - 0.003))
        )
        body = body.cut(top_slot)

    for rail_y in (-0.095, 0.095):
        guide_rail = (
            cq.Workplane("XY")
            .box(0.110, 0.022, 0.012, centered=(True, True, True))
            .translate((-0.012, rail_y, opening_center_z))
        )
        body = body.union(guide_rail)

    return body


def _build_front_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PANEL_THICKNESS, PANEL_WIDTH, PANEL_HEIGHT, centered=(False, False, True))
        .translate((0.0, 0.0, 0.0))
    )

    for slot_y in (
        0.022,
        0.046,
        0.070,
        0.094,
        0.118,
        0.142,
        0.166,
    ):
        slot = (
            cq.Workplane("XY")
            .box(PANEL_THICKNESS * 1.4, 0.009, PANEL_HEIGHT - 0.034, centered=(False, True, True))
            .translate((-0.001, slot_y, 0.0))
        )
        panel = panel.cut(slot)

    return panel


def _build_filter_shape() -> cq.Workplane:
    filter_shape = (
        cq.Workplane("XY")
        .box(FILTER_DEPTH, FILTER_WIDTH, FILTER_HEIGHT, centered=(False, True, True))
        .translate((-FILTER_DEPTH, 0.0, 0.0))
    )

    front_frame = (
        cq.Workplane("XY")
        .box(0.010, FILTER_WIDTH, FILTER_HEIGHT, centered=(False, True, True))
        .translate((-0.010, 0.0, 0.0))
    )
    inner_relief = (
        cq.Workplane("XY")
        .box(0.014, FILTER_WIDTH - 0.020, FILTER_HEIGHT - 0.020, centered=(False, True, True))
        .translate((-0.014, 0.0, 0.0))
    )
    filter_shape = filter_shape.union(front_frame.cut(inner_relief))

    for rib_y in (-0.060, -0.042, -0.024, -0.006, 0.012, 0.030, 0.048, 0.066):
        rib = (
            cq.Workplane("XY")
            .box(0.008, 0.006, FILTER_HEIGHT - 0.024, centered=(False, True, True))
            .translate((-0.008, rib_y, 0.0))
        )
        filter_shape = filter_shape.union(rib)

    return filter_shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_purifier")

    shell_white = model.material("shell_white", rgba=(0.92, 0.94, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.74, 0.77, 0.79, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.46, 0.50, 0.53, 1.0))
    switch_black = model.material("switch_black", rgba=(0.09, 0.10, 0.11, 1.0))

    body = model.part("body")
    shell_overlap = 0.0005
    shell_span_x = BODY_DEPTH - BODY_WALL + shell_overlap
    shell_center_x = (BODY_WALL / 2.0) - (shell_overlap / 2.0)
    shell_inner_width = BODY_WIDTH - (2.0 * BODY_WALL) + shell_overlap
    opening_center_z = FRONT_OPENING_BOTTOM + (FRONT_OPENING_HEIGHT / 2.0)
    front_frame_width = ((BODY_WIDTH - FRONT_OPENING_WIDTH) / 2.0) + shell_overlap

    body.visual(
        Box((BODY_WALL, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=((-BODY_DEPTH / 2.0) + (BODY_WALL / 2.0), 0.0, BODY_HEIGHT / 2.0)),
        material=shell_white,
        name="back_wall",
    )
    for index, y_center in enumerate(
        (
            (BODY_WIDTH / 2.0) - (BODY_WALL / 2.0),
            (-BODY_WIDTH / 2.0) + (BODY_WALL / 2.0),
        )
    ):
        body.visual(
            Box((shell_span_x, BODY_WALL, BODY_HEIGHT)),
            origin=Origin(xyz=(shell_center_x, y_center, BODY_HEIGHT / 2.0)),
            material=shell_white,
            name=f"side_wall_{index}",
        )
    body.visual(
        Box((shell_span_x, shell_inner_width, BODY_WALL)),
        origin=Origin(xyz=(shell_center_x, 0.0, BODY_WALL / 2.0)),
        material=shell_white,
        name="base_wall",
    )
    body.visual(
        Box((shell_span_x, shell_inner_width, BODY_WALL)),
        origin=Origin(xyz=(shell_center_x, 0.0, BODY_HEIGHT - (BODY_WALL / 2.0))),
        material=shell_white,
        name="top_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_WIDTH, FRONT_OPENING_BOTTOM + shell_overlap)),
        origin=Origin(
            xyz=(
                (BODY_DEPTH / 2.0) - (BODY_WALL / 2.0),
                0.0,
                (FRONT_OPENING_BOTTOM + shell_overlap) / 2.0,
            )
        ),
        material=shell_white,
        name="lower_rail",
    )
    body.visual(
        Box(
            (
                BODY_WALL,
                BODY_WIDTH,
                BODY_HEIGHT - (FRONT_OPENING_BOTTOM + FRONT_OPENING_HEIGHT) + shell_overlap,
            )
        ),
        origin=Origin(
            xyz=(
                (BODY_DEPTH / 2.0) - (BODY_WALL / 2.0),
                0.0,
                FRONT_OPENING_BOTTOM
                + FRONT_OPENING_HEIGHT
                + (BODY_HEIGHT - (FRONT_OPENING_BOTTOM + FRONT_OPENING_HEIGHT) + shell_overlap) / 2.0,
            )
        ),
        material=shell_white,
        name="upper_front",
    )
    for index, y_center in enumerate(
        (
            (FRONT_OPENING_WIDTH / 2.0) + (front_frame_width / 2.0),
            (-FRONT_OPENING_WIDTH / 2.0) - (front_frame_width / 2.0),
        )
    ):
        body.visual(
            Box((BODY_WALL, front_frame_width, FRONT_OPENING_HEIGHT + shell_overlap)),
            origin=Origin(
                xyz=(
                    (BODY_DEPTH / 2.0) - (BODY_WALL / 2.0),
                    y_center,
                    opening_center_z,
                )
            ),
            material=shell_white,
            name=f"front_stile_{index}",
        )
    for index, rail_y in enumerate((-0.095, 0.095)):
        body.visual(
            Box((0.110, 0.022, 0.012)),
            origin=Origin(xyz=(-0.012, rail_y, opening_center_z)),
            material=trim_gray,
            name=f"guide_rail_{index}",
        )
    for index, vent_y in enumerate((-0.056, -0.034, -0.012, 0.012, 0.034, 0.056)):
        body.visual(
            Box((0.090, 0.008, 0.002)),
            origin=Origin(xyz=(-0.008, vent_y, BODY_HEIGHT - 0.001)),
            material=trim_gray,
            name=f"top_vent_{index}",
        )

    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_cadquery(_build_front_panel_shape(), "front_panel"),
        material=trim_gray,
        name="panel",
    )
    front_panel.visual(
        Box((0.001, 0.006, PANEL_HEIGHT - 0.010)),
        origin=Origin(xyz=(0.0005, 0.0, 0.0)),
        material=trim_gray,
        name="hinge_leaf",
    )
    for index, z_center in enumerate((-0.057, 0.0, 0.057)):
        front_panel.visual(
            Cylinder(radius=0.0045, length=0.036 if index != 1 else 0.048),
            origin=Origin(xyz=(0.0045, 0.0, z_center)),
            material=trim_gray,
            name=f"knuckle_{index}",
        )

    filter_part = model.part("filter")
    filter_part.visual(
        mesh_from_cadquery(_build_filter_shape(), "filter_cartridge"),
        material=filter_gray,
        name="cartridge",
    )

    rocker_switch = model.part("rocker_switch")
    rocker_switch.visual(
        Box((0.012, SWITCH_WIDTH, 0.005)),
        origin=Origin(xyz=(0.006, 0.0, 0.001)),
        material=switch_black,
        name="switch_front",
    )
    rocker_switch.visual(
        Box((0.012, SWITCH_WIDTH, 0.005)),
        origin=Origin(xyz=(-0.006, 0.0, 0.001)),
        material=switch_black,
        name="switch_rear",
    )
    rocker_switch.visual(
        Box((0.006, SWITCH_WIDTH, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=switch_black,
        name="switch_bridge",
    )
    rocker_switch.visual(
        Box((0.006, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, 0.0105, -0.002)),
        material=switch_black,
        name="switch_tab_0",
    )
    rocker_switch.visual(
        Box((0.006, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.0105, -0.002)),
        material=switch_black,
        name="switch_tab_1",
    )

    front_x = BODY_DEPTH / 2.0
    opening_center_z = FRONT_OPENING_BOTTOM + (FRONT_OPENING_HEIGHT / 2.0)
    filter_origin_x = front_x - PANEL_THICKNESS - FILTER_REST_RECESS

    model.articulation(
        "body_to_front_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_panel,
        origin=Origin(
            xyz=(
                front_x,
                (-FRONT_OPENING_WIDTH / 2.0) + PANEL_GAP,
                opening_center_z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_filter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_part,
        origin=Origin(xyz=(filter_origin_x, 0.0, opening_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.15,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_rocker_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker_switch,
        origin=Origin(xyz=(SWITCH_X, SWITCH_Y, BODY_HEIGHT + 0.005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=-0.30,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    front_panel = object_model.get_part("front_panel")
    filter_part = object_model.get_part("filter")
    rocker_switch = object_model.get_part("rocker_switch")

    panel_hinge = object_model.get_articulation("body_to_front_panel")
    filter_slide = object_model.get_articulation("body_to_filter")
    switch_hinge = object_model.get_articulation("body_to_rocker_switch")

    body_aabb = ctx.part_world_aabb(body)
    panel_aabb = ctx.part_element_world_aabb(front_panel, elem="panel")
    ctx.check(
        "front panel is flush with purifier face when closed",
        body_aabb is not None
        and panel_aabb is not None
        and abs(panel_aabb[0][0] - body_aabb[1][0]) <= 0.0015
        and panel_aabb[1][0] <= body_aabb[1][0] + 0.014,
        details=f"body_aabb={body_aabb}, panel_aabb={panel_aabb}",
    )

    filter_aabb = ctx.part_world_aabb(filter_part)
    ctx.check(
        "filter cartridge fits the lower service opening footprint",
        filter_aabb is not None
        and (filter_aabb[1][1] - filter_aabb[0][1]) <= FRONT_OPENING_WIDTH - 0.010
        and (filter_aabb[1][2] - filter_aabb[0][2]) <= FRONT_OPENING_HEIGHT - 0.008,
        details=f"filter_aabb={filter_aabb}",
    )

    with ctx.pose({panel_hinge: math.radians(72.0)}):
        opened_panel_aabb = ctx.part_world_aabb(front_panel)
        ctx.check(
            "lower panel swings outward from the purifier",
            body_aabb is not None
            and opened_panel_aabb is not None
            and opened_panel_aabb[1][0] >= body_aabb[1][0] + 0.050,
            details=f"body_aabb={body_aabb}, opened_panel_aabb={opened_panel_aabb}",
        )

    with ctx.pose({panel_hinge: math.radians(72.0), filter_slide: FILTER_TRAVEL}):
        extended_filter_aabb = ctx.part_world_aabb(filter_part)
        ctx.check(
            "filter extends out through the lower opening",
            body_aabb is not None
            and extended_filter_aabb is not None
            and extended_filter_aabb[1][0] >= body_aabb[1][0] + 0.060,
            details=f"body_aabb={body_aabb}, extended_filter_aabb={extended_filter_aabb}",
        )
        ctx.expect_overlap(
            filter_part,
            body,
            axes="x",
            min_overlap=0.035,
            name="filter remains retained inside the purifier at full extension",
        )

    with ctx.pose({switch_hinge: 0.28}):
        front_elem_aabb = ctx.part_element_world_aabb(rocker_switch, elem="switch_front")
        rear_elem_aabb = ctx.part_element_world_aabb(rocker_switch, elem="switch_rear")
        front_center_z = None if front_elem_aabb is None else (front_elem_aabb[0][2] + front_elem_aabb[1][2]) / 2.0
        rear_center_z = None if rear_elem_aabb is None else (rear_elem_aabb[0][2] + rear_elem_aabb[1][2]) / 2.0
        ctx.check(
            "top rocker switch tips about its local hinge",
            front_center_z is not None
            and rear_center_z is not None
            and front_center_z >= rear_center_z + 0.0025,
            details=f"front_center_z={front_center_z}, rear_center_z={rear_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
