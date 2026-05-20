from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.56
BODY_HEIGHT = 0.78
BODY_DEPTH = 0.145
BODY_CORNER_RADIUS = 0.026
BODY_BACK_WALL = 0.010

CAVITY_SIDE_MARGIN = 0.028
CAVITY_TOP_MARGIN = 0.045
CAVITY_BOTTOM_MARGIN = 0.050
CAVITY_WIDTH = BODY_WIDTH - (2.0 * CAVITY_SIDE_MARGIN)
CAVITY_HEIGHT = BODY_HEIGHT - CAVITY_TOP_MARGIN - CAVITY_BOTTOM_MARGIN
CAVITY_DEPTH = BODY_DEPTH - BODY_BACK_WALL
CAVITY_CENTER_Y = BODY_BACK_WALL / 2.0
CAVITY_FLOOR_Z = (-BODY_HEIGHT / 2.0) + CAVITY_BOTTOM_MARGIN

DOOR_WIDTH = BODY_WIDTH - 0.020
DOOR_HEIGHT = BODY_HEIGHT - 0.032
DOOR_THICKNESS = 0.018
DOOR_GAP = 0.0
DOOR_OPEN_LIMIT = 1.55

FILTER_WIDTH = CAVITY_WIDTH - 0.016
FILTER_HEIGHT = CAVITY_HEIGHT - 0.022
FILTER_DEPTH = 0.110
FILTER_FRAME_THICKNESS = 0.008
FILTER_TRAVEL = 0.072
FILTER_FRONT_Y = 0.052
FILTER_CENTER_Y = FILTER_FRONT_Y - (FILTER_DEPTH / 2.0)
FILTER_CENTER_Z = CAVITY_FLOOR_Z + 0.004 + (FILTER_HEIGHT / 2.0)

RAIL_WIDTH = 0.012
RAIL_HEIGHT = 0.005
RAIL_LENGTH = FILTER_DEPTH - 0.006
RAIL_CENTER_Y = FILTER_CENTER_Y
RAIL_CENTER_Z = CAVITY_FLOOR_Z + (RAIL_HEIGHT / 2.0) - 0.0005
RAIL_OFFSET_X = (FILTER_WIDTH / 2.0) - 0.034


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_body_shape() -> cq.Workplane:
    body = _box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT), (0.0, 0.0, 0.0))
    body = body.edges("|Z").fillet(BODY_CORNER_RADIUS)

    cavity = _box(
        (CAVITY_WIDTH, CAVITY_DEPTH, CAVITY_HEIGHT),
        (0.0, CAVITY_CENTER_Y, (CAVITY_BOTTOM_MARGIN - CAVITY_TOP_MARGIN) / 2.0),
    )
    body = body.cut(cavity)

    for rail_sign in (-1.0, 1.0):
        rail = _box(
            (RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT),
            (rail_sign * RAIL_OFFSET_X, RAIL_CENTER_Y, RAIL_CENTER_Z),
        )
        body = body.union(rail)

    return body


def _build_door_shape() -> cq.Workplane:
    door = _box(
        (DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT),
        (DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, 0.0),
    )
    door = door.edges("|Z").fillet(0.0065)

    perimeter_recess = _box(
        (DOOR_WIDTH - 0.060, 0.0022, DOOR_HEIGHT - 0.080),
        (DOOR_WIDTH / 2.0, DOOR_THICKNESS - 0.0011, 0.0),
    )
    door = door.cut(perimeter_recess)

    slot_count = 7
    slot_span = DOOR_WIDTH - 0.160
    slot_pitch = slot_span / (slot_count - 1)
    slot_start_x = (DOOR_WIDTH / 2.0) - (slot_span / 2.0)
    for index in range(slot_count):
        slot = _box(
            (0.022, DOOR_THICKNESS + 0.004, 0.275),
            (slot_start_x + (index * slot_pitch), DOOR_THICKNESS / 2.0, -0.105),
        )
        door = door.cut(slot)

    pull_lip = _box(
        (0.016, 0.010, 0.120),
        (DOOR_WIDTH - 0.010, DOOR_THICKNESS + 0.002, -0.010),
    )
    door = door.union(pull_lip)

    indicator = _box(
        (0.160, 0.004, 0.018),
        (DOOR_WIDTH * 0.54, DOOR_THICKNESS - 0.001, DOOR_HEIGHT / 2.0 - 0.080),
    )
    door = door.union(indicator)

    return door


def _build_filter_shape() -> cq.Workplane:
    cartridge = _box(
        (FILTER_WIDTH, FILTER_DEPTH, FILTER_HEIGHT),
        (0.0, -FILTER_DEPTH / 2.0, 0.0),
    )

    inner_void = _box(
        (
            FILTER_WIDTH - (2.0 * FILTER_FRAME_THICKNESS),
            FILTER_DEPTH - (2.0 * FILTER_FRAME_THICKNESS),
            FILTER_HEIGHT - (2.0 * FILTER_FRAME_THICKNESS),
        ),
        (0.0, -FILTER_DEPTH / 2.0, 0.0),
    )
    cartridge = cartridge.cut(inner_void)

    media_width = FILTER_WIDTH - (2.0 * FILTER_FRAME_THICKNESS) + 0.002
    media_height = FILTER_HEIGHT - (2.0 * FILTER_FRAME_THICKNESS) + 0.002
    media_depth = FILTER_DEPTH - (2.0 * FILTER_FRAME_THICKNESS) + 0.002
    media = _box(
        (media_width, media_depth, media_height),
        (0.0, -FILTER_DEPTH / 2.0, 0.0),
    )
    cartridge = cartridge.union(media)

    rib_count = 11
    rib_span = media_width - 0.030
    rib_pitch = rib_span / (rib_count - 1)
    rib_start_x = -(rib_span / 2.0)
    rib_center_y = -FILTER_FRAME_THICKNESS - 0.004
    for index in range(rib_count):
        rib = _box(
            (0.010, 0.012, media_height),
            (rib_start_x + (index * rib_pitch), rib_center_y, 0.0),
        )
        cartridge = cartridge.union(rib)

    groove_width = RAIL_WIDTH + 0.004
    groove_height = RAIL_HEIGHT + 0.002
    groove_length = FILTER_DEPTH - 0.016
    groove_center_z = (-FILTER_HEIGHT / 2.0) + (groove_height / 2.0)
    for rail_sign in (-1.0, 1.0):
        groove = _box(
            (groove_width, groove_length, groove_height),
            (rail_sign * RAIL_OFFSET_X, -FILTER_DEPTH / 2.0, groove_center_z),
        )
        cartridge = cartridge.cut(groove)

    pull_tab = _box(
        (0.072, 0.018, 0.022),
        (0.0, 0.004, (FILTER_HEIGHT / 2.0) - 0.036),
    )
    cartridge = cartridge.union(pull_tab)

    return cartridge


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_panel_air_purifier")

    model.material("body_finish", rgba=(0.87, 0.89, 0.90, 1.0))
    model.material("door_finish", rgba=(0.95, 0.96, 0.96, 1.0))
    model.material("filter_finish", rgba=(0.29, 0.33, 0.36, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_BACK_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, (-BODY_DEPTH / 2.0) + (BODY_BACK_WALL / 2.0), 0.0)),
        material="body_finish",
        name="back_panel",
    )
    body.visual(
        Box((CAVITY_SIDE_MARGIN, BODY_DEPTH - BODY_BACK_WALL + 0.001, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (-BODY_WIDTH / 2.0) + (CAVITY_SIDE_MARGIN / 2.0),
                (BODY_BACK_WALL / 2.0) - 0.0005,
                0.0,
            )
        ),
        material="body_finish",
        name="side_wall_0",
    )
    body.visual(
        Box((CAVITY_SIDE_MARGIN, BODY_DEPTH - BODY_BACK_WALL + 0.001, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH / 2.0) - (CAVITY_SIDE_MARGIN / 2.0),
                (BODY_BACK_WALL / 2.0) - 0.0005,
                0.0,
            )
        ),
        material="body_finish",
        name="side_wall_1",
    )
    body.visual(
        Box((CAVITY_WIDTH + 0.002, BODY_DEPTH - BODY_BACK_WALL + 0.001, CAVITY_TOP_MARGIN)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_BACK_WALL / 2.0) - 0.0005,
                (BODY_HEIGHT / 2.0) - (CAVITY_TOP_MARGIN / 2.0),
            )
        ),
        material="body_finish",
        name="top_wall",
    )
    body.visual(
        Box((CAVITY_WIDTH + 0.002, BODY_DEPTH - BODY_BACK_WALL + 0.001, CAVITY_BOTTOM_MARGIN)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_BACK_WALL / 2.0) - 0.0005,
                (-BODY_HEIGHT / 2.0) + (CAVITY_BOTTOM_MARGIN / 2.0),
            )
        ),
        material="body_finish",
        name="bottom_wall",
    )
    body.visual(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(-RAIL_OFFSET_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material="body_finish",
        name="rail_0",
    )
    body.visual(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(RAIL_OFFSET_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material="body_finish",
        name="rail_1",
    )
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_shape(), "purifier_door"),
        material="door_finish",
        name="panel",
    )

    filter_cartridge = model.part("filter")
    filter_cartridge.visual(
        mesh_from_cadquery(_build_filter_shape(), "purifier_filter"),
        material="filter_finish",
        name="cartridge",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, (BODY_DEPTH / 2.0) + DOOR_GAP, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=DOOR_OPEN_LIMIT, effort=12.0, velocity=1.2),
    )
    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, FILTER_FRONT_Y, FILTER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FILTER_TRAVEL, effort=35.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    filter_cartridge = object_model.get_part("filter")
    door_hinge = object_model.get_articulation("door_hinge")
    filter_slide = object_model.get_articulation("filter_slide")

    with ctx.pose({door_hinge: 0.0, filter_slide: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            max_gap=0.004,
            max_penetration=0.0,
            name="door closes just proud of the housing",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.48,
            name="door covers the purifier front opening",
        )
        ctx.expect_within(
            filter_cartridge,
            body,
            axes="xz",
            margin=0.0,
            name="stored filter stays inside the purifier envelope",
        )
        ctx.expect_gap(
            door,
            filter_cartridge,
            axis="y",
            min_gap=0.006,
            name="closed door clears the parked filter",
        )
        closed_door_aabb = ctx.part_world_aabb(door)
        collapsed_filter_position = ctx.part_world_position(filter_cartridge)

    with ctx.pose({door_hinge: DOOR_OPEN_LIMIT}):
        open_door_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings outward from the housing",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_hinge: DOOR_OPEN_LIMIT, filter_slide: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_cartridge,
            body,
            axes="xz",
            margin=0.02,
            name="extended filter remains laterally guided by the housing",
        )
        ctx.expect_overlap(
            filter_cartridge,
            body,
            axes="y",
            min_overlap=0.05,
            name="extended filter retains insertion in the guide rails",
        )
        extended_filter_position = ctx.part_world_position(filter_cartridge)

    ctx.check(
        "filter slides forward for service",
        collapsed_filter_position is not None
        and extended_filter_position is not None
        and extended_filter_position[1] > collapsed_filter_position[1] + 0.05,
        details=f"collapsed={collapsed_filter_position}, extended={extended_filter_position}",
    )

    return ctx.report()


object_model = build_object_model()
