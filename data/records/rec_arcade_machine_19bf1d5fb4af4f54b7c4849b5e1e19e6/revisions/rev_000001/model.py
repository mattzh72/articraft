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


CABINET_LENGTH = 0.92
CABINET_WIDTH = 0.68
CABINET_HEIGHT = 0.72

BOTTOM_LENGTH = 0.74
BOTTOM_WIDTH = 0.52
WALL_THICKNESS = 0.018
BASE_THICKNESS = 0.028
TOP_THICKNESS = 0.032

LID_GAP = 0.003
LID_LENGTH = 0.742
LID_WIDTH = 0.524
LID_THICKNESS = 0.018
TOP_OPEN_LENGTH = LID_LENGTH + 2.0 * LID_GAP
TOP_OPEN_WIDTH = LID_WIDTH + 2.0 * LID_GAP
LID_HINGE_X = -TOP_OPEN_LENGTH / 2.0

GLASS_OPEN_LENGTH = 0.44
GLASS_OPEN_WIDTH = 0.28
GLASS_THICKNESS = 0.006
GLASS_CENTER_FROM_HINGE = 0.375

WELL_OUTER_LENGTH = TOP_OPEN_LENGTH
WELL_OUTER_WIDTH = TOP_OPEN_WIDTH
WELL_DEPTH = 0.118
WELL_WALL = 0.022
WELL_FLOOR_THICKNESS = 0.014
WELL_TOP_Z = CABINET_HEIGHT - TOP_THICKNESS
SCREEN_LENGTH = 0.42
SCREEN_WIDTH = 0.26
SCREEN_THICKNESS = 0.012

FASCIA_DEPTH = 0.06
FASCIA_WIDTH = 0.34
FASCIA_HEIGHT = 0.36
FASCIA_CENTER_Z = 0.25
FASCIA_PROUD = 0.008
FASCIA_OUTER_X = CABINET_LENGTH / 2.0 + FASCIA_PROUD

COIN_DOOR_WIDTH = 0.22
COIN_DOOR_HEIGHT = 0.28
COIN_DOOR_DEPTH = 0.018
COIN_DOOR_GAP = 0.003
COIN_HINGE_X = FASCIA_OUTER_X - 0.001
COIN_HINGE_Y = -(COIN_DOOR_WIDTH / 2.0 + COIN_DOOR_GAP)


def _cabinet_shape():
    outer_shell = (
        cq.Workplane("XY")
        .box(CABINET_LENGTH, CABINET_WIDTH, CABINET_HEIGHT)
        .translate((0.0, 0.0, CABINET_HEIGHT / 2.0))
    )

    front_fascia = (
        cq.Workplane("XY")
        .box(FASCIA_DEPTH, FASCIA_WIDTH, FASCIA_HEIGHT)
        .translate(
            (
                CABINET_LENGTH / 2.0 - FASCIA_DEPTH / 2.0 + FASCIA_PROUD,
                0.0,
                FASCIA_CENTER_Z,
            )
        )
    )

    cabinet = outer_shell.union(front_fascia)

    inner_height = CABINET_HEIGHT - TOP_THICKNESS - BASE_THICKNESS
    inner_shell = (
        cq.Workplane("XY")
        .box(
            CABINET_LENGTH - 2.0 * WALL_THICKNESS,
            CABINET_WIDTH - 2.0 * WALL_THICKNESS,
            inner_height,
        )
        .translate((0.0, 0.0, BASE_THICKNESS + inner_height / 2.0))
    )
    cabinet = cabinet.cut(inner_shell)

    top_cut_depth = TOP_THICKNESS + 0.08
    cabinet = cabinet.cut(
        cq.Workplane("XY")
        .box(TOP_OPEN_LENGTH, TOP_OPEN_WIDTH, top_cut_depth)
        .translate((0.0, 0.0, CABINET_HEIGHT - top_cut_depth / 2.0 + 0.001))
    )

    door_pocket_depth = COIN_DOOR_DEPTH + 0.026
    cabinet = cabinet.cut(
        cq.Workplane("XY")
        .box(
            door_pocket_depth,
            COIN_DOOR_WIDTH + 2.0 * COIN_DOOR_GAP,
            COIN_DOOR_HEIGHT + 2.0 * COIN_DOOR_GAP,
        )
        .translate(
            (
                FASCIA_OUTER_X - door_pocket_depth / 2.0 + 0.001,
                0.0,
                FASCIA_CENTER_Z,
            )
        )
    )

    return cabinet


def _lid_shape():
    lid_panel = (
        cq.Workplane("XY")
        .box(LID_LENGTH, LID_WIDTH, LID_THICKNESS)
        .translate((LID_LENGTH / 2.0, 0.0, -LID_THICKNESS / 2.0))
    )
    glass_cut = (
        cq.Workplane("XY")
        .box(GLASS_OPEN_LENGTH, GLASS_OPEN_WIDTH, LID_THICKNESS + 0.01)
        .translate((GLASS_CENTER_FROM_HINGE, 0.0, -LID_THICKNESS / 2.0))
    )
    return lid_panel.cut(glass_cut)


def _aabb_max(aabb, axis: int) -> float | None:
    if aabb is None:
        return None
    return float(aabb[1][axis])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cocktail_table_arcade")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.10, 0.13, 0.18, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.18, 0.20, 0.24, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.68, 0.80, 0.88, 0.35))
    screen_finish = model.material("screen_finish", rgba=(0.04, 0.05, 0.06, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.58, 0.60, 0.64, 1.0))
    shadow_finish = model.material("shadow_finish", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet = model.part("cabinet")
    side_wall_height = CABINET_HEIGHT - TOP_THICKNESS - BASE_THICKNESS
    top_rail_depth = (CABINET_LENGTH - TOP_OPEN_LENGTH) / 2.0
    top_rail_width = (CABINET_WIDTH - TOP_OPEN_WIDTH) / 2.0
    front_open_width = COIN_DOOR_WIDTH + 2.0 * COIN_DOOR_GAP
    front_open_height = COIN_DOOR_HEIGHT + 2.0 * COIN_DOOR_GAP
    front_open_bottom = FASCIA_CENTER_Z - front_open_height / 2.0
    front_open_top = FASCIA_CENTER_Z + front_open_height / 2.0
    front_side_width = (CABINET_WIDTH - front_open_width) / 2.0
    front_upper_height = CABINET_HEIGHT - TOP_THICKNESS - front_open_top

    cabinet.visual(
        Box((CABINET_LENGTH, CABINET_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=cabinet_finish,
        name="base_plate",
    )
    cabinet.visual(
        Box((BOTTOM_LENGTH, BOTTOM_WIDTH, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=trim_finish,
        name="base_plinth",
    )
    cabinet.visual(
        Box((CABINET_LENGTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_WIDTH / 2.0 - WALL_THICKNESS / 2.0),
                BASE_THICKNESS + side_wall_height / 2.0,
            )
        ),
        material=cabinet_finish,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((CABINET_LENGTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_WIDTH / 2.0 - WALL_THICKNESS / 2.0,
                BASE_THICKNESS + side_wall_height / 2.0,
            )
        ),
        material=cabinet_finish,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH, side_wall_height)),
        origin=Origin(
            xyz=(
                -CABINET_LENGTH / 2.0 + WALL_THICKNESS / 2.0,
                0.0,
                BASE_THICKNESS + side_wall_height / 2.0,
            )
        ),
        material=cabinet_finish,
        name="rear_panel",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH, front_open_bottom)),
        origin=Origin(
            xyz=(
                CABINET_LENGTH / 2.0 - WALL_THICKNESS / 2.0,
                0.0,
                front_open_bottom / 2.0,
            )
        ),
        material=cabinet_finish,
        name="front_lower",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH, front_upper_height)),
        origin=Origin(
            xyz=(
                CABINET_LENGTH / 2.0 - WALL_THICKNESS / 2.0,
                0.0,
                front_open_top + front_upper_height / 2.0,
            )
        ),
        material=cabinet_finish,
        name="front_upper",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, front_side_width, front_open_height)),
        origin=Origin(
            xyz=(
                CABINET_LENGTH / 2.0 - WALL_THICKNESS / 2.0,
                -(front_open_width / 2.0 + front_side_width / 2.0),
                FASCIA_CENTER_Z,
            )
        ),
        material=cabinet_finish,
        name="front_side_0",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, front_side_width, front_open_height)),
        origin=Origin(
            xyz=(
                CABINET_LENGTH / 2.0 - WALL_THICKNESS / 2.0,
                front_open_width / 2.0 + front_side_width / 2.0,
                FASCIA_CENTER_Z,
            )
        ),
        material=cabinet_finish,
        name="front_side_1",
    )
    cabinet.visual(
        Box((top_rail_depth, TOP_OPEN_WIDTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                -CABINET_LENGTH / 2.0 + top_rail_depth / 2.0,
                0.0,
                CABINET_HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=cabinet_finish,
        name="top_rear",
    )
    cabinet.visual(
        Box((top_rail_depth, TOP_OPEN_WIDTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                CABINET_LENGTH / 2.0 - top_rail_depth / 2.0,
                0.0,
                CABINET_HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=cabinet_finish,
        name="top_front",
    )
    cabinet.visual(
        Box((TOP_OPEN_LENGTH, top_rail_width, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_WIDTH / 2.0 - top_rail_width / 2.0),
                CABINET_HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=cabinet_finish,
        name="top_side_0",
    )
    cabinet.visual(
        Box((TOP_OPEN_LENGTH, top_rail_width, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_WIDTH / 2.0 - top_rail_width / 2.0,
                CABINET_HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=cabinet_finish,
        name="top_side_1",
    )
    cabinet.visual(
        Box((FASCIA_DEPTH, front_side_width / 2.0, FASCIA_HEIGHT)),
        origin=Origin(
            xyz=(
                FASCIA_OUTER_X - FASCIA_DEPTH / 2.0,
                -(front_open_width / 2.0 + front_side_width / 4.0),
                FASCIA_CENTER_Z,
            )
        ),
        material=trim_finish,
        name="fascia_side_0",
    )
    cabinet.visual(
        Box((FASCIA_DEPTH, front_side_width / 2.0, FASCIA_HEIGHT)),
        origin=Origin(
            xyz=(
                FASCIA_OUTER_X - FASCIA_DEPTH / 2.0,
                front_open_width / 2.0 + front_side_width / 4.0,
                FASCIA_CENTER_Z,
            )
        ),
        material=trim_finish,
        name="fascia_side_1",
    )
    cabinet.visual(
        Box((FASCIA_DEPTH, front_open_width, (FASCIA_HEIGHT - front_open_height) / 2.0)),
        origin=Origin(
            xyz=(
                FASCIA_OUTER_X - FASCIA_DEPTH / 2.0,
                0.0,
                FASCIA_CENTER_Z - front_open_height / 2.0 - (FASCIA_HEIGHT - front_open_height) / 4.0,
            )
        ),
        material=trim_finish,
        name="fascia_lower",
    )
    cabinet.visual(
        Box((FASCIA_DEPTH, front_open_width, (FASCIA_HEIGHT - front_open_height) / 2.0)),
        origin=Origin(
            xyz=(
                FASCIA_OUTER_X - FASCIA_DEPTH / 2.0,
                0.0,
                FASCIA_CENTER_Z + front_open_height / 2.0 + (FASCIA_HEIGHT - front_open_height) / 4.0,
            )
        ),
        material=trim_finish,
        name="fascia_upper",
    )

    screen_well = model.part("screen_well")
    wall_height = WELL_DEPTH - WELL_FLOOR_THICKNESS
    screen_well.visual(
        Box((WELL_OUTER_LENGTH, WELL_OUTER_WIDTH, WELL_FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -WELL_DEPTH + WELL_FLOOR_THICKNESS / 2.0)),
        material=trim_finish,
        name="well_floor",
    )
    screen_well.visual(
        Box((WELL_WALL, WELL_OUTER_WIDTH, wall_height)),
        origin=Origin(
            xyz=(
                -(WELL_OUTER_LENGTH / 2.0 - WELL_WALL / 2.0),
                0.0,
                -wall_height / 2.0,
            )
        ),
        material=trim_finish,
        name="well_rear_wall",
    )
    screen_well.visual(
        Box((WELL_WALL, WELL_OUTER_WIDTH, wall_height)),
        origin=Origin(
            xyz=(
                WELL_OUTER_LENGTH / 2.0 - WELL_WALL / 2.0,
                0.0,
                -wall_height / 2.0,
            )
        ),
        material=trim_finish,
        name="well_front_wall",
    )
    screen_well.visual(
        Box((WELL_OUTER_LENGTH - 2.0 * WELL_WALL, WELL_WALL, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(WELL_OUTER_WIDTH / 2.0 - WELL_WALL / 2.0),
                -wall_height / 2.0,
            )
        ),
        material=trim_finish,
        name="well_side_0",
    )
    screen_well.visual(
        Box((WELL_OUTER_LENGTH - 2.0 * WELL_WALL, WELL_WALL, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                WELL_OUTER_WIDTH / 2.0 - WELL_WALL / 2.0,
                -wall_height / 2.0,
            )
        ),
        material=trim_finish,
        name="well_side_1",
    )
    screen_well.visual(
        Box((SCREEN_LENGTH, SCREEN_WIDTH, SCREEN_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -WELL_DEPTH + WELL_FLOOR_THICKNESS + SCREEN_THICKNESS / 2.0,
            )
        ),
        material=screen_finish,
        name="screen",
    )

    service_lid = model.part("service_lid")
    service_lid.visual(
        mesh_from_cadquery(_lid_shape(), "service_lid_frame_v2"),
        material=cabinet_finish,
        name="lid_frame",
    )
    service_lid.visual(
        Box((GLASS_OPEN_LENGTH, GLASS_OPEN_WIDTH, GLASS_THICKNESS)),
        origin=Origin(
            xyz=(
                GLASS_CENTER_FROM_HINGE,
                0.0,
                -GLASS_THICKNESS / 2.0,
            )
        ),
        material=glass_finish,
        name="glass",
    )

    coin_door = model.part("coin_door")
    coin_door.visual(
        Box((COIN_DOOR_DEPTH, COIN_DOOR_WIDTH, COIN_DOOR_HEIGHT)),
        origin=Origin(xyz=(-COIN_DOOR_DEPTH / 2.0, COIN_DOOR_WIDTH / 2.0, 0.0)),
        material=metal_finish,
        name="door_panel",
    )
    coin_door.visual(
        Box((0.004, 0.11, 0.016)),
        origin=Origin(xyz=(0.002, COIN_DOOR_WIDTH * 0.55, COIN_DOOR_HEIGHT * 0.22)),
        material=shadow_finish,
        name="coin_slot",
    )
    coin_door.visual(
        Box((0.01, 0.05, 0.07)),
        origin=Origin(xyz=(0.005, COIN_DOOR_WIDTH * 0.60, -COIN_DOOR_HEIGHT * 0.18)),
        material=trim_finish,
        name="coin_return",
    )

    model.articulation(
        "cabinet_to_screen_well",
        ArticulationType.FIXED,
        parent=cabinet,
        child=screen_well,
        origin=Origin(xyz=(0.0, 0.0, WELL_TOP_Z)),
    )
    model.articulation(
        "cabinet_to_service_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, CABINET_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "cabinet_to_coin_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_door,
        origin=Origin(xyz=(COIN_HINGE_X, COIN_HINGE_Y, FASCIA_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    service_lid = object_model.get_part("service_lid")
    screen_well = object_model.get_part("screen_well")
    coin_door = object_model.get_part("coin_door")
    lid_hinge = object_model.get_articulation("cabinet_to_service_lid")
    coin_hinge = object_model.get_articulation("cabinet_to_coin_door")

    with ctx.pose({lid_hinge: 0.0, coin_hinge: 0.0}):
        ctx.expect_overlap(
            service_lid,
            screen_well,
            axes="xy",
            elem_a="glass",
            elem_b="screen",
            min_overlap=0.20,
            name="glass window covers the horizontal screen",
        )
        ctx.expect_gap(
            service_lid,
            screen_well,
            axis="z",
            positive_elem="glass",
            min_gap=0.02,
            max_gap=0.05,
            name="screen well stays recessed below the glass",
        )

        lid_closed = ctx.part_element_world_aabb(service_lid, elem="lid_frame")
        coin_closed = ctx.part_element_world_aabb(coin_door, elem="door_panel")

        lid_top = _aabb_max(lid_closed, 2)
        ctx.check(
            "service lid closes flush with the cabinet top",
            lid_top is not None and abs(lid_top - CABINET_HEIGHT) <= 0.004,
            details=f"lid_top={lid_top}, cabinet_top={CABINET_HEIGHT}",
        )

        coin_face = _aabb_max(coin_closed, 0)
        ctx.check(
            "coin door sits nearly flush with the cabinet end",
            coin_face is not None and abs(coin_face - COIN_HINGE_X) <= 0.004,
            details=f"coin_face={coin_face}, expected={COIN_HINGE_X}",
        )

    lid_closed = ctx.part_element_world_aabb(service_lid, elem="lid_frame")
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            lid_open = ctx.part_element_world_aabb(service_lid, elem="lid_frame")
        ctx.check(
            "service lid lifts upward from the rear hinge",
            lid_closed is not None
            and lid_open is not None
            and lid_open[1][2] > lid_closed[1][2] + 0.18,
            details=f"closed={lid_closed}, open={lid_open}",
        )

    coin_closed = ctx.part_element_world_aabb(coin_door, elem="door_panel")
    if coin_hinge.motion_limits is not None and coin_hinge.motion_limits.upper is not None:
        with ctx.pose({coin_hinge: coin_hinge.motion_limits.upper}):
            coin_open = ctx.part_element_world_aabb(coin_door, elem="door_panel")
        ctx.check(
            "coin door swings outward on its side hinge",
            coin_closed is not None
            and coin_open is not None
            and coin_open[1][0] > coin_closed[1][0] + 0.10,
            details=f"closed={coin_closed}, open={coin_open}",
        )

    return ctx.report()


object_model = build_object_model()
