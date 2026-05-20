from __future__ import annotations

import math

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


COLUMNS = 5
ROWS = 6

CABINET_WIDTH = 1.18
CABINET_DEPTH = 0.42
CABINET_HEIGHT = 1.26
SIDE_THICKNESS = 0.025
TOP_THICKNESS = 0.025
BOTTOM_THICKNESS = 0.025
BACK_THICKNESS = 0.012
DIVIDER_THICKNESS = 0.018
SHELF_THICKNESS = 0.016
PLINTH_HEIGHT = 0.09
TOE_RECESS = 0.045

INNER_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
GRID_BOTTOM = PLINTH_HEIGHT + BOTTOM_THICKNESS
GRID_TOP = CABINET_HEIGHT - TOP_THICKNESS
GRID_HEIGHT = GRID_TOP - GRID_BOTTOM
OPENING_WIDTH = (INNER_WIDTH - (COLUMNS - 1) * DIVIDER_THICKNESS) / COLUMNS
OPENING_HEIGHT = (GRID_HEIGHT - (ROWS - 1) * SHELF_THICKNESS) / ROWS
INTERIOR_DEPTH = CABINET_DEPTH - BACK_THICKNESS
INTERIOR_CENTER_Y = BACK_THICKNESS / 2.0

DRAWER_FRONT_THICKNESS = 0.020
DRAWER_FRONT_WIDTH = OPENING_WIDTH - 0.006
DRAWER_FRONT_HEIGHT = OPENING_HEIGHT - 0.006
DRAWER_BODY_WIDTH = OPENING_WIDTH - 0.032
DRAWER_BODY_HEIGHT = OPENING_HEIGHT - 0.028
DRAWER_BODY_DEPTH = 0.34
DRAWER_SIDE_THICKNESS = 0.008
DRAWER_BACK_THICKNESS = 0.008
DRAWER_BOTTOM_THICKNESS = 0.008

GUIDE_THICKNESS = 0.008
GUIDE_HEIGHT = 0.012
GUIDE_LENGTH = 0.18
GUIDE_EMBED = 0.0005
GUIDE_CENTER_Y = 0.085

RUNNER_THICKNESS = 0.005
RUNNER_HEIGHT = 0.008
RUNNER_LENGTH = 0.17
RUNNER_EMBED = 0.0005
RUNNER_CENTER_REL_Y = GUIDE_CENTER_Y - (CABINET_DEPTH / 2.0 - DRAWER_FRONT_THICKNESS - DRAWER_BODY_DEPTH / 2.0)
RUNNER_CENTER_X = OPENING_WIDTH / 2.0 - GUIDE_THICKNESS + GUIDE_EMBED - RUNNER_THICKNESS / 2.0
RUNNER_BRACKET_EMBED = 0.0004
RUNNER_BRACKET_GAP = RUNNER_CENTER_X - RUNNER_THICKNESS / 2.0 - DRAWER_BODY_WIDTH / 2.0
RUNNER_BRACKET_WIDTH = RUNNER_BRACKET_GAP + 2.0 * RUNNER_BRACKET_EMBED
RUNNER_BRACKET_CENTER_X = DRAWER_BODY_WIDTH / 2.0 + RUNNER_BRACKET_GAP / 2.0
RUNNER_BRACKET_HEIGHT = 0.014

DRAWER_TRAVEL = 0.14

LABEL_WIDTH = DRAWER_FRONT_WIDTH * 0.30
LABEL_HEIGHT = 0.026
LABEL_THICKNESS = 0.003
KNOB_RADIUS = 0.008
KNOB_LENGTH = 0.026

REP_ROW = 2
REP_COL = 2


def _drawer_name(row: int, col: int) -> str:
    return f"drawer_{row}_{col}"


def _slide_name(row: int, col: int) -> str:
    return f"slide_{row}_{col}"


def _guide_name(side: str, row: int, col: int) -> str:
    return f"guide_{side}_{row}_{col}"


def _column_centers() -> list[float]:
    start = -INNER_WIDTH / 2.0 + OPENING_WIDTH / 2.0
    pitch = OPENING_WIDTH + DIVIDER_THICKNESS
    return [start + idx * pitch for idx in range(COLUMNS)]


def _row_centers() -> list[float]:
    start = GRID_BOTTOM + OPENING_HEIGHT / 2.0
    pitch = OPENING_HEIGHT + SHELF_THICKNESS
    return [start + idx * pitch for idx in range(ROWS)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apothecary_drawer_cabinet")

    case_wood = model.material("case_wood", rgba=(0.35, 0.23, 0.14, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.42, 0.28, 0.18, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.68, 0.56, 0.34, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.32, 0.33, 0.35, 1.0))
    shadow_paint = model.material("shadow_paint", rgba=(0.16, 0.11, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=case_wood,
        name="side_left",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=case_wood,
        name="side_right",
    )
    cabinet.visual(
        Box((INNER_WIDTH, CABINET_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - TOP_THICKNESS / 2.0)),
        material=case_wood,
        name="top",
    )
    cabinet.visual(
        Box((INNER_WIDTH, CABINET_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + BOTTOM_THICKNESS / 2.0)),
        material=case_wood,
        name="bottom",
    )
    cabinet.visual(
        Box((INNER_WIDTH, BACK_THICKNESS, CABINET_HEIGHT - PLINTH_HEIGHT - TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + (CABINET_HEIGHT - PLINTH_HEIGHT - TOP_THICKNESS) / 2.0,
            )
        ),
        material=shadow_paint,
        name="back",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.020, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0 - TOE_RECESS - 0.010, PLINTH_HEIGHT / 2.0)),
        material=case_wood,
        name="plinth",
    )

    divider_height = GRID_HEIGHT
    divider_z = GRID_BOTTOM + divider_height / 2.0
    for idx in range(COLUMNS - 1):
        divider_x = -INNER_WIDTH / 2.0 + OPENING_WIDTH + DIVIDER_THICKNESS / 2.0 + idx * (
            OPENING_WIDTH + DIVIDER_THICKNESS
        )
        cabinet.visual(
            Box((DIVIDER_THICKNESS, INTERIOR_DEPTH, divider_height)),
            origin=Origin(xyz=(divider_x, INTERIOR_CENTER_Y, divider_z)),
            material=case_wood,
            name=f"divider_{idx}",
        )

    shelf_z_start = GRID_BOTTOM + OPENING_HEIGHT + SHELF_THICKNESS / 2.0
    shelf_pitch = OPENING_HEIGHT + SHELF_THICKNESS
    for idx in range(ROWS - 1):
        cabinet.visual(
            Box((INNER_WIDTH, INTERIOR_DEPTH, SHELF_THICKNESS)),
            origin=Origin(xyz=(0.0, INTERIOR_CENTER_Y, shelf_z_start + idx * shelf_pitch)),
            material=case_wood,
            name=f"shelf_{idx}",
        )

    column_centers = _column_centers()
    row_centers = _row_centers()
    left_offset = -OPENING_WIDTH / 2.0 + GUIDE_THICKNESS / 2.0 - GUIDE_EMBED
    right_offset = OPENING_WIDTH / 2.0 - GUIDE_THICKNESS / 2.0 + GUIDE_EMBED

    for row, z in enumerate(row_centers):
        for col, x in enumerate(column_centers):
            cabinet.visual(
                Box((GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT)),
                origin=Origin(xyz=(x + left_offset, GUIDE_CENTER_Y, z)),
                material=slide_steel,
                name=_guide_name("left", row, col),
            )
            cabinet.visual(
                Box((GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT)),
                origin=Origin(xyz=(x + right_offset, GUIDE_CENTER_Y, z)),
                material=slide_steel,
                name=_guide_name("right", row, col),
            )

            drawer = model.part(_drawer_name(row, col))
            drawer.visual(
                Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
                origin=Origin(
                    xyz=(
                        0.0,
                        DRAWER_BODY_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS / 2.0,
                        0.0,
                    )
                ),
                material=drawer_wood,
                name="front",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT)),
                origin=Origin(
                    xyz=(-DRAWER_BODY_WIDTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0, 0.0, 0.0)
                ),
                material=drawer_wood,
                name="tray_left",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT)),
                origin=Origin(
                    xyz=(DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0, 0.0, 0.0)
                ),
                material=drawer_wood,
                name="tray_right",
            )
            drawer.visual(
                Box((DRAWER_BODY_WIDTH, DRAWER_BODY_DEPTH, DRAWER_BOTTOM_THICKNESS)),
                origin=Origin(
                    xyz=(0.0, 0.0, -DRAWER_BODY_HEIGHT / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0)
                ),
                material=drawer_wood,
                name="tray_bottom",
            )
            drawer.visual(
                Box((DRAWER_BODY_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_BODY_HEIGHT)),
                origin=Origin(
                    xyz=(0.0, -DRAWER_BODY_DEPTH / 2.0 + DRAWER_BACK_THICKNESS / 2.0, 0.0)
                ),
                material=drawer_wood,
                name="tray_back",
            )
            drawer.visual(
                Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -RUNNER_CENTER_X,
                        RUNNER_CENTER_REL_Y,
                        0.0,
                    )
                ),
                material=slide_steel,
                name="runner_left",
            )
            drawer.visual(
                Box((RUNNER_BRACKET_WIDTH, RUNNER_LENGTH, RUNNER_BRACKET_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -RUNNER_BRACKET_CENTER_X,
                        RUNNER_CENTER_REL_Y,
                        0.0,
                    )
                ),
                material=drawer_wood,
                name="runner_mount_left",
            )
            drawer.visual(
                Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(
                    xyz=(
                        RUNNER_CENTER_X,
                        RUNNER_CENTER_REL_Y,
                        0.0,
                    )
                ),
                material=slide_steel,
                name="runner_right",
            )
            drawer.visual(
                Box((RUNNER_BRACKET_WIDTH, RUNNER_LENGTH, RUNNER_BRACKET_HEIGHT)),
                origin=Origin(
                    xyz=(
                        RUNNER_BRACKET_CENTER_X,
                        RUNNER_CENTER_REL_Y,
                        0.0,
                    )
                ),
                material=drawer_wood,
                name="runner_mount_right",
            )
            drawer.visual(
                Box((LABEL_WIDTH, LABEL_THICKNESS, LABEL_HEIGHT)),
                origin=Origin(
                    xyz=(
                        0.0,
                        DRAWER_BODY_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS + LABEL_THICKNESS / 2.0 - 0.0005,
                        0.022,
                    )
                ),
                material=aged_brass,
                name="label_holder",
            )
            drawer.visual(
                Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
                origin=Origin(
                    xyz=(
                        0.0,
                        DRAWER_BODY_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS + KNOB_LENGTH / 2.0 - 0.001,
                        -0.026,
                    ),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=aged_brass,
                name="knob",
            )

            closed_y = CABINET_DEPTH / 2.0 - DRAWER_FRONT_THICKNESS - DRAWER_BODY_DEPTH / 2.0
            model.articulation(
                _slide_name(row, col),
                ArticulationType.PRISMATIC,
                parent=cabinet,
                child=drawer,
                origin=Origin(xyz=(x, closed_y, z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=60.0,
                    velocity=0.25,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part(_drawer_name(REP_ROW, REP_COL))
    slide = object_model.get_articulation(_slide_name(REP_ROW, REP_COL))

    with ctx.pose({slide: 0.0}):
        front_aabb = ctx.part_element_world_aabb(drawer, elem="front")
        cabinet_aabb = ctx.part_world_aabb(cabinet)
        flush_ok = (
            front_aabb is not None
            and cabinet_aabb is not None
            and abs(front_aabb[1][1] - cabinet_aabb[1][1]) <= 0.002
        )
        ctx.check(
            "center drawer front stays flush with cabinet face",
            flush_ok,
            details=f"front={front_aabb}, cabinet={cabinet_aabb}",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="xz",
            elem_a="front",
            min_overlap=0.16,
            name="center drawer front remains within cabinet face footprint",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="runner_left",
            elem_b=_guide_name("left", REP_ROW, REP_COL),
            min_overlap=0.16,
            name="center drawer left runner fully engages guide when closed",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="runner_right",
            elem_b=_guide_name("right", REP_ROW, REP_COL),
            min_overlap=0.16,
            name="center drawer right runner fully engages guide when closed",
        )
        closed_pos = ctx.part_world_position(drawer)

    with ctx.pose({slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="runner_left",
            elem_b=_guide_name("left", REP_ROW, REP_COL),
            min_overlap=0.03,
            name="center drawer left runner stays retained when extended",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="runner_right",
            elem_b=_guide_name("right", REP_ROW, REP_COL),
            min_overlap=0.03,
            name="center drawer right runner stays retained when extended",
        )
        open_pos = ctx.part_world_position(drawer)

    ctx.check(
        "center drawer extends outward from the cabinet",
        closed_pos is not None
        and open_pos is not None
        and open_pos[1] > closed_pos[1] + DRAWER_TRAVEL - 0.005,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
