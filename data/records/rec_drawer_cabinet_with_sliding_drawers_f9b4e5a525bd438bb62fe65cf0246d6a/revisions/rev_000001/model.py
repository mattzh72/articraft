from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


ROWS = 12
COLS = 4

CABINET_WIDTH = 1.08
CABINET_DEPTH = 0.46
PLINTH_HEIGHT = 0.06
BODY_HEIGHT = 1.50
SIDE_THICKNESS = 0.022
TOP_BOTTOM_THICKNESS = 0.024
DIVIDER_THICKNESS = 0.012
SHELF_THICKNESS = 0.012
BACK_THICKNESS = 0.010

CLEAR_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
CLEAR_HEIGHT = BODY_HEIGHT - 2.0 * TOP_BOTTOM_THICKNESS
INTERIOR_DEPTH = CABINET_DEPTH - BACK_THICKNESS
CELL_WIDTH = (CLEAR_WIDTH - (COLS - 1) * DIVIDER_THICKNESS) / COLS
CELL_HEIGHT = (CLEAR_HEIGHT - (ROWS - 1) * SHELF_THICKNESS) / ROWS

DRAWER_FRONT_WIDTH = 0.244
DRAWER_FRONT_HEIGHT = 0.104
DRAWER_FRONT_THICKNESS = 0.014
DRAWER_BODY_WIDTH = 0.242
DRAWER_BODY_DEPTH = 0.40
DRAWER_SIDE_THICKNESS = 0.008
DRAWER_BOTTOM_THICKNESS = 0.008
DRAWER_BACK_THICKNESS = 0.008
DRAWER_SIDE_HEIGHT = 0.086

RUNNER_WIDTH = 0.018
RUNNER_HEIGHT = 0.008
RUNNER_LENGTH = 0.38
RUNNER_Y_CENTER = 0.03

LABEL_FRAME_WIDTH = 0.092
LABEL_FRAME_HEIGHT = 0.028
LABEL_FRAME_THICKNESS = 0.004
PULL_KNOB_RADIUS = 0.013
PULL_KNOB_LENGTH = 0.020

TOP_CAP_WIDTH = 1.10
TOP_CAP_DEPTH = 0.48
TOP_CAP_THICKNESS = 0.018

DRAWER_TRAVEL = 0.18


def drawer_name(row: int, col: int) -> str:
    return f"drawer_r{row + 1}_c{col + 1}"


def drawer_joint_name(row: int, col: int) -> str:
    return f"{drawer_name(row, col)}_slide"


def rail_name(row: int, col: int, side: str) -> str:
    return f"rail_r{row + 1}_c{col + 1}_{side}"


def cell_center_x(col: int) -> float:
    clear_min_x = -CABINET_WIDTH / 2.0 + SIDE_THICKNESS
    cell_min_x = clear_min_x + col * (CELL_WIDTH + DIVIDER_THICKNESS)
    return cell_min_x + CELL_WIDTH / 2.0


def cell_center_z(row: int) -> float:
    clear_min_z = PLINTH_HEIGHT + TOP_BOTTOM_THICKNESS
    cell_min_z = clear_min_z + row * (CELL_HEIGHT + SHELF_THICKNESS)
    return cell_min_z + CELL_HEIGHT / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_card_catalog")

    case_wood = model.material("case_wood", rgba=(0.31, 0.20, 0.12, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.43, 0.28, 0.17, 1.0))
    runner_wood = model.material("runner_wood", rgba=(0.50, 0.36, 0.23, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.62, 0.30, 1.0))
    shadow_wood = model.material("shadow_wood", rgba=(0.26, 0.17, 0.10, 1.0))

    carcass = model.part("carcass")

    carcass.visual(
        Box((1.00, 0.40, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.03, PLINTH_HEIGHT / 2.0)),
        material=shadow_wood,
        name="plinth",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + BODY_HEIGHT / 2.0,
            )
        ),
        material=case_wood,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + BODY_HEIGHT / 2.0,
            )
        ),
        material=case_wood,
        name="right_side",
    )
    carcass.visual(
        Box((CLEAR_WIDTH, INTERIOR_DEPTH, TOP_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + TOP_BOTTOM_THICKNESS / 2.0,
            )
        ),
        material=case_wood,
        name="bottom_panel",
    )
    carcass.visual(
        Box((CLEAR_WIDTH, INTERIOR_DEPTH, TOP_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + BODY_HEIGHT - TOP_BOTTOM_THICKNESS / 2.0,
            )
        ),
        material=case_wood,
        name="top_panel",
    )
    carcass.visual(
        Box((CLEAR_WIDTH, BACK_THICKNESS, CLEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + BODY_HEIGHT / 2.0,
            )
        ),
        material=case_wood,
        name="back_panel",
    )
    carcass.visual(
        Box((TOP_CAP_WIDTH, TOP_CAP_DEPTH, TOP_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PLINTH_HEIGHT + BODY_HEIGHT + TOP_CAP_THICKNESS / 2.0,
            )
        ),
        material=case_wood,
        name="top_cap",
    )

    for divider_idx in range(COLS - 1):
        divider_x = (
            -CABINET_WIDTH / 2.0
            + SIDE_THICKNESS
            + (divider_idx + 1) * CELL_WIDTH
            + divider_idx * DIVIDER_THICKNESS
            + DIVIDER_THICKNESS / 2.0
        )
        carcass.visual(
            Box((DIVIDER_THICKNESS, INTERIOR_DEPTH, CLEAR_HEIGHT)),
            origin=Origin(
                xyz=(
                    divider_x,
                    BACK_THICKNESS / 2.0,
                    PLINTH_HEIGHT + BODY_HEIGHT / 2.0,
                )
            ),
            material=case_wood,
            name=f"divider_{divider_idx + 1}",
        )

    for shelf_idx in range(ROWS - 1):
        shelf_z = (
            PLINTH_HEIGHT
            + TOP_BOTTOM_THICKNESS
            + (shelf_idx + 1) * CELL_HEIGHT
            + shelf_idx * SHELF_THICKNESS
            + SHELF_THICKNESS / 2.0
        )
        carcass.visual(
            Box((CLEAR_WIDTH, INTERIOR_DEPTH, SHELF_THICKNESS)),
            origin=Origin(xyz=(0.0, BACK_THICKNESS / 2.0, shelf_z)),
            material=case_wood,
            name=f"shelf_{shelf_idx + 1}",
        )

    runner_z = RUNNER_HEIGHT / 2.0
    runner_x_offset = CELL_WIDTH / 2.0 - RUNNER_WIDTH / 2.0
    for row in range(ROWS):
        row_z_min = cell_center_z(row) - CELL_HEIGHT / 2.0
        for col in range(COLS):
            cx = cell_center_x(col)
            carcass.visual(
                Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(
                    xyz=(
                        cx - runner_x_offset,
                        RUNNER_Y_CENTER,
                        row_z_min + runner_z,
                    )
                ),
                material=runner_wood,
                name=rail_name(row, col, "left"),
            )
            carcass.visual(
                Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(
                    xyz=(
                        cx + runner_x_offset,
                        RUNNER_Y_CENTER,
                        row_z_min + runner_z,
                    )
                ),
                material=runner_wood,
                name=rail_name(row, col, "right"),
            )

    drawer_bottom_center_z = (
        -CELL_HEIGHT / 2.0 + RUNNER_HEIGHT + DRAWER_BOTTOM_THICKNESS / 2.0
    )
    drawer_side_center_z = -CELL_HEIGHT / 2.0 + RUNNER_HEIGHT + DRAWER_SIDE_HEIGHT / 2.0

    for row in range(ROWS):
        for col in range(COLS):
            drawer = model.part(drawer_name(row, col))

            drawer.visual(
                Box(
                    (
                        DRAWER_FRONT_WIDTH,
                        DRAWER_FRONT_THICKNESS,
                        DRAWER_FRONT_HEIGHT,
                    )
                ),
                origin=Origin(
                    xyz=(0.0, DRAWER_FRONT_THICKNESS / 2.0, 0.0),
                ),
                material=drawer_wood,
                name="front_panel",
            )
            drawer.visual(
                Box((DRAWER_BODY_WIDTH, DRAWER_BODY_DEPTH, DRAWER_BOTTOM_THICKNESS)),
                origin=Origin(xyz=(0.0, -DRAWER_BODY_DEPTH / 2.0, drawer_bottom_center_z)),
                material=drawer_wood,
                name="drawer_bottom",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_SIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -DRAWER_BODY_WIDTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0,
                        -DRAWER_BODY_DEPTH / 2.0,
                        drawer_side_center_z,
                    )
                ),
                material=drawer_wood,
                name="left_side",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_SIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
                        -DRAWER_BODY_DEPTH / 2.0,
                        drawer_side_center_z,
                    )
                ),
                material=drawer_wood,
                name="right_side",
            )
            drawer.visual(
                Box((DRAWER_BODY_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_SIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        0.0,
                        -DRAWER_BODY_DEPTH + DRAWER_BACK_THICKNESS / 2.0,
                        drawer_side_center_z,
                    )
                ),
                material=drawer_wood,
                name="drawer_back",
            )
            drawer.visual(
                Box((LABEL_FRAME_WIDTH, LABEL_FRAME_THICKNESS, LABEL_FRAME_HEIGHT)),
                origin=Origin(
                    xyz=(
                        0.0,
                        DRAWER_FRONT_THICKNESS + LABEL_FRAME_THICKNESS / 2.0,
                        0.020,
                    )
                ),
                material=brass,
                name="label_frame",
            )
            drawer.visual(
                Cylinder(radius=PULL_KNOB_RADIUS, length=PULL_KNOB_LENGTH),
                origin=Origin(
                    xyz=(0.0, DRAWER_FRONT_THICKNESS + PULL_KNOB_LENGTH / 2.0, -0.014),
                    rpy=(-pi / 2.0, 0.0, 0.0),
                ),
                material=brass,
                name="pull_knob",
            )

            model.articulation(
                drawer_joint_name(row, col),
                ArticulationType.PRISMATIC,
                parent=carcass,
                child=drawer,
                origin=Origin(
                    xyz=(
                        cell_center_x(col),
                        CABINET_DEPTH / 2.0,
                        cell_center_z(row),
                    )
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=80.0,
                    velocity=0.25,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    representative_drawers = [
        (0, 0),
        (5, 2),
        (11, 3),
    ]

    ctx.check(
        "cabinet has forty-eight drawers",
        len(object_model.parts) == 1 + ROWS * COLS,
        details=f"part_count={len(object_model.parts)}",
    )
    ctx.check(
        "cabinet has forty-eight drawer slides",
        len(object_model.articulations) == ROWS * COLS,
        details=f"articulation_count={len(object_model.articulations)}",
    )

    for row, col in representative_drawers:
        drawer = object_model.get_part(drawer_name(row, col))
        joint = object_model.get_articulation(drawer_joint_name(row, col))
        left_rail = rail_name(row, col, "left")
        right_rail = rail_name(row, col, "right")

        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} slides outward along +Y",
            joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == DRAWER_TRAVEL,
            details=f"axis={joint.axis}, limits={limits}",
        )

        with ctx.pose({joint: 0.0}):
            ctx.expect_gap(
                drawer,
                carcass,
                axis="z",
                positive_elem="drawer_bottom",
                negative_elem=left_rail,
                max_gap=0.001,
                max_penetration=1e-6,
                name=f"{drawer.name} rests on left runner",
            )
            ctx.expect_gap(
                drawer,
                carcass,
                axis="z",
                positive_elem="drawer_bottom",
                negative_elem=right_rail,
                max_gap=0.001,
                max_penetration=1e-6,
                name=f"{drawer.name} rests on right runner",
            )
            ctx.expect_gap(
                drawer,
                carcass,
                axis="y",
                positive_elem="front_panel",
                negative_elem="left_side",
                max_gap=0.01,
                max_penetration=0.0,
                name=f"{drawer.name} front sits flush near carcass face",
            )
            ctx.expect_within(
                drawer,
                carcass,
                axes="xz",
                inner_elem="drawer_bottom",
                margin=0.002,
                name=f"{drawer.name} stays centered in its bay when closed",
            )

        closed_y = ctx.part_world_position(drawer)
        with ctx.pose({joint: DRAWER_TRAVEL}):
            ctx.expect_within(
                drawer,
                carcass,
                axes="xz",
                inner_elem="drawer_bottom",
                margin=0.002,
                name=f"{drawer.name} stays on axis when extended",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="drawer_bottom",
                min_overlap=0.20,
                name=f"{drawer.name} keeps retained insertion at full travel",
            )
            open_y = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer.name} translates outward when opened",
            closed_y is not None
            and open_y is not None
            and open_y[1] > closed_y[1] + 0.15,
            details=f"closed={closed_y}, open={open_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
