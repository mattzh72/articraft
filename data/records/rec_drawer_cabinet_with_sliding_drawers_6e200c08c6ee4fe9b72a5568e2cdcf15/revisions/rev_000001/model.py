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


CABINET_WIDTH = 0.96
CABINET_DEPTH = 0.42
PLINTH_HEIGHT = 0.06
BODY_HEIGHT = 0.70
BODY_THICKNESS = 0.018
CENTER_DIVIDER_THICKNESS = 0.018
BACK_THICKNESS = 0.012
SHELF_THICKNESS = 0.014

BAY_WIDTH = (CABINET_WIDTH - 2.0 * BODY_THICKNESS - CENTER_DIVIDER_THICKNESS) / 2.0
OPENING_HEIGHT = (BODY_HEIGHT - 2.0 * BODY_THICKNESS - 3.0 * SHELF_THICKNESS) / 4.0

DRAWER_FRONT_THICKNESS = 0.018
DRAWER_FRONT_WIDTH = BAY_WIDTH - 0.006
DRAWER_FRONT_HEIGHT = OPENING_HEIGHT - 0.006
DRAWER_BOX_HEIGHT = OPENING_HEIGHT - 0.030
DRAWER_BOX_DEPTH = 0.31
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BACK_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.010
DRAWER_PROUD = 0.002
DRAWER_TRAVEL = 0.16

RUNNER_WIDTH = 0.012
RUNNER_HEIGHT = 0.010
RUNNER_LENGTH = 0.18
RUNNER_FRONT_INSET = 0.045

DRAWER_BOX_WIDTH = BAY_WIDTH - 2.0 * RUNNER_WIDTH

LABEL_WIDTH = 0.12
LABEL_HEIGHT = 0.034
LABEL_THICKNESS = 0.006
KNOB_RADIUS = 0.011
KNOB_LENGTH = 0.022


def _column_center_x(col: int) -> float:
    direction = -1.0 if col == 0 else 1.0
    return direction * (CENTER_DIVIDER_THICKNESS / 2.0 + BAY_WIDTH / 2.0)


def _opening_bottom_z(row: int) -> float:
    inner_bottom = PLINTH_HEIGHT + BODY_THICKNESS
    return inner_bottom + row * (OPENING_HEIGHT + SHELF_THICKNESS)


def _row_center_z(row: int) -> float:
    return _opening_bottom_z(row) + OPENING_HEIGHT / 2.0


def _runner_centers_x(col: int) -> tuple[float, float]:
    if col == 0:
        return (
            -CABINET_WIDTH / 2.0 + BODY_THICKNESS + RUNNER_WIDTH / 2.0,
            -CENTER_DIVIDER_THICKNESS / 2.0 - RUNNER_WIDTH / 2.0,
        )
    return (
        CENTER_DIVIDER_THICKNESS / 2.0 + RUNNER_WIDTH / 2.0,
        CABINET_WIDTH / 2.0 - BODY_THICKNESS - RUNNER_WIDTH / 2.0,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="merchant_drawer_cabinet")

    body_wood = model.material("body_wood", rgba=(0.46, 0.31, 0.19, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.56, 0.39, 0.24, 1.0))
    plinth_wood = model.material("plinth_wood", rgba=(0.32, 0.22, 0.14, 1.0))
    hardware = model.material("hardware", rgba=(0.58, 0.54, 0.47, 1.0))

    carcass = model.part("carcass")

    plinth_width = CABINET_WIDTH - 0.08
    plinth_depth = CABINET_DEPTH - 0.06
    carcass.visual(
        Box((plinth_width, plinth_depth, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material=plinth_wood,
        name="plinth",
    )

    carcass.visual(
        Box((BODY_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_WIDTH / 2.0 + BODY_THICKNESS / 2.0, 0.0, PLINTH_HEIGHT + BODY_HEIGHT / 2.0)
        ),
        material=body_wood,
        name="side_left",
    )
    carcass.visual(
        Box((BODY_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(CABINET_WIDTH / 2.0 - BODY_THICKNESS / 2.0, 0.0, PLINTH_HEIGHT + BODY_HEIGHT / 2.0)
        ),
        material=body_wood,
        name="side_right",
    )
    carcass.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, BODY_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + BODY_THICKNESS / 2.0)),
        material=body_wood,
        name="bottom",
    )
    carcass.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, BODY_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + BODY_HEIGHT - BODY_THICKNESS / 2.0)),
        material=body_wood,
        name="top",
    )
    carcass.visual(
        Box((CENTER_DIVIDER_THICKNESS, CABINET_DEPTH - BACK_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BACK_THICKNESS / 2.0, PLINTH_HEIGHT + BODY_HEIGHT / 2.0)),
        material=body_wood,
        name="center_divider",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * BODY_THICKNESS, BACK_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH / 2.0 - BACK_THICKNESS / 2.0, PLINTH_HEIGHT + BODY_HEIGHT / 2.0)
        ),
        material=body_wood,
        name="back_panel",
    )

    for shelf_index in range(3):
        shelf_z = _opening_bottom_z(shelf_index) + OPENING_HEIGHT + SHELF_THICKNESS / 2.0
        carcass.visual(
            Box((CABINET_WIDTH - 2.0 * BODY_THICKNESS, CABINET_DEPTH - BACK_THICKNESS, SHELF_THICKNESS)),
            origin=Origin(xyz=(0.0, -BACK_THICKNESS / 2.0, shelf_z)),
            material=body_wood,
            name=f"shelf_{shelf_index}",
        )

    runner_center_y = -CABINET_DEPTH / 2.0 + RUNNER_FRONT_INSET + RUNNER_LENGTH / 2.0
    runner_z_offset = -DRAWER_BOX_HEIGHT / 2.0 + 0.028

    for row in range(4):
        row_center_z = _row_center_z(row)
        runner_center_z = row_center_z + runner_z_offset
        for col in range(2):
            runner_left_x, runner_right_x = _runner_centers_x(col)
            carcass.visual(
                Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(xyz=(runner_left_x, runner_center_y, runner_center_z)),
                material=body_wood,
                name=f"runner_{row}_{col}_left",
            )
            carcass.visual(
                Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(xyz=(runner_right_x, runner_center_y, runner_center_z)),
                material=body_wood,
                name=f"runner_{row}_{col}_right",
            )

            drawer = model.part(f"drawer_{row}_{col}")
            drawer.visual(
                Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
                material=drawer_wood,
                name="front_panel",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -DRAWER_BOX_WIDTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0,
                        DRAWER_FRONT_THICKNESS / 2.0 + DRAWER_BOX_DEPTH / 2.0,
                        0.0,
                    )
                ),
                material=drawer_wood,
                name="box_side_left",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
                origin=Origin(
                    xyz=(
                        DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
                        DRAWER_FRONT_THICKNESS / 2.0 + DRAWER_BOX_DEPTH / 2.0,
                        0.0,
                    )
                ),
                material=drawer_wood,
                name="box_side_right",
            )
            drawer.visual(
                Box((DRAWER_BOX_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_BOX_HEIGHT)),
                origin=Origin(
                    xyz=(
                        0.0,
                        DRAWER_FRONT_THICKNESS / 2.0 + DRAWER_BOX_DEPTH - DRAWER_BACK_THICKNESS / 2.0,
                        0.0,
                    )
                ),
                material=drawer_wood,
                name="box_back",
            )
            drawer.visual(
                Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS)),
                origin=Origin(
                    xyz=(
                        0.0,
                        DRAWER_FRONT_THICKNESS / 2.0 + DRAWER_BOX_DEPTH / 2.0,
                        -DRAWER_BOX_HEIGHT / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0,
                    )
                ),
                material=drawer_wood,
                name="box_bottom",
            )
            drawer.visual(
                Box((LABEL_WIDTH, LABEL_THICKNESS, LABEL_HEIGHT)),
                origin=Origin(xyz=(0.0, -DRAWER_FRONT_THICKNESS / 2.0 - LABEL_THICKNESS / 2.0, 0.024)),
                material=hardware,
                name="label_frame",
            )
            drawer.visual(
                Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
                origin=Origin(
                    xyz=(0.0, -DRAWER_FRONT_THICKNESS / 2.0 - KNOB_LENGTH / 2.0, -0.018),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=hardware,
                name="pull_knob",
            )

            drawer_center_x = _column_center_x(col)
            drawer_center_y = -CABINET_DEPTH / 2.0 - DRAWER_PROUD + DRAWER_FRONT_THICKNESS / 2.0

            model.articulation(
                f"carcass_to_drawer_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=carcass,
                child=drawer,
                origin=Origin(xyz=(drawer_center_x, drawer_center_y, row_center_z)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=120.0,
                    velocity=0.25,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    for row in range(4):
        for col in range(2):
            drawer_name = f"drawer_{row}_{col}"
            joint_name = f"carcass_to_drawer_{row}_{col}"
            drawer = object_model.get_part(drawer_name)
            joint = object_model.get_articulation(joint_name)

            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="box_side_left",
                elem_b=f"runner_{row}_{col}_left",
                min_overlap=0.17,
                name=f"{drawer_name} left runner engaged at rest",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="box_side_right",
                elem_b=f"runner_{row}_{col}_right",
                min_overlap=0.17,
                name=f"{drawer_name} right runner engaged at rest",
            )

            rest_position = ctx.part_world_position(drawer)
            upper = joint.motion_limits.upper if joint.motion_limits is not None else DRAWER_TRAVEL

            with ctx.pose({joint: upper}):
                ctx.expect_overlap(
                    drawer,
                    carcass,
                    axes="y",
                    elem_a="box_side_left",
                    elem_b=f"runner_{row}_{col}_left",
                    min_overlap=0.11,
                    name=f"{drawer_name} left runner retains insertion",
                )
                ctx.expect_overlap(
                    drawer,
                    carcass,
                    axes="y",
                    elem_a="box_side_right",
                    elem_b=f"runner_{row}_{col}_right",
                    min_overlap=0.11,
                    name=f"{drawer_name} right runner retains insertion",
                )
                extended_position = ctx.part_world_position(drawer)

            ctx.check(
                f"{drawer_name} extends forward",
                rest_position is not None
                and extended_position is not None
                and extended_position[1] < rest_position[1] - 0.12,
                details=f"rest={rest_position}, extended={extended_position}",
            )

    return ctx.report()


object_model = build_object_model()
