from __future__ import annotations

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


CABINET_WIDTH = 1.50
CABINET_DEPTH = 0.60
PLINTH_HEIGHT = 0.08
CARCASS_HEIGHT = 1.22
CABINET_HEIGHT = PLINTH_HEIGHT + CARCASS_HEIGHT
WALL_THICKNESS = 0.024
DIVIDER_THICKNESS = 0.016
ROWS = 6
COLS = 4

OPENING_WIDTH = (
    CABINET_WIDTH - 2.0 * WALL_THICKNESS - (COLS - 1) * DIVIDER_THICKNESS
) / COLS
OPENING_HEIGHT = (
    CARCASS_HEIGHT - 2.0 * WALL_THICKNESS - (ROWS - 1) * DIVIDER_THICKNESS
) / ROWS

DRAWER_FRONT_THICKNESS = 0.020
DRAWER_FRONT_WIDTH = OPENING_WIDTH - 0.008
DRAWER_FRONT_HEIGHT = OPENING_HEIGHT - 0.008
DRAWER_BOX_WIDTH = OPENING_WIDTH - 0.016
DRAWER_BOX_HEIGHT = OPENING_HEIGHT - 0.016
DRAWER_BOX_DEPTH = 0.54
DRAWER_SIDE_THICKNESS = 0.008
DRAWER_BOTTOM_THICKNESS = 0.006
DRAWER_BACK_THICKNESS = 0.008
DRAWER_TRAVEL = 0.22

LABEL_WIDTH = 0.102
LABEL_HEIGHT = 0.042
LABEL_THICKNESS = 0.0012
LABEL_CENTER_Z = 0.034

PROTECTOR_WIDTH = 0.122
PROTECTOR_HEIGHT = 0.058
PROTECTOR_THICKNESS = 0.0015
PROTECTOR_FRAME = 0.006
PROTECTOR_FRONT_OFFSET = 0.0024
HINGE_AXIS_X = DRAWER_FRONT_THICKNESS + 0.0032
HINGE_BARREL_RADIUS = 0.0032
DRAWER_KNUCKLE_LENGTH = 0.030
PROTECTOR_KNUCKLE_LENGTH = 0.040
PROTECTOR_OPEN_LIMIT = 1.30

HANDLE_Z = -0.040
HANDLE_PLATE_THICKNESS = 0.003
HANDLE_PLATE_WIDTH = 0.074
HANDLE_PLATE_HEIGHT = 0.018
HANDLE_POST_RADIUS = 0.0035
HANDLE_POST_LENGTH = 0.010
HANDLE_BAR_RADIUS = 0.0045
HANDLE_BAR_LENGTH = 0.050
HANDLE_POST_Y = 0.020
HANDLE_BAR_X = DRAWER_FRONT_THICKNESS + HANDLE_POST_LENGTH
RUNNER_HEIGHT = 0.008
RUNNER_WIDTH = 0.018
RUNNER_DEPTH = DRAWER_BOX_DEPTH - 0.030


def _opening_center_y(col: int) -> float:
    inner_left = -CABINET_WIDTH / 2.0 + WALL_THICKNESS
    return inner_left + OPENING_WIDTH / 2.0 + col * (OPENING_WIDTH + DIVIDER_THICKNESS)


def _opening_center_z(row: int) -> float:
    inner_bottom = PLINTH_HEIGHT + WALL_THICKNESS
    return inner_bottom + OPENING_HEIGHT / 2.0 + row * (OPENING_HEIGHT + DIVIDER_THICKNESS)


def _divider_center_y(index: int) -> float:
    inner_left = -CABINET_WIDTH / 2.0 + WALL_THICKNESS
    return inner_left + index * OPENING_WIDTH + (index - 0.5) * DIVIDER_THICKNESS


def _shelf_center_z(index: int) -> float:
    inner_bottom = PLINTH_HEIGHT + WALL_THICKNESS
    return inner_bottom + index * OPENING_HEIGHT + (index - 0.5) * DIVIDER_THICKNESS


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_card_cabinet")

    model.material("case_wood", rgba=(0.43, 0.29, 0.17, 1.0))
    model.material("drawer_wood", rgba=(0.54, 0.37, 0.22, 1.0))
    model.material("interior_wood", rgba=(0.35, 0.24, 0.15, 1.0))
    model.material("hardware", rgba=(0.73, 0.61, 0.33, 1.0))
    model.material("card_stock", rgba=(0.94, 0.90, 0.79, 1.0))
    model.material("protector_glass", rgba=(0.82, 0.90, 0.95, 0.38))
    model.material("shadow", rgba=(0.21, 0.16, 0.11, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(CABINET_DEPTH / 2.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material="shadow",
        name="plinth",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, WALL_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_DEPTH / 2.0,
                -CABINET_WIDTH / 2.0 + WALL_THICKNESS / 2.0,
                PLINTH_HEIGHT + CARCASS_HEIGHT / 2.0,
            )
        ),
        material="case_wood",
        name="side_0",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, WALL_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_DEPTH / 2.0,
                CABINET_WIDTH / 2.0 - WALL_THICKNESS / 2.0,
                PLINTH_HEIGHT + CARCASS_HEIGHT / 2.0,
            )
        ),
        material="case_wood",
        name="side_1",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, WALL_THICKNESS)),
        origin=Origin(
            xyz=(CABINET_DEPTH / 2.0, 0.0, PLINTH_HEIGHT + WALL_THICKNESS / 2.0)
        ),
        material="case_wood",
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, WALL_THICKNESS)),
        origin=Origin(
            xyz=(CABINET_DEPTH / 2.0, 0.0, CABINET_HEIGHT - WALL_THICKNESS / 2.0)
        ),
        material="case_wood",
        name="top_panel",
    )

    inner_height = CARCASS_HEIGHT - 2.0 * WALL_THICKNESS
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH - 2.0 * WALL_THICKNESS, inner_height)),
        origin=Origin(
            xyz=(
                WALL_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + WALL_THICKNESS + inner_height / 2.0,
            )
        ),
        material="interior_wood",
        name="back_panel",
    )

    for divider_index in range(1, COLS):
        cabinet.visual(
            Box((CABINET_DEPTH - WALL_THICKNESS, DIVIDER_THICKNESS, inner_height)),
            origin=Origin(
                xyz=(
                    (CABINET_DEPTH + WALL_THICKNESS) / 2.0,
                    _divider_center_y(divider_index),
                    PLINTH_HEIGHT + WALL_THICKNESS + inner_height / 2.0,
                )
            ),
            material="interior_wood",
            name=f"divider_{divider_index}",
        )

    inner_width = CABINET_WIDTH - 2.0 * WALL_THICKNESS
    for shelf_index in range(1, ROWS):
        cabinet.visual(
            Box((CABINET_DEPTH - WALL_THICKNESS, inner_width, DIVIDER_THICKNESS)),
            origin=Origin(
                xyz=(
                    (CABINET_DEPTH + WALL_THICKNESS) / 2.0,
                    0.0,
                    _shelf_center_z(shelf_index),
                )
            ),
            material="interior_wood",
            name=f"shelf_{shelf_index}",
        )

    for row in range(ROWS):
        for col in range(COLS):
            drawer_name = f"drawer_{row}_{col}"
            protector_name = f"protector_{row}_{col}"
            y_center = _opening_center_y(col)
            z_center = _opening_center_z(row)
            opening_bottom = z_center - OPENING_HEIGHT / 2.0

            for runner_sign, runner_index in ((-1.0, 0), (1.0, 1)):
                cabinet.visual(
                    Box((RUNNER_DEPTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
                    origin=Origin(
                        xyz=(
                            WALL_THICKNESS + RUNNER_DEPTH / 2.0,
                            y_center
                            + runner_sign
                            * (DRAWER_BOX_WIDTH / 2.0 - RUNNER_WIDTH / 2.0),
                            opening_bottom + RUNNER_HEIGHT / 2.0,
                        )
                    ),
                    material="interior_wood",
                    name=f"runner_{row}_{col}_{runner_index}",
                )

            drawer = model.part(drawer_name)
            drawer.visual(
                Box((DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, DRAWER_FRONT_HEIGHT)),
                origin=Origin(xyz=(DRAWER_FRONT_THICKNESS / 2.0, 0.0, 0.0)),
                material="drawer_wood",
                name="drawer_front",
            )
            drawer.visual(
                Box((DRAWER_BOX_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BOX_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -DRAWER_BOX_DEPTH / 2.0,
                        -DRAWER_BOX_WIDTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0,
                        0.0,
                    )
                ),
                material="drawer_wood",
                name="side_0",
            )
            drawer.visual(
                Box((DRAWER_BOX_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BOX_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -DRAWER_BOX_DEPTH / 2.0,
                        DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
                        0.0,
                    )
                ),
                material="drawer_wood",
                name="side_1",
            )
            drawer.visual(
                Box((DRAWER_BOX_DEPTH, DRAWER_BOX_WIDTH, DRAWER_BOTTOM_THICKNESS)),
                origin=Origin(
                    xyz=(
                        -DRAWER_BOX_DEPTH / 2.0,
                        0.0,
                        -DRAWER_BOX_HEIGHT / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0,
                    )
                ),
                material="drawer_wood",
                name="drawer_bottom",
            )
            drawer.visual(
                Box((DRAWER_BACK_THICKNESS, DRAWER_BOX_WIDTH, DRAWER_BOX_HEIGHT)),
                origin=Origin(
                    xyz=(-DRAWER_BOX_DEPTH + DRAWER_BACK_THICKNESS / 2.0, 0.0, 0.0)
                ),
                material="drawer_wood",
                name="drawer_back",
            )
            drawer.visual(
                Box((LABEL_THICKNESS, LABEL_WIDTH, LABEL_HEIGHT)),
                origin=Origin(
                    xyz=(
                        DRAWER_FRONT_THICKNESS + LABEL_THICKNESS / 2.0 - 0.0002,
                        0.0,
                        LABEL_CENTER_Z,
                    )
                ),
                material="card_stock",
                name="label_card",
            )
            drawer.visual(
                Box(
                    (
                        HANDLE_PLATE_THICKNESS,
                        HANDLE_PLATE_WIDTH,
                        HANDLE_PLATE_HEIGHT,
                    )
                ),
                origin=Origin(
                    xyz=(
                        DRAWER_FRONT_THICKNESS + HANDLE_PLATE_THICKNESS / 2.0 - 0.0002,
                        0.0,
                        HANDLE_Z,
                    )
                ),
                material="hardware",
                name="pull_plate",
            )
            for post_sign, post_index in ((-1.0, 0), (1.0, 1)):
                drawer.visual(
                    Cylinder(radius=HANDLE_POST_RADIUS, length=HANDLE_POST_LENGTH),
                    origin=Origin(
                        xyz=(
                            DRAWER_FRONT_THICKNESS + HANDLE_POST_LENGTH / 2.0,
                            post_sign * HANDLE_POST_Y,
                            HANDLE_Z,
                        ),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material="hardware",
                    name=f"pull_post_{post_index}",
                )
            drawer.visual(
                Cylinder(radius=HANDLE_BAR_RADIUS, length=HANDLE_BAR_LENGTH),
                origin=Origin(
                    xyz=(HANDLE_BAR_X, 0.0, HANDLE_Z),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material="hardware",
                name="pull_bar",
            )

            hinge_z = LABEL_CENTER_Z + PROTECTOR_HEIGHT / 2.0
            for knuckle_sign, knuckle_index in ((-1.0, 0), (1.0, 1)):
                drawer.visual(
                    Cylinder(radius=HINGE_BARREL_RADIUS, length=DRAWER_KNUCKLE_LENGTH),
                    origin=Origin(
                        xyz=(
                            HINGE_AXIS_X,
                            knuckle_sign * 0.036,
                            hinge_z,
                        ),
                        rpy=(pi / 2.0, 0.0, 0.0),
                    ),
                    material="hardware",
                    name=f"hinge_knuckle_{knuckle_index}",
                )

            model.articulation(
                f"slide_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=cabinet,
                child=drawer,
                origin=Origin(xyz=(CABINET_DEPTH, y_center, z_center)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                    effort=40.0,
                    velocity=0.25,
                ),
            )

            protector = model.part(protector_name)
            protector.visual(
                Box(
                    (
                        PROTECTOR_THICKNESS,
                        PROTECTOR_WIDTH - 2.0 * PROTECTOR_FRAME,
                        PROTECTOR_HEIGHT - 2.0 * PROTECTOR_FRAME,
                    )
                ),
                origin=Origin(
                    xyz=(PROTECTOR_FRONT_OFFSET, 0.0, -PROTECTOR_HEIGHT / 2.0)
                ),
                material="protector_glass",
                name="plate",
            )
            protector.visual(
                Box((PROTECTOR_THICKNESS, PROTECTOR_WIDTH, PROTECTOR_FRAME)),
                origin=Origin(
                    xyz=(
                        PROTECTOR_FRONT_OFFSET,
                        0.0,
                        -PROTECTOR_FRAME / 2.0,
                    )
                ),
                material="hardware",
                name="frame_top",
            )
            protector.visual(
                Box((PROTECTOR_THICKNESS, PROTECTOR_WIDTH, PROTECTOR_FRAME)),
                origin=Origin(
                    xyz=(
                        PROTECTOR_FRONT_OFFSET,
                        0.0,
                        -PROTECTOR_HEIGHT + PROTECTOR_FRAME / 2.0,
                    )
                ),
                material="hardware",
                name="frame_bottom",
            )
            protector.visual(
                Box((PROTECTOR_THICKNESS, PROTECTOR_FRAME, PROTECTOR_HEIGHT)),
                origin=Origin(
                    xyz=(
                        PROTECTOR_FRONT_OFFSET,
                        -PROTECTOR_WIDTH / 2.0 + PROTECTOR_FRAME / 2.0,
                        -PROTECTOR_HEIGHT / 2.0,
                    )
                ),
                material="hardware",
                name="frame_side_0",
            )
            protector.visual(
                Box((PROTECTOR_THICKNESS, PROTECTOR_FRAME, PROTECTOR_HEIGHT)),
                origin=Origin(
                    xyz=(
                        PROTECTOR_FRONT_OFFSET,
                        PROTECTOR_WIDTH / 2.0 - PROTECTOR_FRAME / 2.0,
                        -PROTECTOR_HEIGHT / 2.0,
                    )
                ),
                material="hardware",
                name="frame_side_1",
            )
            protector.visual(
                Cylinder(radius=HINGE_BARREL_RADIUS, length=PROTECTOR_KNUCKLE_LENGTH),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material="hardware",
                name="hinge_barrel",
            )

            model.articulation(
                f"label_hinge_{row}_{col}",
                ArticulationType.REVOLUTE,
                parent=drawer,
                child=protector,
                origin=Origin(xyz=(HINGE_AXIS_X, 0.0, hinge_z)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    lower=0.0,
                    upper=PROTECTOR_OPEN_LIMIT,
                    effort=0.6,
                    velocity=1.6,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part("drawer_2_1")
    protector = object_model.get_part("protector_2_1")
    slide = object_model.get_articulation("slide_2_1")
    hinge = object_model.get_articulation("label_hinge_2_1")

    slide_upper = 0.0
    if slide.motion_limits is not None and slide.motion_limits.upper is not None:
        slide_upper = slide.motion_limits.upper

    hinge_upper = 0.0
    if hinge.motion_limits is not None and hinge.motion_limits.upper is not None:
        hinge_upper = hinge.motion_limits.upper

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: slide_upper}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="x",
            min_overlap=0.30,
            name="extended drawer keeps substantial retained insertion",
        )

    ctx.check(
        "drawer extends outward from the cabinet front",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.15,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    ctx.expect_gap(
        protector,
        drawer,
        axis="x",
        positive_elem="plate",
        negative_elem="label_card",
        min_gap=0.0005,
        max_gap=0.004,
        name="closed label protector sits just proud of the card",
    )
    ctx.expect_overlap(
        protector,
        drawer,
        axes="yz",
        elem_a="plate",
        elem_b="label_card",
        min_overlap=0.040,
        name="closed label protector covers the label card window",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(protector, elem="plate")
    with ctx.pose({hinge: hinge_upper}):
        open_plate_aabb = ctx.part_element_world_aabb(protector, elem="plate")

    protector_opens_out = False
    if rest_plate_aabb is not None and open_plate_aabb is not None:
        protector_opens_out = (
            open_plate_aabb[1][0] > rest_plate_aabb[1][0] + 0.020
            and open_plate_aabb[0][2] > rest_plate_aabb[0][2] + 0.030
        )
    ctx.check(
        "label protector flips upward and outward",
        protector_opens_out,
        details=f"rest_plate_aabb={rest_plate_aabb}, open_plate_aabb={open_plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
