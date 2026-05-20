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


CABINET_WIDTH = 0.46
CABINET_DEPTH = 0.42
CABINET_HEIGHT = 0.96

SIDE_WALL_THICKNESS = 0.016
TOP_THICKNESS = 0.018
BOTTOM_THICKNESS = 0.024
BACK_THICKNESS = 0.010

DRAWER_COUNT = 6
DRAWER_FACE_WIDTH = 0.414
DRAWER_FACE_HEIGHT = 0.122
DRAWER_FACE_THICKNESS = 0.018
DRAWER_PITCH = 0.128
DRAWER_TRAVEL = 0.18

DRAWER_TRAY_WIDTH = 0.398
DRAWER_TRAY_DEPTH = 0.300
DRAWER_TRAY_HEIGHT = 0.090
DRAWER_WALL_THICKNESS = 0.006
DRAWER_BOTTOM_THICKNESS = 0.006
DRAWER_TRAY_LOWER_Z = -0.051

PULL_WIDTH = 0.250
PULL_HEIGHT = 0.018
PULL_DEPTH = 0.020
PULL_Z_OFFSET = 0.022

RUNNER_WIDTH = 0.017
RUNNER_HEIGHT = 0.006
RUNNER_LENGTH = 0.260
RUNNER_FRONT_Y = 0.170
RUNNER_Z_OFFSET = DRAWER_TRAY_LOWER_Z - RUNNER_HEIGHT / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="specimen_drawer_cabinet")

    carcass = model.material("carcass", rgba=(0.83, 0.84, 0.80, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.90, 0.90, 0.87, 1.0))
    drawer_tray = model.material("drawer_tray", rgba=(0.78, 0.80, 0.82, 1.0))
    pull = model.material("pull", rgba=(0.46, 0.48, 0.50, 1.0))

    body = model.part("body")

    body.visual(
        Box((SIDE_WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH / 2.0) + (SIDE_WALL_THICKNESS / 2.0),
                0.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=carcass,
        name="side_0",
    )
    body.visual(
        Box((SIDE_WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH / 2.0) - (SIDE_WALL_THICKNESS / 2.0),
                0.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=carcass,
        name="side_1",
    )
    body.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - (TOP_THICKNESS / 2.0))),
        material=carcass,
        name="top",
    )
    body.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS / 2.0)),
        material=carcass,
        name="bottom",
    )

    back_height = CABINET_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_WALL_THICKNESS, BACK_THICKNESS, back_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH / 2.0) + (BACK_THICKNESS / 2.0),
                BOTTOM_THICKNESS + (back_height / 2.0),
            )
        ),
        material=carcass,
        name="back",
    )

    runner_center_y = RUNNER_FRONT_Y - (RUNNER_LENGTH / 2.0)
    runner_half_span_x = (
        (CABINET_WIDTH - 2.0 * SIDE_WALL_THICKNESS) / 2.0
    ) - (RUNNER_WIDTH / 2.0)
    first_drawer_center_z = 0.09 + DRAWER_FACE_HEIGHT / 2.0

    for index in range(DRAWER_COUNT):
        drawer_center_z = first_drawer_center_z + index * DRAWER_PITCH
        runner_center_z = drawer_center_z + RUNNER_Z_OFFSET
        for side_sign in (-1.0, 1.0):
            body.visual(
                Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(
                    xyz=(
                        side_sign * runner_half_span_x,
                        runner_center_y,
                        runner_center_z,
                    )
                ),
                material=carcass,
                name=f"runner_{index}_{0 if side_sign < 0.0 else 1}",
            )

    tray_center_y = -(DRAWER_FACE_THICKNESS / 2.0) - (DRAWER_TRAY_DEPTH / 2.0)
    tray_center_z = DRAWER_TRAY_LOWER_Z + (DRAWER_TRAY_HEIGHT / 2.0)
    bottom_center_z = DRAWER_TRAY_LOWER_Z + (DRAWER_BOTTOM_THICKNESS / 2.0)
    side_center_x = (DRAWER_TRAY_WIDTH / 2.0) - (DRAWER_WALL_THICKNESS / 2.0)
    back_center_y = (
        -(DRAWER_FACE_THICKNESS / 2.0)
        - DRAWER_TRAY_DEPTH
        + (DRAWER_WALL_THICKNESS / 2.0)
    )
    front_center_y = (CABINET_DEPTH / 2.0) - (DRAWER_FACE_THICKNESS / 2.0)
    pull_center_y = (DRAWER_FACE_THICKNESS / 2.0) + (PULL_DEPTH / 2.0)

    for index in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{index}")
        drawer.visual(
            Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT)),
            material=drawer_front,
            name="front",
        )
        drawer.visual(
            Box((PULL_WIDTH, PULL_DEPTH, PULL_HEIGHT)),
            origin=Origin(xyz=(0.0, pull_center_y, PULL_Z_OFFSET)),
            material=pull,
            name="pull",
        )
        drawer.visual(
            Box((DRAWER_TRAY_WIDTH, DRAWER_TRAY_DEPTH, DRAWER_BOTTOM_THICKNESS)),
            origin=Origin(xyz=(0.0, tray_center_y, bottom_center_z)),
            material=drawer_tray,
            name="bottom",
        )
        for side_sign in (-1.0, 1.0):
            drawer.visual(
                Box((DRAWER_WALL_THICKNESS, DRAWER_TRAY_DEPTH, DRAWER_TRAY_HEIGHT)),
                origin=Origin(
                    xyz=(
                        side_sign * side_center_x,
                        tray_center_y,
                        tray_center_z,
                    )
                ),
                material=drawer_tray,
                name=f"side_{0 if side_sign < 0.0 else 1}",
            )
        drawer.visual(
            Box(
                (
                    DRAWER_TRAY_WIDTH - 2.0 * DRAWER_WALL_THICKNESS,
                    DRAWER_WALL_THICKNESS,
                    DRAWER_TRAY_HEIGHT,
                )
            ),
            origin=Origin(xyz=(0.0, back_center_y, tray_center_z)),
            material=drawer_tray,
            name="back",
        )

        drawer_center_z = first_drawer_center_z + index * DRAWER_PITCH
        model.articulation(
            f"body_to_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, front_center_y, drawer_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=DRAWER_TRAVEL,
                effort=80.0,
                velocity=0.25,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    for index in range(DRAWER_COUNT):
        drawer = object_model.get_part(f"drawer_{index}")
        slide = object_model.get_articulation(f"body_to_drawer_{index}")
        limits = slide.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else limits.upper

        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            margin=0.0,
            name=f"drawer_{index} stays inside cabinet width and stack at rest",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.28,
            name=f"drawer_{index} is substantially inserted at rest",
        )

        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({slide: upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="xz",
                margin=0.0,
                name=f"drawer_{index} stays laterally guided at full extension",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                min_overlap=0.13,
                name=f"drawer_{index} retains insertion at full extension",
            )
            extended_position = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{index} extends forward",
            rest_position is not None
            and extended_position is not None
            and extended_position[1] > rest_position[1] + 0.12,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


object_model = build_object_model()
