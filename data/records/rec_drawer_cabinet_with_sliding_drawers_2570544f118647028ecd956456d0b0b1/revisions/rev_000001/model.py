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


OUTER_W = 0.40
OUTER_D = 0.68
OUTER_H = 1.05

WALL_T = 0.015
BACK_T = 0.010
DIVIDER_T = 0.015

INNER_W = OUTER_W - 2.0 * WALL_T
BAY_H = (OUTER_H - 2.0 * WALL_T - 2.0 * DIVIDER_T) / 3.0

FRONT_W = INNER_W - 0.004
FRONT_H = BAY_H + DIVIDER_T - 0.004
FRONT_T = 0.020

DRAWER_W = 0.333
DRAWER_D = 0.560
DRAWER_H = 0.255
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.010
DRAWER_BACK_T = 0.012

RUNNER_W = 0.0135
RUNNER_H = 0.030
RUNNER_D = 0.520
RUNNER_CENTER_Y = 0.360

SLIDE_W = 0.005
SLIDE_H = 0.022
SLIDE_D = 0.500
SLIDE_CENTER_Y_LOCAL = -0.330

DRAWER_TRAVEL = 0.360


def bay_bottom(index: int) -> float:
    return WALL_T + index * (BAY_H + DIVIDER_T)


def bay_center(index: int) -> float:
    return bay_bottom(index) + BAY_H / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_drawer_filing_cabinet")

    carcass_steel = model.material("carcass_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    drawer_steel = model.material("drawer_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    runner_steel = model.material("runner_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    pull_dark = model.material("pull_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((WALL_T, OUTER_D, OUTER_H)),
        origin=Origin(xyz=(-OUTER_W / 2.0 + WALL_T / 2.0, OUTER_D / 2.0, OUTER_H / 2.0)),
        material=carcass_steel,
        name="left_wall",
    )
    carcass.visual(
        Box((WALL_T, OUTER_D, OUTER_H)),
        origin=Origin(xyz=(OUTER_W / 2.0 - WALL_T / 2.0, OUTER_D / 2.0, OUTER_H / 2.0)),
        material=carcass_steel,
        name="right_wall",
    )
    carcass.visual(
        Box((INNER_W, OUTER_D - BACK_T, WALL_T)),
        origin=Origin(xyz=(0.0, BACK_T + (OUTER_D - BACK_T) / 2.0, WALL_T / 2.0)),
        material=carcass_steel,
        name="bottom_panel",
    )
    carcass.visual(
        Box((INNER_W, OUTER_D - BACK_T, WALL_T)),
        origin=Origin(
            xyz=(0.0, BACK_T + (OUTER_D - BACK_T) / 2.0, OUTER_H - WALL_T / 2.0)
        ),
        material=carcass_steel,
        name="top_panel",
    )
    carcass.visual(
        Box((INNER_W, BACK_T, OUTER_H)),
        origin=Origin(xyz=(0.0, BACK_T / 2.0, OUTER_H / 2.0)),
        material=carcass_steel,
        name="back_panel",
    )

    divider_z = [
        WALL_T + BAY_H + DIVIDER_T / 2.0,
        WALL_T + 2.0 * BAY_H + 1.5 * DIVIDER_T,
    ]
    for index, z_center in enumerate(divider_z):
        carcass.visual(
            Box((INNER_W, OUTER_D - BACK_T, DIVIDER_T)),
            origin=Origin(xyz=(0.0, BACK_T + (OUTER_D - BACK_T) / 2.0, z_center)),
            material=carcass_steel,
            name=f"divider_{index}",
        )

    runner_x = DRAWER_W / 2.0 + SLIDE_W + RUNNER_W / 2.0
    for index in range(3):
        z_center = bay_center(index)
        carcass.visual(
            Box((RUNNER_W, RUNNER_D, RUNNER_H)),
            origin=Origin(xyz=(-runner_x, RUNNER_CENTER_Y, z_center)),
            material=runner_steel,
            name=f"runner_{index}_left",
        )
        carcass.visual(
            Box((RUNNER_W, RUNNER_D, RUNNER_H)),
            origin=Origin(xyz=(runner_x, RUNNER_CENTER_Y, z_center)),
            material=runner_steel,
            name=f"runner_{index}_right",
        )

    handle_w = FRONT_W * 0.64
    label_w = FRONT_W * 0.28
    label_h = 0.040
    shell_center_y = -(DRAWER_D / 2.0 + FRONT_T / 2.0)

    for index in range(3):
        drawer = model.part(f"drawer_{index}")
        drawer.visual(
            Box((FRONT_W, FRONT_T, FRONT_H)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=drawer_steel,
            name="front_panel",
        )
        drawer.visual(
            Box((handle_w, 0.018, 0.034)),
            origin=Origin(xyz=(0.0, FRONT_T / 2.0 + 0.009, 0.0)),
            material=pull_dark,
            name="pull_bar",
        )
        drawer.visual(
            Box((label_w, 0.006, label_h)),
            origin=Origin(xyz=(0.0, FRONT_T / 2.0 + 0.003, FRONT_H * 0.23)),
            material=runner_steel,
            name="label_frame",
        )

        drawer.visual(
            Box((DRAWER_SIDE_T, DRAWER_D, DRAWER_H)),
            origin=Origin(
                xyz=(-DRAWER_W / 2.0 + DRAWER_SIDE_T / 2.0, shell_center_y, 0.0)
            ),
            material=drawer_steel,
            name="left_side",
        )
        drawer.visual(
            Box((DRAWER_SIDE_T, DRAWER_D, DRAWER_H)),
            origin=Origin(
                xyz=(DRAWER_W / 2.0 - DRAWER_SIDE_T / 2.0, shell_center_y, 0.0)
            ),
            material=drawer_steel,
            name="right_side",
        )
        drawer.visual(
            Box((DRAWER_W - 2.0 * DRAWER_SIDE_T, DRAWER_D, DRAWER_BOTTOM_T)),
            origin=Origin(
                xyz=(
                    0.0,
                    shell_center_y,
                    -DRAWER_H / 2.0 + DRAWER_BOTTOM_T / 2.0,
                )
            ),
            material=drawer_steel,
            name="bottom_pan",
        )
        drawer.visual(
            Box((DRAWER_W - 2.0 * DRAWER_SIDE_T, DRAWER_BACK_T, DRAWER_H)),
            origin=Origin(
                xyz=(
                    0.0,
                    -(FRONT_T / 2.0 + DRAWER_D - DRAWER_BACK_T / 2.0),
                    0.0,
                )
            ),
            material=drawer_steel,
            name="back_pan",
        )
        drawer.visual(
            Box((SLIDE_W, SLIDE_D, SLIDE_H)),
            origin=Origin(
                xyz=(-(DRAWER_W / 2.0 + SLIDE_W / 2.0), SLIDE_CENTER_Y_LOCAL, 0.0)
            ),
            material=runner_steel,
            name="slide_left",
        )
        drawer.visual(
            Box((SLIDE_W, SLIDE_D, SLIDE_H)),
            origin=Origin(
                xyz=((DRAWER_W / 2.0 + SLIDE_W / 2.0), SLIDE_CENTER_Y_LOCAL, 0.0)
            ),
            material=runner_steel,
            name="slide_right",
        )

        model.articulation(
            f"slide_{index}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(
                xyz=(0.0, OUTER_D + FRONT_T / 2.0, bay_center(index)),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.40,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    drawers = [object_model.get_part(f"drawer_{index}") for index in range(3)]
    slides = [object_model.get_articulation(f"slide_{index}") for index in range(3)]

    for index, drawer in enumerate(drawers):
        ctx.expect_gap(
            drawer,
            carcass,
            axis="y",
            positive_elem="front_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"drawer_{index} front closes flush with cabinet face",
        )
        ctx.expect_gap(
            drawer,
            carcass,
            axis="x",
            positive_elem="slide_left",
            negative_elem=f"runner_{index}_left",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"drawer_{index} left slide bears on fixed runner",
        )
        ctx.expect_gap(
            carcass,
            drawer,
            axis="x",
            positive_elem=f"runner_{index}_right",
            negative_elem="slide_right",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"drawer_{index} right slide bears on fixed runner",
        )

    ctx.expect_gap(
        drawers[1],
        drawers[0],
        axis="z",
        positive_elem="front_panel",
        negative_elem="front_panel",
        min_gap=0.001,
        max_gap=0.006,
        name="lower and middle drawer fronts keep a slim reveal gap",
    )
    ctx.expect_gap(
        drawers[2],
        drawers[1],
        axis="z",
        positive_elem="front_panel",
        negative_elem="front_panel",
        min_gap=0.001,
        max_gap=0.006,
        name="middle and upper drawer fronts keep a slim reveal gap",
    )

    for index, (drawer, slide) in enumerate(zip(drawers, slides)):
        rest_position = ctx.part_world_position(drawer)
        upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
        with ctx.pose({slide: upper}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="slide_left",
                elem_b=f"runner_{index}_left",
                min_overlap=0.14,
                name=f"drawer_{index} left slide stays engaged at full extension",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="slide_right",
                elem_b=f"runner_{index}_right",
                min_overlap=0.14,
                name=f"drawer_{index} right slide stays engaged at full extension",
            )
            extended_position = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{index} extends forward",
            rest_position is not None
            and extended_position is not None
            and extended_position[1] > rest_position[1] + 0.30,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


object_model = build_object_model()
