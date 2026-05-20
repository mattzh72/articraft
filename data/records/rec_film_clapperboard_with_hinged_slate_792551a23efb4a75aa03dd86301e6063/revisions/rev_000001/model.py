from __future__ import annotations

from math import radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BOARD_W = 0.305
BOARD_H = 0.215
BOARD_T = 0.012

FACE_W = 0.275
FACE_H = 0.172
FACE_T = 0.0018

TOP_RAIL_W = 0.309
TOP_RAIL_D = 0.022
TOP_RAIL_H = 0.024

TRAY_W = 0.225
TRAY_D = 0.022
TRAY_T = 0.007
TRAY_LIP_D = 0.004
TRAY_LIP_H = 0.013

STICK_W = 0.320
STICK_D = 0.038
STICK_T = 0.020
STRIPE_L = 0.058
STRIPE_D = 0.011
STRIPE_T = 0.003


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="directors_clapperboard")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    dry_erase_white = model.material("dry_erase_white", rgba=(0.94, 0.95, 0.93, 1.0))
    bright_white = model.material("bright_white", rgba=(0.98, 0.98, 0.98, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_W, BOARD_T, BOARD_H)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_H / 2.0)),
        material=charcoal,
        name="board_panel",
    )
    board.visual(
        Box((FACE_W, FACE_T, FACE_H)),
        origin=Origin(xyz=(0.0, BOARD_T / 2.0 + FACE_T / 2.0, 0.100)),
        material=dry_erase_white,
        name="face_panel",
    )
    board.visual(
        Box((TOP_RAIL_W, TOP_RAIL_D, TOP_RAIL_H)),
        origin=Origin(
            xyz=(0.0, -BOARD_T / 2.0 + TOP_RAIL_D / 2.0, BOARD_H + TOP_RAIL_H / 2.0)
        ),
        material=matte_black,
        name="top_rail",
    )
    board.visual(
        Box((TRAY_W, TRAY_D, TRAY_T)),
        origin=Origin(xyz=(0.0, BOARD_T / 2.0 + TRAY_D / 2.0, -TRAY_T / 2.0)),
        material=charcoal,
        name="tray_floor",
    )
    board.visual(
        Box((TRAY_W, TRAY_LIP_D, TRAY_LIP_H)),
        origin=Origin(
            xyz=(
                0.0,
                BOARD_T / 2.0 + TRAY_D - TRAY_LIP_D / 2.0,
                TRAY_LIP_H / 2.0,
            )
        ),
        material=matte_black,
        name="tray_lip",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((STICK_W, STICK_D, STICK_T)),
        origin=Origin(xyz=(0.0, STICK_D / 2.0, STICK_T / 2.0)),
        material=matte_black,
        name="stick_body",
    )

    stripe_xs = (-0.115, -0.060, -0.005, 0.050, 0.105)
    for idx, x_pos in enumerate(stripe_xs):
        clapstick.visual(
            Box((STRIPE_L, STRIPE_D, STRIPE_T)),
            origin=Origin(
                xyz=(x_pos, STICK_D / 2.0, STICK_T + STRIPE_T / 2.0),
                rpy=(0.0, 0.0, radians(-33.0)),
            ),
            material=bright_white,
            name=f"stripe_{idx}",
        )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, -BOARD_T / 2.0, BOARD_H + TOP_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=5.0,
            lower=0.0,
            upper=radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("board_to_clapstick")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            clapstick,
            board,
            elem_a="stick_body",
            elem_b="top_rail",
            contact_tol=1e-6,
            name="closed clapstick seats on the fixed top rail",
        )
        ctx.expect_overlap(
            clapstick,
            board,
            axes="x",
            elem_a="stick_body",
            elem_b="top_rail",
            min_overlap=0.300,
            name="clapstick spans the board width",
        )

        panel_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
        tray_aabb = ctx.part_element_world_aabb(board, elem="tray_floor")
        ctx.check(
            "marker tray sits below the board and projects forward",
            panel_aabb is not None
            and tray_aabb is not None
            and tray_aabb[1][2] <= panel_aabb[0][2] + 1e-6
            and tray_aabb[1][1] > panel_aabb[1][1] + 0.010,
            details=f"panel_aabb={panel_aabb}, tray_aabb={tray_aabb}",
        )

        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")

    limits = hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
            ctx.check(
                "clapstick opens upward from the board",
                closed_aabb is not None
                and open_aabb is not None
                and open_aabb[1][2] > closed_aabb[1][2] + 0.018,
                details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
