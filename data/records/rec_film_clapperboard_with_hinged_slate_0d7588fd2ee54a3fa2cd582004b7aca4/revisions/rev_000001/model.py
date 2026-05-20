from __future__ import annotations

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


BOARD_W = 0.280
BOARD_H = 0.230
BOARD_T = 0.008
WHITE_W = 0.252
WHITE_H = 0.188
WHITE_T = 0.0012
WHITE_CENTER_Z = -0.009

STRAP_W = 0.264
STRAP_H = 0.020
STRAP_T = 0.002
STRAP_CENTER_Z = BOARD_H / 2.0 - 0.006
BARREL_R = 0.005
BARREL_L = 0.048
BARREL_Y = 0.004
BARREL_Z = STRAP_H / 2.0
BARREL_STEP = 0.052
BARREL_CENTERS = (-0.104, -0.052, 0.000, 0.052, 0.104)

STICK_W = 0.286
STICK_H = 0.045
STICK_T = 0.012
STICK_CENTER_Y = 0.0037
STICK_CENTER_Z = -STICK_H / 2.0 - BARREL_R
STRIPE_T = 0.001
STRIPE_Y = STICK_CENTER_Y + STICK_T / 2.0 - STRIPE_T / 2.0 - 0.0002
STRIPE_PITCH = 0.95
STRIPE_SIZE = (0.085, STRIPE_T, 0.012)
STRIPE_X = (-0.104, -0.052, 0.000, 0.052, 0.104)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dry_erase_clapperboard")

    model.material("frame_black", rgba=(0.10, 0.10, 0.10, 1.0))
    model.material("board_white", rgba=(0.97, 0.97, 0.95, 1.0))
    model.material("strap_metal", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("stripe_white", rgba=(0.99, 0.99, 0.99, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_W, BOARD_T, BOARD_H)),
        material="frame_black",
        name="board_body",
    )
    board.visual(
        Box((WHITE_W, WHITE_T, WHITE_H)),
        origin=Origin(xyz=(0.0, BOARD_T / 2.0 - WHITE_T / 2.0 + 0.0002, WHITE_CENTER_Z)),
        material="board_white",
        name="writing_surface",
    )

    strap = model.part("hinge_strap")
    strap.visual(
        Box((STRAP_W, STRAP_T, STRAP_H)),
        material="strap_metal",
        name="strap_plate",
    )
    for index, x_pos in enumerate((BARREL_CENTERS[0], BARREL_CENTERS[2], BARREL_CENTERS[4])):
        strap.visual(
            Cylinder(radius=BARREL_R, length=BARREL_L),
            origin=Origin(
                xyz=(x_pos, BARREL_Y, BARREL_Z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material="strap_metal",
            name=f"strap_knuckle_{index}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((STICK_W, STICK_T, STICK_H)),
        origin=Origin(xyz=(0.0, STICK_CENTER_Y, STICK_CENTER_Z)),
        material="frame_black",
        name="stick_body",
    )
    for index, x_pos in enumerate((BARREL_CENTERS[1], BARREL_CENTERS[3])):
        clapstick.visual(
            Cylinder(radius=BARREL_R, length=BARREL_L),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material="frame_black",
            name=f"stick_knuckle_{index}",
        )
    for index, x_pos in enumerate(STRIPE_X):
        clapstick.visual(
            Box(STRIPE_SIZE),
            origin=Origin(
                xyz=(x_pos, STRIPE_Y, STICK_CENTER_Z),
                rpy=(0.0, -STRIPE_PITCH, 0.0),
            ),
            material="stripe_white",
            name=f"stripe_{index}",
        )

    model.articulation(
        "board_to_hinge_strap",
        ArticulationType.FIXED,
        parent=board,
        child=strap,
        origin=Origin(xyz=(0.0, BOARD_T / 2.0 + STRAP_T / 2.0, STRAP_CENTER_Z)),
    )
    model.articulation(
        "hinge_strap_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=strap,
        child=clapstick,
        origin=Origin(xyz=(0.0, BARREL_Y, BARREL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    strap = object_model.get_part("hinge_strap")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("hinge_strap_to_clapstick")
    limits = hinge.motion_limits

    closed_pose = 0.0
    open_pose = 1.20
    if limits is not None:
        if limits.lower is not None:
            closed_pose = limits.lower
        if limits.upper is not None:
            open_pose = limits.upper

    with ctx.pose({hinge: closed_pose}):
        ctx.expect_gap(
            strap,
            board,
            axis="y",
            positive_elem="strap_plate",
            negative_elem="board_body",
            max_gap=0.0005,
            max_penetration=0.0,
            name="hinge strap sits on the board face",
        )
        ctx.expect_gap(
            clapstick,
            strap,
            axis="y",
            positive_elem="stick_body",
            negative_elem="strap_plate",
            min_gap=0.0003,
            max_gap=0.0025,
            name="clapstick stays visibly separate from the metal strap",
        )
        ctx.expect_overlap(
            clapstick,
            board,
            axes="x",
            elem_a="stick_body",
            elem_b="board_body",
            min_overlap=0.250,
            name="clapstick spans the width of the board",
        )
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")

    with ctx.pose({hinge: open_pose}):
        open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")

    if closed_aabb is not None and open_aabb is not None:
        closed_front = closed_aabb[1][1]
        open_front = open_aabb[1][1]
        closed_bottom = closed_aabb[0][2]
        open_bottom = open_aabb[0][2]
    else:
        closed_front = None
        open_front = None
        closed_bottom = None
        open_bottom = None

    ctx.check(
        "clapstick swings forward when opened",
        closed_front is not None and open_front is not None and open_front > closed_front + 0.020,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )
    ctx.check(
        "clapstick free edge rises when opened",
        closed_bottom is not None and open_bottom is not None and open_bottom > closed_bottom + 0.020,
        details=f"closed_bottom={closed_bottom}, open_bottom={open_bottom}",
    )

    return ctx.report()


object_model = build_object_model()
