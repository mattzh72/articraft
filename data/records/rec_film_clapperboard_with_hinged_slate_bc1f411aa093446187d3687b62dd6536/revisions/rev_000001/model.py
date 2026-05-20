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


BOARD_W = 0.170
BOARD_H = 0.235
PANEL_T = 0.005
TRIM_T = 0.0008
HINGE_RAIL_D = 0.006
HINGE_RAIL_H = 0.010
HINGE_Y = 0.0055
HINGE_Z = BOARD_H + 0.001

CLAP_W = BOARD_W + 0.004
CLAP_T = 0.0055
CLAP_H = 0.028
BARREL_R = 0.0025


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehearsal_clapperboard")

    matte_black = model.material("matte_black", rgba=(0.06, 0.06, 0.07, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.12, 1.0))
    chalk_white = model.material("chalk_white", rgba=(0.95, 0.95, 0.93, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_W, PANEL_T, BOARD_H)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_H / 2.0)),
        material=matte_black,
        name="panel",
    )
    board.visual(
        Box((BOARD_W, HINGE_RAIL_D, HINGE_RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_H - 0.002)),
        material=dark_metal,
        name="hinge_rail",
    )
    board.visual(
        Box((0.108, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.007, 0.018)),
        material=charcoal,
        name="chalk_ledge",
    )

    front_trim_y = PANEL_T / 2.0 + TRIM_T / 2.0 - 0.0001
    inner_w = BOARD_W - 0.020
    board.visual(
        Box((inner_w, TRIM_T, 0.004)),
        origin=Origin(xyz=(0.0, front_trim_y, BOARD_H - 0.016)),
        material=chalk_white,
        name="trim_top",
    )
    board.visual(
        Box((inner_w, TRIM_T, 0.004)),
        origin=Origin(xyz=(0.0, front_trim_y, 0.040)),
        material=chalk_white,
        name="trim_bottom",
    )
    board.visual(
        Box((0.004, TRIM_T, BOARD_H - 0.056)),
        origin=Origin(xyz=(-inner_w / 2.0 + 0.002, front_trim_y, 0.120)),
        material=chalk_white,
        name="trim_left",
    )
    board.visual(
        Box((0.004, TRIM_T, BOARD_H - 0.056)),
        origin=Origin(xyz=(inner_w / 2.0 - 0.002, front_trim_y, 0.120)),
        material=chalk_white,
        name="trim_right",
    )

    for name, z in (
        ("rule_top", 0.152),
        ("rule_mid", 0.124),
        ("rule_low", 0.096),
    ):
        board.visual(
            Box((inner_w - 0.012, TRIM_T, 0.003)),
            origin=Origin(xyz=(0.0, front_trim_y, z)),
            material=chalk_white,
            name=name,
        )

    for name, x in (("column_left", -0.035), ("column_mid", 0.018)):
        board.visual(
            Box((0.003, TRIM_T, 0.084)),
            origin=Origin(xyz=(x, front_trim_y, 0.110)),
            material=chalk_white,
            name=name,
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((CLAP_W, CLAP_T, CLAP_H)),
        origin=Origin(xyz=(0.0, 0.0023, -0.014)),
        material=charcoal,
        name="stick_body",
    )
    clapstick.visual(
        Box((CLAP_W, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.0015, -0.0015)),
        material=charcoal,
        name="hinge_web",
    )
    clapstick.visual(
        Cylinder(radius=BARREL_R, length=BOARD_W),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    clapstick.visual(
        Box((CLAP_W, CLAP_T + 0.0004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0023, -0.026)),
        material=chalk_white,
        name="strike_edge",
    )

    for index, x in enumerate((-0.066, -0.033, 0.0, 0.033, 0.066)):
        clapstick.visual(
            Box((0.016, CLAP_T + 0.0004, 0.040)),
            origin=Origin(xyz=(x, 0.0023, -0.014), rpy=(0.0, 0.62, 0.0)),
            material=chalk_white,
            name=f"stripe_{index}",
        )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("board_to_clapstick")
    limits = hinge.motion_limits

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            clapstick,
            board,
            axes="x",
            elem_a="stick_body",
            elem_b="hinge_rail",
            min_overlap=0.165,
            name="clapstick spans the narrow slate width",
        )
        ctx.expect_gap(
            clapstick,
            board,
            axis="y",
            positive_elem="strike_edge",
            negative_elem="panel",
            min_gap=0.0005,
            max_gap=0.0040,
            name="closed clapstick sits just in front of the board face",
        )
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")

    open_aabb = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.upper}):
            ctx.expect_gap(
                clapstick,
                board,
                axis="y",
                positive_elem="strike_edge",
                negative_elem="panel",
                min_gap=0.020,
                name="opened clapstick swings clear of the slate front",
            )
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")

    ctx.check(
        "clapstick opens upward from the hinge rail",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > closed_aabb[0][2] + 0.015,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
