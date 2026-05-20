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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="documentary_slate")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    white_face = model.material("white_face", rgba=(0.95, 0.96, 0.95, 1.0))
    bright_white = model.material("bright_white", rgba=(0.99, 0.99, 0.99, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))

    board_w = 0.265
    board_h = 0.185
    board_t = 0.010

    hinge_radius = 0.0028
    hinge_axis_y = 0.0018
    hinge_axis_z = board_h / 2.0 + 0.0005

    clap_w = 0.250
    clap_h = 0.034
    clap_t = 0.012
    clap_y = -0.0030
    clap_bottom_z = 0.0025

    board = model.part("board")
    board.visual(
        Box((board_w, board_t, board_h)),
        material=graphite,
        name="board_body",
    )
    board.visual(
        Box((0.221, 0.0016, 0.147)),
        origin=Origin(xyz=(0.0, -(board_t / 2.0) + 0.0008, -0.006)),
        material=white_face,
        name="whiteboard_face",
    )
    board.visual(
        Box((0.205, 0.0035, 0.010)),
        origin=Origin(xyz=(0.0, board_t / 2.0 + 0.00175, -0.055)),
        material=matte_black,
        name="rear_grip",
    )
    for x_pos, name in ((-0.085, "board_knuckle_0"), (0.0, "board_knuckle_1"), (0.085, "board_knuckle_2")):
        board.visual(
            Cylinder(radius=hinge_radius, length=0.034),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=matte_black,
            name=name,
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((clap_w, clap_t, clap_h)),
        origin=Origin(xyz=(0.0, clap_y, clap_bottom_z + clap_h / 2.0)),
        material=matte_black,
        name="clap_bar",
    )
    stripe_x = (-0.093, -0.047, -0.001, 0.045, 0.091)
    for idx, x_pos in enumerate(stripe_x):
        clapstick.visual(
            Box((0.038, 0.0022, 0.009)),
            origin=Origin(
                xyz=(x_pos, -0.0079, clap_bottom_z + 0.0175),
                rpy=(0.0, 0.78, 0.0),
            ),
            material=bright_white,
            name=f"stripe_{idx}",
        )
    for x_pos, name in ((-0.0425, "clap_knuckle_0"), (0.0425, "clap_knuckle_1")):
        clapstick.visual(
            Cylinder(radius=hinge_radius, length=0.034),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=matte_black,
            name=name,
        )

    marker_clip = model.part("marker_clip")
    marker_clip.visual(
        Box((0.056, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=matte_black,
        name="clip_plate",
    )
    marker_clip.visual(
        Box((0.056, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.009, 0.0085)),
        material=matte_black,
        name="clip_canopy",
    )
    marker_clip.visual(
        Box((0.056, 0.011, 0.0025)),
        origin=Origin(xyz=(0.0, 0.0055, -0.0083)),
        material=matte_black,
        name="clip_shelf",
    )
    marker_clip.visual(
        Box((0.056, 0.003, 0.013)),
        origin=Origin(xyz=(0.0, 0.0165, 0.0025)),
        material=matte_black,
        name="clip_lip",
    )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=5.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "board_to_marker_clip",
        ArticulationType.FIXED,
        parent=board,
        child=marker_clip,
        origin=Origin(xyz=(0.0, board_t / 2.0, board_h / 2.0 - 0.008)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    marker_clip = object_model.get_part("marker_clip")
    hinge = object_model.get_articulation("board_to_clapstick")
    limits = hinge.motion_limits

    ctx.expect_contact(
        marker_clip,
        board,
        elem_a="clip_plate",
        elem_b="board_body",
        name="marker clip is mounted to the board back",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            clapstick,
            board,
            axes="x",
            min_overlap=0.220,
            name="closed clapstick spans the slate width",
        )
        ctx.expect_gap(
            clapstick,
            board,
            axis="z",
            positive_elem="clap_bar",
            negative_elem="board_body",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed clapstick sits just above the board top",
        )

    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="clap_bar")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="clap_bar")
            board_aabb = ctx.part_element_world_aabb(board, elem="board_body")
            ctx.check(
                "clapstick opens forward",
                closed_aabb is not None
                and open_aabb is not None
                and open_aabb[0][1] < closed_aabb[0][1] - 0.020,
                details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
            )
            ctx.check(
                "opened clapstick rises clear above the slate top edge",
                open_aabb is not None
                and board_aabb is not None
                and open_aabb[1][2] > board_aabb[1][2] + 0.015,
                details=f"open_aabb={open_aabb}, board_aabb={board_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
