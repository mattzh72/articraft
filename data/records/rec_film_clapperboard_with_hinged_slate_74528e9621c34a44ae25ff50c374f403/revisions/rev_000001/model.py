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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insert_clapperboard")

    frame_black = model.material("frame_black", rgba=(0.08, 0.08, 0.08, 1.0))
    slate_white = model.material("slate_white", rgba=(0.94, 0.94, 0.90, 1.0))
    print_grey = model.material("print_grey", rgba=(0.60, 0.62, 0.64, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    board_w = 0.150
    board_h = 0.098
    board_t = 0.006
    face_w = 0.132
    face_h = 0.080
    face_t = 0.0018
    rail_w = 0.144
    rail_h = 0.008
    rail_t = 0.009

    hinge_axis_z = 0.1055
    stick_len = 0.154
    stick_h = 0.022
    stick_t = 0.007
    stick_front_offset = 0.0080

    board = model.part("board")
    board.visual(
        Box((board_w, board_t, board_h)),
        origin=Origin(xyz=(0.0, 0.0, board_h / 2.0)),
        material=frame_black,
        name="board_body",
    )
    board.visual(
        Box((face_w, face_t, face_h)),
        origin=Origin(xyz=(0.0, board_t / 2.0 - face_t / 2.0 + 0.0004, 0.047)),
        material=slate_white,
        name="writing_surface",
    )
    for idx, z in enumerate((0.061, 0.047, 0.033)):
        board.visual(
            Box((0.114, 0.0009, 0.0012)),
            origin=Origin(xyz=(0.0, board_t / 2.0 - 0.00025, z)),
            material=print_grey,
            name=f"writing_line_{idx}",
        )
    board.visual(
        Box((rail_w, rail_t, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, board_h + rail_h / 2.0 - 0.001)),
        material=frame_black,
        name="hinge_rail",
    )
    board.visual(
        Cylinder(radius=0.0016, length=board_w),
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="hinge_rod",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((stick_len, stick_h, stick_t)),
        origin=Origin(xyz=(0.0, stick_h / 2.0, -stick_front_offset)),
        material=slate_white,
        name="clapstick_body",
    )
    clapstick.visual(
        Box((stick_len * 0.94, 0.004, 0.0032)),
        origin=Origin(xyz=(0.0, 0.002, -stick_front_offset + 0.0015)),
        material=hinge_metal,
        name="hinge_band",
    )
    for idx, x in enumerate((-0.043, 0.043)):
        clapstick.visual(
            Box((0.012, 0.003, 0.003)),
            origin=Origin(xyz=(x, 0.001, -0.0045)),
            material=hinge_metal,
            name=f"hinge_foot_{idx}",
        )
    for idx, x in enumerate((-0.056, -0.027, 0.002, 0.031, 0.060)):
        clapstick.visual(
            Box((0.022, 0.008, 0.0024)),
            origin=Origin(
                xyz=(x, 0.012, -stick_front_offset - 0.0022),
                rpy=(0.0, 0.0, 0.80),
            ),
            material=frame_black,
            name=f"stripe_{idx}",
        )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("board_to_clapstick")
    limits = hinge.motion_limits

    ctx.expect_overlap(
        clapstick,
        board,
        axes="x",
        elem_a="clapstick_body",
        elem_b="writing_surface",
        min_overlap=0.128,
        name="clapstick spans the narrow board width",
    )
    ctx.expect_gap(
        clapstick,
        board,
        axis="z",
        positive_elem="clapstick_body",
        negative_elem="hinge_rail",
        min_gap=0.0002,
        max_gap=0.0025,
        name="closed clapstick sits just above the top hinge rail",
    )

    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="clapstick_body")
        with ctx.pose({hinge: limits.upper}):
            opened_aabb = ctx.part_element_world_aabb(clapstick, elem="clapstick_body")
            ctx.expect_overlap(
                clapstick,
                board,
                axes="x",
                elem_a="clapstick_body",
                elem_b="hinge_rail",
                min_overlap=0.130,
                name="open clapstick stays aligned with the board rail width",
            )
        if closed_aabb is not None and opened_aabb is not None:
            closed_front = closed_aabb[0][1]
            opened_front = opened_aabb[0][1]
            ctx.check(
                "clapstick opens backward from the board face",
                opened_front < closed_front - 0.004,
                details=f"closed_min_y={closed_front}, opened_min_y={opened_front}",
            )
        else:
            ctx.fail("clapstick AABB available", f"closed={closed_aabb}, opened={opened_aabb}")

    return ctx.report()


object_model = build_object_model()
