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
    model = ArticulatedObject(name="freestanding_drafting_station")

    dark_steel = model.material("dark_powder_steel", rgba=(0.045, 0.050, 0.055, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.50, 0.53, 0.55, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    board_face = model.material("warm_drafting_surface", rgba=(0.78, 0.72, 0.60, 1.0))
    board_edge = model.material("dark_edge_banding", rgba=(0.18, 0.13, 0.08, 1.0))
    tray_finish = model.material("tray_gray", rgba=(0.16, 0.18, 0.19, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.34, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="round_foot",
    )
    pedestal.visual(
        Box((0.76, 0.14, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="long_foot",
    )
    pedestal.visual(
        Box((0.14, 0.76, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="cross_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.34, 0.0, 0.006)),
        material=black,
        name="foot_pad_0",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(-0.34, 0.0, 0.006)),
        material=black,
        name="foot_pad_1",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.34, 0.006)),
        material=black,
        name="foot_pad_2",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, -0.34, 0.006)),
        material=black,
        name="foot_pad_3",
    )
    # Four wall sleeve with visible hollow clearance for the moving upright.
    pedestal.visual(
        Box((0.16, 0.025, 0.52)),
        origin=Origin(xyz=(0.0, 0.0675, 0.305)),
        material=satin_steel,
        name="sleeve_front",
    )
    pedestal.visual(
        Box((0.16, 0.025, 0.52)),
        origin=Origin(xyz=(0.0, -0.0675, 0.305)),
        material=satin_steel,
        name="sleeve_rear",
    )
    pedestal.visual(
        Box((0.025, 0.16, 0.52)),
        origin=Origin(xyz=(0.0675, 0.0, 0.305)),
        material=satin_steel,
        name="sleeve_side_0",
    )
    pedestal.visual(
        Box((0.025, 0.16, 0.52)),
        origin=Origin(xyz=(-0.0675, 0.0, 0.305)),
        material=satin_steel,
        name="sleeve_side_1",
    )
    pedestal.visual(
        Box((0.18, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.075, 0.5625)),
        material=dark_steel,
        name="collar_front",
    )
    pedestal.visual(
        Box((0.18, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.075, 0.5625)),
        material=dark_steel,
        name="collar_rear",
    )
    pedestal.visual(
        Box((0.030, 0.18, 0.035)),
        origin=Origin(xyz=(0.075, 0.0, 0.5625)),
        material=dark_steel,
        name="collar_side_0",
    )
    pedestal.visual(
        Box((0.030, 0.18, 0.035)),
        origin=Origin(xyz=(-0.075, 0.0, 0.5625)),
        material=dark_steel,
        name="collar_side_1",
    )

    upright = model.part("upright")
    upright.visual(
        Box((0.070, 0.070, 1.20)),
        # The lower part of the mast remains inside the pedestal sleeve at rest.
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=satin_steel,
        name="inner_mast",
    )
    upright.visual(
        Box((0.020, 0.055, 0.080)),
        origin=Origin(xyz=(0.045, 0.0, -0.420)),
        material=black,
        name="mast_glide_0",
    )
    upright.visual(
        Box((0.020, 0.055, 0.080)),
        origin=Origin(xyz=(-0.045, 0.0, -0.420)),
        material=black,
        name="mast_glide_1",
    )
    upright.visual(
        Box((0.055, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, 0.045, -0.420)),
        material=black,
        name="mast_glide_2",
    )
    upright.visual(
        Box((0.055, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, -0.045, -0.420)),
        material=black,
        name="mast_glide_3",
    )
    upright.visual(
        Box((0.080, 0.14, 0.11)),
        origin=Origin(xyz=(-0.080, 0.0, 0.735)),
        material=dark_steel,
        name="mast_head",
    )
    upright.visual(
        Box((0.050, 0.070, 0.080)),
        origin=Origin(xyz=(-0.035, 0.0, 0.690)),
        material=dark_steel,
        name="head_web",
    )
    upright.visual(
        Box((0.08, 1.30, 0.065)),
        origin=Origin(xyz=(-0.080, 0.0, 0.700)),
        material=dark_steel,
        name="cradle_bridge",
    )
    upright.visual(
        Box((0.105, 0.080, 0.240)),
        origin=Origin(xyz=(0.0, 0.660, 0.780)),
        material=dark_steel,
        name="cradle_cheek_0",
    )
    upright.visual(
        Box((0.105, 0.080, 0.240)),
        origin=Origin(xyz=(0.0, -0.660, 0.780)),
        material=dark_steel,
        name="cradle_cheek_1",
    )

    board = model.part("board")
    board.visual(
        Box((0.86, 1.16, 0.060)),
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
        material=board_face,
        name="board_slab",
    )
    board.visual(
        Box((0.040, 1.20, 0.070)),
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
        material=board_edge,
        name="rear_banding",
    )
    board.visual(
        Box((0.045, 1.20, 0.060)),
        origin=Origin(xyz=(0.8525, 0.0, 0.030)),
        material=board_edge,
        name="front_lip",
    )
    board.visual(
        Box((0.86, 0.035, 0.055)),
        origin=Origin(xyz=(0.43, 0.5975, 0.0275)),
        material=board_edge,
        name="side_lip_0",
    )
    board.visual(
        Box((0.86, 0.035, 0.055)),
        origin=Origin(xyz=(0.43, -0.5975, 0.0275)),
        material=board_edge,
        name="side_lip_1",
    )
    board.visual(
        Cylinder(radius=0.035, length=1.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="trunnion_pin",
    )
    # Fixed short runners under the side edge for the shallow tray drawer.
    board.visual(
        Box((0.028, 0.340, 0.025)),
        origin=Origin(xyz=(0.34, 0.620, -0.0425)),
        material=dark_steel,
        name="tray_runner_0",
    )
    board.visual(
        Box((0.028, 0.340, 0.025)),
        origin=Origin(xyz=(0.62, 0.620, -0.0425)),
        material=dark_steel,
        name="tray_runner_1",
    )

    side_tray = model.part("side_tray")
    side_tray.visual(
        Box((0.46, 0.22, 0.016)),
        origin=Origin(xyz=(0.48, 0.10, -0.132)),
        material=tray_finish,
        name="tray_bottom",
    )
    side_tray.visual(
        Box((0.46, 0.018, 0.055)),
        origin=Origin(xyz=(0.48, -0.009, -0.0965)),
        material=tray_finish,
        name="inner_wall",
    )
    side_tray.visual(
        Box((0.46, 0.018, 0.055)),
        origin=Origin(xyz=(0.48, 0.209, -0.0965)),
        material=tray_finish,
        name="outer_wall",
    )
    side_tray.visual(
        Box((0.018, 0.22, 0.055)),
        origin=Origin(xyz=(0.241, 0.10, -0.0965)),
        material=tray_finish,
        name="end_wall_0",
    )
    side_tray.visual(
        Box((0.018, 0.22, 0.055)),
        origin=Origin(xyz=(0.719, 0.10, -0.0965)),
        material=tray_finish,
        name="end_wall_1",
    )
    side_tray.visual(
        Box((0.022, 0.310, 0.018)),
        origin=Origin(xyz=(0.34, 0.005, -0.064)),
        material=satin_steel,
        name="tray_slide_0",
    )
    side_tray.visual(
        Box((0.022, 0.310, 0.018)),
        origin=Origin(xyz=(0.62, 0.005, -0.064)),
        material=satin_steel,
        name="tray_slide_1",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=upright,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.34),
    )
    model.articulation(
        "board_trunnion",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.6, lower=-0.25, upper=1.10),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=board,
        child=side_tray,
        origin=Origin(xyz=(0.0, 0.600, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upright = object_model.get_part("upright")
    board = object_model.get_part("board")
    side_tray = object_model.get_part("side_tray")
    mast_slide = object_model.get_articulation("mast_slide")
    board_trunnion = object_model.get_articulation("board_trunnion")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.expect_contact(
        upright,
        pedestal,
        elem_a="mast_glide_0",
        elem_b="sleeve_side_0",
        name="upright glide bears on sleeve",
    )
    ctx.expect_overlap(
        upright,
        pedestal,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_side_0",
        min_overlap=0.35,
        name="collapsed mast remains deeply sleeved",
    )
    rest_upright_pos = ctx.part_world_position(upright)
    with ctx.pose({mast_slide: 0.34}):
        ctx.expect_contact(
            upright,
            pedestal,
            elem_a="mast_glide_0",
            elem_b="sleeve_side_0",
            name="extended upright still guided",
        )
        ctx.expect_overlap(
            upright,
            pedestal,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_side_0",
            min_overlap=0.12,
            name="extended mast retains insertion",
        )
        raised_upright_pos = ctx.part_world_position(upright)
    ctx.check(
        "upright slides upward",
        rest_upright_pos is not None
        and raised_upright_pos is not None
        and raised_upright_pos[2] > rest_upright_pos[2] + 0.30,
        details=f"rest={rest_upright_pos}, raised={raised_upright_pos}",
    )

    ctx.expect_contact(
        board,
        upright,
        elem_a="trunnion_pin",
        elem_b="cradle_cheek_0",
        name="trunnion pin seated in cradle",
    )
    rest_front = ctx.part_element_world_aabb(board, elem="front_lip")
    with ctx.pose({board_trunnion: 1.0}):
        tilted_front = ctx.part_element_world_aabb(board, elem="front_lip")
    ctx.check(
        "board rotates upward on horizontal trunnion",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[1][2] > rest_front[1][2] + 0.45,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    ctx.expect_contact(
        side_tray,
        board,
        elem_a="tray_slide_0",
        elem_b="tray_runner_0",
        name="tray slide rests on runner",
    )
    ctx.expect_overlap(
        side_tray,
        board,
        axes="y",
        elem_a="tray_slide_0",
        elem_b="tray_runner_0",
        min_overlap=0.25,
        name="closed tray runner engagement",
    )
    rest_tray_pos = ctx.part_world_position(side_tray)
    with ctx.pose({tray_slide: 0.22}):
        ctx.expect_contact(
            side_tray,
            board,
            elem_a="tray_slide_0",
            elem_b="tray_runner_0",
            name="extended tray still on runner",
        )
        ctx.expect_overlap(
            side_tray,
            board,
            axes="y",
            elem_a="tray_slide_0",
            elem_b="tray_runner_0",
            min_overlap=0.10,
            name="extended tray keeps runner engagement",
        )
        extended_tray_pos = ctx.part_world_position(side_tray)
    ctx.check(
        "side tray slides outward",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > rest_tray_pos[1] + 0.20,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
