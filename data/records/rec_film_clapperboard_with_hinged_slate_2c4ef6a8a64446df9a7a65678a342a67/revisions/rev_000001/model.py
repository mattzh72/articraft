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
    model = ArticulatedObject(name="cinema_clapperboard")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.08, 1.0))
    deep_black = model.material("deep_black", rgba=(0.03, 0.03, 0.03, 1.0))
    white_paint = model.material("white_paint", rgba=(0.95, 0.95, 0.93, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.14, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.18, 0.18, 0.18, 1.0))

    slate_width = 0.320
    slate_height = 0.230
    slate_thickness = 0.014

    tray_width = 0.286
    tray_mount_depth = 0.008
    tray_mount_height = 0.018
    tray_floor_depth = 0.024
    tray_floor_height = 0.004
    tray_lip_depth = 0.004
    tray_lip_height = 0.012

    clap_width = 0.326
    clap_height = 0.052
    clap_thickness = 0.018
    hinge_radius = 0.006
    hinge_rail_depth = 0.012
    hinge_rail_height = 0.010
    hinge_y = 0.017
    top_z = slate_height / 2.0
    bottom_z = -slate_height / 2.0

    board = model.part("board")
    board.visual(
        Box((slate_width, slate_thickness, slate_height)),
        origin=Origin(),
        material=matte_black,
        name="slate_panel",
    )
    board.visual(
        Box((0.252, 0.0015, 0.070)),
        origin=Origin(xyz=(0.0, slate_thickness / 2.0 + 0.0004, -0.016)),
        material=charcoal,
        name="writing_field",
    )
    board.visual(
        Box((tray_width, tray_mount_depth, tray_mount_height)),
        origin=Origin(xyz=(0.0, 0.001, bottom_z - 0.007)),
        material=tray_gray,
        name="tray_mount",
    )
    board.visual(
        Box((tray_width, tray_floor_depth, tray_floor_height)),
        origin=Origin(xyz=(0.0, 0.010, bottom_z - 0.016)),
        material=tray_gray,
        name="tray_floor",
    )
    board.visual(
        Box((tray_width, tray_lip_depth, tray_lip_height)),
        origin=Origin(xyz=(0.0, 0.020, bottom_z - 0.010)),
        material=tray_gray,
        name="tray_lip",
    )
    board.visual(
        Box((0.300, hinge_rail_depth, hinge_rail_height)),
        origin=Origin(xyz=(0.0, 0.011, top_z - 0.012)),
        material=charcoal,
        name="hinge_rail",
    )
    board.visual(
        Cylinder(radius=hinge_radius, length=0.115),
        origin=Origin(xyz=(-0.095, hinge_y, top_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=deep_black,
        name="hinge_barrel_0",
    )
    board.visual(
        Box((0.115, 0.007, 0.008)),
        origin=Origin(xyz=(-0.095, 0.0145, top_z - 0.006)),
        material=charcoal,
        name="hinge_tab_0",
    )
    board.visual(
        Cylinder(radius=hinge_radius, length=0.115),
        origin=Origin(xyz=(0.095, hinge_y, top_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=deep_black,
        name="hinge_barrel_1",
    )
    board.visual(
        Box((0.115, 0.007, 0.008)),
        origin=Origin(xyz=(0.095, 0.0145, top_z - 0.006)),
        material=charcoal,
        name="hinge_tab_1",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((clap_width, clap_thickness, clap_height)),
        origin=Origin(xyz=(0.0, hinge_y, -0.031)),
        material=deep_black,
        name="clap_core",
    )
    clapstick.visual(
        Cylinder(radius=hinge_radius, length=0.075),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=deep_black,
        name="clap_barrel",
    )
    clapstick.visual(
        Box((0.075, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.005)),
        material=deep_black,
        name="hinge_cheek",
    )

    stripe_centers = (-0.132, -0.078, -0.024, 0.030, 0.084, 0.138)
    for index, x_pos in enumerate(stripe_centers):
        clapstick.visual(
            Box((0.010, 0.002, 0.055)),
            origin=Origin(xyz=(x_pos, 0.025, -0.032), rpy=(0.0, 0.55, 0.0)),
            material=white_paint,
            name=f"stripe_{index}",
        )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, hinge_y, top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=5.0,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("board_to_clapstick")

    ctx.expect_overlap(
        clapstick,
        board,
        axes="x",
        min_overlap=0.300,
        elem_a="clap_core",
        elem_b="slate_panel",
        name="clapstick spans nearly the full board width",
    )

    board_panel_aabb = ctx.part_element_world_aabb(board, elem="slate_panel")
    tray_floor_aabb = ctx.part_element_world_aabb(board, elem="tray_floor")
    tray_lip_aabb = ctx.part_element_world_aabb(board, elem="tray_lip")
    tray_is_below = (
        board_panel_aabb is not None
        and tray_floor_aabb is not None
        and tray_floor_aabb[1][2] < board_panel_aabb[0][2] + 0.002
    )
    tray_projects_forward = (
        board_panel_aabb is not None
        and tray_lip_aabb is not None
        and tray_lip_aabb[1][1] > board_panel_aabb[1][1] + 0.010
    )
    ctx.check(
        "chalk tray stays mounted below the slate and projects forward",
        tray_is_below and tray_projects_forward,
        details=(
            f"panel_aabb={board_panel_aabb}, tray_floor_aabb={tray_floor_aabb}, "
            f"tray_lip_aabb={tray_lip_aabb}"
        ),
    )

    limits = hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.upper}):
            ctx.expect_gap(
                clapstick,
                board,
                axis="z",
                positive_elem="clap_core",
                negative_elem="slate_panel",
                min_gap=0.008,
                name="opened clapstick lifts clearly above the slate",
            )

    return ctx.report()


object_model = build_object_model()
