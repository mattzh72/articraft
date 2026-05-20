from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BOARD_W = 0.285
BOARD_H = 0.225
BOARD_T = 0.010
FRAME_MARGIN = 0.014
TOP_RAIL_H = 0.018
TOP_RAIL_D = 0.014

CLAP_W = 0.305
CLAP_H = 0.033
CLAP_T = 0.012
CLAP_BEVEL = 0.018
HINGE_R = 0.004
HINGE_GAP = 0.001

TRAY_W = 0.255
TRAY_D = 0.024
TRAY_FLOOR_T = 0.004
TRAY_WALL_T = 0.004
TRAY_LIP_H = 0.008
TRAY_SIDE_T = 0.006


def _clapstick_body_mesh():
    half_w = CLAP_W / 2.0
    profile = [
        (-half_w, HINGE_R),
        (half_w, HINGE_R),
        (half_w, HINGE_R + CLAP_H - CLAP_BEVEL),
        (half_w - CLAP_BEVEL, HINGE_R + CLAP_H),
        (-half_w + CLAP_BEVEL, HINGE_R + CLAP_H),
        (-half_w, HINGE_R + CLAP_H - CLAP_BEVEL),
    ]
    shape = cq.Workplane("XZ").polyline(profile).close().extrude(CLAP_T)
    return mesh_from_cadquery(shape, "clapstick_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dry_erase_camera_slate")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    dry_erase_white = model.material("dry_erase_white", rgba=(0.96, 0.96, 0.94, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.98, 0.98, 0.98, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_W, BOARD_T, BOARD_H)),
        material=matte_black,
        name="backer",
    )
    board.visual(
        Box((BOARD_W - 2.0 * FRAME_MARGIN, BOARD_T * 0.36, BOARD_H - 2.0 * FRAME_MARGIN - 0.022)),
        origin=Origin(xyz=(0.0, BOARD_T * 0.18, -0.005)),
        material=dry_erase_white,
        name="write_face",
    )
    board.visual(
        Box((BOARD_W, TOP_RAIL_D, TOP_RAIL_H)),
        origin=Origin(
            xyz=(
                0.0,
                BOARD_T / 2.0 + TOP_RAIL_D / 2.0 - 0.001,
                BOARD_H / 2.0 - TOP_RAIL_H / 2.0,
            )
        ),
        material=satin_black,
        name="top_rail",
    )

    hinge_axis_y = BOARD_T / 2.0 + TOP_RAIL_D - 0.001
    hinge_axis_z = BOARD_H / 2.0 - (HINGE_R - HINGE_GAP)
    hinge_segments = (
        (-0.118, 0.044, "hinge_barrel_0"),
        (0.0, 0.070, "hinge_barrel_1"),
        (0.118, 0.044, "hinge_barrel_2"),
    )
    for x, length, name in hinge_segments:
        board.visual(
            Cylinder(radius=HINGE_R, length=length),
            origin=Origin(xyz=(x, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_black,
            name=name,
        )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_WALL_T, 0.014)),
        origin=Origin(xyz=(0.0, TRAY_WALL_T / 2.0, 0.007)),
        material=satin_black,
        name="back_wall",
    )
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_FLOOR_T)),
        origin=Origin(xyz=(0.0, TRAY_D / 2.0, -TRAY_FLOOR_T / 2.0)),
        material=satin_black,
        name="floor",
    )
    tray.visual(
        Box((TRAY_W, TRAY_WALL_T, TRAY_LIP_H)),
        origin=Origin(xyz=(0.0, TRAY_D - TRAY_WALL_T / 2.0, TRAY_LIP_H / 2.0)),
        material=satin_black,
        name="front_lip",
    )
    for x, name in (
        (TRAY_W / 2.0 - TRAY_SIDE_T / 2.0, "side_wall_0"),
        (-TRAY_W / 2.0 + TRAY_SIDE_T / 2.0, "side_wall_1"),
    ):
        tray.visual(
            Box((TRAY_SIDE_T, TRAY_D, 0.014)),
            origin=Origin(xyz=(x, TRAY_D / 2.0, 0.007)),
            material=satin_black,
            name=name,
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        _clapstick_body_mesh(),
        origin=Origin(xyz=(0.0, -CLAP_T, 0.0)),
        material=matte_black,
        name="stick_body",
    )
    clapstick_barrels = (
        (-0.067, 0.052, "stick_barrel_0"),
        (0.067, 0.052, "stick_barrel_1"),
    )
    for x, length, name in clapstick_barrels:
        clapstick.visual(
            Cylinder(radius=HINGE_R, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_black,
            name=name,
        )

    stripe_angle = math.radians(36.0)
    stripe_centers = (-0.092, -0.034, 0.024, 0.082)
    for index, x in enumerate(stripe_centers):
        clapstick.visual(
            Box((0.060, CLAP_T + 0.001, 0.006)),
            origin=Origin(
                xyz=(x, -CLAP_T / 2.0, HINGE_R + CLAP_H * 0.58),
                rpy=(0.0, stripe_angle, 0.0),
            ),
            material=stripe_white,
            name=f"stripe_{index}",
        )

    model.articulation(
        "board_to_tray",
        ArticulationType.FIXED,
        parent=board,
        child=tray,
        origin=Origin(xyz=(0.0, BOARD_T / 2.0, -BOARD_H / 2.0)),
    )
    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    tray = object_model.get_part("tray")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("board_to_clapstick")
    limits = hinge.motion_limits

    ctx.expect_contact(
        tray,
        board,
        elem_a="back_wall",
        elem_b="backer",
        name="tray back wall mounts to board",
    )
    ctx.expect_overlap(
        tray,
        board,
        axes="x",
        min_overlap=0.22,
        elem_a="floor",
        elem_b="backer",
        name="tray spans most of board width",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            clapstick,
            board,
            axis="z",
            positive_elem="stick_body",
            negative_elem="top_rail",
            min_gap=0.0005,
            max_gap=0.0045,
            name="closed clapstick sits just above top rail",
        )
        ctx.expect_overlap(
            clapstick,
            board,
            axes="x",
            elem_a="stick_body",
            elem_b="top_rail",
            min_overlap=0.26,
            name="closed clapstick tracks board width",
        )

    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
            ctx.expect_gap(
                clapstick,
                board,
                axis="z",
                positive_elem="stick_body",
                negative_elem="top_rail",
                min_gap=0.008,
                name="opened clapstick lifts clear of the rail",
            )

        ctx.check(
            "clapstick opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] > closed_aabb[0][2] + 0.008,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
