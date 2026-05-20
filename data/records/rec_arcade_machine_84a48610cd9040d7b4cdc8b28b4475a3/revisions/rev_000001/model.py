from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OVERALL_LENGTH = 0.92
OVERALL_WIDTH = 0.64

BODY_LENGTH = 0.86
BODY_WIDTH = 0.58
BODY_HEIGHT = 0.685
BODY_WALL = 0.022
BOTTOM_THICK = 0.03
PLINTH_LENGTH = 0.84
PLINTH_WIDTH = 0.56
PLINTH_HEIGHT = 0.055

TOP_FRAME_THICK = 0.035
FRAME_OPEN_LENGTH = 0.64
FRAME_OPEN_WIDTH = 0.42
GLASS_LENGTH = 0.628
GLASS_WIDTH = 0.408
GLASS_THICK = 0.006
GLASS_LIP_THICK = 0.004

SCREEN_WELL_OUTER_LENGTH = 0.60
SCREEN_WELL_OUTER_WIDTH = 0.38
SCREEN_WELL_DEPTH = 0.105
SCREEN_WELL_WALL = 0.016
SCREEN_WELL_FLOOR = 0.014
SCREEN_BEZEL_LENGTH = 0.51
SCREEN_BEZEL_WIDTH = 0.33
SCREEN_BEZEL_THICK = 0.008
SCREEN_ACTIVE_LENGTH = 0.46
SCREEN_ACTIVE_WIDTH = 0.28
SCREEN_ACTIVE_THICK = 0.004

END_PANEL_THICK = 0.018
END_PANEL_WIDTH = 0.532
END_PANEL_HEIGHT = 0.56
END_PANEL_BOTTOM_Z = 0.07
END_PANEL_RAIL_LENGTH = 0.012

DOOR_OPEN_WIDTH = 0.264
DOOR_OPEN_HEIGHT = 0.214
DOOR_WIDTH = 0.256
DOOR_HEIGHT = 0.206
DOOR_THICK = 0.014
DOOR_CENTER_Y = 0.0
DOOR_BOTTOM_Z = -0.18

COIN_PLATE_WIDTH = 0.18
COIN_PLATE_HEIGHT = 0.085
COIN_PLATE_CENTER_Y = 0.02
COIN_PLATE_CENTER_Z = 0.165
SLOT_LENGTH = 0.058
SLOT_HEIGHT = 0.006
SLOT_CENTER_Y = 0.07
SLOT_CENTER_Z = 0.172
FLAP_OPEN_WIDTH = 0.062
FLAP_OPEN_HEIGHT = 0.036
FLAP_WIDTH = 0.058
FLAP_HEIGHT = 0.032
FLAP_THICK = 0.01
FLAP_CENTER_Y = -0.03
FLAP_CENTER_Z = 0.165


def _body_shape() -> cq.Workplane:
    main_height = BODY_HEIGHT - PLINTH_HEIGHT
    main_shell = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, main_height).translate(
        (0.0, 0.0, PLINTH_HEIGHT + main_height / 2.0)
    )
    plinth = cq.Workplane("XY").box(PLINTH_LENGTH, PLINTH_WIDTH, PLINTH_HEIGHT).translate(
        (0.0, 0.0, PLINTH_HEIGHT / 2.0)
    )
    cavity = cq.Workplane("XY").box(
        BODY_LENGTH - 2.0 * BODY_WALL,
        BODY_WIDTH - 2.0 * BODY_WALL,
        BODY_HEIGHT,
    ).translate((0.0, 0.0, BOTTOM_THICK + BODY_HEIGHT / 2.0))
    return main_shell.union(plinth).cut(cavity)


def _top_frame_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(OVERALL_LENGTH, OVERALL_WIDTH, TOP_FRAME_THICK)
    opening = cq.Workplane("XY").box(
        FRAME_OPEN_LENGTH,
        FRAME_OPEN_WIDTH,
        TOP_FRAME_THICK + 0.002,
    )
    lip_outer = cq.Workplane("XY").box(
        FRAME_OPEN_LENGTH,
        FRAME_OPEN_WIDTH,
        GLASS_LIP_THICK,
    )
    lip_inner = cq.Workplane("XY").box(
        GLASS_LENGTH - 0.006,
        GLASS_WIDTH - 0.006,
        GLASS_LIP_THICK + 0.002,
    )
    lip = lip_outer.cut(lip_inner).translate(
        (
            0.0,
            0.0,
            TOP_FRAME_THICK / 2.0 - GLASS_THICK - GLASS_LIP_THICK / 2.0 - 0.001,
        )
    )
    return outer.cut(opening).union(lip)


def _screen_well_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        SCREEN_WELL_OUTER_LENGTH,
        SCREEN_WELL_OUTER_WIDTH,
        SCREEN_WELL_DEPTH,
    )
    inner = cq.Workplane("XY").box(
        SCREEN_WELL_OUTER_LENGTH - 2.0 * SCREEN_WELL_WALL,
        SCREEN_WELL_OUTER_WIDTH - 2.0 * SCREEN_WELL_WALL,
        SCREEN_WELL_DEPTH,
    ).translate((0.0, 0.0, SCREEN_WELL_FLOOR))
    return outer.cut(inner)


def _end_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(END_PANEL_THICK, END_PANEL_WIDTH, END_PANEL_HEIGHT)
    coin_plate = cq.Workplane("XY").box(0.004, COIN_PLATE_WIDTH, COIN_PLATE_HEIGHT).translate(
        (
            END_PANEL_THICK / 2.0 + 0.002,
            COIN_PLATE_CENTER_Y,
            COIN_PLATE_CENTER_Z,
        )
    )
    slot_cut = cq.Workplane("XY").box(
        END_PANEL_THICK + 0.008,
        SLOT_LENGTH,
        SLOT_HEIGHT,
    ).translate((0.0, SLOT_CENTER_Y, SLOT_CENTER_Z))
    return panel.union(coin_plate).cut(slot_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cocktail_table_arcade")

    wood_finish = model.material("wood_finish", rgba=(0.31, 0.19, 0.10, 1.0))
    black_finish = model.material("black_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal_finish = model.material("charcoal_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.68, 0.69, 0.72, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.08, 0.24, 0.58, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.54, 0.70, 0.76, 0.35))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        mesh_from_cadquery(_body_shape(), "cocktail_arcade_body"),
        material=wood_finish,
        name="cabinet_shell",
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        mesh_from_cadquery(_top_frame_shape(), "cocktail_arcade_top_frame"),
        material=black_finish,
        name="frame_ring",
    )
    top_frame.visual(
        Box((GLASS_LENGTH, GLASS_WIDTH, GLASS_THICK)),
        origin=Origin(xyz=(0.0, 0.0, TOP_FRAME_THICK / 2.0 - GLASS_THICK / 2.0 - 0.001)),
        material=glass_finish,
        name="glass_panel",
    )

    screen_well = model.part("screen_well")
    screen_well.visual(
        mesh_from_cadquery(_screen_well_shape(), "cocktail_arcade_screen_well"),
        material=charcoal_finish,
        name="well_shell",
    )
    screen_well.visual(
        Box((SCREEN_WELL_OUTER_LENGTH, 0.04, 0.006)),
        origin=Origin(xyz=(0.0, FRAME_OPEN_WIDTH / 2.0, SCREEN_WELL_DEPTH / 2.0 - 0.003)),
        material=charcoal_finish,
        name="support_strip_front",
    )
    screen_well.visual(
        Box((SCREEN_WELL_OUTER_LENGTH, 0.04, 0.006)),
        origin=Origin(
            xyz=(0.0, -FRAME_OPEN_WIDTH / 2.0, SCREEN_WELL_DEPTH / 2.0 - 0.003)
        ),
        material=charcoal_finish,
        name="support_strip_rear",
    )
    screen_well.visual(
        Box((0.04, SCREEN_WELL_OUTER_WIDTH, 0.006)),
        origin=Origin(xyz=(FRAME_OPEN_LENGTH / 2.0, 0.0, SCREEN_WELL_DEPTH / 2.0 - 0.003)),
        material=charcoal_finish,
        name="support_strip_side_0",
    )
    screen_well.visual(
        Box((0.04, SCREEN_WELL_OUTER_WIDTH, 0.006)),
        origin=Origin(
            xyz=(-FRAME_OPEN_LENGTH / 2.0, 0.0, SCREEN_WELL_DEPTH / 2.0 - 0.003)
        ),
        material=charcoal_finish,
        name="support_strip_side_1",
    )
    screen_well.visual(
        Box((SCREEN_BEZEL_LENGTH, SCREEN_BEZEL_WIDTH, SCREEN_BEZEL_THICK)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -SCREEN_WELL_DEPTH / 2.0 + SCREEN_WELL_FLOOR + SCREEN_BEZEL_THICK / 2.0,
            )
        ),
        material=black_finish,
        name="screen_bezel",
    )
    screen_well.visual(
        Box((SCREEN_ACTIVE_LENGTH, SCREEN_ACTIVE_WIDTH, SCREEN_ACTIVE_THICK)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -SCREEN_WELL_DEPTH / 2.0
                + SCREEN_WELL_FLOOR
                + SCREEN_BEZEL_THICK
                + SCREEN_ACTIVE_THICK / 2.0,
            )
        ),
        material=screen_finish,
        name="screen_panel",
    )

    end_panel = model.part("end_panel")
    end_panel.visual(
        mesh_from_cadquery(_end_panel_shape(), "cocktail_arcade_end_panel"),
        material=charcoal_finish,
        name="panel_face",
    )
    for index, rail_y in enumerate(
        (
            -(END_PANEL_WIDTH / 2.0 - 0.03),
            END_PANEL_WIDTH / 2.0 - 0.03,
        )
    ):
        end_panel.visual(
            Box((END_PANEL_RAIL_LENGTH, 0.04, END_PANEL_HEIGHT - 0.10)),
            origin=Origin(
                xyz=(
                    -END_PANEL_THICK / 2.0 - END_PANEL_RAIL_LENGTH / 2.0,
                    rail_y,
                    0.0,
                )
            ),
            material=charcoal_finish,
            name=f"mount_rail_{index}",
        )

    cashbox_door = model.part("cashbox_door")
    cashbox_door.visual(
        Box((DOOR_THICK, DOOR_WIDTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, -DOOR_WIDTH / 2.0, DOOR_HEIGHT / 2.0)),
        material=black_finish,
        name="door_panel",
    )
    cashbox_door.visual(
        Box((0.01, 0.018, 0.06)),
        origin=Origin(xyz=(DOOR_THICK / 2.0 + 0.005, -DOOR_WIDTH + 0.03, 0.12)),
        material=metal_finish,
        name="door_pull",
    )

    coin_flap = model.part("coin_flap")
    coin_flap.visual(
        Box((FLAP_THICK, FLAP_WIDTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FLAP_HEIGHT / 2.0)),
        material=metal_finish,
        name="flap_panel",
    )
    coin_flap.visual(
        Box((0.006, FLAP_WIDTH * 0.7, 0.006)),
        origin=Origin(
            xyz=(
                FLAP_THICK / 2.0 + 0.003,
                0.0,
                -FLAP_HEIGHT + 0.003,
            )
        ),
        material=metal_finish,
        name="flap_lip",
    )

    model.articulation(
        "body_to_top_frame",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=top_frame,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + TOP_FRAME_THICK / 2.0)),
    )
    model.articulation(
        "top_frame_to_screen_well",
        ArticulationType.FIXED,
        parent=top_frame,
        child=screen_well,
        origin=Origin(
            xyz=(0.0, 0.0, -TOP_FRAME_THICK / 2.0 - SCREEN_WELL_DEPTH / 2.0)
        ),
    )
    model.articulation(
        "body_to_end_panel",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=end_panel,
        origin=Origin(
            xyz=(
                BODY_LENGTH / 2.0 + END_PANEL_RAIL_LENGTH + END_PANEL_THICK / 2.0,
                0.0,
                END_PANEL_BOTTOM_Z + END_PANEL_HEIGHT / 2.0,
            )
        ),
    )
    model.articulation(
        "end_panel_to_cashbox_door",
        ArticulationType.REVOLUTE,
        parent=end_panel,
        child=cashbox_door,
        origin=Origin(
            xyz=(
                END_PANEL_THICK / 2.0 + DOOR_THICK / 2.0,
                DOOR_CENTER_Y + DOOR_WIDTH / 2.0,
                DOOR_BOTTOM_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(100.0),
            effort=12.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "end_panel_to_coin_flap",
        ArticulationType.REVOLUTE,
        parent=end_panel,
        child=coin_flap,
        origin=Origin(
            xyz=(
                END_PANEL_THICK / 2.0 + FLAP_THICK / 2.0,
                FLAP_CENTER_Y,
                FLAP_CENTER_Z + FLAP_HEIGHT / 2.0,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(55.0),
            effort=2.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top_frame = object_model.get_part("top_frame")
    end_panel = object_model.get_part("end_panel")
    cashbox_door = object_model.get_part("cashbox_door")
    coin_flap = object_model.get_part("coin_flap")
    door_hinge = object_model.get_articulation("end_panel_to_cashbox_door")
    flap_hinge = object_model.get_articulation("end_panel_to_coin_flap")

    ctx.expect_gap(
        top_frame,
        end_panel,
        axis="z",
        min_gap=0.04,
        name="service end panel stays visibly separate from top frame",
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_overlap(
            cashbox_door,
            end_panel,
            axes="yz",
            min_overlap=0.18,
            name="cashbox door sits within the service end footprint",
        )
        ctx.expect_overlap(
            coin_flap,
            end_panel,
            axes="yz",
            min_overlap=0.025,
            name="coin return flap sits on the service end panel",
        )
        closed_door_aabb = ctx.part_element_world_aabb(cashbox_door, elem="door_panel")
        closed_flap_aabb = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")

    open_door_aabb = None
    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper, flap_hinge: 0.0}):
            open_door_aabb = ctx.part_element_world_aabb(cashbox_door, elem="door_panel")

    ctx.check(
        "cashbox door swings outward on a vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.08,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    open_flap_aabb = None
    flap_limits = flap_hinge.motion_limits
    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({door_hinge: 0.0, flap_hinge: flap_limits.upper}):
            open_flap_aabb = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")

    ctx.check(
        "coin return flap tips outward from its short horizontal hinge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.012,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
