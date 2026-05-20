from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_W = 0.64
CABINET_D = 0.68
CABINET_H = 0.92
TOP_DECK_T = 0.026
SHELL_H = CABINET_H - TOP_DECK_T
WALL_T = 0.018
BOTTOM_T = 0.022

OPENING_X = 0.07
OPENING_Y = -0.02
OPENING_W = 0.40
OPENING_D = 0.49

LID_W = 0.44
LID_D = 0.53
LID_T = 0.032

CLUSTER_W = 0.160
CLUSTER_D = 0.148
CLUSTER_H = 0.092
CLUSTER_REAR_RISE = 0.028
CLUSTER_X = -0.235
CLUSTER_Y = 0.212

KNOB_OFFSET = (-0.040, 0.000)
POWER_OFFSET = (0.010, 0.000)
WATER_OFFSET_0 = (0.042, 0.010)
WATER_OFFSET_1 = (0.042, -0.010)

POWER_BUTTON_SIZE = (0.030, 0.020, 0.008)
WATER_BUTTON_SIZE = (0.024, 0.018, 0.007)
BUTTON_TRAVEL = 0.004

FLAP_X = 0.252
FLAP_Y = 0.292
FLAP_W = 0.096
FLAP_D = 0.066
FLAP_T = 0.014

TUB_CENTER = (OPENING_X, OPENING_Y, 0.23)
TUB_OUTER_R = 0.238
TUB_INNER_R = 0.221
TUB_H = 0.60
TUB_BOTTOM_T = 0.010

BASKET_OUTER_R = 0.214
BASKET_INNER_R = 0.204
BASKET_H = 0.52
BASKET_BOTTOM_T = 0.008


def _cabinet_shell_shape():
    outer = cq.Workplane("XY").box(CABINET_W, CABINET_D, SHELL_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BOTTOM_T)
        .box(
            CABINET_W - 2.0 * WALL_T,
            CABINET_D - 2.0 * WALL_T,
            SHELL_H - BOTTOM_T + 0.002,
            centered=(True, True, False),
        )
    )
    return outer.cut(inner)


def _top_deck_shape():
    deck = cq.Workplane("XY").box(CABINET_W, CABINET_D, TOP_DECK_T, centered=(True, True, False))
    main_opening = (
        cq.Workplane("XY")
        .center(OPENING_X, OPENING_Y)
        .box(OPENING_W, OPENING_D, TOP_DECK_T + 0.010, centered=(True, True, False))
    )
    flap_opening = (
        cq.Workplane("XY")
        .center(FLAP_X, FLAP_Y)
        .box(FLAP_W, FLAP_D, TOP_DECK_T + 0.010, centered=(True, True, False))
    )
    return deck.cut(main_opening).cut(flap_opening)


def _outer_tub_shape():
    tub_shell = cq.Workplane("XY").circle(TUB_OUTER_R).extrude(TUB_H)
    inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=TUB_BOTTOM_T)
        .circle(TUB_INNER_R)
        .extrude(TUB_H - TUB_BOTTOM_T + 0.002)
    )
    rim = (
        cq.Workplane("XY")
        .workplane(offset=TUB_H - 0.016)
        .circle(TUB_OUTER_R + 0.012)
        .circle(TUB_OUTER_R)
        .extrude(0.016)
    )
    return tub_shell.cut(inner_cut).union(rim)


def _basket_shape():
    basket_shell = cq.Workplane("XY").circle(BASKET_OUTER_R).extrude(BASKET_H)
    inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=BASKET_BOTTOM_T)
        .circle(BASKET_INNER_R)
        .extrude(BASKET_H - BASKET_BOTTOM_T + 0.002)
    )
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BASKET_H - 0.018)
        .circle(BASKET_OUTER_R + 0.008)
        .circle(BASKET_OUTER_R)
        .extrude(0.018)
    )
    wash_plate = (
        cq.Workplane("XY")
        .workplane(offset=BASKET_BOTTOM_T)
        .circle(0.082)
        .extrude(0.020)
    )
    return basket_shell.cut(inner_cut).union(rim).union(wash_plate)


def _opening_ring_shape():
    return (
        cq.Workplane("XY")
        .circle(0.255)
        .circle(0.232)
        .extrude(0.040)
    )


def _control_cluster_shape():
    cluster = cq.Workplane("XY").box(CLUSTER_W, CLUSTER_D, CLUSTER_H, centered=(True, True, False))
    rear_cap = (
        cq.Workplane("XY")
        .center(0.0, CLUSTER_D * 0.34)
        .box(CLUSTER_W, 0.030, CLUSTER_REAR_RISE, centered=(True, True, False))
        .translate((0.0, 0.0, CLUSTER_H))
    )
    cluster = cluster.union(rear_cap)

    knob_bezel = (
        cq.Workplane("XY")
        .center(KNOB_OFFSET[0], KNOB_OFFSET[1])
        .circle(0.031)
        .extrude(0.004)
        .translate((0.0, 0.0, CLUSTER_H))
    )
    cluster = cluster.union(knob_bezel)

    for x_pos, y_pos, width, depth in (
        (POWER_OFFSET[0], POWER_OFFSET[1], POWER_BUTTON_SIZE[0] + 0.006, POWER_BUTTON_SIZE[1] + 0.006),
        (WATER_OFFSET_0[0], WATER_OFFSET_0[1], WATER_BUTTON_SIZE[0] + 0.006, WATER_BUTTON_SIZE[1] + 0.006),
        (WATER_OFFSET_1[0], WATER_OFFSET[1] if False else WATER_OFFSET_1[1], WATER_BUTTON_SIZE[0] + 0.006, WATER_BUTTON_SIZE[1] + 0.006),
    ):
        bezel = (
            cq.Workplane("XY")
            .center(x_pos, y_pos)
            .box(width, depth, 0.004, centered=(True, True, False))
            .translate((0.0, 0.0, CLUSTER_H))
        )
        cluster = cluster.union(bezel)

    pocket_specs = [
        (POWER_OFFSET[0], POWER_OFFSET[1], POWER_BUTTON_SIZE[0] + 0.008, POWER_BUTTON_SIZE[1] + 0.008, 0.022),
        (WATER_OFFSET_0[0], WATER_OFFSET_0[1], WATER_BUTTON_SIZE[0] + 0.008, WATER_BUTTON_SIZE[1] + 0.008, 0.020),
        (WATER_OFFSET_1[0], WATER_OFFSET_1[1], WATER_BUTTON_SIZE[0] + 0.008, WATER_BUTTON_SIZE[1] + 0.008, 0.020),
    ]
    for x_pos, y_pos, width, depth, pocket_depth in pocket_specs:
        pocket = (
            cq.Workplane("XY")
            .workplane(offset=CLUSTER_H - pocket_depth)
            .center(x_pos, y_pos)
            .box(width, depth, pocket_depth, centered=(True, True, False))
        )
        cluster = cluster.cut(pocket)

    knob_seat = (
        cq.Workplane("XY")
        .workplane(offset=CLUSTER_H - 0.012)
        .center(KNOB_OFFSET[0], KNOB_OFFSET[1])
        .circle(0.035)
        .extrude(0.012)
    )
    return cluster.cut(knob_seat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    deck_white = model.material("deck_white", rgba=(0.90, 0.91, 0.93, 1.0))
    lid_white = model.material("lid_white", rgba=(0.96, 0.97, 0.98, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    tub_white = model.material("tub_white", rgba=(0.90, 0.91, 0.92, 1.0))
    basket_gray = model.material("basket_gray", rgba=(0.75, 0.77, 0.79, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_W, WALL_T, SHELL_H)),
        origin=Origin(xyz=(0.0, -CABINET_D * 0.5 + WALL_T * 0.5, SHELL_H * 0.5)),
        material=cabinet_white,
        name="front_panel",
    )
    cabinet.visual(
        Box((CABINET_W, WALL_T, SHELL_H)),
        origin=Origin(xyz=(0.0, CABINET_D * 0.5 - WALL_T * 0.5, SHELL_H * 0.5)),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((WALL_T, CABINET_D - 2.0 * WALL_T, SHELL_H)),
        origin=Origin(xyz=(-CABINET_W * 0.5 + WALL_T * 0.5, 0.0, SHELL_H * 0.5)),
        material=cabinet_white,
        name="left_panel",
    )
    cabinet.visual(
        Box((WALL_T, CABINET_D - 2.0 * WALL_T, SHELL_H)),
        origin=Origin(xyz=(CABINET_W * 0.5 - WALL_T * 0.5, 0.0, SHELL_H * 0.5)),
        material=cabinet_white,
        name="right_panel",
    )
    cabinet.visual(
        Box((CABINET_W - 2.0 * WALL_T, CABINET_D - 2.0 * WALL_T, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T * 0.5)),
        material=cabinet_white,
        name="bottom_pan",
    )
    cabinet.visual(
        mesh_from_cadquery(_top_deck_shape(), "washer_top_deck"),
        origin=Origin(xyz=(0.0, 0.0, SHELL_H)),
        material=deck_white,
        name="top_deck",
    )
    cabinet.visual(
        Box((0.48, 0.016, 0.036)),
        origin=Origin(xyz=(0.0, -CABINET_D * 0.5 + 0.008, 0.090)),
        material=trim_gray,
        name="toe_kick",
    )
    cabinet.visual(
        mesh_from_cadquery(_opening_ring_shape(), "washer_opening_ring"),
        origin=Origin(xyz=(OPENING_X, OPENING_Y, SHELL_H - 0.040)),
        material=tub_white,
        name="opening_ring",
    )
    cabinet.visual(
        Cylinder(radius=0.058, length=0.218),
        origin=Origin(xyz=(OPENING_X, OPENING_Y, 0.022 + 0.109)),
        material=tub_white,
        name="drive_pedestal",
    )
    cabinet.visual(
        Box((CLUSTER_W, CLUSTER_D, CLUSTER_H)),
        origin=Origin(xyz=(CLUSTER_X, CLUSTER_Y, CABINET_H + CLUSTER_H * 0.5)),
        material=deck_white,
        name="control_cluster",
    )
    cabinet.visual(
        Box((CLUSTER_W, 0.030, CLUSTER_REAR_RISE + 0.004)),
        origin=Origin(
            xyz=(
                CLUSTER_X,
                CLUSTER_Y + CLUSTER_D * 0.31,
                CABINET_H + CLUSTER_H - 0.004 + (CLUSTER_REAR_RISE + 0.004) * 0.5,
            )
        ),
        material=deck_white,
        name="cluster_riser",
    )
    cabinet.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(
            xyz=(CLUSTER_X + KNOB_OFFSET[0], CLUSTER_Y + KNOB_OFFSET[1], CABINET_H + CLUSTER_H + 0.001)
        ),
        material=trim_gray,
        name="knob_seat",
    )
    cabinet.visual(
        Box((POWER_BUTTON_SIZE[0] + 0.006, POWER_BUTTON_SIZE[1] + 0.006, 0.005)),
        origin=Origin(
            xyz=(CLUSTER_X + POWER_OFFSET[0], CLUSTER_Y + POWER_OFFSET[1], CABINET_H + CLUSTER_H + 0.0015)
        ),
        material=trim_gray,
        name="power_bezel",
    )
    cabinet.visual(
        Box((WATER_BUTTON_SIZE[0] + 0.006, WATER_BUTTON_SIZE[1] + 0.006, 0.005)),
        origin=Origin(
            xyz=(CLUSTER_X + WATER_OFFSET_0[0], CLUSTER_Y + WATER_OFFSET_0[1], CABINET_H + CLUSTER_H + 0.0015)
        ),
        material=trim_gray,
        name="water_bezel_0",
    )
    cabinet.visual(
        Box((WATER_BUTTON_SIZE[0] + 0.006, WATER_BUTTON_SIZE[1] + 0.006, 0.005)),
        origin=Origin(
            xyz=(CLUSTER_X + WATER_OFFSET_1[0], CLUSTER_Y + WATER_OFFSET_1[1], CABINET_H + CLUSTER_H + 0.0015)
        ),
        material=trim_gray,
        name="water_bezel_1",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "washer_basket"),
        material=basket_gray,
        name="basket_wall",
    )
    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(OPENING_X, OPENING_Y, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, -LID_D * 0.5, LID_T * 0.5)),
        material=lid_white,
        name="lid_panel",
    )
    lid.visual(
        Box((0.150, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -LID_D + 0.018, LID_T + 0.007)),
        material=trim_gray,
        name="front_handle",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(OPENING_X, OPENING_Y + OPENING_D * 0.5, CABINET_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.32),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.028,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.058, 0.006, flare=0.07),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "washer_control_knob",
        ),
        material=trim_gray,
        name="knob_body",
    )
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(CLUSTER_X + KNOB_OFFSET[0], CLUSTER_Y + KNOB_OFFSET[1], CABINET_H + CLUSTER_H + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box(POWER_BUTTON_SIZE),
        origin=Origin(xyz=(0.0, 0.0, POWER_BUTTON_SIZE[2] * 0.5)),
        material=trim_gray,
        name="button_cap",
    )
    model.articulation(
        "cabinet_to_power_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=power_button,
        origin=Origin(xyz=(CLUSTER_X + POWER_OFFSET[0], CLUSTER_Y + POWER_OFFSET[1], CABINET_H + CLUSTER_H + 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )

    water_button_0 = model.part("water_button_0")
    water_button_0.visual(
        Box(WATER_BUTTON_SIZE),
        origin=Origin(xyz=(0.0, 0.0, WATER_BUTTON_SIZE[2] * 0.5)),
        material=trim_gray,
        name="button_cap",
    )
    model.articulation(
        "cabinet_to_water_button_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=water_button_0,
        origin=Origin(
            xyz=(CLUSTER_X + WATER_OFFSET_0[0], CLUSTER_Y + WATER_OFFSET_0[1], CABINET_H + CLUSTER_H + 0.004)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )

    water_button_1 = model.part("water_button_1")
    water_button_1.visual(
        Box(WATER_BUTTON_SIZE),
        origin=Origin(xyz=(0.0, 0.0, WATER_BUTTON_SIZE[2] * 0.5)),
        material=trim_gray,
        name="button_cap",
    )
    model.articulation(
        "cabinet_to_water_button_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=water_button_1,
        origin=Origin(
            xyz=(CLUSTER_X + WATER_OFFSET_1[0], CLUSTER_Y + WATER_OFFSET_1[1], CABINET_H + CLUSTER_H + 0.004)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )

    flap = model.part("detergent_flap")
    flap.visual(
        Box((FLAP_W, FLAP_D, FLAP_T)),
        origin=Origin(xyz=(0.0, -FLAP_D * 0.5, FLAP_T * 0.5)),
        material=deck_white,
        name="flap_panel",
    )
    flap.visual(
        Box((0.040, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -FLAP_D + 0.010, FLAP_T + 0.004)),
        material=trim_gray,
        name="flap_tab",
    )
    model.articulation(
        "cabinet_to_detergent_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(FLAP_X, FLAP_Y + FLAP_D * 0.5, CABINET_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("knob")
    power_button = object_model.get_part("power_button")
    water_button_0 = object_model.get_part("water_button_0")
    water_button_1 = object_model.get_part("water_button_1")
    flap = object_model.get_part("detergent_flap")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    knob_joint = object_model.get_articulation("cabinet_to_knob")
    power_joint = object_model.get_articulation("cabinet_to_power_button")
    water_joint_0 = object_model.get_articulation("cabinet_to_water_button_0")
    water_joint_1 = object_model.get_articulation("cabinet_to_water_button_1")
    flap_hinge = object_model.get_articulation("cabinet_to_detergent_flap")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_deck",
            max_gap=0.002,
            max_penetration=0.0,
            name="lid closes flush to the top deck",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            elem_a="lid_panel",
            elem_b="top_deck",
            min_overlap=0.24,
            name="lid covers the washer opening footprint",
        )

    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        elem_a="basket_wall",
        elem_b="opening_ring",
        margin=0.010,
        name="basket stays centered under the washer opening ring",
    )
    ctx.expect_contact(
        basket,
        cabinet,
        elem_b="drive_pedestal",
        name="basket is mounted on the central drive pedestal",
    )

    rest_handle = None
    open_handle = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        rest_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            ctx.expect_gap(
                lid,
                cabinet,
                axis="z",
                positive_elem="front_handle",
                negative_elem="top_deck",
                min_gap=0.16,
                name="opened lid front edge rises above the deck",
            )
            open_handle = ctx.part_element_world_aabb(lid, elem="front_handle")

    ctx.check(
        "lid front lifts upward",
        rest_handle is not None
        and open_handle is not None
        and float(open_handle[1][2]) > float(rest_handle[1][2]) + 0.18,
        details=f"rest={rest_handle}, open={open_handle}",
    )

    ctx.expect_gap(
        knob,
        cabinet,
        axis="z",
        positive_elem="knob_body",
        negative_elem="knob_seat",
        max_gap=0.001,
        max_penetration=0.0,
        name="control knob sits on the cluster top",
    )

    rest_power_pos = ctx.part_world_position(power_button)
    pressed_power_pos = None
    with ctx.pose({power_joint: BUTTON_TRAVEL}):
        pressed_power_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses downward",
        rest_power_pos is not None
        and pressed_power_pos is not None
        and float(pressed_power_pos[2]) < float(rest_power_pos[2]) - 0.003,
        details=f"rest={rest_power_pos}, pressed={pressed_power_pos}",
    )

    for button_part, button_joint, button_name in (
        (water_button_0, water_joint_0, "water button 0"),
        (water_button_1, water_joint_1, "water button 1"),
    ):
        rest_pos = ctx.part_world_position(button_part)
        pressed_pos = None
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"{button_name} depresses downward",
            rest_pos is not None and pressed_pos is not None and float(pressed_pos[2]) < float(rest_pos[2]) - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    rest_flap = ctx.part_element_world_aabb(flap, elem="flap_tab")
    open_flap = None
    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_gap(
            flap,
            cabinet,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="top_deck",
            max_gap=0.002,
            max_penetration=0.0,
            name="detergent flap closes flush with the deck",
        )
    with ctx.pose({flap_hinge: 1.10}):
        open_flap = ctx.part_element_world_aabb(flap, elem="flap_tab")
    ctx.check(
        "detergent flap opens upward",
        rest_flap is not None and open_flap is not None and float(open_flap[1][2]) > float(rest_flap[1][2]) + 0.035,
        details=f"rest={rest_flap}, open={open_flap}",
    )

    return ctx.report()


object_model = build_object_model()
