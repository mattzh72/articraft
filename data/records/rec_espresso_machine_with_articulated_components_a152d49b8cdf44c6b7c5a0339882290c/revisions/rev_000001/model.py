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


BODY_WIDTH = 0.300
BODY_DEPTH = 0.350
BODY_HEIGHT = 0.390
BODY_WALL = 0.018
BODY_BASE = 0.030

DOOR_THICKNESS = 0.012
DOOR_WIDTH = 0.215
DOOR_HEIGHT = 0.245
DOOR_Y0 = -0.110
DOOR_Z0 = 0.095

TRAY_WIDTH = 0.200
TRAY_LENGTH = 0.180
TRAY_HEIGHT = 0.026


def _tray_shape() -> cq.Workplane:
    tray = (
        cq.Workplane("XY")
        .box(TRAY_WIDTH, TRAY_LENGTH, TRAY_HEIGHT)
        .translate((0.0, -TRAY_LENGTH / 2.0, TRAY_HEIGHT / 2.0))
        .faces(">Z")
        .shell(-0.003)
    )
    front_lip = cq.Workplane("XY").box(TRAY_WIDTH, 0.010, 0.032).translate((0.0, 0.004, 0.016))
    return tray.union(front_lip)


def _steam_wheel_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.023).circle(0.017).extrude(0.010)
    hub = cq.Workplane("XY").circle(0.007).extrude(0.010)
    spokes = (
        cq.Workplane("XY")
        .box(0.032, 0.004, 0.010)
        .union(cq.Workplane("XY").box(0.004, 0.032, 0.010))
        .translate((0.0, 0.0, 0.005))
    )
    return ring.union(hub).union(spokes)


def _door_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT)
        .translate((-DOOR_THICKNESS / 2.0, DOOR_WIDTH / 2.0, DOOR_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.005)
    )
    recess = cq.Workplane("XY").box(
        DOOR_THICKNESS * 0.6,
        DOOR_WIDTH - 0.040,
        DOOR_HEIGHT - 0.050,
    ).translate(
        (
            -DOOR_THICKNESS + (DOOR_THICKNESS * 0.6) / 2.0 + 0.0005,
            DOOR_WIDTH / 2.0,
            DOOR_HEIGHT / 2.0,
        )
    )
    return panel.cut(recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="espresso_machine")

    shell_metal = model.material("shell_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.18, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.62, 0.76, 0.88, 0.42))
    tank_cap = model.material("tank_cap", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_BASE)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE / 2.0)),
        material=shell_metal,
        name="base",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.010)),
        material=shell_metal,
        name="roof",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + BODY_WALL / 2.0, BODY_HEIGHT / 2.0)),
        material=shell_metal,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - BODY_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=shell_metal,
        name="right_wall",
    )
    rear_side_len = DOOR_Y0 + BODY_DEPTH / 2.0
    front_side_len = BODY_DEPTH / 2.0 - (DOOR_Y0 + DOOR_WIDTH)
    body.visual(
        Box((BODY_WALL, rear_side_len, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH / 2.0 + BODY_WALL / 2.0,
                -BODY_DEPTH / 2.0 + rear_side_len / 2.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material=shell_metal,
        name="left_rear_wall",
    )
    body.visual(
        Box((BODY_WALL, front_side_len, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH / 2.0 + BODY_WALL / 2.0,
                DOOR_Y0 + DOOR_WIDTH + front_side_len / 2.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material=shell_metal,
        name="left_front_wall",
    )
    body.visual(
        Box((BODY_WALL, DOOR_WIDTH, DOOR_Z0)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH / 2.0 + BODY_WALL / 2.0,
                DOOR_Y0 + DOOR_WIDTH / 2.0,
                DOOR_Z0 / 2.0,
            )
        ),
        material=shell_metal,
        name="left_lower_frame",
    )
    body.visual(
        Box((BODY_WALL, DOOR_WIDTH, BODY_HEIGHT - (DOOR_Z0 + DOOR_HEIGHT))),
        origin=Origin(
            xyz=(
                -BODY_WIDTH / 2.0 + BODY_WALL / 2.0,
                DOOR_Y0 + DOOR_WIDTH / 2.0,
                DOOR_Z0 + DOOR_HEIGHT + (BODY_HEIGHT - (DOOR_Z0 + DOOR_HEIGHT)) / 2.0,
            )
        ),
        material=shell_metal,
        name="left_upper_frame",
    )
    body.visual(
        Box((0.042, BODY_WALL, 0.060)),
        origin=Origin(xyz=(-0.129, BODY_DEPTH / 2.0 - BODY_WALL / 2.0, 0.064)),
        material=shell_metal,
        name="front_left_post",
    )
    body.visual(
        Box((0.042, BODY_WALL, 0.060)),
        origin=Origin(xyz=(0.129, BODY_DEPTH / 2.0 - BODY_WALL / 2.0, 0.064)),
        material=shell_metal,
        name="front_right_post",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_WALL, 0.036)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - BODY_WALL / 2.0, 0.018)),
        material=shell_metal,
        name="front_sill",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_WALL, BODY_HEIGHT - 0.096)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH / 2.0 - BODY_WALL / 2.0,
                0.096 + (BODY_HEIGHT - 0.096) / 2.0,
            )
        ),
        material=shell_metal,
        name="front_fascia",
    )
    body.visual(
        Box((0.160, 0.160, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + 0.004)),
        material=dark_trim,
        name="cup_warmer",
    )
    body.visual(
        Box((0.220, 0.145, 0.006)),
        origin=Origin(xyz=(0.0, 0.030, BODY_BASE + 0.001)),
        material=dark_trim,
        name="tray_floor",
    )
    body.visual(
        Box((0.122, 0.128, 0.006)),
        origin=Origin(xyz=(-0.076, -0.094, BODY_BASE + 0.001)),
        material=dark_trim,
        name="tank_floor",
    )
    body.visual(
        Box((0.210, 0.010, 0.160)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.003, 0.185)),
        material=dark_trim,
        name="front_panel",
    )
    body.visual(
        Box((0.060, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.004, 0.255)),
        material=dark_trim,
        name="group_mount",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(
            xyz=(-0.010, BODY_DEPTH / 2.0 + 0.014, 0.118),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="steam_boss",
    )
    body.visual(
        Box((0.022, 0.018, 0.024)),
        origin=Origin(xyz=(0.110, BODY_DEPTH / 2.0 + 0.004, 0.255)),
        material=dark_trim,
        name="wand_bracket",
    )

    tank = model.part("tank")
    tank.visual(
        Box((0.108, 0.118, 0.205)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=glass_blue,
        name="tank_body",
    )
    tank.visual(
        Box((0.070, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material=tank_cap,
        name="tank_handle",
    )
    model.articulation(
        "body_to_tank",
        ArticulationType.FIXED,
        parent=body,
        child=tank,
        origin=Origin(xyz=(-0.076, -0.094, BODY_BASE + 0.004)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel_shape(), "espresso_door_panel"),
        material=shell_metal,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.095),
        origin=Origin(xyz=(-0.015, DOOR_WIDTH - 0.026, 0.126)),
        material=dark_trim,
        name="door_handle",
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-BODY_WIDTH / 2.0, DOOR_Y0, DOOR_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.65),
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "espresso_tray"),
        material=dark_trim,
        name="tray_shell",
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - BODY_WALL - 0.001, BODY_BASE + 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.082),
    )

    group_head = model.part("group_head")
    group_head.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(xyz=(0.0, -0.007, 0.022), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="support",
    )
    group_head.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="brew_body",
    )
    group_head.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_trim,
        name="screen",
    )
    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.041, 0.228)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="collar",
    )
    portafilter.visual(
        Cylinder(radius=0.029, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.026, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.017, -0.022)),
        material=steel,
        name="lug",
    )
    portafilter.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.0, 0.077, -0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="handle",
    )
    model.articulation(
        "group_head_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=portafilter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=-0.60, upper=0.18),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Box((0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=steel,
        name="pivot_lug",
    )
    steam_wand.visual(
        Cylinder(radius=0.0055, length=0.036),
        origin=Origin(xyz=(0.0, 0.023, -0.008), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.145),
        origin=Origin(xyz=(0.0, 0.041, -0.082)),
        material=steel,
        name="pipe",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.041, -0.150), rpy=(0.22, 0.0, 0.0)),
        material=steel,
        name="tip",
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.110, BODY_DEPTH / 2.0 + 0.013, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=-1.10, upper=0.70),
    )

    steam_wheel = model.part("steam_wheel")
    steam_wheel.visual(
        mesh_from_cadquery(_steam_wheel_shape(), "espresso_steam_wheel"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="wheel",
    )
    steam_wheel.visual(
        Box((0.006, 0.010, 0.006)),
        origin=Origin(xyz=(0.018, 0.005, 0.0)),
        material=shell_metal,
        name="marker",
    )
    model.articulation(
        "body_to_steam_wheel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wheel,
        origin=Origin(xyz=(-0.010, BODY_DEPTH / 2.0 + 0.020, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=4.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    tank = object_model.get_part("tank")
    tray = object_model.get_part("tray")
    group_head = object_model.get_part("group_head")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    steam_wheel = object_model.get_part("steam_wheel")
    door_joint = object_model.get_articulation("body_to_door")
    tray_joint = object_model.get_articulation("body_to_tray")
    portafilter_joint = object_model.get_articulation("group_head_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    wheel_joint = object_model.get_articulation("body_to_steam_wheel")

    ctx.expect_gap(
        body,
        door,
        axis="x",
        max_gap=0.010,
        max_penetration=0.012,
        negative_elem="door_panel",
        name="door sits close to the side shell when closed",
    )
    ctx.expect_gap(
        tank,
        body,
        axis="z",
        max_gap=0.006,
        max_penetration=0.001,
        positive_elem="tank_body",
        negative_elem="tank_floor",
        name="tank rests on the internal floor",
    )
    ctx.expect_gap(
        tray,
        body,
        axis="z",
        max_gap=0.006,
        max_penetration=0.001,
        positive_elem="tray_shell",
        negative_elem="tray_floor",
        name="tray rides on the tray floor",
    )
    ctx.expect_gap(
        group_head,
        portafilter,
        axis="z",
        max_gap=0.008,
        max_penetration=0.0,
        positive_elem="screen",
        negative_elem="collar",
        name="portafilter seats just below the brew screen",
    )
    ctx.expect_origin_gap(
        steam_wheel,
        tray,
        axis="z",
        min_gap=0.050,
        name="steam wheel sits above the drip tray",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.082}):
        tray_extended = ctx.part_world_position(tray)

    ctx.check(
        "tray slides outward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] > tray_rest[1] + 0.06,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    with ctx.pose({door_joint: 1.20}):
        door_open_aabb = ctx.part_world_aabb(door)
    door_closed_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][0] < door_closed_aabb[0][0] - 0.050,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    handle_locked = ctx.part_element_world_aabb(portafilter, elem="handle")
    with ctx.pose({portafilter_joint: -0.55}):
        handle_unlocked = ctx.part_element_world_aabb(portafilter, elem="handle")
    ctx.check(
        "portafilter rotates about the brew axis",
        handle_locked is not None
        and handle_unlocked is not None
        and ((handle_unlocked[0][0] + handle_unlocked[1][0]) * 0.5)
        > ((handle_locked[0][0] + handle_locked[1][0]) * 0.5) + 0.020,
        details=f"locked={handle_locked}, unlocked={handle_unlocked}",
    )

    wand_rest = ctx.part_element_world_aabb(steam_wand, elem="pipe")
    with ctx.pose({wand_joint: 0.65}):
        wand_swung = ctx.part_element_world_aabb(steam_wand, elem="pipe")
    ctx.check(
        "steam wand swings across the front corner",
        wand_rest is not None
        and wand_swung is not None
        and ((wand_swung[0][0] + wand_swung[1][0]) * 0.5)
        < ((wand_rest[0][0] + wand_rest[1][0]) * 0.5) - 0.018,
        details=f"rest={wand_rest}, swung={wand_swung}",
    )

    marker_rest = ctx.part_element_world_aabb(steam_wheel, elem="marker")
    with ctx.pose({wheel_joint: 1.60}):
        marker_turned = ctx.part_element_world_aabb(steam_wheel, elem="marker")
    ctx.check(
        "steam wheel rotates around its shaft",
        marker_rest is not None
        and marker_turned is not None
        and abs(
            ((marker_turned[0][2] + marker_turned[1][2]) * 0.5)
            - ((marker_rest[0][2] + marker_rest[1][2]) * 0.5)
        )
        > 0.010,
        details=f"rest={marker_rest}, turned={marker_turned}",
    )

    return ctx.report()


object_model = build_object_model()
