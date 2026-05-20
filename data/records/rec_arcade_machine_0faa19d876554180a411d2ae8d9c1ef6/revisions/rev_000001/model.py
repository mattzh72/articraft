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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_LENGTH = 0.96
TOP_WIDTH = 0.66
TOP_HEIGHT = 0.23
TOP_BOTTOM_Z = 0.51
TOP_Z = TOP_BOTTOM_Z + TOP_HEIGHT

PLINTH_LENGTH = 0.54
PLINTH_WIDTH = 0.42
PLINTH_HEIGHT = 0.06

PEDESTAL_LENGTH = 0.40
PEDESTAL_WIDTH = 0.28
PEDESTAL_HEIGHT = TOP_BOTTOM_Z - PLINTH_HEIGHT

GLASS_LENGTH = 0.56
GLASS_WIDTH = 0.40
GLASS_THICKNESS = 0.006

SCREEN_LENGTH = 0.46
SCREEN_WIDTH = 0.30
SCREEN_THICKNESS = 0.006
SCREEN_WELL_DEPTH = 0.11
SCREEN_WELL_BOTTOM_Z = TOP_Z - GLASS_THICKNESS - SCREEN_WELL_DEPTH
OUTER_WALL = 0.018
TOP_DECK_THICKNESS = 0.018
SCREEN_FLOOR_THICKNESS = 0.012

PANEL_LENGTH = 0.34
PANEL_DEPTH = 0.18
PANEL_THICKNESS = 0.012
PANEL_ANGLE = math.radians(32.0)
PLAYER_PANEL_CENTER = (0.0, 0.255, 0.645)
OPPONENT_PANEL_CENTER = (0.0, -0.255, 0.645)
PANEL_INSET = -0.010

JOYSTICK_LOCAL_X = -0.10
JOYSTICK_OPENING_RADIUS = 0.022
JOYSTICK_TUNNEL_RADIUS = 0.030
JOYSTICK_TUNNEL_LENGTH = 0.12
GIMBAL_BORE_RADIUS = 0.010

DOOR_WIDTH = 0.24
DOOR_HEIGHT = 0.15
DOOR_THICKNESS = 0.014
DOOR_CENTER_Z = 0.625
CASHBOX_OPENING_WIDTH = 0.18
CASHBOX_OPENING_HEIGHT = 0.11
CASHBOX_DEPTH = 0.11


def _rotate_x(local_xyz: tuple[float, float, float], roll: float) -> tuple[float, float, float]:
    x, y, z = local_xyz
    c = math.cos(roll)
    s = math.sin(roll)
    return (x, y * c - z * s, y * s + z * c)


def _panel_point(
    center_xyz: tuple[float, float, float],
    roll: float,
    local_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx, dy, dz = _rotate_x(local_xyz, roll)
    return (center_xyz[0] + dx, center_xyz[1] + dy, center_xyz[2] + dz)


def _panel_mesh(with_joystick_hole: bool) -> cq.Workplane:
    panel = cq.Workplane("XY").box(PANEL_LENGTH, PANEL_DEPTH, PANEL_THICKNESS)
    if with_joystick_hole:
        hole = (
            cq.Workplane("XY")
            .cylinder(PANEL_THICKNESS * 3.0, JOYSTICK_OPENING_RADIUS)
            .translate((JOYSTICK_LOCAL_X, 0.0, 0.0))
        )
        panel = panel.cut(hole)
    return panel


def _cabinet_shell_mesh() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(TOP_LENGTH, TOP_WIDTH, TOP_HEIGHT)
        .translate((0.0, 0.0, TOP_BOTTOM_Z + TOP_HEIGHT / 2.0))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, PLINTH_HEIGHT + PEDESTAL_HEIGHT / 2.0))
    )
    plinth = (
        cq.Workplane("XY")
        .box(PLINTH_LENGTH, PLINTH_WIDTH, PLINTH_HEIGHT)
        .translate((0.0, 0.0, PLINTH_HEIGHT / 2.0))
    )

    glass_pocket = (
        cq.Workplane("XY")
        .box(GLASS_LENGTH, GLASS_WIDTH, GLASS_THICKNESS)
        .translate((0.0, 0.0, TOP_Z - GLASS_THICKNESS / 2.0))
    )
    screen_well = (
        cq.Workplane("XY")
        .box(SCREEN_LENGTH, SCREEN_WIDTH, SCREEN_WELL_DEPTH)
        .translate(
            (
                0.0,
                0.0,
                TOP_Z - GLASS_THICKNESS - SCREEN_WELL_DEPTH / 2.0,
            )
        )
    )
    cashbox_cavity = (
        cq.Workplane("XY")
        .box(CASHBOX_DEPTH, CASHBOX_OPENING_WIDTH, CASHBOX_OPENING_HEIGHT)
        .translate(
            (
                TOP_LENGTH / 2.0 - CASHBOX_DEPTH / 2.0 + 0.001,
                0.0,
                DOOR_CENTER_Z,
            )
        )
    )

    player_tunnel_center = _panel_point(
        PLAYER_PANEL_CENTER,
        -PANEL_ANGLE,
        (JOYSTICK_LOCAL_X, 0.0, 0.0),
    )
    player_tunnel = (
        cq.Workplane("XY")
        .cylinder(JOYSTICK_TUNNEL_LENGTH, JOYSTICK_TUNNEL_RADIUS)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(-PANEL_ANGLE))
        .translate(player_tunnel_center)
    )

    return shell.union(pedestal).union(plinth).cut(glass_pocket).cut(screen_well).cut(cashbox_cavity).cut(player_tunnel)


def _gimbal_trim_mesh() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .cylinder(0.0035, 0.027)
        .translate((0.0, 0.0, PANEL_THICKNESS / 2.0 + 0.00175))
    )
    collar = (
        cq.Workplane("XY")
        .cylinder(0.018, 0.017)
        .translate((0.0, 0.0, PANEL_THICKNESS / 2.0 - 0.004))
    )
    bore = cq.Workplane("XY").cylinder(0.060, GIMBAL_BORE_RADIUS)
    return flange.union(collar).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cocktail_arcade_table")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.16, 0.12, 0.08, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.04, 0.04, 0.05, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.45, 0.62, 0.74, 0.32))
    screen_finish = model.material("screen_finish", rgba=(0.03, 0.05, 0.07, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.73, 0.75, 0.78, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))
    red_button = model.material("red_button", rgba=(0.78, 0.12, 0.12, 1.0))
    yellow_button = model.material("yellow_button", rgba=(0.89, 0.73, 0.18, 1.0))
    blue_button = model.material("blue_button", rgba=(0.16, 0.34, 0.88, 1.0))

    cabinet = model.part("cabinet")
    top_deck_side = (TOP_WIDTH - GLASS_WIDTH) / 2.0
    top_deck_end = (TOP_LENGTH - GLASS_LENGTH) / 2.0

    def add_cabinet_box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material=cabinet_finish,
    ) -> None:
        cabinet.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    add_cabinet_box(
        "plinth",
        (PLINTH_LENGTH, PLINTH_WIDTH, PLINTH_HEIGHT),
        (0.0, 0.0, PLINTH_HEIGHT / 2.0),
    )
    add_cabinet_box(
        "pedestal",
        (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
        (0.0, 0.0, PLINTH_HEIGHT + PEDESTAL_HEIGHT / 2.0),
    )
    add_cabinet_box(
        "bottom_floor",
        (TOP_LENGTH, TOP_WIDTH, OUTER_WALL),
        (0.0, 0.0, TOP_BOTTOM_Z + OUTER_WALL / 2.0),
    )
    add_cabinet_box(
        "side_wall_0",
        (TOP_LENGTH, OUTER_WALL, TOP_HEIGHT - OUTER_WALL),
        (
            0.0,
            TOP_WIDTH / 2.0 - OUTER_WALL / 2.0,
            TOP_BOTTOM_Z + OUTER_WALL + (TOP_HEIGHT - OUTER_WALL) / 2.0,
        ),
    )
    add_cabinet_box(
        "side_wall_1",
        (TOP_LENGTH, OUTER_WALL, TOP_HEIGHT - OUTER_WALL),
        (
            0.0,
            -TOP_WIDTH / 2.0 + OUTER_WALL / 2.0,
            TOP_BOTTOM_Z + OUTER_WALL + (TOP_HEIGHT - OUTER_WALL) / 2.0,
        ),
    )
    add_cabinet_box(
        "end_wall_0",
        (OUTER_WALL, TOP_WIDTH - 2.0 * OUTER_WALL, TOP_HEIGHT - OUTER_WALL),
        (
            -TOP_LENGTH / 2.0 + OUTER_WALL / 2.0,
            0.0,
            TOP_BOTTOM_Z + OUTER_WALL + (TOP_HEIGHT - OUTER_WALL) / 2.0,
        ),
    )

    opening_bottom = DOOR_CENTER_Z - CASHBOX_OPENING_HEIGHT / 2.0
    opening_top = DOOR_CENTER_Z + CASHBOX_OPENING_HEIGHT / 2.0
    bottom_strip_height = opening_bottom - (TOP_BOTTOM_Z + OUTER_WALL)
    top_strip_height = TOP_Z - opening_top

    add_cabinet_box(
        "cashbox_bottom_strip",
        (OUTER_WALL, TOP_WIDTH - 2.0 * OUTER_WALL, bottom_strip_height),
        (
            TOP_LENGTH / 2.0 - OUTER_WALL / 2.0,
            0.0,
            TOP_BOTTOM_Z + OUTER_WALL + bottom_strip_height / 2.0,
        ),
    )
    add_cabinet_box(
        "cashbox_top_strip",
        (OUTER_WALL, TOP_WIDTH - 2.0 * OUTER_WALL, top_strip_height),
        (
            TOP_LENGTH / 2.0 - OUTER_WALL / 2.0,
            0.0,
            opening_top + top_strip_height / 2.0,
        ),
    )
    jamb_width = ((TOP_WIDTH - 2.0 * OUTER_WALL) - CASHBOX_OPENING_WIDTH) / 2.0
    add_cabinet_box(
        "cashbox_jamb_0",
        (OUTER_WALL, jamb_width, CASHBOX_OPENING_HEIGHT),
        (
            TOP_LENGTH / 2.0 - OUTER_WALL / 2.0,
            CASHBOX_OPENING_WIDTH / 2.0 + jamb_width / 2.0,
            DOOR_CENTER_Z,
        ),
    )
    add_cabinet_box(
        "cashbox_jamb_1",
        (OUTER_WALL, jamb_width, CASHBOX_OPENING_HEIGHT),
        (
            TOP_LENGTH / 2.0 - OUTER_WALL / 2.0,
            -CASHBOX_OPENING_WIDTH / 2.0 - jamb_width / 2.0,
            DOOR_CENTER_Z,
        ),
    )

    add_cabinet_box(
        "top_deck_side_0",
        (GLASS_LENGTH, top_deck_side, TOP_DECK_THICKNESS),
        (
            0.0,
            GLASS_WIDTH / 2.0 + top_deck_side / 2.0,
            TOP_Z - TOP_DECK_THICKNESS / 2.0,
        ),
    )
    add_cabinet_box(
        "top_deck_side_1",
        (GLASS_LENGTH, top_deck_side, TOP_DECK_THICKNESS),
        (
            0.0,
            -GLASS_WIDTH / 2.0 - top_deck_side / 2.0,
            TOP_Z - TOP_DECK_THICKNESS / 2.0,
        ),
    )
    add_cabinet_box(
        "top_deck_end_0",
        (top_deck_end, TOP_WIDTH, TOP_DECK_THICKNESS),
        (
            -GLASS_LENGTH / 2.0 - top_deck_end / 2.0,
            0.0,
            TOP_Z - TOP_DECK_THICKNESS / 2.0,
        ),
    )
    add_cabinet_box(
        "top_deck_end_1",
        (top_deck_end, TOP_WIDTH, TOP_DECK_THICKNESS),
        (
            GLASS_LENGTH / 2.0 + top_deck_end / 2.0,
            0.0,
            TOP_Z - TOP_DECK_THICKNESS / 2.0,
        ),
    )

    add_cabinet_box(
        "screen_floor",
        (GLASS_LENGTH, GLASS_WIDTH, SCREEN_FLOOR_THICKNESS),
        (
            0.0,
            0.0,
            SCREEN_WELL_BOTTOM_Z - SCREEN_FLOOR_THICKNESS / 2.0,
        ),
    )
    well_side_thickness = (GLASS_WIDTH - SCREEN_WIDTH) / 2.0
    well_end_thickness = (GLASS_LENGTH - SCREEN_LENGTH) / 2.0
    add_cabinet_box(
        "well_side_0",
        (GLASS_LENGTH, well_side_thickness, SCREEN_WELL_DEPTH),
        (
            0.0,
            SCREEN_WIDTH / 2.0 + well_side_thickness / 2.0,
            SCREEN_WELL_BOTTOM_Z + SCREEN_WELL_DEPTH / 2.0,
        ),
        material=trim_finish,
    )
    add_cabinet_box(
        "well_side_1",
        (GLASS_LENGTH, well_side_thickness, SCREEN_WELL_DEPTH),
        (
            0.0,
            -SCREEN_WIDTH / 2.0 - well_side_thickness / 2.0,
            SCREEN_WELL_BOTTOM_Z + SCREEN_WELL_DEPTH / 2.0,
        ),
        material=trim_finish,
    )
    add_cabinet_box(
        "well_end_0",
        (well_end_thickness, SCREEN_WIDTH, SCREEN_WELL_DEPTH),
        (
            -SCREEN_LENGTH / 2.0 - well_end_thickness / 2.0,
            0.0,
            SCREEN_WELL_BOTTOM_Z + SCREEN_WELL_DEPTH / 2.0,
        ),
        material=trim_finish,
    )
    add_cabinet_box(
        "well_end_1",
        (well_end_thickness, SCREEN_WIDTH, SCREEN_WELL_DEPTH),
        (
            SCREEN_LENGTH / 2.0 + well_end_thickness / 2.0,
            0.0,
            SCREEN_WELL_BOTTOM_Z + SCREEN_WELL_DEPTH / 2.0,
        ),
        material=trim_finish,
    )

    player_panel_mount = _panel_point(
        PLAYER_PANEL_CENTER,
        -PANEL_ANGLE,
        (0.0, 0.0, PANEL_INSET),
    )
    opponent_panel_mount = _panel_point(
        OPPONENT_PANEL_CENTER,
        PANEL_ANGLE,
        (0.0, 0.0, PANEL_INSET),
    )
    cabinet.visual(
        mesh_from_cadquery(_panel_mesh(with_joystick_hole=True), "player_panel"),
        origin=Origin(xyz=player_panel_mount, rpy=(-PANEL_ANGLE, 0.0, 0.0)),
        material=trim_finish,
        name="player_panel",
    )
    cabinet.visual(
        mesh_from_cadquery(_panel_mesh(with_joystick_hole=False), "opponent_panel"),
        origin=Origin(xyz=opponent_panel_mount, rpy=(PANEL_ANGLE, 0.0, 0.0)),
        material=trim_finish,
        name="opponent_panel",
    )

    player_button_specs = (
        (0.035, 0.012, red_button, "player_button_0"),
        (0.095, 0.006, yellow_button, "player_button_1"),
        (0.155, 0.000, blue_button, "player_button_2"),
    )
    for local_x, local_y, material, name in player_button_specs:
        cabinet.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(
                xyz=_panel_point(
                    player_panel_mount,
                    -PANEL_ANGLE,
                    (local_x, local_y, 0.0005),
                ),
                rpy=(-PANEL_ANGLE, 0.0, 0.0),
            ),
            material=material,
            name=name,
        )

    opponent_button_specs = (
        (-0.09, 0.010, blue_button, "opponent_button_0"),
        (-0.03, 0.004, yellow_button, "opponent_button_1"),
        (0.03, -0.002, red_button, "opponent_button_2"),
    )
    for local_x, local_y, material, name in opponent_button_specs:
        cabinet.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(
                xyz=_panel_point(
                    opponent_panel_mount,
                    PANEL_ANGLE,
                    (local_x, local_y, 0.0005),
                ),
                rpy=(PANEL_ANGLE, 0.0, 0.0),
            ),
            material=material,
            name=name,
        )

    joystick_origin = _panel_point(
        player_panel_mount,
        -PANEL_ANGLE,
        (JOYSTICK_LOCAL_X, 0.0, 0.0),
    )
    cabinet.visual(
        mesh_from_cadquery(_gimbal_trim_mesh(), "gimbal_trim"),
        origin=Origin(xyz=joystick_origin, rpy=(-PANEL_ANGLE, 0.0, 0.0)),
        material=metal_finish,
        name="gimbal_trim",
    )

    glass = model.part("glass")
    glass.visual(
        Box((GLASS_LENGTH, GLASS_WIDTH, GLASS_THICKNESS)),
        material=glass_finish,
        name="glass_panel",
    )
    model.articulation(
        "cabinet_to_glass",
        ArticulationType.FIXED,
        parent=cabinet,
        child=glass,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z - GLASS_THICKNESS / 2.0)),
    )

    screen = model.part("screen")
    screen.visual(
        Box((SCREEN_LENGTH, SCREEN_WIDTH, SCREEN_THICKNESS)),
        material=screen_finish,
        name="screen_panel",
    )
    model.articulation(
        "cabinet_to_screen",
        ArticulationType.FIXED,
        parent=cabinet,
        child=screen,
        origin=Origin(
            xyz=(0.0, 0.0, SCREEN_WELL_BOTTOM_Z + SCREEN_THICKNESS / 2.0)
        ),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_THICKNESS / 2.0, -DOOR_WIDTH / 2.0, 0.0)),
        material=trim_finish,
        name="door_panel",
    )
    door.visual(
        Box((0.014, 0.045, 0.050)),
        origin=Origin(xyz=(DOOR_THICKNESS + 0.007, -0.19, 0.0)),
        material=metal_finish,
        name="door_pull",
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(TOP_LENGTH / 2.0, DOOR_WIDTH / 2.0, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.75,
            effort=10.0,
            velocity=1.4,
        ),
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=control_black,
        name="shaft_lower",
    )
    joystick.visual(
        Cylinder(radius=GIMBAL_BORE_RADIUS, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=control_black,
        name="pivot_boss",
    )
    joystick.visual(
        Cylinder(radius=0.007, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=metal_finish,
        name="shaft_upper",
    )
    joystick.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=red_button,
        name="ball",
    )
    model.articulation(
        "cabinet_to_joystick",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=joystick,
        origin=Origin(xyz=joystick_origin, rpy=(-PANEL_ANGLE, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.42,
            upper=0.42,
            effort=3.0,
            velocity=4.0,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    glass = object_model.get_part("glass")
    screen = object_model.get_part("screen")
    door = object_model.get_part("door")
    joystick = object_model.get_part("joystick")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    joystick_gimbal = object_model.get_articulation("cabinet_to_joystick")

    ctx.allow_overlap(
        cabinet,
        joystick,
        elem_a="gimbal_trim",
        elem_b="pivot_boss",
        reason="The joystick's pivot boss is intentionally nested inside the fixed gimbal trim as a simplified socketed gimbal support.",
    )

    ctx.expect_within(
        screen,
        glass,
        axes="xy",
        margin=0.0,
        name="screen stays under the glass top",
    )
    ctx.expect_gap(
        glass,
        screen,
        axis="z",
        min_gap=0.09,
        max_gap=0.13,
        name="screen remains recessed below the glass",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="x",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="door_panel",
            name="cashbox door closes flush to the cabinet end",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="yz",
            min_overlap=0.10,
            elem_a="door_panel",
            name="cashbox door covers the opening at rest",
        )
        rest_door_aabb = ctx.part_world_aabb(door)

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            open_door_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "cashbox door swings outward",
            rest_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][0] > rest_door_aabb[1][0] + 0.12,
            details=f"rest={rest_door_aabb}, open={open_door_aabb}",
        )

    with ctx.pose({joystick_gimbal: 0.0}):
        rest_ball_center = _aabb_center(ctx.part_element_world_aabb(joystick, elem="ball"))

    joystick_limits = joystick_gimbal.motion_limits
    if joystick_limits is not None and joystick_limits.upper is not None:
        with ctx.pose({joystick_gimbal: joystick_limits.upper}):
            tilted_ball_center = _aabb_center(ctx.part_element_world_aabb(joystick, elem="ball"))
        ctx.check(
            "joystick ball tilts sideways from center",
            rest_ball_center is not None
            and tilted_ball_center is not None
            and tilted_ball_center[0] > rest_ball_center[0] + 0.03,
            details=f"rest={rest_ball_center}, tilted={tilted_ball_center}",
        )

    return ctx.report()


object_model = build_object_model()
