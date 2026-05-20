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


CABINET_WIDTH = 0.78
CABINET_DEPTH = 0.82
LOWER_HEIGHT = 0.95
TOTAL_HEIGHT = 1.72
PANEL_THICKNESS = 0.02
UPPER_DEPTH = 0.50
FRONT_Y = CABINET_DEPTH / 2.0 - PANEL_THICKNESS / 2.0
REAR_Y = -CABINET_DEPTH / 2.0 + PANEL_THICKNESS / 2.0

DECK_WIDTH = 0.78
DECK_DEPTH = 0.30
DECK_THICKNESS = 0.02
DECK_ROLL = -math.radians(14.0)
DECK_CENTER = (0.0, 0.19, 0.97)

SCREEN_WIDTH = 0.50
SCREEN_HEIGHT = 0.40
SCREEN_TILT = math.radians(12.0)
SCREEN_CENTER = (0.0, 0.105, 1.24)

MARQUEE_CENTER = (0.0, 0.015, 1.585)

DOOR_WIDTH = 0.32
DOOR_HEIGHT = 0.38
DOOR_THICKNESS = 0.018
DOOR_BOTTOM = 0.18

JOYSTICK_HOLE_RADIUS = 0.022
SUPPORT_OUTER_RADIUS = 0.027
SUPPORT_INNER_RADIUS = 0.018
JOYSTICK_PIVOT_LOCAL_Z = -0.020
JOYSTICK_SHAFT_RADIUS = 0.006
JOYSTICK_SHAFT_LENGTH = 0.115
JOYSTICK_BALL_RADIUS = 0.019
JOYSTICK_ACTUATOR_RADIUS = 0.009
JOYSTICK_ACTUATOR_LENGTH = 0.030

JOYSTICK_POSITIONS = (
    (-0.215, 0.01),
    (0.215, 0.01),
)


def _rot_x(point: tuple[float, float, float], roll: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(roll)
    s = math.sin(roll)
    return (x, y * c - z * s, y * s + z * c)


def _offset(
    base: tuple[float, float, float],
    delta: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (base[0] + delta[0], base[1] + delta[1], base[2] + delta[2])


def _deck_point(local_x: float, local_y: float, local_z: float = 0.0) -> tuple[float, float, float]:
    return _offset(DECK_CENTER, _rot_x((local_x, local_y, local_z), DECK_ROLL))


def _deck_panel_mesh() -> object:
    panel = cq.Workplane("XY").box(DECK_WIDTH, DECK_DEPTH, DECK_THICKNESS)
    for local_x, local_y in JOYSTICK_POSITIONS:
        cutter = (
            cq.Workplane("XY")
            .center(local_x, local_y)
            .circle(JOYSTICK_HOLE_RADIUS)
            .extrude(DECK_THICKNESS * 4.0)
            .translate((0.0, 0.0, -DECK_THICKNESS * 2.0))
        )
        panel = panel.cut(cutter)
    return mesh_from_cadquery(panel, "arcade_control_deck")


def _ring_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str) -> object:
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    return mesh_from_cadquery(ring, name)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_player_arcade_machine")

    cabinet_black = model.material("cabinet_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.21, 0.22, 0.24, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.16, 0.17, 0.18, 1.0))
    speaker_grey = model.material("speaker_grey", rgba=(0.30, 0.32, 0.34, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.07, 0.16, 0.20, 0.42))
    marquee_glow = model.material("marquee_glow", rgba=(0.82, 0.23, 0.20, 0.90))
    joystick_black = model.material("joystick_black", rgba=(0.08, 0.08, 0.09, 1.0))
    joystick_red = model.material("joystick_red", rgba=(0.78, 0.10, 0.11, 1.0))
    button_red = model.material("button_red", rgba=(0.74, 0.16, 0.15, 1.0))
    button_blue = model.material("button_blue", rgba=(0.16, 0.31, 0.70, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.83, 0.69, 0.16, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.07, 0.07, 0.08, 1.0))
    cashbox_grey = model.material("cashbox_grey", rgba=(0.36, 0.38, 0.40, 1.0))

    support_ring_mesh = _ring_mesh(
        SUPPORT_OUTER_RADIUS,
        SUPPORT_INNER_RADIUS,
        0.022,
        "arcade_joystick_support_ring",
    )
    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + PANEL_THICKNESS / 2.0, 0.0, LOWER_HEIGHT / 2.0)),
        material=cabinet_black,
        name="left_side",
    )
    cabinet.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - PANEL_THICKNESS / 2.0, 0.0, LOWER_HEIGHT / 2.0)),
        material=cabinet_black,
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * PANEL_THICKNESS, PANEL_THICKNESS, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, REAR_Y, LOWER_HEIGHT / 2.0)),
        material=cabinet_black,
        name="rear_lower",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * PANEL_THICKNESS, CABINET_DEPTH - 2.0 * PANEL_THICKNESS, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PANEL_THICKNESS / 2.0,
            )
        ),
        material=cabinet_black,
        name="floor",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * PANEL_THICKNESS, 0.44, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.18, LOWER_HEIGHT)),
        material=cabinet_black,
        name="lower_top_back",
    )
    cabinet.visual(
        Box((0.21, PANEL_THICKNESS, 0.82)),
        origin=Origin(xyz=(-0.275, FRONT_Y, 0.41)),
        material=cabinet_black,
        name="cashbox_frame_left",
    )
    cabinet.visual(
        Box((0.21, PANEL_THICKNESS, 0.82)),
        origin=Origin(xyz=(0.275, FRONT_Y, 0.41)),
        material=cabinet_black,
        name="cashbox_frame_right",
    )
    cabinet.visual(
        Box((DOOR_WIDTH, PANEL_THICKNESS, DOOR_BOTTOM)),
        origin=Origin(xyz=(0.0, FRONT_Y, DOOR_BOTTOM / 2.0)),
        material=cabinet_black,
        name="cashbox_frame_bottom",
    )
    cabinet.visual(
        Box((DOOR_WIDTH, PANEL_THICKNESS, 0.26)),
        origin=Origin(xyz=(0.0, FRONT_Y, 0.69)),
        material=cabinet_black,
        name="cashbox_frame_top",
    )
    cabinet.visual(
        Box((0.70, PANEL_THICKNESS, 0.12)),
        origin=Origin(xyz=(0.0, FRONT_Y, 0.88)),
        material=trim_grey,
        name="coin_tray_band",
    )
    cabinet.visual(
        Box((0.58, 0.40, TOTAL_HEIGHT - LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.12, 1.335)),
        material=cabinet_black,
        name="upper_body",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * PANEL_THICKNESS, PANEL_THICKNESS, TOTAL_HEIGHT - LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.31, 1.335)),
        material=cabinet_black,
        name="rear_upper",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * PANEL_THICKNESS, 0.40, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.12, TOTAL_HEIGHT - PANEL_THICKNESS / 2.0)),
        material=cabinet_black,
        name="roof",
    )
    cabinet.visual(
        _deck_panel_mesh(),
        origin=Origin(xyz=DECK_CENTER, rpy=(DECK_ROLL, 0.0, 0.0)),
        material=dark_grey,
        name="control_deck",
    )
    cabinet.visual(
        Box((0.74, 0.055, 0.11)),
        origin=Origin(xyz=(0.0, 0.27, 0.88), rpy=(DECK_ROLL, 0.0, 0.0)),
        material=trim_grey,
        name="deck_apron",
    )

    for index, (local_x, local_y) in enumerate(JOYSTICK_POSITIONS):
        cabinet.visual(
            support_ring_mesh,
            origin=Origin(
                xyz=_deck_point(local_x, local_y, 0.001),
                rpy=(DECK_ROLL, 0.0, 0.0),
            ),
            material=hinge_black,
            name=f"joystick_support_{index}",
        )

    button_layout = (
        ((-0.155, 0.028), button_red, "button_red_0"),
        ((-0.110, 0.052), button_blue, "button_blue_0"),
        ((-0.065, 0.074), button_yellow, "button_yellow_0"),
        ((0.065, 0.074), button_yellow, "button_yellow_1"),
        ((0.110, 0.052), button_blue, "button_blue_1"),
        ((0.155, 0.028), button_red, "button_red_1"),
    )
    for (local_x, local_y), material, name in button_layout:
        cabinet.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(
                xyz=_deck_point(local_x, local_y, DECK_THICKNESS / 2.0 + 0.008),
                rpy=(DECK_ROLL, 0.0, 0.0),
            ),
            material=material,
            name=name,
        )

    start_button_positions = (-0.055, 0.055)
    for index, local_x in enumerate(start_button_positions):
        cabinet.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(
                xyz=_deck_point(local_x, -0.090, DECK_THICKNESS / 2.0 + 0.004),
                rpy=(DECK_ROLL, 0.0, 0.0),
            ),
            material=speaker_grey,
            name=f"start_button_{index}",
        )

    cabinet.visual(
        Box((0.60, 0.032, 0.48)),
        origin=Origin(xyz=SCREEN_CENTER, rpy=(SCREEN_TILT, 0.0, 0.0)),
        material=trim_grey,
        name="screen_bezel",
    )
    cabinet.visual(
        Box((SCREEN_WIDTH, 0.010, SCREEN_HEIGHT)),
        origin=Origin(
            xyz=(
                SCREEN_CENTER[0],
                SCREEN_CENTER[1] + 0.007,
                SCREEN_CENTER[2] - 0.006,
            ),
            rpy=(SCREEN_TILT, 0.0, 0.0),
        ),
        material=screen_glass,
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.60, 0.10, 0.16)),
        origin=Origin(xyz=MARQUEE_CENTER),
        material=cabinet_black,
        name="marquee_box",
    )
    cabinet.visual(
        Box((0.56, 0.012, 0.10)),
        origin=Origin(xyz=(0.0, 0.072, 1.585)),
        material=marquee_glow,
        name="marquee_face",
    )
    cabinet.visual(
        Box((0.22, PANEL_THICKNESS, 0.08)),
        origin=Origin(xyz=(-0.18, FRONT_Y, 0.87)),
        material=dark_grey,
        name="coin_mech_left",
    )
    cabinet.visual(
        Box((0.22, PANEL_THICKNESS, 0.08)),
        origin=Origin(xyz=(0.18, FRONT_Y, 0.87)),
        material=dark_grey,
        name="coin_mech_right",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.07),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0 - 0.005, FRONT_Y, 0.37)),
        material=hinge_black,
        name="cashbox_hinge_mid",
    )

    cashbox_door = model.part("cashbox_door")
    cashbox_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=cashbox_grey,
        name="door_panel",
    )
    cashbox_door.visual(
        Box((DOOR_WIDTH - 0.05, 0.006, DOOR_HEIGHT - 0.05)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, -0.003, DOOR_HEIGHT / 2.0)),
        material=trim_grey,
        name="door_inset",
    )
    cashbox_door.visual(
        Box((0.06, 0.010, 0.018)),
        origin=Origin(xyz=(0.23, -0.005, 0.13)),
        material=dark_grey,
        name="cashbox_pull",
    )
    cashbox_door.visual(
        Cylinder(radius=0.0065, length=0.11),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, 0.09)),
        material=hinge_black,
        name="hinge_lower",
    )
    cashbox_door.visual(
        Cylinder(radius=0.0065, length=0.11),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT - 0.09)),
        material=hinge_black,
        name="hinge_upper",
    )

    model.articulation(
        "cabinet_to_cashbox_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cashbox_door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, FRONT_Y + PANEL_THICKNESS / 2.0, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    for index, (local_x, local_y) in enumerate(JOYSTICK_POSITIONS):
        gimbal = model.part(f"joystick_gimbal_{index}")
        gimbal.visual(
            Box((0.024, 0.006, 0.020)),
            origin=Origin(xyz=(0.0, -0.015, 0.0)),
            material=hinge_black,
            name="rear_cheek",
        )
        gimbal.visual(
            Box((0.024, 0.006, 0.020)),
            origin=Origin(xyz=(0.0, 0.015, 0.0)),
            material=hinge_black,
            name="front_cheek",
        )
        gimbal.visual(
            Box((0.006, 0.036, 0.006)),
            origin=Origin(xyz=(0.015, 0.0, -0.013)),
            material=hinge_black,
            name="lower_bridge",
        )

        stick = model.part(f"joystick_{index}")
        stick.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=joystick_black,
            name="trunnion",
        )
        stick.visual(
            Cylinder(radius=JOYSTICK_SHAFT_RADIUS, length=JOYSTICK_SHAFT_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, JOYSTICK_SHAFT_LENGTH / 2.0)),
            material=joystick_black,
            name="shaft",
        )
        stick.visual(
            Sphere(radius=JOYSTICK_BALL_RADIUS),
            origin=Origin(xyz=(0.0, 0.0, JOYSTICK_SHAFT_LENGTH + JOYSTICK_BALL_RADIUS * 0.55)),
            material=joystick_red,
            name="ball",
        )
        stick.visual(
            Cylinder(radius=JOYSTICK_ACTUATOR_RADIUS, length=JOYSTICK_ACTUATOR_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, -JOYSTICK_ACTUATOR_LENGTH / 2.0)),
            material=joystick_black,
            name="actuator",
        )

        pivot_xyz = _deck_point(local_x, local_y, JOYSTICK_PIVOT_LOCAL_Z)
        model.articulation(
            f"cabinet_to_joystick_gimbal_{index}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=gimbal,
            origin=Origin(xyz=pivot_xyz, rpy=(DECK_ROLL, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=3.0,
                lower=-0.32,
                upper=0.32,
            ),
        )
        model.articulation(
            f"joystick_gimbal_{index}_to_joystick_{index}",
            ArticulationType.REVOLUTE,
            parent=gimbal,
            child=stick,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=3.0,
                lower=-0.32,
                upper=0.32,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    cashbox_door = object_model.get_part("cashbox_door")
    cashbox_hinge = object_model.get_articulation("cabinet_to_cashbox_door")
    left_gimbal = object_model.get_articulation("cabinet_to_joystick_gimbal_0")
    left_tilt = object_model.get_articulation("joystick_gimbal_0_to_joystick_0")
    left_stick = object_model.get_part("joystick_0")

    for index in range(2):
        ctx.allow_overlap(
            cabinet,
            object_model.get_part(f"joystick_gimbal_{index}"),
            elem_a=f"joystick_support_{index}",
            elem_b="rear_cheek",
            reason="The joystick yoke is intentionally captured inside the under-deck support collar as a simplified nested gimbal mount.",
        )
        ctx.allow_overlap(
            cabinet,
            object_model.get_part(f"joystick_gimbal_{index}"),
            elem_a=f"joystick_support_{index}",
            elem_b="front_cheek",
            reason="The joystick yoke is intentionally captured inside the under-deck support collar as a simplified nested gimbal mount.",
        )

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check("commercial_arcade_height", cabinet_aabb is not None, "Expected a cabinet AABB.")
    if cabinet_aabb is not None:
        mins, maxs = cabinet_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("cabinet_is_tall", 1.64 <= size[2] <= 1.78, f"size={size!r}")
        ctx.check("cabinet_is_two_player_wide", 0.74 <= size[0] <= 0.82, f"size={size!r}")
        ctx.check("cabinet_has_coin_op_depth", 0.78 <= size[1] <= 0.88, f"size={size!r}")

    ctx.expect_overlap(
        cashbox_door,
        cabinet,
        axes="xz",
        min_overlap=0.20,
        name="cashbox door sits within the front opening footprint",
    )

    closed_door_center = _aabb_center(ctx.part_element_world_aabb(cashbox_door, elem="door_panel"))
    limits = cashbox_hinge.motion_limits
    if limits is not None and limits.upper is not None and closed_door_center is not None:
        with ctx.pose({cashbox_hinge: limits.upper}):
            open_door_center = _aabb_center(ctx.part_element_world_aabb(cashbox_door, elem="door_panel"))
        ctx.check(
            "cashbox door swings outward",
            open_door_center is not None and open_door_center[1] > closed_door_center[1] + 0.10,
            details=f"closed={closed_door_center}, open={open_door_center}",
        )

    rest_ball_center = _aabb_center(ctx.part_element_world_aabb(left_stick, elem="ball"))
    with ctx.pose({left_gimbal: 0.22, left_tilt: -0.18}):
        tilted_ball_center = _aabb_center(ctx.part_element_world_aabb(left_stick, elem="ball"))
    if rest_ball_center is not None and tilted_ball_center is not None:
        displacement = math.dist(rest_ball_center, tilted_ball_center)
        ctx.check(
            "joystick ball displaces under tilt",
            displacement > 0.02,
            details=f"rest={rest_ball_center}, tilted={tilted_ball_center}, displacement={displacement:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
