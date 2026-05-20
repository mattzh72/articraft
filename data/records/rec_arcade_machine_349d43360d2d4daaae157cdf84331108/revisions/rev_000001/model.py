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
CABINET_DEPTH = 0.76
PLINTH_HEIGHT = 0.10
LOWER_BODY_HEIGHT = 0.86
UPPER_BODY_WIDTH = 0.68
UPPER_BODY_DEPTH = 0.54
UPPER_BODY_HEIGHT = 0.76
DECK_THICKNESS = 0.08
DECK_ROLL = -math.radians(20.0)
DECK_CENTER_Y = 0.22
DECK_CENTER_Z = 1.01
SCREEN_ROLL = math.radians(13.0)
JOYSTICK_LOCAL_Y = -0.015
START_BUTTON_LOCAL_Y = 0.070


def _rolled_mount(
    *,
    local_x: float,
    local_y: float,
    local_z: float,
    center_y: float,
    center_z: float,
    roll: float,
) -> Origin:
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    return Origin(
        xyz=(
            local_x,
            center_y + local_y * cos_r - local_z * sin_r,
            center_z + local_y * sin_r + local_z * cos_r,
        ),
        rpy=(roll, 0.0, 0.0),
    )


def _deck_mount(local_x: float, local_y: float, normal_offset: float = 0.0) -> Origin:
    return _rolled_mount(
        local_x=local_x,
        local_y=local_y,
        local_z=DECK_THICKNESS / 2.0 + normal_offset,
        center_y=DECK_CENTER_Y,
        center_z=DECK_CENTER_Z,
        roll=DECK_ROLL,
    )


def _build_control_deck_mesh() -> object:
    deck = cq.Workplane("XY").box(0.78, 0.34, DECK_THICKNESS)
    deck = (
        deck.faces(">Z")
        .workplane()
        .pushPoints([(-0.23, JOYSTICK_LOCAL_Y), (0.23, JOYSTICK_LOCAL_Y)])
        .hole(0.046)
    )
    deck = (
        deck.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.12, START_BUTTON_LOCAL_Y),
                (-0.04, START_BUTTON_LOCAL_Y),
                (0.04, START_BUTTON_LOCAL_Y),
                (0.12, START_BUTTON_LOCAL_Y),
            ]
        )
        .hole(0.045)
    )
    return mesh_from_cadquery(deck, "arcade_control_deck")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_machine")

    body_finish = model.material("body_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    marquee_finish = model.material("marquee_finish", rgba=(0.21, 0.05, 0.36, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.06, 0.12, 0.16, 1.0))
    door_finish = model.material("door_finish", rgba=(0.55, 0.57, 0.60, 1.0))
    joystick_finish = model.material("joystick_finish", rgba=(0.06, 0.06, 0.07, 1.0))
    joystick_ball_finish = model.material("joystick_ball_finish", rgba=(0.88, 0.12, 0.14, 1.0))
    button_blue = model.material("button_blue", rgba=(0.15, 0.43, 0.94, 1.0))
    button_green = model.material("button_green", rgba=(0.15, 0.78, 0.32, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.92, 0.73, 0.12, 1.0))
    button_red = model.material("button_red", rgba=(0.88, 0.15, 0.18, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material=trim_finish,
        name="plinth",
    )
    cabinet.visual(
        Box((0.76, 0.74, LOWER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + LOWER_BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="lower_body",
    )
    cabinet.visual(
        Box((UPPER_BODY_WIDTH, UPPER_BODY_DEPTH, UPPER_BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -0.08,
                PLINTH_HEIGHT + LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="upper_body",
    )
    cabinet.visual(
        Box((0.74, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, 0.04, 1.70)),
        material=body_finish,
        name="marquee_box",
    )
    cabinet.visual(
        Box((0.70, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, 0.13, 1.70)),
        material=marquee_finish,
        name="marquee_panel",
    )
    cabinet.visual(
        _build_control_deck_mesh(),
        origin=Origin(xyz=(0.0, DECK_CENTER_Y, DECK_CENTER_Z), rpy=(DECK_ROLL, 0.0, 0.0)),
        material=trim_finish,
        name="control_deck",
    )
    cabinet.visual(
        Box((0.78, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, 0.31, 0.89)),
        material=trim_finish,
        name="control_apron",
    )
    cabinet.visual(
        Box((0.58, 0.06, 0.42)),
        origin=Origin(xyz=(0.0, 0.14, 1.30), rpy=(SCREEN_ROLL, 0.0, 0.0)),
        material=trim_finish,
        name="screen_bezel",
    )
    cabinet.visual(
        Box((0.48, 0.016, 0.32)),
        origin=Origin(xyz=(0.0, 0.17, 1.28), rpy=(SCREEN_ROLL, 0.0, 0.0)),
        material=screen_finish,
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.60, 0.04, 0.15)),
        origin=Origin(xyz=(0.0, 0.13, 1.52), rpy=(0.08, 0.0, 0.0)),
        material=trim_finish,
        name="speaker_panel",
    )

    joystick_positions = (-0.23, 0.23)
    for index, local_x in enumerate(joystick_positions):
        joystick = model.part(f"joystick_{index}")
        joystick.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=joystick_finish,
            name="base_collar",
        )
        joystick.visual(
            Cylinder(radius=0.007, length=0.078),
            origin=Origin(xyz=(0.0, 0.0, 0.045)),
            material=joystick_finish,
            name="shaft",
        )
        joystick.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.093)),
            material=joystick_ball_finish,
            name="ball_top",
        )
        model.articulation(
            f"cabinet_to_joystick_{index}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=joystick,
            origin=_deck_mount(local_x, JOYSTICK_LOCAL_Y, -0.001),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.5,
                lower=-0.40,
                upper=0.40,
            ),
        )

    button_materials = (button_blue, button_green, button_yellow, button_red)
    button_x_positions = (-0.12, -0.04, 0.04, 0.12)
    for index, (local_x, button_material) in enumerate(zip(button_x_positions, button_materials)):
        button = model.part(f"start_button_{index}")
        button.visual(
            Cylinder(radius=0.029, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=trim_finish,
            name="button_bezel",
        )
        button.visual(
            Cylinder(radius=0.021, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=trim_finish,
            name="button_plunger",
        )
        button.visual(
            Cylinder(radius=0.031, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=button_material,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_start_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=_deck_mount(local_x, START_BUTTON_LOCAL_Y),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    cashbox_door = model.part("cashbox_door")
    cashbox_door.visual(
        Box((0.34, 0.015, 0.30)),
        origin=Origin(xyz=(0.17, 0.0075, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    cashbox_door.visual(
        Cylinder(radius=0.009, length=0.18),
        origin=Origin(xyz=(0.275, 0.022, 0.0)),
        material=trim_finish,
        name="door_handle",
    )
    cashbox_door.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.10, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="door_lock",
    )
    model.articulation(
        "cabinet_to_cashbox_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cashbox_door,
        origin=Origin(xyz=(-0.17, 0.37, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    cashbox_door = object_model.get_part("cashbox_door")
    door_hinge = object_model.get_articulation("cabinet_to_cashbox_door")
    joystick_0 = object_model.get_part("joystick_0")
    joystick_1 = object_model.get_part("joystick_1")
    button_0 = object_model.get_part("start_button_0")
    button_1 = object_model.get_part("start_button_1")
    button_joint_0 = object_model.get_articulation("cabinet_to_start_button_0")
    button_joint_1 = object_model.get_articulation("cabinet_to_start_button_1")

    for index in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"start_button_{index}"),
            cabinet,
            elem_a="button_bezel",
            elem_b="control_deck",
            reason="Each illuminated start button is intentionally seated into the holed control deck as a simplified bezel fit.",
        )
    for index in range(2):
        ctx.allow_overlap(
            object_model.get_part(f"joystick_{index}"),
            cabinet,
            elem_a="base_collar",
            elem_b="control_deck",
            reason="Each joystick collar is intentionally represented as a seated grommet fit into the control deck opening.",
        )

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check("cabinet_aabb_available", cabinet_aabb is not None, details=f"aabb={cabinet_aabb!r}")
    if cabinet_aabb is not None:
        mins, maxs = cabinet_aabb
        size = tuple(float(maxs[index] - mins[index]) for index in range(3))
        ctx.check("cabinet_commercial_width", 0.72 <= size[0] <= 0.86, details=f"size={size!r}")
        ctx.check("cabinet_commercial_depth", 0.72 <= size[1] <= 0.86, details=f"size={size!r}")
        ctx.check("cabinet_commercial_height", 1.65 <= size[2] <= 1.85, details=f"size={size!r}")

    ctx.expect_gap(
        cashbox_door,
        cabinet,
        axis="y",
        positive_elem="door_panel",
        negative_elem="lower_body",
        min_gap=0.0,
        max_gap=0.02,
        name="cashbox door sits on the lower front face",
    )
    ctx.expect_gap(
        cabinet,
        cashbox_door,
        axis="z",
        positive_elem="screen_glass",
        negative_elem="door_panel",
        min_gap=0.18,
        name="screen sits above the cashbox door",
    )
    ctx.expect_origin_distance(
        joystick_0,
        joystick_1,
        axes="x",
        min_dist=0.40,
        name="joysticks are separated for a two player deck",
    )

    closed_door_aabb = ctx.part_element_world_aabb(cashbox_door, elem="door_panel")
    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            open_door_aabb = ctx.part_element_world_aabb(cashbox_door, elem="door_panel")
        ctx.check(
            "cashbox door swings outward",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
            details=f"closed={closed_door_aabb!r}, open={open_door_aabb!r}",
        )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_0_limits = button_joint_0.motion_limits
    if button_0_limits is not None and button_0_limits.upper is not None:
        with ctx.pose({button_joint_0: button_0_limits.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_while_0_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "start button 0 depresses downward",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_0_pressed[2] < button_0_rest[2] - 0.0015,
            details=f"rest={button_0_rest!r}, pressed={button_0_pressed!r}",
        )
        ctx.check(
            "start buttons move independently",
            button_1_rest is not None
            and button_1_while_0_pressed is not None
            and abs(button_1_while_0_pressed[0] - button_1_rest[0]) < 1e-8
            and abs(button_1_while_0_pressed[1] - button_1_rest[1]) < 1e-8
            and abs(button_1_while_0_pressed[2] - button_1_rest[2]) < 1e-8,
            details=f"rest={button_1_rest!r}, during_other_press={button_1_while_0_pressed!r}",
        )

    button_1_limits = button_joint_1.motion_limits
    if button_1_limits is not None and button_1_limits.upper is not None:
        with ctx.pose({button_joint_1: button_1_limits.upper}):
            button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "start button 1 also depresses",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_1_pressed[2] < button_1_rest[2] - 0.0015,
            details=f"rest={button_1_rest!r}, pressed={button_1_pressed!r}",
        )

    return ctx.report()


object_model = build_object_model()
