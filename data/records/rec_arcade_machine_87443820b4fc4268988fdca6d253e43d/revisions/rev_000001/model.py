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


CABINET_WIDTH = 0.82
CABINET_FRONT_Y = 0.39
CABINET_HEIGHT = 1.74

DOOR_WIDTH = 0.42
DOOR_HEIGHT = 0.34
DOOR_THICKNESS = 0.018
DOOR_CENTER_Z = 0.45
DOOR_AXIS_Y = 0.389

FLAP_WIDTH = 0.075
FLAP_HEIGHT = 0.045
FLAP_THICKNESS = 0.012
FLAP_CENTER_X = 0.145
FLAP_TOP_Z = 0.722
FLAP_AXIS_Y = 0.402

CONTROL_PANEL_ANGLE = math.atan2(0.12, 0.19)
SCREEN_ANGLE = math.atan2(0.06, 0.26)


def _cabinet_mesh() -> object:
    side_profile = [
        (-0.37, 0.00),
        (0.39, 0.00),
        (0.39, 0.92),
        (0.27, 0.97),
        (0.08, 1.09),
        (0.02, 1.23),
        (-0.04, 1.49),
        (-0.02, 1.63),
        (-0.15, 1.74),
        (-0.37, 1.74),
    ]

    shell = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(CABINET_WIDTH / 2.0, both=True)
    )

    door_opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.31, DOOR_CENTER_Z))
        .box(DOOR_WIDTH + 0.028, 0.16, DOOR_HEIGHT + 0.04)
    )

    return shell.cut(door_opening)


def _panel_normal_offset(distance: float) -> tuple[float, float]:
    return (
        math.sin(CONTROL_PANEL_ANGLE) * distance,
        math.cos(CONTROL_PANEL_ANGLE) * distance,
    )


def _control_panel_y(z_pos: float) -> float:
    z0 = 0.97
    y0 = 0.27
    slope = -0.19 / 0.12
    return y0 + (z_pos - z0) * slope


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_machine")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    bezel_finish = model.material("bezel_finish", rgba=(0.03, 0.03, 0.04, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.06, 0.08, 0.12, 1.0))
    marquee_finish = model.material("marquee_finish", rgba=(0.82, 0.72, 0.22, 1.0))
    coin_finish = model.material("coin_finish", rgba=(0.56, 0.58, 0.62, 1.0))
    door_finish = model.material("door_finish", rgba=(0.54, 0.08, 0.10, 1.0))
    joystick_finish = model.material("joystick_finish", rgba=(0.85, 0.10, 0.12, 1.0))
    joystick_alt = model.material("joystick_alt", rgba=(0.10, 0.35, 0.80, 1.0))
    button_finish = model.material("button_finish", rgba=(0.90, 0.90, 0.92, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_mesh(), "arcade_cabinet_shell"),
        material=cabinet_finish,
        name="cabinet_shell",
    )

    cabinet.visual(
        Box((0.64, 0.020, 0.48)),
        origin=Origin(xyz=(0.0, -0.008, 1.36), rpy=(SCREEN_ANGLE, 0.0, 0.0)),
        material=bezel_finish,
        name="screen_bezel",
    )
    cabinet.visual(
        Box((0.55, 0.010, 0.40)),
        origin=Origin(xyz=(0.0, -0.003, 1.36), rpy=(SCREEN_ANGLE, 0.0, 0.0)),
        material=screen_finish,
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.70, 0.016, 0.20)),
        origin=Origin(xyz=(0.0, -0.010, 1.60), rpy=(0.10, 0.0, 0.0)),
        material=marquee_finish,
        name="marquee_panel",
    )
    cabinet.visual(
        Box((0.18, 0.004, 0.10)),
        origin=Origin(xyz=(0.11, 0.392, 0.72)),
        material=coin_finish,
        name="coin_plate",
    )
    cabinet.visual(
        Box((0.070, 0.003, 0.007)),
        origin=Origin(xyz=(0.075, 0.394, 0.758)),
        material=bezel_finish,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.030, 0.016, 0.320)),
        origin=Origin(xyz=(-0.225, 0.381, DOOR_CENTER_Z)),
        material=bezel_finish,
        name="door_jamb",
    )

    joystick_surface_z = 1.03
    joystick_surface_y = _control_panel_y(joystick_surface_z)
    washer_y_off, washer_z_off = _panel_normal_offset(0.0)
    shaft_y_off, shaft_z_off = _panel_normal_offset(0.058)
    ball_y_off, ball_z_off = _panel_normal_offset(0.133)
    for index, x_pos in enumerate((-0.22, 0.22)):
        ball_material = joystick_finish if index == 0 else joystick_alt
        cabinet.visual(
            Cylinder(radius=0.036, length=0.003),
            origin=Origin(
                xyz=(x_pos, joystick_surface_y + washer_y_off, joystick_surface_z + washer_z_off),
                rpy=(-CONTROL_PANEL_ANGLE, 0.0, 0.0),
            ),
            material=bezel_finish,
            name=f"joystick_{index}_washer",
        )
        cabinet.visual(
            Cylinder(radius=0.0065, length=0.120),
            origin=Origin(
                xyz=(x_pos, joystick_surface_y + shaft_y_off, joystick_surface_z + shaft_z_off),
                rpy=(-CONTROL_PANEL_ANGLE, 0.0, 0.0),
            ),
            material=coin_finish,
            name=f"joystick_{index}_shaft",
        )
        cabinet.visual(
            Sphere(radius=0.026),
            origin=Origin(
                xyz=(x_pos, joystick_surface_y + ball_y_off, joystick_surface_z + ball_z_off),
            ),
            material=ball_material,
            name=f"joystick_{index}_ball",
        )

    start_surface_z = 1.045
    start_surface_y = _control_panel_y(start_surface_z)
    start_y_off, start_z_off = _panel_normal_offset(0.005)
    for index, x_pos in enumerate((-0.08, 0.08)):
        cabinet.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(
                xyz=(x_pos, start_surface_y + start_y_off, start_surface_z + start_z_off),
                rpy=(-CONTROL_PANEL_ANGLE, 0.0, 0.0),
            ),
            material=button_finish,
            name=f"start_button_{index}",
        )

    door = model.part("cashbox_door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, -DOOR_THICKNESS / 2.0, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(
            xyz=(DOOR_WIDTH - 0.055, 0.009, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=coin_finish,
        name="door_pull",
    )
    door.visual(
        Box((0.010, 0.016, 0.320)),
        origin=Origin(xyz=(0.005, -0.008, 0.0)),
        material=bezel_finish,
        name="door_leaf",
    )

    flap = model.part("return_flap")
    flap.visual(
        Box((FLAP_WIDTH, FLAP_THICKNESS, FLAP_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -FLAP_THICKNESS / 2.0, -FLAP_HEIGHT / 2.0),
        ),
        material=coin_finish,
        name="flap_panel",
    )

    model.articulation(
        "cabinet_to_cashbox_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, DOOR_AXIS_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=15.0, velocity=1.2),
    )
    model.articulation(
        "cabinet_to_return_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(FLAP_CENTER_X, FLAP_AXIS_Y, FLAP_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("cashbox_door")
    flap = object_model.get_part("return_flap")
    door_hinge = object_model.get_articulation("cabinet_to_cashbox_door")
    flap_hinge = object_model.get_articulation("cabinet_to_return_flap")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check("cabinet_aabb_present", cabinet_aabb is not None, "Expected cabinet bounds.")
    if cabinet_aabb is not None:
        mins, maxs = cabinet_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("commercial_arcade_width", 0.76 <= size[0] <= 0.90, f"size={size!r}")
        ctx.check("commercial_arcade_depth", 0.70 <= size[1] <= 0.84, f"size={size!r}")
        ctx.check("commercial_arcade_height", 1.68 <= size[2] <= 1.82, f"size={size!r}")

    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen_glass")
    door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "screen_sits_above_cashbox_door",
        screen_aabb is not None
        and door_aabb is not None
        and screen_aabb[0][2] > door_aabb[1][2] + 0.22,
        details=f"screen={screen_aabb!r}, door={door_aabb!r}",
    )

    joy0_aabb = ctx.part_element_world_aabb(cabinet, elem="joystick_0_ball")
    joy1_aabb = ctx.part_element_world_aabb(cabinet, elem="joystick_1_ball")
    ctx.check(
        "twin_joystick_spacing",
        joy0_aabb is not None
        and joy1_aabb is not None
        and abs(((joy1_aabb[0][0] + joy1_aabb[1][0]) / 2.0) - ((joy0_aabb[0][0] + joy0_aabb[1][0]) / 2.0))
        >= 0.38,
        details=f"joy0={joy0_aabb!r}, joy1={joy1_aabb!r}",
    )

    ctx.expect_contact(
        flap,
        cabinet,
        elem_a="flap_panel",
        elem_b="coin_plate",
        name="return_flap_seats_on_coin_plate",
    )

    closed_door = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.15}):
        opened_door = ctx.part_world_aabb(door)
    ctx.check(
        "cashbox_door_opens_outward",
        closed_door is not None
        and opened_door is not None
        and opened_door[1][1] > closed_door[1][1] + 0.16,
        details=f"closed={closed_door!r}, opened={opened_door!r}",
    )

    closed_flap = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 0.70}):
        opened_flap = ctx.part_world_aabb(flap)
    ctx.check(
        "coin_return_flap_swings_outward",
        closed_flap is not None
        and opened_flap is not None
        and opened_flap[1][1] > closed_flap[1][1] + 0.015,
        details=f"closed={closed_flap!r}, opened={opened_flap!r}",
    )

    return ctx.report()


object_model = build_object_model()
