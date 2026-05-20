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

BODY_W = 0.360
BODY_D = 0.240
BODY_H = 0.580
BODY_LOWER_H = 0.276
BODY_UPPER_H = BODY_H - BODY_LOWER_H

TANK_W = 0.278
TANK_D = 0.166
TANK_H = 0.232
TANK_TRAVEL = 0.145
TANK_FRONT_Y = 0.118
TANK_BOTTOM_Z = 0.022

UI_CENTER_Z = 0.468
UI_PANEL_W = 0.192
UI_PANEL_H = 0.104
UI_PANEL_T = 0.004
UI_PANEL_Y = BODY_D / 2.0

HANDLE_AXIS_Z = BODY_H + 0.005
HANDLE_AXIS_Y = 0.004


def _upper_body_shape() -> object:
    return (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_UPPER_H)
        .translate((0.0, 0.0, BODY_LOWER_H + BODY_UPPER_H / 2.0))
        .edges("|Z")
        .fillet(0.026)
    )


def _tank_shape() -> object:
    front_wall = 0.007
    side_wall = 0.004
    bottom_wall = 0.004

    outer = (
        cq.Workplane("XY")
        .box(TANK_W, TANK_D, TANK_H)
        .translate((0.0, -TANK_D / 2.0, TANK_H / 2.0))
        .edges("|Z")
        .fillet(0.016)
    )

    inner = (
        cq.Workplane("XY")
        .box(TANK_W - 2.0 * side_wall, TANK_D - front_wall - side_wall, TANK_H)
        .translate(
            (
                0.0,
                -(front_wall + (TANK_D - front_wall - side_wall) / 2.0),
                bottom_wall + TANK_H / 2.0,
            )
        )
    )

    grip_cut = (
        cq.Workplane("XY")
        .box(0.128, 0.022, 0.030)
        .translate((0.0, 0.006, 0.145))
    )

    sight_cut = (
        cq.Workplane("XY")
        .box(0.050, 0.012, 0.072)
        .translate((0.108, 0.004, 0.102))
    )

    return outer.cut(inner).cut(grip_cut).cut(sight_cut)


def _interface_panel_shape() -> object:
    panel = cq.Workplane("XZ").rect(UI_PANEL_W, UI_PANEL_H).extrude(UI_PANEL_T)

    dial_hole = cq.Workplane("XZ").circle(0.020).extrude(UI_PANEL_T + 0.002).translate((0.0, -0.001, 0.0))
    panel = panel.cut(dial_hole)

    hole_positions = [
        (-0.050, 0.027),
        (0.000, 0.043),
        (0.050, 0.027),
        (-0.040, -0.030),
        (0.040, -0.030),
    ]
    for x_pos, z_pos in hole_positions:
        hole = (
            cq.Workplane("XZ")
            .rect(0.014, 0.010)
            .extrude(UI_PANEL_T + 0.002)
            .translate((x_pos, -0.001, z_pos))
        )
        panel = panel.cut(hole)

    return panel


def _dial_shape() -> object:
    ring = cq.Workplane("XZ").circle(0.034).circle(0.022).extrude(0.006).translate((0.0, UI_PANEL_T, 0.0))
    collar = cq.Workplane("XZ").circle(0.023).extrude(UI_PANEL_T)
    return ring.union(collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_dehumidifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    shell_shadow = model.material("shell_shadow", rgba=(0.84, 0.86, 0.88, 1.0))
    panel_black = model.material("panel_black", rgba=(0.11, 0.12, 0.13, 1.0))
    control_black = model.material("control_black", rgba=(0.15, 0.16, 0.17, 1.0))
    button_grey = model.material("button_grey", rgba=(0.80, 0.82, 0.84, 1.0))
    tank_white = model.material("tank_white", rgba=(0.96, 0.97, 0.98, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_upper_body_shape(), "dehumidifier_upper_body"),
        material=shell_white,
        name="upper_shell",
    )
    body.visual(
        Box((0.037, BODY_D, BODY_LOWER_H)),
        origin=Origin(xyz=(-0.1615, 0.0, BODY_LOWER_H / 2.0)),
        material=shell_white,
        name="side_cheek_0",
    )
    body.visual(
        Box((0.037, BODY_D, BODY_LOWER_H)),
        origin=Origin(xyz=(0.1615, 0.0, BODY_LOWER_H / 2.0)),
        material=shell_white,
        name="side_cheek_1",
    )
    body.visual(
        Box((0.286, 0.038, BODY_LOWER_H)),
        origin=Origin(xyz=(0.0, -0.101, BODY_LOWER_H / 2.0)),
        material=shell_white,
        name="rear_lower",
    )
    body.visual(
        Box((0.286, 0.206, 0.022)),
        origin=Origin(xyz=(0.0, 0.009, 0.011)),
        material=shell_shadow,
        name="tank_floor",
    )
    body.visual(
        Box((0.286, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.112, 0.005)),
        material=shell_shadow,
        name="front_threshold",
    )

    tank = model.part("tank")
    tank.visual(
        mesh_from_cadquery(_tank_shape(), "dehumidifier_tank"),
        material=tank_white,
        name="tank_shell",
    )

    panel = model.part("interface_panel")
    panel.visual(
        mesh_from_cadquery(_interface_panel_shape(), "dehumidifier_interface_panel"),
        material=panel_black,
        name="panel_face",
    )

    handle = model.part("carry_handle")
    handle.visual(
        Cylinder(radius=0.005, length=0.244),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_black,
        name="pivot_rod",
    )
    handle.visual(
        Box((0.022, 0.054, 0.020)),
        origin=Origin(xyz=(-0.111, 0.024, 0.010)),
        material=control_black,
        name="arm_0",
    )
    handle.visual(
        Box((0.022, 0.054, 0.020)),
        origin=Origin(xyz=(0.111, 0.024, 0.010)),
        material=control_black,
        name="arm_1",
    )
    handle.visual(
        Box((0.236, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.048, 0.010)),
        material=control_black,
        name="grip",
    )

    dial = model.part("ring_dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "dehumidifier_ring_dial"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=control_black,
        name="dial_ring",
    )

    button_positions = [
        (-0.050, 0.495),
        (0.000, 0.511),
        (0.050, 0.495),
        (-0.040, 0.438),
        (0.040, 0.438),
    ]

    button_parts = []
    for index, (x_pos, z_pos) in enumerate(button_positions):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.024, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, UI_PANEL_T + 0.003, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.014, UI_PANEL_T, 0.010)),
            origin=Origin(xyz=(0.0, UI_PANEL_T / 2.0, 0.0)),
            material=button_grey,
            name="button_stem",
        )
        button_parts.append((button, x_pos, z_pos))

    model.articulation(
        "body_to_tank",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank,
        origin=Origin(xyz=(0.0, TANK_FRONT_Y, TANK_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=TANK_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_interface_panel",
        ArticulationType.FIXED,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.0, UI_PANEL_Y, UI_CENTER_Z)),
    )

    model.articulation(
        "body_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_AXIS_Y, HANDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.32,
        ),
    )

    model.articulation(
        "panel_to_ring_dial",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=8.0,
        ),
    )

    for index, (button, x_pos, z_pos) in enumerate(button_parts):
        model.articulation(
            f"panel_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, z_pos - UI_CENTER_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tank = object_model.get_part("tank")
    handle = object_model.get_part("carry_handle")
    panel = object_model.get_part("interface_panel")
    dial_joint = object_model.get_articulation("panel_to_ring_dial")
    tank_joint = object_model.get_articulation("body_to_tank")
    handle_joint = object_model.get_articulation("body_to_carry_handle")

    ctx.check(
        "dial articulation is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )

    ctx.expect_overlap(
        tank,
        body,
        axes="xz",
        min_overlap=0.22,
        name="tank sits within the body footprint",
    )
    ctx.expect_overlap(
        tank,
        body,
        axes="y",
        min_overlap=0.08,
        name="tank is inserted into the body when closed",
    )

    tank_rest = ctx.part_world_position(tank)
    with ctx.pose({tank_joint: TANK_TRAVEL}):
        ctx.expect_overlap(
            tank,
            body,
            axes="xz",
            min_overlap=0.22,
            name="tank stays laterally aligned at full extension",
        )
        ctx.expect_overlap(
            tank,
            body,
            axes="y",
            min_overlap=0.02,
            name="tank retains insertion at full extension",
        )
        tank_extended = ctx.part_world_position(tank)
    ctx.check(
        "tank pulls forward",
        tank_rest is not None
        and tank_extended is not None
        and tank_extended[1] > tank_rest[1] + 0.12,
        details=f"rest={tank_rest!r}, extended={tank_extended!r}",
    )

    body_aabb = ctx.part_world_aabb(body)
    handle_closed = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        handle_open = ctx.part_world_aabb(handle)
    ctx.check(
        "handle folds flat near the top cap",
        body_aabb is not None
        and handle_closed is not None
        and handle_closed[1][2] <= body_aabb[1][2] + 0.030,
        details=f"body={body_aabb!r}, closed={handle_closed!r}",
    )
    ctx.check(
        "handle lifts upward when opened",
        handle_closed is not None
        and handle_open is not None
        and handle_open[1][2] > handle_closed[1][2] + 0.035,
        details=f"closed={handle_closed!r}, open={handle_open!r}",
    )

    for index in range(5):
        button = object_model.get_part(f"program_button_{index}")
        button_joint = object_model.get_articulation(f"panel_to_program_button_{index}")
        button_rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            button_pressed = ctx.part_world_position(button)
        ctx.check(
            f"program button {index} presses inward",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] < button_rest[1] - 0.001,
            details=f"rest={button_rest!r}, pressed={button_pressed!r}",
        )

    ctx.expect_contact(
        panel,
        body,
        contact_tol=0.006,
        name="interface panel mounts to the body front",
    )

    return ctx.report()


object_model = build_object_model()
