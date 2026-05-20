from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.340
BODY_D = 0.240
BODY_H = 0.580

TANK_W = 0.252
TANK_D = 0.156
TANK_H = 0.200
TANK_TRAVEL = 0.130
TANK_Z0 = 0.050

VENT_W = 0.168
VENT_D = 0.090
VENT_CENTER_Y = 0.035
VENT_HINGE_Y = VENT_CENTER_Y + VENT_D * 0.5
def _tank_shape() -> cq.Workplane:
    wall = 0.004
    outer = (
        cq.Workplane("XY")
        .box(TANK_W, TANK_D, TANK_H)
        .translate((0.0, TANK_D * 0.5, TANK_H * 0.5))
        .edges("|Z")
        .fillet(0.016)
    )
    cavity = (
        cq.Workplane("XY")
        .box(TANK_W - 2.0 * wall, TANK_D - 2.0 * wall, TANK_H)
        .translate((0.0, TANK_D * 0.5, wall + TANK_H * 0.5))
    )
    grip_cut = (
        cq.Workplane("XY")
        .box(0.116, 0.016, 0.028)
        .translate((0.0, 0.006, 0.156))
    )
    front_window = (
        cq.Workplane("XY")
        .box(0.176, 0.004, 0.078)
        .translate((0.0, 0.002, 0.102))
    )

    return outer.cut(cavity).cut(grip_cut).cut(front_window)


def _surround_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.190, 0.145, 0.008)
        .translate((0.0, 0.0, 0.004))
        .edges("|Z")
        .fillet(0.012)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_dehumidifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    shell_grey = model.material("shell_grey", rgba=(0.82, 0.84, 0.86, 1.0))
    smoke = model.material("smoke", rgba=(0.15, 0.20, 0.23, 0.55))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.11, 0.12, 1.0))
    tank_tint = model.material("tank_tint", rgba=(0.74, 0.80, 0.86, 0.92))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=shell_white,
        name="base_shell",
    )
    body.visual(
        Box((0.012, BODY_D, BODY_H)),
        origin=Origin(xyz=(-0.164, 0.0, BODY_H * 0.5)),
        material=shell_white,
        name="left_wall",
    )
    body.visual(
        Box((0.012, BODY_D, BODY_H)),
        origin=Origin(xyz=(0.164, 0.0, BODY_H * 0.5)),
        material=shell_white,
        name="right_wall",
    )
    body.visual(
        Box((0.316, 0.012, BODY_H)),
        origin=Origin(xyz=(0.0, 0.114, BODY_H * 0.5)),
        material=shell_white,
        name="back_wall",
    )
    body.visual(
        Box((0.340, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, -0.114, 0.033)),
        material=shell_white,
        name="front_sill",
    )
    body.visual(
        Box((0.037, 0.012, 0.220)),
        origin=Origin(xyz=(-0.1515, -0.114, 0.160)),
        material=shell_white,
        name="left_jamb",
    )
    body.visual(
        Box((0.037, 0.012, 0.220)),
        origin=Origin(xyz=(0.1515, -0.114, 0.160)),
        material=shell_white,
        name="right_jamb",
    )
    body.visual(
        Box((0.340, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.114, 0.285)),
        material=shell_white,
        name="tank_header",
    )
    body.visual(
        Box((0.340, 0.012, 0.280)),
        origin=Origin(xyz=(0.0, -0.114, 0.440)),
        material=shell_white,
        name="upper_front",
    )
    body.visual(
        Box((0.316, 0.228, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=shell_grey,
        name="bay_floor",
    )
    body.visual(
        Box((0.316, 0.228, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.279)),
        material=shell_grey,
        name="bay_roof",
    )
    body.visual(
        Box((BODY_W, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, -0.065, BODY_H - 0.010)),
        material=shell_white,
        name="top_front",
    )
    body.visual(
        Box((BODY_W, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.100, BODY_H - 0.010)),
        material=shell_white,
        name="top_rear",
    )
    body.visual(
        Box((0.086, 0.090, 0.020)),
        origin=Origin(xyz=(-0.127, 0.035, BODY_H - 0.010)),
        material=shell_white,
        name="top_left_rail",
    )
    body.visual(
        Box((0.086, 0.090, 0.020)),
        origin=Origin(xyz=(0.127, 0.035, BODY_H - 0.010)),
        material=shell_white,
        name="top_right_rail",
    )

    tank = model.part("tank")
    tank.visual(
        mesh_from_cadquery(_tank_shape(), "dehumidifier_tank"),
        material=tank_tint,
        name="tank_shell",
    )

    display_surround = model.part("display_surround")
    display_surround.visual(
        mesh_from_cadquery(_surround_plate_shape(), "display_surround_plate"),
        material=shell_grey,
        name="surround_plate",
    )
    display_surround.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.092, 0.034),
                (0.126, 0.060),
                0.008,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.006,
                outer_corner_radius=0.010,
            ),
            "display_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.030, 0.004)),
        material=shell_white,
        name="screen_bezel",
    )
    display_surround.visual(
        Box((0.094, 0.036, 0.002)),
        origin=Origin(xyz=(0.0, 0.030, 0.002)),
        material=smoke,
        name="screen_glass",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.018,
                body_style="skirted",
                top_diameter=0.043,
                edge_radius=0.0015,
                side_draft_deg=6.0,
                center=False,
            ),
            "control_dial",
        ),
        material=graphite,
        name="dial_cap",
    )
    dial.visual(
        Box((0.003, 0.014, 0.001)),
        origin=Origin(xyz=(0.0, 0.020, 0.0185)),
        material=shell_grey,
        name="dial_indicator",
    )

    button_origins = (
        Origin(xyz=(0.0, 0.016, 0.008)),
        Origin(xyz=(-0.056, -0.012, 0.008)),
        Origin(xyz=(0.056, -0.012, 0.008)),
    )
    button_parts = []
    for index in range(3):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=soft_black,
            name="button_cap",
        )
        button_parts.append(button)

    exhaust_flap = model.part("exhaust_flap")
    exhaust_flap.visual(
        Box((0.160, 0.082, 0.0035)),
        origin=Origin(xyz=(0.0, -0.041, 0.00175)),
        material=shell_grey,
        name="flap_panel",
    )
    exhaust_flap.visual(
        Cylinder(radius=0.003, length=0.136),
        origin=Origin(xyz=(0.0, -0.004, 0.005), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=shell_grey,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_tank",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank,
        origin=Origin(xyz=(0.0, -0.117, TANK_Z0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=TANK_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_display_surround",
        ArticulationType.FIXED,
        parent=body,
        child=display_surround,
        origin=Origin(xyz=(0.0, -0.120, 0.405), rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "display_surround_to_dial",
        ArticulationType.CONTINUOUS,
        parent=display_surround,
        child=dial,
        origin=Origin(xyz=(0.0, -0.028, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    for index, button in enumerate(button_parts):
        model.articulation(
            f"display_surround_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display_surround,
            child=button,
            origin=button_origins[index],
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.10,
                lower=0.0,
                upper=0.003,
            ),
        )

    model.articulation(
        "body_to_exhaust_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=exhaust_flap,
        origin=Origin(xyz=(0.0, VENT_HINGE_Y, BODY_H - 0.005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tank = object_model.get_part("tank")
    flap = object_model.get_part("exhaust_flap")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")

    tank_slide = object_model.get_articulation("body_to_tank")
    flap_hinge = object_model.get_articulation("body_to_exhaust_flap")
    dial_spin = object_model.get_articulation("display_surround_to_dial")
    button_joints = [
        object_model.get_articulation("display_surround_to_button_0"),
        object_model.get_articulation("display_surround_to_button_1"),
        object_model.get_articulation("display_surround_to_button_2"),
    ]

    ctx.check(
        "dial_is_continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_spin.articulation_type!r}",
    )
    for index, joint in enumerate(button_joints):
        ctx.check(
            f"button_{index}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type!r}",
        )

    tank_limits = tank_slide.motion_limits
    if tank_limits is not None and tank_limits.upper is not None:
        closed_pos = ctx.part_world_position(tank)
        closed_aabb = ctx.part_world_aabb(tank)
        with ctx.pose({tank_slide: tank_limits.upper}):
            extended_pos = ctx.part_world_position(tank)
            extended_aabb = ctx.part_world_aabb(tank)

        ctx.check(
            "tank_starts_near_front_face",
            closed_aabb is not None and -0.123 <= float(closed_aabb[0][1]) <= -0.112,
            details=f"closed_aabb={closed_aabb!r}",
        )
        ctx.check(
            "tank_pulls_forward",
            closed_pos is not None
            and extended_pos is not None
            and float(extended_pos[1]) < float(closed_pos[1]) - 0.10,
            details=f"closed_pos={closed_pos!r}, extended_pos={extended_pos!r}",
        )
        ctx.check(
            "tank_retains_insertion",
            extended_aabb is not None and float(extended_aabb[1][1]) > -0.100,
            details=f"extended_aabb={extended_aabb!r}",
        )

    flap_limits = flap_hinge.motion_limits
    if flap_limits is not None and flap_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(flap)
        with ctx.pose({flap_hinge: flap_limits.upper}):
            open_aabb = ctx.part_world_aabb(flap)

        ctx.check(
            "exhaust_flap_opens_upward",
            closed_aabb is not None
            and open_aabb is not None
            and float(open_aabb[1][2]) > float(closed_aabb[1][2]) + 0.050,
            details=f"closed_aabb={closed_aabb!r}, open_aabb={open_aabb!r}",
        )

    for button_part, joint, index in zip((button_0, button_1, button_2), button_joints, range(3)):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"button_{index}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and float(pressed_pos[1]) > float(rest_pos[1]) + 0.002,
            details=f"rest_pos={rest_pos!r}, pressed_pos={pressed_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
