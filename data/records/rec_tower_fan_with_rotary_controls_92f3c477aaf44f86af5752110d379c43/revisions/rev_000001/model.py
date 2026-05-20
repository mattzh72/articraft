from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_WIDTH = 0.34
BASE_DEPTH = 0.24
BASE_HEIGHT = 0.055
BASE_CORNER = 0.042
PEDESTAL_WIDTH = 0.11
PEDESTAL_DEPTH = 0.085
PEDESTAL_HEIGHT = 0.018

BODY_WIDTH = 0.135
BODY_DEPTH = 0.105
BODY_HEIGHT = 0.93
BODY_CORNER = 0.022
WALL = 0.0035
BOTTOM_LIP = 0.010
TOP_THICKNESS = 0.012

VENT_WIDTH = 0.089
VENT_HEIGHT = 0.735
VENT_BOTTOM = 0.105
GRILLE_THICKNESS = 0.0032
GRILLE_FRAME = 0.004
GRILLE_SLOT = 0.0034
GRILLE_PITCH = 0.0072

HANDLE_WIDTH = 0.082
HANDLE_DEPTH = 0.034
HANDLE_HEIGHT = 0.030
HANDLE_BOTTOM = BODY_HEIGHT - 0.048

SPEED_DIAL_X = -0.030
TIMER_DIAL_X = 0.030
CONTROL_Y = 0.006
SPEED_DIAL_DIAMETER = 0.048
TIMER_DIAL_DIAMETER = 0.039
SPEED_DIAL_HEIGHT = 0.026
TIMER_DIAL_HEIGHT = 0.022
RECESS_DEPTH = 0.003

BLOWER_RADIUS = 0.036
BLOWER_INNER_RADIUS = 0.016
BLOWER_HEIGHT = 0.720
BLOWER_CENTER_Z = VENT_BOTTOM + VENT_HEIGHT / 2.0


def _build_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BASE_CORNER)
    )
    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.015)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    return foot.union(pedestal)


def _build_body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_WIDTH - 2.0 * WALL,
            BODY_DEPTH - 2.0 * WALL,
            BODY_HEIGHT - BOTTOM_LIP - TOP_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(BODY_CORNER - WALL)
        .translate((0.0, 0.0, BOTTOM_LIP))
    )
    shell = outer.cut(inner)

    vent_cutter = (
        cq.Workplane("XY")
        .box(VENT_WIDTH, BODY_DEPTH * 0.55, VENT_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, BODY_DEPTH / 2.0 - BODY_DEPTH * 0.55 / 2.0 + 0.002, VENT_BOTTOM))
    )
    shell = shell.cut(vent_cutter)

    handle_cutter = (
        cq.Workplane("XY")
        .box(HANDLE_WIDTH, HANDLE_DEPTH, HANDLE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate(
            (
                0.0,
                -BODY_DEPTH / 2.0 + HANDLE_DEPTH / 2.0 - 0.002,
                HANDLE_BOTTOM,
            )
        )
    )
    shell = shell.cut(handle_cutter)

    for x_pos, recess_radius, shaft_radius in (
        (SPEED_DIAL_X, SPEED_DIAL_DIAMETER * 0.62 / 2.0, 0.0045),
        (TIMER_DIAL_X, TIMER_DIAL_DIAMETER * 0.62 / 2.0, 0.0040),
    ):
        recess = (
            cq.Workplane("XY")
            .circle(recess_radius)
            .extrude(RECESS_DEPTH)
            .translate((x_pos, CONTROL_Y, BODY_HEIGHT - RECESS_DEPTH))
        )
        shaft_hole = (
            cq.Workplane("XY")
            .circle(shaft_radius)
            .extrude(TOP_THICKNESS + 0.002)
            .translate((x_pos, CONTROL_Y, BODY_HEIGHT - TOP_THICKNESS - 0.001))
        )
        shell = shell.cut(recess).cut(shaft_hole)

    return shell


def _build_grille_shape() -> cq.Workplane:
    grille = cq.Workplane("XY").box(
        VENT_WIDTH - 0.003,
        VENT_HEIGHT - 0.008,
        GRILLE_THICKNESS,
    )
    slot_height = VENT_HEIGHT - 0.008 - 2.0 * GRILLE_FRAME
    x_limit = (VENT_WIDTH - 0.003) / 2.0 - GRILLE_FRAME - GRILLE_SLOT / 2.0
    x_pos = -x_limit
    while x_pos <= x_limit + 1e-9:
        slot = cq.Workplane("XY").box(GRILLE_SLOT, slot_height, GRILLE_THICKNESS + 0.002)
        grille = grille.cut(slot.translate((x_pos, 0.0, 0.0)))
        x_pos += GRILLE_PITCH
    return grille


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    base_finish = model.material("base_finish", color=(0.23, 0.24, 0.26))
    shell_finish = model.material("shell_finish", color=(0.90, 0.91, 0.92))
    grille_finish = model.material("grille_finish", color=(0.84, 0.85, 0.86))
    dial_finish = model.material("dial_finish", color=(0.18, 0.19, 0.20))
    blower_finish = model.material("blower_finish", color=(0.18, 0.18, 0.19))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "tower_fan_base"),
        material=base_finish,
        name="base_shell",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "tower_fan_body"),
        material=shell_finish,
        name="body_shell",
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_cadquery(_build_grille_shape(), "tower_fan_grille"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_finish,
        name="grille_panel",
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                BLOWER_RADIUS,
                BLOWER_INNER_RADIUS,
                BLOWER_HEIGHT,
                28,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
            ),
            "tower_fan_blower",
        ),
        material=blower_finish,
        name="blower_wheel",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                SPEED_DIAL_DIAMETER,
                SPEED_DIAL_HEIGHT,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.054, 0.004, flare=0.06),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "tower_fan_speed_dial",
        ),
        material=dial_finish,
        name="speed_cap",
    )
    speed_dial.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dial_finish,
        name="speed_shaft",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                TIMER_DIAL_DIAMETER,
                TIMER_DIAL_HEIGHT,
                body_style="tapered",
                top_diameter=0.028,
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "tower_fan_timer_dial",
        ),
        material=dial_finish,
        name="timer_cap",
    )
    timer_dial.visual(
        Cylinder(radius=0.0036, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dial_finish,
        name="timer_shaft",
    )

    oscillation = model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.85,
            upper=0.85,
        ),
    )

    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH / 2.0 - WALL - GRILLE_THICKNESS / 2.0,
                VENT_BOTTOM + VENT_HEIGHT / 2.0,
            )
        ),
    )

    blower_joint = model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, BLOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=28.0),
    )

    speed_joint = model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=(SPEED_DIAL_X, CONTROL_Y, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    timer_joint = model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(TIMER_DIAL_X, CONTROL_Y, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    oscillation.meta["qc_samples"] = [-0.65, 0.0, 0.65]
    blower_joint.meta["qc_samples"] = [0.0, 2.0]
    speed_joint.meta["qc_samples"] = [0.0, 1.5]
    timer_joint.meta["qc_samples"] = [0.0, 2.3]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    speed_dial = object_model.get_part("speed_dial")
    timer_dial = object_model.get_part("timer_dial")

    oscillation = object_model.get_articulation("base_to_body")
    speed_joint = object_model.get_articulation("body_to_speed_dial")
    timer_joint = object_model.get_articulation("body_to_timer_dial")

    ctx.allow_isolated_part(
        blower,
        reason="The internal crossflow blower is carried on hidden end bearings inside the tower body.",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        margin=0.014,
        name="blower stays centered inside the tower body",
    )
    ctx.expect_gap(
        speed_dial,
        body,
        axis="z",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem="speed_cap",
        negative_elem="body_shell",
        name="speed dial seats on the top cap",
    )
    ctx.expect_gap(
        timer_dial,
        body,
        axis="z",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem="timer_cap",
        negative_elem="body_shell",
        name="timer dial seats on the top cap",
    )
    ctx.expect_origin_distance(
        speed_dial,
        timer_dial,
        axes="x",
        min_dist=0.050,
        name="top dials remain clearly separated",
    )

    limits = oscillation.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({oscillation: limits.upper, speed_joint: 1.0, timer_joint: 2.0}):
            ctx.expect_gap(
                body,
                base,
                axis="z",
                min_gap=0.0,
                max_gap=0.025,
                name="body clears the base while oscillated",
            )
            ctx.expect_within(
                blower,
                body,
                axes="xy",
                margin=0.014,
                name="blower remains inside body at oscillation limit",
            )

    return ctx.report()


object_model = build_object_model()
