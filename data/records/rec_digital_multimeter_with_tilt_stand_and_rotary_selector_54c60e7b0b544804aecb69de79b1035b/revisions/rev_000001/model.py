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


BODY_W = 0.086
BODY_D = 0.022
BODY_H = 0.176
SHELL_W = 0.100
SHELL_D = 0.030
SHELL_H = 0.192

FRONT_Y = -BODY_D / 2.0
BACK_Y = BODY_D / 2.0
PANEL_Y = FRONT_Y + 0.0032
BUTTON_Z = 0.017
SELECTOR_Z = -0.022
DISPLAY_Z = 0.056

STAND_W = 0.072
STAND_H = 0.126
STAND_T = 0.0038
STAND_PIVOT_Z = -BODY_H / 2.0 + 0.018
STAND_PIVOT_Y = BACK_Y - 0.0025


def _front_mount(x: float, z: float, *, y: float = PANEL_Y) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _make_body_core() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    body = body.edges("|Z").fillet(0.0025)

    front_recess = (
        cq.Workplane("XY")
        .box(0.073, 0.0042, 0.142)
        .translate((0.0, FRONT_Y + 0.0021, 0.018))
    )
    button_slot = (
        cq.Workplane("XY")
        .box(0.070, 0.0090, 0.010)
        .translate((0.0, FRONT_Y + 0.0045, BUTTON_Z))
    )
    rear_stand_recess = (
        cq.Workplane("XY")
        .box(0.074, 0.0055, 0.136)
        .translate((0.0, BACK_Y - 0.00275, 0.003))
    )

    return body.cut(front_recess).cut(button_slot).cut(rear_stand_recess)


def _make_bumper_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(SHELL_W, SHELL_D, SHELL_H)
    shell = shell.edges("|Z").fillet(0.0050)

    front_window = (
        cq.Workplane("XY")
        .box(0.081, 0.040, 0.166)
        .translate((0.0, FRONT_Y + 0.002, 0.010))
    )
    rear_window = (
        cq.Workplane("XY")
        .box(0.074, 0.040, 0.148)
        .translate((0.0, BACK_Y - 0.002, 0.002))
    )

    return shell.cut(front_window).cut(rear_window)


def _make_stand_shape() -> cq.Workplane:
    stand = cq.Workplane("XY").box(STAND_W, STAND_T, STAND_H, centered=(True, True, False))
    stand = stand.edges("|Z").fillet(0.0008)

    stand_window = (
        cq.Workplane("XY")
        .box(STAND_W - 0.018, STAND_T * 4.0, STAND_H - 0.046, centered=(True, True, False))
        .translate((0.0, 0.0, 0.028))
    )
    top_foot = (
        cq.Workplane("XY")
        .box(STAND_W * 0.58, STAND_T * 1.35, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, STAND_H - 0.010))
    )
    lower_ears = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-STAND_W * 0.38, 0.0),
                (STAND_W * 0.38, 0.0),
            ]
        )
        .box(0.014, 0.008, 0.012, centered=(True, True, False), combine=True)
    )

    return stand.cut(stand_window).union(top_foot).union(lower_ears)


def _make_selector_knob():
    knob = KnobGeometry(
        0.039,
        0.019,
        body_style="skirted",
        top_diameter=0.031,
        skirt=KnobSkirt(0.048, 0.0048, flare=0.10),
        grip=KnobGrip(style="knurled", count=34, depth=0.0014, helix_angle_deg=22.0),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(knob, "multimeter_selector_knob")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_multimeter")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    shell_yellow = model.material("shell_yellow", rgba=(0.90, 0.73, 0.14, 1.0))
    dial_grey = model.material("dial_grey", rgba=(0.58, 0.60, 0.62, 1.0))
    display_glass = model.material("display_glass", rgba=(0.22, 0.37, 0.33, 0.62))
    lcd_tint = model.material("lcd_tint", rgba=(0.66, 0.74, 0.56, 1.0))
    soft_key_grey = model.material("soft_key_grey", rgba=(0.73, 0.76, 0.78, 1.0))
    terminal_black = model.material("terminal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    terminal_red = model.material("terminal_red", rgba=(0.73, 0.09, 0.09, 1.0))
    terminal_orange = model.material("terminal_orange", rgba=(0.88, 0.40, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_core(), "multimeter_body_core"),
        material=charcoal,
        name="core_shell",
    )
    body.visual(
        mesh_from_cadquery(_make_bumper_shell(), "multimeter_bumper_shell"),
        material=shell_yellow,
        name="bumper_shell",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.0018),
        origin=Origin(xyz=(0.0, PANEL_Y - 0.0009, SELECTOR_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_grey,
        name="dial_bezel",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.0012),
        origin=Origin(xyz=(0.0, PANEL_Y - 0.0006, SELECTOR_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="dial_face",
    )

    terminal_xs = (-0.028, -0.010, 0.010, 0.029)
    terminal_materials = (terminal_black, terminal_orange, terminal_black, terminal_red)
    for index, (x_pos, terminal_material) in enumerate(zip(terminal_xs, terminal_materials)):
        body.visual(
            Cylinder(radius=0.0066, length=0.0025),
            origin=Origin(
                xyz=(x_pos, FRONT_Y - 0.00125, -0.071),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=terminal_material,
            name=f"terminal_{index}",
        )

    for index, pad_x in enumerate((-0.039, 0.039)):
        body.visual(
            Cylinder(radius=0.0032, length=0.004),
            origin=Origin(
                xyz=(pad_x, BACK_Y, STAND_PIVOT_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=charcoal,
            name=f"pivot_pad_{index}",
        )

    display = model.part("display")
    display.visual(
        Box((0.050, 0.031, 0.0034)),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material=display_glass,
        name="glass",
    )
    display.visual(
        Box((0.039, 0.019, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0031)),
        material=lcd_tint,
        name="lcd",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=_front_mount(0.0, DISPLAY_Z, y=FRONT_Y + 0.0042),
    )

    selector = model.part("selector")
    selector.visual(_make_selector_knob(), material=charcoal, name="knob")
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=_front_mount(0.0, SELECTOR_Z),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    key_x_positions = (-0.028, -0.014, 0.0, 0.014, 0.028)
    for index, x_pos in enumerate(key_x_positions):
        key = model.part(f"soft_key_{index}")
        key.visual(
            Box((0.0115, 0.0052, 0.0046)),
            origin=Origin(xyz=(0.0, 0.0, 0.0023)),
            material=soft_key_grey,
            name="cap",
        )
        key.visual(
            Box((0.0080, 0.0032, 0.0070)),
            origin=Origin(xyz=(0.0, 0.0, -0.0035)),
            material=soft_key_grey,
            name="stem",
        )
        model.articulation(
            f"body_to_soft_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=_front_mount(x_pos, BUTTON_Z),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0018,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand_shape(), "multimeter_tilt_stand"),
        origin=Origin(xyz=(0.0, 0.0019, 0.0)),
        material=charcoal,
        name="stand_frame",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, STAND_PIVOT_Y, STAND_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    display = object_model.get_part("display")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")

    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.check(
        "selector uses continuous rotation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={selector_joint.articulation_type}",
    )
    ctx.check(
        "five soft keys are authored",
        all(object_model.get_part(f"soft_key_{index}") is not None for index in range(5)),
        details="Missing one or more front soft keys.",
    )
    ctx.expect_origin_gap(
        display,
        selector,
        axis="z",
        min_gap=0.055,
        name="display sits clearly above selector",
    )

    body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({stand_joint: 0.0}):
        stand_closed = ctx.part_world_aabb(stand)
        ctx.check(
            "stand folds into rear pocket",
            body_aabb is not None
            and stand_closed is not None
            and stand_closed[1][1] <= body_aabb[1][1] + 0.0008
            and stand_closed[0][1] >= body_aabb[1][1] - 0.0115,
            details=f"body={body_aabb}, closed_stand={stand_closed}",
        )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        with ctx.pose({stand_joint: stand_limits.upper}):
            stand_open = ctx.part_world_aabb(stand)
            ctx.check(
                "stand swings behind the meter",
                body_aabb is not None
                and stand_open is not None
                and stand_open[1][1] > body_aabb[1][1] + 0.030,
                details=f"body={body_aabb}, open_stand={stand_open}",
            )

    for index in range(5):
        key = object_model.get_part(f"soft_key_{index}")
        key_joint = object_model.get_articulation(f"body_to_soft_key_{index}")
        limits = key_joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(key)
        with ctx.pose({key_joint: limits.upper}):
            pressed_pos = ctx.part_world_position(key)
        ctx.check(
            f"soft key {index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
