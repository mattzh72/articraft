from __future__ import annotations

from math import pi

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


BODY_W = 0.58
BODY_D = 0.38
BODY_H = 0.32
FRONT_Y = -BODY_D / 2.0
FASCIA_Y = FRONT_Y - 0.004
DOOR_Y = FRONT_Y - 0.013
DOOR_H = 0.195
DOOR_W = 0.260
DOOR_T = 0.016
DOOR_Z = 0.195
HINGE_X = 0.265
BUTTON_TRAVEL = 0.006


def _oven_shell() -> cq.Workplane:
    """One-piece countertop toaster-oven housing with an open cooking cavity."""
    wall = 0.025
    cavity_bottom = 0.095
    cavity_h = 0.195
    cutter_d = BODY_D - wall + 0.020
    cutter_y = (-wall - 0.020) / 2.0
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BODY_H / 2.0))
    )
    cavity_cut = cq.Workplane("XY").box(
        BODY_W - 2.0 * wall,
        cutter_d,
        cavity_h,
    ).translate((0.0, cutter_y, cavity_bottom + cavity_h / 2.0))
    return shell.cut(cavity_cut)


def _door_frame(sign: float) -> cq.Workplane:
    """Glass-door metal surround, with local origin on the vertical hinge line."""
    border = 0.024
    outer = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H).translate(
        (sign * DOOR_W / 2.0, 0.0, 0.0)
    )
    opening = cq.Workplane("XY").box(
        DOOR_W - 2.0 * border,
        DOOR_T + 0.006,
        DOOR_H - 2.0 * border,
    ).translate((sign * DOOR_W / 2.0, 0.0, 0.0))
    return outer.cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_french_door_toaster_oven")

    brushed = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("matte_black_trim", rgba=(0.015, 0.014, 0.013, 1.0))
    liner = model.material("dark_oven_liner", rgba=(0.045, 0.043, 0.040, 1.0))
    glass = model.material("smoked_glass", rgba=(0.30, 0.48, 0.58, 0.38))
    display = model.material("black_display", rgba=(0.0, 0.015, 0.025, 1.0))
    glow = model.material("cool_blue_digits", rgba=(0.05, 0.80, 1.0, 1.0))
    button_mat = model.material("soft_touch_buttons", rgba=(0.055, 0.055, 0.058, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_oven_shell(), "oven_shell", tolerance=0.0015),
        material=brushed,
        name="shell",
    )
    body.visual(
        Box((0.505, 0.014, 0.170)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.024, 0.190)),
        material=liner,
        name="rear_liner",
    )
    body.visual(
        Box((0.505, 0.300, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.096)),
        material=liner,
        name="crumb_floor",
    )
    body.visual(
        Box((0.012, 0.285, 0.012)),
        origin=Origin(xyz=(-0.261, -0.015, 0.160)),
        material=brushed,
        name="rack_rail_0",
    )
    body.visual(
        Box((0.012, 0.285, 0.012)),
        origin=Origin(xyz=(0.261, -0.015, 0.160)),
        material=brushed,
        name="rack_rail_1",
    )
    body.visual(
        Box((0.540, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, FASCIA_Y + 0.001, 0.045)),
        material=dark,
        name="front_fascia",
    )
    body.visual(
        Box((0.115, 0.003, 0.033)),
        origin=Origin(xyz=(-0.135, FASCIA_Y - 0.004, 0.055)),
        material=display,
        name="display_window",
    )
    # Simple seven-segment-like display strokes, slightly proud of the screen.
    for i, (x, z, sx, sz) in enumerate(
        (
            (-0.168, 0.063, 0.030, 0.003),
            (-0.168, 0.050, 0.030, 0.003),
            (-0.135, 0.063, 0.020, 0.003),
            (-0.135, 0.050, 0.020, 0.003),
            (-0.102, 0.056, 0.026, 0.003),
        )
    ):
        body.visual(
            Box((sx, 0.002, sz)),
            origin=Origin(xyz=(x, FASCIA_Y - 0.006, z)),
            material=glow,
            name=f"display_segment_{i}",
        )
    for i, x in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.080, 0.055, 0.018)),
            origin=Origin(xyz=(x, -0.105, -0.007)),
            material=rubber,
            name=f"foot_{i}",
        )

    left_door = model.part("door_0")
    left_door.visual(
        mesh_from_cadquery(_door_frame(1.0), "door_0_frame", tolerance=0.001),
        material=dark,
        name="frame",
    )
    left_door.visual(
        Box((DOOR_W - 0.036, 0.006, DOOR_H - 0.046)),
        origin=Origin(xyz=(DOOR_W / 2.0, -0.001, 0.0)),
        material=glass,
        name="glass",
    )
    left_door.visual(
        Cylinder(radius=0.008, length=DOOR_H + 0.018),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material=brushed,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.014, 0.026, 0.118)),
        origin=Origin(xyz=(DOOR_W - 0.017, -0.020, 0.0)),
        material=dark,
        name="pull_handle",
    )

    right_door = model.part("door_1")
    right_door.visual(
        mesh_from_cadquery(_door_frame(-1.0), "door_1_frame", tolerance=0.001),
        material=dark,
        name="frame",
    )
    right_door.visual(
        Box((DOOR_W - 0.036, 0.006, DOOR_H - 0.046)),
        origin=Origin(xyz=(-DOOR_W / 2.0, -0.001, 0.0)),
        material=glass,
        name="glass",
    )
    right_door.visual(
        Cylinder(radius=0.008, length=DOOR_H + 0.018),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material=brushed,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.014, 0.026, 0.118)),
        origin=Origin(xyz=(-DOOR_W + 0.017, -0.020, 0.0)),
        material=dark,
        name="pull_handle",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.076,
                0.030,
                body_style="skirted",
                top_diameter=0.058,
                skirt=KnobSkirt(0.088, 0.006, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=24, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "large_dial",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="dial_cap",
    )

    button_positions = [
        (-0.045, 0.062),
        (0.000, 0.062),
        (0.045, 0.062),
        (-0.045, 0.029),
        (0.000, 0.029),
        (0.045, 0.029),
    ]
    buttons = []
    for idx, (x, z) in enumerate(button_positions):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.032, 0.011, 0.018)),
            origin=Origin(xyz=(0.0, -0.011 / 2.0, 0.0)),
            material=button_mat,
            name="cap",
        )
        buttons.append((button, x, z))

    model.articulation(
        "body_to_door_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(-HINGE_X, FRONT_Y - 0.008, DOOR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.65),
    )
    model.articulation(
        "body_to_door_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(HINGE_X, FRONT_Y - 0.008, DOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.65),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.210, FASCIA_Y - 0.003, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.45, velocity=5.0),
    )
    for idx, (button, x, z) in enumerate(buttons):
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FASCIA_Y - 0.003, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    dial = object_model.get_part("dial")
    door_0_joint = object_model.get_articulation("body_to_door_0")
    door_1_joint = object_model.get_articulation("body_to_door_1")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.002,
        max_gap=0.020,
        elem_a="frame",
        elem_b="frame",
        name="closed french doors meet with a narrow center seam",
    )
    ctx.expect_contact(
        body,
        door_0,
        elem_a="shell",
        elem_b="hinge_barrel",
        contact_tol=0.002,
        name="left door hinge barrel is carried by the oven side frame",
    )
    ctx.expect_contact(
        body,
        door_1,
        elem_a="shell",
        elem_b="hinge_barrel",
        contact_tol=0.002,
        name="right door hinge barrel is carried by the oven side frame",
    )
    ctx.expect_gap(
        body,
        object_model.get_part("button_0"),
        axis="y",
        min_gap=-0.0005,
        max_gap=0.0015,
        positive_elem="front_fascia",
        negative_elem="cap",
        name="button caps sit flush on the lower fascia",
    )
    ctx.check(
        "large dial is a continuous rotary control",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"dial joint type is {dial_joint.articulation_type}",
    )

    closed_left = ctx.part_world_aabb(door_0)
    closed_right = ctx.part_world_aabb(door_1)
    with ctx.pose({door_0_joint: 1.35, door_1_joint: 1.35}):
        opened_left = ctx.part_world_aabb(door_0)
        opened_right = ctx.part_world_aabb(door_1)
    ctx.check(
        "both side hinged glass doors swing outward",
        closed_left is not None
        and opened_left is not None
        and closed_right is not None
        and opened_right is not None
        and opened_left[0][1] < closed_left[0][1] - 0.10
        and opened_right[0][1] < closed_right[0][1] - 0.10,
        details=f"closed_left={closed_left}, opened_left={opened_left}, closed_right={closed_right}, opened_right={opened_right}",
    )

    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    rest_0 = ctx.part_world_position(button_0)
    rest_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_0_joint: BUTTON_TRAVEL}):
        pressed_0 = ctx.part_world_position(button_0)
        still_1 = ctx.part_world_position(button_1)
    ctx.check(
        "lower buttons depress independently",
        rest_0 is not None
        and rest_1 is not None
        and pressed_0 is not None
        and still_1 is not None
        and pressed_0[1] > rest_0[1] + BUTTON_TRAVEL * 0.8
        and abs(still_1[1] - rest_1[1]) < 1e-6,
        details=f"rest_0={rest_0}, pressed_0={pressed_0}, rest_1={rest_1}, still_1={still_1}",
    )

    return ctx.report()


object_model = build_object_model()
