from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BLACK = Material("satin_black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
RUBBER = Material("matte_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
DARK = Material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
METAL = Material("brushed_metal", rgba=(0.55, 0.55, 0.52, 1.0))
LCD = Material("blue_black_lcd", rgba=(0.02, 0.08, 0.11, 1.0))
DIFFUSER = Material("milky_diffuser", rgba=(0.82, 0.90, 0.96, 0.58))
CARD = Material("white_bounce_card", rgba=(0.96, 0.96, 0.92, 1.0))


def _rounded_box_mesh(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    radius: float,
    name: str,
):
    shape = (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges("|Z")
        .fillet(radius)
        .translate(center)
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.08)


def _head_shell_mesh():
    outer = (
        cq.Workplane("XY")
        .box(0.100, 0.104, 0.080)
        .edges("|Z")
        .fillet(0.007)
        .translate((0.045, 0.0, -0.005))
    )
    # A real slot through the top leaves a bottom web, so the shell is one
    # connected part while the bounce card has a clear sleeve to slide inside.
    card_slot = cq.Workplane("XY").box(0.008, 0.086, 0.085).translate((0.010, 0.0, 0.0))
    return mesh_from_cadquery(outer.cut(card_slot), "head_shell", tolerance=0.0005, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    body = model.part("body")
    body.visual(
        Box((0.070, 0.052, 0.007)),
        origin=Origin(xyz=(0.000, 0.000, 0.0035)),
        material=METAL,
        name="foot_plate",
    )
    body.visual(
        Box((0.052, 0.040, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=DARK,
        name="shoe_block",
    )
    body.visual(
        _rounded_box_mesh((0.060, 0.078, 0.120), (-0.005, 0.0, 0.075), 0.008, "body_shell"),
        material=BLACK,
        name="body_shell",
    )
    body.visual(
        Box((0.002, 0.062, 0.083)),
        origin=Origin(xyz=(-0.036, 0.000, 0.082)),
        material=DARK,
        name="rear_panel",
    )
    body.visual(
        Box((0.001, 0.052, 0.027)),
        origin=Origin(xyz=(-0.0375, 0.000, 0.108)),
        material=LCD,
        name="lcd_window",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.139)),
        material=DARK,
        name="swivel_socket",
    )

    dial = model.part("rear_dial")
    dial.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(-0.0025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=RUBBER,
        name="dial_wheel",
    )
    dial.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(-0.0055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK,
        name="dial_hub",
    )
    model.articulation(
        "body_to_rear_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(-0.037, 0.0, 0.041)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    button_locations = [
        (-0.019, 0.063),
        (0.019, 0.063),
        (-0.019, 0.080),
        (0.019, 0.080),
    ]
    for index, (y, z) in enumerate(button_locations):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.004, 0.014, 0.009)),
            origin=Origin(xyz=(-0.002, 0.0, 0.0)),
            material=RUBBER,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.037, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.03, lower=0.0, upper=0.002),
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=BLACK,
        name="swivel_disk",
    )
    yoke.visual(
        Box((0.032, 0.050, 0.025)),
        origin=Origin(xyz=(0.014, 0.000, 0.018)),
        material=BLACK,
        name="neck_block",
    )
    yoke.visual(
        Box((0.022, 0.150, 0.050)),
        origin=Origin(xyz=(0.026, 0.000, 0.030)),
        material=BLACK,
        name="rear_bridge",
    )
    yoke.visual(
        Box((0.062, 0.018, 0.080)),
        origin=Origin(xyz=(0.061, -0.067, 0.038)),
        material=BLACK,
        name="yoke_arm_0",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.055, -0.067, 0.042), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK,
        name="tilt_bushing_0",
    )
    yoke.visual(
        Box((0.062, 0.018, 0.080)),
        origin=Origin(xyz=(0.061, 0.067, 0.038)),
        material=BLACK,
        name="yoke_arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.055, 0.067, 0.042), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK,
        name="tilt_bushing_1",
    )
    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.000, 0.000, 0.143)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    head = model.part("head")
    head.visual(
        Box((0.011, 0.104, 0.080)),
        origin=Origin(xyz=(0.0005, 0.000, -0.005)),
        material=BLACK,
        name="rear_shell",
    )
    head.visual(
        Box((0.081, 0.104, 0.080)),
        origin=Origin(xyz=(0.0545, 0.000, -0.005)),
        material=BLACK,
        name="front_shell",
    )
    head.visual(
        Box((0.008, 0.009, 0.080)),
        origin=Origin(xyz=(0.010, -0.0475, -0.005)),
        material=BLACK,
        name="slot_rail_0",
    )
    head.visual(
        Box((0.008, 0.009, 0.080)),
        origin=Origin(xyz=(0.010, 0.0475, -0.005)),
        material=BLACK,
        name="slot_rail_1",
    )
    head.visual(
        Box((0.004, 0.086, 0.054)),
        origin=Origin(xyz=(0.097, 0.000, -0.004)),
        material=DIFFUSER,
        name="front_diffuser",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.000, -0.055, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK,
        name="tilt_pin_0",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.000, 0.055, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK,
        name="tilt_pin_1",
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.055, 0.000, 0.042)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.17, upper=1.35),
    )

    card = model.part("bounce_card")
    card.visual(
        Box((0.008, 0.076, 0.072)),
        origin=Origin(xyz=(0.000, 0.000, -0.036)),
        material=CARD,
        name="card_insert",
    )
    model.articulation(
        "head_to_bounce_card",
        ArticulationType.PRISMATIC,
        parent=head,
        child=card,
        origin=Origin(xyz=(0.010, 0.000, 0.037)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.12, lower=0.0, upper=0.045),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    card = object_model.get_part("bounce_card")
    swivel = object_model.get_articulation("body_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")
    slide = object_model.get_articulation("head_to_bounce_card")

    ctx.expect_contact(
        body,
        yoke,
        elem_a="swivel_socket",
        elem_b="swivel_disk",
        contact_tol=0.001,
        name="swivel disk sits on body socket",
    )
    ctx.expect_overlap(
        head,
        yoke,
        axes="xz",
        elem_a="tilt_pin_1",
        elem_b="tilt_bushing_1",
        min_overlap=0.006,
        name="visible side tilt axis is coaxial",
    )
    ctx.expect_within(
        card,
        head,
        axes="xy",
        inner_elem="card_insert",
        margin=0.004,
        name="bounce card stays inside top slot in plan",
    )
    ctx.expect_overlap(
        card,
        head,
        axes="z",
        elem_a="card_insert",
        min_overlap=0.035,
        name="bounce card has retained insertion when stowed",
    )

    rest_card = ctx.part_world_position(card)
    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({slide: 0.045}):
        ctx.expect_within(
            card,
            head,
            axes="xy",
            inner_elem="card_insert",
            margin=0.004,
            name="extended bounce card remains guided by slot",
        )
        ctx.expect_overlap(
            card,
            head,
            axes="z",
            elem_a="card_insert",
            min_overlap=0.020,
            name="extended bounce card remains retained",
        )
        extended_card = ctx.part_world_position(card)
    ctx.check(
        "bounce card slides upward",
        rest_card is not None and extended_card is not None and extended_card[2] > rest_card[2] + 0.035,
        details=f"rest={rest_card}, extended={extended_card}",
    )

    with ctx.pose({tilt: 1.0}):
        tilted_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head pitches upward on yoke",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.035,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_origin_distance(
            head,
            body,
            axes="xy",
            min_dist=0.04,
            name="head assembly swivels around body top",
        )

    return ctx.report()


object_model = build_object_model()
