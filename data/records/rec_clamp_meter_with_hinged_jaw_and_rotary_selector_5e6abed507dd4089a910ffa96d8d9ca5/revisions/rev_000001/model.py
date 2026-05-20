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


BODY_THICKNESS = 0.038
BODY_FRONT_Y = -BODY_THICKNESS * 0.5


def _build_body_shape() -> object:
    profile = [
        (-0.033, 0.000),
        (0.033, 0.000),
        (0.038, 0.060),
        (0.043, 0.096),
        (0.038, 0.120),
        (0.031, 0.138),
        (0.018, 0.150),
        (0.018, 0.178),
        (-0.018, 0.178),
        (-0.018, 0.150),
        (-0.031, 0.138),
        (-0.038, 0.120),
        (-0.043, 0.096),
        (-0.038, 0.060),
    ]

    body = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(BODY_THICKNESS)
        .translate((0.0, -BODY_FRONT_Y, 0.0))
        .edges("|Y")
        .fillet(0.0045)
    )

    display_recess = (
        cq.Workplane("XY")
        .box(0.054, 0.0035, 0.031)
        .translate((0.0, BODY_FRONT_Y + 0.00175, 0.121))
    )
    dial_recess = (
        cq.Workplane("XZ")
        .center(0.0, 0.080)
        .circle(0.028)
        .extrude(0.0025)
        .translate((0.0, BODY_FRONT_Y, 0.0))
    )
    button_pocket_0 = (
        cq.Workplane("XY")
        .box(0.017, 0.009, 0.009)
        .translate((-0.017, BODY_FRONT_Y + 0.0045, 0.037))
    )
    button_pocket_1 = (
        cq.Workplane("XY")
        .box(0.017, 0.009, 0.009)
        .translate((0.017, BODY_FRONT_Y + 0.0045, 0.037))
    )
    hinge_boss = (
        cq.Workplane("XZ")
        .center(0.0, 0.196)
        .circle(0.0055)
        .extrude(0.002)
        .translate((0.0, BODY_FRONT_Y + 0.002, 0.0))
    )
    hinge_neck = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.022)
        .translate((0.0, BODY_FRONT_Y + 0.002, 0.186))
    )

    body = body.cut(display_recess)
    body = body.cut(dial_recess)
    body = body.cut(button_pocket_0)
    body = body.cut(button_pocket_1)
    body = body.union(hinge_neck)
    body = body.union(hinge_boss)
    return body


def _build_jaw_shape() -> object:
    loop = (
        cq.Workplane("XZ")
        .center(0.0, -0.018)
        .circle(0.030)
        .circle(0.022)
        .extrude(0.016)
        .translate((0.0, 0.008, 0.0))
    )
    lower_opening = (
        cq.Workplane("XY")
        .box(0.024, 0.024, 0.018)
        .translate((0.0, 0.0, -0.047))
    )
    bridge = cq.Workplane("XY").box(0.016, 0.016, 0.008).translate((0.0, 0.0, -0.006))
    hinge_collar = cq.Workplane("XZ").circle(0.006).extrude(0.020).translate((0.0, 0.010, 0.0))
    return loop.cut(lower_opening).union(bridge).union(hinge_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    body_material = model.material("body_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    trim_material = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dial_material = model.material("dial_black", rgba=(0.11, 0.11, 0.12, 1.0))
    button_material = model.material("button_grey", rgba=(0.32, 0.34, 0.36, 1.0))
    glass_material = model.material("glass_tint", rgba=(0.19, 0.34, 0.39, 0.45))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=body_material,
        name="body_shell",
    )
    body.visual(
        Box((0.050, 0.0036, 0.027)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0018, 0.121)),
        material=glass_material,
        name="display_glass",
    )
    body.visual(
        Box((0.044, 0.0018, 0.020)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0009, 0.122)),
        material=trim_material,
        name="display_bezel",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_build_jaw_shape(), "jaw_loop"),
        origin=Origin(xyz=(0.0, -0.029, 0.0)),
        material=trim_material,
        name="jaw_loop",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_material,
        name="dial_rim",
    )
    dial.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_material,
        name="dial_cap",
    )

    for index, x_pos in enumerate((-0.017, 0.017)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.020, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, -0.0015, 0.0)),
            material=button_material,
            name="button_cap",
        )
        button.visual(
            Box((0.016, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=button_material,
            name="button_stem",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_FRONT_Y, 0.037)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.002,
            ),
        )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, 0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")
    jaw_joint = object_model.get_articulation("body_to_jaw")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joint_0 = object_model.get_articulation("body_to_mode_button_0")
    button_joint_1 = object_model.get_articulation("body_to_mode_button_1")

    ctx.expect_gap(
        body,
        dial,
        axis="y",
        positive_elem="body_shell",
        negative_elem="dial_rim",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial seats on the front face",
    )
    ctx.expect_gap(
        body,
        button_0,
        axis="y",
        positive_elem="body_shell",
        negative_elem="button_cap",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left mode button starts flush with the front face",
    )
    ctx.expect_gap(
        body,
        button_1,
        axis="y",
        positive_elem="body_shell",
        negative_elem="button_cap",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right mode button starts flush with the front face",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    jaw_rest_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_loop")

    with ctx.pose({dial_joint: math.pi * 0.75}):
        ctx.expect_gap(
            body,
            dial,
            axis="y",
            positive_elem="body_shell",
            negative_elem="dial_rim",
            max_gap=0.0005,
            max_penetration=0.0,
            name="dial stays seated while rotated",
        )

    with ctx.pose({button_joint_0: 0.002, button_joint_1: 0.002}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)

    with ctx.pose({jaw_joint: 1.05}):
        jaw_open_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_loop")

    ctx.check(
        "mode buttons press inward",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_pressed is not None
        and button_0_pressed[1] > button_0_rest[1] + 0.0015
        and button_1_pressed[1] > button_1_rest[1] + 0.0015,
        details=(
            f"rest_left={button_0_rest}, pressed_left={button_0_pressed}, "
            f"rest_right={button_1_rest}, pressed_right={button_1_pressed}"
        ),
    )
    ctx.check(
        "jaw opens outward from the head",
        jaw_rest_aabb is not None
        and jaw_open_aabb is not None
        and jaw_open_aabb[1][0] > jaw_rest_aabb[1][0] + 0.014,
        details=f"rest={jaw_rest_aabb}, open={jaw_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
