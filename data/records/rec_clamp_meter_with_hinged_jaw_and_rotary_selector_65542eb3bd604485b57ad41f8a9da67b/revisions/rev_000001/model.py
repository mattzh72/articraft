from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _body_shell_shape() -> cq.Workplane:
    lower_body = cq.Workplane("XY").box(0.078, 0.034, 0.110).translate((0.0, 0.0, 0.055))
    shoulder = (
        cq.Workplane("XY")
        .rect(0.074, 0.034)
        .workplane(offset=0.053)
        .rect(0.050, 0.032)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.092))
    )
    head = cq.Workplane("XY").box(0.046, 0.034, 0.030).translate((0.0, 0.0, 0.159))
    hinge_block = cq.Workplane("XY").box(0.030, 0.036, 0.014).translate((0.0, 0.0, 0.180))
    chin = cq.Workplane("XY").box(0.044, 0.026, 0.014).translate((0.0, 0.0, 0.014))

    shell = lower_body.union(shoulder).union(head).union(hinge_block).union(chin)
    jaw_clearance = cq.Workplane("XY").box(0.040, 0.030, 0.050).translate((0.020, 0.0, 0.162))
    throat_relief = cq.Workplane("XY").box(0.022, 0.024, 0.024).translate((0.028, 0.0, 0.140))

    return shell.cut(jaw_clearance).cut(throat_relief)


def _jaw_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .center(0.028, -0.023)
        .circle(0.024)
        .circle(0.0155)
        .extrude(0.016)
        .translate((0.0, -0.008, 0.0))
    )
    opening = cq.Workplane("XY").box(0.028, 0.022, 0.034).translate((0.047, 0.0, -0.023))
    hinge_lug = cq.Workplane("XY").box(0.018, 0.016, 0.010).translate((0.010, 0.0, -0.004))
    return ring.cut(opening).union(hinge_lug)


def _button_shape() -> cq.Workplane:
    return cq.Workplane("XZ").rect(0.013, 0.009).extrude(0.0045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    shell_yellow = model.material("shell_yellow", rgba=(0.90, 0.73, 0.16, 1.0))
    jaw_charcoal = model.material("jaw_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    control_black = model.material("control_black", rgba=(0.11, 0.12, 0.13, 1.0))
    button_black = model.material("button_black", rgba=(0.14, 0.15, 0.16, 1.0))
    display_glass = model.material("display_glass", rgba=(0.28, 0.42, 0.46, 0.55))
    label_grey = model.material("label_grey", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "clamp_meter_body_shell"),
        material=shell_yellow,
        name="shell",
    )
    body.visual(
        Box((0.050, 0.0015, 0.016)),
        origin=Origin(xyz=(0.0, 0.0165, 0.036)),
        material=label_grey,
        name="button_bank_divider",
    )
    body.visual(
        Box((0.024, 0.002, 0.016)),
        origin=Origin(xyz=(0.008, 0.018, 0.182)),
        material=jaw_charcoal,
        name="jaw_mount",
    )

    anvil = model.part("anvil")
    anvil.visual(
        Box((0.022, 0.042, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, -0.008)),
        material=jaw_charcoal,
        name="stem",
    )
    anvil.visual(
        Box((0.010, 0.016, 0.022)),
        origin=Origin(xyz=(0.023, 0.005, 0.0)),
        material=jaw_charcoal,
        name="block",
    )
    model.articulation(
        "body_to_anvil",
        ArticulationType.FIXED,
        parent=body,
        child=anvil,
        origin=Origin(xyz=(0.018, 0.039, 0.146)),
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "clamp_meter_jaw"),
        material=jaw_charcoal,
        name="jaw_loop",
    )
    jaw.visual(
        Box((0.010, 0.016, 0.008)),
        origin=Origin(xyz=(0.051, 0.0, -0.044)),
        material=jaw_charcoal,
        name="jaw_tip",
    )
    jaw.visual(
        Box((0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.034, -0.013, -0.041)),
        material=jaw_charcoal,
        name="jaw_bridge",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.043, 0.192)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.0,
        ),
    )

    display = model.part("display")
    display.visual(
        Box((0.042, 0.003, 0.026)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=display_glass,
        name="display_window",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, 0.0170, 0.111)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.018,
                body_style="skirted",
                top_diameter=0.039,
                base_diameter=0.054,
                edge_radius=0.0015,
                center=False,
            ),
            "clamp_meter_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="dial_knob",
    )
    dial.visual(
        Box((0.004, 0.0016, 0.016)),
        origin=Origin(xyz=(0.0, 0.0188, 0.015), rpy=(0.0, 0.0, 0.0)),
        material=label_grey,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0172, 0.071)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=8.0,
        ),
    )

    button_mesh = mesh_from_cadquery(_button_shape(), "clamp_meter_button")
    button_x_positions = (-0.018, 0.0, 0.018)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=button_black,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0173, 0.036)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    anvil = object_model.get_part("anvil")
    jaw = object_model.get_part("jaw")
    display = object_model.get_part("display")
    dial = object_model.get_part("dial")
    jaw_hinge = object_model.get_articulation("body_to_jaw")
    dial_joint = object_model.get_articulation("body_to_dial")

    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    ctx.expect_origin_gap(
        display,
        dial,
        axis="z",
        min_gap=0.028,
        name="display sits clearly above selector dial",
    )
    ctx.expect_origin_gap(
        dial,
        button_parts[1],
        axis="z",
        min_gap=0.028,
        name="button bank is separated below the dial",
    )
    ctx.expect_overlap(
        button_parts[0],
        button_parts[1],
        axes="z",
        min_overlap=0.008,
        name="buttons share a common horizontal row",
    )

    with ctx.pose({jaw_hinge: 0.0}):
        ctx.expect_gap(
            jaw,
            anvil,
            axis="x",
            positive_elem="jaw_tip",
            negative_elem="block",
            max_gap=0.003,
            max_penetration=0.0005,
            name="closed jaw tip sits close to the fixed anvil",
        )

    jaw_limits = jaw_hinge.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        jaw_rest_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_tip")
        with ctx.pose({jaw_hinge: jaw_limits.upper}):
            ctx.expect_gap(
                jaw,
                anvil,
                axis="x",
                positive_elem="jaw_tip",
                negative_elem="block",
                min_gap=0.012,
                name="jaw swings outward when opened",
            )
            jaw_open_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_tip")
        ctx.check(
            "jaw rises as it opens",
            jaw_rest_aabb is not None
            and jaw_open_aabb is not None
            and ((jaw_open_aabb[0][2] + jaw_open_aabb[1][2]) * 0.5)
            > ((jaw_rest_aabb[0][2] + jaw_rest_aabb[1][2]) * 0.5) + 0.004,
            details=f"rest={jaw_rest_aabb}, open={jaw_open_aabb}",
        )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.3}):
        dial_rotated = ctx.part_world_position(dial)
    ctx.check(
        "dial spins in place",
        dial_rest is not None
        and dial_rotated is not None
        and abs(dial_rotated[0] - dial_rest[0]) < 1e-6
        and abs(dial_rotated[1] - dial_rest[1]) < 1e-6
        and abs(dial_rotated[2] - dial_rest[2]) < 1e-6,
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest is not None and pressed is not None and pressed[1] < rest[1] - 0.002,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
