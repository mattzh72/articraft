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


BODY_THICKNESS = 0.034
BODY_FRONT_Y = BODY_THICKNESS * 0.5
JAW_CENTER_Z = 0.188
JAW_OUTER_RADIUS = 0.028
JAW_INNER_RADIUS = 0.020
HINGE_Z = JAW_CENTER_Z + JAW_OUTER_RADIUS


def _rounded_box(size: tuple[float, float, float], edge_radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if edge_radius > 0.0:
        shape = shape.edges("|Z").fillet(edge_radius)
    return shape


def _build_body_shell() -> cq.Workplane:
    handle = _rounded_box((0.052, 0.032, 0.128), 0.007).translate((0.0, 0.0, 0.064))
    shoulder = _rounded_box((0.062, 0.033, 0.040), 0.006).translate((0.0, 0.0, 0.114))
    head = _rounded_box((0.074, 0.034, 0.064), 0.006).translate((0.0, 0.0, 0.146))

    shell = handle.union(shoulder).union(head)

    display_pocket = cq.Workplane("XY").box(0.044, 0.010, 0.034).translate((0.0, 0.013, 0.114))
    selector_pocket = cq.Workplane("XY").box(0.040, 0.010, 0.040).translate((0.0, 0.013, 0.074))
    button_pocket_0 = cq.Workplane("XY").box(0.014, 0.008, 0.011).translate((-0.013, 0.014, 0.042))
    button_pocket_1 = cq.Workplane("XY").box(0.014, 0.008, 0.011).translate((0.013, 0.014, 0.042))
    trigger_slot = cq.Workplane("XY").box(0.028, 0.016, 0.030).translate((0.0, 0.010, 0.133))
    speaker_relief = cq.Workplane("XY").box(0.036, 0.006, 0.010).translate((0.0, 0.014, 0.022))

    result = (
        shell.cut(display_pocket)
        .cut(selector_pocket)
        .cut(button_pocket_0)
        .cut(button_pocket_1)
        .cut(trigger_slot)
        .cut(speaker_relief)
    )
    return result.findSolid()


def _build_fixed_jaw() -> cq.Workplane:
    left_leg = cq.Workplane("XY").box(0.010, BODY_THICKNESS, 0.030).translate((-0.019, 0.0, 0.177))
    right_leg = cq.Workplane("XY").box(0.010, BODY_THICKNESS, 0.030).translate((0.019, 0.0, 0.177))
    bridge = cq.Workplane("XY").box(0.048, BODY_THICKNESS, 0.010).translate((0.0, 0.0, 0.162))
    return left_leg.union(right_leg).union(bridge).findSolid()


def _build_jaw_arc() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(0.048, 0.024, 0.008).translate((0.0, 0.0, -0.004))
    left_leg = cq.Workplane("XY").box(0.010, 0.024, 0.016).translate((-0.019, 0.0, -0.016))
    right_leg = cq.Workplane("XY").box(0.010, 0.024, 0.016).translate((0.019, 0.0, -0.016))
    return bridge.union(left_leg).union(right_leg).findSolid()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_clamp_meter")

    body_orange = model.material("body_orange", rgba=(0.93, 0.66, 0.14, 1.0))
    jaw_charcoal = model.material("jaw_charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.10, 0.11, 1.0))
    button_black = model.material("button_black", rgba=(0.16, 0.17, 0.18, 1.0))
    display_glass = model.material("display_glass", rgba=(0.19, 0.32, 0.36, 0.45))
    display_bezel = model.material("display_bezel", rgba=(0.05, 0.05, 0.06, 1.0))
    trigger_grey = model.material("trigger_grey", rgba=(0.18, 0.18, 0.19, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.28, 0.29, 0.30, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "clamp_meter_body_shell_v4"),
        material=body_orange,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_fixed_jaw(), "clamp_meter_fixed_jaw_v4"),
        material=jaw_charcoal,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.048, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0015, 0.114)),
        material=display_bezel,
        name="display_bezel",
    )
    body.visual(
        Box((0.038, 0.0014, 0.026)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0024, 0.116)),
        material=display_glass,
        name="display_lens",
    )
    body.visual(
        Box((0.012, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, 0.014, 0.196)),
        material=jaw_charcoal,
        name="hinge_post_0",
    )
    body.visual(
        Box((0.012, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, -0.014, 0.196)),
        material=jaw_charcoal,
        name="hinge_post_1",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_build_jaw_arc(), "clamp_meter_jaw_arc_v4"),
        material=jaw_charcoal,
        name="jaw_arc",
    )
    jaw.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=jaw_charcoal,
        name="hinge_barrel",
    )
    jaw.visual(
        Box((0.016, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=jaw_charcoal,
        name="jaw_lug",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.012, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=trigger_grey,
        name="trigger_stem",
    )
    trigger.visual(
        Box((0.024, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.007, -0.022)),
        material=trigger_grey,
        name="trigger_pad",
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.0155, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="selector_cap",
    )
    selector.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="selector_shaft",
    )
    selector.visual(
        Box((0.0035, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.009, 0.007)),
        material=accent_grey,
        name="selector_pointer",
    )

    for index, x_pos in enumerate((-0.013, 0.013)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.011, 0.004, 0.007)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Box((0.006, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=control_black,
            name="button_stem",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_FRONT_Y, 0.042)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.03, lower=0.0, upper=0.0022),
        )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.02),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, 0.010, 0.141)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.010),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, 0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    trigger = object_model.get_part("trigger")
    selector = object_model.get_part("selector")
    selector_joint = object_model.get_articulation("body_to_selector")
    jaw_joint = object_model.get_articulation("body_to_jaw")
    trigger_joint = object_model.get_articulation("body_to_trigger")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")
    button_joint_0 = object_model.get_articulation("body_to_mode_button_0")
    button_joint_1 = object_model.get_articulation("body_to_mode_button_1")

    ctx.allow_overlap(
        body,
        selector,
        elem_a="body_shell",
        elem_b="selector_shaft",
        reason="The rotary selector shaft is intentionally retained inside the front housing behind the knob cap.",
    )

    jaw_limits = jaw_joint.motion_limits
    trigger_limits = trigger_joint.motion_limits
    button_limits_0 = button_joint_0.motion_limits
    button_limits_1 = button_joint_1.motion_limits
    selector_limits = selector_joint.motion_limits

    display_aabb = ctx.part_element_world_aabb(body, elem="display_lens")
    button_0_pos = ctx.part_world_position(button_0)
    button_1_pos = ctx.part_world_position(button_1)

    ctx.check(
        "mode buttons sit below display",
        display_aabb is not None
        and button_0_pos is not None
        and button_1_pos is not None
        and button_0_pos[2] < display_aabb[0][2] - 0.010
        and button_1_pos[2] < display_aabb[0][2] - 0.010,
        details=f"display={display_aabb}, button_0={button_0_pos}, button_1={button_1_pos}",
    )
    ctx.check(
        "mode buttons remain separate controls",
        button_0_pos is not None and button_1_pos is not None and abs(button_0_pos[0] - button_1_pos[0]) > 0.020,
        details=f"button_0={button_0_pos}, button_1={button_1_pos}",
    )
    ctx.check(
        "selector uses continuous rotation",
        selector_limits is not None and selector_limits.lower is None and selector_limits.upper is None,
        details=f"selector_limits={selector_limits}",
    )

    if jaw_limits is not None and jaw_limits.upper is not None:
        jaw_rest_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_arc")
        ctx.expect_contact(
            jaw,
            body,
            elem_a="jaw_arc",
            elem_b="fixed_jaw",
            contact_tol=0.001,
            name="jaw closes against fixed jaw",
        )
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            jaw_open_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_arc")
        jaw_rest_center_x = None
        jaw_open_center_x = None
        if jaw_rest_aabb is not None:
            jaw_rest_center_x = (jaw_rest_aabb[0][0] + jaw_rest_aabb[1][0]) * 0.5
        if jaw_open_aabb is not None:
            jaw_open_center_x = (jaw_open_aabb[0][0] + jaw_open_aabb[1][0]) * 0.5
        ctx.check(
            "jaw swings clear of clamp opening",
            jaw_rest_aabb is not None
            and jaw_open_aabb is not None
            and jaw_open_aabb[1][2] > jaw_rest_aabb[1][2] + 0.018
            and jaw_rest_center_x is not None
            and jaw_open_center_x is not None
            and abs(jaw_open_center_x - jaw_rest_center_x) > 0.008,
            details=f"rest={jaw_rest_aabb}, open={jaw_open_aabb}, rest_center_x={jaw_rest_center_x}, open_center_x={jaw_open_center_x}",
        )

    if trigger_limits is not None and trigger_limits.upper is not None:
        trigger_rest = ctx.part_world_position(trigger)
        with ctx.pose({trigger_joint: trigger_limits.upper}):
            trigger_retracted = ctx.part_world_position(trigger)
        ctx.check(
            "trigger retracts into upper handle",
            trigger_rest is not None and trigger_retracted is not None and trigger_retracted[2] > trigger_rest[2] + 0.008,
            details=f"rest={trigger_rest}, retracted={trigger_retracted}",
        )

    if button_limits_0 is not None and button_limits_0.upper is not None:
        button_0_rest = ctx.part_world_position(button_0)
        with ctx.pose({button_joint_0: button_limits_0.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "mode button 0 presses inward",
            button_0_rest is not None and button_0_pressed is not None and button_0_pressed[1] < button_0_rest[1] - 0.0015,
            details=f"rest={button_0_rest}, pressed={button_0_pressed}",
        )

    if button_limits_1 is not None and button_limits_1.upper is not None:
        button_1_rest = ctx.part_world_position(button_1)
        with ctx.pose({button_joint_1: button_limits_1.upper}):
            button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "mode button 1 presses inward",
            button_1_rest is not None and button_1_pressed is not None and button_1_pressed[1] < button_1_rest[1] - 0.0015,
            details=f"rest={button_1_rest}, pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
