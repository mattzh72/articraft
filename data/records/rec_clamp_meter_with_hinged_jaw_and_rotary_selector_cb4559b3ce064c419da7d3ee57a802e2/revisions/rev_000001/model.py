from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


JAW_CENTER_Z = 0.224
JAW_OUTER_R = 0.0285
JAW_INNER_R = 0.0190
JAW_THICKNESS = 0.010
HINGE_Y = JAW_OUTER_R
BODY_FRONT_X = 0.016
BODY_SIDE_Y = 0.026


def _arc_points(
    center_y: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 20,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append(
            (
                center_y + radius * math.cos(angle),
                center_z + radius * math.sin(angle),
            )
        )
    return points


def _jaw_segment(
    *,
    center_y: float,
    center_z: float,
    inner_r: float,
    outer_r: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
) -> cq.Workplane:
    outer = _arc_points(center_y, center_z, outer_r, start_deg, end_deg, segments=28)
    inner = list(reversed(_arc_points(center_y, center_z, inner_r, start_deg, end_deg, segments=28)))
    return cq.Workplane("YZ").polyline(outer + inner).close().extrude(thickness, both=True)


def _body_shell_shape() -> cq.Workplane:
    handle = (
        cq.Workplane("XY")
        .box(0.038, 0.052, 0.145)
        .translate((-0.003, 0.0, 0.0725))
        .edges("|Z")
        .fillet(0.007)
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.032, 0.056, 0.022)
        .translate((0.001, 0.0, 0.153))
        .edges("|Z")
        .fillet(0.005)
    )
    head = (
        cq.Workplane("XY")
        .box(0.050, 0.074, 0.034)
        .translate((0.006, 0.0, 0.180))
        .edges("|Z")
        .fillet(0.006)
    )
    chin = (
        cq.Workplane("XY")
        .box(0.024, 0.050, 0.016)
        .translate((0.007, 0.0, 0.165))
        .edges("|Z")
        .fillet(0.004)
    )
    shell = handle.union(shoulder).union(head).union(chin)

    display_recess = cq.Workplane("XY").box(0.008, 0.036, 0.026).translate((0.012, 0.0, 0.134))
    trigger_slot = cq.Workplane("XY").box(0.022, 0.028, 0.020).translate((0.007, 0.0, 0.026))
    button_pocket_0 = cq.Workplane("XY").box(0.010, 0.015, 0.010).translate((0.013, -0.011, 0.056))
    button_pocket_1 = cq.Workplane("XY").box(0.010, 0.015, 0.010).translate((0.013, 0.011, 0.056))
    side_pocket = cq.Workplane("XY").box(0.012, 0.014, 0.014).translate((0.005, 0.020, 0.082))
    selector_seat = (
        cq.Workplane("YZ")
        .center(0.0, 0.094)
        .circle(0.020)
        .extrude(0.005)
        .translate((0.011, 0.0, 0.0))
    )

    shell = shell.cut(display_recess)
    shell = shell.cut(trigger_slot)
    shell = shell.cut(button_pocket_0)
    shell = shell.cut(button_pocket_1)
    shell = shell.cut(side_pocket)
    shell = shell.cut(selector_seat)
    return shell


def _lower_jaw_shape() -> cq.Workplane:
    jaw = _jaw_segment(
        center_y=0.0,
        center_z=JAW_CENTER_Z,
        inner_r=JAW_INNER_R,
        outer_r=JAW_OUTER_R,
        start_deg=124.0,
        end_deg=344.0,
        thickness=JAW_THICKNESS,
    )
    saddle = (
        cq.Workplane("XY")
        .box(0.020, 0.038, 0.008)
        .translate((0.000, 0.0, 0.201))
        .edges("|Z")
        .fillet(0.003)
    )
    return jaw.union(saddle)


def _upper_jaw_shape() -> cq.Workplane:
    jaw = _jaw_segment(
        center_y=-HINGE_Y,
        center_z=0.0,
        inner_r=JAW_INNER_R,
        outer_r=JAW_OUTER_R,
        start_deg=-2.0,
        end_deg=124.0,
        thickness=JAW_THICKNESS,
    )
    knuckle = cq.Workplane("YZ").center(0.0, 0.0).circle(0.0042).extrude(0.005, both=True)
    return jaw.union(knuckle)


def _trigger_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.010, 0.028, 0.020)
        .translate((0.005, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )
    back_plate = cq.Workplane("XY").box(0.001, 0.028, 0.020).translate((-0.0005, 0.0, 0.0))
    return body.union(back_plate)


def _front_button_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.0045, 0.015, 0.010)
        .translate((0.00225, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.002)
    )


def _selector_shape() -> cq.Workplane:
    mount = cq.Workplane("YZ").circle(0.020).extrude(0.001).translate((-0.001, 0.0, 0.0))
    skirt = cq.Workplane("YZ").circle(0.022).extrude(0.003)
    knob = cq.Workplane("YZ").circle(0.018).extrude(0.010).translate((0.003, 0.0, 0.0))
    pointer = cq.Workplane("XY").box(0.002, 0.004, 0.010).translate((0.012, 0.0, 0.013))
    return mount.union(skirt).union(knob).union(pointer)


def _side_button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.012, 0.010, 0.014)
        .translate((0.0, 0.005, 0.0))
        .edges("|Z")
        .fillet(0.002)
    )
    back_plate = cq.Workplane("XY").box(0.012, 0.001, 0.014).translate((0.0, -0.0005, 0.0))
    return cap.union(back_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hvac_clamp_meter")

    body_dark = model.material("body_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    jaw_accent = model.material("jaw_accent", rgba=(0.86, 0.56, 0.14, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_gray = model.material("button_gray", rgba=(0.52, 0.54, 0.57, 1.0))
    display_glass = model.material("display_glass", rgba=(0.22, 0.36, 0.42, 0.55))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=body_dark,
        name="shell",
    )

    lower_jaw = model.part("lower_jaw")
    lower_jaw.visual(
        mesh_from_cadquery(_lower_jaw_shape(), "lower_jaw"),
        material=jaw_accent,
        name="jaw_shell",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_upper_jaw_shape(), "upper_jaw"),
        material=jaw_accent,
        name="jaw_shell",
    )

    display = model.part("display")
    display.visual(
        Box((0.003, 0.033, 0.023)),
        material=display_glass,
        name="screen",
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_cadquery(_selector_shape(), "selector_knob"),
        material=control_black,
        name="knob",
    )

    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_shape(), "trigger"),
        material=control_black,
        name="trigger_shell",
    )

    side_button = model.part("side_button")
    side_button.visual(
        mesh_from_cadquery(_side_button_shape(), "side_button"),
        material=button_gray,
        name="button_shell",
    )

    front_button_0 = model.part("front_button_0")
    front_button_0.visual(
        mesh_from_cadquery(_front_button_shape(), "front_button_0"),
        material=button_gray,
        name="button_shell",
    )

    front_button_1 = model.part("front_button_1")
    front_button_1.visual(
        mesh_from_cadquery(_front_button_shape(), "front_button_1"),
        material=button_gray,
        name="button_shell",
    )

    model.articulation(
        "body_to_lower_jaw",
        ArticulationType.FIXED,
        parent=body,
        child=lower_jaw,
        origin=Origin(),
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0145, 0.0, 0.134)),
    )
    model.articulation(
        "lower_jaw_to_jaw",
        ArticulationType.REVOLUTE,
        parent=lower_jaw,
        child=jaw,
        origin=Origin(xyz=(0.0, HINGE_Y, JAW_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.017, 0.0, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=8.0,
        ),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.017, 0.0, 0.026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.12,
            lower=0.0,
            upper=0.011,
        ),
    )
    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(0.005, 0.027, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0025,
        ),
    )
    model.articulation(
        "body_to_front_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_0,
        origin=Origin(xyz=(0.016, -0.011, 0.056)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0025,
        ),
    )
    model.articulation(
        "body_to_front_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_1,
        origin=Origin(xyz=(0.016, 0.011, 0.056)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lower_jaw = object_model.get_part("lower_jaw")
    jaw = object_model.get_part("jaw")
    selector = object_model.get_part("selector")
    trigger = object_model.get_part("trigger")
    side_button = object_model.get_part("side_button")
    front_button_0 = object_model.get_part("front_button_0")
    front_button_1 = object_model.get_part("front_button_1")

    jaw_joint = object_model.get_articulation("lower_jaw_to_jaw")
    trigger_joint = object_model.get_articulation("body_to_trigger")
    side_joint = object_model.get_articulation("body_to_side_button")
    front_joint_0 = object_model.get_articulation("body_to_front_button_0")
    front_joint_1 = object_model.get_articulation("body_to_front_button_1")

    ctx.allow_overlap(
        body,
        selector,
        elem_a="shell",
        elem_b="knob",
        reason="The rotary selector includes a simplified rear mounting flange seated into the control face.",
    )
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="shell",
        elem_b="trigger_shell",
        reason="The lower trigger uses a simplified retained slide plate inside the handle slot.",
    )
    ctx.allow_overlap(
        body,
        side_button,
        elem_a="shell",
        elem_b="button_shell",
        reason="The side flashlight button uses a shallow retained mounting plate inside the side pocket.",
    )

    ctx.expect_within(selector, body, axes="yz", margin=0.012, name="selector stays centered on the control face")
    ctx.expect_within(trigger, body, axes="yz", margin=0.010, name="trigger stays within the handle slot footprint")
    ctx.expect_within(front_button_0, body, axes="yz", margin=0.006, name="front button 0 stays within the front button pad")
    ctx.expect_within(front_button_1, body, axes="yz", margin=0.006, name="front button 1 stays within the front button pad")
    ctx.expect_within(side_button, body, axes="xz", margin=0.006, name="side button stays within the side boss")
    ctx.expect_overlap(jaw, lower_jaw, axes="x", min_overlap=0.008, name="upper jaw shares the hinge thickness with the fixed jaw")

    trigger_rest = ctx.part_world_position(trigger)
    side_rest = ctx.part_world_position(side_button)
    front_0_rest = ctx.part_world_position(front_button_0)
    front_1_rest = ctx.part_world_position(front_button_1)
    jaw_rest_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_shell")

    trigger_upper = trigger_joint.motion_limits.upper
    side_upper = side_joint.motion_limits.upper
    front_0_upper = front_joint_0.motion_limits.upper
    front_1_upper = front_joint_1.motion_limits.upper
    jaw_upper = jaw_joint.motion_limits.upper

    with ctx.pose(
        {
            trigger_joint: trigger_upper,
            side_joint: side_upper,
            front_joint_0: front_0_upper,
            front_joint_1: front_1_upper,
        }
    ):
        trigger_pressed = ctx.part_world_position(trigger)
        side_pressed = ctx.part_world_position(side_button)
        front_0_pressed = ctx.part_world_position(front_button_0)
        front_1_pressed = ctx.part_world_position(front_button_1)

    ctx.check(
        "trigger slides inward into the handle",
        trigger_rest is not None
        and trigger_pressed is not None
        and trigger_pressed[0] < trigger_rest[0] - 0.008,
        details=f"rest={trigger_rest}, pressed={trigger_pressed}",
    )
    ctx.check(
        "side button depresses inward from the side wall",
        side_rest is not None
        and side_pressed is not None
        and side_pressed[1] < side_rest[1] - 0.0015,
        details=f"rest={side_rest}, pressed={side_pressed}",
    )
    ctx.check(
        "front button 0 depresses inward",
        front_0_rest is not None
        and front_0_pressed is not None
        and front_0_pressed[0] < front_0_rest[0] - 0.0015,
        details=f"rest={front_0_rest}, pressed={front_0_pressed}",
    )
    ctx.check(
        "front button 1 depresses inward",
        front_1_rest is not None
        and front_1_pressed is not None
        and front_1_pressed[0] < front_1_rest[0] - 0.0015,
        details=f"rest={front_1_rest}, pressed={front_1_pressed}",
    )

    with ctx.pose({jaw_joint: jaw_upper}):
        jaw_open_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_shell")

    jaw_rest_max_z = jaw_rest_aabb[1][2] if jaw_rest_aabb is not None else None
    jaw_open_max_z = jaw_open_aabb[1][2] if jaw_open_aabb is not None else None
    ctx.check(
        "jaw opens upward from the head",
        jaw_rest_max_z is not None
        and jaw_open_max_z is not None
        and jaw_open_max_z > jaw_rest_max_z + 0.012,
        details=f"rest_max_z={jaw_rest_max_z}, open_max_z={jaw_open_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
