from __future__ import annotations

import cadquery as cq
import math

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


def _build_body_shell() -> object:
    handle_profile = [
        (-0.018, 0.000),
        (-0.021, 0.030),
        (-0.024, 0.060),
        (-0.028, 0.090),
        (-0.031, 0.118),
        (-0.031, 0.137),
        (-0.025, 0.148),
        (-0.020, 0.152),
        (-0.014, 0.148),
        (-0.014, 0.118),
        (0.014, 0.118),
        (0.014, 0.148),
        (0.020, 0.152),
        (0.025, 0.148),
        (0.031, 0.137),
        (0.031, 0.118),
        (0.028, 0.090),
        (0.024, 0.060),
        (0.021, 0.030),
        (0.018, 0.000),
    ]

    shell = cq.Workplane("XZ").polyline(handle_profile).close().extrude(0.018, both=True)

    jaw_ring = (
        cq.Workplane("XZ")
        .center(0.000, 0.158)
        .circle(0.028)
        .circle(0.018)
        .extrude(0.014, both=True)
    )
    jaw_gate_cut = cq.Workplane("XY").box(0.070, 0.056, 0.084).translate((0.021, 0.000, 0.164))
    fixed_jaw = jaw_ring.cut(jaw_gate_cut)

    return shell.union(fixed_jaw)


def _build_jaw_gate() -> object:
    hinge_x = 0.017
    hinge_z = 0.180

    jaw_ring = (
        cq.Workplane("XZ")
        .center(0.000, 0.158)
        .circle(0.028)
        .circle(0.018)
        .extrude(0.012, both=True)
    )
    left_cut = cq.Workplane("XY").box(0.060, 0.050, 0.090).translate((-0.028, 0.000, 0.158))
    lower_cut = cq.Workplane("XY").box(0.080, 0.050, 0.046).translate((0.000, 0.000, 0.134))
    hinge_barrel = (
        cq.Workplane("XZ").center(hinge_x, hinge_z).circle(0.0045).extrude(0.012, both=True)
    )

    jaw = jaw_ring.cut(left_cut).cut(lower_cut).union(hinge_barrel)
    return jaw.translate((-hinge_x, 0.000, -hinge_z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_clamp_meter")

    housing = model.material("housing", rgba=(0.93, 0.73, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.17, 0.18, 1.0))
    jaw_dark = model.material("jaw_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.26, 0.36, 0.40, 0.55))
    button_grey = model.material("button_grey", rgba=(0.72, 0.75, 0.77, 1.0))
    side_blue = model.material("side_blue", rgba=(0.23, 0.51, 0.69, 1.0))
    selector_black = model.material("selector_black", rgba=(0.10, 0.10, 0.11, 1.0))
    selector_mark = model.material("selector_mark", rgba=(0.94, 0.94, 0.94, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "clamp_meter_body"),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        Box((0.034, 0.004, 0.060)),
        origin=Origin(xyz=(0.000, -0.020, 0.071)),
        material=rubber,
        name="control_face",
    )
    body.visual(
        Box((0.030, 0.003, 0.052)),
        origin=Origin(xyz=(0.000, -0.0205, 0.040)),
        material=rubber,
        name="front_grip",
    )
    body.visual(
        Box((0.032, 0.004, 0.026)),
        origin=Origin(xyz=(0.000, -0.020, 0.103)),
        material=rubber,
        name="display_bezel",
    )
    body.visual(
        Box((0.024, 0.0015, 0.019)),
        origin=Origin(xyz=(0.000, -0.0224, 0.103)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.0165, length=0.0025),
        origin=Origin(xyz=(0.000, -0.0207, 0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="selector_bezel",
    )
    body.visual(
        Box((0.036, 0.0035, 0.016)),
        origin=Origin(xyz=(0.000, -0.0204, 0.041)),
        material=rubber,
        name="button_panel",
    )
    body.visual(
        Box((0.004, 0.016, 0.020)),
        origin=Origin(xyz=(0.023, 0.000, 0.058)),
        material=rubber,
        name="side_button_pad",
    )
    body.visual(
        Box((0.006, 0.008, 0.020)),
        origin=Origin(xyz=(-0.010, -0.015, 0.109)),
        material=rubber,
        name="trigger_guard_0",
    )
    body.visual(
        Box((0.006, 0.008, 0.020)),
        origin=Origin(xyz=(0.010, -0.015, 0.109)),
        material=rubber,
        name="trigger_guard_1",
    )
    body.visual(
        Box((0.006, 0.028, 0.008)),
        origin=Origin(xyz=(0.024, 0.000, 0.155)),
        material=jaw_dark,
        name="jaw_seat",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_build_jaw_gate(), "clamp_meter_jaw"),
        material=jaw_dark,
        name="jaw_gate",
    )
    jaw.visual(
        Box((0.006, 0.022, 0.008)),
        origin=Origin(xyz=(0.013, 0.000, -0.025)),
        material=jaw_dark,
        name="jaw_tip",
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.017, 0.000, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.2),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.000, -0.005, -0.007)),
        material=rubber,
        name="trigger_stem",
    )
    trigger.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.000, -0.006, -0.018)),
        material=rubber,
        name="trigger_pad",
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.000, -0.018, 0.118)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.10, lower=0.0, upper=0.007),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.014, length=0.009),
        origin=Origin(xyz=(0.000, -0.0045, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=selector_black,
        name="selector_cap",
    )
    selector.visual(
        Box((0.003, 0.002, 0.010)),
        origin=Origin(xyz=(0.000, -0.009, 0.008)),
        material=selector_mark,
        name="selector_pointer",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.000, -0.022, 0.073)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Box((0.012, 0.004, 0.007)),
        origin=Origin(xyz=(0.000, -0.002, 0.0035)),
        material=button_grey,
        name="button_cap",
    )
    model.articulation(
        "body_to_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(-0.012, -0.022, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.0025),
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Box((0.012, 0.004, 0.007)),
        origin=Origin(xyz=(0.000, -0.002, 0.0035)),
        material=button_grey,
        name="button_cap",
    )
    model.articulation(
        "body_to_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.012, -0.022, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.0025),
    )

    side_button = model.part("side_button")
    side_button.visual(
        Box((0.004, 0.012, 0.008)),
        origin=Origin(xyz=(0.002, 0.000, 0.004)),
        material=side_blue,
        name="side_button_cap",
    )
    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(0.025, 0.000, 0.058)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.002),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    trigger = object_model.get_part("trigger")
    selector = object_model.get_part("selector")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    side_button = object_model.get_part("side_button")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    trigger_joint = object_model.get_articulation("body_to_trigger")
    selector_joint = object_model.get_articulation("body_to_selector")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")
    side_joint = object_model.get_articulation("body_to_side_button")

    jaw_limits = jaw_joint.motion_limits
    trigger_limits = trigger_joint.motion_limits
    selector_limits = selector_joint.motion_limits
    button_0_limits = button_0_joint.motion_limits
    button_1_limits = button_1_joint.motion_limits
    side_limits = side_joint.motion_limits

    with ctx.pose({jaw_joint: 0.0}):
        ctx.expect_contact(
            jaw,
            body,
            elem_a="jaw_tip",
            elem_b="jaw_seat",
            name="jaw closes onto the fixed seat",
        )

    if jaw_limits is not None and jaw_limits.upper is not None:
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            ctx.expect_gap(
                jaw,
                body,
                axis="x",
                positive_elem="jaw_tip",
                negative_elem="jaw_seat",
                min_gap=0.009,
                name="jaw tip swings clear of the fixed seat",
            )

    trigger_rest = ctx.part_world_position(trigger)
    if trigger_limits is not None and trigger_limits.upper is not None:
        with ctx.pose({trigger_joint: trigger_limits.upper}):
            trigger_pressed = ctx.part_world_position(trigger)
        ctx.check(
            "trigger retracts upward into the head",
            trigger_rest is not None
            and trigger_pressed is not None
            and trigger_pressed[2] > trigger_rest[2] + 0.005,
            details=f"rest={trigger_rest}, pressed={trigger_pressed}",
        )

    ctx.check(
        "selector uses continuous rotation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_limits is not None
        and selector_limits.lower is None
        and selector_limits.upper is None,
        details=f"type={selector_joint.articulation_type}, limits={selector_limits}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    if button_0_limits is not None and button_0_limits.upper is not None:
        with ctx.pose({button_0_joint: button_0_limits.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_idle = ctx.part_world_position(button_1)
        ctx.check(
            "button_0 depresses independently",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_1_rest is not None
            and button_1_idle is not None
            and button_0_pressed[1] > button_0_rest[1] + 0.0015
            and abs(button_1_idle[1] - button_1_rest[1]) < 1e-9,
            details=(
                f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
                f"button_1_rest={button_1_rest}, button_1_during={button_1_idle}"
            ),
        )

    if button_1_limits is not None and button_1_limits.upper is not None:
        with ctx.pose({button_1_joint: button_1_limits.upper}):
            button_1_pressed = ctx.part_world_position(button_1)
            button_0_idle = ctx.part_world_position(button_0)
        ctx.check(
            "button_1 depresses independently",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_0_rest is not None
            and button_0_idle is not None
            and button_1_pressed[1] > button_1_rest[1] + 0.0015
            and abs(button_0_idle[1] - button_0_rest[1]) < 1e-9,
            details=(
                f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}, "
                f"button_0_rest={button_0_rest}, button_0_during={button_0_idle}"
            ),
        )

    side_rest = ctx.part_world_position(side_button)
    if side_limits is not None and side_limits.upper is not None:
        with ctx.pose({side_joint: side_limits.upper}):
            side_pressed = ctx.part_world_position(side_button)
            button_0_idle = ctx.part_world_position(button_0)
        ctx.check(
            "side button depresses without moving the front keys",
            side_rest is not None
            and side_pressed is not None
            and button_0_rest is not None
            and button_0_idle is not None
            and side_pressed[0] < side_rest[0] - 0.001
            and abs(button_0_idle[1] - button_0_rest[1]) < 1e-9,
            details=(
                f"side_rest={side_rest}, side_pressed={side_pressed}, "
                f"button_0_rest={button_0_rest}, button_0_during={button_0_idle}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
