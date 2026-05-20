from __future__ import annotations

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
)

BODY_DEPTH = 0.034
HANDLE_WIDTH = 0.058
HEAD_WIDTH = 0.074
JAW_DEPTH = 0.026
HINGE_Z = 0.184


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def _build_housing_shape():
    housing = _cq_box((HANDLE_WIDTH, BODY_DEPTH, 0.108), (0.0, 0.0, 0.054))
    housing = housing.union(_cq_box((HEAD_WIDTH, BODY_DEPTH, 0.042), (0.0, 0.0, 0.118)))
    housing = housing.union(_cq_box((0.024, BODY_DEPTH, 0.024), (0.0, 0.0, 0.145)))
    housing = housing.union(_cq_box((0.042, BODY_DEPTH, 0.018), (0.0, 0.0, 0.016)))
    housing = housing.cut(_cq_box((0.042, BODY_DEPTH + 0.004, 0.056), (0.021, 0.0, 0.136)))
    housing = housing.cut(_cq_box((0.024, 0.022, 0.020), (0.0, BODY_DEPTH * 0.34, 0.126)))
    housing = housing.edges("|Z").fillet(0.005)
    return housing


def _build_fixed_jaw_shape():
    jaw = _cq_box((0.012, 0.026, 0.040), (-0.006, 0.0, 0.148))
    jaw = jaw.union(_cq_box((0.034, JAW_DEPTH, 0.010), (-0.017, 0.0, 0.176)))
    jaw = jaw.union(_cq_box((0.012, JAW_DEPTH, 0.050), (-0.032, 0.0, 0.149)))
    jaw = jaw.union(_cq_box((0.026, JAW_DEPTH, 0.010), (-0.020, 0.0, 0.124)))
    jaw = jaw.union(_cq_box((0.006, 0.010, 0.010), (-0.001, 0.0, HINGE_Z - 0.001)))
    return jaw


def _build_moving_jaw_shape():
    jaw = _cq_box((0.030, JAW_DEPTH, 0.010), (0.020, 0.0, -0.006))
    jaw = jaw.union(_cq_box((0.012, JAW_DEPTH, 0.046), (0.033, 0.0, -0.033)))
    jaw = jaw.union(_cq_box((0.026, JAW_DEPTH, 0.010), (0.021, 0.0, -0.055)))
    jaw = jaw.union(_y_cylinder(0.0055, 0.008, (0.0, -0.009, 0.0)))
    jaw = jaw.union(_y_cylinder(0.0055, 0.008, (0.0, 0.009, 0.0)))
    return jaw


def _build_trigger_shape():
    stem = _cq_box((0.014, 0.008, 0.022), (0.0, 0.004, -0.011))
    pad = _cq_box((0.024, 0.012, 0.010), (0.0, 0.006, -0.025))
    return stem.union(pad).edges("|X").fillet(0.0025)


def _build_dial_shape():
    dial = _y_cylinder(0.020, 0.004, (0.0, 0.002, 0.0))
    dial = dial.union(_y_cylinder(0.017, 0.012, (0.0, 0.006, 0.0)))
    return dial


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_clamp_meter")

    body_orange = model.material("body_orange", rgba=(0.95, 0.53, 0.12, 1.0))
    body_black = model.material("body_black", rgba=(0.12, 0.13, 0.15, 1.0))
    panel_black = model.material("panel_black", rgba=(0.07, 0.08, 0.09, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.91, 0.80, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.43, 0.62, 0.58, 0.55))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_housing_shape(), "clamp_meter_body"),
        material=body_orange,
        name="housing_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_fixed_jaw_shape(), "clamp_meter_fixed_jaw"),
        material=body_black,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.050, 0.004, 0.098)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - 0.001, 0.089)),
        material=panel_black,
        name="front_panel",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_build_moving_jaw_shape(), "clamp_meter_moving_jaw"),
        origin=Origin(rpy=(0.0, -0.10, 0.0)),
        material=body_black,
        name="jaw_arm",
    )

    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_build_trigger_shape(), "clamp_meter_trigger"),
        material=body_black,
        name="trigger_body",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial_shape(), "clamp_meter_dial"),
        material=panel_black,
        name="dial_knob",
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.030, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0015, 0.010)),
        material=glass,
        name="screen_glass",
    )

    side_button = model.part("side_button")
    side_button.visual(
        Box((0.004, 0.016, 0.009)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=button_yellow,
        name="side_button_cap",
    )

    front_button_0 = model.part("front_button_0")
    front_button_0.visual(
        Box((0.014, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=button_yellow,
        name="front_button_cap",
    )

    front_button_1 = model.part("front_button_1")
    front_button_1.visual(
        Box((0.014, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=button_yellow,
        name="front_button_cap",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.004, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.2, lower=0.0, upper=0.9),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - 0.001, 0.132)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=0.12, lower=0.0, upper=0.010),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.010, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=8.0),
    )
    model.articulation(
        "body_to_screen",
        ArticulationType.FIXED,
        parent=body,
        child=screen,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - 0.0015, 0.114)),
    )
    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(HEAD_WIDTH / 2.0, 0.0, 0.100)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0018),
    )
    model.articulation(
        "body_to_front_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_0,
        origin=Origin(xyz=(-0.013, BODY_DEPTH / 2.0 - 0.0015, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0016),
    )
    model.articulation(
        "body_to_front_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_1,
        origin=Origin(xyz=(0.013, BODY_DEPTH / 2.0 - 0.0015, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0016),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    trigger = object_model.get_part("trigger")
    dial = object_model.get_part("dial")
    screen = object_model.get_part("screen")
    side_button = object_model.get_part("side_button")
    front_button_0 = object_model.get_part("front_button_0")
    front_button_1 = object_model.get_part("front_button_1")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    trigger_joint = object_model.get_articulation("body_to_trigger")
    side_button_joint = object_model.get_articulation("body_to_side_button")
    front_button_0_joint = object_model.get_articulation("body_to_front_button_0")
    front_button_1_joint = object_model.get_articulation("body_to_front_button_1")

    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="dial_knob",
        negative_elem="front_panel",
        max_gap=0.012,
        max_penetration=0.004,
        name="dial seats on the front panel",
    )
    ctx.expect_gap(
        screen,
        body,
        axis="y",
        positive_elem="screen_glass",
        negative_elem="front_panel",
        max_gap=0.004,
        max_penetration=0.003,
        name="screen sits flush in the front panel",
    )
    ctx.expect_gap(
        front_button_0,
        body,
        axis="y",
        positive_elem="front_button_cap",
        negative_elem="front_panel",
        max_gap=0.004,
        max_penetration=0.003,
        name="front button 0 is panel-mounted",
    )
    ctx.expect_gap(
        front_button_1,
        body,
        axis="y",
        positive_elem="front_button_cap",
        negative_elem="front_panel",
        max_gap=0.004,
        max_penetration=0.003,
        name="front button 1 is panel-mounted",
    )
    ctx.expect_gap(
        side_button,
        body,
        axis="x",
        positive_elem="side_button_cap",
        negative_elem="housing_shell",
        max_gap=0.004,
        max_penetration=0.003,
        name="side button is side-mounted",
    )
    ctx.expect_origin_gap(
        side_button,
        front_button_0,
        axis="z",
        min_gap=0.030,
        name="side flashlight button sits above the front buttons",
    )
    ctx.expect_origin_gap(
        screen,
        dial,
        axis="z",
        min_gap=0.024,
        max_gap=0.055,
        name="screen is above the function dial",
    )
    ctx.expect_origin_gap(
        dial,
        front_button_0,
        axis="z",
        min_gap=0.020,
        max_gap=0.045,
        name="front buttons sit below the function dial",
    )

    jaw_rest = ctx.part_world_aabb(jaw)
    jaw_limits = jaw_joint.motion_limits
    jaw_open = None
    if jaw_limits is not None and jaw_limits.upper is not None:
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            jaw_open = ctx.part_world_aabb(jaw)
    ctx.check(
        "jaw opens outward",
        jaw_rest is not None
        and jaw_open is not None
        and jaw_open[1][0] > jaw_rest[1][0] + 0.012,
        details=f"rest={jaw_rest!r}, open={jaw_open!r}",
    )

    trigger_rest = ctx.part_world_aabb(trigger)
    trigger_limits = trigger_joint.motion_limits
    trigger_pulled = None
    if trigger_limits is not None and trigger_limits.upper is not None:
        with ctx.pose({trigger_joint: trigger_limits.upper}):
            trigger_pulled = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger retracts upward",
        trigger_rest is not None
        and trigger_pulled is not None
        and trigger_pulled[0][2] > trigger_rest[0][2] + 0.007,
        details=f"rest={trigger_rest!r}, pulled={trigger_pulled!r}",
    )

    side_rest = ctx.part_world_aabb(side_button)
    side_limits = side_button_joint.motion_limits
    side_pressed = None
    if side_limits is not None and side_limits.upper is not None:
        with ctx.pose({side_button_joint: side_limits.upper}):
            side_pressed = ctx.part_world_aabb(side_button)
    ctx.check(
        "side button presses inward",
        side_rest is not None
        and side_pressed is not None
        and side_pressed[1][0] < side_rest[1][0] - 0.001,
        details=f"rest={side_rest!r}, pressed={side_pressed!r}",
    )

    button_0_rest = ctx.part_world_aabb(front_button_0)
    button_0_pressed = None
    button_0_limits = front_button_0_joint.motion_limits
    if button_0_limits is not None and button_0_limits.upper is not None:
        with ctx.pose({front_button_0_joint: button_0_limits.upper}):
            button_0_pressed = ctx.part_world_aabb(front_button_0)
    ctx.check(
        "front button 0 presses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[1][1] < button_0_rest[1][1] - 0.001,
        details=f"rest={button_0_rest!r}, pressed={button_0_pressed!r}",
    )

    button_1_rest = ctx.part_world_aabb(front_button_1)
    button_1_pressed = None
    button_1_limits = front_button_1_joint.motion_limits
    if button_1_limits is not None and button_1_limits.upper is not None:
        with ctx.pose({front_button_1_joint: button_1_limits.upper}):
            button_1_pressed = ctx.part_world_aabb(front_button_1)
    ctx.check(
        "front button 1 presses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[1][1] < button_1_rest[1][1] - 0.001,
        details=f"rest={button_1_rest!r}, pressed={button_1_pressed!r}",
    )

    return ctx.report()


object_model = build_object_model()
