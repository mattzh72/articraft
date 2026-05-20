from __future__ import annotations

from math import pi

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


BODY_WIDTH = 0.084
BODY_DEPTH = 0.046
BODY_FRONT_Y = BODY_DEPTH / 2.0
BODY_SIDE_X = BODY_WIDTH / 2.0

DISPLAY_CENTER_Z = 0.126
DISPLAY_SIZE = (0.049, 0.0011, 0.034)

DIAL_CENTER_Z = 0.084
DIAL_DIAMETER = 0.047
DIAL_HEIGHT = 0.017

BUTTON_CENTER_Z = 0.037
BUTTON_X_OFFSET = 0.015
BUTTON_TRAVEL = 0.0018

SIDE_BUTTON_CENTER_Z = 0.068
SIDE_BUTTON_TRAVEL = 0.0012

JAW_CENTER_Z = 0.204
JAW_OUTER_RADIUS = 0.034
JAW_INNER_RADIUS = 0.021
JAW_DEPTH = 0.018
JAW_PIVOT_Z = JAW_CENTER_Z + JAW_OUTER_RADIUS
JAW_OPEN_ANGLE = 0.98


def _body_shell_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, 0.118).translate((0.0, 0.0, 0.059))
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=0.100)
        .rect(BODY_WIDTH, BODY_DEPTH)
        .workplane(offset=0.046)
        .rect(0.060, 0.039)
        .loft(combine=True)
    )
    neck = cq.Workplane("XY").box(0.052, 0.038, 0.020).translate((0.0, 0.0, 0.156))

    shell = lower.union(shoulder).union(neck)
    shell = shell.edges("|Z").fillet(0.0065)
    shell = shell.edges(">Z").fillet(0.0035)

    front_plane = cq.Workplane("XZ", origin=(0.0, BODY_FRONT_Y + 0.001, 0.0))
    side_plane = cq.Workplane("YZ", origin=(BODY_SIDE_X + 0.001, 0.0, 0.0))

    display_cut = front_plane.center(0.0, DISPLAY_CENTER_Z).rect(0.056, 0.040).extrude(-0.0045)
    dial_cut = front_plane.center(0.0, DIAL_CENTER_Z).circle(0.027).extrude(-0.0048)
    button_cut = (
        front_plane.pushPoints(
            [(-BUTTON_X_OFFSET, BUTTON_CENTER_Z), (BUTTON_X_OFFSET, BUTTON_CENTER_Z)]
        )
        .rect(0.020, 0.012)
        .extrude(-0.0050)
    )
    side_button_cut = side_plane.center(0.0, SIDE_BUTTON_CENTER_Z).rect(0.015, 0.020).extrude(-0.0014)

    return shell.cut(display_cut).cut(dial_cut).cut(button_cut).cut(side_button_cut)


def _jaw_base_shape() -> cq.Workplane:
    hinge_block = cq.Workplane("XY").box(0.034, 0.032, 0.024).translate((0.0, 0.0, 0.184))
    connector = cq.Workplane("XY").box(0.024, 0.022, 0.030).translate((-0.012, 0.0, 0.167))
    hinge_pin = cq.Workplane("XZ").center(0.012, JAW_PIVOT_Z).circle(0.0036).extrude(0.011, both=True)
    hinge_cheek_top = cq.Workplane("XY").box(0.008, 0.004, 0.048).translate((0.012, 0.013, 0.216))
    hinge_cheek_bottom = cq.Workplane("XY").box(0.008, 0.004, 0.048).translate((0.012, -0.013, 0.216))

    hook = (
        cq.Workplane("XZ")
        .center(0.0, JAW_CENTER_Z)
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(JAW_DEPTH / 2.0, both=True)
    )
    hook = hook.cut(
        cq.Workplane("XY")
        .box(0.095, JAW_DEPTH * 2.0, 0.054)
        .translate((0.004, 0.0, JAW_CENTER_Z + 0.024))
    )
    hook = hook.cut(
        cq.Workplane("XY")
        .box(0.062, JAW_DEPTH * 2.0, 0.100)
        .translate((0.026, 0.0, JAW_CENTER_Z - 0.002))
    )

    return (
        hinge_block.union(connector)
        .union(hook)
        .union(hinge_pin)
        .union(hinge_cheek_top)
        .union(hinge_cheek_bottom)
    )


def _dial_shape() -> cq.Workplane:
    skirt = cq.Workplane("XZ").circle(0.026).extrude(0.0045)
    body = cq.Workplane("XZ").circle(0.022).extrude(0.015).translate((0.0, 0.002, 0.0))
    face = cq.Workplane("XZ").circle(0.0165).extrude(0.017).translate((0.0, 0.0005, 0.0))
    return skirt.union(body).union(face)


def _jaw_shape() -> cq.Workplane:
    segment = (
        cq.Workplane("XZ")
        .center(0.0, -JAW_OUTER_RADIUS)
        .circle(JAW_OUTER_RADIUS - 0.0005)
        .circle(JAW_INNER_RADIUS - 0.0005)
        .extrude(JAW_DEPTH / 2.0, both=True)
    )
    segment = segment.cut(
        cq.Workplane("XY")
        .box(0.080, JAW_DEPTH * 2.0, 0.060)
        .translate((-0.026, 0.0, -0.038))
    )
    segment = segment.cut(
        cq.Workplane("XY")
        .box(0.050, JAW_DEPTH * 2.0, 0.028)
        .translate((-0.022, 0.0, 0.010))
    )
    segment = segment.cut(
        cq.Workplane("XY")
        .box(0.042, JAW_DEPTH * 2.0, 0.088)
        .translate((-0.018, 0.0, -0.020))
    )
    pivot_lug = cq.Workplane("XZ").circle(0.0065).extrude(JAW_DEPTH / 2.0, both=True)
    pivot_lug = pivot_lug.cut(cq.Workplane("XZ").circle(0.0036).extrude(JAW_DEPTH / 2.0 + 0.001, both=True))
    nose = cq.Workplane("XY").box(0.012, JAW_DEPTH, 0.014).translate((0.022, 0.0, -0.046))
    return pivot_lug.union(segment).union(nose)


def _front_button_shape() -> cq.Workplane:
    button = cq.Workplane("XY").box(0.016, 0.0048, 0.0095).translate((0.0, 0.0024, 0.0))
    button = button.edges("|Y").fillet(0.0021)
    button = button.edges(">Y").fillet(0.0011)
    return button


def _side_button_shape() -> cq.Workplane:
    button = cq.Workplane("XY").box(0.0048, 0.0135, 0.0175).translate((0.0024, 0.0, 0.0))
    button = button.edges("|X").fillet(0.0031)
    button = button.edges(">X").fillet(0.0010)
    stem = cq.Workplane("XY").box(0.0016, 0.0070, 0.0110).translate((0.0008, 0.0, 0.0))
    return button.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    shell_yellow = model.material("shell_yellow", rgba=(0.90, 0.74, 0.18, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    dial_black = model.material("dial_black", rgba=(0.10, 0.10, 0.11, 1.0))
    screen_black = model.material("screen_black", rgba=(0.08, 0.12, 0.11, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.28, 0.38, 0.34, 0.95))
    button_gray = model.material("button_gray", rgba=(0.54, 0.58, 0.62, 1.0))
    button_red = model.material("button_red", rgba=(0.73, 0.19, 0.17, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.93, 0.93, 0.91, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=shell_yellow,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_jaw_base_shape(), "jaw_base"),
        material=trim_dark,
        name="jaw_base",
    )
    body.visual(
        Box((0.059, 0.0014, 0.042)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0007, DISPLAY_CENTER_Z)),
        material=screen_black,
        name="display_bezel",
    )
    body.visual(
        Box(DISPLAY_SIZE),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0016, DISPLAY_CENTER_Z)),
        material=screen_glass,
        name="display_glass",
    )
    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "jaw_segment"),
        material=trim_dark,
        name="jaw_shell",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "function_dial"),
        origin=Origin(xyz=(0.0, 0.0165, 0.0)),
        material=dial_black,
        name="dial_knob",
    )
    dial.visual(
        Box((0.0040, 0.0012, 0.0135)),
        origin=Origin(xyz=(0.0, 0.0184, 0.014)),
        material=pointer_white,
        name="dial_pointer",
    )

    side_button = model.part("side_button")
    side_button.visual(
        mesh_from_cadquery(_side_button_shape(), "side_button_cap"),
        material=button_red,
        name="button_cap",
    )

    for button_name in ("front_button_0", "front_button_1"):
        button = model.part(button_name)
        button.visual(
            mesh_from_cadquery(_front_button_shape(), f"{button_name}_cap"),
            material=button_gray,
            name="button_cap",
        )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.012, 0.0, JAW_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=JAW_OPEN_ANGLE, effort=5.0, velocity=2.0),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(BODY_SIDE_X - 0.0004, 0.0, SIDE_BUTTON_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SIDE_BUTTON_TRAVEL, effort=8.0, velocity=0.04),
    )

    for x_offset, button_name in (
        (-BUTTON_X_OFFSET, "front_button_0"),
        (BUTTON_X_OFFSET, "front_button_1"),
    ):
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_name,
            origin=Origin(xyz=(x_offset, BODY_FRONT_Y, BUTTON_CENTER_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=8.0, velocity=0.04),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    dial = object_model.get_part("dial")
    side_button = object_model.get_part("side_button")
    front_button_0 = object_model.get_part("front_button_0")
    front_button_1 = object_model.get_part("front_button_1")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    dial_joint = object_model.get_articulation("body_to_dial")
    side_button_joint = object_model.get_articulation("body_to_side_button")
    front_button_joint_0 = object_model.get_articulation("body_to_front_button_0")
    front_button_joint_1 = object_model.get_articulation("body_to_front_button_1")

    ctx.allow_overlap(
        body,
        jaw,
        elem_a="jaw_base",
        elem_b="jaw_shell",
        reason="The moving clamp jaw is retained on a simplified hinge pin and barrel at the head.",
    )

    ctx.expect_gap(
        dial,
        body,
        axis="y",
        negative_elem="shell",
        max_gap=0.0008,
        max_penetration=0.0,
        name="dial seats against front face",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_b="shell",
        min_overlap=0.035,
        name="dial stays centered under the display",
    )
    ctx.expect_gap(
        front_button_0,
        body,
        axis="y",
        negative_elem="shell",
        max_gap=0.0008,
        max_penetration=0.0,
        name="left front button seats in its pocket",
    )
    ctx.expect_gap(
        front_button_1,
        body,
        axis="y",
        negative_elem="shell",
        max_gap=0.0008,
        max_penetration=0.0,
        name="right front button seats in its pocket",
    )
    ctx.expect_gap(
        side_button,
        body,
        axis="x",
        negative_elem="shell",
        max_gap=0.0008,
        max_penetration=0.0005,
        name="side flashlight button seats on side wall",
    )
    ctx.expect_origin_gap(
        side_button,
        front_button_0,
        axis="z",
        min_gap=0.020,
        name="side flashlight button sits above the lower front controls",
    )

    rest_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    with ctx.pose({dial_joint: pi / 2.0}):
        rotated_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    ctx.check(
        "function dial rotates visibly",
        rest_pointer is not None
        and rotated_pointer is not None
        and rotated_pointer[0] > rest_pointer[0] + 0.010
        and rotated_pointer[2] < rest_pointer[2] - 0.010,
        details=f"rest_pointer={rest_pointer}, rotated_pointer={rotated_pointer}",
    )

    rest_jaw = ctx.part_element_world_aabb(jaw, elem="jaw_shell")
    with ctx.pose({jaw_joint: JAW_OPEN_ANGLE}):
        open_jaw = ctx.part_element_world_aabb(jaw, elem="jaw_shell")
    ctx.check(
        "jaw swings open from the head",
        rest_jaw is not None
        and open_jaw is not None
        and open_jaw[1][0] > rest_jaw[1][0] + 0.016
        and open_jaw[0][2] > rest_jaw[0][2] + 0.010,
        details=f"rest_jaw={rest_jaw}, open_jaw={open_jaw}",
    )

    rest_front_0 = ctx.part_world_position(front_button_0)
    rest_front_1 = ctx.part_world_position(front_button_1)
    with ctx.pose({front_button_joint_0: BUTTON_TRAVEL}):
        pressed_front_0 = ctx.part_world_position(front_button_0)
        other_front_1 = ctx.part_world_position(front_button_1)
    ctx.check(
        "front button 0 depresses independently",
        rest_front_0 is not None
        and pressed_front_0 is not None
        and rest_front_1 is not None
        and other_front_1 is not None
        and pressed_front_0[1] < rest_front_0[1] - 0.0012
        and abs(other_front_1[1] - rest_front_1[1]) < 1e-6,
        details=(
            f"rest_front_0={rest_front_0}, pressed_front_0={pressed_front_0}, "
            f"rest_front_1={rest_front_1}, other_front_1={other_front_1}"
        ),
    )

    with ctx.pose({front_button_joint_1: BUTTON_TRAVEL}):
        pressed_front_1 = ctx.part_world_position(front_button_1)
        other_front_0 = ctx.part_world_position(front_button_0)
    ctx.check(
        "front button 1 depresses independently",
        rest_front_1 is not None
        and pressed_front_1 is not None
        and rest_front_0 is not None
        and other_front_0 is not None
        and pressed_front_1[1] < rest_front_1[1] - 0.0012
        and abs(other_front_0[1] - rest_front_0[1]) < 1e-6,
        details=(
            f"rest_front_1={rest_front_1}, pressed_front_1={pressed_front_1}, "
            f"rest_front_0={rest_front_0}, other_front_0={other_front_0}"
        ),
    )

    rest_side = ctx.part_world_position(side_button)
    with ctx.pose({side_button_joint: SIDE_BUTTON_TRAVEL}):
        pressed_side = ctx.part_world_position(side_button)
        untouched_front_0 = ctx.part_world_position(front_button_0)
    ctx.check(
        "side flashlight button depresses independently",
        rest_side is not None
        and pressed_side is not None
        and rest_front_0 is not None
        and untouched_front_0 is not None
        and pressed_side[0] < rest_side[0] - 0.0008
        and abs(untouched_front_0[1] - rest_front_0[1]) < 1e-6,
        details=(
            f"rest_side={rest_side}, pressed_side={pressed_side}, "
            f"rest_front_0={rest_front_0}, untouched_front_0={untouched_front_0}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
