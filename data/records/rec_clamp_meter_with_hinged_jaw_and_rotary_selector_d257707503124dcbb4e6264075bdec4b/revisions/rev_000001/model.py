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


BODY_WIDTH = 0.108
BODY_THICKNESS = 0.048
BODY_HEIGHT = 0.166
HEAD_WIDTH = 0.120
HEAD_HEIGHT = 0.062
CLAMP_CENTER_Z = 0.225
CLAMP_OUTER_R = 0.056
CLAMP_INNER_R = 0.033
JAW_HINGE_Z = CLAMP_CENTER_Z + CLAMP_OUTER_R


def _body_shell() -> object:
    lower_body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_THICKNESS, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, BODY_HEIGHT * 0.5))
    )
    head = (
        cq.Workplane("XY")
        .box(HEAD_WIDTH, BODY_THICKNESS + 0.004, HEAD_HEIGHT)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, BODY_HEIGHT - 0.012 + HEAD_HEIGHT * 0.5))
    )
    fixed_clamp = (
        cq.Workplane("XZ")
        .center(0.0, CLAMP_CENTER_Z)
        .circle(CLAMP_OUTER_R)
        .circle(CLAMP_INNER_R)
        .extrude(BODY_THICKNESS * 0.5, both=True)
        .cut(
            cq.Workplane("XY")
            .box(0.180, 0.090, 0.100)
            .translate((0.0, 0.0, CLAMP_CENTER_Z + 0.029))
        )
    )
    trigger_bridge = (
        cq.Workplane("XY")
        .box(0.040, BODY_THICKNESS + 0.002, 0.040)
        .edges("|Y")
        .fillet(0.006)
        .translate((0.0, 0.0, 0.162))
    )
    return lower_body.union(head).union(fixed_clamp).union(trigger_bridge)


def _jaw_shape() -> object:
    return (
        cq.Workplane("XZ")
        .center(0.0, -CLAMP_OUTER_R)
        .circle(CLAMP_OUTER_R)
        .circle(CLAMP_INNER_R)
        .extrude((BODY_THICKNESS - 0.006) * 0.5, both=True)
        .cut(
            cq.Workplane("XY")
            .box(0.180, 0.090, 0.090)
            .translate((0.0, 0.0, -0.110))
        )
        .union(
            cq.Workplane("XY")
            .box(0.070, BODY_THICKNESS - 0.010, 0.014)
            .edges("|Y")
            .fillet(0.005)
            .translate((0.0, 0.0, -0.006))
        )
    )


def _stand_frame() -> object:
    outer = (
        cq.Workplane("XY")
        .box(0.084, 0.010, 0.118)
        .translate((0.0, -0.006, 0.059))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.062, 0.014, 0.088)
        .translate((0.0, -0.006, 0.060))
    )
    bottom_foot = (
        cq.Workplane("XY")
        .box(0.050, 0.010, 0.014)
        .translate((0.0, -0.006, 0.004))
    )
    return outer.cut(inner).union(bottom_foot)

def _button_cap(width: float) -> object:
    return (
        cq.Workplane("XY")
        .box(width, 0.009, 0.012)
        .edges("|Y")
        .fillet(0.0025)
        .translate((0.0, 0.0045, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    shell = model.material("shell", rgba=(0.83, 0.74, 0.19, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.18, 0.19, 0.20, 1.0))
    trim = model.material("trim", rgba=(0.30, 0.31, 0.33, 1.0))
    stand_mat = model.material("stand", rgba=(0.22, 0.23, 0.24, 1.0))
    screen = model.material("screen", rgba=(0.16, 0.33, 0.39, 0.55))
    button_mat = model.material("button", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "clamp_meter_body"),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Box((0.084, 0.003, 0.066)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.0015, 0.156)),
        material=dark_panel,
        name="display_bezel",
    )
    body.visual(
        Box((0.060, 0.002, 0.038)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.004, 0.160)),
        material=screen,
        name="screen",
    )
    body.visual(
        Box((0.072, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.001, 0.110)),
        material=trim,
        name="selector_recess",
    )
    body.visual(
        Box((0.074, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.002, 0.183)),
        material=trim,
        name="button_panel",
    )
    body.visual(
        Box((0.086, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -BODY_THICKNESS * 0.5 - 0.003, 0.048)),
        material=trim,
        name="stand_seat",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "clamp_meter_jaw"),
        material=shell,
        name="jaw_shell",
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.0, JAW_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_frame(), "clamp_meter_stand"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stand_mat,
        name="stand_frame",
    )
    stand.visual(
        Box((0.050, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.004)),
        material=stand_mat,
        name="stand_foot",
    )
    stand.visual(
        Box((0.070, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.112)),
        material=stand_mat,
        name="stand_top",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -BODY_THICKNESS * 0.5 - 0.005, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="selector_skirt",
    )
    selector.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="selector_knob",
    )
    selector.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="selector_crown",
    )
    selector.visual(
        Box((0.004, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.020, 0.011)),
        material=trim,
        name="selector_pointer",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.003, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=6.0,
        ),
    )

    button_specs = (
        ("hold_button", -0.024),
        ("range_button", 0.0),
        ("light_button", 0.024),
    )
    for part_name, x_pos in button_specs:
        button = model.part(part_name)
        button.visual(
            mesh_from_cadquery(_button_cap(0.018), part_name),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_THICKNESS * 0.5 + 0.0035, 0.183)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    stand = object_model.get_part("stand")
    selector = object_model.get_part("selector")
    hold_button = object_model.get_part("hold_button")
    range_button = object_model.get_part("range_button")
    light_button = object_model.get_part("light_button")
    jaw_hinge = object_model.get_articulation("body_to_jaw")
    stand_hinge = object_model.get_articulation("body_to_stand")
    selector_joint = object_model.get_articulation("body_to_selector")
    hold_joint = object_model.get_articulation("body_to_hold_button")
    range_joint = object_model.get_articulation("body_to_range_button")
    light_joint = object_model.get_articulation("body_to_light_button")

    with ctx.pose({jaw_hinge: 0.0}):
        ctx.expect_overlap(
            jaw,
            body,
            axes="x",
            min_overlap=0.090,
            name="jaw spans the clamp width",
        )

    closed_jaw_aabb = ctx.part_world_aabb(jaw)
    with ctx.pose({jaw_hinge: jaw_hinge.motion_limits.upper}):
        open_jaw_aabb = ctx.part_world_aabb(jaw)
    ctx.check(
        "jaw opens forward",
        closed_jaw_aabb is not None
        and open_jaw_aabb is not None
        and open_jaw_aabb[1][1] > closed_jaw_aabb[1][1] + 0.025,
        details=f"closed={closed_jaw_aabb}, open={open_jaw_aabb}",
    )

    with ctx.pose({stand_hinge: 0.0}):
        ctx.expect_gap(
            body,
            stand,
            axis="y",
            max_gap=0.010,
            max_penetration=0.0,
            name="folded stand stays close to the rear seat",
        )

    closed_top = ctx.part_element_world_aabb(stand, elem="stand_top")
    with ctx.pose({stand_hinge: stand_hinge.motion_limits.upper}):
        open_top = ctx.part_element_world_aabb(stand, elem="stand_top")
    ctx.check(
        "stand folds rearward",
        closed_top is not None
        and open_top is not None
        and open_top[0][1] < closed_top[0][1] - 0.030,
        details=f"closed={closed_top}, open={open_top}",
    )

    ctx.expect_contact(
        selector,
        body,
        elem_a="selector_skirt",
        elem_b="selector_recess",
        name="selector seats on the front recess",
    )

    selector_rest = ctx.part_element_world_aabb(selector, elem="selector_pointer")
    with ctx.pose({selector_joint: math.pi / 2.0}):
        selector_turn = ctx.part_element_world_aabb(selector, elem="selector_pointer")
    ctx.check(
        "selector rotates about the front axis",
        selector_rest is not None
        and selector_turn is not None
        and abs(((selector_turn[0][0] + selector_turn[1][0]) * 0.5)) > 0.008
        and abs(((selector_rest[0][2] + selector_rest[1][2]) * 0.5)) > 0.008,
        details=f"rest={selector_rest}, turn={selector_turn}",
    )

    for button_part, button_joint, label in (
        (hold_button, hold_joint, "hold"),
        (range_button, range_joint, "range"),
        (light_button, light_joint, "light"),
    ):
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"{label} button presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
