from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.086
CASE_H = 0.190
CASE_D = 0.041

BUMPER_W = 0.098
BUMPER_H = 0.205
BUMPER_D = 0.055

FRONT_Z = CASE_D * 0.5
BACK_Z = -CASE_D * 0.5

DISPLAY_Y = 0.058
SELECTOR_Y = 0.004
BUTTON_BANK_Y = -0.050
JACK_Y = -0.080
JACK_XS = (-0.022, 0.0, 0.022)
JACK_RADIUS = 0.0046

SELECTOR_RADIUS = 0.028
SELECTOR_HEIGHT = 0.013

BUTTON_W = 0.015
BUTTON_H = 0.008
BUTTON_FACE_T = 0.0016
BUTTON_TRAVEL = 0.0014
BUTTON_RECESS = 0.0018
BUTTON_FRAME_Z = FRONT_Z - BUTTON_RECESS
BUTTON_FACE_Z = 0.0014
BUTTON_XS = (-0.019, 0.0, 0.019)

ROCKER_T = 0.004
ROCKER_H = 0.014
ROCKER_W = 0.007
ROCKER_CENTER_X = (CASE_W * 0.5) + 0.0033
ROCKER_CENTER_Y = 0.016

STAND_W = 0.056
STAND_LEN = 0.110
STAND_T = 0.0035


def _make_case_shape() -> cq.Workplane:
    case = cq.Workplane("XY").box(CASE_W, CASE_H, CASE_D)
    case = case.edges("|Z").fillet(0.010)

    case = (
        case.faces(">Z")
        .workplane()
        .center(0.0, DISPLAY_Y)
        .rect(0.080, 0.056)
        .cutBlind(-0.0022)
    )
    case = (
        case.faces(">Z")
        .workplane()
        .center(0.0, SELECTOR_Y)
        .circle(0.031)
        .cutBlind(-0.0013)
    )
    case = (
        case.faces(">Z")
        .workplane()
        .center(0.0, BUTTON_BANK_Y)
        .rect(0.062, 0.018)
        .cutBlind(-BUTTON_RECESS)
    )
    case = (
        case.faces(">Z")
        .workplane()
        .center(0.0, JACK_Y)
        .rect(0.074, 0.026)
        .cutBlind(-0.0012)
    )
    for jack_x in JACK_XS:
        case = (
            case.faces(">Z")
            .workplane()
            .center(jack_x, JACK_Y)
            .circle(JACK_RADIUS)
            .cutBlind(-0.010)
        )

    return case


def _make_bumper_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BUMPER_W, BUMPER_H, BUMPER_D)
    outer = outer.edges("|Z").fillet(0.014)

    inner = cq.Workplane("XY").box(CASE_W + 0.004, CASE_H + 0.004, BUMPER_D - 0.008)
    shell = outer.cut(inner)

    front_opening = cq.Workplane("XY").box(BUMPER_W - 0.024, BUMPER_H - 0.034, 0.020)
    front_opening = front_opening.translate((0.0, 0.0, BUMPER_D * 0.5 - 0.010))
    back_opening = cq.Workplane("XY").box(BUMPER_W - 0.022, BUMPER_H - 0.030, 0.020)
    back_opening = back_opening.translate((0.0, 0.0, -BUMPER_D * 0.5 + 0.010))

    return shell.cut(front_opening).cut(back_opening)


def _make_selector_shape() -> cq.Workplane:
    selector = cq.Workplane("XY").circle(SELECTOR_RADIUS).extrude(0.002)
    selector = selector.faces(">Z").workplane().circle(0.022).extrude(SELECTOR_HEIGHT - 0.002)
    pointer = (
        cq.Workplane("XY")
        .box(0.0035, 0.017, 0.0014, centered=(True, False, False))
        .translate((0.0, 0.0045, SELECTOR_HEIGHT - 0.0012))
    )
    return selector.union(pointer)


def _make_button_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(BUTTON_W, BUTTON_H, BUTTON_FACE_T, centered=(True, True, False))


def _make_rocker_shape() -> cq.Workplane:
    rocker = cq.Workplane("XY").box(ROCKER_T, ROCKER_H, ROCKER_W)
    crown = cq.Workplane("XY").box(0.002, ROCKER_H * 0.8, ROCKER_W * 0.7).translate((0.0012, 0.0, 0.0))
    return rocker.union(crown)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_multimeter")

    housing = model.material("housing", rgba=(0.23, 0.25, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.86, 0.68, 0.18, 1.0))
    bezel = model.material("bezel", rgba=(0.09, 0.10, 0.11, 1.0))
    screen = model.material("screen", rgba=(0.20, 0.32, 0.22, 0.92))
    control_dark = model.material("control_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    button_gray = model.material("button_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    jack_black = model.material("jack_black", rgba=(0.06, 0.06, 0.07, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_case_shape(), "multimeter_case_shell"),
        material=housing,
        name="case_shell",
    )
    body.visual(
        mesh_from_cadquery(_make_bumper_shape(), "multimeter_bumper_shell"),
        material=rubber,
        name="bumper_shell",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.064, 0.032),
                (0.082, 0.052),
                0.004,
                opening_corner_radius=0.004,
                outer_corner_radius=0.008,
                center=False,
            ),
            "multimeter_display_bezel",
        ),
        origin=Origin(xyz=(0.0, DISPLAY_Y, FRONT_Z - 0.0002)),
        material=bezel,
        name="display_bezel",
    )
    body.visual(
        Box((0.064, 0.032, 0.0020)),
        origin=Origin(xyz=(0.0, DISPLAY_Y, FRONT_Z - 0.0011)),
        material=screen,
        name="screen",
    )
    for index, jack_x in enumerate(JACK_XS):
        body.visual(
            Cylinder(radius=0.0068, length=0.0045),
            origin=Origin(xyz=(jack_x, JACK_Y, FRONT_Z - 0.0005)),
            material=jack_black,
            name=f"jack_{index}",
        )

    selector_mesh = mesh_from_cadquery(_make_selector_shape(), "multimeter_selector")
    selector = model.part("selector")
    selector.visual(selector_mesh, material=control_dark, name="selector_cap")

    button_mesh = mesh_from_cadquery(_make_button_shape(), "multimeter_mode_button")
    for index in range(3):
        button = model.part(f"mode_button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, 0.0, BUTTON_FACE_Z)),
            material=button_gray,
            name="button_cap",
        )

    rocker = model.part("power_rocker")
    rocker.visual(
        mesh_from_cadquery(_make_rocker_shape(), "multimeter_power_rocker"),
        material=control_dark,
        name="switch_cap",
    )

    stand = model.part("stand")
    stand.visual(
        Box((STAND_W, STAND_LEN, STAND_T)),
        origin=Origin(xyz=(0.0, STAND_LEN * 0.5, -STAND_T * 0.5)),
        material=stand_gray,
        name="stand_panel",
    )
    stand.visual(
        Box((STAND_W * 0.75, 0.012, 0.005)),
        origin=Origin(xyz=(0.0, STAND_LEN - 0.006, -0.0045)),
        material=stand_gray,
        name="stand_foot",
    )
    stand.visual(
        Cylinder(radius=0.0032, length=STAND_W * 0.74),
        origin=Origin(xyz=(0.0, 0.0, -0.0032), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stand_gray,
        name="stand_barrel",
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, SELECTOR_Y, FRONT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    for index, button_x in enumerate(BUTTON_XS):
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"mode_button_{index}",
            origin=Origin(xyz=(button_x, BUTTON_BANK_Y, BUTTON_FRAME_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(ROCKER_CENTER_X, ROCKER_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.18,
            upper=0.18,
        ),
    )

    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.070, BACK_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=1.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    rocker = object_model.get_part("power_rocker")
    stand = object_model.get_part("stand")
    selector_joint = object_model.get_articulation("body_to_selector")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.check(
        "selector_joint_is_continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_joint.articulation_type!r}",
    )
    ctx.check(
        "selector_axis_faces_forward",
        tuple(selector_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={selector_joint.axis!r}",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="xy",
        elem_a="selector_cap",
        elem_b="case_shell",
        min_overlap=0.042,
        name="selector sits centered in the front dial area",
    )
    ctx.expect_gap(
        selector,
        body,
        axis="z",
        positive_elem="selector_cap",
        negative_elem="case_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector seats on the front case without sinking in",
    )

    button_parts = [object_model.get_part(f"mode_button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_mode_button_{index}") for index in range(3)]
    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"mode_button_{index}_depresses_inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.001,
            details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    ctx.check(
        "power_rocker_is_revolute",
        rocker_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={rocker_joint.articulation_type!r}",
    )
    ctx.expect_origin_distance(
        rocker,
        button_parts[1],
        axes="x",
        min_dist=0.040,
        name="side rocker stays separate from the front button bank",
    )

    with ctx.pose({rocker_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(rocker)
    rocker_upper = rocker_joint.motion_limits.upper if rocker_joint.motion_limits is not None else None
    with ctx.pose({rocker_joint: rocker_upper if rocker_upper is not None else 0.0}):
        tilted_aabb = ctx.part_world_aabb(rocker)
    rest_x_span = None if rest_aabb is None else float(rest_aabb[1][0] - rest_aabb[0][0])
    tilted_x_span = None if tilted_aabb is None else float(tilted_aabb[1][0] - tilted_aabb[0][0])
    ctx.check(
        "power_rocker_tilts_about_side_pivot",
        rest_x_span is not None and tilted_x_span is not None and tilted_x_span > rest_x_span + 0.001,
        details=f"rest_x_span={rest_x_span!r}, tilted_x_span={tilted_x_span!r}",
    )

    ctx.expect_gap(
        body,
        stand,
        axis="z",
        positive_elem="case_shell",
        negative_elem="stand_panel",
        max_gap=0.002,
        max_penetration=0.0,
        name="rear stand folds nearly flush against the back case",
    )

    stand_rest_aabb = ctx.part_element_world_aabb(stand, elem="stand_foot")
    stand_upper = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else None
    with ctx.pose({stand_joint: stand_upper if stand_upper is not None else 0.0}):
        stand_open_aabb = ctx.part_element_world_aabb(stand, elem="stand_foot")
    ctx.check(
        "rear_stand_swings_backward",
        stand_rest_aabb is not None and stand_open_aabb is not None and stand_open_aabb[0][2] < stand_rest_aabb[0][2] - 0.035,
        details=f"rest={stand_rest_aabb!r}, open={stand_open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
