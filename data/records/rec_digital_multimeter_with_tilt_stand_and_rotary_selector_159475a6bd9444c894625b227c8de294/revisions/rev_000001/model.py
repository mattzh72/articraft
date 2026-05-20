from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.094
BODY_H = 0.182
BODY_D = 0.050
WALL = 0.0035
PANEL_T = 0.004
FRONT_Z = BODY_D * 0.5
BACK_Z = -BODY_D * 0.5

DISPLAY_W = 0.054
DISPLAY_H = 0.032
DISPLAY_Y = 0.047

DIAL_Y = -0.006
DIAL_RADIUS = 0.0205

KEY_Y = -0.048
KEY_XS = (-0.023, 0.0, 0.023)
KEY_SLOT_W = 0.0155
KEY_SLOT_H = 0.0105

FLASH_X = 0.028
FLASH_Y = 0.073

JACK_Y = -0.076
JACK_XS = (-0.024, 0.0, 0.024)

HINGE_Y = -0.079
HINGE_Z = -0.0295


def _panel_z() -> float:
    return FRONT_Z - PANEL_T * 0.5


def _body_mesh():
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_H, BODY_D)
        .edges("|Z")
        .fillet(0.011)
        .faces(">Z")
        .shell(-WALL)
    )

    panel = (
        cq.Workplane("XY")
        .box(BODY_W - 0.010, BODY_H - 0.010, PANEL_T)
        .translate((0.0, 0.0, _panel_z()))
    )

    display_cut = (
        cq.Workplane("XY")
        .box(DISPLAY_W, DISPLAY_H, PANEL_T + 0.012)
        .translate((0.0, DISPLAY_Y, _panel_z()))
    )
    dial_cut = (
        cq.Workplane("XY")
        .cylinder(PANEL_T + 0.012, 0.009)
        .translate((0.0, DIAL_Y, _panel_z() - 0.5 * (PANEL_T + 0.012)))
    )
    flash_cut = (
        cq.Workplane("XY")
        .cylinder(PANEL_T + 0.012, 0.006)
        .translate((FLASH_X, FLASH_Y, _panel_z() - 0.5 * (PANEL_T + 0.012)))
    )

    for key_x in KEY_XS:
        panel = panel.cut(
            cq.Workplane("XY")
            .box(KEY_SLOT_W, KEY_SLOT_H, PANEL_T + 0.012)
            .translate((key_x, KEY_Y, _panel_z()))
        )

    for jack_x in JACK_XS:
        panel = panel.cut(
            cq.Workplane("XY")
            .cylinder(PANEL_T + 0.012, 0.0065)
            .translate((jack_x, JACK_Y, _panel_z() - 0.5 * (PANEL_T + 0.012)))
        )

    shell = shell.union(panel.cut(display_cut).cut(dial_cut).cut(flash_cut))
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_multimeter")

    shell_yellow = model.material("shell_yellow", rgba=(0.87, 0.73, 0.14, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.11, 0.12, 0.13, 1.0))
    screen_black = model.material("screen_black", rgba=(0.05, 0.07, 0.08, 1.0))
    jack_red = model.material("jack_red", rgba=(0.72, 0.10, 0.10, 1.0))
    jack_black = model.material("jack_black", rgba=(0.08, 0.08, 0.08, 1.0))
    key_gray = model.material("key_gray", rgba=(0.62, 0.64, 0.66, 1.0))
    button_blue = model.material("button_blue", rgba=(0.15, 0.32, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_mesh(), "multimeter_body"),
        material=shell_yellow,
        name="case_shell",
    )
    body.visual(
        Box((DISPLAY_W + 0.010, DISPLAY_H + 0.010, 0.005)),
        origin=Origin(xyz=(0.0, DISPLAY_Y, FRONT_Z - 0.004)),
        material=screen_black,
        name="display_lens",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.0035),
        origin=Origin(xyz=(0.0, DIAL_Y, FRONT_Z - 0.0015)),
        material=bezel_gray,
        name="dial_bezel",
    )
    for index, jack_x in enumerate(JACK_XS):
        material = jack_black if index == 0 else jack_red if index == 2 else dark_gray
        body.visual(
            Cylinder(radius=0.0062, length=0.008),
            origin=Origin(xyz=(jack_x, JACK_Y, FRONT_Z - 0.0005)),
            material=material,
            name=f"jack_{index}",
        )

    for side_sign in (-1.0, 1.0):
        side_name = "0" if side_sign < 0.0 else "1"
        body.visual(
            Cylinder(radius=0.0038, length=0.014),
            origin=Origin(
                xyz=(side_sign * 0.0205, HINGE_Y, HINGE_Z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=bezel_gray,
            name=f"kickstand_ear_{side_name}",
        )
        body.visual(
            Box((0.014, 0.010, 0.0055)),
            origin=Origin(xyz=(side_sign * 0.0205, HINGE_Y, -0.0262)),
            material=bezel_gray,
            name=f"kickstand_pad_{side_name}",
        )

    body.inertial = Inertial.from_geometry(Box((BODY_W, BODY_H, BODY_D)), mass=0.45)

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.00875)),
        material=dark_gray,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.0165, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=bezel_gray,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0105, 0.0205)),
        material=shell_yellow,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(Cylinder(radius=0.021, length=0.021), mass=0.04)

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0034, length=0.028),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=bezel_gray,
        name="hinge_barrel",
    )
    kickstand.visual(
        Box((0.029, 0.128, 0.006)),
        origin=Origin(xyz=(0.0, 0.064, 0.0015)),
        material=bezel_gray,
        name="stand_bar",
    )
    kickstand.visual(
        Box((0.038, 0.018, 0.007)),
        origin=Origin(xyz=(0.0, 0.123, 0.0012)),
        material=bezel_gray,
        name="foot_pad",
    )
    kickstand.inertial = Inertial.from_geometry(
        Box((0.038, 0.130, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.064, 0.001)),
    )

    function_keys = []
    for index, key_x in enumerate(KEY_XS):
        key = model.part(f"function_key_{index}")
        key.visual(
            Box((0.018, 0.012, 0.0046)),
            origin=Origin(xyz=(0.0, 0.0, 0.0023)),
            material=key_gray,
            name="key_cap",
        )
        key.visual(
            Box((0.009, 0.0045, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=dark_gray,
            name="key_stem",
        )
        key.inertial = Inertial.from_geometry(Box((0.014, 0.010, 0.012)), mass=0.008)
        function_keys.append((key, key_x))

    flashlight_button = model.part("flashlight_button")
    flashlight_button.visual(
        Cylinder(radius=0.0072, length=0.0046),
        origin=Origin(xyz=(0.0, 0.0, 0.00205)),
        material=button_blue,
        name="button_cap",
    )
    flashlight_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0072, length=0.006),
        mass=0.006,
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_Y, FRONT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    for index, (key, key_x) in enumerate(function_keys):
        model.articulation(
            f"body_to_function_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(key_x, KEY_Y, FRONT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.2,
                velocity=0.04,
                lower=0.0,
                upper=0.0035,
            ),
        )
    model.articulation(
        "body_to_flashlight_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flashlight_button,
        origin=Origin(xyz=(FLASH_X, FLASH_Y, FRONT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=0.04,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    kickstand = object_model.get_part("kickstand")
    kickstand_joint = object_model.get_articulation("body_to_kickstand")
    flashlight_button = object_model.get_part("flashlight_button")
    flashlight_joint = object_model.get_articulation("body_to_flashlight_button")
    function_key_0 = object_model.get_part("function_key_0")
    key_joint_0 = object_model.get_articulation("body_to_function_key_0")

    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_body",
        negative_elem="dial_bezel",
        min_gap=0.0,
        max_gap=0.004,
        name="dial sits proud of front bezel",
    )

    with ctx.pose({kickstand_joint: kickstand_joint.motion_limits.upper}):
        ctx.check(
            "kickstand swings behind the body",
            (
                ctx.part_element_world_aabb(kickstand, elem="stand_bar") is not None
                and ctx.part_element_world_aabb(body, elem="case_shell") is not None
                and ctx.part_element_world_aabb(kickstand, elem="stand_bar")[0][2]
                < ctx.part_element_world_aabb(body, elem="case_shell")[0][2] - 0.02
            ),
            details=(
                f"kickstand={ctx.part_element_world_aabb(kickstand, elem='stand_bar')}, "
                f"body={ctx.part_element_world_aabb(body, elem='case_shell')}"
            ),
        )

    key_cap_aabb = ctx.part_element_world_aabb(function_key_0, elem="key_cap")
    jack_aabb = ctx.part_element_world_aabb(body, elem="jack_1")
    ctx.check(
        "function keys stay clearly above the input jacks",
        key_cap_aabb is not None and jack_aabb is not None and key_cap_aabb[0][1] > jack_aabb[1][1] + 0.010,
        details=f"key_cap={key_cap_aabb}, jack={jack_aabb}",
    )

    rest_key_aabb = ctx.part_element_world_aabb(function_key_0, elem="key_cap")
    with ctx.pose({key_joint_0: key_joint_0.motion_limits.upper}):
        pressed_key_aabb = ctx.part_element_world_aabb(function_key_0, elem="key_cap")
    ctx.check(
        "function key presses inward",
        rest_key_aabb is not None
        and pressed_key_aabb is not None
        and pressed_key_aabb[1][2] < rest_key_aabb[1][2] - 0.0025,
        details=f"rest={rest_key_aabb}, pressed={pressed_key_aabb}",
    )

    rest_button_aabb = ctx.part_element_world_aabb(flashlight_button, elem="button_cap")
    with ctx.pose({flashlight_joint: flashlight_joint.motion_limits.upper}):
        pressed_button_aabb = ctx.part_element_world_aabb(flashlight_button, elem="button_cap")
    ctx.check(
        "flashlight button presses inward",
        rest_button_aabb is not None
        and pressed_button_aabb is not None
        and pressed_button_aabb[1][2] < rest_button_aabb[1][2] - 0.002,
        details=f"rest={rest_button_aabb}, pressed={pressed_button_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
