from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.34
BODY_D = 0.39
BODY_H = 0.41

BODY_SIDE_T = 0.022
BODY_BACK_T = 0.02
BODY_FRONT_T = 0.012
BODY_TOP_T = 0.012

CAVITY_W = 0.294
CAVITY_D = 0.30
CAVITY_H = 0.23
CAVITY_Y = 0.01
CAVITY_Z0 = 0.078

OPENING_W = 0.294
OPENING_D = BODY_FRONT_T
OPENING_H = 0.17
OPENING_Z = 0.153

CONTROL_W = OPENING_W + 0.002
CONTROL_H = 0.134
CONTROL_Z = 0.316
CONTROL_T = BODY_FRONT_T
CONTROL_CENTER_Y = -BODY_D / 2.0 + CONTROL_T / 2.0
CONTROL_FRONT_Y = -BODY_D / 2.0

DISPLAY_W = 0.10
DISPLAY_H = 0.034
DISPLAY_T = 0.002
DISPLAY_Z = 0.352

BUTTON_X = 0.058
BUTTON_ZS = (0.346, 0.316, 0.286)
BUTTON_CAP_W = 0.020
BUTTON_CAP_D = 0.003
BUTTON_CAP_H = 0.010
BUTTON_STEM_W = 0.013
BUTTON_STEM_D = 0.008
BUTTON_STEM_H = 0.006
BUTTON_TRAVEL = 0.001

DIAL_Z = 0.28
DIAL_SHAFT_R = 0.007
DIAL_SHAFT_L = 0.010

DRAWER_W = 0.286
DRAWER_D = 0.27
DRAWER_H = 0.142
DRAWER_WALL = 0.004
DRAWER_PANEL_W = 0.304
DRAWER_PANEL_T = 0.03
DRAWER_PANEL_H = 0.16
DRAWER_SLOT_W = 0.060
DRAWER_SLOT_H = 0.036
DRAWER_SLOT_Z = 0.105
DRAWER_TRAVEL = 0.16
DRAWER_ORIGIN = Origin(xyz=(0.0, -BODY_D / 2.0 + BODY_FRONT_T, CAVITY_Z0))

BASKET_W = 0.256
BASKET_D = 0.236
BASKET_H = 0.115
BASKET_WALL = 0.003
BASKET_OFFSET = Origin(xyz=(0.0, 0.015, 0.016))

HANDLE_STRAP_W = 0.052
HANDLE_STRAP_D = 0.042
HANDLE_STRAP_H = 0.026
HANDLE_Z = 0.086
HANDLE_Y = -0.047
HANDLE_W = 0.108
HANDLE_D = 0.032
HANDLE_H = 0.036

RELEASE_CAP_W = 0.038
RELEASE_CAP_D = 0.014
RELEASE_CAP_H = 0.006
RELEASE_STEM_W = 0.024
RELEASE_STEM_D = 0.012
RELEASE_STEM_H = 0.008
RELEASE_TRAVEL = 0.001
RELEASE_ORIGIN = Origin(xyz=(0.0, HANDLE_Y, HANDLE_Z + HANDLE_H / 2.0))


def _button_centers() -> list[tuple[float, float]]:
    centers: list[tuple[float, float]] = []
    for side_x in (-BUTTON_X, BUTTON_X):
        for z in BUTTON_ZS:
            centers.append((side_x, z))
    return centers


def _control_face_shape() -> cq.Workplane:
    face = cq.Workplane("XY").box(CONTROL_W, CONTROL_T, CONTROL_H).translate((0.0, CONTROL_CENTER_Y, CONTROL_Z))

    for x, z in _button_centers():
        hole = cq.Workplane("XY").box(BUTTON_STEM_W + 0.004, 0.03, BUTTON_STEM_H + 0.004).translate((x, CONTROL_CENTER_Y, z))
        face = face.cut(hole)

    display_cut = cq.Workplane("XY").box(DISPLAY_W - 0.002, 0.03, DISPLAY_H - 0.002).translate((0.0, CONTROL_CENTER_Y, DISPLAY_Z))
    dial_cut = cq.Workplane("XY").box(0.024, 0.03, 0.024).translate((0.0, CONTROL_CENTER_Y, DIAL_Z))
    return face.cut(display_cut).cut(dial_cut)


def _dial_mesh():
    dial_geometry = KnobGeometry(
        0.056,
        0.024,
        body_style="skirted",
        top_diameter=0.044,
        skirt=KnobSkirt(0.066, 0.005, flare=0.08),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    return mesh_from_geometry(dial_geometry, "selector_dial")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_air_fryer")

    body_black = model.material("body_black", rgba=(0.12, 0.13, 0.14, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    glossy_black = model.material("glossy_black", rgba=(0.08, 0.09, 0.10, 1.0))
    display_black = model.material("display_black", rgba=(0.05, 0.10, 0.13, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.13, 0.13, 0.14, 1.0))
    basket_black = model.material("basket_black", rgba=(0.11, 0.11, 0.12, 1.0))
    button_gray = model.material("button_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    release_red = model.material("release_red", rgba=(0.80, 0.12, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_SIDE_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + BODY_SIDE_T / 2.0, 0.0, BODY_H / 2.0)),
        material=body_black,
        name="left_wall",
    )
    body.visual(
        Box((BODY_SIDE_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - BODY_SIDE_T / 2.0, 0.0, BODY_H / 2.0)),
        material=body_black,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_SIDE_T + 0.002, BODY_BACK_T, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - BODY_BACK_T / 2.0, BODY_H / 2.0)),
        material=body_black,
        name="back_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_SIDE_T + 0.002, BODY_D - BODY_BACK_T + 0.002, CAVITY_Z0)),
        origin=Origin(xyz=(0.0, -BODY_BACK_T / 2.0, CAVITY_Z0 / 2.0)),
        material=body_black,
        name="base_block",
    )
    body.visual(
        Box((OPENING_W + 0.002, BODY_FRONT_T, CAVITY_Z0 + 0.002)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + BODY_FRONT_T / 2.0, CAVITY_Z0 / 2.0)),
        material=body_black,
        name="lower_front",
    )
    body.visual(
        Box((BODY_SIDE_T, BODY_FRONT_T, BODY_H - CAVITY_Z0)),
        origin=Origin(
            xyz=(
                -BODY_W / 2.0 + BODY_SIDE_T / 2.0,
                -BODY_D / 2.0 + BODY_FRONT_T / 2.0,
                CAVITY_Z0 + (BODY_H - CAVITY_Z0) / 2.0,
            )
        ),
        material=body_black,
        name="front_left_column",
    )
    body.visual(
        Box((BODY_SIDE_T, BODY_FRONT_T, BODY_H - CAVITY_Z0)),
        origin=Origin(
            xyz=(
                BODY_W / 2.0 - BODY_SIDE_T / 2.0,
                -BODY_D / 2.0 + BODY_FRONT_T / 2.0,
                CAVITY_Z0 + (BODY_H - CAVITY_Z0) / 2.0,
            )
        ),
        material=body_black,
        name="front_right_column",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_SIDE_T + 0.002, 0.19, BODY_TOP_T)),
        origin=Origin(xyz=(0.0, -0.088, BODY_H - BODY_TOP_T / 2.0)),
        material=body_black,
        name="top_front_strip",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_SIDE_T + 0.002, 0.095, BODY_TOP_T)),
        origin=Origin(xyz=(0.0, 0.1475, BODY_H - BODY_TOP_T / 2.0)),
        material=body_black,
        name="top_rear_strip",
    )
    body.visual(
        Box((0.072, 0.09, BODY_TOP_T)),
        origin=Origin(xyz=(-0.111, 0.055, BODY_H - BODY_TOP_T / 2.0)),
        material=body_black,
        name="top_left_strip",
    )
    body.visual(
        Box((0.072, 0.09, BODY_TOP_T)),
        origin=Origin(xyz=(0.111, 0.055, BODY_H - BODY_TOP_T / 2.0)),
        material=body_black,
        name="top_right_strip",
    )
    body.visual(
        mesh_from_cadquery(_control_face_shape(), "air_fryer_control_face"),
        material=glossy_black,
        name="control_face",
    )
    body.visual(
        Box((DISPLAY_W, DISPLAY_T, DISPLAY_H)),
        origin=Origin(xyz=(0.0, CONTROL_FRONT_Y - DISPLAY_T / 2.0, DISPLAY_Z)),
        material=display_black,
        name="display_window",
    )
    for index, (foot_x, foot_y) in enumerate(((-0.11, -0.125), (0.11, -0.125), (-0.11, 0.125), (0.11, 0.125))):
        body.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(foot_x, foot_y, 0.0025)),
            material=rubber_black,
            name=f"foot_{index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_W, DRAWER_D, DRAWER_WALL)),
        origin=Origin(xyz=(0.0, DRAWER_D / 2.0, DRAWER_WALL / 2.0)),
        material=drawer_black,
        name="bottom",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_D, DRAWER_H)),
        origin=Origin(xyz=(-DRAWER_W / 2.0 + DRAWER_WALL / 2.0, DRAWER_D / 2.0, DRAWER_H / 2.0)),
        material=drawer_black,
        name="left_wall",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_D, DRAWER_H)),
        origin=Origin(xyz=(DRAWER_W / 2.0 - DRAWER_WALL / 2.0, DRAWER_D / 2.0, DRAWER_H / 2.0)),
        material=drawer_black,
        name="right_wall",
    )
    drawer.visual(
        Box((DRAWER_W - 2.0 * DRAWER_WALL + 0.002, DRAWER_WALL, DRAWER_H)),
        origin=Origin(xyz=(0.0, DRAWER_D - DRAWER_WALL / 2.0, DRAWER_H / 2.0)),
        material=drawer_black,
        name="back_wall",
    )
    drawer.visual(
        Box((0.088, DRAWER_PANEL_T, DRAWER_PANEL_H)),
        origin=Origin(xyz=(-0.108, -DRAWER_PANEL_T / 2.0, DRAWER_PANEL_H / 2.0)),
        material=drawer_black,
        name="front_left",
    )
    drawer.visual(
        Box((0.088, DRAWER_PANEL_T, DRAWER_PANEL_H)),
        origin=Origin(xyz=(0.108, -DRAWER_PANEL_T / 2.0, DRAWER_PANEL_H / 2.0)),
        material=drawer_black,
        name="front_right",
    )
    drawer.visual(
        Box((DRAWER_PANEL_W, DRAWER_PANEL_T, 0.052)),
        origin=Origin(xyz=(0.0, -DRAWER_PANEL_T / 2.0, 0.026)),
        material=drawer_black,
        name="front_bottom",
    )
    drawer.visual(
        Box((DRAWER_PANEL_W, DRAWER_PANEL_T, 0.038)),
        origin=Origin(xyz=(0.0, -DRAWER_PANEL_T / 2.0, 0.141)),
        material=drawer_black,
        name="front_top",
    )
    basket = model.part("basket")
    basket.visual(
        Box((BASKET_W, BASKET_D, BASKET_WALL)),
        origin=Origin(xyz=(0.0, BASKET_D / 2.0, BASKET_WALL / 2.0)),
        material=basket_black,
        name="bottom",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_D, BASKET_H)),
        origin=Origin(xyz=(-BASKET_W / 2.0 + BASKET_WALL / 2.0, BASKET_D / 2.0, BASKET_H / 2.0)),
        material=basket_black,
        name="left_wall",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_D, BASKET_H)),
        origin=Origin(xyz=(BASKET_W / 2.0 - BASKET_WALL / 2.0, BASKET_D / 2.0, BASKET_H / 2.0)),
        material=basket_black,
        name="right_wall",
    )
    basket.visual(
        Box((BASKET_W - 2.0 * BASKET_WALL + 0.002, BASKET_WALL, BASKET_H)),
        origin=Origin(xyz=(0.0, BASKET_D - BASKET_WALL / 2.0, BASKET_H / 2.0)),
        material=basket_black,
        name="back_wall",
    )
    basket.visual(
        Box((BASKET_W - 2.0 * BASKET_WALL + 0.002, BASKET_WALL, 0.092)),
        origin=Origin(xyz=(0.0, BASKET_WALL / 2.0, 0.046)),
        material=basket_black,
        name="front_wall",
    )
    basket.visual(
        Box((HANDLE_STRAP_W, HANDLE_STRAP_D, HANDLE_STRAP_H)),
        origin=Origin(xyz=(0.0, -0.010, HANDLE_Z)),
        material=basket_black,
        name="handle_strap",
    )
    basket.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_Z)),
        material=shell_dark,
        name="handle",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((RELEASE_CAP_W, RELEASE_CAP_D, RELEASE_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, RELEASE_CAP_H / 2.0)),
        material=release_red,
        name="cap",
    )
    release_button.visual(
        Box((RELEASE_STEM_W, RELEASE_STEM_D, RELEASE_STEM_H)),
        origin=Origin(xyz=(0.0, 0.0, -RELEASE_STEM_H / 2.0 + 0.0005)),
        material=release_red,
        name="stem",
    )

    dial = model.part("dial")
    dial.visual(
        _dial_mesh(),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_dark,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=DIAL_SHAFT_R, length=DIAL_SHAFT_L),
        origin=Origin(xyz=(0.0, DIAL_SHAFT_L / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_dark,
        name="dial_shaft",
    )

    for index, (button_x, button_z) in enumerate(_button_centers()):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_CAP_W, BUTTON_CAP_D, BUTTON_CAP_H)),
            origin=Origin(xyz=(0.0, -BUTTON_CAP_D / 2.0, 0.0)),
            material=button_gray,
            name="cap",
        )
        button.visual(
            Box((BUTTON_STEM_W, BUTTON_STEM_D, BUTTON_STEM_H)),
            origin=Origin(xyz=(0.0, BUTTON_STEM_D / 2.0 - 0.0005, 0.0)),
            material=button_gray,
            name="stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_FRONT_Y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=BUTTON_TRAVEL),
        )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=DRAWER_ORIGIN,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=BASKET_OFFSET,
    )
    model.articulation(
        "basket_to_release_button",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=release_button,
        origin=RELEASE_ORIGIN,
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=RELEASE_TRAVEL),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, CONTROL_FRONT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    button_0 = object_model.get_part("button_0")
    release_button = object_model.get_part("release_button")

    ctx.allow_overlap(
        basket,
        release_button,
        elem_a="handle",
        elem_b="stem",
        reason="The release button stem is intentionally nested into the basket handle housing.",
    )

    drawer_joint = object_model.get_articulation("body_to_drawer")
    button_joint = object_model.get_articulation("body_to_button_0")
    release_joint = object_model.get_articulation("basket_to_release_button")

    ctx.expect_within(
        basket,
        drawer,
        axes="xz",
        margin=0.004,
        name="basket stays centered inside the drawer",
    )
    ctx.expect_overlap(
        basket,
        drawer,
        axes="y",
        min_overlap=0.18,
        name="basket remains deeply nested in the drawer",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        margin=0.025,
        name="drawer stays aligned with the body cavity",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        min_overlap=0.20,
        name="drawer is inserted at rest",
    )

    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            margin=0.025,
            name="extended drawer stays aligned with the cavity",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.09,
            name="extended drawer still retains insertion",
        )
        extended_drawer = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends out from the front opening",
        rest_drawer is not None and extended_drawer is not None and extended_drawer[1] < rest_drawer[1] - 0.12,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    rest_button = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed_button = ctx.part_world_position(button_0)
    ctx.check(
        "program button presses inward",
        rest_button is not None and pressed_button is not None and pressed_button[1] > rest_button[1] + 0.0008,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    rest_release = ctx.part_world_position(release_button)
    with ctx.pose({release_joint: RELEASE_TRAVEL}):
        pressed_release = ctx.part_world_position(release_button)
    ctx.check(
        "release button presses downward",
        rest_release is not None and pressed_release is not None and pressed_release[2] < rest_release[2] - 0.0008,
        details=f"rest={rest_release}, pressed={pressed_release}",
    )

    return ctx.report()


object_model = build_object_model()
