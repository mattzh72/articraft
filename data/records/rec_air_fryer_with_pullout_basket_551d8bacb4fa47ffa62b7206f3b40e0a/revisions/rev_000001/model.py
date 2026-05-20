from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


BODY_D = 0.340
BODY_W = 0.300
BODY_H = 0.385
BODY_CORNER_R = 0.026
BODY_FRONT_FRAME_T = 0.020

CAVITY_D = 0.252
CAVITY_W = 0.238
CAVITY_H = 0.218
CAVITY_Z = 0.060

OPENING_W = 0.246
OPENING_H = 0.166
OPENING_Z = 0.058

DRAWER_D = 0.248
DRAWER_W = 0.226
DRAWER_H = 0.120
DRAWER_WALL = 0.003
DRAWER_FRONT_T = 0.018
DRAWER_FRONT_W = 0.258
DRAWER_FRONT_H = 0.172
DRAWER_TRAVEL = 0.170

BASKET_D = 0.214
BASKET_W = 0.202
BASKET_H = 0.098
BASKET_WALL = 0.002

HANDLE_D = 0.056
HANDLE_W = 0.148
HANDLE_H = 0.034

DRAWER_ORIGIN_X = BODY_D * 0.5 - BODY_FRONT_FRAME_T - CAVITY_D + 0.003
DRAWER_ORIGIN_Z = CAVITY_Z + 0.004


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_D, BODY_W, BODY_H, centered=(True, True, False))

    cavity = (
        cq.Workplane("XY")
        .box(CAVITY_D, CAVITY_W, CAVITY_H, centered=(True, True, False))
        .translate((BODY_D * 0.5 - BODY_FRONT_FRAME_T - CAVITY_D * 0.5, 0.0, CAVITY_Z))
    )
    opening = (
        cq.Workplane("XY")
        .box(BODY_FRONT_FRAME_T + 0.010, OPENING_W, OPENING_H, centered=(True, True, False))
        .translate((BODY_D * 0.5 - BODY_FRONT_FRAME_T * 0.5, 0.0, OPENING_Z))
    )
    control_recess = (
        cq.Workplane("XY")
        .box(0.010, 0.180, 0.110, centered=(True, True, False))
        .translate((BODY_D * 0.5 - 0.005, 0.0, 0.248))
    )
    body = body.cut(cavity).cut(opening).cut(control_recess)

    feet = []
    for x in (-0.110, 0.110):
        for y in (-0.100, 0.100):
            feet.append(
                cq.Workplane("XY")
                .cylinder(0.010, 0.010)
                .translate((x, y, 0.010))
            )
    for foot in feet:
        body = body.union(foot)

    return body


def _drawer_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(DRAWER_D, DRAWER_W, DRAWER_H, centered=(False, True, False))

    inner = (
        cq.Workplane("XY")
        .box(
            DRAWER_D - DRAWER_FRONT_T - DRAWER_WALL,
            DRAWER_W - 2.0 * DRAWER_WALL,
            DRAWER_H - DRAWER_WALL,
            centered=(False, True, False),
        )
        .translate((DRAWER_WALL, 0.0, DRAWER_WALL))
    )
    shell = shell.cut(inner)

    fascia = (
        cq.Workplane("XY")
        .box(DRAWER_FRONT_T, DRAWER_FRONT_W, DRAWER_FRONT_H, centered=(False, True, False))
        .translate((DRAWER_D, 0.0, 0.0))
    )
    shell = shell.union(fascia)

    window = (
        cq.Workplane("XY")
        .box(DRAWER_FRONT_T + 0.006, 0.150, 0.074, centered=(False, True, False))
        .translate((DRAWER_D - 0.003, 0.0, 0.056))
    )
    shell = shell.cut(window)

    return shell


def _basket_shape() -> cq.Workplane:
    basket = cq.Workplane("XY").box(BASKET_D, BASKET_W, BASKET_H, centered=(False, True, False))

    inner = (
        cq.Workplane("XY")
        .box(
            BASKET_D - 2.0 * BASKET_WALL,
            BASKET_W - 2.0 * BASKET_WALL,
            BASKET_H - BASKET_WALL,
            centered=(False, True, False),
        )
        .translate((BASKET_WALL, 0.0, BASKET_WALL))
    )
    basket = basket.cut(inner)
    return basket


def _handle_shape() -> cq.Workplane:
    handle = cq.Workplane("XY").box(HANDLE_D, HANDLE_W, HANDLE_H, centered=(False, True, False))

    finger_recess = (
        cq.Workplane("XY")
        .box(HANDLE_D * 0.72, HANDLE_W * 0.76, HANDLE_H * 0.60, centered=(False, True, False))
        .translate((0.010, 0.0, 0.006))
    )
    handle = handle.cut(finger_recess)

    button_pocket = (
        cq.Workplane("XY")
        .box(0.030, 0.040, 0.006, centered=(False, True, False))
        .translate((0.014, 0.0, HANDLE_H - 0.004))
    )
    handle = handle.cut(button_pocket)

    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_window_air_fryer")

    body_dark = model.material("body_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    body_trim = model.material("body_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    basket_dark = model.material("basket_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.17, 0.25, 0.30, 0.38))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    wall_t = 0.006
    base_t = 0.014
    body.visual(
        Box((BODY_D, wall_t, BODY_H)),
        origin=Origin(xyz=(0.0, -(BODY_W * 0.5 - wall_t * 0.5), BODY_H * 0.5)),
        material=body_dark,
        name="body_left_wall",
    )
    body.visual(
        Box((BODY_D, wall_t, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_W * 0.5 - wall_t * 0.5, BODY_H * 0.5)),
        material=body_dark,
        name="body_right_wall",
    )
    body.visual(
        Box((wall_t, BODY_W - 2.0 * wall_t, BODY_H)),
        origin=Origin(xyz=(-BODY_D * 0.5 + wall_t * 0.5, 0.0, BODY_H * 0.5)),
        material=body_dark,
        name="body_back_wall",
    )
    body.visual(
        Box((BODY_D - wall_t, BODY_W - 2.0 * wall_t, wall_t)),
        origin=Origin(xyz=(wall_t * 0.5, 0.0, BODY_H - wall_t * 0.5)),
        material=body_dark,
        name="body_top_shell",
    )
    body.visual(
        Box((BODY_D - wall_t, BODY_W - 2.0 * wall_t, base_t)),
        origin=Origin(xyz=(wall_t * 0.5, 0.0, base_t * 0.5)),
        material=body_dark,
        name="body_base",
    )
    body.visual(
        Box((BODY_FRONT_FRAME_T, BODY_W, BODY_H - (OPENING_Z + OPENING_H))),
        origin=Origin(
            xyz=(
                BODY_D * 0.5 - BODY_FRONT_FRAME_T * 0.5,
                0.0,
                OPENING_Z + OPENING_H + (BODY_H - (OPENING_Z + OPENING_H)) * 0.5,
            )
        ),
        material=body_dark,
        name="body_upper_front",
    )
    body.visual(
        Box((BODY_FRONT_FRAME_T, BODY_W, OPENING_Z)),
        origin=Origin(xyz=(BODY_D * 0.5 - BODY_FRONT_FRAME_T * 0.5, 0.0, OPENING_Z * 0.5)),
        material=body_dark,
        name="body_lower_sill",
    )
    body.visual(
        Box((0.012, 0.196, 0.096)),
        origin=Origin(xyz=(BODY_D * 0.5 - 0.006, 0.0, 0.278)),
        material=body_trim,
        name="control_panel",
    )
    body.visual(
        Box((0.004, 0.064, 0.036)),
        origin=Origin(xyz=(BODY_D * 0.5 + 0.002, 0.0, 0.283)),
        material=glass,
        name="display_glass",
    )
    for foot_x in (-0.105, 0.105):
        for foot_y in (-0.090, 0.090):
            body.visual(
                Box((0.026, 0.026, 0.010)),
                origin=Origin(xyz=(foot_x, foot_y, 0.005)),
                material=handle_dark,
                name=f"foot_{int((foot_x > 0))}_{int((foot_y > 0))}",
            )
    body.inertial = Inertial.from_geometry(
        Box((BODY_D, BODY_W, BODY_H)),
        mass=7.6,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_D, DRAWER_W, DRAWER_WALL)),
        origin=Origin(xyz=(DRAWER_D * 0.5, 0.0, DRAWER_WALL * 0.5)),
        material=basket_dark,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((DRAWER_D, DRAWER_WALL, DRAWER_H)),
        origin=Origin(
            xyz=(DRAWER_D * 0.5, -(DRAWER_W * 0.5 - DRAWER_WALL * 0.5), DRAWER_H * 0.5)
        ),
        material=basket_dark,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((DRAWER_D, DRAWER_WALL, DRAWER_H)),
        origin=Origin(
            xyz=(DRAWER_D * 0.5, DRAWER_W * 0.5 - DRAWER_WALL * 0.5, DRAWER_H * 0.5)
        ),
        material=basket_dark,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_W - 2.0 * DRAWER_WALL, DRAWER_H)),
        origin=Origin(xyz=(DRAWER_WALL * 0.5, 0.0, DRAWER_H * 0.5)),
        material=basket_dark,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((DRAWER_FRONT_T, DRAWER_FRONT_W, 0.056)),
        origin=Origin(xyz=(DRAWER_D + DRAWER_FRONT_T * 0.5, 0.0, 0.028)),
        material=basket_dark,
        name="drawer_front_bottom",
    )
    drawer.visual(
        Box((DRAWER_FRONT_T, DRAWER_FRONT_W, 0.030)),
        origin=Origin(
            xyz=(
                DRAWER_D + DRAWER_FRONT_T * 0.5,
                0.0,
                0.145,
            )
        ),
        material=basket_dark,
        name="drawer_front_top",
    )
    side_rail_w = (DRAWER_FRONT_W - 0.150) * 0.5
    drawer.visual(
        Box((DRAWER_FRONT_T, side_rail_w, 0.074)),
        origin=Origin(xyz=(DRAWER_D + DRAWER_FRONT_T * 0.5, -(0.150 * 0.5 + side_rail_w * 0.5), 0.093)),
        material=basket_dark,
        name="drawer_front_left_rail",
    )
    drawer.visual(
        Box((DRAWER_FRONT_T, side_rail_w, 0.074)),
        origin=Origin(xyz=(DRAWER_D + DRAWER_FRONT_T * 0.5, 0.150 * 0.5 + side_rail_w * 0.5, 0.093)),
        material=basket_dark,
        name="drawer_front_right_rail",
    )
    drawer.visual(
        Box((0.004, 0.168, 0.086)),
        origin=Origin(xyz=(DRAWER_D + 0.006, 0.0, 0.093)),
        material=glass,
        name="window_glass",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_D + DRAWER_FRONT_T, DRAWER_FRONT_W, DRAWER_FRONT_H)),
        mass=1.2,
        origin=Origin(xyz=((DRAWER_D + DRAWER_FRONT_T) * 0.5, 0.0, DRAWER_FRONT_H * 0.5)),
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_ORIGIN_X, 0.0, DRAWER_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    basket = model.part("basket")
    basket.visual(
        Box((BASKET_D, BASKET_W, BASKET_WALL)),
        origin=Origin(xyz=(BASKET_D * 0.5, 0.0, BASKET_WALL * 0.5)),
        material=steel,
        name="basket_bottom",
    )
    basket.visual(
        Box((BASKET_D, BASKET_WALL, BASKET_H)),
        origin=Origin(
            xyz=(BASKET_D * 0.5, -(BASKET_W * 0.5 - BASKET_WALL * 0.5), BASKET_H * 0.5)
        ),
        material=steel,
        name="basket_left_wall",
    )
    basket.visual(
        Box((BASKET_D, BASKET_WALL, BASKET_H)),
        origin=Origin(
            xyz=(BASKET_D * 0.5, BASKET_W * 0.5 - BASKET_WALL * 0.5, BASKET_H * 0.5)
        ),
        material=steel,
        name="basket_right_wall",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_W - 2.0 * BASKET_WALL, BASKET_H)),
        origin=Origin(xyz=(BASKET_WALL * 0.5, 0.0, BASKET_H * 0.5)),
        material=steel,
        name="basket_back_wall",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_W - 2.0 * BASKET_WALL, BASKET_H * 0.78)),
        origin=Origin(xyz=(BASKET_D - BASKET_WALL * 0.5, 0.0, BASKET_H * 0.39)),
        material=steel,
        name="basket_front_wall",
    )
    for foot_x in (0.030, BASKET_D - 0.030):
        for foot_y in (-0.060, 0.060):
            basket.visual(
                Box((0.020, 0.020, 0.006)),
                origin=Origin(xyz=(foot_x, foot_y, -0.001)),
                material=steel,
                name=f"basket_foot_{int(foot_x > BASKET_D * 0.5)}_{int(foot_y > 0)}",
            )
    basket.inertial = Inertial.from_geometry(
        Box((BASKET_D, BASKET_W, BASKET_H)),
        mass=0.55,
        origin=Origin(xyz=(BASKET_D * 0.5, 0.0, BASKET_H * 0.5)),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(0.020, 0.0, 0.007)),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.026, 0.020, 0.028)),
        origin=Origin(xyz=(0.013, -0.052, 0.014)),
        material=handle_dark,
        name="handle_left_post",
    )
    handle.visual(
        Box((0.026, 0.020, 0.028)),
        origin=Origin(xyz=(0.013, 0.052, 0.014)),
        material=handle_dark,
        name="handle_right_post",
    )
    handle.visual(
        Box((0.042, 0.128, 0.016)),
        origin=Origin(xyz=(0.026, 0.0, 0.024)),
        material=handle_dark,
        name="handle_grip",
    )
    handle.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(0.014, -0.022, 0.037)),
        material=handle_dark,
        name="handle_left_shoulder",
    )
    handle.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(0.014, 0.022, 0.037)),
        material=handle_dark,
        name="handle_right_shoulder",
    )
    handle.visual(
        Box((0.012, 0.052, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.037)),
        material=handle_dark,
        name="handle_rear_shoulder",
    )
    handle.inertial = Inertial.from_geometry(
        Box((HANDLE_D, HANDLE_W, HANDLE_H)),
        mass=0.18,
        origin=Origin(xyz=(HANDLE_D * 0.5, 0.0, HANDLE_H * 0.5)),
    )
    model.articulation(
        "drawer_to_handle",
        ArticulationType.FIXED,
        parent=drawer,
        child=handle,
        origin=Origin(xyz=(DRAWER_D + DRAWER_FRONT_T, 0.0, 0.056)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.024, 0.036, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_trim,
        name="release_button",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.024, 0.036, 0.008)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "handle_to_release_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=release_button,
        origin=Origin(xyz=(0.018, 0.0, 0.033)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.05,
            lower=0.0,
            upper=0.005,
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.026,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.056, 0.005, flare=0.08),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", angle_deg=0.0, depth=0.0008),
            ),
            "air_fryer_selector_knob",
        ),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_dark,
        name="knob_shell",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.030, 0.056, 0.056)),
        mass=0.08,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(BODY_D * 0.5, 0.0, 0.236)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    button_positions = [
        (-0.056, 0.302),
        (0.056, 0.302),
        (-0.056, 0.252),
        (0.056, 0.252),
    ]
    for index, (button_y, button_z) in enumerate(button_positions):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.008, 0.028, 0.016)),
            origin=Origin(xyz=(0.004, 0.0, 0.0)),
            material=body_trim,
            name="button_cap",
        )
        button.visual(
            Box((0.004, 0.018, 0.010)),
            origin=Origin(xyz=(0.002, 0.0, 0.0)),
            material=handle_dark,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.012, 0.028, 0.016)),
            mass=0.015,
            origin=Origin(xyz=(0.006, 0.0, 0.0)),
        )
        model.articulation(
            f"body_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BODY_D * 0.5, button_y, button_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.05,
                lower=-0.003,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    handle = object_model.get_part("handle")
    release_button = object_model.get_part("release_button")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    release = object_model.get_articulation("handle_to_release_button")
    selector_knob = object_model.get_articulation("body_to_selector_knob")

    ctx.allow_overlap(
        handle,
        release_button,
        reason="The safety release button is intentionally represented as nested into the handle grip in its rest position.",
    )

    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        margin=0.012,
        name="basket stays nested inside drawer footprint",
    )
    ctx.expect_overlap(
        drawer,
        "body",
        axes="yz",
        min_overlap=0.12,
        name="drawer covers lower front opening",
    )

    closed_drawer_x = ctx.part_world_position(drawer)
    open_drawer_x = None
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        open_drawer_x = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        closed_drawer_x is not None
        and open_drawer_x is not None
        and open_drawer_x[0] > closed_drawer_x[0] + 0.16,
        details=f"closed={closed_drawer_x}, open={open_drawer_x}",
    )

    closed_button_z = ctx.part_world_position(release_button)
    pressed_button_z = None
    with ctx.pose({release: 0.005}):
        pressed_button_z = ctx.part_world_position(release_button)

    ctx.check(
        "release button depresses downward",
        closed_button_z is not None
        and pressed_button_z is not None
        and pressed_button_z[2] < closed_button_z[2] - 0.003,
        details=f"closed={closed_button_z}, pressed={pressed_button_z}",
    )
    ctx.expect_gap(
        handle,
        drawer,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        name="handle mounts flush to drawer front",
    )
    ctx.check(
        "selector knob uses continuous rotation",
        selector_knob.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_knob.articulation_type}",
    )

    for index in range(4):
        button_joint = object_model.get_articulation(f"body_to_program_button_{index}")
        button_part = object_model.get_part(f"program_button_{index}")
        rest_pos = ctx.part_world_position(button_part)
        pressed_pos = None
        lower = button_joint.motion_limits.lower if button_joint.motion_limits is not None else None
        if lower is not None:
            with ctx.pose({button_joint: lower}):
                pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"program button {index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] < rest_pos[0] - 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
