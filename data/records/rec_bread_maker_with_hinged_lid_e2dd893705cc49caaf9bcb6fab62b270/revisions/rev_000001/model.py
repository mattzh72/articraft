from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_W = 0.25
BODY_D = 0.34
BASE_H = 0.03
BODY_H = 0.335
WALL_T = 0.016
INNER_W = BODY_W - (2.0 * WALL_T)
WALL_H = BODY_H - BASE_H

RELEASE_SLOT_W = 0.060
RELEASE_SLOT_H = 0.066
RELEASE_SLOT_CENTER_Z = 0.158
FRONT_SEG_W = (INNER_W - RELEASE_SLOT_W) / 2.0
FRONT_LOWER_H = 0.095
FRONT_UPPER_H = WALL_H - FRONT_LOWER_H - RELEASE_SLOT_H
FRONT_Y = (BODY_D / 2.0) - (WALL_T / 2.0)

TOP_DECK_D = 0.125
TOP_DECK_Y = 0.0915
TOP_DECK_H = 0.020

LINER_W = 0.150
LINER_D = 0.210
LINER_T = 0.004
LINER_H = 0.275
LINER_CENTER_Y = -0.060
LINER_FLOOR_T = 0.006
LINER_FLOOR_Z = BASE_H + (LINER_FLOOR_T / 2.0)
LINER_WALL_Z = BASE_H + LINER_FLOOR_T + (LINER_H / 2.0)

SPINDLE_Z = BASE_H + LINER_FLOOR_T + 0.005
PADDLE_SEAT_Z = SPINDLE_Z + 0.005

POD_BASE_W = 0.190
POD_BASE_D = 0.100
POD_BASE_H = 0.035
POD_BASE_Y = 0.108
POD_CAP_W = 0.130
POD_CAP_D = 0.040
POD_CAP_H = 0.026
POD_CAP_Y = 0.100

BUTTON_SEAT_Z = BODY_H + POD_BASE_H
BUTTON_LAYOUT = (
    (-0.056, 0.072),
    (0.056, 0.072),
    (-0.056, 0.128),
    (0.056, 0.128),
)

KNOB_SEAT_Z = BODY_H + POD_BASE_H + POD_CAP_H
KNOB_CENTER_Y = POD_CAP_Y

LID_W = 0.196
LID_D = 0.214
LID_T = 0.014
LID_HINGE_Y = -0.164
LID_HINGE_Z = BODY_H


def _rounded_plate_mesh(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius),
            height,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_loaf_bread_maker")

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.92, 1.0))
    shell_shadow = model.material("shell_shadow", rgba=(0.84, 0.84, 0.83, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    window_smoke = model.material("window_smoke", rgba=(0.22, 0.28, 0.30, 0.45))
    steel = model.material("steel", rgba=(0.74, 0.75, 0.77, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.16, 0.16, 0.17, 1.0))

    pod_base_mesh = _rounded_plate_mesh(POD_BASE_W, POD_BASE_D, POD_BASE_H, 0.018, "pod_base")
    pod_cap_mesh = _rounded_plate_mesh(POD_CAP_W, POD_CAP_D, POD_CAP_H, 0.012, "pod_cap")
    lid_plate_mesh = _rounded_plate_mesh(LID_W, LID_D, LID_T, 0.020, "lid_plate")
    lid_window_mesh = _rounded_plate_mesh(0.118, 0.100, 0.004, 0.014, "lid_window")

    selector_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.052, 0.006, flare=0.08),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0007,
                angle_deg=0.0,
            ),
            center=False,
        ),
        "selector_knob",
    )

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=shell_white,
        name="base_shell",
    )
    body.visual(
        Box((WALL_T, BODY_D, WALL_H)),
        origin=Origin(xyz=(-(BODY_W / 2.0) + (WALL_T / 2.0), 0.0, BASE_H + (WALL_H / 2.0))),
        material=shell_white,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, WALL_H)),
        origin=Origin(xyz=((BODY_W / 2.0) - (WALL_T / 2.0), 0.0, BASE_H + (WALL_H / 2.0))),
        material=shell_white,
        name="right_wall",
    )
    body.visual(
        Box((INNER_W, WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, -FRONT_Y, BASE_H + (WALL_H / 2.0))),
        material=shell_white,
        name="back_wall",
    )
    body.visual(
        Box((FRONT_SEG_W, WALL_T, WALL_H)),
        origin=Origin(
            xyz=(
                -((RELEASE_SLOT_W / 2.0) + (FRONT_SEG_W / 2.0)),
                FRONT_Y,
                BASE_H + (WALL_H / 2.0),
            )
        ),
        material=shell_white,
        name="front_left",
    )
    body.visual(
        Box((FRONT_SEG_W, WALL_T, WALL_H)),
        origin=Origin(
            xyz=(
                (RELEASE_SLOT_W / 2.0) + (FRONT_SEG_W / 2.0),
                FRONT_Y,
                BASE_H + (WALL_H / 2.0),
            )
        ),
        material=shell_white,
        name="front_right",
    )
    body.visual(
        Box((RELEASE_SLOT_W, WALL_T, FRONT_LOWER_H)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_Y,
                BASE_H + (FRONT_LOWER_H / 2.0),
            )
        ),
        material=shell_white,
        name="front_lower",
    )
    body.visual(
        Box((RELEASE_SLOT_W, WALL_T, FRONT_UPPER_H)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_Y,
                BODY_H - (FRONT_UPPER_H / 2.0),
            )
        ),
        material=shell_white,
        name="front_upper",
    )
    body.visual(
        Box((INNER_W, TOP_DECK_D, TOP_DECK_H)),
        origin=Origin(xyz=(0.0, TOP_DECK_Y, BODY_H - (TOP_DECK_H / 2.0))),
        material=shell_shadow,
        name="top_deck",
    )
    body.visual(
        Box((LINER_W, LINER_D, LINER_FLOOR_T)),
        origin=Origin(xyz=(0.0, LINER_CENTER_Y, LINER_FLOOR_Z)),
        material=cavity_dark,
        name="liner_floor",
    )
    body.visual(
        Box((LINER_T, LINER_D, LINER_H)),
        origin=Origin(
            xyz=(
                -(LINER_W / 2.0) + (LINER_T / 2.0),
                LINER_CENTER_Y,
                LINER_WALL_Z,
            )
        ),
        material=cavity_dark,
        name="liner_left",
    )
    body.visual(
        Box((LINER_T, LINER_D, LINER_H)),
        origin=Origin(
            xyz=(
                (LINER_W / 2.0) - (LINER_T / 2.0),
                LINER_CENTER_Y,
                LINER_WALL_Z,
            )
        ),
        material=cavity_dark,
        name="liner_right",
    )
    body.visual(
        Box((LINER_W - (2.0 * LINER_T), LINER_T, LINER_H)),
        origin=Origin(
            xyz=(
                0.0,
                LINER_CENTER_Y - (LINER_D / 2.0) + (LINER_T / 2.0),
                LINER_WALL_Z,
            )
        ),
        material=cavity_dark,
        name="liner_back",
    )
    body.visual(
        Box((LINER_W - (2.0 * LINER_T), LINER_T, LINER_H)),
        origin=Origin(
            xyz=(
                0.0,
                LINER_CENTER_Y + (LINER_D / 2.0) - (LINER_T / 2.0),
                LINER_WALL_Z,
            )
        ),
        material=cavity_dark,
        name="liner_front",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, LINER_CENTER_Y, SPINDLE_Z)),
        material=steel,
        name="spindle_mount",
    )
    body.visual(
        pod_base_mesh,
        origin=Origin(xyz=(0.0, POD_BASE_Y, BODY_H)),
        material=panel_dark,
        name="pod_base",
    )
    body.visual(
        pod_cap_mesh,
        origin=Origin(xyz=(0.0, POD_CAP_Y, BODY_H + POD_BASE_H)),
        material=panel_dark,
        name="pod_cap",
    )

    lid = model.part("lid")
    lid.visual(
        lid_plate_mesh,
        origin=Origin(xyz=(0.0, LID_D / 2.0, 0.0)),
        material=shell_white,
        name="lid_plate",
    )
    lid.visual(
        lid_window_mesh,
        origin=Origin(xyz=(0.0, 0.115, 0.010)),
        material=window_smoke,
        name="lid_window",
    )
    lid.visual(
        Box((LID_W - 0.030, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, LID_D - 0.009, 0.020)),
        material=shell_shadow,
        name="front_lip",
    )
    lid.visual(
        Box((0.012, LID_D - 0.032, 0.018)),
        origin=Origin(xyz=(-(LID_W / 2.0) + 0.006, 0.107, -0.009)),
        material=shell_shadow,
        name="left_skirt",
    )
    lid.visual(
        Box((0.012, LID_D - 0.032, 0.018)),
        origin=Origin(xyz=((LID_W / 2.0) - 0.006, 0.107, -0.009)),
        material=shell_shadow,
        name="right_skirt",
    )
    lid.visual(
        Box((0.012, 0.012, 0.012)),
        origin=Origin(xyz=(-0.024, 0.185, 0.020)),
        material=control_black,
        name="handle_post_0",
    )
    lid.visual(
        Box((0.012, 0.012, 0.012)),
        origin=Origin(xyz=(0.024, 0.185, 0.020)),
        material=control_black,
        name="handle_post_1",
    )
    lid.visual(
        Box((0.082, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.185, 0.026)),
        material=control_black,
        name="handle_bar",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="shaft_collar",
    )
    selector_knob.visual(
        selector_knob_mesh,
        material=control_black,
        name="knob_shell",
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, KNOB_CENTER_Y, KNOB_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    for index, (x_pos, y_pos) in enumerate(BUTTON_LAYOUT):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.020, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=control_black,
            name="button_stem",
        )
        button.visual(
            Box((0.028, 0.020, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=rubber_dark,
            name="button_cap",
        )
        model.articulation(
            f"body_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, BUTTON_SEAT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.060, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=steel,
        name="plunger",
    )
    release_button.visual(
        Box((0.060, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=panel_dark,
        name="release_cap",
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, RELEASE_SLOT_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.010,
        ),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel,
        name="hub",
    )
    paddle.visual(
        Box((0.060, 0.022, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.012)),
        material=steel,
        name="main_blade",
    )
    paddle.visual(
        Box((0.022, 0.016, 0.014)),
        origin=Origin(xyz=(-0.013, 0.0, 0.011)),
        material=steel,
        name="rear_fin",
    )
    model.articulation(
        "body_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=paddle,
        origin=Origin(xyz=(0.0, LINER_CENTER_Y, PADDLE_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("selector_knob")
    release_button = object_model.get_part("release_button")
    paddle = object_model.get_part("paddle")

    lid_hinge = object_model.get_articulation("body_to_lid")
    knob_spin = object_model.get_articulation("body_to_selector_knob")
    release_slide = object_model.get_articulation("body_to_release_button")
    paddle_spin = object_model.get_articulation("body_to_paddle")

    preset_buttons = [object_model.get_part(f"preset_button_{index}") for index in range(4)]
    preset_joints = [object_model.get_articulation(f"body_to_preset_button_{index}") for index in range(4)]

    ctx.check(
        "lid_is_revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"type={lid_hinge.articulation_type!r}",
    )
    ctx.check(
        "selector_knob_is_continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"type={knob_spin.articulation_type!r}",
    )
    ctx.check(
        "paddle_is_continuous",
        paddle_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"type={paddle_spin.articulation_type!r}",
    )
    for index, joint in enumerate(preset_joints):
        ctx.check(
            f"preset_button_{index}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"type={joint.articulation_type!r}",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="top_deck",
            max_gap=0.002,
            max_penetration=0.0,
            name="lid closes flush onto top deck",
        )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_plate")
    with ctx.pose({lid_hinge: math.radians(112.0)}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_plate")
    ctx.check(
        "lid_opens_upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.10,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    release_rest = ctx.part_world_position(release_button)
    with ctx.pose({release_slide: 0.010}):
        release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release_button_moves_into_front_shell",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[1] < release_rest[1] - 0.008,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    ctx.expect_gap(
        paddle,
        body,
        axis="z",
        positive_elem="hub",
        negative_elem="spindle_mount",
        max_gap=0.0,
        max_penetration=0.0,
        name="paddle hub sits on spindle mount",
    )
    ctx.expect_within(
        paddle,
        body,
        axes="xy",
        inner_elem="main_blade",
        outer_elem="liner_floor",
        margin=0.0,
        name="paddle stays within loaf pan footprint",
    )
    with ctx.pose({paddle_spin: math.pi / 2.0}):
        ctx.expect_within(
            paddle,
            body,
            axes="xy",
            inner_elem="main_blade",
            outer_elem="liner_floor",
            margin=0.0,
            name="paddle remains within loaf pan footprint after rotation",
        )

    for index, button in enumerate(preset_buttons):
        ctx.expect_origin_distance(
            button,
            knob,
            axes="xy",
            min_dist=0.045,
            max_dist=0.085,
            name=f"preset_button_{index}_surrounds_selector_knob",
        )

    knob_pos = ctx.part_world_position(knob)
    button_positions = [ctx.part_world_position(button) for button in preset_buttons]
    if knob_pos is not None and all(position is not None for position in button_positions):
        xs = [position[0] for position in button_positions if position is not None]
        ys = [position[1] for position in button_positions if position is not None]
        ctx.check(
            "selector_knob_centered_in_button_cluster",
            min(xs) < knob_pos[0] < max(xs) and min(ys) < knob_pos[1] < max(ys),
            details=f"knob={knob_pos}, buttons={button_positions}",
        )
    else:
        ctx.fail(
            "selector_knob_centered_in_button_cluster",
            f"knob={knob_pos}, buttons={button_positions}",
        )

    return ctx.report()


object_model = build_object_model()
