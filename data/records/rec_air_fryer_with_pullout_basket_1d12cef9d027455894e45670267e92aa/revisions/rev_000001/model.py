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
)


BODY_WIDTH = 0.340
BODY_DEPTH = 0.392
BODY_HEIGHT = 0.360

DRAWER_OPENING_WIDTH = 0.292
DRAWER_OPENING_HEIGHT = 0.160
DRAWER_SEAT_X = 0.150
DRAWER_CENTER_Z = 0.126
DRAWER_TRAVEL = 0.170

PANEL_LOWER_X = 0.182
PANEL_LOWER_Z = 0.278
PANEL_UPSLOPE_RUN = 0.106
PANEL_UPSLOPE_RISE = 0.082
PANEL_PITCH = math.atan2(PANEL_UPSLOPE_RISE, PANEL_UPSLOPE_RUN)
PANEL_TAN_X = -math.cos(PANEL_PITCH)
PANEL_TAN_Z = math.sin(PANEL_PITCH)
PANEL_NORM_X = math.sin(PANEL_PITCH)
PANEL_NORM_Z = math.cos(PANEL_PITCH)


def _panel_point(distance_up: float, lateral: float, normal_offset: float = 0.0) -> tuple[float, float, float]:
    return (
        PANEL_LOWER_X + PANEL_TAN_X * distance_up + PANEL_NORM_X * normal_offset,
        lateral,
        PANEL_LOWER_Z + PANEL_TAN_Z * distance_up + PANEL_NORM_Z * normal_offset,
    )


def _panel_origin(distance_up: float, lateral: float, normal_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=_panel_point(distance_up, lateral, normal_offset),
        rpy=(0.0, PANEL_PITCH, 0.0),
    )


def _panel_transform(shape: cq.Workplane, *, distance_up: float, lateral: float, normal_offset: float = 0.0):
    return (
        shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(PANEL_PITCH))
        .translate(_panel_point(distance_up, lateral, normal_offset))
    )


def _make_body_housing() -> cq.Workplane:
    outer_profile = [
        (-0.196, 0.000),
        (0.196, 0.000),
        (0.196, 0.274),
        (0.078, 0.360),
        (-0.182, 0.360),
        (-0.196, 0.338),
    ]
    housing = cq.Workplane("XZ").polyline(outer_profile).close().extrude(BODY_WIDTH / 2.0, both=True)

    chamber_profile = [
        (-0.156, 0.052),
        (0.146, 0.052),
        (0.146, 0.232),
        (0.028, 0.278),
        (-0.150, 0.278),
    ]
    chamber = cq.Workplane("XZ").polyline(chamber_profile).close().extrude(0.302 / 2.0, both=True)
    housing = housing.cut(chamber.val())

    front_opening = (
        cq.Workplane("YZ")
        .transformed(offset=(0.195, 0.0, DRAWER_CENTER_Z))
        .box(0.065, DRAWER_OPENING_WIDTH, DRAWER_OPENING_HEIGHT, centered=(False, True, True))
    )
    housing = housing.cut(front_opening.val())

    chamber_lip_relief = (
        cq.Workplane("YZ")
        .transformed(offset=(0.172, 0.0, DRAWER_CENTER_Z))
        .box(0.016, 0.302, 0.170, centered=(False, True, True))
    )
    housing = housing.cut(chamber_lip_relief.val())

    knob_clearance = _panel_transform(
        cq.Workplane("XY").circle(0.0135).extrude(0.020).translate((0.0, 0.0, -0.016)),
        distance_up=0.074,
        lateral=0.0,
    )
    housing = housing.cut(knob_clearance.val())

    for lateral in (-0.105, -0.035, 0.035, 0.105):
        button_pocket = _panel_transform(
            cq.Workplane("XY")
            .box(0.028, 0.058, 0.012, centered=(True, True, False))
            .translate((0.0, 0.0, -0.010)),
            distance_up=0.032,
            lateral=lateral,
        )
        housing = housing.cut(button_pocket.val())

    top_exhaust = (
        cq.Workplane("XY")
        .box(0.090, 0.180, 0.020, centered=(True, True, False))
        .translate((-0.098, 0.0, BODY_HEIGHT - 0.012))
    )
    housing = housing.cut(top_exhaust.val())

    return housing


def _make_drawer_shell() -> cq.Workplane:
    drawer_length = 0.262
    drawer_width = 0.292
    drawer_height = 0.146
    wall = 0.005
    drawer_bottom = -0.074
    drawer_center_x = -0.126

    shell = (
        cq.Workplane("XY")
        .box(drawer_length, drawer_width, drawer_height, centered=(True, True, False))
        .translate((drawer_center_x, 0.0, drawer_bottom))
    )
    cavity = (
        cq.Workplane("XY")
        .box(drawer_length - 2.0 * wall, drawer_width - 2.0 * wall, drawer_height - wall, centered=(True, True, False))
        .translate((drawer_center_x - 0.003, 0.0, drawer_bottom + wall))
    )
    shell = shell.cut(cavity.val())

    rear_window = (
        cq.Workplane("XY")
        .box(0.040, drawer_width - 0.040, 0.070, centered=(True, True, False))
        .translate((-0.248, 0.0, -0.020))
    )
    shell = shell.cut(rear_window.val())

    fascia = (
        cq.Workplane("XY")
        .box(0.028, 0.312, 0.160, centered=(True, True, True))
        .translate((0.014, 0.0, 0.000))
    )

    handle = (
        cq.Workplane("XY")
        .box(0.058, 0.112, 0.030, centered=(True, True, False))
        .translate((0.044, 0.0, 0.006))
    )
    handle_grip = (
        cq.Workplane("XY")
        .box(0.038, 0.082, 0.015, centered=(True, True, False))
        .translate((0.052, 0.0, 0.006))
    )
    handle = handle.cut(handle_grip.val())

    latch_pocket = (
        cq.Workplane("XY")
        .box(0.022, 0.040, 0.014, centered=(True, True, False))
        .translate((0.050, 0.0, 0.022))
    )
    handle = handle.cut(latch_pocket.val())

    shell = shell.union(fascia.val()).union(handle.val())

    slot_length = 0.215
    slot_depth = 0.0032
    slot_height = 0.012
    slot_center_x = -0.140
    slot_base_z = 0.018
    half_width = drawer_width / 2.0
    for sign in (-1.0, 1.0):
        slot = (
            cq.Workplane("XY")
            .box(slot_length, slot_depth, slot_height, centered=(True, True, False))
            .translate((slot_center_x, sign * (half_width - slot_depth / 2.0), slot_base_z))
        )
        shell = shell.cut(slot.val())

    return shell


def _make_basket_shell() -> cq.Workplane:
    basket_length = 0.210
    basket_width = 0.248
    basket_height = 0.096
    wall = 0.003
    basket_bottom = -0.058
    basket_center_x = -0.148

    basket = (
        cq.Workplane("XY")
        .box(basket_length, basket_width, basket_height, centered=(True, True, False))
        .translate((basket_center_x, 0.0, basket_bottom))
    )
    inner = (
        cq.Workplane("XY")
        .box(basket_length - 2.0 * wall, basket_width - 2.0 * wall, basket_height - wall, centered=(True, True, False))
        .translate((basket_center_x, 0.0, basket_bottom + wall))
    )
    basket = basket.cut(inner.val())

    for foot_y in (-0.086, 0.086):
        foot = (
            cq.Workplane("XY")
            .box(0.132, 0.012, 0.011, centered=(True, True, False))
            .translate((basket_center_x - 0.010, foot_y, basket_bottom - 0.011))
        )
        basket = basket.union(foot.val())

    for slot_x in (-0.072, -0.028, 0.016):
        for slot_y in (-0.064, 0.0, 0.064):
            bottom_slot = (
                cq.Workplane("XY")
                .box(0.028, 0.018, 0.006, centered=(True, True, False))
                .translate((basket_center_x + slot_x, slot_y, basket_bottom - 0.001))
            )
            basket = basket.cut(bottom_slot.val())

    side_slot_centers = (-0.066, -0.016, 0.030)
    for sign in (-1.0, 1.0):
        for slot_x in side_slot_centers:
            side_slot = (
                cq.Workplane("XY")
                .box(0.032, 0.008, 0.026, centered=(True, True, False))
                .translate((basket_center_x + slot_x, sign * (basket_width / 2.0), basket_bottom + 0.024))
            )
            basket = basket.cut(side_slot.val())

    return basket


def _make_button(cap_x: float, cap_y: float, cap_height: float, stem_x: float, stem_y: float, stem_depth: float):
    cap = cq.Workplane("XY").box(cap_x, cap_y, cap_height, centered=(True, True, False))
    stem = (
        cq.Workplane("XY")
        .box(stem_x, stem_y, stem_depth, centered=(True, True, False))
        .translate((0.0, 0.0, -stem_depth))
    )
    return cap.union(stem.val())


def _make_selector_knob():
    stem = cq.Workplane("XY").circle(0.007).extrude(0.004).translate((0.0, 0.0, -0.004))
    skirt = cq.Workplane("XY").circle(0.025).extrude(0.004)
    body = cq.Workplane("XY").circle(0.021).extrude(0.016).translate((0.0, 0.0, 0.004))
    crown = cq.Workplane("XY").circle(0.018).extrude(0.008).translate((0.0, 0.0, 0.020))
    knob = stem.union(skirt.val()).union(body.val()).union(crown.val())
    indicator = (
        cq.Workplane("XY")
        .box(0.010, 0.0024, 0.0018, centered=(True, True, False))
        .translate((0.010, 0.0, 0.027))
    )
    return knob.union(indicator.val())


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_family_air_fryer")

    body_mat = model.material("body_matte_black", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_mat = model.material("trim_dark_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    drawer_mat = model.material("drawer_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    basket_mat = model.material("basket_dark", rgba=(0.24, 0.25, 0.26, 1.0))
    metal_mat = model.material("metal_grey", rgba=(0.58, 0.60, 0.62, 1.0))
    button_mat = model.material("button_black", rgba=(0.12, 0.13, 0.14, 1.0))
    latch_mat = model.material("latch_red", rgba=(0.79, 0.16, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.368, BODY_WIDTH, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=body_mat,
        name="base_shell",
    )
    body.visual(
        Box((0.014, 0.320, 0.348)),
        origin=Origin(xyz=(-0.189, 0.000, 0.174)),
        material=body_mat,
        name="rear_shell",
    )
    body.visual(
        Box((0.330, 0.012, 0.236)),
        origin=Origin(xyz=(-0.021, -0.164, 0.174)),
        material=body_mat,
        name="left_shell",
    )
    body.visual(
        Box((0.330, 0.012, 0.236)),
        origin=Origin(xyz=(-0.021, 0.164, 0.174)),
        material=body_mat,
        name="right_shell",
    )
    body.visual(
        Box((0.270, 0.320, 0.012)),
        origin=Origin(xyz=(-0.061, 0.000, 0.354)),
        material=body_mat,
        name="top_shell",
    )
    body.visual(
        Box((0.136, 0.320, 0.012)),
        origin=_panel_origin(0.068, 0.0),
        material=body_mat,
        name="sloped_panel",
    )
    body.visual(
        Box((0.020, 0.320, 0.050)),
        origin=Origin(xyz=(0.141, 0.000, 0.255)),
        material=body_mat,
        name="front_upper",
    )
    body.visual(
        Box((0.020, 0.274, 0.040)),
        origin=Origin(xyz=(0.140, 0.000, 0.020)),
        material=body_mat,
        name="lower_sill",
    )
    body.visual(
        Box((0.018, 0.012, 0.194)),
        origin=Origin(xyz=(0.141, -0.164, 0.137)),
        material=body_mat,
        name="left_frame",
    )
    body.visual(
        Box((0.018, 0.012, 0.194)),
        origin=Origin(xyz=(0.141, 0.164, 0.137)),
        material=body_mat,
        name="right_frame",
    )

    rail_length = 0.250
    rail_thickness = 0.014
    rail_height = 0.012
    rail_center_x = 0.004
    rail_center_z = 0.150
    rail_center_y = 0.151
    body.visual(
        Box((rail_length, rail_thickness, rail_height)),
        origin=Origin(xyz=(rail_center_x, -rail_center_y, rail_center_z)),
        material=metal_mat,
        name="left_guide",
    )
    body.visual(
        Box((rail_length, rail_thickness, rail_height)),
        origin=Origin(xyz=(rail_center_x, rail_center_y, rail_center_z)),
        material=metal_mat,
        name="right_guide",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_shell(), "air_fryer_drawer"),
        material=drawer_mat,
        name="drawer_shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_make_basket_shell(), "air_fryer_basket"),
        material=basket_mat,
        name="basket_shell",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_cadquery(_make_selector_knob(), "air_fryer_selector_knob"),
        material=trim_mat,
        name="knob",
    )

    preset_origins = (-0.105, -0.035, 0.035, 0.105)
    preset_buttons = []
    for index, lateral in enumerate(preset_origins):
        button = model.part(f"preset_button_{index}")
        button.visual(
            mesh_from_cadquery(
                _make_button(0.020, 0.050, 0.006, 0.010, 0.032, 0.003),
                f"air_fryer_preset_button_{index}",
            ),
            material=button_mat,
            name="button",
        )
        preset_buttons.append((button, lateral))

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(
            _make_button(0.022, 0.032, 0.006, 0.014, 0.020, 0.004),
            "air_fryer_latch_button",
        ),
        material=latch_mat,
        name="button",
    )

    drawer_joint = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_SEAT_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    basket_joint = model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(),
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=_panel_origin(0.074, 0.0, 0.010),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    for index, (button, lateral) in enumerate(preset_buttons):
        model.articulation(
            f"body_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=_panel_origin(0.032, lateral, 0.009),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=16.0,
                velocity=0.10,
                lower=0.0,
                upper=0.004,
            ),
        )

    model.articulation(
        "drawer_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=latch_button,
        origin=Origin(xyz=(0.050, 0.0, 0.036)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=0.005,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    latch_button = object_model.get_part("latch_button")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    latch_joint = object_model.get_articulation("drawer_to_latch_button")
    preset_joint = object_model.get_articulation("body_to_preset_button_0")

    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"limits={knob_joint.motion_limits}",
    )

    ctx.allow_overlap(
        "body",
        "drawer",
        elem_a="left_guide",
        elem_b="drawer_shell",
        reason="The drawer is intentionally represented as sliding on a retained left side guide nested into its groove.",
    )
    ctx.allow_overlap(
        "body",
        "drawer",
        elem_a="right_guide",
        elem_b="drawer_shell",
        reason="The drawer is intentionally represented as sliding on a retained right side guide nested into its groove.",
    )
    ctx.allow_overlap(
        "basket",
        "drawer",
        elem_a="basket_shell",
        elem_b="drawer_shell",
        reason="The removable fryer basket is intentionally modeled as a retained insert nested inside the outer drawer shell.",
    )
    ctx.allow_overlap(
        "body",
        "selector_knob",
        elem_a="sloped_panel",
        elem_b="knob",
        reason="The selector knob's hidden mount passes through the sloped control panel even though the panel opening is not explicitly modeled.",
    )
    for index in range(4):
        ctx.allow_overlap(
            "body",
            f"preset_button_{index}",
            elem_a="sloped_panel",
            elem_b="button",
            reason="Each preset button includes a hidden push stem passing through the sloped control panel without an explicitly modeled opening.",
        )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_shell",
            elem_b="left_guide",
            min_overlap=0.20,
            name="closed drawer stays deeply engaged on the left guide",
        )
        ctx.expect_within(
            basket,
            drawer,
            axes="yz",
            elem_a="basket_shell",
            elem_b="drawer_shell",
            margin=0.020,
            name="basket nests inside the drawer shell",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_shell",
            elem_b="left_guide",
            min_overlap=0.055,
            name="extended drawer retains insertion on the left guide",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_shell",
            elem_b="right_guide",
            min_overlap=0.055,
            name="extended drawer retains insertion on the right guide",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            elem_a="drawer_shell",
            margin=0.035,
            name="drawer stays centered in the chamber opening",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.15,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    preset_rest = ctx.part_world_position(object_model.get_part("preset_button_0"))
    with ctx.pose({preset_joint: 0.004}):
        preset_pressed = ctx.part_world_position(object_model.get_part("preset_button_0"))

    ctx.check(
        "preset button presses inward along the sloped panel",
        preset_rest is not None
        and preset_pressed is not None
        and preset_pressed[0] < preset_rest[0] - 0.0015
        and preset_pressed[2] < preset_rest[2] - 0.0020,
        details=f"rest={preset_rest}, pressed={preset_pressed}",
    )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_joint: 0.005}):
        latch_pressed = ctx.part_world_position(latch_button)

    ctx.check(
        "latch button presses downward into the handle",
        latch_rest is not None
        and latch_pressed is not None
        and latch_pressed[2] < latch_rest[2] - 0.004,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
