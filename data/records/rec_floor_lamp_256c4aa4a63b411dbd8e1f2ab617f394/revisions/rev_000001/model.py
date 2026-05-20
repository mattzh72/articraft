from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.145
BASE_THICKNESS = 0.020
COLLAR_RADIUS = 0.038
COLLAR_HEIGHT = 0.055
SLEEVE_OUTER_RADIUS = 0.022
SLEEVE_INNER_RADIUS = 0.0168
SLEEVE_HEIGHT = 0.560
SLEEVE_ENTRY_Z = BASE_THICKNESS - 0.001 + SLEEVE_HEIGHT

UPRIGHT_RADIUS = 0.014
UPRIGHT_INSERTION = 0.300
UPRIGHT_VISIBLE_HEIGHT = 0.470
UPRIGHT_LENGTH = UPRIGHT_INSERTION + UPRIGHT_VISIBLE_HEIGHT
UPRIGHT_CENTER_Z = (UPRIGHT_VISIBLE_HEIGHT - UPRIGHT_INSERTION) / 2.0
PIVOT_AXIS_Z = 0.492
LIFT_TRAVEL = 0.220

SHADE_BARREL_RADIUS = 0.0065
SHADE_BARREL_LENGTH = 0.022
SHADE_TOP_Z = 0.027
SLIDER_SLOT_CENTER_X = 0.250
SLIDER_TRAVEL = 0.015


def _build_lower_sleeve_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS - 0.001))
    )
    sleeve = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .extrude(SLEEVE_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS - 0.001))
    )
    bore = (
        cq.Workplane("XY")
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_HEIGHT + 0.020)
        .translate((0.0, 0.0, BASE_THICKNESS + 0.020))
    )
    return base.union(collar).union(sleeve).cut(bore)


def _build_upright_shape() -> cq.Workplane:
    mast = (
        cq.Workplane("XY")
        .circle(UPRIGHT_RADIUS)
        .extrude(UPRIGHT_LENGTH)
        .translate((0.0, 0.0, -UPRIGHT_INSERTION))
    )
    stop_collar = (
        cq.Workplane("XY")
        .circle(0.024)
        .extrude(0.006)
    )
    top_post = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(0.052)
        .translate((0.0, 0.0, PIVOT_AXIS_Z - 0.036))
    )
    pivot_head = (
        cq.Workplane("XY")
        .box(0.012, 0.018, 0.012, centered=(True, True, True))
        .translate((0.0, 0.0, PIVOT_AXIS_Z + 0.014))
    )
    return mast.union(stop_collar).union(top_post).union(pivot_head)


def _build_shade_bar_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(SHADE_BARREL_RADIUS).extrude(
        SHADE_BARREL_LENGTH / 2.0,
        both=True,
    )
    mount = (
        cq.Workplane("XY")
        .box(0.080, 0.016, 0.012, centered=(False, True, True))
        .translate((0.006, 0.0, 0.008))
    )
    head = (
        cq.Workplane("XY")
        .box(0.300, 0.058, 0.034, centered=(False, True, True))
        .translate((0.075, 0.0, 0.010))
    )
    light_recess = (
        cq.Workplane("XY")
        .box(0.220, 0.040, 0.010, centered=(True, True, False))
        .translate((0.240, 0.0, -0.007))
    )
    slider_slot = (
        cq.Workplane("XY")
        .box(0.055, 0.010, 0.004, centered=(True, True, False))
        .translate((SLIDER_SLOT_CENTER_X, 0.0, 0.023))
    )
    return barrel.union(mount).union(head).cut(light_recess).cut(slider_slot)


def _build_slider_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.028, 0.022, 0.008, centered=(True, True, False))
    thumb_ridge = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.003, centered=(True, True, False))
        .translate((0.0, 0.0, 0.008))
    )
    return cap.union(thumb_ridge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp")

    model.material("brushed_nickel", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("satin_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("shade_bronze", rgba=(0.27, 0.24, 0.21, 1.0))
    model.material("slider_black", rgba=(0.12, 0.12, 0.13, 1.0))

    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.visual(
        mesh_from_cadquery(_build_lower_sleeve_shape(), "lower_sleeve_shell"),
        material="brushed_nickel",
        name="lower_sleeve_shell",
    )

    upright = model.part("upright")
    upright.visual(
        mesh_from_cadquery(_build_upright_shape(), "upright_shell"),
        material="satin_steel",
        name="upright_shell",
    )

    shade_bar = model.part("shade_bar")
    shade_bar.visual(
        mesh_from_cadquery(_build_shade_bar_shape(), "shade_bar_shell"),
        material="shade_bronze",
        name="shade_bar_shell",
    )

    dimmer_slider = model.part("dimmer_slider")
    dimmer_slider.visual(
        mesh_from_cadquery(_build_slider_shape(), "dimmer_slider_shell"),
        material="slider_black",
        name="dimmer_slider_shell",
    )

    model.articulation(
        "sleeve_to_upright",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=upright,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_ENTRY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.22,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    model.articulation(
        "upright_to_shade_bar",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=shade_bar,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.35,
            upper=0.85,
        ),
    )

    model.articulation(
        "shade_bar_to_dimmer_slider",
        ArticulationType.PRISMATIC,
        parent=shade_bar,
        child=dimmer_slider,
        origin=Origin(xyz=(SLIDER_SLOT_CENTER_X, 0.0, SHADE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=0.05,
            lower=-SLIDER_TRAVEL,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_sleeve = object_model.get_part("lower_sleeve")
    upright = object_model.get_part("upright")
    shade_bar = object_model.get_part("shade_bar")
    dimmer_slider = object_model.get_part("dimmer_slider")

    lift_joint = object_model.get_articulation("sleeve_to_upright")
    pitch_joint = object_model.get_articulation("upright_to_shade_bar")
    slider_joint = object_model.get_articulation("shade_bar_to_dimmer_slider")

    ctx.allow_overlap(
        shade_bar,
        upright,
        elem_a="shade_bar_shell",
        elem_b="upright_shell",
        reason="The top pivot is represented as a compact mesh knuckle, and the compiled mesh overlap is confined to that intended hinge embedding.",
    )

    ctx.expect_origin_distance(
        lower_sleeve,
        upright,
        axes="xy",
        max_dist=0.001,
        name="upright stays centered in the base sleeve",
    )
    ctx.expect_overlap(
        upright,
        lower_sleeve,
        axes="z",
        min_overlap=0.28,
        name="upright remains deeply inserted at rest",
    )

    rest_upright_pos = ctx.part_world_position(upright)
    with ctx.pose({lift_joint: LIFT_TRAVEL}):
        ctx.expect_origin_distance(
            lower_sleeve,
            upright,
            axes="xy",
            max_dist=0.001,
            name="extended upright stays centered in the sleeve",
        )
        ctx.expect_overlap(
            upright,
            lower_sleeve,
            axes="z",
            min_overlap=0.075,
            name="upright retains insertion at full height",
        )
        extended_upright_pos = ctx.part_world_position(upright)

    ctx.check(
        "upright extends upward",
        rest_upright_pos is not None
        and extended_upright_pos is not None
        and extended_upright_pos[2] > rest_upright_pos[2] + 0.18,
        details=f"rest={rest_upright_pos}, extended={extended_upright_pos}",
    )

    with ctx.pose({lift_joint: 0.120, pitch_joint: 0.0}):
        rest_shade_aabb = ctx.part_world_aabb(shade_bar)
    with ctx.pose({lift_joint: 0.120, pitch_joint: 0.70}):
        raised_shade_aabb = ctx.part_world_aabb(shade_bar)

    ctx.check(
        "shade bar pitches upward",
        rest_shade_aabb is not None
        and raised_shade_aabb is not None
        and raised_shade_aabb[1][2] > rest_shade_aabb[1][2] + 0.11,
        details=f"rest={rest_shade_aabb}, raised={raised_shade_aabb}",
    )

    with ctx.pose({lift_joint: 0.120, pitch_joint: 0.0, slider_joint: -SLIDER_TRAVEL}):
        slider_low_pos = ctx.part_world_position(dimmer_slider)
    with ctx.pose({lift_joint: 0.120, pitch_joint: 0.0, slider_joint: SLIDER_TRAVEL}):
        slider_high_pos = ctx.part_world_position(dimmer_slider)

    slider_delta = None
    if slider_low_pos is not None and slider_high_pos is not None:
        slider_delta = (
            slider_high_pos[0] - slider_low_pos[0],
            slider_high_pos[1] - slider_low_pos[1],
            slider_high_pos[2] - slider_low_pos[2],
        )

    ctx.check(
        "dimmer slider travels along the shade bar",
        slider_delta is not None
        and 0.024 <= slider_delta[0] <= 0.040
        and abs(slider_delta[1]) < 0.003
        and abs(slider_delta[2]) < 0.003,
        details=f"low={slider_low_pos}, high={slider_high_pos}, delta={slider_delta}",
    )

    return ctx.report()


object_model = build_object_model()
