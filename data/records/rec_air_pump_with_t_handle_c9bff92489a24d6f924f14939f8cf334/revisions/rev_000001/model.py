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


GUIDE_LENGTH = 0.180
GUIDE_DEPTH = 0.056
GUIDE_HEIGHT = 0.038
GUIDE_CENTER_Z = 0.019
GUIDE_BORE_RADIUS = 0.013

SUPPORT_WIDTH = 0.070
SUPPORT_DEPTH = 0.040
SUPPORT_HEIGHT = 0.100
SUPPORT_CENTER_Z = 0.087

BARREL_BASE_Z = 0.136
BARREL_LENGTH = 0.340
BARREL_OUTER_RADIUS = 0.027
BARREL_INNER_RADIUS = 0.022
LOWER_COLLAR_START_Z = 0.124
LOWER_COLLAR_LENGTH = 0.022
TOP_COLLAR_LENGTH = 0.028
TOP_COLLAR_OUTER_RADIUS = 0.031
ROD_CLEAR_RADIUS = 0.0105
HANDLE_JOINT_Z = BARREL_BASE_Z + BARREL_LENGTH + TOP_COLLAR_LENGTH

GAUGE_STEM_LENGTH = 0.018
GAUGE_BODY_DEPTH = 0.022
GAUGE_BEZEL_DEPTH = 0.007
GAUGE_RADIUS = 0.042
GAUGE_BEZEL_RADIUS = 0.045
GAUGE_ATTACH_Z = 0.068
BUTTON_X = 0.020
BUTTON_Z = 0.018
BUTTON_COLLAR_RADIUS = 0.009
BUTTON_HOLE_RADIUS = 0.0058
BUTTON_POCKET_DEPTH = 0.014
GAUGE_FRONT_Y = GAUGE_STEM_LENGTH + GAUGE_BODY_DEPTH + GAUGE_BEZEL_DEPTH


def _guide_housing_shape():
    housing = cq.Workplane("XY").box(GUIDE_LENGTH, GUIDE_DEPTH, GUIDE_HEIGHT).translate(
        (0.0, 0.0, GUIDE_CENTER_Z)
    )
    housing = housing.edges("|X").fillet(0.006)
    bore = (
        cq.Workplane("YZ")
        .circle(GUIDE_BORE_RADIUS)
        .extrude(GUIDE_LENGTH + 0.030)
        .translate((-(GUIDE_LENGTH + 0.030) / 2.0, 0.0, GUIDE_CENTER_Z))
    )
    return housing.cut(bore)


def _barrel_shell_shape():
    shell = (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_RADIUS)
        .circle(BARREL_INNER_RADIUS)
        .extrude(BARREL_LENGTH)
        .translate((0.0, 0.0, BARREL_BASE_Z))
    )
    lower_collar = (
        cq.Workplane("XY")
        .circle(TOP_COLLAR_OUTER_RADIUS)
        .circle(BARREL_INNER_RADIUS)
        .extrude(LOWER_COLLAR_LENGTH)
        .translate((0.0, 0.0, LOWER_COLLAR_START_Z))
    )
    top_collar = (
        cq.Workplane("XY")
        .circle(TOP_COLLAR_OUTER_RADIUS)
        .circle(ROD_CLEAR_RADIUS)
        .extrude(TOP_COLLAR_LENGTH)
        .translate((0.0, 0.0, BARREL_BASE_Z + BARREL_LENGTH))
    )
    return shell.union(lower_collar).union(top_collar)


def _gauge_shape():
    stem = (
        cq.Workplane("XY")
        .box(0.020, 0.060, GAUGE_STEM_LENGTH)
        .translate((0.0, 0.0, GAUGE_STEM_LENGTH / 2.0))
    )
    body = (
        cq.Workplane("XY")
        .circle(GAUGE_RADIUS)
        .extrude(GAUGE_BODY_DEPTH)
        .translate((0.0, 0.0, GAUGE_STEM_LENGTH))
    )
    bezel = (
        cq.Workplane("XY")
        .circle(GAUGE_BEZEL_RADIUS)
        .extrude(GAUGE_BEZEL_DEPTH)
        .translate((0.0, 0.0, GAUGE_STEM_LENGTH + GAUGE_BODY_DEPTH))
    )
    button_collar = (
        cq.Workplane("XY")
        .center(BUTTON_X, BUTTON_Z)
        .circle(BUTTON_COLLAR_RADIUS)
        .extrude(0.010)
        .translate((0.0, 0.0, GAUGE_FRONT_Y - 0.010))
    )
    button_pocket = (
        cq.Workplane("XY")
        .center(BUTTON_X, BUTTON_Z)
        .circle(BUTTON_HOLE_RADIUS)
        .extrude(BUTTON_POCKET_DEPTH)
        .translate((0.0, 0.0, GAUGE_FRONT_Y - BUTTON_POCKET_DEPTH))
    )
    return (
        stem.union(body)
        .union(bezel)
        .union(button_collar)
        .cut(button_pocket)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_floor_pump")

    powder_black = model.material("powder_black", rgba=(0.17, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.27, 0.28, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.85, 0.88, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    signal_red = model.material("signal_red", rgba=(0.72, 0.09, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_guide_housing_shape(), "pump_guide_housing"),
        material=graphite,
        name="guide_housing",
    )
    body.visual(
        Box((SUPPORT_WIDTH, SUPPORT_DEPTH, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_CENTER_Z)),
        material=powder_black,
        name="support_spine",
    )
    body.visual(
        Box((0.024, 0.009, 0.060)),
        origin=Origin(xyz=(0.0, 0.020, GAUGE_ATTACH_Z)),
        material=powder_black,
        name="gauge_mount",
    )
    body.visual(
        mesh_from_cadquery(_barrel_shell_shape(), "pump_barrel_shell"),
        material=powder_black,
        name="barrel_shell",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0085, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="rod",
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.208), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="rod_collar",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.095),
        origin=Origin(xyz=(-0.074, 0.0, 0.208), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.095),
        origin=Origin(xyz=(0.074, 0.0, 0.208), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    stabilizer_bar = model.part("stabilizer_bar")
    stabilizer_bar.visual(
        Cylinder(radius=0.011, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="stabilizer_bar",
    )
    stabilizer_bar.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="foot_0",
    )
    stabilizer_bar.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="foot_1",
    )
    stabilizer_bar.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="stop_collar",
    )

    gauge = model.part("gauge")
    gauge.visual(
        mesh_from_cadquery(_gauge_shape(), "pump_gauge"),
        material=powder_black,
        name="gauge_body",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.0048, length=0.008),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="button_stem",
    )
    release_button.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=signal_red,
        name="release_button",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.80,
            lower=-0.140,
            upper=0.060,
        ),
    )
    model.articulation(
        "body_to_stabilizer_bar",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stabilizer_bar,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.25,
            lower=0.0,
            upper=0.100,
        ),
    )
    model.articulation(
        "body_to_gauge",
        ArticulationType.FIXED,
        parent=body,
        child=gauge,
        origin=Origin(xyz=(0.0, GUIDE_DEPTH / 2.0 - 0.0035, GAUGE_ATTACH_Z)),
    )
    model.articulation(
        "gauge_to_release_button",
        ArticulationType.PRISMATIC,
        parent=gauge,
        child=release_button,
        origin=Origin(xyz=(BUTTON_X, GAUGE_FRONT_Y, BUTTON_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=-0.004,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    stabilizer_bar = object_model.get_part("stabilizer_bar")
    gauge = object_model.get_part("gauge")
    release_button = object_model.get_part("release_button")

    handle_slide = object_model.get_articulation("body_to_handle")
    bar_slide = object_model.get_articulation("body_to_stabilizer_bar")
    button_slide = object_model.get_articulation("gauge_to_release_button")

    ctx.allow_overlap(
        gauge,
        release_button,
        elem_a="gauge_body",
        elem_b="button_stem",
        reason="The pressure-release button uses a captured plunger stem that is intentionally represented inside the gauge housing.",
    )

    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="rod",
        outer_elem="barrel_shell",
        margin=0.001,
        name="piston rod stays centered in the cylinder",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="rod",
        elem_b="barrel_shell",
        min_overlap=0.160,
        name="resting piston rod remains inserted in the cylinder",
    )

    handle_rest = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: handle_slide.motion_limits.upper}):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="rod",
            outer_elem="barrel_shell",
            margin=0.001,
            name="raised piston rod stays centered in the cylinder",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="rod",
            elem_b="barrel_shell",
            min_overlap=0.110,
            name="raised piston rod keeps retained insertion",
        )
        handle_up = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: handle_slide.motion_limits.lower}):
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="rod",
            elem_b="barrel_shell",
            min_overlap=0.300,
            name="compressed piston rod still overlaps the cylinder shell",
        )
        handle_down = ctx.part_world_position(handle)

    ctx.check(
        "handle travels along the cylinder axis",
        handle_rest is not None
        and handle_up is not None
        and handle_down is not None
        and handle_up[2] > handle_rest[2] + 0.04
        and handle_down[2] < handle_rest[2] - 0.10,
        details=f"rest={handle_rest}, up={handle_up}, down={handle_down}",
    )

    ctx.expect_within(
        stabilizer_bar,
        body,
        axes="yz",
        inner_elem="stabilizer_bar",
        outer_elem="guide_housing",
        margin=0.0015,
        name="stabilizer bar stays nested in the lower guide",
    )
    ctx.expect_overlap(
        stabilizer_bar,
        body,
        axes="x",
        elem_a="stabilizer_bar",
        elem_b="guide_housing",
        min_overlap=0.180,
        name="centered stabilizer bar remains captured by the guide",
    )

    bar_rest = ctx.part_world_position(stabilizer_bar)
    with ctx.pose({bar_slide: bar_slide.motion_limits.upper}):
        ctx.expect_within(
            stabilizer_bar,
            body,
            axes="yz",
            inner_elem="stabilizer_bar",
            outer_elem="guide_housing",
            margin=0.0015,
            name="extended stabilizer bar stays centered in the guide",
        )
        ctx.expect_overlap(
            stabilizer_bar,
            body,
            axes="x",
            elem_a="stabilizer_bar",
            elem_b="guide_housing",
            min_overlap=0.150,
            name="extended stabilizer bar keeps retained insertion",
        )
        bar_extended = ctx.part_world_position(stabilizer_bar)

    ctx.check(
        "stabilizer bar slides across the base",
        bar_rest is not None and bar_extended is not None and bar_extended[0] > bar_rest[0] + 0.08,
        details=f"rest={bar_rest}, extended={bar_extended}",
    )

    ctx.expect_gap(
        release_button,
        gauge,
        axis="y",
        positive_elem="release_button",
        negative_elem="gauge_body",
        min_gap=0.0008,
        max_gap=0.0015,
        name="release button sits visibly proud of the gauge body",
    )

    button_rest = ctx.part_world_position(release_button)
    with ctx.pose({button_slide: button_slide.motion_limits.lower}):
        button_pressed = ctx.part_world_position(release_button)

    ctx.check(
        "release button presses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] < button_rest[1] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
