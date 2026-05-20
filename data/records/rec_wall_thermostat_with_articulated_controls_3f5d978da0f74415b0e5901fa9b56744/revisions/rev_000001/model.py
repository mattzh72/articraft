from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.140
BODY_H = 0.095
BODY_D = 0.022
FRONT_Y = BODY_D / 2.0
DIAL_Z = 0.010
SLIDER_Z = -0.032


def _body_shell() -> cq.Workplane:
    """Shallow rounded thermostat housing with through guide openings."""
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).edges("|Y").fillet(0.010)

    button_cut_w = 0.016
    button_cut_h = 0.027
    for x in (-0.052, 0.052):
        cutter = cq.Workplane("XY").box(button_cut_w, BODY_D + 0.006, button_cut_h)
        body = body.cut(cutter.translate((x, 0.0, DIAL_Z)))

    slot_cutter = cq.Workplane("XY").box(0.066, BODY_D + 0.006, 0.011)
    body = body.cut(slot_cutter.translate((0.0, 0.0, SLIDER_Z)))
    return body


def _button_shape() -> cq.Workplane:
    """A proud rectangular cap that bears against the housing face."""
    return (
        cq.Workplane("XY")
        .box(0.020, 0.005, 0.031)
        .edges("|Y")
        .fillet(0.0035)
        .translate((0.0, 0.0025, 0.0))
    )


def _slider_shape() -> cq.Workplane:
    """Small lower fan-mode tab riding on the horizontal guide."""
    return (
        cq.Workplane("XY")
        .box(0.018, 0.0055, 0.013)
        .edges("|Y")
        .fillet(0.0022)
        .translate((0.0, 0.00275, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_dial_thermostat")

    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.08, 0.09, 0.09, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    blue_gray = model.material("blue_gray", rgba=(0.25, 0.42, 0.52, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "rounded_thermostat_body", tolerance=0.0006),
        material=warm_white,
        name="housing_shell",
    )

    # A dark backer behind the lower cutout makes the slider read as a recessed guide.
    body.visual(
        Box((0.074, 0.0010, 0.017)),
        origin=Origin(xyz=(0.0, -FRONT_Y + 0.00045, SLIDER_Z)),
        material=dark_gray,
        name="slider_shadow",
    )

    # Printed temperature tick marks around the central dial.
    for i, angle_deg in enumerate((-70, -45, -22, 0, 22, 45, 70)):
        angle = math.radians(angle_deg)
        radius = 0.037
        tick_len = 0.007 if angle_deg in (-70, 0, 70) else 0.005
        body.visual(
            Box((0.0014, 0.0005, tick_len)),
            origin=Origin(
                xyz=(radius * math.sin(angle), FRONT_Y + 0.00010, DIAL_Z + radius * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=graphite,
            name=f"temperature_tick_{i}",
        )

    # Fan-mode stop marks below the lower guide.
    for i, x in enumerate((-0.024, 0.0, 0.024)):
        body.visual(
            Box((0.0022, 0.0005, 0.006)),
            origin=Origin(xyz=(x, FRONT_Y + 0.00010, SLIDER_Z - 0.014)),
            material=graphite,
            name=f"fan_mark_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.055,
                0.012,
                body_style="domed",
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=36, depth=0.0007, width=0.0010),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "domed_temperature_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0030, 0.0010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0122, 0.010)),
        material=dark_gray,
        name="dial_pointer",
    )

    for i, x in enumerate((-0.052, 0.052)):
        button = model.part(f"side_button_{i}")
        button.visual(
            mesh_from_cadquery(_button_shape(), f"side_button_{i}", tolerance=0.0004),
            material=soft_gray,
            name="button_cap",
        )
        model.articulation(
            f"button_{i}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y, DIAL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=2.0, velocity=0.04),
        )

    fan_slider = model.part("fan_slider")
    fan_slider.visual(
        mesh_from_cadquery(_slider_shape(), "fan_slider_tab", tolerance=0.0004),
        material=blue_gray,
        name="slider_tab",
    )

    model.articulation(
        "dial_axis",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, FRONT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
    )

    model.articulation(
        "slider_guide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=fan_slider,
        origin=Origin(xyz=(0.0, FRONT_Y, SLIDER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.020, upper=0.020, effort=2.0, velocity=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("side_button_0")
    button_1 = object_model.get_part("side_button_1")
    slider = object_model.get_part("fan_slider")

    dial_axis = object_model.get_articulation("dial_axis")
    button_0_press = object_model.get_articulation("button_0_press")
    button_1_press = object_model.get_articulation("button_1_press")
    slider_guide = object_model.get_articulation("slider_guide")

    ctx.expect_gap(
        dial,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0005,
        name="dial seats on front face",
    )
    ctx.expect_overlap(dial, body, axes="xz", min_overlap=0.045, name="dial is centered on the body")

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_axis: math.pi * 1.25}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates about fixed center",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_rest[0] - dial_turned[0]) < 1e-6
        and abs(dial_rest[1] - dial_turned[1]) < 1e-6
        and abs(dial_rest[2] - dial_turned[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_0_press: 0.0025}):
        button_0_in = ctx.part_world_position(button_0)
        button_1_still = ctx.part_world_position(button_1)
    ctx.check(
        "button 0 depresses independently",
        button_0_rest is not None
        and button_0_in is not None
        and button_1_rest is not None
        and button_1_still is not None
        and button_0_in[1] < button_0_rest[1] - 0.002
        and abs(button_1_still[1] - button_1_rest[1]) < 1e-6,
        details=f"button0_rest={button_0_rest}, button0_in={button_0_in}, button1={button_1_still}",
    )
    with ctx.pose({button_1_press: 0.0025}):
        button_1_in = ctx.part_world_position(button_1)
    ctx.check(
        "button 1 depresses independently",
        button_1_rest is not None
        and button_1_in is not None
        and button_1_in[1] < button_1_rest[1] - 0.002,
        details=f"button1_rest={button_1_rest}, button1_in={button_1_in}",
    )

    slider_rest = ctx.part_world_position(slider)
    with ctx.pose({slider_guide: 0.020}):
        slider_right = ctx.part_world_position(slider)
    with ctx.pose({slider_guide: -0.020}):
        slider_left = ctx.part_world_position(slider)
    ctx.check(
        "fan slider follows the horizontal guide",
        slider_rest is not None
        and slider_left is not None
        and slider_right is not None
        and slider_left[0] < slider_rest[0] - 0.018
        and slider_right[0] > slider_rest[0] + 0.018
        and abs(slider_left[2] - slider_rest[2]) < 1e-6
        and abs(slider_right[2] - slider_rest[2]) < 1e-6,
        details=f"left={slider_left}, rest={slider_rest}, right={slider_right}",
    )

    return ctx.report()


object_model = build_object_model()
