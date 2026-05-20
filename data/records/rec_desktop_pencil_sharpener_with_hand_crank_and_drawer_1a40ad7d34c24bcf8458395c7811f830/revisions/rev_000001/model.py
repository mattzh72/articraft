from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.072
BODY_D = 0.088
BODY_H = 0.082
BODY_WALL = 0.0028

PORT_X = -0.011
PORT_Z = 0.051
PORT_OUTER_R = 0.0076
PORT_INNER_R = 0.0048

SLIDER_X = 0.021
SLIDER_BASE_Z = 0.044
SLIDER_TRAVEL = 0.010

TRAY_TRAVEL = 0.024

CRANK_AXLE_Y = -0.006
CRANK_AXLE_Z = 0.046


def _body_shape():
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(0.008)
    shell = outer

    tray_opening = (
        cq.Workplane("XZ", origin=(0.0, BODY_D / 2.0 + 0.001, 0.012))
        .rect(0.062, 0.022)
        .extrude(-0.056)
    )
    shell = shell.cut(tray_opening)

    shavings_chamber = (
        cq.Workplane("XZ", origin=(0.0, BODY_D / 2.0 + 0.001, 0.041))
        .rect(0.054, 0.044)
        .extrude(-0.036)
    )
    shell = shell.cut(shavings_chamber)

    guide_recess = (
        cq.Workplane("XZ", origin=(SLIDER_X, BODY_D / 2.0 + 0.0008, 0.053))
        .rect(0.010, 0.024)
        .extrude(-0.0014)
    )
    slider_slot = (
        cq.Workplane("XZ", origin=(SLIDER_X, BODY_D / 2.0 + 0.0012, 0.053))
        .rect(0.0042, 0.019)
        .extrude(-0.007)
    )
    shell = shell.cut(guide_recess).cut(slider_slot)

    port_sleeve = (
        cq.Workplane("XZ", origin=(PORT_X, BODY_D / 2.0 - 0.002, PORT_Z))
        .circle(PORT_OUTER_R)
        .extrude(0.008)
    )
    shell = shell.union(port_sleeve)

    port_bore = (
        cq.Workplane("XZ", origin=(PORT_X, BODY_D / 2.0 + 0.006, PORT_Z))
        .circle(PORT_INNER_R)
        .extrude(-0.078)
    )
    port_leadin = (
        cq.Workplane("XZ", origin=(PORT_X, BODY_D / 2.0 + 0.006, PORT_Z))
        .circle(PORT_INNER_R + 0.0011)
        .extrude(-0.0022)
    )
    shell = shell.cut(port_bore).cut(port_leadin)

    crank_collar = (
        cq.Workplane("YZ", origin=(BODY_W / 2.0 - 0.001, CRANK_AXLE_Y, CRANK_AXLE_Z))
        .circle(0.0105)
        .extrude(0.004)
    )
    shell = shell.union(crank_collar)

    return shell.val()


def _tray_shape():
    tray_w = 0.060
    tray_d = 0.050
    tray_h = 0.020
    wall = 0.0022
    lip_t = 0.004

    bin_outer = (
        cq.Workplane("XY")
        .box(tray_w, tray_d, tray_h, centered=(True, True, False))
        .translate((0.0, -(tray_d + lip_t) * 0.5, 0.0))
    )
    lip = cq.Workplane("XY").box(0.064, lip_t, 0.021, centered=(True, True, False))
    tray = bin_outer.union(lip)

    bin_inner = (
        cq.Workplane("XY")
        .box(tray_w - 2.0 * wall, 0.042, tray_h, centered=(True, True, False))
        .translate((0.0, -(tray_d + lip_t) * 0.5, wall))
    )
    tray = tray.cut(bin_inner)

    finger_pull = (
        cq.Workplane("YZ", origin=(-0.032, 0.0016, 0.012))
        .circle(0.007)
        .extrude(0.064)
    )
    tray = tray.cut(finger_pull)

    return tray.val()


def _slider_shape():
    knob = (
        cq.Workplane("XY")
        .box(0.011, 0.0065, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0036, 0.001))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.0032, 0.0032, 0.018, centered=(True, True, False))
        .translate((0.0, -0.0006, 0.0))
    )
    carrier = (
        cq.Workplane("XY")
        .box(0.008, 0.010, 0.010, centered=(True, True, False))
        .translate((0.0, -0.007, 0.004))
    )
    return knob.union(stem).union(carrier).val()


def _crank_shape():
    hub = cq.Workplane("YZ").circle(0.010).extrude(0.008)
    drop_arm = (
        cq.Workplane("XY")
        .box(0.004, 0.006, 0.026, centered=(True, True, False))
        .translate((0.006, 0.0, -0.026))
    )
    hand_arm = cq.Workplane("XY").box(0.004, 0.024, 0.006).translate((0.006, 0.011, -0.024))
    grip = (
        cq.Workplane("YZ", origin=(0.001, 0.022, -0.024))
        .circle(0.004)
        .extrude(0.018)
    )
    return hub.union(drop_arm).union(hand_arm).union(grip).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pencil_sharpener")

    shell_plastic = model.material("shell_plastic", rgba=(0.78, 0.80, 0.82, 1.0))
    crank_plastic = model.material("crank_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    slider_plastic = model.material("slider_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    tray_smoke = model.material("tray_smoke", rgba=(0.34, 0.37, 0.40, 0.88))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "sharpener_body"),
        material=shell_plastic,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "sharpener_tray"),
        material=tray_smoke,
        name="tray_shell",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.064, 0.054, 0.021)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.026, 0.0105)),
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        mesh_from_cadquery(_slider_shape(), "sharpener_mode_slider"),
        material=slider_plastic,
        name="slider_body",
    )
    mode_slider.inertial = Inertial.from_geometry(
        Box((0.011, 0.018, 0.018)),
        mass=0.01,
        origin=Origin(xyz=(0.0, -0.001, 0.009)),
    )

    crank = model.part("crank")
    crank.visual(
        mesh_from_cadquery(_crank_shape(), "sharpener_crank"),
        material=crank_plastic,
        name="crank_arm",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.020, 0.032, 0.038)),
        mass=0.04,
        origin=Origin(xyz=(0.010, 0.010, -0.016)),
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.002, 0.001)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_slider,
        origin=Origin(xyz=(SLIDER_X, BODY_D / 2.0, SLIDER_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.03,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_W / 2.0 + 0.003, CRANK_AXLE_Y, CRANK_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    mode_slider = object_model.get_part("mode_slider")
    crank = object_model.get_part("crank")

    tray_joint = object_model.get_articulation("body_to_tray")
    slider_joint = object_model.get_articulation("body_to_mode_slider")
    crank_joint = object_model.get_articulation("body_to_crank")

    ctx.allow_overlap(
        body,
        tray,
        elem_a="body_shell",
        elem_b="tray_shell",
        reason="The pullout tray is intentionally represented as sliding inside a simplified lower body cavity proxy.",
    )
    ctx.allow_overlap(
        body,
        mode_slider,
        elem_a="body_shell",
        elem_b="slider_body",
        reason="The short mode slider uses an internal carrier inside the simplified front guide slot proxy.",
    )

    ctx.expect_within(
        tray,
        body,
        axes="xz",
        margin=0.003,
        name="tray stays aligned with the lower body opening",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        min_overlap=0.030,
        name="tray remains inserted when closed",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        ctx.expect_within(
            tray,
            body,
            axes="xz",
            margin=0.003,
            name="extended tray stays aligned with the lower body opening",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.020,
            name="extended tray still retains insertion",
        )
        tray_extended = ctx.part_world_position(tray)

    ctx.check(
        "tray slides forward from the front opening",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] > tray_rest[1] + 0.020,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    slider_rest = ctx.part_world_position(mode_slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        slider_extended = ctx.part_world_position(mode_slider)

    ctx.check(
        "mode slider moves upward along its guide",
        slider_rest is not None
        and slider_extended is not None
        and slider_extended[2] > slider_rest[2] + 0.008,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    crank_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="crank_arm"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        crank_quarter_turn = _aabb_center(ctx.part_element_world_aabb(crank, elem="crank_arm"))

    ctx.check(
        "crank rotates around the side axle",
        crank_rest is not None
        and crank_quarter_turn is not None
        and abs(crank_quarter_turn[2] - crank_rest[2]) > 0.012,
        details=f"rest={crank_rest}, quarter_turn={crank_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
