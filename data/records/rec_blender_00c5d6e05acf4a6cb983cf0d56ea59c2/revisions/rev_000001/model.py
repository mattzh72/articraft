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


BASE_HEIGHT = 0.145
JUG_HEIGHT = 0.220
JUG_BODY_OFFSET = 0.020
FLAP_OPEN_ANGLE = math.radians(115.0)
LEVER_SWING_ANGLE = math.radians(34.0)


def _build_base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .ellipse(0.105, 0.090)
        .workplane(offset=0.028)
        .ellipse(0.100, 0.085)
        .workplane(offset=0.075)
        .ellipse(0.079, 0.074)
        .workplane(offset=0.042)
        .ellipse(0.074, 0.069)
        .loft(combine=True)
    )

    body = body.faces(">Z").workplane().circle(0.046).cutBlind(-0.006)

    lever_pad = (
        cq.Workplane("XY")
        .box(0.062, 0.018, 0.050)
        .translate((0.054, 0.086, 0.050))
    )

    return body.union(lever_pad)


def _build_jug_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .rect(0.094, 0.092)
        .workplane(offset=0.070)
        .rect(0.112, 0.108)
        .workplane(offset=0.085)
        .rect(0.132, 0.124)
        .workplane(offset=0.065)
        .rect(0.142, 0.128)
        .loft(combine=True)
    )
    shell = shell.faces(">Z").shell(-0.004)

    spout_lip = (
        cq.Workplane("XY")
        .box(0.020, 0.046, 0.008)
        .translate((0.073, 0.0, 0.216))
    )

    return shell.union(spout_lip)


def _build_jug_handle() -> cq.Workplane:
    handle = (
        cq.Workplane("XY")
        .box(0.038, 0.024, 0.020)
        .translate((-0.079, 0.0, 0.173))
        .union(
            cq.Workplane("XY")
            .box(0.050, 0.024, 0.018)
            .translate((-0.071, 0.0, 0.062))
        )
        .union(
            cq.Workplane("XY")
            .box(0.020, 0.026, 0.122)
            .translate((-0.100, 0.0, 0.116))
        )
    )
    return handle


def _build_jug_collar() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.100, 0.096, 0.032)
        .translate((0.0, 0.0, 0.016))
        .cut(cq.Workplane("XY").circle(0.030).extrude(0.040))
    )


def _build_lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(0.144, 0.130, 0.012)
        .translate((0.0, 0.0, 0.006))
    )

    plug = (
        cq.Workplane("XY")
        .box(0.088, 0.078, 0.002)
        .translate((0.0, 0.0, -0.001))
    )

    rear_bridge = cq.Workplane("XY").box(0.018, 0.070, 0.010).translate((0.011, 0.0, 0.015))
    center_dome = cq.Workplane("XY").box(0.050, 0.060, 0.008).translate((-0.014, 0.0, 0.014))

    opening = cq.Workplane("XY").box(0.056, 0.046, 0.030).translate((0.037, 0.0, 0.006))

    return lid.union(plug).union(rear_bridge).union(center_dome).cut(opening)


def _build_flap_shape() -> cq.Workplane:
    knuckle = cq.Workplane("XZ").circle(0.0035).extrude(0.044, both=True)
    panel = (
        cq.Workplane("XY")
        .box(0.048, 0.050, 0.006)
        .translate((0.024, 0.0, -0.003))
    )
    finger_lip = cq.Workplane("XY").box(0.010, 0.032, 0.010).translate((0.047, 0.0, 0.004))
    return knuckle.union(panel).union(finger_lip)


def _build_blade_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.004))
        .union(cq.Workplane("XY").circle(0.008).extrude(0.010).translate((0.0, 0.0, 0.004)))
    )

    blade_specs = (
        (35.0, 12.0, 0.003),
        (215.0, 12.0, 0.003),
        (125.0, -12.0, 0.007),
        (305.0, -12.0, 0.007),
    )
    for yaw_deg, pitch_deg, z_pos in blade_specs:
        wing = cq.Workplane("XY").box(0.042, 0.010, 0.002).translate((0.022, 0.0, z_pos))
        wing = wing.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), pitch_deg)
        wing = wing.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw_deg)
        blade = blade.union(wing)

    return blade


def _build_pulse_lever_shape() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(0.0045).extrude(0.034, both=True)
    paddle = cq.Workplane("XY").box(0.030, 0.010, 0.068).translate((0.0, 0.008, 0.034))
    finger_tab = cq.Workplane("XY").box(0.040, 0.014, 0.012).translate((0.0, 0.010, 0.067))
    return barrel.union(paddle).union(finger_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_blender")

    base_body = model.material("base_body", rgba=(0.22, 0.24, 0.27, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    jug_clear = model.material("jug_clear", rgba=(0.82, 0.90, 0.95, 0.38))
    blade_metal = model.material("blade_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    lid_black = model.material("lid_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_shell"),
        material=base_body,
        name="base_shell",
    )
    jug = model.part("jug")
    jug.visual(
        mesh_from_cadquery(_build_jug_shell(), "jar_shell"),
        origin=Origin(xyz=(0.0, 0.0, JUG_BODY_OFFSET)),
        material=jug_clear,
        name="jar_shell",
    )
    jug.visual(
        mesh_from_cadquery(_build_jug_handle(), "jug_handle"),
        origin=Origin(xyz=(0.0, 0.0, JUG_BODY_OFFSET)),
        material=trim_black,
        name="jug_handle",
    )
    jug.visual(
        mesh_from_cadquery(_build_jug_collar(), "jug_collar"),
        material=trim_black,
        name="jug_collar",
    )
    jug.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=trim_black,
        name="blade_bearing",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "lid_shell"),
        material=lid_black,
        name="lid_shell",
    )

    pour_flap = model.part("pour_flap")
    pour_flap.visual(
        mesh_from_cadquery(_build_flap_shape(), "flap_panel"),
        material=lid_black,
        name="flap_panel",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_build_blade_shape(), "blade_cross"),
        material=blade_metal,
        name="blade_cross",
    )

    pulse_lever = model.part("pulse_lever")
    pulse_lever.visual(
        mesh_from_cadquery(_build_pulse_lever_shape(), "lever_paddle"),
        material=trim_black,
        name="lever_paddle",
    )

    model.articulation(
        "base_to_jug",
        ArticulationType.FIXED,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )
    model.articulation(
        "jug_to_lid",
        ArticulationType.FIXED,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, JUG_HEIGHT + JUG_BODY_OFFSET)),
    )
    model.articulation(
        "lid_to_pour_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=pour_flap,
        origin=Origin(xyz=(0.014, 0.0, 0.023)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=FLAP_OPEN_ANGLE,
        ),
    )
    model.articulation(
        "jug_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "base_to_pulse_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pulse_lever,
        origin=Origin(xyz=(0.054, 0.099, 0.050)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=LEVER_SWING_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jug = object_model.get_part("jug")
    lid = object_model.get_part("lid")
    pour_flap = object_model.get_part("pour_flap")
    blade = object_model.get_part("blade")
    pulse_lever = object_model.get_part("pulse_lever")

    flap_hinge = object_model.get_articulation("lid_to_pour_flap")
    lever_hinge = object_model.get_articulation("base_to_pulse_lever")

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_overlap(
            pour_flap,
            lid,
            axes="xy",
            elem_a="flap_panel",
            elem_b="lid_shell",
            min_overlap=0.026,
            name="flap covers the pour opening region",
        )

    ctx.expect_gap(
        lid,
        blade,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="blade_cross",
        min_gap=0.140,
        name="blade sits well below the lid",
    )
    ctx.expect_within(
        blade,
        jug,
        axes="xy",
        inner_elem="blade_cross",
        outer_elem="jar_shell",
        margin=0.010,
        name="blade stays within the jug footprint",
    )
    with ctx.pose({flap_hinge: 0.0}):
        flap_closed = ctx.part_element_world_aabb(pour_flap, elem="flap_panel")
    with ctx.pose({flap_hinge: FLAP_OPEN_ANGLE}):
        flap_open = ctx.part_element_world_aabb(pour_flap, elem="flap_panel")

    flap_lifts = (
        flap_closed is not None
        and flap_open is not None
        and flap_open[1][2] > flap_closed[1][2] + 0.020
        and flap_open[1][0] < flap_closed[1][0] - 0.010
    )
    ctx.check(
        "pour flap opens upward from the spout",
        flap_lifts,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    with ctx.pose({lever_hinge: 0.0}):
        lever_closed = ctx.part_element_world_aabb(pulse_lever, elem="lever_paddle")
    with ctx.pose({lever_hinge: LEVER_SWING_ANGLE}):
        lever_open = ctx.part_element_world_aabb(pulse_lever, elem="lever_paddle")

    lever_swings_out = (
        lever_closed is not None
        and lever_open is not None
        and lever_open[1][1] > lever_closed[1][1] + 0.018
        and lever_open[0][2] < lever_closed[0][2] - 0.005
    )
    ctx.check(
        "pulse lever swings outward from the base side",
        lever_swings_out,
        details=f"closed={lever_closed}, open={lever_open}",
    )

    return ctx.report()


object_model = build_object_model()
