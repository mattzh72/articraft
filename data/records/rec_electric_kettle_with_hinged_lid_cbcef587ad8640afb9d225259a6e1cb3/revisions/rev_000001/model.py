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


def _power_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.078).extrude(0.018)
    heater_plate = cq.Workplane("XY").circle(0.060).extrude(0.010).translate((0.0, 0.0, 0.018))
    connector = cq.Workplane("XY").circle(0.016).extrude(0.008).translate((0.0, 0.0, 0.028))
    return base.union(heater_plate).union(connector)


def _vessel_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .ellipse(0.042, 0.032)
        .workplane(offset=0.040)
        .ellipse(0.064, 0.048)
        .workplane(offset=0.055)
        .ellipse(0.071, 0.053)
        .workplane(offset=0.050)
        .ellipse(0.048, 0.037)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .ellipse(0.031, 0.023)
        .workplane(offset=0.036)
        .ellipse(0.055, 0.041)
        .workplane(offset=0.060)
        .ellipse(0.060, 0.045)
        .workplane(offset=0.049)
        .ellipse(0.039, 0.029)
        .loft(combine=True)
    )

    spout_outer = (
        cq.Workplane("XZ")
        .moveTo(0.038, 0.104)
        .lineTo(0.058, 0.112)
        .lineTo(0.085, 0.138)
        .lineTo(0.081, 0.150)
        .lineTo(0.062, 0.154)
        .lineTo(0.045, 0.141)
        .close()
        .extrude(0.036, both=True)
    )
    spout_inner = (
        cq.Workplane("XZ")
        .moveTo(0.030, 0.115)
        .lineTo(0.052, 0.122)
        .lineTo(0.075, 0.141)
        .lineTo(0.072, 0.147)
        .lineTo(0.054, 0.148)
        .lineTo(0.037, 0.136)
        .close()
        .extrude(0.018, both=True)
    )
    rear_lug_0 = cq.Workplane("XY").box(0.014, 0.010, 0.012).translate((-0.045, -0.017, 0.145))
    rear_lug_1 = cq.Workplane("XY").box(0.014, 0.010, 0.012).translate((-0.045, 0.017, 0.145))

    return outer.cut(inner).union(spout_outer).cut(spout_inner).union(rear_lug_0).union(rear_lug_1)


def _handle_shape() -> cq.Workplane:
    grip = cq.Workplane("XY").box(0.020, 0.030, 0.120).translate((-0.094, 0.0, 0.102))
    upper_bridge = cq.Workplane("XY").box(0.052, 0.032, 0.026).translate((-0.073, 0.0, 0.171))
    lower_mount = cq.Workplane("XY").box(0.044, 0.030, 0.028).translate((-0.071, 0.0, 0.047))
    mid_brace = cq.Workplane("XY").box(0.018, 0.028, 0.040).translate((-0.084, 0.0, 0.142))
    button_pocket = cq.Workplane("XY").box(0.024, 0.016, 0.010).translate((-0.078, 0.0, 0.182))
    return grip.union(upper_bridge).union(lower_mount).union(mid_brace).cut(button_pocket)


def _lid_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").center(0.039, 0.0).ellipse(0.045, 0.034).extrude(0.006)
    lip = cq.Workplane("XY").center(0.039, 0.0).ellipse(0.036, 0.027).extrude(-0.008)
    barrel = cq.Workplane("XZ").center(0.0, 0.003).circle(0.004).extrude(0.046, both=True)
    front_lift = cq.Workplane("XY").box(0.016, 0.020, 0.006).translate((0.073, 0.0, 0.004))
    return cap.union(lip).union(barrel).union(front_lift)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_electric_kettle")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    lid_trim = model.material("lid_trim", rgba=(0.84, 0.85, 0.86, 1.0))

    power_base = model.part("power_base")
    power_base.visual(
        mesh_from_cadquery(_power_base_shape(), "power_base"),
        material=base_black,
        name="base_shell",
    )
    power_base.inertial = Inertial.from_geometry(
        Box((0.156, 0.156, 0.036)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_vessel_shell_shape(), "kettle_vessel"),
        material=steel,
        name="vessel_shell",
    )
    body.visual(
        mesh_from_cadquery(_handle_shape(), "kettle_handle"),
        material=plastic_black,
        name="handle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.180, 0.110, 0.190)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "kettle_lid"),
        material=lid_trim,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.096, 0.072, 0.018)),
        mass=0.12,
        origin=Origin(xyz=(0.040, 0.0, 0.003)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.020, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=plastic_black,
        name="button_cap",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.006)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    lever_switch = model.part("lever_switch")
    lever_switch.visual(
        Cylinder(radius=0.003, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="switch_barrel",
    )
    lever_switch.visual(
        Box((0.016, 0.008, 0.024)),
        origin=Origin(xyz=(-0.005, 0.0, -0.012)),
        material=plastic_black,
        name="switch_paddle",
    )
    lever_switch.inertial = Inertial.from_geometry(
        Box((0.020, 0.010, 0.028)),
        mass=0.02,
        origin=Origin(xyz=(-0.002, 0.0, -0.008)),
    )

    keep_warm_dial = model.part("keep_warm_dial")
    keep_warm_dial.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=plastic_black,
        name="dial_body",
    )
    keep_warm_dial.visual(
        Box((0.010, 0.003, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.011)),
        material=lid_trim,
        name="dial_pointer",
    )
    keep_warm_dial.inertial = Inertial.from_geometry(
        Box((0.026, 0.026, 0.013)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
    )

    mode_button_0 = model.part("mode_button_0")
    mode_button_0.visual(
        Box((0.017, 0.011, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=lid_trim,
        name="button_cap",
    )
    mode_button_0.inertial = Inertial.from_geometry(
        Box((0.017, 0.011, 0.006)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    mode_button_1 = model.part("mode_button_1")
    mode_button_1.visual(
        Box((0.017, 0.011, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=lid_trim,
        name="button_cap",
    )
    mode_button_1.inertial = Inertial.from_geometry(
        Box((0.017, 0.011, 0.006)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "base_lift",
        ArticulationType.PRISMATIC,
        parent=power_base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=0.065,
        ),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.0415, 0.0, 0.158)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(-0.078, 0.0, 0.177)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "switch_toggle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever_switch,
        origin=Origin(xyz=(-0.076, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=power_base,
        child=keep_warm_dial,
        origin=Origin(xyz=(-0.024, 0.060, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
        ),
    )
    model.articulation(
        "mode_button_0_press",
        ArticulationType.PRISMATIC,
        parent=power_base,
        child=mode_button_0,
        origin=Origin(xyz=(0.012, 0.060, 0.0268)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0025,
        ),
    )
    model.articulation(
        "mode_button_1_press",
        ArticulationType.PRISMATIC,
        parent=power_base,
        child=mode_button_1,
        origin=Origin(xyz=(0.030, 0.060, 0.0268)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    power_base = object_model.get_part("power_base")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    release_button = object_model.get_part("release_button")
    lever_switch = object_model.get_part("lever_switch")
    keep_warm_dial = object_model.get_part("keep_warm_dial")
    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")
    base_lift = object_model.get_articulation("base_lift")
    lid_hinge = object_model.get_articulation("lid_hinge")
    button_press = object_model.get_articulation("button_press")
    switch_toggle = object_model.get_articulation("switch_toggle")
    dial_turn = object_model.get_articulation("dial_turn")
    mode_button_0_press = object_model.get_articulation("mode_button_0_press")
    mode_button_1_press = object_model.get_articulation("mode_button_1_press")

    ctx.allow_overlap(
        power_base,
        keep_warm_dial,
        elem_a="base_shell",
        elem_b="dial_body",
        reason="The keep-warm dial is intentionally represented as a shallow inset control seated into the base top.",
    )
    ctx.allow_overlap(
        power_base,
        mode_button_0,
        elem_a="base_shell",
        elem_b="button_cap",
        reason="The first mode button is intentionally modeled as a slightly inset push button on the base surface.",
    )
    ctx.allow_overlap(
        power_base,
        mode_button_1,
        elem_a="base_shell",
        elem_b="button_cap",
        reason="The second mode button is intentionally modeled as a slightly inset push button on the base surface.",
    )
    ctx.allow_overlap(
        body,
        release_button,
        elem_a="handle_shell",
        elem_b="button_cap",
        reason="The lid-release button sits inside a guided pocket at the top of the handle.",
    )
    ctx.allow_overlap(
        body,
        lever_switch,
        elem_a="handle_shell",
        elem_b="switch_paddle",
        reason="The lower handle switch is represented as a pivoting lever nested into the handle-root recess.",
    )

    ctx.expect_gap(
        body,
        power_base,
        axis="z",
        positive_elem="vessel_shell",
        negative_elem="base_shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="kettle rests on the heating base",
    )
    ctx.expect_overlap(
        body,
        power_base,
        axes="xy",
        elem_a="vessel_shell",
        elem_b="base_shell",
        min_overlap=0.070,
        name="kettle body sits over the circular base",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="vessel_shell",
        min_overlap=0.055,
        name="closed lid covers the opening footprint",
    )

    body_rest = ctx.part_world_position(body)
    with ctx.pose({base_lift: 0.050}):
        body_lifted = ctx.part_world_position(body)
    ctx.check(
        "kettle lifts vertically off the base",
        body_rest is not None and body_lifted is not None and body_lifted[2] > body_rest[2] + 0.040,
        details=f"rest={body_rest}, lifted={body_lifted}",
    )

    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: math.radians(85.0)}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the rear hinge",
        lid_closed is not None and lid_open is not None and lid_open[1][2] > lid_closed[1][2] + 0.030,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    button_rest = ctx.part_world_position(release_button)
    with ctx.pose({button_press: 0.004}):
        button_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button depresses into the handle top",
        button_rest is not None and button_pressed is not None and button_pressed[2] < button_rest[2] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    with ctx.pose({switch_toggle: -0.35}):
        switch_low = ctx.part_element_world_aabb(lever_switch, elem="switch_paddle")
    with ctx.pose({switch_toggle: 0.35}):
        switch_high = ctx.part_element_world_aabb(lever_switch, elem="switch_paddle")
    ctx.check(
        "lower handle lever visibly toggles",
        switch_low is not None
        and switch_high is not None
        and (
            abs(((switch_high[0][0] + switch_high[1][0]) * 0.5) - ((switch_low[0][0] + switch_low[1][0]) * 0.5)) > 0.004
            or abs(((switch_high[0][2] + switch_high[1][2]) * 0.5) - ((switch_low[0][2] + switch_low[1][2]) * 0.5)) > 0.004
        ),
        details=f"low={switch_low}, high={switch_high}",
    )

    dial_rest = ctx.part_element_world_aabb(keep_warm_dial, elem="dial_pointer")
    with ctx.pose({dial_turn: 1.2}):
        dial_rotated = ctx.part_element_world_aabb(keep_warm_dial, elem="dial_pointer")
    ctx.check(
        "keep warm dial pointer rotates around the dial",
        dial_rest is not None
        and dial_rotated is not None
        and abs(((dial_rotated[0][1] + dial_rotated[1][1]) * 0.5) - ((dial_rest[0][1] + dial_rest[1][1]) * 0.5)) > 0.004,
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    ctx.expect_origin_distance(
        mode_button_0,
        mode_button_1,
        axes="xy",
        min_dist=0.015,
        max_dist=0.024,
        name="mode buttons remain adjacent on the base",
    )

    mode_button_0_rest = ctx.part_world_position(mode_button_0)
    mode_button_1_rest = ctx.part_world_position(mode_button_1)
    with ctx.pose({mode_button_0_press: 0.0025, mode_button_1_press: 0.0025}):
        mode_button_0_down = ctx.part_world_position(mode_button_0)
        mode_button_1_down = ctx.part_world_position(mode_button_1)
    ctx.check(
        "mode buttons press downward",
        mode_button_0_rest is not None
        and mode_button_1_rest is not None
        and mode_button_0_down is not None
        and mode_button_1_down is not None
        and mode_button_0_down[2] < mode_button_0_rest[2] - 0.002
        and mode_button_1_down[2] < mode_button_1_rest[2] - 0.002,
        details=(
            f"button0_rest={mode_button_0_rest}, button0_down={mode_button_0_down}, "
            f"button1_rest={mode_button_1_rest}, button1_down={mode_button_1_down}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
