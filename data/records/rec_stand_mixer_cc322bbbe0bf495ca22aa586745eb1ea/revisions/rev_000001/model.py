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


HEAD_TILT_MAX = math.radians(58.0)
HUB_TWIST_MAX = math.radians(40.0)


def _build_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.228, 0.164, 0.022, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
    )
    rear_cheek_0 = (
        cq.Workplane("XY")
        .transformed(offset=(-0.074, -0.040, 0.022))
        .box(0.076, 0.036, 0.106, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    rear_cheek_1 = (
        cq.Workplane("XY")
        .transformed(offset=(-0.074, 0.040, 0.022))
        .box(0.076, 0.036, 0.106, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    bowl_pad = (
        cq.Workplane("XY")
        .transformed(offset=(0.044, 0.0, 0.022))
        .circle(0.052)
        .extrude(0.006)
    )
    ear_0 = cq.Workplane("XY").transformed(offset=(-0.074, -0.040, 0.138)).box(0.022, 0.014, 0.036)
    ear_1 = cq.Workplane("XY").transformed(offset=(-0.074, 0.040, 0.138)).box(0.022, 0.014, 0.036)
    return plate.union(rear_cheek_0).union(rear_cheek_1).union(bowl_pad).union(ear_0).union(ear_1)


def _build_head_shape() -> cq.Workplane:
    hinge_barrel = (
        cq.Workplane("XZ")
        .circle(0.011)
        .extrude(0.048)
        .translate((0.0, 0.024, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0.022, 0.0, 0.024))
        .box(0.028, 0.038, 0.042)
        .edges("|Z")
        .fillet(0.006)
    )
    main_body = (
        cq.Workplane("XY")
        .transformed(offset=(0.090, 0.0, 0.050))
        .box(0.132, 0.098, 0.082)
        .edges("|Z")
        .fillet(0.014)
    )
    nose = (
        cq.Workplane("XY")
        .transformed(offset=(0.151, 0.0, 0.034))
        .box(0.054, 0.070, 0.060)
        .edges("|Z")
        .fillet(0.010)
    )
    drive_collar = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.026)
        .translate((0.112, 0.0, -0.018))
    )
    hub_receiver = (
        cq.Workplane("YZ")
        .circle(0.019)
        .extrude(0.028)
        .translate((0.150, 0.0, 0.032))
    )
    return neck.union(hinge_barrel).union(main_body).union(nose).union(drive_collar).union(hub_receiver)


def _build_hub_cap_shape() -> cq.Workplane:
    cap_body = cq.Workplane("YZ").circle(0.023).extrude(0.012)
    grip_tab = cq.Workplane("XY").transformed(offset=(0.007, 0.016, 0.018)).box(0.010, 0.014, 0.006)
    return cap_body.union(grip_tab)


def _build_beater_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.0055).extrude(0.022).translate((0.0, 0.0, -0.022))
    collar = cq.Workplane("XY").circle(0.0085).extrude(0.010).translate((0.0, 0.0, -0.032))
    spine = cq.Workplane("XY").transformed(offset=(0.0, 0.0, -0.050)).box(0.010, 0.006, 0.036)
    lower_blade = cq.Workplane("XY").transformed(offset=(0.0, 0.0, -0.070)).box(0.058, 0.006, 0.008)
    right_leg = cq.Workplane("XY").transformed(offset=(0.024, 0.0, -0.058)).box(0.008, 0.006, 0.028)
    left_leg = cq.Workplane("XY").transformed(offset=(-0.022, 0.0, -0.055)).box(0.008, 0.006, 0.022)
    scraper = cq.Workplane("XY").transformed(offset=(0.015, 0.0, -0.043)).box(0.018, 0.006, 0.008)
    return shaft.union(collar).union(spine).union(lower_blade).union(right_leg).union(left_leg).union(scraper)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.92, 0.92, 0.88, 1.0))
    trim = model.material("trim", rgba=(0.22, 0.22, 0.24, 1.0))
    beater_finish = model.material("beater_finish", rgba=(0.72, 0.74, 0.76, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base"),
        material=body_finish,
        name="base_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shape(), "head"),
        material=body_finish,
        name="head_shell",
    )

    hub_cap = model.part("hub_cap")
    hub_cap.visual(
        mesh_from_cadquery(_build_hub_cap_shape(), "hub_cap"),
        material=trim,
        name="hub_cap_shell",
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_cadquery(_build_beater_shape(), "beater"),
        material=beater_finish,
        name="beater_shell",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.074, 0.0, 0.138)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=HEAD_TILT_MAX),
    )

    model.articulation(
        "head_to_hub_cap",
        ArticulationType.REVOLUTE,
        parent=head,
        child=hub_cap,
        origin=Origin(xyz=(0.178, 0.0, 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=HUB_TWIST_MAX),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.112, 0.0, -0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beater = object_model.get_part("beater")
    head = object_model.get_part("head")
    hub_cap = object_model.get_part("hub_cap")
    head_tilt = object_model.get_articulation("base_to_head")
    hub_twist = object_model.get_articulation("head_to_hub_cap")
    beater_spin = object_model.get_articulation("head_to_beater")

    ctx.allow_overlap(
        base,
        head,
        reason="The rear tilt-head hinge is intentionally represented as a nested barrel inside the base yoke.",
    )

    ctx.expect_origin_gap(
        beater,
        base,
        axis="x",
        min_gap=0.030,
        max_gap=0.060,
        name="beater hangs ahead of the low base pedestal",
    )

    rest_pos = ctx.part_world_position(beater)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        raised_pos = ctx.part_world_position(beater)
    ctx.check(
        "tilt head opens upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({hub_twist: 0.0}):
        closed_cap = ctx.part_world_aabb(hub_cap)
    with ctx.pose({hub_twist: hub_twist.motion_limits.upper}):
        twisted_cap = ctx.part_world_aabb(hub_cap)

    hub_visible = False
    hub_details = f"closed={closed_cap}, twisted={twisted_cap}"
    if closed_cap is not None and twisted_cap is not None:
        closed_z_max = closed_cap[1][2]
        twisted_z_max = twisted_cap[1][2]
        closed_y_min = closed_cap[0][1]
        twisted_y_min = twisted_cap[0][1]
        hub_visible = twisted_z_max > closed_z_max + 0.007 and twisted_y_min < closed_y_min - 0.007
    ctx.check("hub cap shows a short bayonet twist", hub_visible, details=hub_details)

    with ctx.pose({beater_spin: 0.0}):
        spin_0 = ctx.part_world_aabb(beater)
    with ctx.pose({beater_spin: math.pi / 2.0}):
        spin_90 = ctx.part_world_aabb(beater)

    beater_spins = False
    spin_details = f"spin_0={spin_0}, spin_90={spin_90}"
    if spin_0 is not None and spin_90 is not None:
        span_x_0 = spin_0[1][0] - spin_0[0][0]
        span_y_0 = spin_0[1][1] - spin_0[0][1]
        span_x_90 = spin_90[1][0] - spin_90[0][0]
        span_y_90 = spin_90[1][1] - spin_90[0][1]
        beater_spins = span_x_0 > span_y_0 + 0.030 and span_y_90 > span_x_90 + 0.030
        spin_details = (
            f"spin_0={spin_0}, spin_90={spin_90}, "
            f"spans_0=({span_x_0:.4f}, {span_y_0:.4f}), "
            f"spans_90=({span_x_90:.4f}, {span_y_90:.4f})"
        )
    ctx.check("beater spins on the vertical drive axis", beater_spins, details=spin_details)

    return ctx.report()


object_model = build_object_model()
