from __future__ import annotations

from math import pi

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


BODY_THICKNESS = 0.038
JAW_THICKNESS = 0.032
HINGE_X = -0.006
HINGE_Z = 0.157
JAW_CENTER_X = 0.016
JAW_CENTER_Z = 0.135
JAW_OUTER_RADIUS = 0.034
JAW_INNER_RADIUS = 0.021


def rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    sx, sy, sz = size
    safe_radius = min(radius, sx * 0.48, sz * 0.48)
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Y").fillet(safe_radius)


def annulus_segment(
    *,
    center_x: float,
    center_z: float,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    clip_center_x: float,
    clip_center_z: float,
    clip_size_x: float,
    clip_size_z: float,
) -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(outer_radius)
        .extrude(thickness / 2.0, both=True)
        .cut(
            cq.Workplane("XZ")
            .center(center_x, center_z)
            .circle(inner_radius)
            .extrude(thickness, both=True)
        )
    )
    clip = cq.Workplane("XY").box(
        clip_size_x,
        thickness + 0.010,
        clip_size_z,
    ).translate((clip_center_x, 0.0, clip_center_z))
    return ring.intersect(clip)


def build_body_shape() -> cq.Workplane:
    grip = rounded_box((0.058, BODY_THICKNESS, 0.150), 0.018).translate((0.0, 0.0, -0.044))
    shoulder = rounded_box((0.072, BODY_THICKNESS, 0.050), 0.014).translate((0.0, 0.0, 0.014))
    head = rounded_box((0.092, BODY_THICKNESS, 0.084), 0.016).translate((0.0, 0.0, 0.073))
    hinge_neck = rounded_box((0.022, BODY_THICKNESS, 0.060), 0.007).translate((-0.004, 0.0, 0.128))
    lower_jaw_support = rounded_box((0.020, BODY_THICKNESS, 0.032), 0.006).translate((0.038, 0.0, 0.110))

    fixed_jaw = annulus_segment(
        center_x=JAW_CENTER_X,
        center_z=JAW_CENTER_Z,
        outer_radius=JAW_OUTER_RADIUS,
        inner_radius=JAW_INNER_RADIUS,
        thickness=BODY_THICKNESS,
        clip_center_x=0.030,
        clip_center_z=0.112,
        clip_size_x=0.056,
        clip_size_z=0.052,
    )
    fixed_tip = rounded_box((0.010, BODY_THICKNESS, 0.012), 0.003).translate((0.042, 0.0, 0.105))

    stand_axle = (
        cq.Workplane("YZ")
        .center(-BODY_THICKNESS / 2.0, -0.082)
        .circle(0.0022)
        .extrude(0.015, both=True)
    )

    body = grip.union(shoulder).union(head).union(hinge_neck).union(lower_jaw_support).union(fixed_jaw).union(fixed_tip)

    display_cut = cq.Workplane("XY").box(0.048, 0.0045, 0.034).translate((0.0, BODY_THICKNESS / 2.0 - 0.00225, 0.056))
    stand_pocket = cq.Workplane("XY").box(0.042, 0.0020, 0.084).translate((0.0, -BODY_THICKNESS / 2.0 + 0.0010, -0.034))
    hinge_clearance = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(0.0105)
        .extrude((BODY_THICKNESS + 0.004) / 2.0, both=True)
    )

    body = body.cut(display_cut).cut(stand_pocket).cut(hinge_clearance)
    body = body.union(stand_axle)
    return body


def build_jaw_shape() -> cq.Workplane:
    connector = rounded_box((0.012, JAW_THICKNESS, 0.010), 0.003).translate((0.006, 0.0, 0.004))
    top_arm = rounded_box((0.030, JAW_THICKNESS, 0.010), 0.003).translate((0.020, 0.0, 0.008))
    nose = annulus_segment(
        center_x=0.038,
        center_z=0.006,
        outer_radius=0.016,
        inner_radius=0.009,
        thickness=JAW_THICKNESS,
        clip_center_x=0.042,
        clip_center_z=0.004,
        clip_size_x=0.024,
        clip_size_z=0.026,
    )
    sleeve = (
        cq.Workplane("XZ")
        .circle(0.0056)
        .extrude(0.010, both=True)
        .cut(cq.Workplane("XZ").circle(0.0028).extrude(0.016, both=True))
    )
    jaw = sleeve.union(connector).union(top_arm).union(nose)
    return jaw


def build_dial_shape() -> cq.Workplane:
    skirt = cq.Workplane("XZ").circle(0.024).extrude(0.004)
    body = cq.Workplane("XZ").circle(0.0205).extrude(0.0085).translate((0.0, 0.004, 0.0))
    cap = cq.Workplane("XZ").circle(0.0175).extrude(0.0035).translate((0.0, 0.0125, 0.0))
    pointer = cq.Workplane("XY").box(0.004, 0.0016, 0.016).translate((0.0, 0.0153, 0.010))
    return skirt.union(body).union(cap).union(pointer)


def build_front_button_shape() -> cq.Workplane:
    base = rounded_box((0.013, 0.0032, 0.007), 0.0018).translate((0.0, 0.0016, 0.0))
    crown = rounded_box((0.0095, 0.0018, 0.0055), 0.0014).translate((0.0, 0.0031, 0.0))
    return base.union(crown)


def build_side_button_shape() -> cq.Workplane:
    body = rounded_box((0.0032, 0.014, 0.007), 0.0016).translate((0.0016, 0.0, 0.0))
    crown = rounded_box((0.0018, 0.010, 0.0055), 0.0012).translate((0.0031, 0.0, 0.0))
    return body.union(crown)


def build_stand_shape() -> cq.Workplane:
    width = 0.038
    thickness = 0.003
    height = 0.076

    panel = rounded_box((width, thickness, height), 0.004).translate((0.0, -thickness / 2.0, height / 2.0))
    cutout = cq.Workplane("XY").box(0.018, thickness * 2.0, 0.040).translate((0.0, -thickness / 2.0, 0.044))
    brace = rounded_box((width, thickness, 0.010), 0.002).translate((0.0, -thickness / 2.0, 0.005))

    left_sleeve = (
        cq.Workplane("YZ")
        .center(-thickness / 2.0, 0.0)
        .circle(0.0034)
        .extrude(0.0055, both=True)
        .translate((-0.011, 0.0, 0.0))
        .cut(
            cq.Workplane("YZ")
            .center(-thickness / 2.0, 0.0)
            .circle(0.00275)
            .extrude(0.0070, both=True)
            .translate((-0.011, 0.0, 0.0))
        )
    )
    right_sleeve = (
        cq.Workplane("YZ")
        .center(-thickness / 2.0, 0.0)
        .circle(0.0034)
        .extrude(0.0055, both=True)
        .translate((0.011, 0.0, 0.0))
        .cut(
            cq.Workplane("YZ")
            .center(-thickness / 2.0, 0.0)
            .circle(0.00275)
            .extrude(0.0070, both=True)
            .translate((0.011, 0.0, 0.0))
        )
    )

    return panel.cut(cutout).union(brace).union(left_sleeve).union(right_sleeve)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_clamp_meter")

    model.material("body", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("jaw", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("dial", rgba=(0.95, 0.43, 0.12, 1.0))
    model.material("button", rgba=(0.20, 0.21, 0.22, 1.0))
    model.material("flashlight_button", rgba=(0.96, 0.74, 0.18, 1.0))
    model.material("stand", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("screen", rgba=(0.10, 0.18, 0.16, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(build_body_shape(), "body"), material="body", name="housing")
    body.visual(
        Box((0.042, 0.0024, 0.028)),
        origin=Origin(xyz=(0.0, 0.0155, 0.056)),
        material="screen",
        name="screen",
    )

    jaw = model.part("jaw")
    jaw.visual(mesh_from_cadquery(build_jaw_shape(), "jaw"), material="jaw", name="jaw_shape")

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dial",
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.0205, length=0.009),
        origin=Origin(xyz=(0.0, 0.0075, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dial",
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.0175, length=0.006),
        origin=Origin(xyz=(0.0, 0.0125, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dial",
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.0018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0160, 0.010)),
        material="screen",
        name="dial_pointer",
    )

    stand = model.part("stand")
    stand.visual(mesh_from_cadquery(build_stand_shape(), "stand"), material="stand", name="stand_panel")

    flashlight_button = model.part("flashlight_button")
    flashlight_button.visual(
        mesh_from_cadquery(build_side_button_shape(), "flashlight_button"),
        material="flashlight_button",
        name="button",
    )

    front_button_0 = model.part("front_button_0")
    front_button_0.visual(
        mesh_from_cadquery(build_front_button_shape(), "front_button_0"),
        material="button",
        name="button",
    )

    front_button_1 = model.part("front_button_1")
    front_button_1.visual(
        mesh_from_cadquery(build_front_button_shape(), "front_button_1"),
        material="button",
        name="button",
    )

    model.articulation(
        "jaw_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=10.0, velocity=2.0),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_THICKNESS / 2.0, -0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )
    model.articulation(
        "stand_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -BODY_THICKNESS / 2.0, -0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=3.0, velocity=2.0),
    )
    model.articulation(
        "flashlight_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flashlight_button,
        origin=Origin(xyz=(0.029, 0.0, -0.044)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=2.0, velocity=0.05),
    )
    model.articulation(
        "front_button_0_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_0,
        origin=Origin(xyz=(-0.017, BODY_THICKNESS / 2.0, -0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=2.0, velocity=0.05),
    )
    model.articulation(
        "front_button_1_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_1,
        origin=Origin(xyz=(0.017, BODY_THICKNESS / 2.0, -0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=2.0, velocity=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    flashlight_button = object_model.get_part("flashlight_button")
    front_button_0 = object_model.get_part("front_button_0")
    front_button_1 = object_model.get_part("front_button_1")

    jaw_hinge = object_model.get_articulation("jaw_hinge")
    stand_hinge = object_model.get_articulation("stand_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    flashlight_button_slide = object_model.get_articulation("flashlight_button_slide")
    front_button_0_slide = object_model.get_articulation("front_button_0_slide")
    front_button_1_slide = object_model.get_articulation("front_button_1_slide")

    ctx.allow_isolated_part(
        jaw,
        reason="The jaw is intentionally hinge-mounted with a small visual clearance around the head saddle rather than face-contacting the body shell.",
    )

    ctx.expect_origin_distance(dial, body, axes="x", max_dist=0.003, name="dial stays centered on the front panel")
    ctx.expect_origin_gap(dial, body, axis="y", min_gap=0.018, max_gap=0.020, name="dial sits on the front face")
    ctx.expect_origin_gap(front_button_0, body, axis="y", min_gap=0.018, max_gap=0.020, name="front button 0 sits on the front face")
    ctx.expect_origin_gap(front_button_1, body, axis="y", min_gap=0.018, max_gap=0.020, name="front button 1 sits on the front face")
    ctx.expect_origin_gap(body, stand, axis="y", min_gap=0.018, max_gap=0.020, name="stand is mounted on the back face")
    ctx.expect_origin_gap(flashlight_button, body, axis="x", min_gap=0.027, max_gap=0.031, name="flashlight button sits on the side wall")
    ctx.expect_origin_gap(flashlight_button, front_button_0, axis="z", min_gap=0.020, name="flashlight button is above the lower front buttons")
    ctx.expect_origin_gap(flashlight_button, front_button_1, axis="z", min_gap=0.020, name="flashlight button is above both lower front buttons")
    ctx.expect_origin_gap(dial, front_button_0, axis="z", min_gap=0.055, name="lower front button row sits below the center dial")
    ctx.expect_origin_gap(dial, front_button_1, axis="z", min_gap=0.055, name="both lower front buttons sit below the center dial")

    jaw_rest = ctx.part_world_aabb(jaw)
    stand_rest = ctx.part_world_aabb(stand)
    dial_rest = ctx.part_world_position(dial)
    front_button_0_rest = ctx.part_world_position(front_button_0)
    front_button_1_rest = ctx.part_world_position(front_button_1)
    flashlight_button_rest = ctx.part_world_position(flashlight_button)

    jaw_upper = jaw_hinge.motion_limits.upper if jaw_hinge.motion_limits is not None else None
    if jaw_upper is not None:
        with ctx.pose({jaw_hinge: jaw_upper}):
            jaw_open = ctx.part_world_aabb(jaw)
        ctx.check(
            "jaw opens upward",
            jaw_rest is not None
            and jaw_open is not None
            and jaw_open[1][2] > jaw_rest[1][2] + 0.020,
            details=f"rest={jaw_rest}, open={jaw_open}",
        )

    stand_upper = stand_hinge.motion_limits.upper if stand_hinge.motion_limits is not None else None
    if stand_upper is not None:
        with ctx.pose({stand_hinge: stand_upper}):
            stand_open = ctx.part_world_aabb(stand)
        ctx.check(
            "stand swings out behind the body",
            stand_rest is not None
            and stand_open is not None
            and stand_open[0][1] < stand_rest[0][1] - 0.020,
            details=f"rest={stand_rest}, open={stand_open}",
        )

    dial_pose = pi / 2.0
    with ctx.pose({dial_spin: dial_pose}):
        dial_rotated = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place",
        dial_rest is not None
        and dial_rotated is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(dial_rest, dial_rotated)),
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    button_upper = front_button_0_slide.motion_limits.upper if front_button_0_slide.motion_limits is not None else None
    if button_upper is not None:
        with ctx.pose({front_button_0_slide: button_upper}):
            front_button_0_pressed = ctx.part_world_position(front_button_0)
        ctx.check(
            "front button 0 depresses inward",
            front_button_0_rest is not None
            and front_button_0_pressed is not None
            and front_button_0_pressed[1] < front_button_0_rest[1] - 0.0015,
            details=f"rest={front_button_0_rest}, pressed={front_button_0_pressed}",
        )

    flashlight_upper = (
        flashlight_button_slide.motion_limits.upper if flashlight_button_slide.motion_limits is not None else None
    )
    if flashlight_upper is not None:
        with ctx.pose({flashlight_button_slide: flashlight_upper}):
            flashlight_button_pressed = ctx.part_world_position(flashlight_button)
        ctx.check(
            "flashlight button depresses inward from the side",
            flashlight_button_rest is not None
            and flashlight_button_pressed is not None
            and flashlight_button_pressed[0] < flashlight_button_rest[0] - 0.0015,
            details=f"rest={flashlight_button_rest}, pressed={flashlight_button_pressed}",
        )

    with ctx.pose({front_button_1_slide: front_button_1_slide.motion_limits.upper}):
        front_button_1_pressed = ctx.part_world_position(front_button_1)
    ctx.check(
        "front button 1 depresses independently",
        front_button_1_rest is not None
        and front_button_1_pressed is not None
        and front_button_1_pressed[1] < front_button_1_rest[1] - 0.0015,
        details=f"rest={front_button_1_rest}, pressed={front_button_1_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
