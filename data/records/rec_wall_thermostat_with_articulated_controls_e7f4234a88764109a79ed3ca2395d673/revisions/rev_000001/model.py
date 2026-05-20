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


BODY_RADIUS = 0.036
BODY_DEPTH = 0.024
REAR_PLATE_RADIUS = 0.039
REAR_PLATE_THICKNESS = 0.002
FRONT_POCKET_DEPTH = 0.0065
BODY_BEARING_DEPTH = 0.004
BODY_BEARING_Z = 0.002

DISPLAY_RADIUS = 0.0305
DISPLAY_THICKNESS = 0.004
DISPLAY_Z = BODY_DEPTH * 0.5 - FRONT_POCKET_DEPTH + DISPLAY_THICKNESS * 0.5 + 0.0004

RING_OUTER_RADIUS = 0.043
RING_CLEAR_RADIUS = BODY_RADIUS + 0.0008
RING_WINDOW_RADIUS = 0.0315
RING_SLEEVE_DEPTH = 0.014
RING_FACE_DEPTH = 0.0032
RING_CENTER_Z = 0.004
RING_FACE_CENTER_Z = BODY_DEPTH * 0.5 - RING_FACE_DEPTH * 0.5

SLIDER_SLOT_Y = 0.022
SLIDER_SLOT_Z = 0.009
SLIDER_SLOT_X = 0.012
SLIDER_Z = -0.008
SLIDER_TRAVEL = 0.009


def _ring_annulus(outer_radius: float, inner_radius: float, depth: float):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(depth * 0.5, both=True)
        .cut(
            cq.Workplane("XY")
            .circle(inner_radius)
            .extrude(depth * 0.5 + 0.002, both=True)
        )
    )


def _make_body_shape():
    body = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_DEPTH * 0.5, both=True)

    rear_plate = (
        cq.Workplane("XY")
        .circle(REAR_PLATE_RADIUS)
        .extrude(REAR_PLATE_THICKNESS * 0.5, both=True)
        .translate((0.0, 0.0, -BODY_DEPTH * 0.5))
    )
    body = body.union(rear_plate)

    front_pocket = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS + 0.003)
        .extrude(FRONT_POCKET_DEPTH)
        .translate((0.0, 0.0, BODY_DEPTH * 0.5 - FRONT_POCKET_DEPTH * 0.5))
    )
    body = body.cut(front_pocket)

    bearing_collar = (
        cq.Workplane("XY")
        .circle(RING_CLEAR_RADIUS)
        .extrude(BODY_BEARING_DEPTH * 0.5, both=True)
        .translate((0.0, 0.0, BODY_BEARING_Z))
    )
    body = body.union(bearing_collar)

    slot_cutter = (
        cq.Workplane("XY")
        .box(SLIDER_SLOT_X, SLIDER_SLOT_Y, SLIDER_SLOT_Z)
        .edges("|X")
        .fillet(0.0024)
        .translate((BODY_RADIUS - 0.002, 0.0, SLIDER_Z))
    )
    body = body.cut(slot_cutter)

    return body


def _make_ring_shape():
    ring = _ring_annulus(RING_OUTER_RADIUS, RING_CLEAR_RADIUS, RING_SLEEVE_DEPTH).translate(
        (0.0, 0.0, RING_CENTER_Z)
    )

    groove_radius = RING_OUTER_RADIUS - 0.0003
    for index in range(24):
        angle_deg = index * 15.0
        cutter = (
            cq.Workplane("XY")
            .box(0.0015, 0.0042, RING_SLEEVE_DEPTH + 0.002)
            .translate((groove_radius, 0.0, RING_CENTER_Z))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        ring = ring.cut(cutter)

    return ring


def _make_display_shape():
    return cq.Workplane("XY").circle(DISPLAY_RADIUS).extrude(DISPLAY_THICKNESS * 0.5, both=True)


def _make_slider_shape():
    thumb = (
        cq.Workplane("XY")
        .box(0.0045, 0.009, 0.0065)
        .edges("|X")
        .fillet(0.0022)
        .translate((0.0042, 0.0, 0.0))
    )
    stem = cq.Workplane("XY").box(0.007, 0.005, 0.0065).translate((-0.001, 0.0, 0.0))
    guide = cq.Workplane("XY").box(0.0035, 0.004, 0.0035).translate((-0.00625, 0.0, -0.001))
    return thumb.union(stem).union(guide)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_thermostat")

    housing_finish = model.material("housing_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    ring_finish = model.material("ring_finish", rgba=(0.74, 0.75, 0.78, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.08, 0.11, 0.13, 0.88))
    slider_finish = model.material("slider_finish", rgba=(0.28, 0.30, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "thermostat_body"),
        material=housing_finish,
        name="housing_shell",
    )

    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_make_ring_shape(), "thermostat_ring"),
        material=ring_finish,
        name="outer_ring",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_make_display_shape(), "thermostat_display"),
        material=glass_finish,
        name="display_glass",
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        mesh_from_cadquery(_make_slider_shape(), "thermostat_mode_slider"),
        material=slider_finish,
        name="slider_cap",
    )

    model.articulation(
        "body_to_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, DISPLAY_Z)),
    )

    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_slider,
        origin=Origin(xyz=(BODY_RADIUS, 0.0, SLIDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=-SLIDER_TRAVEL * 0.5,
            upper=SLIDER_TRAVEL * 0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("ring")
    display = object_model.get_part("display")
    mode_slider = object_model.get_part("mode_slider")
    ring_joint = object_model.get_articulation("body_to_ring")
    slider_joint = object_model.get_articulation("body_to_mode_slider")

    ring_limits = ring_joint.motion_limits
    slider_limits = slider_joint.motion_limits

    ctx.allow_overlap(
        body,
        ring,
        elem_a="housing_shell",
        elem_b="outer_ring",
        reason="The rotating outer ring is intentionally modeled as a captured annulus riding on the housing bearing collar.",
    )
    ctx.allow_overlap(
        body,
        mode_slider,
        elem_a="housing_shell",
        elem_b="slider_cap",
        reason="The mode slider includes an internal guide tongue that intentionally rides inside the side-slot proxy in the housing.",
    )

    ctx.check(
        "outer ring uses continuous rotation",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS
        and ring_limits is not None
        and ring_limits.lower is None
        and ring_limits.upper is None,
        details=f"type={ring_joint.articulation_type}, limits={ring_limits!r}",
    )
    ctx.check(
        "mode slider travel stays short",
        slider_joint.articulation_type == ArticulationType.PRISMATIC
        and slider_limits is not None
        and slider_limits.lower is not None
        and slider_limits.upper is not None
        and 0.006 <= slider_limits.upper - slider_limits.lower <= 0.012,
        details=f"type={slider_joint.articulation_type}, limits={slider_limits!r}",
    )

    ctx.expect_origin_distance(
        ring,
        display,
        axes="xy",
        max_dist=0.001,
        name="display stays concentric inside ring",
    )
    ctx.expect_origin_distance(
        mode_slider,
        body,
        axes="y",
        max_dist=0.001,
        name="slider starts centered in side slot",
    )

    lower_pos = None
    upper_pos = None
    if slider_limits is not None and slider_limits.lower is not None and slider_limits.upper is not None:
        with ctx.pose({slider_joint: slider_limits.lower}):
            lower_pos = ctx.part_world_position(mode_slider)
        with ctx.pose({slider_joint: slider_limits.upper}):
            upper_pos = ctx.part_world_position(mode_slider)

    ctx.check(
        "mode slider moves upward along housing edge",
        lower_pos is not None and upper_pos is not None and upper_pos[1] > lower_pos[1] + 0.006,
        details=f"lower={lower_pos}, upper={upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
