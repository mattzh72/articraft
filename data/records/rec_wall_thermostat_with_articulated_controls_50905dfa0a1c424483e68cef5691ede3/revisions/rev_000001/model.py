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


OUTER_RING_RADIUS = 0.042
BODY_FRONT_RADIUS = 0.0348
RING_INNER_RADIUS = 0.0348
BODY_REAR_RADIUS = 0.0375
BUTTON_RADIUS = 0.0330
BUTTON_TRAVEL = 0.0034


def _build_body_shape() -> cq.Workplane:
    rear_shell = cq.Workplane("XY").circle(BODY_REAR_RADIUS).extrude(0.012)
    front_shell = (
        cq.Workplane("XY")
        .circle(BODY_FRONT_RADIUS)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.012))
    )
    body = rear_shell.union(front_shell)

    front_recess = (
        cq.Workplane("XY")
        .circle(0.0330)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.018))
    )
    guide_bore = (
        cq.Workplane("XY")
        .circle(0.0108)
        .extrude(0.0194)
        .translate((0.0, 0.0, 0.007))
    )
    return body.cut(front_recess).cut(guide_bore).translate((0.0, 0.0, 0.002))


def _build_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(OUTER_RING_RADIUS)
        .circle(RING_INNER_RADIUS)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.0145))
    )


def _build_button_face_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(BUTTON_RADIUS).extrude(0.0032)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_wall_thermostat")

    wall_white = model.material("wall_white", rgba=(0.95, 0.95, 0.94, 1.0))
    housing_black = model.material("housing_black", rgba=(0.14, 0.15, 0.16, 1.0))
    ring_metal = model.material("ring_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    glass_black = model.material("glass_black", rgba=(0.09, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.090, 0.090, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=wall_white,
        name="wall_plate",
    )
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "thermostat_body"),
        material=housing_black,
        name="housing",
    )

    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_build_ring_shape(), "thermostat_ring"),
        material=ring_metal,
        name="outer_ring",
    )

    center = model.part("center")
    center.visual(
        mesh_from_cadquery(_build_button_face_shape(), "thermostat_button_face"),
        material=glass_black,
        name="button_face",
    )
    center.visual(
        Cylinder(radius=0.0100, length=0.0114),
        origin=Origin(xyz=(0.0, 0.0, -0.0055)),
        material=housing_black,
        name="button_stem",
    )

    model.articulation(
        "body_to_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "body_to_center",
        ArticulationType.PRISMATIC,
        parent=body,
        child=center,
        origin=Origin(xyz=(0.0, 0.0, 0.0222)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.04,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    ring = object_model.get_part("ring")
    center = object_model.get_part("center")
    ring_joint = object_model.get_articulation("body_to_ring")
    button_joint = object_model.get_articulation("body_to_center")

    ctx.allow_overlap(
        body,
        ring,
        elem_a="housing",
        elem_b="outer_ring",
        reason="The rotating metal ring is intentionally modeled as a close nested bearing shell around the thermostat body shoulder.",
    )

    ctx.expect_origin_distance(
        ring,
        body,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="ring stays concentric with body",
    )
    ctx.expect_within(
        center,
        body,
        axes="xy",
        inner_elem="button_face",
        outer_elem="housing",
        margin=0.002,
        name="button face stays within housing footprint",
    )

    ring_top = ctx.part_element_world_aabb(ring, elem="outer_ring")
    button_top = ctx.part_element_world_aabb(center, elem="button_face")
    ctx.check(
        "button face sits slightly below the rotating ring",
        ring_top is not None
        and button_top is not None
        and button_top[1][2] < ring_top[1][2]
        and button_top[1][2] > ring_top[1][2] - 0.0025,
        details=f"ring_aabb={ring_top}, button_aabb={button_top}",
    )

    rest_ring_position = ctx.part_world_position(ring)
    with ctx.pose({ring_joint: math.pi / 2.0}):
        quarter_turn_position = ctx.part_world_position(ring)
    ctx.check(
        "outer ring rotates in place about the center axis",
        rest_ring_position is not None
        and quarter_turn_position is not None
        and abs(rest_ring_position[0] - quarter_turn_position[0]) < 1e-6
        and abs(rest_ring_position[1] - quarter_turn_position[1]) < 1e-6
        and abs(rest_ring_position[2] - quarter_turn_position[2]) < 1e-6,
        details=f"rest={rest_ring_position}, quarter_turn={quarter_turn_position}",
    )

    rest_button_position = ctx.part_world_position(center)
    button_upper = button_joint.motion_limits.upper
    with ctx.pose({button_joint: button_upper}):
        ctx.expect_within(
            center,
            body,
            axes="xy",
            inner_elem="button_face",
            outer_elem="housing",
            margin=0.002,
            name="pressed button face stays within housing footprint",
        )
        pressed_button_position = ctx.part_world_position(center)
        pressed_button_aabb = ctx.part_element_world_aabb(center, elem="button_face")
    ctx.check(
        "center push surface depresses inward",
        rest_button_position is not None
        and pressed_button_position is not None
        and pressed_button_position[2] < rest_button_position[2] - 0.003,
        details=f"rest={rest_button_position}, pressed={pressed_button_position}",
    )
    ctx.check(
        "pressed button remains visibly forward of the wall plane",
        pressed_button_aabb is not None and pressed_button_aabb[1][2] > 0.018,
        details=f"pressed_button_aabb={pressed_button_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
