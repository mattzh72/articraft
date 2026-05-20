from __future__ import annotations

import cadquery as cq
import math

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


def hollow_cylinder(
    outer_radius: float,
    inner_radius: float,
    length: float,
    z0: float,
) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(outer_radius)
        .extrude(length)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.0005)
        .circle(inner_radius)
        .extrude(length + 0.001)
    )
    return outer.cut(inner)


def grooved_ring(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    groove_depth: float,
    groove_width: float,
    groove_count: int,
) -> cq.Workplane:
    ring = hollow_cylinder(outer_radius, inner_radius, length, -length / 2.0)
    for idx in range(groove_count):
        z_center = -length / 2.0 + (idx + 1) * length / (groove_count + 1)
        groove = hollow_cylinder(
            outer_radius,
            outer_radius - groove_depth,
            groove_width,
            z_center - groove_width / 2.0,
        )
        ring = ring.cut(groove)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilized_zoom_lens")

    body_black = model.material("body_black", rgba=(0.08, 0.08, 0.085, 1.0))
    ring_black = model.material("ring_black", rgba=(0.14, 0.14, 0.145, 1.0))
    detail_gray = model.material("detail_gray", rgba=(0.38, 0.38, 0.40, 1.0))

    body = model.part("body")
    body_shape = hollow_cylinder(0.0385, 0.0285, 0.004, 0.0)
    body_shape = body_shape.union(hollow_cylinder(0.0365, 0.0285, 0.044, 0.004))
    body_shape = body_shape.union(hollow_cylinder(0.0350, 0.0285, 0.038, 0.048))
    body_shape = body_shape.union(hollow_cylinder(0.0365, 0.0285, 0.034, 0.086))
    body_shape = body_shape.union(hollow_cylinder(0.0348, 0.0285, 0.034, 0.120))
    body_shape = body_shape.union(hollow_cylinder(0.0354, 0.0285, 0.002, 0.066))
    body_shape = body_shape.union(hollow_cylinder(0.0352, 0.0285, 0.002, 0.136))
    body_shape = body_shape.union(hollow_cylinder(0.0395, 0.0285, 0.025, 0.154))
    body_shape = body_shape.union(hollow_cylinder(0.0410, 0.0285, 0.006, 0.179))
    body_shape = body_shape.union(
        cq.Workplane("XY").box(0.010, 0.024, 0.034).translate((0.0405, 0.0, 0.103))
    )
    body.visual(
        mesh_from_cadquery(body_shape, "lens_body"),
        material=body_black,
        name="body_shell",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_cadquery(
            grooved_ring(
                outer_radius=0.0415,
                inner_radius=0.0354,
                length=0.032,
                groove_depth=0.0012,
                groove_width=0.0022,
                groove_count=7,
            ),
            "zoom_ring",
        ),
        material=ring_black,
        name="zoom_shell",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            grooved_ring(
                outer_radius=0.0408,
                inner_radius=0.0352,
                length=0.030,
                groove_depth=0.0010,
                groove_width=0.0018,
                groove_count=9,
            ),
            "focus_ring",
        ),
        material=ring_black,
        name="focus_shell",
    )

    model.articulation(
        "body_to_zoom_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=10.0),
    )
    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=14.0),
    )

    stabilizer_switch = model.part("stabilizer_switch")
    stabilizer_switch.visual(
        Cylinder(radius=0.0018, length=0.014),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=detail_gray,
        name="barrel",
    )
    stabilizer_switch.visual(
        Box((0.0030, 0.012, 0.0030)),
        origin=Origin(xyz=(0.0022, 0.0, -0.0014)),
        material=detail_gray,
        name="arm",
    )
    stabilizer_switch.visual(
        Box((0.0032, 0.0115, 0.0105)),
        origin=Origin(xyz=(0.0027, 0.0, -0.0065)),
        material=detail_gray,
        name="paddle",
    )
    stabilizer_switch.visual(
        Box((0.0040, 0.0085, 0.0016)),
        origin=Origin(xyz=(0.0034, 0.0, -0.0120)),
        material=detail_gray,
        name="tip",
    )

    focus_mode_switch = model.part("focus_mode_switch")
    focus_mode_switch.visual(
        Cylinder(radius=0.0018, length=0.014),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=detail_gray,
        name="barrel",
    )
    focus_mode_switch.visual(
        Box((0.0030, 0.012, 0.0030)),
        origin=Origin(xyz=(0.0022, 0.0, -0.0012)),
        material=detail_gray,
        name="arm",
    )
    focus_mode_switch.visual(
        Box((0.0030, 0.0110, 0.0090)),
        origin=Origin(xyz=(0.0026, 0.0, -0.0058)),
        material=detail_gray,
        name="paddle",
    )
    focus_mode_switch.visual(
        Box((0.0038, 0.0085, 0.0014)),
        origin=Origin(xyz=(0.0032, 0.0, -0.0104)),
        material=detail_gray,
        name="tip",
    )

    model.articulation(
        "body_to_stabilizer_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stabilizer_switch,
        origin=Origin(xyz=(0.0473, 0.0, 0.098)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "body_to_focus_mode_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_mode_switch,
        origin=Origin(xyz=(0.0473, 0.0, 0.114)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    stabilizer_switch = object_model.get_part("stabilizer_switch")
    focus_mode_switch = object_model.get_part("focus_mode_switch")
    zoom_joint = object_model.get_articulation("body_to_zoom_ring")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    stabilizer_joint = object_model.get_articulation("body_to_stabilizer_switch")
    focus_mode_joint = object_model.get_articulation("body_to_focus_mode_switch")

    ctx.allow_overlap(
        body,
        zoom_ring,
        reason="The zoom ring is intentionally modeled as a close nested sleeve rotating around the barrel core.",
    )
    ctx.allow_overlap(
        body,
        focus_ring,
        reason="The focus ring is intentionally modeled as a close nested sleeve rotating around the front barrel core.",
    )

    ctx.expect_origin_distance(
        body,
        zoom_ring,
        axes="xy",
        max_dist=0.001,
        name="zoom ring stays coaxial with the barrel",
    )
    ctx.expect_origin_distance(
        body,
        focus_ring,
        axes="xy",
        max_dist=0.001,
        name="focus ring stays coaxial with the barrel",
    )
    ctx.expect_origin_gap(
        focus_ring,
        zoom_ring,
        axis="z",
        min_gap=0.055,
        max_gap=0.080,
        name="focus ring sits forward of the zoom ring",
    )
    ctx.expect_origin_gap(
        focus_mode_switch,
        stabilizer_switch,
        axis="z",
        min_gap=0.012,
        max_gap=0.020,
        name="side switch levers are vertically stacked",
    )
    ctx.expect_gap(
        stabilizer_switch,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="stabilizer switch is hinged against the side panel",
    )
    ctx.expect_gap(
        focus_mode_switch,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="focus mode switch is hinged against the side panel",
    )

    with ctx.pose({zoom_joint: 1.1, focus_joint: -0.85}):
        ctx.expect_origin_distance(
            body,
            zoom_ring,
            axes="xy",
            max_dist=0.001,
            name="zoom ring remains centered while rotated",
        )
        ctx.expect_origin_distance(
            body,
            focus_ring,
            axes="xy",
            max_dist=0.001,
            name="focus ring remains centered while rotated",
        )

    rest_stabilizer = ctx.part_element_world_aabb(stabilizer_switch, elem="paddle")
    rest_focus_mode = ctx.part_element_world_aabb(focus_mode_switch, elem="paddle")
    with ctx.pose({stabilizer_joint: 0.24}):
        stabilizer_swung = ctx.part_element_world_aabb(stabilizer_switch, elem="paddle")
        focus_mode_steady = ctx.part_element_world_aabb(focus_mode_switch, elem="paddle")
    with ctx.pose({focus_mode_joint: 0.24}):
        focus_mode_swung = ctx.part_element_world_aabb(focus_mode_switch, elem="paddle")

    ctx.check(
        "stabilizer switch pivots outward",
        rest_stabilizer is not None
        and stabilizer_swung is not None
        and stabilizer_swung[1][0] > rest_stabilizer[1][0] + 0.001,
        details=f"rest={rest_stabilizer}, swung={stabilizer_swung}",
    )
    ctx.check(
        "focus mode switch pivots outward",
        rest_focus_mode is not None
        and focus_mode_swung is not None
        and focus_mode_swung[1][0] > rest_focus_mode[1][0] + 0.001,
        details=f"rest={rest_focus_mode}, swung={focus_mode_swung}",
    )
    ctx.check(
        "switch levers pivot independently",
        rest_focus_mode is not None
        and focus_mode_steady is not None
        and abs(focus_mode_steady[1][0] - rest_focus_mode[1][0]) < 1e-6,
        details=f"rest={rest_focus_mode}, while_stabilizer_moves={focus_mode_steady}",
    )

    return ctx.report()


object_model = build_object_model()
