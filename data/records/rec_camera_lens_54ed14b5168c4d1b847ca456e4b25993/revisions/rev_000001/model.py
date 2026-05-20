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


LENS_LENGTH = 0.256
FOCUS_RING_CENTER_X = 0.105
FOCUS_RING_WIDTH = 0.038
COLLAR_CENTER_X = 0.204
COLLAR_WIDTH = 0.024


def _x_cylinder(x_start: float, length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").workplane(offset=x_start).circle(radius).extrude(length)


def _body_shape() -> cq.Workplane:
    focus_left_edge = FOCUS_RING_CENTER_X - (FOCUS_RING_WIDTH * 0.5)
    focus_right_edge = FOCUS_RING_CENTER_X + (FOCUS_RING_WIDTH * 0.5)
    collar_left_edge = COLLAR_CENTER_X - (COLLAR_WIDTH * 0.5)
    collar_right_edge = COLLAR_CENTER_X + (COLLAR_WIDTH * 0.5)

    body = _x_cylinder(0.000, 0.008, 0.0425)
    body = body.union(_x_cylinder(0.008, 0.030, 0.0410))
    body = body.union(_x_cylinder(0.038, 0.034, 0.0430))
    body = body.union(_x_cylinder(0.072, 0.066, 0.0445))
    body = body.union(_x_cylinder(0.138, 0.028, 0.0470))
    body = body.union(_x_cylinder(0.166, 0.016, 0.0510))
    body = body.union(_x_cylinder(0.182, 0.058, 0.0545))
    body = body.union(_x_cylinder(0.240, 0.016, 0.0580))
    body = body.union(_x_cylinder(focus_left_edge - 0.004, 0.004, 0.0462))
    body = body.union(_x_cylinder(focus_right_edge, 0.004, 0.0462))
    body = body.union(_x_cylinder(collar_left_edge - 0.004, 0.004, 0.0562))
    body = body.union(_x_cylinder(collar_right_edge, 0.004, 0.0562))

    front_bore = _x_cylinder(0.208, 0.048, 0.0430)
    front_recess = _x_cylinder(0.226, 0.030, 0.0465)
    rear_socket = _x_cylinder(0.000, 0.010, 0.0200)

    return body.cut(front_bore).cut(front_recess).cut(rear_socket)


def _geared_ring_shape(
    *,
    width: float,
    inner_radius: float,
    ring_radius: float,
    tooth_height: float,
    tooth_depth: float,
    tooth_count: int,
) -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(ring_radius)
        .circle(inner_radius)
        .extrude(width / 2.0, both=True)
    )

    tooth_radius = ring_radius + (tooth_height * 0.5)
    for index in range(tooth_count):
        angle_deg = 360.0 * index / tooth_count
        tooth = (
            cq.Workplane("XY")
            .box(width, tooth_depth, tooth_height, centered=(True, True, True))
            .translate((0.0, 0.0, tooth_radius))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        )
        ring = ring.union(tooth)
    return ring


def _orientation_collar_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("YZ")
        .circle(0.0615)
        .circle(0.0558)
        .extrude(COLLAR_WIDTH / 2.0, both=True)
    )
    collar = collar.union(
        cq.Workplane("YZ")
        .circle(0.0635)
        .circle(0.0605)
        .extrude(0.0035, both=True)
        .translate((COLLAR_WIDTH * 0.5 - 0.0035, 0.0, 0.0))
    )
    collar = collar.union(
        cq.Workplane("YZ")
        .circle(0.0635)
        .circle(0.0605)
        .extrude(0.0035, both=True)
        .translate((-COLLAR_WIDTH * 0.5 + 0.0035, 0.0, 0.0))
    )

    rib_radius = 0.0624
    for index in range(32):
        angle_deg = 360.0 * index / 32.0
        rib = (
            cq.Workplane("XY")
            .box(COLLAR_WIDTH * 0.82, 0.0032, 0.0028, centered=(True, True, True))
            .translate((0.0, 0.0, rib_radius))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        )
        collar = collar.union(rib)
    return collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cine_anamorphic_lens")

    barrel_black = model.material("barrel_black", rgba=(0.08, 0.08, 0.09, 1.0))
    ring_black = model.material("ring_black", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.10, 0.14, 0.20, 0.55))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "anamorphic_body"),
        material=barrel_black,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.0470, length=0.0035),
        origin=Origin(xyz=(0.230, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_dark,
        name="front_element",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            _geared_ring_shape(
                width=FOCUS_RING_WIDTH,
                inner_radius=0.0456,
                ring_radius=0.0503,
                tooth_height=0.0030,
                tooth_depth=0.0044,
                tooth_count=72,
            ),
            "focus_ring",
        ),
        material=ring_black,
        name="focus_band",
    )
    focus_ring.visual(
        Box((0.012, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0515)),
        material=trim_gray,
        name="focus_witness",
    )

    orientation_collar = model.part("orientation_collar")
    orientation_collar.visual(
        mesh_from_cadquery(_orientation_collar_shape(), "orientation_collar"),
        material=ring_black,
        name="collar_band",
    )
    orientation_collar.visual(
        Box((0.010, 0.0035, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0633)),
        material=trim_gray,
        name="collar_witness",
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(FOCUS_RING_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "body_to_orientation_collar",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=orientation_collar,
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    orientation_collar = object_model.get_part("orientation_collar")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    collar_joint = object_model.get_articulation("body_to_orientation_collar")

    ctx.allow_overlap(
        body,
        focus_ring,
        reason=(
            "The geared focus ring is intentionally represented as a captured rotating sleeve "
            "around the central barrel, and the coaxial sleeve fit is simplified as nested solids."
        ),
    )
    ctx.allow_overlap(
        body,
        orientation_collar,
        reason=(
            "The orientation collar is intentionally represented as a rotating sleeve around the "
            "front squeeze housing, using a simplified nested-fit proxy."
        ),
    )

    ctx.expect_origin_distance(
        focus_ring,
        body,
        axes="yz",
        max_dist=0.0005,
        name="focus ring stays coaxial with the barrel",
    )
    ctx.expect_origin_distance(
        orientation_collar,
        body,
        axes="yz",
        max_dist=0.0005,
        name="orientation collar stays coaxial with the front housing",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="x",
        min_overlap=0.030,
        name="focus ring overlaps its support band",
    )
    ctx.expect_overlap(
        orientation_collar,
        body,
        axes="x",
        min_overlap=0.018,
        name="orientation collar wraps the front housing",
    )
    ctx.expect_origin_gap(
        orientation_collar,
        focus_ring,
        axis="x",
        min_gap=0.085,
        max_gap=0.115,
        name="orientation collar sits ahead of the focus ring",
    )

    rest_focus_witness = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_witness"))
    with ctx.pose({focus_joint: math.pi / 2.0}):
        quarter_focus_witness = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_witness"))
    ctx.check(
        "focus ring witness sweeps around the lens axis",
        rest_focus_witness is not None
        and quarter_focus_witness is not None
        and abs(quarter_focus_witness[0] - rest_focus_witness[0]) < 0.001
        and rest_focus_witness[2] > 0.045
        and abs(quarter_focus_witness[1]) > 0.045
        and abs(quarter_focus_witness[2]) < 0.010,
        details=f"rest={rest_focus_witness}, quarter_turn={quarter_focus_witness}",
    )

    rest_collar_witness = _aabb_center(
        ctx.part_element_world_aabb(orientation_collar, elem="collar_witness")
    )
    with ctx.pose({collar_joint: math.pi / 2.0}):
        quarter_collar_witness = _aabb_center(
            ctx.part_element_world_aabb(orientation_collar, elem="collar_witness")
        )
    ctx.check(
        "orientation collar witness rotates around the front housing",
        rest_collar_witness is not None
        and quarter_collar_witness is not None
        and abs(quarter_collar_witness[0] - rest_collar_witness[0]) < 0.001
        and rest_collar_witness[2] > 0.055
        and abs(quarter_collar_witness[1]) > 0.055
        and abs(quarter_collar_witness[2]) < 0.012,
        details=f"rest={rest_collar_witness}, quarter_turn={quarter_collar_witness}",
    )

    return ctx.report()


object_model = build_object_model()
