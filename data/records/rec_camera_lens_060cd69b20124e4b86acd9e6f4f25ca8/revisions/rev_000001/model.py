from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.061
FOCUS_RING_START = 0.015
FOCUS_RING_LENGTH = 0.033
INNER_TRAVEL = 0.018


def _tube(outer_radius: float, inner_radius: float, length: float, *, z0: float = 0.0):
    thickness = outer_radius - inner_radius
    mid_radius = inner_radius + thickness * 0.5
    tangent_width = (2.0 * math.pi * mid_radius / 32.0) * 1.08

    shell = None
    for index in range(32):
        angle_deg = 360.0 * index / 32.0
        panel = (
            cq.Workplane("XY")
            .box(thickness, tangent_width, length)
            .translate((mid_radius, 0.0, z0 + length * 0.5))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        shell = panel if shell is None else shell.union(panel)

    return shell


def _solid_cylinder(radius: float, length: float, *, z0: float = 0.0):
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, z0))


def _build_body_shape():
    body = _tube(0.0325, 0.0275, 0.005, z0=0.0)
    body = body.union(_tube(0.0310, 0.0275, 0.010, z0=0.005))
    body = body.union(_tube(0.0285, 0.0275, 0.034, z0=0.015))
    body = body.union(_tube(0.0305, 0.0280, 0.010, z0=0.049))
    body = body.union(_tube(0.0315, 0.0280, 0.002, z0=0.059))
    return body


def _build_focus_ring_shape():
    ring = _tube(0.0331, 0.0294, FOCUS_RING_LENGTH, z0=0.0)
    rib_starts = (0.001, 0.005, 0.009, 0.013, 0.017, 0.021, 0.025, 0.029)
    for z0 in rib_starts:
        ring = ring.union(_tube(0.0344, 0.0294, 0.002, z0=z0))
    return ring


def _build_inner_barrel_shape():
    inner_barrel = _tube(0.0262, 0.0212, 0.061, z0=-0.031)
    inner_barrel = inner_barrel.union(_tube(0.0273, 0.0212, 0.008, z0=-0.031))
    inner_barrel = inner_barrel.union(_tube(0.0295, 0.0212, 0.002, z0=0.0))
    inner_barrel = inner_barrel.union(_tube(0.0274, 0.0212, 0.008, z0=0.018))
    inner_barrel = inner_barrel.union(_tube(0.0290, 0.0212, 0.004, z0=0.030))
    inner_barrel = inner_barrel.union(_tube(0.0218, 0.0185, 0.010, z0=0.016))
    return inner_barrel


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_camera_lens")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.14, 0.15, 1.0))
    index_gray = model.material("index_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "lens_body"),
        material=anodized_black,
        name="outer_shell",
    )
    body.visual(
        Box((0.010, 0.005, 0.0025)),
        origin=Origin(xyz=(0.0, 0.0308, 0.023)),
        material=index_gray,
        name="distance_window",
    )
    body.visual(
        Box((0.020, 0.0012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0287, 0.036)),
        material=index_gray,
        name="focus_index",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_build_focus_ring_shape(), "focus_ring"),
        material=rubber_black,
        name="ring_shell",
    )
    focus_ring.visual(
        Box((0.0016, 0.0044, 0.010)),
        origin=Origin(xyz=(0.0332, 0.0, 0.0165)),
        material=rubber_black,
        name="seam_pad",
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        mesh_from_cadquery(_build_inner_barrel_shape(), "inner_barrel"),
        material=satin_black,
        name="telescoping_shell",
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, FOCUS_RING_START)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "close_focus_extension",
        ArticulationType.PRISMATIC,
        parent=body,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, BODY_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.08,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    inner_barrel = object_model.get_part("inner_barrel")

    focus_rotation = object_model.get_articulation("focus_rotation")
    close_focus_extension = object_model.get_articulation("close_focus_extension")

    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="outer_shell",
        elem_b="ring_shell",
        reason=(
            "The focus ring is intentionally represented as a concentric sleeve around the barrel, "
            "so the nested shell fit is an intended coaxial embedding."
        ),
    )
    ctx.allow_overlap(
        body,
        inner_barrel,
        elem_a="outer_shell",
        elem_b="telescoping_shell",
        reason=(
            "The telescoping inner barrel is intentionally represented as a close-fitting nested sleeve "
            "inside the outer barrel."
        ),
    )

    ctx.expect_origin_distance(
        focus_ring,
        body,
        axes="xy",
        max_dist=0.0005,
        name="focus ring stays concentric with the barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="xy",
        min_overlap=0.056,
        name="focus ring wraps the fixed barrel footprint",
    )
    ctx.expect_within(
        inner_barrel,
        body,
        axes="xy",
        margin=0.001,
        name="collapsed inner barrel stays centered in the outer barrel",
    )
    ctx.expect_overlap(
        inner_barrel,
        body,
        axes="z",
        min_overlap=0.029,
        name="collapsed inner barrel remains deeply inserted",
    )

    rest_inner_pos = ctx.part_world_position(inner_barrel)
    rest_ring_pos = ctx.part_world_position(focus_ring)
    seam_rest_center = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="seam_pad"))

    with ctx.pose({close_focus_extension: INNER_TRAVEL}):
        ctx.expect_within(
            inner_barrel,
            body,
            axes="xy",
            margin=0.001,
            name="extended inner barrel stays centered in the outer barrel",
        )
        ctx.expect_overlap(
            inner_barrel,
            body,
            axes="z",
            min_overlap=0.010,
            name="extended inner barrel retains insertion in the body",
        )
        extended_inner_pos = ctx.part_world_position(inner_barrel)

    with ctx.pose({focus_rotation: math.pi / 2.0}):
        seam_rotated_center = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="seam_pad"))
        rotated_ring_pos = ctx.part_world_position(focus_ring)

    ctx.check(
        "inner barrel extends forward along the optical axis",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[2] > rest_inner_pos[2] + 0.017,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )
    ctx.check(
        "focus ring spins in place",
        rest_ring_pos is not None
        and rotated_ring_pos is not None
        and abs(rotated_ring_pos[0] - rest_ring_pos[0]) < 1e-6
        and abs(rotated_ring_pos[1] - rest_ring_pos[1]) < 1e-6
        and abs(rotated_ring_pos[2] - rest_ring_pos[2]) < 1e-6,
        details=f"rest={rest_ring_pos}, rotated={rotated_ring_pos}",
    )
    ctx.check(
        "focus ring seam visibly rotates around the barrel",
        seam_rest_center is not None
        and seam_rotated_center is not None
        and abs(seam_rest_center[0] - seam_rotated_center[0]) > 0.020
        and abs(seam_rest_center[1] - seam_rotated_center[1]) > 0.020
        and abs(seam_rest_center[2] - seam_rotated_center[2]) < 0.002,
        details=f"rest={seam_rest_center}, rotated={seam_rotated_center}",
    )
    ctx.check(
        "articulations match a manual macro lens",
        focus_rotation.articulation_type == ArticulationType.CONTINUOUS
        and close_focus_extension.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"focus_rotation={focus_rotation.articulation_type}, "
            f"close_focus_extension={close_focus_extension.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
