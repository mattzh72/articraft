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


def _tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _grooved_ring(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    groove_count: int,
    groove_width: float,
    groove_depth: float,
    end_margin: float,
) -> cq.Workplane:
    ring = _tube(outer_radius, inner_radius, length)
    active_length = max(length - (2.0 * end_margin), 0.001)
    z_center = end_margin + (0.5 * active_length)
    radial_center = outer_radius - (0.5 * groove_depth)
    for index in range(groove_count):
        angle_deg = 360.0 * index / groove_count
        cutter = (
            cq.Workplane("XY")
            .box(groove_depth + 0.006, groove_width, active_length)
            .translate((radial_center, 0.0, z_center))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        ring = ring.cut(cutter)
    return ring


def _build_body_shape() -> cq.Workplane:
    body = _tube(0.0188, 0.0102, 0.0500)
    body = body.union(_tube(0.0260, 0.0130, 0.0028))
    body = body.union(_tube(0.0222, 0.0188, 0.0118).translate((0.0, 0.0, 0.0024)))
    body = body.union(_tube(0.0217, 0.0188, 0.0168).translate((0.0, 0.0, 0.0154)))
    body = body.union(_tube(0.0194, 0.0188, 0.0178).translate((0.0, 0.0, 0.0322)))
    body = body.union(_tube(0.0210, 0.0148, 0.0022).translate((0.0, 0.0, 0.0486)))
    body = body.union(_tube(0.0102, 0.0076, 0.0022).translate((0.0, 0.0, 0.0226)))
    body = body.union(_tube(0.0136, 0.0112, 0.0016).translate((0.0, 0.0, 0.0438)))
    body = body.cut(cq.Workplane("XY").circle(0.0128).extrude(0.0085))
    body = body.cut(cq.Workplane("XY").circle(0.0136).extrude(0.0130).translate((0.0, 0.0, 0.0375)))
    return body


def _build_focus_ring_shape() -> cq.Workplane:
    ring = _grooved_ring(
        outer_radius=0.0254,
        inner_radius=0.0217,
        length=0.0188,
        groove_count=28,
        groove_width=0.0017,
        groove_depth=0.0014,
        end_margin=0.0012,
    )
    focus_tab = (
        cq.Workplane("XY")
        .box(0.0080, 0.0042, 0.0032)
        .translate((0.0281, 0.0, 0.0064))
    )
    return ring.union(focus_tab)


def _build_aperture_ring_shape() -> cq.Workplane:
    ring = _grooved_ring(
        outer_radius=0.0240,
        inner_radius=0.0222,
        length=0.0072,
        groove_count=20,
        groove_width=0.0012,
        groove_depth=0.0009,
        end_margin=0.0009,
    )
    index_lug = (
        cq.Workplane("XY")
        .box(0.0038, 0.0020, 0.0018)
        .translate((0.0252, 0.0, 0.0052))
    )
    return ring.union(index_lug)


def _build_hood_shape() -> cq.Workplane:
    sleeve = _tube(0.0228, 0.0201, 0.0220)
    sleeve = sleeve.union(_tube(0.0234, 0.0201, 0.0042))
    sleeve = sleeve.union(_tube(0.0242, 0.0208, 0.0030).translate((0.0, 0.0, 0.0186)))
    return sleeve


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rangefinder_lens")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.20, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "lens_body"),
        material=anodized_black,
        name="body_shell",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_cadquery(_build_aperture_ring_shape(), "aperture_ring"),
        material=graphite,
        name="aperture_band",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_build_focus_ring_shape(), "focus_ring"),
        material=satin_black,
        name="focus_band",
    )

    hood_sleeve = model.part("hood_sleeve")
    hood_sleeve.visual(
        mesh_from_cadquery(_build_hood_shape(), "hood_sleeve"),
        material=graphite,
        name="hood_shell",
    )

    model.articulation(
        "aperture_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0158)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.5),
    )
    model.articulation(
        "hood_extension",
        ArticulationType.PRISMATIC,
        parent=body,
        child=hood_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.0332)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_ring = object_model.get_part("focus_ring")
    hood_sleeve = object_model.get_part("hood_sleeve")

    focus_rotation = object_model.get_articulation("focus_rotation")
    aperture_rotation = object_model.get_articulation("aperture_rotation")
    hood_extension = object_model.get_articulation("hood_extension")

    ctx.allow_overlap(
        body,
        aperture_ring,
        elem_a="body_shell",
        elem_b="aperture_band",
        reason="The aperture ring is intentionally modeled as a close-fitting rotating sleeve around the rear barrel band.",
    )
    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="body_shell",
        elem_b="focus_band",
        reason="The focus ring is intentionally represented as a concentric rotating sleeve nested over the barrel core.",
    )
    ctx.allow_overlap(
        body,
        hood_sleeve,
        elem_a="body_shell",
        elem_b="hood_shell",
        reason="The pull-out hood is intentionally represented as a telescoping sleeve nested over the front barrel.",
    )

    ctx.expect_origin_distance(
        aperture_ring,
        body,
        axes="xy",
        max_dist=1e-6,
        name="aperture ring stays coaxial at rest",
    )
    ctx.expect_overlap(
        aperture_ring,
        body,
        axes="z",
        min_overlap=0.006,
        name="aperture ring sits on the rear barrel band",
    )
    ctx.expect_origin_distance(
        focus_ring,
        body,
        axes="xy",
        max_dist=1e-6,
        name="focus ring stays coaxial at rest",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="z",
        min_overlap=0.016,
        name="focus ring spans the mid barrel band",
    )
    ctx.expect_origin_distance(
        hood_sleeve,
        body,
        axes="xy",
        max_dist=1e-6,
        name="hood sleeve stays centered on the front barrel",
    )
    ctx.expect_overlap(
        hood_sleeve,
        body,
        axes="z",
        min_overlap=0.016,
        name="hood sleeve remains nested when collapsed",
    )

    with ctx.pose({focus_rotation: 1.1, aperture_rotation: -0.7}):
        ctx.expect_origin_distance(
            focus_ring,
            body,
            axes="xy",
            max_dist=1e-6,
            name="focus ring remains coaxial while rotated",
        )
        ctx.expect_origin_distance(
            aperture_ring,
            body,
            axes="xy",
            max_dist=1e-6,
            name="aperture ring remains coaxial while rotated",
        )

    hood_rest = ctx.part_world_position(hood_sleeve)
    with ctx.pose({hood_extension: 0.010}):
        ctx.expect_overlap(
            hood_sleeve,
            body,
            axes="z",
            min_overlap=0.007,
            name="extended hood sleeve keeps retained insertion",
        )
        hood_extended = ctx.part_world_position(hood_sleeve)

    ctx.check(
        "hood sleeve extends forward",
        hood_rest is not None
        and hood_extended is not None
        and hood_extended[2] > hood_rest[2] + 0.008,
        details=f"rest={hood_rest}, extended={hood_extended}",
    )

    return ctx.report()


object_model = build_object_model()
