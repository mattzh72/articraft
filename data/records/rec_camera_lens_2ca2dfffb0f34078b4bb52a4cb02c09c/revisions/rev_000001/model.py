from __future__ import annotations

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


def _tube_segment(
    start_x: float,
    end_x: float,
    *,
    outer_radius: float,
    inner_radius: float,
) -> cq.Workplane:
    length = end_x - start_x
    profile = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    if inner_radius > 0.0:
        bore = cq.Workplane("YZ").circle(inner_radius).extrude(length)
        profile = profile.cut(bore)
    return profile.translate((start_x, 0.0, 0.0))


def _union_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _grip_ring(
    *,
    length: float,
    inner_radius: float,
    sleeve_radius: float,
    rib_radius: float,
    rib_count: int,
) -> cq.Workplane:
    start_x = -length * 0.5
    end_x = length * 0.5
    shapes = [
        _tube_segment(
            start_x,
            end_x,
            outer_radius=sleeve_radius,
            inner_radius=inner_radius,
        )
    ]

    edge_margin = 0.0016
    pitch = (length - 2.0 * edge_margin) / rib_count
    rib_width = pitch * 0.52

    for index in range(rib_count):
        rib_start = start_x + edge_margin + index * pitch + 0.5 * (pitch - rib_width)
        shapes.append(
            _tube_segment(
                rib_start,
                rib_start + rib_width,
                outer_radius=rib_radius,
                inner_radius=inner_radius,
            )
        )

    return _union_shapes(*shapes)


def _outer_barrel_body() -> cq.Workplane:
    return _union_shapes(
        _tube_segment(0.000, 0.006, outer_radius=0.0360, inner_radius=0.0260),
        _tube_segment(0.006, 0.054, outer_radius=0.0375, inner_radius=0.0315),
        _tube_segment(0.054, 0.084, outer_radius=0.0346, inner_radius=0.0315),
        _tube_segment(0.084, 0.088, outer_radius=0.0366, inner_radius=0.0315),
        _tube_segment(0.088, 0.108, outer_radius=0.0338, inner_radius=0.0308),
    )


def _front_socket() -> cq.Workplane:
    return _tube_segment(0.106, 0.126, outer_radius=0.0354, inner_radius=0.0308)


def _inner_barrel() -> cq.Workplane:
    return _union_shapes(
        _tube_segment(-0.040, 0.020, outer_radius=0.0308, inner_radius=0.0248),
        _tube_segment(0.018, 0.028, outer_radius=0.0322, inner_radius=0.0248),
    )


def _motion_is_continuous_about_x(joint) -> bool:
    return (
        joint.articulation_type == ArticulationType.CONTINUOUS
        and joint.axis == (1.0, 0.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zoom_camera_lens")

    housing_black = model.material("housing_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        mesh_from_cadquery(_outer_barrel_body(), "lens_outer_barrel_body"),
        material=housing_black,
        name="body_shell",
    )
    outer_barrel.visual(
        mesh_from_cadquery(_front_socket(), "lens_front_socket"),
        material=satin_black,
        name="front_socket",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_cadquery(
            _grip_ring(
                length=0.026,
                inner_radius=0.0346,
                sleeve_radius=0.0394,
                rib_radius=0.0405,
                rib_count=11,
            ),
            "lens_zoom_ring",
        ),
        material=rubber_black,
        name="zoom_band",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            _grip_ring(
                length=0.018,
                inner_radius=0.0338,
                sleeve_radius=0.0385,
                rib_radius=0.0395,
                rib_count=8,
            ),
            "lens_focus_ring",
        ),
        material=rubber_black,
        name="focus_band",
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        mesh_from_cadquery(_inner_barrel(), "lens_inner_barrel"),
        material=satin_black,
        name="inner_tube",
    )

    model.articulation(
        "zoom_spin",
        ArticulationType.CONTINUOUS,
        parent=outer_barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=10.0),
    )
    model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=outer_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )
    model.articulation(
        "barrel_extension",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.032,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_barrel = object_model.get_part("outer_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    inner_barrel = object_model.get_part("inner_barrel")

    zoom_spin = object_model.get_articulation("zoom_spin")
    focus_spin = object_model.get_articulation("focus_spin")
    barrel_extension = object_model.get_articulation("barrel_extension")

    ctx.allow_overlap(
        zoom_ring,
        outer_barrel,
        elem_a="zoom_band",
        elem_b="body_shell",
        reason="The zoom ring is intentionally represented as a rotating sleeve around the fixed barrel support.",
    )
    ctx.allow_overlap(
        focus_ring,
        outer_barrel,
        elem_a="focus_band",
        elem_b="body_shell",
        reason="The focus ring is intentionally represented as a rotating sleeve around the front support barrel.",
    )
    ctx.allow_overlap(
        inner_barrel,
        outer_barrel,
        elem_a="inner_tube",
        elem_b="body_shell",
        reason="The extending inner barrel is intentionally represented as sliding inside the outer barrel support sleeve.",
    )
    ctx.allow_overlap(
        inner_barrel,
        outer_barrel,
        elem_a="inner_tube",
        elem_b="front_socket",
        reason="The extending inner barrel continues through the front socket as an intentional telescoping fit.",
    )

    ctx.expect_origin_distance(
        zoom_ring,
        outer_barrel,
        axes="yz",
        max_dist=0.0005,
        name="zoom ring remains coaxial with the barrel",
    )
    ctx.expect_origin_distance(
        focus_ring,
        outer_barrel,
        axes="yz",
        max_dist=0.0005,
        name="focus ring remains coaxial with the barrel",
    )
    ctx.expect_overlap(
        zoom_ring,
        outer_barrel,
        axes="x",
        min_overlap=0.022,
        name="zoom ring occupies a full-width zoom band",
    )
    ctx.expect_overlap(
        focus_ring,
        outer_barrel,
        axes="x",
        min_overlap=0.015,
        name="focus ring occupies a full-width focus band",
    )

    ctx.check(
        "zoom ring uses continuous rotation about the optical axis",
        _motion_is_continuous_about_x(zoom_spin),
        details=f"type={zoom_spin.articulation_type}, axis={zoom_spin.axis}, limits={zoom_spin.motion_limits}",
    )
    ctx.check(
        "focus ring uses continuous rotation about the optical axis",
        _motion_is_continuous_about_x(focus_spin),
        details=f"type={focus_spin.articulation_type}, axis={focus_spin.axis}, limits={focus_spin.motion_limits}",
    )

    ctx.expect_origin_distance(
        inner_barrel,
        outer_barrel,
        axes="yz",
        max_dist=0.0005,
        name="collapsed inner barrel stays centered on the optical axis",
    )
    ctx.expect_overlap(
        inner_barrel,
        outer_barrel,
        axes="x",
        elem_a="inner_tube",
        elem_b="front_socket",
        min_overlap=0.018,
        name="collapsed inner barrel remains inserted in the front socket",
    )

    rest_pos = ctx.part_world_position(inner_barrel)
    upper = barrel_extension.motion_limits.upper if barrel_extension.motion_limits is not None else None

    with ctx.pose({barrel_extension: upper if upper is not None else 0.0}):
        ctx.expect_origin_distance(
            inner_barrel,
            outer_barrel,
            axes="yz",
            max_dist=0.0005,
            name="extended inner barrel stays centered on the optical axis",
        )
        ctx.expect_overlap(
            inner_barrel,
            outer_barrel,
            axes="x",
            elem_a="inner_tube",
            elem_b="front_socket",
            min_overlap=0.007,
            name="extended inner barrel still retains insertion in the front socket",
        )
        extended_pos = ctx.part_world_position(inner_barrel)

    ctx.check(
        "inner barrel extends forward along the optical axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.02,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
