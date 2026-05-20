from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _annulus(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """Flat ring/washer in local XY, centered through local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_ring_light_mirror")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    satin_black = model.material("satin_black", rgba=(0.04, 0.04, 0.038, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.12, 0.13, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.91, 0.72, 0.96))
    mirror_glass = model.material("mirror_glass", rgba=(0.62, 0.82, 0.95, 0.70))

    # Root: weighted table base plus the fixed outer tube of the telescoping mast.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.125, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=matte_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=satin_black,
        name="base_boss",
    )
    base.visual(
        mesh_from_cadquery(_annulus(0.024, 0.017, 0.560), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=dark_metal,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_annulus(0.031, 0.0165, 0.052), "top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.594)),
        material=satin_black,
        name="top_collar",
    )
    base.visual(
        Box((0.013, 0.010, 0.030)),
        origin=Origin(xyz=(0.0205, 0.0, 0.595)),
        material=satin_black,
        name="guide_pad_0",
    )
    base.visual(
        Box((0.013, 0.010, 0.030)),
        origin=Origin(xyz=(-0.0205, 0.0, 0.595)),
        material=satin_black,
        name="guide_pad_1",
    )

    # Sliding child: the inner post, top clamp/bracket, fixed ring light, and
    # the parent-side yoke for the mirror hinge.
    post = model.part("post")
    post.visual(
        Cylinder(radius=0.014, length=0.990),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark_metal,
        name="inner_post",
    )
    post.visual(
        Box((0.070, 0.112, 0.050)),
        origin=Origin(xyz=(0.0, -0.045, 0.655)),
        material=satin_black,
        name="top_bracket",
    )

    # The ring light is a flat vertical annulus: a black rear housing with a
    # warm translucent diffuser set into the front face.
    ring_origin = Origin(xyz=(0.0, -0.085, 0.470), rpy=(math.pi / 2.0, 0.0, 0.0))
    post.visual(
        mesh_from_cadquery(_annulus(0.205, 0.128, 0.026), "ring_housing"),
        origin=ring_origin,
        material=matte_black,
        name="ring_housing",
    )
    diffuser = _annulus(0.184, 0.145, 0.006).translate((0.0, 0.0, 0.015))
    post.visual(
        mesh_from_cadquery(diffuser, "light_diffuser"),
        origin=ring_origin,
        material=warm_diffuser,
        name="light_diffuser",
    )

    # Lower yoke bridge hangs from the ring's bottom edge and frames the mirror hinge.
    post.visual(
        Box((0.125, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.085, 0.272)),
        material=satin_black,
        name="hinge_bridge",
    )
    post.visual(
        Box((0.018, 0.014, 0.058)),
        origin=Origin(xyz=(-0.059, -0.085, 0.242)),
        material=satin_black,
        name="hinge_ear_0",
    )
    post.visual(
        Box((0.018, 0.014, 0.058)),
        origin=Origin(xyz=(0.059, -0.085, 0.242)),
        material=satin_black,
        name="hinge_ear_1",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.250),
    )

    mirror = model.part("mirror")
    mirror.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_pin",
    )
    mirror.visual(
        Box((0.082, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_black,
        name="hinge_leaf",
    )
    frame_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.150, 0.130, 0.018, corner_segments=8),
            0.012,
            center=True,
        ),
        "mirror_frame",
    )
    mirror.visual(
        frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.077), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="mirror_frame",
    )
    glass_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.116, 0.092, 0.012, corner_segments=8),
            0.003,
            center=True,
        ),
        "mirror_glass",
    )
    mirror.visual(
        glass_mesh,
        origin=Origin(xyz=(0.0, -0.0075, -0.077), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mirror_glass,
        name="mirror_glass",
    )

    model.articulation(
        "post_to_mirror",
        ArticulationType.REVOLUTE,
        parent=post,
        child=mirror,
        origin=Origin(xyz=(0.0, -0.085, 0.222)),
        # Positive motion swings the hanging mirror forward, away from the post.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    mirror = object_model.get_part("mirror")
    slide = object_model.get_articulation("sleeve_to_post")
    hinge = object_model.get_articulation("post_to_mirror")

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            post,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="inner post is centered in the outer sleeve",
        )
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.30,
            name="collapsed post remains deeply inserted",
        )
        ctx.expect_gap(
            post,
            mirror,
            axis="z",
            positive_elem="ring_housing",
            negative_elem="mirror_frame",
            min_gap=0.020,
            max_gap=0.090,
            name="mirror panel hangs below the ring",
        )
        rest_pos = ctx.part_world_position(post)

    with ctx.pose({slide: 0.250}):
        ctx.expect_within(
            post,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="extended post stays centered in the sleeve",
        )
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.070,
            name="extended post keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(post)

    ctx.check(
        "center post slides upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(mirror)
    with ctx.pose({hinge: 0.95}):
        tilted_aabb = ctx.part_world_aabb(mirror)

    ctx.check(
        "mirror hinge swings panel forward",
        closed_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][1] < closed_aabb[0][1] - 0.045,
        details=f"closed={closed_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
