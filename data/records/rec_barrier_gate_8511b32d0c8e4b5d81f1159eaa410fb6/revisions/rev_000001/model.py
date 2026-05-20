from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_post_barrier")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.86, 0.88, 1.0))
    dark_steel = model.material("dark_socket_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    concrete_mat = model.material("brushed_concrete", rgba=(0.46, 0.45, 0.42, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    socket = model.part("socket")

    ground_cut = cq.Workplane("XY").circle(0.116).extrude(0.10).translate((0.0, 0.0, -0.05))
    ground_slab = (
        cq.Workplane("XY")
        .box(0.62, 0.62, 0.050)
        .translate((0.0, 0.0, -0.025))
        .cut(ground_cut)
        .edges("|Z")
        .fillet(0.006)
    )
    socket.visual(
        mesh_from_cadquery(ground_slab, "ground_slab", tolerance=0.0008),
        material=concrete_mat,
        name="ground_slab",
    )

    flush_rim = (
        cq.Workplane("XY")
        .circle(0.116)
        .circle(0.058)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.012))
        .edges(">Z")
        .fillet(0.003)
    )
    socket.visual(
        mesh_from_cadquery(flush_rim, "flush_rim", tolerance=0.0005),
        material=chrome,
        name="flush_rim",
    )

    sleeve_wall = (
        cq.Workplane("XY")
        .circle(0.072)
        .circle(0.056)
        .extrude(1.020)
        .translate((0.0, 0.0, -1.020))
    )
    socket.visual(
        mesh_from_cadquery(sleeve_wall, "sleeve_wall", tolerance=0.0008),
        material=dark_steel,
        name="sleeve_wall",
    )

    socket.visual(
        Cylinder(radius=0.072, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -1.021)),
        material=dark_steel,
        name="socket_floor",
    )

    guide_bushing = (
        cq.Workplane("XY")
        .circle(0.057)
        .circle(0.045)
        .extrude(0.020)
        .translate((0.0, 0.0, -0.030))
    )
    socket.visual(
        mesh_from_cadquery(guide_bushing, "guide_bushing", tolerance=0.0005),
        material=rubber,
        name="guide_bushing",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.045, length=1.000),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=chrome,
        name="post_tube",
    )
    post.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.770)),
        material=chrome,
        name="rounded_cap",
    )
    post.visual(
        Cylinder(radius=0.047, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=chrome,
        name="retaining_foot",
    )

    model.articulation(
        "socket_to_post",
        ArticulationType.PRISMATIC,
        parent=socket,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.740, upper=0.0, effort=450.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket")
    post = object_model.get_part("post")
    slide = object_model.get_articulation("socket_to_post")

    ctx.allow_overlap(
        post,
        socket,
        elem_a="post_tube",
        elem_b="guide_bushing",
        reason=(
            "The black polymer guide bushing is intentionally modeled as a slight "
            "interference fit around the chrome post so the sliding bollard reads "
            "as captured and supported rather than floating in the socket."
        ),
    )
    ctx.expect_overlap(
        post,
        socket,
        axes="z",
        elem_a="post_tube",
        elem_b="guide_bushing",
        min_overlap=0.015,
        name="guide bushing grips the sliding post",
    )
    ctx.expect_within(
        post,
        socket,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="sleeve_wall",
        margin=0.0,
        name="post remains centered in socket bore",
    )
    ctx.expect_overlap(
        post,
        socket,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_wall",
        min_overlap=0.20,
        name="extended post keeps hidden retained insertion",
    )

    extended_aabb = ctx.part_element_world_aabb(post, elem="rounded_cap")
    extended_pos = ctx.part_world_position(post)
    with ctx.pose({slide: -0.740}):
        ctx.expect_within(
            post,
            socket,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="sleeve_wall",
            margin=0.0,
            name="retracted post remains centered in socket bore",
        )
        ctx.expect_overlap(
            post,
            socket,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_wall",
            min_overlap=0.90,
            name="retracted post nests down inside socket",
        )
        retracted_aabb = ctx.part_element_world_aabb(post, elem="rounded_cap")
        retracted_pos = ctx.part_world_position(post)

    ctx.check(
        "prismatic travel lowers the post into the ground socket",
        extended_pos is not None
        and retracted_pos is not None
        and retracted_pos[2] < extended_pos[2] - 0.70,
        details=f"extended={extended_pos}, retracted={retracted_pos}",
    )
    ctx.check(
        "deployed post stands above the flush socket",
        extended_aabb is not None and extended_aabb[1][2] > 0.78,
        details=f"rounded_cap_aabb={extended_aabb}",
    )
    ctx.check(
        "retracted post finishes nearly flush with the ground rim",
        retracted_aabb is not None and -0.01 <= retracted_aabb[1][2] <= 0.08,
        details=f"rounded_cap_aabb={retracted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
