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


def _make_sleeve_shell() -> cq.Workplane:
    sleeve_outer = (
        cq.Workplane("XY").circle(0.145).extrude(0.58).translate((0.0, 0.0, -0.58))
    )
    collar_outer = (
        cq.Workplane("XY")
        .circle(0.165)
        .workplane(offset=0.07)
        .circle(0.150)
        .loft(combine=True)
    )
    outer_shell = sleeve_outer.union(collar_outer)

    bore = (
        cq.Workplane("XY").circle(0.111).extrude(0.625).translate((0.0, 0.0, -0.58))
    )
    lead_in = (
        cq.Workplane("XY")
        .workplane(offset=0.045)
        .circle(0.111)
        .workplane(offset=0.025)
        .circle(0.116)
        .loft(combine=True)
    )

    return outer_shell.cut(bore.union(lead_in))


def _make_post_body() -> cq.Workplane:
    shaft = (
        cq.Workplane("XY").circle(0.104).extrude(0.91).translate((0.0, 0.0, -0.20))
    )
    guide_boss = (
        cq.Workplane("XY").circle(0.010).extrude(0.06).translate((0.101, 0.0, -0.16))
    )
    guide_boss_120 = guide_boss.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 120.0)
    guide_boss_240 = guide_boss.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 240.0)
    crown = (
        cq.Workplane("XY")
        .workplane(offset=0.71)
        .circle(0.104)
        .workplane(offset=0.04)
        .circle(0.098)
        .loft(combine=True)
    )
    return shaft.union(guide_boss).union(guide_boss_120).union(guide_boss_240).union(crown)


def _make_latch_cover() -> cq.Workplane:
    cover_plate = (
        cq.Workplane("XY").box(0.044, 0.056, 0.010).translate((-0.022, 0.0, -0.001))
    )
    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(0.072)
        .translate((0.0, 0.0, -0.036))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )
    return cover_plate.union(hinge_barrel)


def _make_lock_cap() -> cq.Workplane:
    cap_base = cq.Workplane("XY").circle(0.038).extrude(0.012)
    cap_bevel = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .circle(0.038)
        .workplane(offset=0.006)
        .circle(0.032)
        .loft(combine=True)
    )
    return cap_base.union(cap_bevel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_bollard")

    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.26, 0.28, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_make_sleeve_shell(), "sleeve_shell"),
        material=graphite,
        name="sleeve_shell",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_make_post_body(), "post_body"),
        material=steel,
        name="post_body",
    )

    latch_cover = model.part("latch_cover")
    latch_cover.visual(
        mesh_from_cadquery(_make_latch_cover(), "cover_plate"),
        material=dark_trim,
        name="cover_plate",
    )

    lock_cap = model.part("lock_cap")
    lock_cap.visual(
        mesh_from_cadquery(_make_lock_cap(), "cap_disk"),
        material=dark_trim,
        name="cap_disk",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.30,
            lower=-0.35,
            upper=0.10,
        ),
    )
    model.articulation(
        "sleeve_to_latch_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=latch_cover,
        origin=Origin(xyz=(0.1705, 0.0, 0.076)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "post_to_lock_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=lock_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    latch_cover = object_model.get_part("latch_cover")
    lock_cap = object_model.get_part("lock_cap")

    post_slide = object_model.get_articulation("sleeve_to_post")
    latch_hinge = object_model.get_articulation("sleeve_to_latch_cover")
    cap_spin = object_model.get_articulation("post_to_lock_cap")

    ctx.allow_overlap(
        post,
        sleeve,
        elem_a="post_body",
        elem_b="sleeve_shell",
        reason="The bollard post is intentionally represented as a guided telescoping member running inside the sleeve bore.",
    )

    ctx.expect_origin_distance(
        post,
        sleeve,
        axes="xy",
        max_dist=0.001,
        name="post stays concentric with sleeve",
    )
    ctx.expect_gap(
        latch_cover,
        sleeve,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="sleeve_shell",
        max_penetration=1e-6,
        max_gap=0.001,
        name="latch cover rests on collar top",
    )
    ctx.expect_contact(
        lock_cap,
        post,
        elem_a="cap_disk",
        elem_b="post_body",
        contact_tol=0.0005,
        name="lock cap seats on post top",
    )
    ctx.expect_origin_distance(
        lock_cap,
        post,
        axes="xy",
        max_dist=0.001,
        name="lock cap stays centered on post",
    )

    slide_lower = post_slide.motion_limits.lower
    slide_upper = post_slide.motion_limits.upper
    lower_post_position = None
    upper_post_position = None
    if slide_lower is not None:
        with ctx.pose({post_slide: slide_lower}):
            lower_post_position = ctx.part_world_position(post)
    if slide_upper is not None:
        with ctx.pose({post_slide: slide_upper}):
            upper_post_position = ctx.part_world_position(post)
            ctx.expect_overlap(
                post,
                sleeve,
                axes="z",
                elem_a="post_body",
                elem_b="sleeve_shell",
                min_overlap=0.16,
                name="raised post remains inserted in sleeve",
            )
    ctx.check(
        "post raises upward",
        lower_post_position is not None
        and upper_post_position is not None
        and upper_post_position[2] > lower_post_position[2] + 0.40,
        details=f"lower={lower_post_position}, upper={upper_post_position}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(latch_cover, elem="cover_plate")
    cover_upper = latch_hinge.motion_limits.upper
    open_cover_aabb = None
    if cover_upper is not None:
        with ctx.pose({latch_hinge: cover_upper}):
            open_cover_aabb = ctx.part_element_world_aabb(latch_cover, elem="cover_plate")
    ctx.check(
        "latch cover opens upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.03,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    ctx.check(
        "lock cap uses continuous rotation",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS
        and cap_spin.motion_limits is not None
        and cap_spin.motion_limits.lower is None
        and cap_spin.motion_limits.upper is None,
        details=f"type={cap_spin.articulation_type}, limits={cap_spin.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
