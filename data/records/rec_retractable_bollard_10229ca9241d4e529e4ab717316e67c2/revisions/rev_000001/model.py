from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POST_OUTER_RADIUS = 0.0635
POST_INNER_RADIUS = 0.0555
POST_BOTTOM_Z = -0.24
POST_TOP_Z = 0.92

SLEEVE_INNER_RADIUS = 0.0670
SLEEVE_OUTER_RADIUS = 0.0820
SLEEVE_DEPTH = 1.18
SLEEVE_FLANGE_RADIUS = 0.110
SLEEVE_FLANGE_THICKNESS = 0.018
SLEEVE_GUIDE_OUTER_RADIUS = 0.074

CAP_PLUG_RADIUS = POST_INNER_RADIUS
CAP_PLUG_LENGTH = 0.048
CAP_FLANGE_RADIUS = 0.074
CAP_FLANGE_THICKNESS = 0.018
CAP_GAP_TO_POST = 0.002
LOCK_HEAD_RADIUS = 0.023
LOCK_HEAD_HEIGHT = 0.020

POST_TRAVEL = 0.93


def _build_sleeve_shape() -> cq.Workplane:
    socket = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_DEPTH)
        .translate((0.0, 0.0, -SLEEVE_DEPTH))
    )
    flange = (
        cq.Workplane("XY")
        .circle(SLEEVE_FLANGE_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_FLANGE_THICKNESS)
    )
    guide_collar = (
        cq.Workplane("XY")
        .circle(SLEEVE_GUIDE_OUTER_RADIUS)
        .circle(POST_OUTER_RADIUS)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.060))
    )
    return socket.union(flange).union(guide_collar)


def _build_post_tube() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(POST_OUTER_RADIUS)
        .circle(POST_INNER_RADIUS)
        .extrude(POST_TOP_Z - POST_BOTTOM_Z)
        .translate((0.0, 0.0, POST_BOTTOM_Z))
    )


def _build_lock_head() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(LOCK_HEAD_RADIUS)
        .extrude(LOCK_HEAD_HEIGHT)
        .faces(">Z")
        .workplane()
        .rect(0.018, 0.004)
        .cutBlind(-0.0045)
        .translate((0.0, 0.0, -LOCK_HEAD_HEIGHT * 0.5))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_bollard")

    sleeve_finish = model.material("sleeve_finish", rgba=(0.25, 0.27, 0.29, 1.0))
    post_finish = model.material("post_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    cap_finish = model.material("cap_finish", rgba=(0.47, 0.49, 0.51, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_build_sleeve_shape(), "sleeve_socket"),
        material=sleeve_finish,
        name="sleeve_socket",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_build_post_tube(), "post_tube"),
        material=post_finish,
        name="post_tube",
    )

    crown_cap = model.part("crown_cap")
    crown_cap.visual(
        Cylinder(radius=CAP_PLUG_RADIUS, length=CAP_PLUG_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * CAP_PLUG_LENGTH + CAP_GAP_TO_POST)),
        material=cap_finish,
        name="cap_plug",
    )
    crown_cap.visual(
        Cylinder(radius=CAP_FLANGE_RADIUS, length=CAP_FLANGE_THICKNESS),
        origin=Origin(
            xyz=(0.0, 0.0, CAP_GAP_TO_POST + 0.5 * CAP_FLANGE_THICKNESS),
        ),
        material=cap_finish,
        name="crown_flange",
    )
    crown_cap.visual(
        mesh_from_cadquery(_build_lock_head(), "crown_lock_head"),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CAP_GAP_TO_POST + CAP_FLANGE_THICKNESS + 0.5 * LOCK_HEAD_HEIGHT,
            ),
        ),
        material=cap_finish,
        name="lock_head",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "post_to_crown_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=crown_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    crown_cap = object_model.get_part("crown_cap")
    slide = object_model.get_articulation("sleeve_to_post")
    spin = object_model.get_articulation("post_to_crown_cap")
    slide_limits = slide.motion_limits

    ctx.allow_overlap(
        sleeve,
        post,
        elem_a="sleeve_socket",
        elem_b="post_tube",
        reason="The bollard post is intentionally modeled as nested inside the underground receiving sleeve; the current compiler treats the sleeve receiver as a solid overlap proxy.",
    )
    ctx.allow_overlap(
        crown_cap,
        post,
        elem_a="cap_plug",
        elem_b="post_tube",
        reason="The keyed crown cap is retained by an internal plug seated inside the hollow post crown; the current compiler treats the tube receiver as a solid overlap proxy.",
    )

    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_socket",
        min_overlap=0.22,
        name="raised post remains inserted in the sleeve",
    )
    ctx.expect_gap(
        crown_cap,
        post,
        axis="z",
        positive_elem="crown_flange",
        negative_elem="post_tube",
        min_gap=0.0015,
        max_gap=0.0035,
        name="crown cap stays visibly separate from the post tube",
    )
    ctx.expect_overlap(
        crown_cap,
        post,
        axes="xy",
        elem_a="crown_flange",
        elem_b="post_tube",
        min_overlap=0.12,
        name="crown cap remains centered over the post crown",
    )

    rest_post_pos = ctx.part_world_position(post)
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide: slide_limits.upper}):
            lowered_post_pos = ctx.part_world_position(post)
            ctx.expect_overlap(
                post,
                sleeve,
                axes="z",
                elem_a="post_tube",
                elem_b="sleeve_socket",
                min_overlap=1.10,
                name="lowered post remains retained inside the sleeve",
            )
        ctx.check(
            "post retracts downward into the sleeve",
            rest_post_pos is not None
            and lowered_post_pos is not None
            and lowered_post_pos[2] < rest_post_pos[2] - 0.85,
            details=f"rest={rest_post_pos}, lowered={lowered_post_pos}",
        )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            crown_cap,
            post,
            axis="z",
            positive_elem="crown_flange",
            negative_elem="post_tube",
            min_gap=0.0015,
            max_gap=0.0035,
            name="rotated crown cap still sits cleanly above the post tube",
        )
        ctx.expect_overlap(
            crown_cap,
            post,
            axes="xy",
            elem_a="crown_flange",
            elem_b="post_tube",
            min_overlap=0.12,
            name="rotated crown cap stays on the post axis",
        )

    return ctx.report()


object_model = build_object_model()
