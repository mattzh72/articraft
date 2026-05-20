from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.23
BASE_HEIGHT = 0.11
SLEEVE_RADIUS = 0.112
CAVITY_LENGTH = 0.076
CAVITY_WIDTH = 0.094
CAVITY_DEPTH = 0.040
CAVITY_CENTER_X = 0.165

POST_RADIUS = 0.102
POST_BODY_LENGTH = 0.98
POST_CROWN_HEIGHT = 0.020
POST_CROWN_RADIUS = 0.120
POST_COLLAPSED_TOP = 0.020
POST_TRAVEL = 0.78

COVER_LENGTH = 0.076
COVER_WIDTH = 0.094
COVER_THICKNESS = 0.008
COVER_BARREL_RADIUS = 0.004
COVER_BARREL_LENGTH = 0.070

KNOB_STEM_RADIUS = 0.007
KNOB_STEM_HEIGHT = 0.014
KNOB_HEAD_RADIUS = 0.026
KNOB_HEAD_HEIGHT = 0.012
KNOB_GRIP_LENGTH = 0.042
KNOB_GRIP_WIDTH = 0.012
KNOB_GRIP_HEIGHT = 0.006


def _make_base_collar_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    sleeve_cut = cq.Workplane("XY").circle(SLEEVE_RADIUS).extrude(BASE_HEIGHT)
    cavity_cut = (
        cq.Workplane("XY")
        .box(CAVITY_LENGTH, CAVITY_WIDTH, CAVITY_DEPTH)
        .translate((CAVITY_CENTER_X, 0.0, BASE_HEIGHT - (CAVITY_DEPTH / 2.0)))
    )
    return collar.cut(sleeve_cut).cut(cavity_cut)


def _make_post_shape() -> cq.Workplane:
    post = cq.Workplane("XY").circle(POST_RADIUS).extrude(POST_BODY_LENGTH)
    crown = (
        cq.Workplane("XY")
        .circle(POST_CROWN_RADIUS)
        .extrude(POST_CROWN_HEIGHT)
        .translate((0.0, 0.0, POST_BODY_LENGTH))
    )
    total_height = POST_BODY_LENGTH + POST_CROWN_HEIGHT
    return post.union(crown).translate((0.0, 0.0, POST_COLLAPSED_TOP - total_height))


def _make_cover_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(COVER_LENGTH, COVER_WIDTH, COVER_THICKNESS)
        .translate((COVER_LENGTH / 2.0, 0.0, COVER_BARREL_RADIUS))
    )
    barrel = (
        cq.Workplane("XZ")
        .circle(COVER_BARREL_RADIUS)
        .extrude(COVER_BARREL_LENGTH)
        .translate((0.0, -(COVER_BARREL_LENGTH / 2.0), COVER_BARREL_RADIUS))
    )
    return plate.union(barrel)


def _make_release_knob_shape() -> cq.Workplane:
    stem = cq.Workplane("XY").circle(KNOB_STEM_RADIUS).extrude(KNOB_STEM_HEIGHT)
    head = (
        cq.Workplane("XY")
        .circle(KNOB_HEAD_RADIUS)
        .extrude(KNOB_HEAD_HEIGHT)
        .translate((0.0, 0.0, KNOB_STEM_HEIGHT))
    )
    grip = (
        cq.Workplane("XY")
        .box(KNOB_GRIP_LENGTH, KNOB_GRIP_WIDTH, KNOB_GRIP_HEIGHT)
        .translate(
            (
                0.0,
                0.0,
                KNOB_STEM_HEIGHT + KNOB_HEAD_HEIGHT + (KNOB_GRIP_HEIGHT / 2.0),
            )
        )
    )
    return stem.union(head).union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_bollard")

    model.material("brushed_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("base_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_collar")
    base.visual(
        mesh_from_cadquery(_make_base_collar_shape(), "base_collar"),
        material="base_black",
        name="collar_shell",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_make_post_shape(), "post"),
        material="brushed_metal",
        name="post_shell",
    )

    cover = model.part("cover_plate")
    cover.visual(
        mesh_from_cadquery(_make_cover_plate_shape(), "cover_plate"),
        material="base_black",
        name="cover_shell",
    )

    knob = model.part("release_knob")
    knob.visual(
        mesh_from_cadquery(_make_release_knob_shape(), "release_knob"),
        material="knob_black",
        name="knob_shell",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=POST_TRAVEL,
            effort=2000.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cover,
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X - (CAVITY_LENGTH / 2.0),
                0.0,
                BASE_HEIGHT,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.00,
            effort=15.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, POST_COLLAPSED_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_collar")
    post = object_model.get_part("post")
    cover = object_model.get_part("cover_plate")
    knob = object_model.get_part("release_knob")
    slide = object_model.get_articulation("post_slide")
    cover_hinge = object_model.get_articulation("cover_hinge")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.allow_overlap(
        base,
        post,
        elem_a="collar_shell",
        elem_b="post_shell",
        reason=(
            "The bollard post is intentionally represented as remaining seated inside "
            "the buried guide sleeve of the collar at rest."
        ),
    )

    ctx.expect_origin_distance(
        post,
        base,
        axes="xy",
        max_dist=0.001,
        name="post stays centered over the sleeve axis",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="xy",
        min_overlap=0.20,
        name="collapsed post remains aligned with the collar opening",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: POST_TRAVEL}):
        ctx.expect_overlap(
            post,
            base,
            axes="xy",
            min_overlap=0.20,
            name="raised post remains coaxial with the collar",
        )
        extended_pos = ctx.part_world_position(post)

    ctx.check(
        "post raises to barrier height",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        cover,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed cover sits flush on the collar deck",
    )
    ctx.expect_overlap(
        cover,
        base,
        axes="xy",
        min_overlap=0.05,
        name="closed cover stays over the actuator pocket",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 0.95}):
        opened_cover_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "cover plate pivots upward over the cavity",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.05,
        details=f"closed={closed_cover_aabb}, open={opened_cover_aabb}",
    )

    rest_knob_aabb = ctx.part_world_aabb(knob)
    with ctx.pose({knob_spin: pi / 2.0}):
        turned_knob_aabb = ctx.part_world_aabb(knob)

    rest_knob_dx = (
        (rest_knob_aabb[1][0] - rest_knob_aabb[0][0]) if rest_knob_aabb is not None else None
    )
    rest_knob_dy = (
        (rest_knob_aabb[1][1] - rest_knob_aabb[0][1]) if rest_knob_aabb is not None else None
    )
    turned_knob_dx = (
        (turned_knob_aabb[1][0] - turned_knob_aabb[0][0]) if turned_knob_aabb is not None else None
    )
    turned_knob_dy = (
        (turned_knob_aabb[1][1] - turned_knob_aabb[0][1]) if turned_knob_aabb is not None else None
    )
    ctx.check(
        "release knob spins about its short shaft",
        rest_knob_dx is not None
        and rest_knob_dy is not None
        and turned_knob_dx is not None
        and turned_knob_dy is not None
        and rest_knob_dx > rest_knob_dy
        and turned_knob_dy > turned_knob_dx,
        details=(
            f"rest=({rest_knob_dx}, {rest_knob_dy}), "
            f"turned=({turned_knob_dx}, {turned_knob_dy})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
