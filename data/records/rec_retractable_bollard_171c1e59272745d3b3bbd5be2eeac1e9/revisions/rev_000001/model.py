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


SLEEVE_DEPTH = 0.22
SLEEVE_OUTER_RADIUS = 0.060
SLEEVE_INNER_RADIUS = 0.0495
FLANGE_RADIUS = 0.078
FLANGE_THICKNESS = 0.012

POST_RADIUS = 0.0495
POST_LENGTH = 0.24
POST_REST_PROTRUSION = 0.018
POST_TRAVEL = 0.125

HOUSING_X = 0.064
HOUSING_LENGTH = 0.020
HOUSING_WIDTH = 0.016
HOUSING_DEPTH = 0.0025

COVER_LENGTH = 0.019
COVER_WIDTH = 0.015
COVER_THICKNESS = 0.0024
HINGE_RADIUS = 0.0012
HINGE_LENGTH = 0.014
HINGE_AXIS_Z = 0.0016

LOCK_CORE_RADIUS = 0.008
LOCK_CORE_THICKNESS = 0.004
LOCK_CORE_CENTER_Z = -0.0023
LOCK_CORE_SEAT_DEPTH = 0.0043


def _make_sleeve_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .cylinder(SLEEVE_DEPTH, SLEEVE_OUTER_RADIUS, centered=(True, True, False))
        .translate((0.0, 0.0, -SLEEVE_DEPTH))
    )
    flange = (
        cq.Workplane("XY")
        .cylinder(FLANGE_THICKNESS, FLANGE_RADIUS, centered=(True, True, False))
        .translate((0.0, 0.0, -FLANGE_THICKNESS))
    )

    sleeve = body.union(flange)

    bore = (
        cq.Workplane("XY")
        .cylinder(SLEEVE_DEPTH + 0.006, SLEEVE_INNER_RADIUS, centered=(True, True, False))
        .translate((0.0, 0.0, -SLEEVE_DEPTH - 0.003))
    )
    sleeve = sleeve.cut(bore)

    housing_pocket = (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_DEPTH, centered=(True, True, False))
        .translate((HOUSING_X, 0.0, -HOUSING_DEPTH))
    )
    core_pocket = (
        cq.Workplane("XY")
        .cylinder(LOCK_CORE_SEAT_DEPTH, LOCK_CORE_RADIUS, centered=(True, True, False))
        .translate((HOUSING_X, 0.0, -LOCK_CORE_SEAT_DEPTH))
    )
    sleeve = sleeve.cut(housing_pocket).cut(core_pocket)

    slot_center_z = -0.192
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        slot = (
            cq.Workplane("XY")
            .box(0.020, 0.013, 0.030, centered=(True, True, True))
            .translate((SLEEVE_OUTER_RADIUS - 0.006, 0.0, slot_center_z))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        sleeve = sleeve.cut(slot)

    top_rim_relief = (
        cq.Workplane("XY")
        .cylinder(0.004, SLEEVE_INNER_RADIUS + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -0.004))
    )
    sleeve = sleeve.cut(top_rim_relief)

    return sleeve


def _make_post_shape() -> cq.Workplane:
    post = (
        cq.Workplane("XY")
        .cylinder(POST_LENGTH, POST_RADIUS, centered=(True, True, False))
        .translate((0.0, 0.0, -(POST_LENGTH - POST_REST_PROTRUSION)))
    )
    return post.faces(">Z").edges().chamfer(0.003)


def _make_lock_core_shape() -> cq.Workplane:
    core = cq.Workplane("XY").cylinder(
        LOCK_CORE_THICKNESS,
        LOCK_CORE_RADIUS,
        centered=(True, True, True),
    )
    key_slot = (
        cq.Workplane("XY")
        .box(0.011, 0.0022, 0.0014, centered=(True, True, True))
        .translate((0.0, 0.0, 0.0010))
    )
    index_flat = (
        cq.Workplane("XY")
        .box(0.004, 0.012, LOCK_CORE_THICKNESS + 0.001, centered=(True, True, True))
        .translate((0.0065, 0.0, 0.0))
    )
    return core.cut(key_slot).cut(index_flat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_bollard")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.60, 0.62, 0.64, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.86, 0.71, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))
    lock_metal = model.material("lock_metal", rgba=(0.78, 0.79, 0.80, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_make_sleeve_shape(), "sleeve"),
        material=galvanized_steel,
        name="sleeve_shell",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_make_post_shape(), "post"),
        material=warning_yellow,
        name="post_body",
    )

    cover = model.part("lock_cover")
    cover.visual(
        Box((COVER_LENGTH, COVER_WIDTH, COVER_THICKNESS)),
        origin=Origin(xyz=(-COVER_LENGTH * 0.5, 0.0, -HINGE_AXIS_Z + COVER_THICKNESS * 0.5)),
        material=rubber_black,
        name="cover_panel",
    )
    cover.visual(
        Box((0.0022, 0.008, 0.0014)),
        origin=Origin(
            xyz=(-COVER_LENGTH + 0.0011, 0.0, -HINGE_AXIS_Z + COVER_THICKNESS - 0.0007)
        ),
        material=rubber_black,
        name="cover_lip",
    )
    cover.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").cylinder(HINGE_LENGTH, HINGE_RADIUS, centered=(True, True, True)),
            "lock_cover_hinge",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="hinge_barrel",
    )

    lock_core = model.part("lock_core")
    lock_core.visual(
        mesh_from_cadquery(_make_lock_core_shape(), "lock_core"),
        material=lock_metal,
        name="core_body",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "sleeve_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=cover,
        origin=Origin(
            xyz=(HOUSING_X + HOUSING_LENGTH * 0.5, 0.0, HINGE_AXIS_Z),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "sleeve_to_lock_core",
        ArticulationType.CONTINUOUS,
        parent=sleeve,
        child=lock_core,
        origin=Origin(xyz=(HOUSING_X, 0.0, LOCK_CORE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cover = object_model.get_part("lock_cover")
    lock_core = object_model.get_part("lock_core")

    slide = object_model.get_articulation("sleeve_to_post")
    cover_hinge = object_model.get_articulation("sleeve_to_lock_cover")
    core_spin = object_model.get_articulation("sleeve_to_lock_core")

    ctx.allow_overlap(
        sleeve,
        post,
        elem_a="sleeve_shell",
        elem_b="post_body",
        reason=(
            "The post is intentionally represented sliding inside the hollow sleeve; "
            "the mesh-backed sleeve shell reads with a real internal cavity, but the "
            "compiler-owned overlap stack treats the enclosing mesh volume as solid."
        ),
    )

    ctx.expect_origin_distance(
        post,
        sleeve,
        axes="xy",
        max_dist=0.0005,
        name="post stays coaxial with sleeve",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            cover,
            sleeve,
            axis="z",
            min_gap=0.0,
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lock cover sits near flush on the top face",
        )
        sleeve_aabb = ctx.part_world_aabb(sleeve)
        rest_post_aabb = ctx.part_world_aabb(post)

    sleeve_top = None if sleeve_aabb is None else sleeve_aabb[1][2]
    rest_post_top = None if rest_post_aabb is None else rest_post_aabb[1][2]
    ctx.check(
        "collapsed post remains only slightly proud of the flange",
        sleeve_top is not None
        and rest_post_top is not None
        and 0.014 <= (rest_post_top - sleeve_top) <= 0.022,
        details=f"sleeve_top={sleeve_top}, rest_post_top={rest_post_top}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({cover_hinge: math.radians(70.0)}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")

    closed_cover_top = None if closed_cover_aabb is None else closed_cover_aabb[1][2]
    open_cover_top = None if open_cover_aabb is None else open_cover_aabb[1][2]
    ctx.check(
        "lock cover lifts clear of the sleeve",
        closed_cover_top is not None
        and open_cover_top is not None
        and open_cover_top > closed_cover_top + 0.012,
        details=f"closed_top={closed_cover_top}, open_top={open_cover_top}",
    )

    with ctx.pose({slide: POST_TRAVEL}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            min_overlap=0.095,
            name="extended post retains insertion inside the sleeve",
        )
        extended_post_aabb = ctx.part_world_aabb(post)

    extended_post_top = None if extended_post_aabb is None else extended_post_aabb[1][2]
    ctx.check(
        "extended post rises to a short bollard height",
        sleeve_top is not None
        and extended_post_top is not None
        and 0.138 <= (extended_post_top - sleeve_top) <= 0.146,
        details=f"sleeve_top={sleeve_top}, extended_post_top={extended_post_top}",
    )

    core_rest = ctx.part_world_position(lock_core)
    with ctx.pose({core_spin: 1.4}):
        core_turned = ctx.part_world_position(lock_core)

    core_limits = core_spin.motion_limits
    ctx.check(
        "lock core uses continuous vertical rotation",
        core_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(core_spin.axis) == (0.0, 0.0, 1.0)
        and core_limits is not None
        and core_limits.lower is None
        and core_limits.upper is None
        and core_rest == core_turned,
        details=(
            f"type={core_spin.articulation_type}, axis={core_spin.axis}, "
            f"limits={core_limits}, rest={core_rest}, turned={core_turned}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
