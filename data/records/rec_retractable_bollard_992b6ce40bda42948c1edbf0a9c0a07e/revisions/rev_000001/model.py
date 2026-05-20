from __future__ import annotations

import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SLEEVE_TOP_Z = 0.018
POST_TRAVEL = 0.280
POST_RADIUS = 0.086
SLEEVE_INNER_RADIUS = 0.097
SLEEVE_OUTER_RADIUS = 0.115
POST_LENGTH = 1.320
POST_CENTER_Z = 0.160


def _sleeve_shell_mesh():
    sleeve_wall = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(0.410)
        .translate((0.0, 0.0, -0.392))
    )
    rim_ring = (
        cq.Workplane("XY")
        .circle(0.136)
        .circle(0.094)
        .extrude(0.018)
    )
    base_flange = (
        cq.Workplane("XY")
        .circle(0.126)
        .circle(0.100)
        .extrude(0.045)
        .translate((0.0, 0.0, -0.392))
    )
    return mesh_from_cadquery(sleeve_wall.union(rim_ring).union(base_flange), "bollard_sleeve_shell")


def _sidewalk_mesh():
    slab = (
        cq.Workplane("XY")
        .box(0.900, 0.900, 0.120)
        .translate((0.0, 0.0, -0.060))
    )
    opening = (
        cq.Workplane("XY")
        .circle(0.112)
        .extrude(0.160)
        .translate((0.0, 0.0, -0.120))
    )
    return mesh_from_cadquery(slab.cut(opening), "bollard_sidewalk")


def _lock_cover_mesh():
    plate = (
        cq.Workplane("XY")
        .box(0.044, 0.028, 0.004)
        .translate((-0.022, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.0012)
    )
    crown = (
        cq.Workplane("XY")
        .box(0.028, 0.018, 0.0025)
        .translate((-0.024, 0.0, 0.0032))
        .edges("|Z")
        .fillet(0.0009)
    )
    return mesh_from_cadquery(plate.union(crown), "bollard_lock_cover")


def _warning_cover_mesh():
    plate = (
        cq.Workplane("XY")
        .box(0.110, 0.070, 0.006)
        .translate((-0.055, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.002)
    )
    crown = (
        cq.Workplane("XY")
        .box(0.074, 0.044, 0.004)
        .translate((-0.061, 0.0, 0.005))
        .edges("|Z")
        .fillet(0.0015)
    )
    return mesh_from_cadquery(plate.union(crown), "bollard_warning_cover")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="removable_access_bollard")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.64, 1.0))
    steel = model.material("steel", rgba=(0.32, 0.34, 0.36, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.72, 0.12, 1.0))
    reflector = model.material("reflector", rgba=(0.92, 0.94, 0.96, 1.0))
    amber = model.material("amber", rgba=(0.93, 0.63, 0.18, 0.95))

    sleeve = model.part("sleeve")
    sleeve.visual(
        _sidewalk_mesh(),
        material=concrete,
        name="sidewalk",
    )
    sleeve.visual(
        _sleeve_shell_mesh(),
        material=steel,
        name="sleeve_shell",
    )
    sleeve.visual(
        Box((0.054, 0.050, 0.022)),
        origin=Origin(xyz=(0.149, 0.0, 0.011)),
        material=graphite,
        name="lock_boss",
    )
    sleeve.visual(
        Box((0.014, 0.036, 0.014)),
        origin=Origin(xyz=(0.176, 0.0, 0.015)),
        material=steel,
        name="lock_hinge_block",
    )
    sleeve.visual(
        Cylinder(radius=0.0038, length=0.009),
        origin=Origin(
            xyz=(0.180, -0.0115, 0.024),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="lock_hinge_knuckle_0",
    )
    sleeve.visual(
        Cylinder(radius=0.0038, length=0.009),
        origin=Origin(
            xyz=(0.180, 0.0115, 0.024),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="lock_hinge_knuckle_1",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=safety_yellow,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=0.111, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=graphite,
        name="seat_collar",
    )
    post.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        material=safety_yellow,
        name="crown_collar",
    )
    post.visual(
        Cylinder(radius=0.088, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.885)),
        material=graphite,
        name="crown_cap",
    )
    post.visual(
        Cylinder(radius=0.0885, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=reflector,
        name="reflective_band",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.898)),
        material=amber,
        name="warning_lens_base",
    )
    post.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.906)),
        material=amber,
        name="warning_lens",
    )
    post.visual(
        Box((0.026, 0.036, 0.064)),
        origin=Origin(xyz=(0.077, 0.0, 0.900)),
        material=graphite,
        name="crown_hinge_block",
    )
    post.visual(
        Cylinder(radius=0.0038, length=0.009),
        origin=Origin(
            xyz=(0.086, -0.0115, 0.935),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="crown_hinge_knuckle_0",
    )
    post.visual(
        Cylinder(radius=0.0038, length=0.009),
        origin=Origin(
            xyz=(0.086, 0.0115, 0.935),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="crown_hinge_knuckle_1",
    )

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        _lock_cover_mesh(),
        material=graphite,
        name="lock_flap",
    )
    lock_cover.visual(
        Box((0.004, 0.022, 0.010)),
        origin=Origin(xyz=(-0.041, 0.0, 0.005)),
        material=steel,
        name="lock_lip",
    )
    lock_cover.visual(
        Cylinder(radius=0.0034, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lock_hinge_knuckle",
    )

    warning_cover = model.part("warning_cover")
    warning_cover.visual(
        _warning_cover_mesh(),
        material=graphite,
        name="warning_flap",
    )
    warning_cover.visual(
        Box((0.006, 0.030, 0.010)),
        origin=Origin(xyz=(-0.106, 0.0, 0.005)),
        material=steel,
        name="warning_lip",
    )
    warning_cover.visual(
        Cylinder(radius=0.0034, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="warning_hinge_knuckle",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.20,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "sleeve_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=lock_cover,
        origin=Origin(xyz=(0.180, 0.0, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "post_to_warning_cover",
        ArticulationType.REVOLUTE,
        parent=post,
        child=warning_cover,
        origin=Origin(xyz=(0.086, 0.0, 0.935)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.2,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    slide = object_model.get_articulation("sleeve_to_post")
    lock_cover = object_model.get_part("lock_cover")
    warning_cover = object_model.get_part("warning_cover")
    lock_hinge = object_model.get_articulation("sleeve_to_lock_cover")
    warning_hinge = object_model.get_articulation("post_to_warning_cover")

    ctx.expect_overlap(
        post,
        sleeve,
        axes="xy",
        elem_a="post_body",
        elem_b="sleeve_shell",
        min_overlap=0.170,
        name="post stays centered over sleeve",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_shell",
        min_overlap=0.180,
        name="rest pose keeps deep post insertion",
    )
    ctx.expect_overlap(
        lock_cover,
        sleeve,
        axes="xy",
        elem_a="lock_flap",
        elem_b="lock_boss",
        min_overlap=0.026,
        name="lock cover spans the sleeve-side lock recess",
    )
    ctx.expect_gap(
        lock_cover,
        sleeve,
        axis="z",
        positive_elem="lock_flap",
        negative_elem="lock_boss",
        max_gap=0.0001,
        max_penetration=0.0,
        name="lock cover seats on the lock housing",
    )
    ctx.expect_overlap(
        warning_cover,
        post,
        axes="xy",
        elem_a="warning_flap",
        elem_b="warning_lens",
        min_overlap=0.040,
        name="warning cover shelters the crown light",
    )
    ctx.expect_gap(
        warning_cover,
        post,
        axis="z",
        positive_elem="warning_flap",
        negative_elem="warning_lens",
        min_gap=0.002,
        max_gap=0.008,
        name="warning cover hovers just above the crown light",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: POST_TRAVEL}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_shell",
            min_overlap=0.125,
            name="extended pose still retains post in sleeve",
        )
        extended_pos = ctx.part_world_position(post)

    ctx.check(
        "post lifts upward when extracted",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    lock_limits = lock_hinge.motion_limits
    if lock_limits is not None and lock_limits.upper is not None:
        lock_closed = ctx.part_element_world_aabb(lock_cover, elem="lock_flap")
        with ctx.pose({lock_hinge: lock_limits.upper}):
            lock_open = ctx.part_element_world_aabb(lock_cover, elem="lock_flap")
        ctx.check(
            "lock cover flips upward from the rim",
            lock_closed is not None
            and lock_open is not None
            and lock_open[1][2] > lock_closed[1][2] + 0.020,
            details=f"closed={lock_closed}, open={lock_open}",
        )

    warning_limits = warning_hinge.motion_limits
    if warning_limits is not None and warning_limits.upper is not None:
        warning_closed = ctx.part_element_world_aabb(warning_cover, elem="warning_flap")
        with ctx.pose({warning_hinge: warning_limits.upper}):
            warning_open = ctx.part_element_world_aabb(warning_cover, elem="warning_flap")
        ctx.check(
            "warning cover remains hinged and opens above the crown",
            warning_closed is not None
            and warning_open is not None
            and warning_open[1][2] > warning_closed[1][2] + 0.035,
            details=f"closed={warning_closed}, open={warning_open}",
        )

    return ctx.report()


object_model = build_object_model()
