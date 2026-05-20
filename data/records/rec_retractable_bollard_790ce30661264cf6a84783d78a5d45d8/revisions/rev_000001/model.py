from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

PAVEMENT_SIZE = 1.0
PAVEMENT_THICKNESS = 0.10
OPENING_RADIUS = 0.145
SLEEVE_INNER_RADIUS = 0.116
SLEEVE_OUTER_RADIUS = 0.135
COLLAR_RADIUS = 0.165
COLLAR_HEIGHT = 0.012
SLEEVE_DEPTH = 1.22

POST_RADIUS = 0.106
POST_WALL = 0.006
POST_LENGTH = 1.10
POST_TOP_AT_REST = 0.82
POST_CENTER_Z = POST_TOP_AT_REST - (POST_LENGTH * 0.5)
POST_TRAVEL = 0.92

CAP_RADIUS = 0.112
CAP_HEIGHT = 0.032
CAP_SEAM = 0.0

HINGE_AXIS_X = 0.142
HINGE_AXIS_Z = 0.016
FLAP_OPEN_ANGLE = math.radians(102.0)


def _aabb_dims(aabb):
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _build_pavement_mesh():
    slab = (
        cq.Workplane("XY")
        .box(PAVEMENT_SIZE, PAVEMENT_SIZE, PAVEMENT_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, -PAVEMENT_THICKNESS))
    )
    slab = (
        slab.faces(">Z")
        .workplane()
        .circle(OPENING_RADIUS)
        .cutBlind(-PAVEMENT_THICKNESS)
    )
    return mesh_from_cadquery(slab, "bollard_pavement")


def _build_sleeve_mesh():
    sleeve = (
        cq.Workplane("XY")
        .workplane(offset=-SLEEVE_DEPTH)
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_DEPTH + COLLAR_HEIGHT)
    )
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )
    return mesh_from_cadquery(sleeve.union(collar), "bollard_sleeve")


def _build_post_mesh():
    post = (
        cq.Workplane("XY")
        .workplane(offset=POST_TOP_AT_REST - POST_LENGTH)
        .circle(POST_RADIUS)
        .circle(POST_RADIUS - POST_WALL)
        .extrude(POST_LENGTH)
    )
    return mesh_from_cadquery(post, "bollard_post")


def _build_flap_mesh():
    cover = (
        cq.Workplane("XY")
        .box(0.190, 0.170, 0.008, centered=(False, True, False))
        .translate((-0.240, 0.0, 0.004))
    )
    rib = (
        cq.Workplane("XY")
        .box(0.110, 0.032, 0.018, centered=(False, True, False))
        .translate((-0.175, 0.0, -0.001))
    )
    leaf = (
        cq.Workplane("XY")
        .box(0.050, 0.080, 0.010, centered=(False, True, False))
        .translate((-0.050, 0.0, 0.003))
    )
    return mesh_from_cadquery(cover.union(rib).union(leaf), "bollard_flap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_bollard")

    concrete = model.material("concrete", rgba=(0.62, 0.62, 0.60, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_lock = model.material("dark_lock", rgba=(0.18, 0.19, 0.20, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(_build_pavement_mesh(), material=concrete, name="pavement")
    sleeve.visual(_build_sleeve_mesh(), material=stainless, name="guide_sleeve")
    sleeve.visual(
        Box((0.044, 0.280, 0.016)),
        origin=Origin(xyz=(0.171, 0.0, 0.008)),
        material=stainless,
        name="hinge_pedestal",
    )
    for index, y in enumerate((-0.110, 0.110)):
        sleeve.visual(
            Cylinder(radius=0.008, length=0.046),
            origin=Origin(xyz=(HINGE_AXIS_X, y, HINGE_AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"hinge_knuckle_{index}",
        )

    post = model.part("post")
    post.visual(_build_post_mesh(), material=stainless, name="tube")
    post.visual(
        Cylinder(radius=0.128, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_steel,
        name="stop_collar",
    )

    flap = model.part("flap")
    flap.visual(
        _build_flap_mesh(),
        origin=Origin(rpy=(0.0, FLAP_OPEN_ANGLE, 0.0)),
        material=brushed_steel,
        name="cover_plate",
    )
    flap.visual(
        Cylinder(radius=0.0075, length=0.066),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_knuckle",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=CAP_RADIUS, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stainless,
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(),
        material=brushed_steel,
        name="cap_stem",
    )
    cap.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_lock,
        name="lock_boss",
    )
    cap.visual(
        Box((0.034, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_lock,
        name="key_insert",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.40,
            lower=-POST_TRAVEL,
            upper=0.0,
        ),
    )
    model.articulation(
        "sleeve_to_flap",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=flap,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-FLAP_OPEN_ANGLE,
            upper=0.0,
        ),
    )
    model.articulation(
        "post_to_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_AT_REST)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    flap = object_model.get_part("flap")
    cap = object_model.get_part("cap")

    post_joint = object_model.get_articulation("sleeve_to_post")
    flap_joint = object_model.get_articulation("sleeve_to_flap")
    cap_joint = object_model.get_articulation("post_to_cap")

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        elem_a="tube",
        elem_b="guide_sleeve",
        margin=0.001,
        name="post stays centered in the sleeve",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="tube",
        elem_b="guide_sleeve",
        min_overlap=0.27,
        name="extended post remains inserted in the sleeve",
    )
    ctx.expect_contact(
        cap,
        post,
        elem_a="cap_shell",
        elem_b="tube",
        contact_tol=1e-5,
        name="cap stays seated on the post tube",
    )

    extended_tube = ctx.part_element_world_aabb(post, elem="tube")
    open_flap = ctx.part_element_world_aabb(flap, elem="cover_plate")
    cap_shell = ctx.part_element_world_aabb(cap, elem="cap_shell")
    rest_key = ctx.part_element_world_aabb(cap, elem="key_insert")

    ctx.check(
        "cap reads as a distinct top piece",
        extended_tube is not None
        and cap_shell is not None
        and abs(cap_shell[0][2] - extended_tube[1][2]) <= 0.001
        and _aabb_dims(cap_shell)[0] > _aabb_dims(extended_tube)[0] + 0.008,
        details=f"tube={extended_tube}, cap={cap_shell}",
    )

    post_lower = post_joint.motion_limits.lower if post_joint.motion_limits is not None else None
    flap_lower = flap_joint.motion_limits.lower if flap_joint.motion_limits is not None else None

    if post_lower is not None:
        with ctx.pose({post_joint: post_lower}):
            ctx.expect_within(
                post,
                sleeve,
                axes="xy",
                elem_a="tube",
                elem_b="guide_sleeve",
                margin=0.001,
                name="retracted post stays centered in the sleeve",
            )
            ctx.expect_overlap(
                post,
                sleeve,
                axes="z",
                elem_a="tube",
                elem_b="guide_sleeve",
                min_overlap=0.35,
                name="retracted post remains deeply guided in the sleeve",
            )
            retracted_tube = ctx.part_element_world_aabb(post, elem="tube")

        ctx.check(
            "post retracts downward into the sleeve",
            extended_tube is not None
            and retracted_tube is not None
            and retracted_tube[1][2] < extended_tube[1][2] - 0.80,
            details=f"extended={extended_tube}, retracted={retracted_tube}",
        )

    if post_lower is not None and flap_lower is not None:
        with ctx.pose({post_joint: post_lower, flap_joint: flap_lower}):
            ctx.expect_overlap(
                flap,
                sleeve,
                axes="xy",
                elem_a="cover_plate",
                elem_b="guide_sleeve",
                min_overlap=0.12,
                name="closed flap bridges the sleeve opening",
            )
            closed_flap = ctx.part_element_world_aabb(flap, elem="cover_plate")

        ctx.check(
            "flap lifts up beside the opening",
            open_flap is not None
            and closed_flap is not None
            and open_flap[1][2] > closed_flap[1][2] + 0.12,
            details=f"open={open_flap}, closed={closed_flap}",
        )

    with ctx.pose({cap_joint: math.pi / 2.0}):
        turned_key = ctx.part_element_world_aabb(cap, elem="key_insert")

    ctx.check(
        "keyed cap rotates on its local axis",
        rest_key is not None
        and turned_key is not None
        and _aabb_dims(rest_key)[0] > _aabb_dims(rest_key)[1]
        and _aabb_dims(turned_key)[1] > _aabb_dims(turned_key)[0],
        details=f"rest={rest_key}, turned={turned_key}",
    )

    return ctx.report()


object_model = build_object_model()
