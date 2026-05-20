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

BASE_RADIUS = 0.135
BASE_THICKNESS = 0.016
COLLAR_RADIUS = 0.032
COLLAR_HEIGHT = 0.022
SLEEVE_OUTER_RADIUS = 0.022
SLEEVE_INNER_RADIUS = 0.0185
SLEEVE_HEIGHT = 0.550
SLEEVE_SOCKET_FLOOR = 0.012
POST_RADIUS = 0.0155
POST_HIDDEN_BELOW_JOINT = 0.340
POST_VISIBLE_ABOVE_JOINT = 0.580
POST_TRAVEL = 0.240
POST_LENGTH = POST_HIDDEN_BELOW_JOINT + POST_VISIBLE_ABOVE_JOINT
POST_CENTER_Z = (POST_VISIBLE_ABOVE_JOINT - POST_HIDDEN_BELOW_JOINT) / 2.0
SLEEVE_TOP_Z = BASE_THICKNESS + SLEEVE_HEIGHT
NECK_REACH = 0.062
NECK_RISE = 0.034
NECK_LENGTH = math.hypot(NECK_REACH, NECK_RISE)
NECK_PITCH = math.atan2(NECK_REACH, NECK_RISE)
HINGE_X = NECK_REACH
HINGE_Z = POST_VISIBLE_ABOVE_JOINT + NECK_RISE
HEAD_LENGTH = 0.094
HEAD_WIDTH = 0.050
HEAD_HEIGHT = 0.042
HEAD_SHELL_X = 0.012
HEAD_SHELL_Z = -0.012
HEAD_NEUTRAL_PITCH = -0.28
HEAD_LENS_THICKNESS = 0.003
HEAD_TILT_LOWER = -0.35
HEAD_TILT_UPPER = 0.78


def _sleeve_shell_shape() -> cq.Workplane:
    sleeve_outer = cq.Workplane("XY").circle(SLEEVE_OUTER_RADIUS).extrude(SLEEVE_HEIGHT)
    lower_collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )
    top_trim = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_HEIGHT - 0.018)
        .circle(0.025)
        .extrude(0.018)
    )
    sleeve = sleeve_outer.union(lower_collar).union(top_trim)
    sleeve_cavity = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_SOCKET_FLOOR)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_HEIGHT - SLEEVE_SOCKET_FLOOR)
    )
    return sleeve.cut(sleeve_cavity)


def _head_shell_shape() -> cq.Workplane:
    body = (
        cq.Workplane("YZ")
        .rect(HEAD_WIDTH, HEAD_HEIGHT)
        .extrude(HEAD_LENGTH)
        .edges()
        .fillet(0.005)
    )
    lens_recess = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_LENGTH - 0.008)
        .rect(HEAD_WIDTH * 0.72, HEAD_HEIGHT * 0.54)
        .extrude(0.010)
    )
    return body.cut(lens_recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_reading_lamp")

    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.95, 0.90, 0.92))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_sleeve_shell_shape(), "lamp_sleeve_shell"),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
        material=graphite,
        name="sleeve_shell",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.028),
        origin=Origin(
            xyz=(SLEEVE_OUTER_RADIUS + 0.008, 0.0, SLEEVE_TOP_Z - 0.030),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="clamp_knob",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=satin_steel,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.0245, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=graphite,
        name="guide_collar",
    )
    post.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, POST_VISIBLE_ABOVE_JOINT - 0.012)),
        material=graphite,
        name="top_collar",
    )
    post.visual(
        Cylinder(radius=0.010, length=NECK_LENGTH),
        origin=Origin(
            xyz=(NECK_REACH / 2.0, 0.0, POST_VISIBLE_ABOVE_JOINT + NECK_RISE / 2.0),
            rpy=(0.0, NECK_PITCH, 0.0),
        ),
        material=graphite,
        name="neck_arm",
    )
    post.visual(
        Box((0.014, 0.022, 0.024)),
        origin=Origin(xyz=(HINGE_X - 0.007, 0.0, HINGE_Z)),
        material=graphite,
        name="hinge_block",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "lamp_head_shell"),
        origin=Origin(
            xyz=(HEAD_SHELL_X, 0.0, HEAD_SHELL_Z),
            rpy=(0.0, HEAD_NEUTRAL_PITCH, 0.0),
        ),
        material=graphite,
        name="head_shell",
    )
    head.visual(
        Box((HEAD_LENS_THICKNESS, HEAD_WIDTH * 0.64, HEAD_HEIGHT * 0.42)),
        origin=Origin(
            xyz=(HEAD_SHELL_X + HEAD_LENGTH - HEAD_LENS_THICKNESS * 0.35, 0.0, HEAD_SHELL_Z),
            rpy=(0.0, HEAD_NEUTRAL_PITCH, 0.0),
        ),
        material=diffuser_white,
        name="diffuser",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.007, 0.0, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rear_knuckle",
    )

    model.articulation(
        "base_to_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "post_to_head",
        ArticulationType.REVOLUTE,
        parent=post,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=HEAD_TILT_LOWER,
            upper=HEAD_TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    head = object_model.get_part("head")
    slide = object_model.get_articulation("base_to_post")
    tilt = object_model.get_articulation("post_to_head")

    ctx.expect_within(
        post,
        base,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="sleeve_shell",
        margin=0.003,
        name="post stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_shell",
        min_overlap=0.320,
        name="collapsed post remains deeply inserted in the sleeve",
    )

    rest_post_pos = ctx.part_world_position(post)
    with ctx.pose({slide: POST_TRAVEL}):
        ctx.expect_within(
            post,
            base,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="sleeve_shell",
            margin=0.003,
            name="extended post stays centered in the sleeve",
        )
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_shell",
            min_overlap=0.095,
            name="extended post still retains insertion in the sleeve",
        )
        extended_post_pos = ctx.part_world_position(post)

    ctx.check(
        "post extends upward",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.20,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    with ctx.pose({tilt: HEAD_TILT_LOWER}):
        low_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
    with ctx.pose({tilt: HEAD_TILT_UPPER}):
        high_aabb = ctx.part_element_world_aabb(head, elem="diffuser")

    low_center_z = None if low_aabb is None else (low_aabb[0][2] + low_aabb[1][2]) / 2.0
    high_center_z = None if high_aabb is None else (high_aabb[0][2] + high_aabb[1][2]) / 2.0
    ctx.check(
        "task head tilts upward",
        low_center_z is not None and high_center_z is not None and high_center_z > low_center_z + 0.05,
        details=f"low_center_z={low_center_z}, high_center_z={high_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
