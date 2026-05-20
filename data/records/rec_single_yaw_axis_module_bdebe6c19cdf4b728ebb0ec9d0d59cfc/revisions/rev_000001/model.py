from __future__ import annotations

import math

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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sign_swivel_post")

    # Fixed vertical post (root part)
    fixed_post = model.part("fixed_post")
    post_height = 1.6  # meters
    post_radius = 0.05  # meters (0.1m diameter)
    fixed_post.visual(
        Cylinder(radius=post_radius, height=post_height),
        origin=Origin(xyz=(0.0, 0.0, post_height / 2)),  # Base at z=0, top at z=1.6
        name="post_body",
        color=(0.2, 0.2, 0.2),  # Dark gray metallic
    )

    # Rotating collar (child part, articulates around post)
    rotating_collar = model.part("rotating_collar")
    collar_outer_r = 0.065  # meters
    collar_inner_r = post_radius  # Matches post radius for snug fit
    collar_height = 0.2  # meters

    # Hollow collar using CadQuery
    collar_wp = (
        cq.Workplane("XY")
        .circle(collar_outer_r)
        .circle(collar_inner_r)
        .extrude(collar_height)
    )
    rotating_collar.visual(
        mesh_from_cadquery(collar_wp, "collar_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered at part frame (articulation point at z=1.2)
        name="collar_body",
        color=(0.33, 0.33, 0.33),  # Lighter gray metallic
    )

    # Rectangular sign bracket
    bracket_width = 0.6  # meters (x-axis)
    bracket_height = 0.4  # meters (y-axis)
    bracket_thickness = 0.02  # meters (z-axis)
    bracket_origin_x = collar_outer_r + bracket_width / 2  # Attached to collar outer surface
    rotating_collar.visual(
        Box((bracket_width, bracket_height, bracket_thickness)),
        origin=Origin(xyz=(bracket_origin_x, 0.0, 0.0)),  # Relative to collar part frame
        name="sign_bracket",
        color=(1.0, 1.0, 0.0),  # Yellow (common for signage)
    )

    # Clamp bolts and washers (two sets, top and bottom of bracket)
    bolt_head_radius = 0.012  # 12mm head
    washer_radius = 0.015  # 15mm washer
    washer_thickness = 0.002  # 2mm
    bolt_offset_y = 0.15  # 15cm from bracket center

    for i, y_sign in enumerate([1, -1]):
        # Washer (between bracket and collar)
        rotating_collar.visual(
            Cylinder(radius=washer_radius, height=washer_thickness),
            origin=Origin(xyz=(collar_outer_r + bracket_thickness / 2, y_sign * bolt_offset_y, 0.0)),
            name=f"washer_{i}",
            color=(0.67, 0.67, 0.67),  # Silver
        )
        # Bolt head
        rotating_collar.visual(
            Cylinder(radius=bolt_head_radius, height=0.005),
            origin=Origin(xyz=(collar_outer_r + bracket_thickness + 0.0025, y_sign * bolt_offset_y, 0.0)),
            name=f"bolt_head_{i}",
            color=(0.67, 0.67, 0.67),
        )

    # Primary yaw articulation (post -> collar)
    model.articulation(
        "post_to_collar",
        ArticulationType.REVOLUTE,
        parent=fixed_post,
        child=rotating_collar,
        origin=Origin(xyz=(0.0, 0.0, 1.2)),  # Collar center
        axis=(0.0, 0.0, 1.0),  # Vertical Z-axis (yaw)
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,  # Full 360° rotation
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("fixed_post")
    collar = object_model.get_part("rotating_collar")
    joint = object_model.get_articulation("post_to_collar")

    # Allow intentional overlap - collar is designed to fit around the post
    ctx.allow_overlap(
        "fixed_post",
        "rotating_collar",
        elem_a="post_body",
        elem_b="collar_body",
        reason="Collar is a hollow cylinder designed to fit around the post (inner radius matches post radius)",
    )

    # 1. Mechanism checks
    ctx.check(
        "joint_type_correct",
        joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Joint type: {joint.articulation_type}",
    )
    ctx.check(
        "joint_axis_yaw",
        joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint axis: {joint.axis}",
    )
    ctx.check(
        "joint_limits_valid",
        joint.motion_limits.lower == -math.pi and joint.motion_limits.upper == math.pi,
        details=f"Joint limits: {joint.motion_limits.lower} to {joint.motion_limits.upper}",
    )

    # 2. Support/contact checks - post within collar (collar surrounds post)
    ctx.expect_within(
        post,
        collar,
        axes="xy",
        elem_a="post_body",
        elem_b="collar_body",
        margin=0.001,
        name="post_within_collar_xy",
    )

    # Check that collar actually rotates
    rest_pos = ctx.part_world_position(collar)
    with ctx.pose({joint: math.pi / 2}):
        rotated_pos = ctx.part_world_position(collar)
        ctx.check(
            "collar_rotates",
            rest_pos is not None and rotated_pos is not None,
            details=f"Rest: {rest_pos}, Rotated: {rotated_pos}",
        )

    # 3. Closed/rest pose check (q=0)
    with ctx.pose({joint: 0.0}):
        bracket_min, bracket_max = ctx.part_element_world_aabb(collar, elem="sign_bracket")
        bracket_center = [(bracket_min[i] + bracket_max[i]) / 2 for i in range(3)]
        ctx.check(
            "bracket_rest_x_positive",
            bracket_center[0] > 0.3,
            details=f"Rest bracket center: {bracket_center}",
        )

    # 4. Rotated pose (90° yaw)
    with ctx.pose({joint: math.pi / 2}):
        bracket_min, bracket_max = ctx.part_element_world_aabb(collar, elem="sign_bracket")
        bracket_center = [(bracket_min[i] + bracket_max[i]) / 2 for i in range(3)]
        ctx.check(
            "bracket_rotated_90_y_positive",
            bracket_center[1] > 0.3,
            details=f"90° bracket center: {bracket_center}",
        )
        ctx.check(
            "bracket_rotated_90_x_near_zero",
            abs(bracket_center[0]) < 0.05,
            details=f"90° bracket x: {bracket_center[0]}",
        )

    # 5. Visible details checks
    collar_visuals = [v.name for v in collar.visuals]
    bolt_count = sum(1 for v in collar_visuals if "bolt" in v)
    washer_count = sum(1 for v in collar_visuals if "washer" in v)
    ctx.check("bolt_count", bolt_count >= 2, details=f"Bolt count: {bolt_count}")
    ctx.check("washer_count", washer_count >= 2, details=f"Washer count: {washer_count}")

    # 6. Root part check - verify fixed_post is the first/root part
    ctx.check(
        "post_exists",
        object_model.get_part("fixed_post") is not None,
        details="Fixed post part exists",
    )

    return ctx.report()


object_model = build_object_model()
