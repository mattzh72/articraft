from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    Origin,
    TestContext,
    TestReport,
    MotionLimits,
    Material,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_tube_teeter_totter")

    # Register materials
    metal_tube = Material(name="metal_tube", rgba=(0.7, 0.7, 0.7, 1.0))
    seat_material = Material(name="seat_material", rgba=(0.2, 0.6, 0.2, 1.0))
    bumper_material = Material(name="bumper_material", rgba=(0.8, 0.2, 0.2, 1.0))
    handle_material = Material(name="handle_material", rgba=(0.3, 0.3, 0.3, 1.0))
    model.material(metal_tube)
    model.material(seat_material)
    model.material(bumper_material)
    model.material(handle_material)

    # A-frame support (root part)
    a_frame = model.part("a_frame")

    # A-frame leg geometry: 0.8m base width, 1.2m height to pivot
    leg_length = math.sqrt(0.4**2 + 1.2**2)  # ~1.265m
    leg_radius = 0.03  # 3cm diameter tube
    leg_angle = math.atan2(0.4, 1.2)  # ~0.3218 radians

    # Left A-leg (positive X side)
    a_frame.visual(
        Cylinder(radius=leg_radius, length=leg_length),
        origin=Origin(
            xyz=(0.2, 0, 0.6),  # midpoint of leg
            rpy=(0, -leg_angle, 0)  # rotate to span from (0.4,0,0) to (0,0,1.2)
        ),
        material=metal_tube,
        name="left_leg"
    )

    # Right A-leg (negative X side)
    a_frame.visual(
        Cylinder(radius=leg_radius, length=leg_length),
        origin=Origin(
            xyz=(-0.2, 0, 0.6),  # midpoint of leg
            rpy=(0, leg_angle, 0)  # rotate to span from (-0.4,0,0) to (0,0,1.2)
        ),
        material=metal_tube,
        name="right_leg"
    )

    # Central pivot axle (2cm diameter, 10cm long along Y-axis)
    a_frame.visual(
        Cylinder(radius=0.02, length=0.1),
        origin=Origin(
            xyz=(0, 0, 1.2),  # pivot point at top of A-frame
            rpy=(1.5708, 0, 0)  # rotate 90° around X to align along Y
        ),
        material=metal_tube,
        name="pivot_axle"
    )

    # Beam assembly (moving part with seats, handles, bumpers)
    beam = model.part("beam")

    # Main tubular beam: 3m long, 10cm diameter, aligned along X-axis
    beam.visual(
        Cylinder(radius=0.05, length=3.0),
        origin=Origin(rpy=(0, 1.5708, 0)),  # rotate 90° around Y to align along X
        material=metal_tube,
        name="main_beam"
    )

    # Molded seats (one on each end, 30cm long × 20cm wide × 5cm thick)
    seat_size = (0.3, 0.2, 0.05)
    seat_offset_x = 1.4  # 1.4m from center (10cm from each end for bumper)
    seat_z = 0.075  # top of beam (0.05m) + half seat thickness (0.025m)

    beam.visual(
        Box(seat_size),
        origin=Origin(xyz=(seat_offset_x, 0, seat_z)),
        material=seat_material,
        name="seat_positive"
    )
    beam.visual(
        Box(seat_size),
        origin=Origin(xyz=(-seat_offset_x, 0, seat_z)),
        material=seat_material,
        name="seat_negative"
    )

    # U-shaped handles (one near each seat, 1cm diameter tubing)
    handle_radius = 0.01
    handle_height = 0.3  # 30cm tall
    handle_width = 0.1  # 10cm between posts
    handle_y = -0.05  # closer to beam (local Y negative)
    handle_base_z = 0.04  # base of handle inserted into beam by 1cm for connection

    # Positive X handle (near seat_positive)
    for post_y in [handle_y - handle_width/2, handle_y + handle_width/2]:
        beam.visual(
            Cylinder(radius=handle_radius, length=handle_height),
            origin=Origin(
                xyz=(seat_offset_x, post_y, handle_base_z),
                rpy=(1.5708, 0, 0)  # stand vertical along Z
            ),
            material=handle_material,
            name=f"handle_positive_{'left' if post_y < handle_y else 'right'}"
        )
    # Handle top bar (extended length to overlap with posts)
    beam.visual(
        Cylinder(radius=handle_radius, length=handle_width + 0.06),  # extended by 6cm total (3cm per side)
        origin=Origin(
            xyz=(seat_offset_x, handle_y - handle_width/2 + 0.03, handle_base_z + handle_height/2),  # centered to overlap posts
            rpy=(0, 1.5708, 0)  # align along Y
        ),
        material=handle_material,
        name="handle_positive_top"
    )

    # Negative X handle (near seat_negative)
    for post_y in [handle_y - handle_width/2, handle_y + handle_width/2]:
        beam.visual(
            Cylinder(radius=handle_radius, length=handle_height),
            origin=Origin(
                xyz=(-seat_offset_x, post_y, handle_base_z),
                rpy=(1.5708, 0, 0)
            ),
            material=handle_material,
            name=f"handle_negative_{'left' if post_y < handle_y else 'right'}"
        )
    # Handle top bar (extended length to overlap with posts)
    beam.visual(
        Cylinder(radius=handle_radius, length=handle_width + 0.06),  # extended by 6cm total (3cm per side)
        origin=Origin(
            xyz=(-seat_offset_x, handle_y - handle_width/2 + 0.03, handle_base_z + handle_height/2),  # centered to overlap posts
            rpy=(0, 1.5708, 0)
        ),
        material=handle_material,
        name="handle_negative_top"
    )

    # End bumpers (5cm radius rubber spheres)
    for x_sign in [1, -1]:
        beam.visual(
            Sphere(radius=0.05),
            origin=Origin(xyz=(x_sign * 1.5, 0, 0)),
            material=bumper_material,
            name=f"bumper_{'positive' if x_sign > 0 else 'negative'}"
        )

    # Central pitch joint (revolute, ±15° rotation around Y-axis)
    model.articulation(
        "a_frame_to_beam",
        ArticulationType.REVOLUTE,
        parent=a_frame,
        child=beam,
        origin=Origin(xyz=(0, 0, 1.2)),  # pivot point in A-frame frame
        axis=(0, -1.0, 0),  # pitch rotation around Y-axis (positive q lifts +X end)
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=10.0,
            lower=-0.2618,  # -15°
            upper=0.2618   # +15°
        )
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    model = object_model

    # Get parts and articulation
    a_frame = model.get_part("a_frame")
    beam = model.get_part("beam")
    joint = model.get_articulation("a_frame_to_beam")

    # Mechanism checks
    ctx.check("articulation_exists", joint is not None, details="Central pitch joint not found")
    ctx.check(
        "articulation_type_revolute",
        joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Expected REVOLUTE, got {joint.articulation_type}"
    )
    ctx.check(
        "articulation_axis_y",
        joint.axis == (0.0, -1.0, 0.0),
        details=f"Expected axis (0,-1,0), got {joint.axis}"
    )
    ctx.check(
        "motion_limits_valid",
        joint.motion_limits and joint.motion_limits.lower is not None and joint.motion_limits.upper is not None,
        details="Motion limits missing or incomplete"
    )
    if joint.motion_limits:
        ctx.check(
            "motion_limits_range",
            abs(joint.motion_limits.lower + 0.2618) < 1e-4 and abs(joint.motion_limits.upper - 0.2618) < 1e-4,
            details=f"Expected limits [-0.2618, 0.2618], got [{joint.motion_limits.lower}, {joint.motion_limits.upper}]"
        )

    # Support and structure checks
    ctx.check(
        "beam_child_of_a_frame",
        joint.parent == a_frame.name and joint.child == beam.name,
        details="Beam is not the child of A-frame"
    )

    # Visible details checks
    for seat_name in ["seat_positive", "seat_negative"]:
        ctx.check(
            f"{seat_name}_exists",
            beam.get_visual(seat_name) is not None,
            details=f"{seat_name} missing"
        )
    for bumper_name in ["bumper_positive", "bumper_negative"]:
        ctx.check(
            f"{bumper_name}_exists",
            beam.get_visual(bumper_name) is not None,
            details=f"{bumper_name} missing"
        )
    for handle_side in ["positive", "negative"]:
        ctx.check(
            f"handle_{handle_side}_exists",
            beam.get_visual(f"handle_{handle_side}_left") is not None,
            details=f"{handle_side} handle missing"
        )

    # Pose-specific checks
    # Rest pose (q=0): beam level
    with ctx.pose({"a_frame_to_beam": 0.0}):
        seat_pos_aabb = ctx.part_element_world_aabb(beam, elem="seat_positive")
        seat_neg_aabb = ctx.part_element_world_aabb(beam, elem="seat_negative")
        if seat_pos_aabb and seat_neg_aabb:
            z_diff = abs(seat_pos_aabb[1][2] - seat_neg_aabb[1][2])
            ctx.check("rest_pose_level", z_diff < 0.01, details=f"Seat Z difference at rest: {z_diff}")

    # Positive rotation (+15°): +X end up
    with ctx.pose({"a_frame_to_beam": 0.2618}):
        seat_pos_aabb = ctx.part_element_world_aabb(beam, elem="seat_positive")
        seat_neg_aabb = ctx.part_element_world_aabb(beam, elem="seat_negative")
        if seat_pos_aabb and seat_neg_aabb:
            ctx.check(
                "positive_pose_positive_end_up",
                seat_pos_aabb[1][2] > seat_neg_aabb[1][2] + 0.1,
                details=f"+X Z: {seat_pos_aabb[1][2]}, -X Z: {seat_neg_aabb[1][2]}"
            )

    # Negative rotation (-15°): -X end up
    with ctx.pose({"a_frame_to_beam": -0.2618}):
        seat_pos_aabb = ctx.part_element_world_aabb(beam, elem="seat_positive")
        seat_neg_aabb = ctx.part_element_world_aabb(beam, elem="seat_negative")
        if seat_pos_aabb and seat_neg_aabb:
            ctx.check(
                "negative_pose_negative_end_up",
                seat_neg_aabb[1][2] > seat_pos_aabb[1][2] + 0.1,
                details=f"-X Z: {seat_neg_aabb[1][2]}, +X Z: {seat_pos_aabb[1][2]}"
            )

    # Overlap allowance: intentional pivot overlap between axle and beam
    ctx.allow_overlap(
        "a_frame",
        "beam",
        reason="Beam rotates around pivot axle, intentional nested overlap at central pivot",
        elem_a="pivot_axle",
        elem_b="main_beam"
    )
    # Overlap allowance: A-frame legs support beam at top, intentional contact
    ctx.allow_overlap(
        "a_frame",
        "beam",
        reason="A-frame legs support beam at pivot point, intentional contact",
        elem_a="left_leg",
        elem_b="main_beam"
    )
    ctx.allow_overlap(
        "a_frame",
        "beam",
        reason="A-frame legs support beam at pivot point, intentional contact",
        elem_a="right_leg",
        elem_b="main_beam"
    )
    with ctx.pose({"a_frame_to_beam": 0.0}):
        ctx.expect_contact(
            "a_frame",
            "beam",
            elem_a="pivot_axle",
            elem_b="main_beam",
            contact_tol=0.01,
            name="pivot_axle_beam_contact"
        )

    return ctx.report()


object_model = build_object_model()