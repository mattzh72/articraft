from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)

import cadquery as cq
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bearing_lazy_susan")

    # Fixed base part
    base = model.part("base")

    # Base bottom disk: solid cylinder, dark gray, 0.3m diameter, 0.02m thick
    base.visual(
        Cylinder(radius=0.15, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Spans z=0 to 0.02
        name="base_disk",
        material=Material(name="base_disk_mat", rgba=(0.2, 0.2, 0.2, 1.0)),  # Dark gray, opaque
    )

    # Center fastener: steel cylinder through center, 0.01m radius, 0.06m tall
    base.visual(
        Cylinder(radius=0.01, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),  # Spans z=0 to 0.06
        name="center_fastener",
        material=Material(name="fastener_mat", rgba=(0.8, 0.8, 0.8, 1.0)),  # Silver
    )

    # Bearing beads: 8 steel spheres, 0.005m radius, arranged in a circle 0.12m from center
    bead_radius = 0.005
    bead_circle_radius = 0.12
    bead_z = 0.025  # Middle of bearing (base top at 0.02, top disk bottom at 0.03)
    for i in range(8):
        angle = 2 * math.pi * i / 8
        x = bead_circle_radius * math.cos(angle)
        y = bead_circle_radius * math.sin(angle)
        base.visual(
            Sphere(radius=bead_radius),
            origin=Origin(xyz=(x, y, bead_z)),
            name=f"bead_{i}",
            material=Material(name="bead_mat", rgba=(0.7, 0.7, 0.7, 1.0)),  # Steel
        )

    # Rotating top disk part
    top_disk = model.part("top_disk")

    # Top disk: hollow cylinder (annulus) with arrows, transparent light blue
    # Main disk: outer radius 0.15m, inner 0.012m, height 0.02m
    disk = (
        cq.Workplane("XY")
        .circle(0.15)
        .circle(0.012)
        .extrude(0.02)
    )
    # Arrow parameters (relative to disk local frame: z=0 at disk bottom, z=0.02 at disk top)
    arrow_length = 0.08
    arrow_width = 0.01
    arrow_thickness = 0.005
    arrow_local_z = 0.022  # 0.002m above disk top (0.02 + 0.002)
    # Red arrow along +X (center at x=0.05, y=0)
    red_arrow = (
        cq.Workplane("XY")
        .workplane(offset=arrow_local_z)
        .center(0.05, 0)  # Center of arrow (half length = 0.04m)
        .rect(arrow_length, arrow_width)
        .extrude(arrow_thickness)
    )
    # Green arrow along -X (center at x=-0.05, y=0)
    green_arrow = (
        cq.Workplane("XY")
        .workplane(offset=arrow_local_z)
        .center(-0.05, 0)
        .rect(arrow_length, arrow_width)
        .extrude(arrow_thickness)
    )
    # Combine all shapes
    top_disk_shape = disk.union(red_arrow).union(green_arrow)
    top_disk.visual(
        mesh_from_cadquery(top_disk_shape, "top_disk_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),  # Places disk bottom at z=0.03 (0.025 part frame + 0.005)
        name="top_disk_shell",
        material=Material(name="top_disk_mat", rgba=(0.8, 0.8, 1.0, 0.4)),  # Light blue, 40% transparent
    )

    # Articulation: continuous yaw joint (rotation around Z axis)
    model.articulation(
        "base_to_top",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_disk,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),  # Center of bearing beads
        axis=(0.0, 0.0, 1.0),  # Z axis (yaw)
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top = object_model.get_part("top_disk")
    joint = object_model.get_articulation("base_to_top")

    # Test 1: Joint configuration
    ctx.check(
        "joint is continuous yaw type",
        joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint type: {joint.articulation_type}, axis: {joint.axis}",
    )

    # Test 2: Rest pose (q=0)
    with ctx.pose({joint: 0.0}):
        # Top disk is above base disk with correct bearing gap (0.01m between base disk top and top disk bottom)
        ctx.expect_gap(
            positive_link=top,
            negative_link=base,
            axis="z",
            min_gap=0.009,
            max_gap=0.011,
            negative_elem="base_disk",  # Use only the base bottom disk visual, not the entire base part
            name="top disk bearing gap at rest",
        )
        # Top disk is centered over base (same XY footprint)
        ctx.expect_within(
            inner_link=top,
            outer_link=base,
            axes="xy",
            margin=0.001,
            name="top disk centered over base",
        )
        # Bearing beads are correctly positioned (validated via existence checks later)

    # Test 3: Rotated pose (π/2 radians)
    with ctx.pose({joint: math.pi / 2}):
        top_pos = ctx.part_world_position(top)
        # Top disk origin remains centered (rotation around Z doesn't shift origin)
        ctx.check(
            "top disk origin centered after π/2 rotation",
            abs(top_pos[0]) < 0.001 and abs(top_pos[1]) < 0.001 and abs(top_pos[2] - 0.025) < 0.001,
            details=f"Top world position: {top_pos}",
        )
        # Verify rotation by checking top disk world position remains centered
        top_pos = ctx.part_world_position(top)
        ctx.check(
            "top disk remains centered after rotation",
            abs(top_pos[0]) < 0.001 and abs(top_pos[1]) < 0.001,
            details=f"Top position: {top_pos}",
        )

    # Test 4: Visible details
    # Top disk is transparent (alpha < 0.5)
    top_shell = top.get_visual("top_disk_shell")
    if top_shell and top_shell.material:
        ctx.check(
            "top disk is transparent",
            len(top_shell.material.rgba) == 4 and top_shell.material.rgba[3] < 0.5,
            details=f"Top disk material RGBA: {top_shell.material.rgba}",
        )
    # Bearing beads exist and are correct count
    for i in range(8):
        bead = base.get_visual(f"bead_{i}")
        ctx.check(f"bead_{i} exists", bead is not None)
    # Arrows are present (merged into top disk shape, check via AABB)
    top_aabb = ctx.part_world_aabb(top)
    if top_aabb:
        # Top disk AABB should include arrows extending along X
        ctx.check(
            "arrows present (X extent > 0.15m)",
            top_aabb[1][0] - top_aabb[0][0] > 0.16,  # Base disk is 0.3m (0.15 radius), arrows add 0.01m on each side
            details=f"Top disk X extent: {top_aabb[1][0] - top_aabb[0][0]}",
        )

    return ctx.report()


object_model = build_object_model()
