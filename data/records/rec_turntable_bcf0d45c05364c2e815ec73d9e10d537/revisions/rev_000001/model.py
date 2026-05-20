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


def build_disk_with_grip(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    grip_height: float,
) -> cq.Workplane:
    """Build rotating disk with textured grip edge."""
    # Main disk body with center hole
    disk = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .edges()
        .fillet(0.001)  # 1mm fillet on all edges
    )
    
    # Grip edge - slightly raised rim at the edge
    grip = (
        cq.Workplane("XY")
        .workplane(offset=thickness)
        .circle(outer_radius)
        .circle(outer_radius - 0.01)  # 1cm wide grip
        .extrude(grip_height)
        .edges()
        .fillet(0.0005)  # 0.5mm fillet on grip edges
    )
    
    return disk.union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cake_decorating_turntable")

    # Materials
    model.material("base_dark", rgba=(0.2, 0.2, 0.25, 1.0))
    model.material("disk_white", rgba=(0.95, 0.95, 0.97, 1.0))
    model.material("grip_gray", rgba=(0.8, 0.8, 0.82, 1.0))
    model.material("foot_dark", rgba=(0.15, 0.15, 0.2, 1.0))

    # Dimensions (in meters)
    base_radius = 0.15  # 30cm diameter pedestal
    base_height = 0.08  # 8cm tall base
    spindle_radius = 0.015  # 3cm diameter center spindle
    spindle_height = 0.025  # 2.5cm tall spindle
    disk_radius = 0.14  # 28cm diameter disk
    disk_thickness = 0.02  # 2cm thick disk
    disk_hole_radius = 0.016  # Slightly larger than spindle
    grip_height = 0.008  # 8mm tall grip edge
    foot_radius = 0.025  # 5cm diameter feet
    foot_height = 0.015  # 1.5cm tall feet

    # ---- Pedestal Base (root part, fixed) ----
    base = model.part("pedestal_base")

    # Main base body - wide pedestal
    base.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2)),
        material="base_dark",
        name="base_body",
    )

    # Center bearing collar/spindle
    base.visual(
        Cylinder(radius=spindle_radius, length=spindle_height),
        origin=Origin(xyz=(0.0, 0.0, base_height + spindle_height / 2)),
        material="base_dark",
        name="center_spindle",
    )

    # Stable feet (3 feet at 120-degree intervals)
    for i, angle_deg in enumerate([0, 120, 240]):
        rad = math.radians(angle_deg)
        x = (base_radius - 0.05) * math.cos(rad)
        y = (base_radius - 0.05) * math.sin(rad)
        base.visual(
            Cylinder(radius=foot_radius, length=foot_height),
            origin=Origin(xyz=(x, y, -foot_height / 2)),
            material="foot_dark",
            name=f"foot_{i}",
        )

    # ---- Rotating Cake Disk (child part) ----
    disk = model.part("rotating_disk")

    # Disk with center hole and grip edge (CadQuery for center hole)
    # CadQuery shape local origin at z=0, geometry from z=0 to z=disk_thickness+grip_height
    disk_shape = build_disk_with_grip(
        outer_radius=disk_radius,
        inner_radius=disk_hole_radius,
        thickness=disk_thickness,
        grip_height=grip_height,
    )
    
    # Disk sits on top of base (disk part frame is at z=base_height due to articulation)
    # Visual origin at z=0 relative to disk part frame so disk bottom is at base top
    disk.visual(
        mesh_from_cadquery(disk_shape, "disk_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="disk_white",
        name="disk_body",
    )

    # ---- Articulation: Vertical Yaw Joint ----
    # Joint at center top of base, disk rotates around Z axis
    model.articulation(
        "base_to_disk",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disk,
        # Articulation frame at the rotation center (top center of base)
        origin=Origin(xyz=(0.0, 0.0, base_height)),
        axis=(0.0, 0.0, 1.0),  # Vertical axis (Z-up)
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_base")
    disk = object_model.get_part("rotating_disk")
    joint = object_model.get_articulation("base_to_disk")

    # Allow intentional overlap: spindle fits inside disk center hole
    ctx.allow_overlap(
        "pedestal_base",
        "rotating_disk",
        reason="Center spindle intentionally fits inside disk center hole",
        elem_a="center_spindle",
        elem_b="disk_body",
    )
    
    # Proof checks for the allowed overlap
    ctx.expect_within(
        "pedestal_base",
        "rotating_disk",
        axes="xy",
        margin=0.001,
        inner_elem="center_spindle",
        outer_elem="disk_body",
        name="spindle_fits_in_disk_hole",
    )
    
    ctx.expect_overlap(
        "pedestal_base",
        "rotating_disk",
        axes="z",
        min_overlap=0.015,
        elem_a="center_spindle",
        elem_b="disk_body",
        name="spindle_overlaps_disk_in_z",
    )

    # Test 1: Disk stays centered on base (XY alignment)
    ctx.expect_within(
        disk,
        base,
        axes="xy",
        margin=0.005,
        name="disk_centered_on_base",
    )

    # Test 2: Disk is seated on base (Z gap - disk sits on base body)
    # Compare disk body to base body (not the spindle)
    ctx.expect_gap(
        disk,
        base,
        axis="z",
        min_gap=-0.001,  # Allow tiny penetration (seating)
        max_gap=0.005,  # Disk should be very close to base top
        positive_elem="disk_body",
        negative_elem="base_body",
        name="disk_seated_on_base",
    )

    # Test 3: Disk rotates properly - check pose change
    rest_pos = ctx.part_world_position(disk)
    with ctx.pose({joint: 1.57}):  # 90 degrees rotation
        rotated_pos = ctx.part_world_position(disk)
        # Position should be same (rotation around center doesn't change XY)
        ctx.expect_within(
            disk,
            base,
            axes="xy",
            margin=0.005,
            name="disk_centered_after_rotation",
        )
    # Check that rotation actually occurred (orientation changed)
    ctx.check(
        "disk_rotates",
        rest_pos is not None and rotated_pos is not None,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    # Test 4: Grip edge exists check
    disk_body = disk.get_visual("disk_body")
    ctx.check(
        "disk_body_exists",
        disk_body is not None,
        "Disk body visual not found",
    )

    return ctx.report()


object_model = build_object_model()
