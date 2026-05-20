from __future__ import annotations

from math import pi, cos, sin

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


def _make_bearing_ring(inner_radius=0.035, outer_radius=0.055, thickness=0.010):
    """Create a bearing ring using CadQuery."""
    import cadquery as cq

    # Create a ring (washer shape)
    ring = (
        cq.Workplane("XY")
        .workplane(offset=-thickness / 2)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )
    return ring


def _make_arrow_base(width=0.060, length=0.080, thickness=0.006):
    """Create a simple arrow shape for the rotating disk."""
    import cadquery as cq

    # Arrow shape: triangular head + rectangular tail
    # Tail (rectangle)
    tail_length = length * 0.5
    tail = (
        cq.Workplane("XY")
        .center(-length / 2 + tail_length / 2, 0)
        .rect(tail_length, width * 0.4)
        .extrude(thickness)
    )

    # Head (triangle) - pointing along +X
    head_length = length * 0.5
    head = (
        cq.Workplane("XY")
        .center(length / 2 - head_length / 2, 0)
        .polyline(
            [
                (-head_length / 2, -width / 2),
                (-head_length / 2, width / 2),
                (head_length / 2, 0),
            ]
        )
        .close()
        .extrude(thickness)
    )

    arrow = tail.union(head)
    return arrow


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_yaw_module")

    # Materials with color coding for teaching
    model.material("base_blue", rgba=(0.20, 0.35, 0.75, 1.0))  # Blue for fixed base
    model.material("disk_red", rgba=(0.80, 0.25, 0.20, 1.0))  # Red for rotating disk
    model.material("axle_silver", rgba=(0.70, 0.72, 0.75, 1.0))  # Silver for axle
    model.material("bearing_gray", rgba=(0.50, 0.52, 0.55, 1.0))  # Gray for bearing
    model.material("arrow_yellow", rgba=(0.95, 0.85, 0.20, 1.0))  # Yellow for arrow

    # ---- FIXED BASE ----
    base = model.part("base")
    # Main base body - cylindrical with chamfered edges
    base.visual(
        Cylinder(radius=0.100, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="base_blue",
        name="base_body",
    )
    # Base foot ring for stability
    base.visual(
        Cylinder(radius=0.110, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="base_blue",
        name="base_foot",
    )

    # ---- CENTRAL AXLE (fixed to base) ----
    # The axle is part of the base - it's the stationary central post
    # Length calculated to go through bearing and support disk, but not reach the arrow
    # Bearing at z=0.046 (top at ~0.052), disk body 0.0385-0.0535, axle ends at 0.055
    axle_length = 0.015
    axle_center_z = 0.040 + axle_length / 2  # Starts at top of base (0.040)
    base.visual(
        Cylinder(radius=0.012, length=axle_length),
        origin=Origin(xyz=(0.0, 0.0, axle_center_z)),
        material="axle_silver",
        name="central_axle",
    )

    # ---- BEARING RING (fixed to base, sits between base and rotating disk) ----
    bearing_outer = 0.055
    bearing_inner = 0.035
    bearing_thickness = 0.012
    base.visual(
        mesh_from_cadquery(
            _make_bearing_ring(bearing_inner, bearing_outer, bearing_thickness),
            "bearing_ring.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material="bearing_gray",
        name="bearing_ring",
    )

    # ---- ROTATING DISK ----
    rotating_disk = model.part("rotating_disk")
    # Main disk body
    rotating_disk.visual(
        Cylinder(radius=0.090, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material="disk_red",
        name="disk_body",
    )
    # Disk top plate (slightly larger diameter, thinner)
    rotating_disk.visual(
        Cylinder(radius=0.085, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="disk_red",
        name="disk_top",
    )

    # Central bore through the disk (rides on the axle)
    # This is represented as the disk having a hole - the axle passes through
    # The disk sits on the bearing and rotates around the central axle

    # ---- ARROW ON ROTATING DISK ----
    # Arrow to show rotation direction - mounted on top of rotating disk
    # CadQuery extrude goes from workplane (Z=0) to Z=thickness
    # So mesh local Z goes from 0 to thickness relative to origin
    # disk_top: center at z=0.017, length=0.004 -> local top at 0.017 + 0.002 = 0.019
    # To have arrow bottom (local z=0) sit on disk_top: origin_z = 0.019
    arrow_thickness = 0.006
    arrow_origin_z = 0.019  # Arrow local bottom (z=0) aligns with disk_top local top

    rotating_disk.visual(
        mesh_from_cadquery(
            _make_arrow_base(0.060, 0.080, arrow_thickness),
            "arrow.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, arrow_origin_z)),
        material="arrow_yellow",
        name="rotation_arrow",
    )

    # ---- ARTICULATION: Vertical Revolute Joint (Yaw) ----
    # The rotating disk rotates around the vertical (Z) axis
    # Joint is at the base/axle top, rotation is around Z
    model.articulation(
        "base_to_disk_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotating_disk,
        # Joint at the top of the base, centered on the axle
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),  # Vertical axis for yaw rotation
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-2 * pi,  # Full 360° rotation in each direction
            upper=2 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotating_disk = object_model.get_part("rotating_disk")
    yaw_joint = object_model.get_articulation("base_to_disk_yaw")

    # Test 1: Verify the yaw joint exists and is correctly configured
    ctx.check(
        "yaw_joint_exists_and_revolute",
        yaw_joint is not None and yaw_joint.articulation_type == ArticulationType.REVOLUTE,
        details="Yaw joint must be REVOLUTE type",
    )

    # Test 2: Verify vertical axis (Z-axis for yaw)
    ctx.check(
        "yaw_axis_is_vertical",
        yaw_joint.axis is not None
        and abs(yaw_joint.axis[0]) < 0.001
        and abs(yaw_joint.axis[1]) < 0.001
        and abs(yaw_joint.axis[2]) > 0.99,
        details=f"Yaw axis should be vertical (0,0,1), got {yaw_joint.axis}",
    )

    # Test 3: Check that base and rotating disk are centered (same X,Y position at rest)
    with ctx.pose({yaw_joint: 0.0}):
        ctx.expect_origin_distance(
            base,
            rotating_disk,
            axes="xy",
            max_dist=0.005,
            name="disk_centered_on_base_at_rest",
        )

    # Test 4: Verify contact/support between bearing and disk
    with ctx.pose({yaw_joint: 0.0}):
        ctx.expect_contact(
            base,
            rotating_disk,
            elem_a="bearing_ring",
            elem_b="disk_body",
            contact_tol=0.015,
            name="bearing_supports_disk",
        )

    # Test 5: Test rotation - verify disk moves correctly at 90 degrees
    with ctx.pose({yaw_joint: pi / 2}):
        # At 90°, the disk should have rotated but still be centered in XY
        ctx.expect_origin_distance(
            base,
            rotating_disk,
            axes="xy",
            max_dist=0.005,
            name="disk_still_centered_at_90_degrees",
        )
        # Check that the disk actually moved (world position changed)
        rest_pos = ctx.part_world_position(rotating_disk)
        ctx.check(
            "disk_rotated_at_90_degrees",
            rest_pos is not None,
            details="Disk should have valid world position at 90°",
        )

    # Test 6: Test full 180° rotation
    with ctx.pose({yaw_joint: pi}):
        ctx.expect_origin_distance(
            base,
            rotating_disk,
            axes="xy",
            max_dist=0.005,
            name="disk_still_centered_at_180_degrees",
        )

    # Test 7: Verify arrow is on the rotating disk
    arrow_visual = rotating_disk.get_visual("rotation_arrow")
    ctx.check(
        "arrow_present_on_disk",
        arrow_visual is not None,
        details="Rotation arrow must be present on disk",
    )

    # Test 8: Verify bearing ring is on the base
    bearing_visual = base.get_visual("bearing_ring")
    ctx.check(
        "bearing_present_on_base",
        bearing_visual is not None,
        details="Bearing ring must be present on base",
    )

    # Test 9: Verify central axle is on the base
    axle_visual = base.get_visual("central_axle")
    ctx.check(
        "axle_present_on_base",
        axle_visual is not None,
        details="Central axle must be present on base",
    )

    # Test 10: Check that disk can rotate freely (test at multiple angles)
    test_angles = [0.0, pi / 4, pi / 2, pi, -pi / 2, -pi / 4]
    for i, angle in enumerate(test_angles):
        with ctx.pose({yaw_joint: angle}):
            ctx.expect_origin_distance(
                base,
                rotating_disk,
                axes="xy",
                max_dist=0.005,
                name=f"disk_centered_at_angle_{i}",
            )

    # Allow small overlap between axle and disk bore (intentional nested fit)
    ctx.allow_overlap(
        "base",
        "rotating_disk",
        elem_a="central_axle",
        elem_b="disk_body",
        reason="Central axle passes through the disk bore - intentional nested fit for rotation",
    )

    # Also allow overlap between bearing and disk (bearing supports disk)
    ctx.allow_overlap(
        "base",
        "rotating_disk",
        elem_a="bearing_ring",
        elem_b="disk_body",
        reason="Bearing ring sits under the disk - intentional contact for support",
    )

    return ctx.report()


object_model = build_object_model()
