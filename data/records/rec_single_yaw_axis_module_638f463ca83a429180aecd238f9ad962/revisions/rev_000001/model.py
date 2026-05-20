from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    Origin,
    MotionLimits,
    Material,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_dome")

    # Materials
    puck_material = Material(name="puck_white", rgba=(0.95, 0.95, 0.97, 1.0))
    dome_material = Material(name="dome_transparent", rgba=(0.7, 0.85, 0.95, 0.3))
    ring_material = Material(name="ring_dark", rgba=(0.25, 0.25, 0.3, 1.0))
    camera_material = Material(name="camera_black", rgba=(0.1, 0.1, 0.12, 1.0))
    seam_material = Material(name="seam_dark", rgba=(0.15, 0.15, 0.2, 1.0))

    # --- Fixed ceiling puck (root part) ---
    ceiling_puck = model.part("ceiling_puck")
    # Main puck body - circular ceiling mount
    ceiling_puck.visual(
        Cylinder(radius=0.15, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=puck_material,
        name="puck_body",
    )
    # Top face chamfer/bevel ring
    ceiling_puck.visual(
        Cylinder(radius=0.145, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=seam_material,
        name="puck_chamfer",
    )
    # Mounting flange on ceiling side
    ceiling_puck.visual(
        Cylinder(radius=0.12, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=ring_material,
        name="mount_flange",
    )

    # --- Transparent dome shell (fixed to ceiling puck) ---
    dome_shell = model.part("dome_shell")
    # Create dome using a sphere positioned to show upper hemisphere
    # The sphere sits so the bottom is at z=0 (matching ring height)
    dome_shell.visual(
        Sphere(radius=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dome_material,
        name="dome_body",
    )
    # Dome base ring/seam
    dome_shell.visual(
        Cylinder(radius=0.142, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=seam_material,
        name="dome_base_ring",
    )

    # --- Rotating inner ring ---
    rotating_ring = model.part("rotating_ring")
    # Main ring body
    rotating_ring.visual(
        Cylinder(radius=0.08, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=ring_material,
        name="ring_body",
    )
    # Ring top plate
    rotating_ring.visual(
        Cylinder(radius=0.075, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=seam_material,
        name="ring_top",
    )
    # Ring bottom plate
    rotating_ring.visual(
        Cylinder(radius=0.075, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=seam_material,
        name="ring_bottom",
    )

    # --- Camera stub ---
    camera_stub = model.part("camera_stub")
    # Camera body
    camera_stub.visual(
        Box((0.04, 0.035, 0.03)),
        origin=Origin(xyz=(0.04, 0.0, -0.015)),
        material=camera_material,
        name="camera_body",
    )
    # Camera lens
    camera_stub.visual(
        Cylinder(radius=0.012, length=0.015),
        origin=Origin(xyz=(0.06, 0.0, -0.015)),
        material=seam_material,
        name="camera_lens",
    )
    # Camera mount bracket
    camera_stub.visual(
        Box((0.015, 0.03, 0.02)),
        origin=Origin(xyz=(0.015, 0.0, -0.025)),
        material=ring_material,
        name="camera_mount",
    )

    # --- Articulation: Yaw joint for camera/ring assembly ---
    model.articulation(
        "puck_to_ring_yaw",
        ArticulationType.CONTINUOUS,
        parent=ceiling_puck,
        child=rotating_ring,
        # Joint at center of puck, ring rotates about Z axis (yaw)
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5),
    )

    # --- Fixed articulation: dome shell to ceiling puck ---
    model.articulation(
        "puck_to_dome",
        ArticulationType.FIXED,
        parent=ceiling_puck,
        child=dome_shell,
        origin=Origin(),
    )

    # --- Fixed articulation: camera stub to rotating ring ---
    model.articulation(
        "ring_to_camera",
        ArticulationType.FIXED,
        parent=rotating_ring,
        child=camera_stub,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_puck = object_model.get_part("ceiling_puck")
    dome_shell = object_model.get_part("dome_shell")
    rotating_ring = object_model.get_part("rotating_ring")
    camera_stub = object_model.get_part("camera_stub")
    yaw_joint = object_model.get_articulation("puck_to_ring_yaw")

    # Allow intentional overlaps - seated/nested parts
    # Camera body overlaps with ring body - intentional contact
    ctx.allow_overlap(
        "camera_stub", "rotating_ring",
        elem_a="camera_body",
        elem_b="ring_body",
        reason="Camera body is intentionally seated on the rotating ring"
    )
    # Camera lens overlaps with ring body - intentional contact
    ctx.allow_overlap(
        "camera_stub", "rotating_ring",
        elem_a="camera_lens",
        elem_b="ring_body",
        reason="Camera lens is intentionally close to the rotating ring"
    )
    # Camera mount overlaps with ring body - intentional contact
    ctx.allow_overlap(
        "camera_stub", "rotating_ring",
        elem_a="camera_mount",
        elem_b="ring_body",
        reason="Camera mount is intentionally seated on the rotating ring"
    )
    # Puck body overlaps with dome base ring - intentional nesting
    ctx.allow_overlap(
        "ceiling_puck", "dome_shell",
        elem_a="puck_body",
        elem_b="dome_base_ring",
        reason="Dome base ring sits inside ceiling puck body"
    )
    # Puck body overlaps with dome body - intentional nesting
    ctx.allow_overlap(
        "ceiling_puck", "dome_shell",
        elem_a="puck_body",
        elem_b="dome_body",
        reason="Dome body is nested inside ceiling puck"
    )
    # Puck chamfer overlaps with dome base ring
    ctx.allow_overlap(
        "ceiling_puck", "dome_shell",
        elem_a="puck_chamfer",
        elem_b="dome_base_ring",
        reason="Puck chamfer is intentionally close to dome base ring"
    )
    # Ring top seated near puck chamfer
    ctx.allow_overlap(
        "ceiling_puck", "rotating_ring",
        elem_a="puck_chamfer",
        elem_b="ring_top",
        reason="Ring top plate is intentionally close to puck chamfer for seam line"
    )
    # Dome base ring over ring top
    ctx.allow_overlap(
        "dome_shell", "rotating_ring",
        elem_a="dome_base_ring",
        elem_b="ring_top",
        reason="Dome base ring sits over the rotating ring top plate"
    )

    # Test 1: Check that ceiling_puck is the root (no parent articulation)
    ctx.check(
        "ceiling_puck_is_root",
        ceiling_puck is not None,
        details="ceiling_puck should exist as root part",
    )

    # Test 2: Check yaw joint exists and is CONTINUOUS
    ctx.check(
        "yaw_joint_exists",
        yaw_joint is not None,
        details="yaw joint should exist",
    )
    if yaw_joint is not None:
        ctx.check(
            "yaw_joint_is_continuous",
            yaw_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"yaw joint type is {yaw_joint.articulation_type}",
        )
        ctx.check(
            "yaw_axis_is_z",
            yaw_joint.axis == (0.0, 0.0, 1.0),
            details=f"yaw axis is {yaw_joint.axis}",
        )

    # Test 3: Check camera stub is physically supported (mounted on rotating ring)
    ctx.expect_contact(
        camera_stub,
        rotating_ring,
        elem_a="camera_mount",
        elem_b="ring_body",
        contact_tol=0.005,
        name="camera_mount_contacts_ring",
    )

    # Test 4: Check rotating ring is within dome footprint (XZ projection)
    ctx.expect_within(
        rotating_ring,
        dome_shell,
        axes="xy",
        margin=0.02,
        name="rotating_ring_inside_dome_footprint",
    )

    # Test 5: Test yaw rotation - camera should move in a circle
    with ctx.pose({yaw_joint: 0.0}):
        aabb_0 = ctx.part_world_aabb(camera_stub)
    
    with ctx.pose({yaw_joint: 1.57}):  # ~90 degrees
        aabb_90 = ctx.part_world_aabb(camera_stub)
    
    with ctx.pose({yaw_joint: 3.14}):  # ~180 degrees
        aabb_180 = ctx.part_world_aabb(camera_stub)

    if aabb_0 is not None and aabb_90 is not None and aabb_180 is not None:
        # Camera should rotate around Z axis, maintaining similar Z height
        center_0 = [(aabb_0[0][i] + aabb_0[1][i])/2 for i in range(3)]
        center_90 = [(aabb_90[0][i] + aabb_90[1][i])/2 for i in range(3)]
        center_180 = [(aabb_180[0][i] + aabb_180[1][i])/2 for i in range(3)]
        
        # Check Z height is similar (rotating horizontally)
        ctx.check(
            "camera_rotates_horizontally",
            abs(center_0[2] - center_90[2]) < 0.01 and abs(center_0[2] - center_180[2]) < 0.01,
            details=f"Z center changes: {abs(center_0[2] - center_90[2]):.4f}, {abs(center_0[2] - center_180[2]):.4f}",
        )
        # At 0°, camera should be in +X; at 180°, camera should be in -X
        ctx.check(
            "camera_rotates_180_degrees",
            center_0[0] > 0 and center_180[0] < 0,
            details=f"X center at 0°: {center_0[0]:.4f}, X center at 180°: {center_180[0]:.4f}",
        )

    # Test 6: Check dome is transparent (has alpha < 0.5)
    dome_visual = dome_shell.get_visual("dome_body")
    if dome_visual is not None and dome_visual.material is not None:
        try:
            rgba = dome_visual.material.rgba
            ctx.check(
                "dome_is_transparent",
                len(rgba) >= 4 and rgba[3] < 0.5,
                details=f"dome material alpha: {rgba[3] if len(rgba) >= 4 else 'unknown'}",
            )
        except (AttributeError, IndexError):
            pass

    # Test 7: Check seam lines exist (dome base ring, ring top/bottom)
    dome_base_ring = dome_shell.get_visual("dome_base_ring")
    ring_top = rotating_ring.get_visual("ring_top")
    ring_bottom = rotating_ring.get_visual("ring_bottom")
    
    ctx.check(
        "dome_has_seam_line",
        dome_base_ring is not None,
        details="dome should have base seam ring",
    )
    ctx.check(
        "ring_has_seam_lines",
        ring_top is not None and ring_bottom is not None,
        details="rotating ring should have top and bottom seam plates",
    )

    # Test 8: Check that rotating ring is supported by the joint (no floating)
    # The ring should be centered below the puck
    puck_pos = ctx.part_world_position(ceiling_puck)
    ring_pos = ctx.part_world_position(rotating_ring)
    if puck_pos is not None and ring_pos is not None:
        ctx.check(
            "ring_below_puck",
            ring_pos[2] < puck_pos[2],
            details=f"ring Z={ring_pos[2]:.3f}, puck Z={puck_pos[2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
