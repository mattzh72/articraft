from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pan_head")

    # Materials
    base_material = Material(name="base_material", rgba=(0.2, 0.2, 0.22, 1.0))
    pan_material = Material(name="pan_material", rgba=(0.3, 0.3, 0.32, 1.0))
    platform_material = Material(name="platform_material", rgba=(0.35, 0.35, 0.37, 1.0))
    knob_material = Material(name="knob_material", rgba=(0.8, 0.25, 0.2, 1.0))
    bearing_material = Material(name="bearing_material", rgba=(0.7, 0.7, 0.7, 1.0))

    # === ROOT PART: Base Socket ===
    base_socket = model.part("base_socket")

    # Base: outer radius 30mm, height 35mm
    base_outer = cq.Workplane("XY").circle(0.030).extrude(0.035)
    base_inner = cq.Workplane("XY").circle(0.025).extrude(0.030)
    base_shape = base_outer.cut(base_inner)

    # Add tripod socket recess at bottom
    socket_hole = cq.Workplane("XY").workplane(offset=-0.001).circle(0.004).extrude(-0.008)
    base_shape = base_shape.cut(socket_hole)

    # Base CadQuery: from z=0 to 0.035, center at z=0.0175
    # Want base to span z=0 to 0.035 in world
    # Visual center = 0 + 0 + 0.0175 = 0.0175
    base_socket.visual(
        mesh_from_cadquery(base_shape, "base_socket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=base_material,
        name="base_shell",
    )

    # Bearing ring: sits on top of base (base top is at z=0.035)
    # Bearing: thickness 2mm, from z=0.035 to 0.037
    bearing_shape = (
        cq.Workplane("XY")
        .circle(0.032)
        .circle(0.028)
        .extrude(0.002)
    )
    # Bearing CadQuery: from z=0 to 0.002, center at z=0.001
    # Want bearing center at z=0.036
    # Visual center = 0 + visual_origin + 0.001 = 0.036
    # So visual_origin = 0.036 - 0.001 = 0.035
    base_socket.visual(
        mesh_from_cadquery(bearing_shape, "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=bearing_material,
        name="bearing_ring",
    )

    # Locking knob mounted on base side at mid-height
    knob_shape = cq.Workplane("XY").circle(0.010).extrude(0.012)
    # Knob CadQuery: from z=0 to 0.012, center at z=0.006
    # Want knob center at world z=0.0175 (mid-height of base)
    # Visual center = 0 + visual_origin + 0.006 = 0.0175
    # So visual_origin z = 0.0175 - 0.006 = 0.0115
    base_socket.visual(
        mesh_from_cadquery(knob_shape, "locking_knob"),
        origin=Origin(xyz=(0.030, 0.0, 0.0115), rpy=(0.0, 1.57, 0.0)),
        material=knob_material,
        name="locking_knob",
    )

    # === CHILD PART: Pan Disk ===
    # Rotates around vertical Z axis at center of bearing (z=0.036)
    pan_disk = model.part("pan_disk")
    # Part frame will be at z=0.036 (from articulation)

    # Pan disk: 70mm diameter, 12mm thick
    # Sits on top of bearing (bearing top is at z=0.037)
    # Pan disk: from z=0.037 to 0.049, center at z=0.043
    pan_shape = cq.Workplane("XY").circle(0.035).extrude(0.012)
    # Pan CadQuery: from z=0 to 0.012, center at z=0.006
    # Want pan center at z=0.043 in world
    # Visual center = 0.036 (part frame) + visual_origin + 0.006 = 0.043
    # So visual_origin = 0.043 - 0.036 - 0.006 = 0.001
    pan_disk.visual(
        mesh_from_cadquery(pan_shape, "pan_disk_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=pan_material,
        name="pan_disk_body",
    )

    # === CHILD PART: Top Platform (offset from center for visibility) ===
    top_platform = model.part("top_platform")
    # Part frame at z=0.049 (top of pan disk, from articulation)

    # Platform: 50mm diameter, 8mm thick
    # Sits on top of pan disk (pan top is at z=0.049)
    # Platform: from z=0.049 to 0.057, center at z=0.053
    platform_shape = (
        cq.Workplane("XY")
        .circle(0.025)
        .extrude(0.008)
        .faces(">Z")
        .workplane()
        .circle(0.00325)
        .cutBlind(-0.008)
    )
    # Platform CadQuery: from z=0 to 0.008, center at z=0.004
    # Want platform center at z=0.053 in world
    # Visual center = 0.049 (part frame) + visual_origin + 0.004 = 0.053
    # So visual_origin = 0.053 - 0.049 - 0.004 = 0.0
    top_platform.visual(
        mesh_from_cadquery(platform_shape, "platform_body"),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),  # Offset +15mm in X for visibility of rotation
        material=platform_material,
        name="platform_body",
    )

    # === ARTICULATIONS ===
    # Pan disk rotates around vertical Z axis at center of bearing (z=0.036)
    model.articulation(
        "pan_yaw",
        ArticulationType.REVOLUTE,
        parent=base_socket,
        child=pan_disk,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14, effort=5.0, velocity=2.0),
    )

    # Platform fixed to pan disk at top of pan (z=0.049)
    model.articulation(
        "pan_to_platform",
        ArticulationType.FIXED,
        parent=pan_disk,
        child=top_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),  # 0.049 - 0.036 = 0.013 relative to pan
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_socket")
    pan = object_model.get_part("pan_disk")
    platform = object_model.get_part("top_platform")
    yaw_joint = object_model.get_articulation("pan_yaw")

    # Test 1: Verify yaw joint exists and has correct axis
    ctx.check("yaw_joint_exists", yaw_joint is not None, "Pan yaw joint not found")
    if yaw_joint is not None:
        ctx.check("yaw_axis_correct", yaw_joint.axis == (0.0, 0.0, 1.0),
                  f"Expected Z axis, got {yaw_joint.axis}")
        ctx.check("yaw_has_limits", yaw_joint.motion_limits is not None
                  and yaw_joint.motion_limits.lower is not None
                  and yaw_joint.motion_limits.upper is not None,
                  "Yaw joint missing motion limits")

    # Test 2: Check pan disk is supported by base (via bearing)
    ctx.expect_contact(base, pan, name="pan_disk_supported")

    # Test 3: Verify bearing seam - pan should be just above bearing
    # Check gap between base_shell (ends at z=0.035) and pan_disk_body (starts at z=0.037)
    # This gap (2mm) represents the visible bearing seam
    ctx.expect_gap(
        pan,
        base,
        axis="z",
        min_gap=0.001,  # At least 1mm gap (bearing thickness)
        max_gap=0.005,  # But not too large
        positive_elem="pan_disk_body",
        negative_elem="base_shell",
        name="bearing_seam_visible",
    )

    # Test 4: Check top platform is supported by pan disk
    ctx.expect_contact(pan, platform, name="platform_supported")

    # Test 5: Verify locking knob is mounted on base
    ctx.expect_contact(base, base, elem_b="locking_knob", name="knob_mounted")

    # Test 6: Test yaw rotation moves the platform (offset from center)
    with ctx.pose({yaw_joint: 0.0}):
        platform_aabb_0 = ctx.link_world_aabb(platform)
    with ctx.pose({yaw_joint: 1.57}):
        platform_aabb_90 = ctx.link_world_aabb(platform)
        if platform_aabb_0 and platform_aabb_90:
            # Platform is offset +15mm in X, so rotating 90deg should move it to Y
            ctx.check("platform_moves_in_yaw",
                      abs(platform_aabb_0[0][0] - platform_aabb_90[0][0]) > 0.01,
                      f"Platform didn't appear to move in yaw: {platform_aabb_0} vs {platform_aabb_90}")

    # Test 7: Check all parts have appropriate visuals
    ctx.check("base_has_visuals", len(base.visuals) >= 3, "Base missing visuals")
    ctx.check("pan_has_visual", len(pan.visuals) >= 1, "Pan disk missing visuals")
    ctx.check("platform_has_visual", len(platform.visuals) >= 1, "Platform missing visuals")

    # Allow overlap between base and pan (bearing interface)
    ctx.allow_overlap("base_socket", "pan_disk",
                       reason="Pan disk rotates on bearing seated on base")

    return ctx.report()


object_model = build_object_model()
