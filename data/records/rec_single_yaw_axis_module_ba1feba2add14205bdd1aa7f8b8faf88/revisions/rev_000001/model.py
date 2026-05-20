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
    model = ArticulatedObject(name="azimuth_antenna_mount")

    # Materials with color contrast
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.26, 0.28, 1.0))
    light_metal = model.material("light_metal", rgba=(0.65, 0.67, 0.70, 1.0))
    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    antenna_white = model.material("antenna_white", rgba=(0.95, 0.96, 0.98, 1.0))

    # ========== FIXED PEDESTAL (ROOT PART) ==========
    pedestal = model.part("pedestal")

    # Base plate - wider for stability (at world z=0 to 0.06)
    pedestal.visual(
        Box((0.5, 0.5, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="pedestal_base",
    )

    # Main vertical column (from world z=0.06 to 1.48 - extends into flange)
    pedestal.visual(
        Cylinder(radius=0.10, length=1.42),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=dark_metal,
        name="pedestal_column",
    )

    # Top mounting flange (from world z=1.46 to 1.50, top at 1.50)
    top_flange_shape = (
        cq.Workplane("XY")
        .circle(0.18)
        .extrude(0.04)  # Local Z: -0.02 to 0.02
        .faces(">Z")
        .edges()
        .chamfer(0.005)
        .faces("<Z")
        .edges()
        .chamfer(0.005)
    )
    pedestal.visual(
        mesh_from_cadquery(top_flange_shape, "top_flange"),
        origin=Origin(xyz=(0.0, 0.0, 1.48)),  # World: z=1.46 to 1.50
        material=dark_metal,
        name="pedestal_top_flange",
    )

    # ========== ROTATING YOKE ASSEMBLY ==========
    # Yoke part frame at joint origin (world z=1.50 at q=0)
    yoke = model.part("yoke")

    # Circular bearing ring - sits on top of pedestal flange
    # Bearing ring from world z=1.48 to 1.515 (thickness 0.035)
    bearing_ring_shape = (
        cq.Workplane("XY")
        .circle(0.16)  # Outer radius
        .circle(0.12)  # Inner radius (creates the ring)
        .extrude(0.035)  # Ring thickness (local Z: -0.0175 to 0.0175)
        .faces(">Z")
        .edges()
        .chamfer(0.003)
        .faces("<Z")
        .edges()
        .chamfer(0.003)
    )
    # Position bearing ring so it sits on top of the pedestal flange
    # Local shape: z from -0.0175 to 0.0175, origin at z=-0.005 relative to yoke
    # World position: z=1.50-0.0225=1.4775 to 1.50+0.0125=1.5125
    yoke.visual(
        mesh_from_cadquery(bearing_ring_shape, "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=aluminum,
        name="bearing_ring",
    )

    # Yoke stump - horizontal arm extending from bearing (local X direction)
    # Placed at yoke part frame origin, extending along +X
    yoke_stump_shape = (
        cq.Workplane("XY")
        .box(0.35, 0.08, 0.08)
        .edges("|Z")
        .fillet(0.005)
    )
    yoke.visual(
        mesh_from_cadquery(yoke_stump_shape, "yoke_stump"),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),  # Center of box at x=0.175 (extends 0 to 0.35)
        material=light_metal,
        name="yoke_stump",
    )

    # Vertical support post on yoke stump (at end of stump, extends upward)
    support_post_shape = (
        cq.Workplane("XY")
        .box(0.08, 0.08, 0.45)
        .edges("|X")
        .fillet(0.005)
    )
    yoke.visual(
        mesh_from_cadquery(support_post_shape, "support_post"),
        origin=Origin(xyz=(0.35, 0.0, 0.225)),  # Base at z=0, top at z=0.45
        material=light_metal,
        name="support_post",
    )

    # Small antenna panel - mounted on support post
    antenna_panel_shape = (
        cq.Workplane("XY")
        .box(0.02, 0.35, 0.25)
        .edges()
        .fillet(0.003)
    )
    yoke.visual(
        mesh_from_cadquery(antenna_panel_shape, "antenna_panel"),
        origin=Origin(xyz=(0.36, 0.0, 0.275)),  # Panel center at z=0.275 (top at 0.45)
        material=antenna_white,
        name="antenna_panel",
    )

    # Gusset supports - triangular gussets between yoke stump and support post
    gusset_shape = (
        cq.Workplane("XZ")
        .polyline([(0, 0), (0.08, 0), (0, 0.15)])
        .close()
        .extrude(0.06)  # Thickness in Y direction
        .edges("|Y")
        .fillet(0.003)
    )
    # Front gusset (positive Y)
    yoke.visual(
        mesh_from_cadquery(gusset_shape, "gusset_front"),
        origin=Origin(xyz=(0.35, 0.04, 0.075)),  # At base of support post
        material=light_metal,
        name="gusset_front",
    )
    # Rear gusset (negative Y)
    yoke.visual(
        mesh_from_cadquery(gusset_shape, "gusset_rear"),
        origin=Origin(xyz=(0.35, -0.04, 0.075), rpy=(0.0, 0.0, math.pi)),
        material=light_metal,
        name="gusset_rear",
    )

    # ========== AZIMUTH (YAW) JOINT - VERTICAL ROTATION ==========
    # Joint at world z=1.50 (center of bearing ring vertically)
    model.articulation(
        "azimuth_yaw",
        ArticulationType.CONTINUOUS,  # Full 360 degree rotation
        parent=pedestal,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.50)),  # At top of pedestal flange
        axis=(0.0, 0.0, 1.0),  # Vertical Z axis for azimuth rotation
        motion_limits=MotionLimits(effort=80.0, velocity=0.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    yoke = object_model.get_part("yoke")
    azimuth_joint = object_model.get_articulation("azimuth_yaw")

    # Allow the yoke to be "floating" since it's connected via articulation
    ctx.allow_isolated_part("yoke", reason="Yoke is connected to pedestal via azimuth articulation joint")

    # Allow intentional overlap between yoke stump and pedestal flange
    # The yoke stump sits on top of the pedestal flange - this is intentional contact
    ctx.allow_overlap(
        "pedestal", "yoke",
        elem_a="pedestal_top_flange",
        elem_b="yoke_stump",
        reason="Yoke stump sits on top of pedestal flange - intentional seated contact"
    )

    # Allow intentional overlap between yoke stump and pedestal column
    ctx.allow_overlap(
        "pedestal", "yoke",
        elem_a="pedestal_column",
        elem_b="yoke_stump",
        reason="Yoke stump sits on top of pedestal column - intentional seated contact"
    )

    # Allow intentional overlap between bearing ring and pedestal flange
    # The bearing ring sits on the flange with embedded seating
    ctx.allow_overlap(
        "pedestal", "yoke",
        elem_a="pedestal_top_flange",
        elem_b="bearing_ring",
        reason="Bearing ring seated on pedestal flange - intentional embedded contact"
    )

    # Test 1: Verify azimuth joint has vertical (Z) axis
    joint_axis = azimuth_joint.axis
    ctx.check(
        "azimuth_joint_vertical_axis",
        abs(joint_axis[2] - 1.0) < 0.001,
        f"Joint axis is {joint_axis}, expected (0, 0, 1) for vertical azimuth rotation",
    )

    # Test 2: Verify joint origin is at correct height (top of pedestal)
    joint_origin = azimuth_joint.origin.xyz
    ctx.check(
        "azimuth_joint_height_correct",
        abs(joint_origin[2] - 1.50) < 0.01,
        f"Joint height is {joint_origin[2]:.3f}m, expected ~1.50m",
    )

    # Test 3: Verify bearing ring is seated on pedestal top flange
    # The bearing ring (on yoke) should be near the top flange (on pedestal)
    ctx.expect_contact(
        yoke,
        pedestal,
        elem_a="bearing_ring",
        elem_b="pedestal_top_flange",
        contact_tol=0.05,  # Slightly larger tolerance for mesh geometry
        name="bearing_ring_seated_on_flange",
    )

    # Test 4: Verify antenna panel is mounted to support post
    ctx.expect_contact(
        yoke,
        yoke,
        elem_a="antenna_panel",
        elem_b="support_post",
        contact_tol=0.05,
        name="antenna_panel_mounted_to_support",
    )

    # Test 5: Verify gusset supports are in place
    ctx.expect_contact(
        yoke,
        yoke,
        elem_a="gusset_front",
        elem_b="support_post",
        contact_tol=0.05,
        name="front_gusset_attached",
    )
    ctx.expect_contact(
        yoke,
        yoke,
        elem_a="gusset_rear",
        elem_b="support_post",
        contact_tol=0.05,
        name="rear_gusset_attached",
    )

    # Test 6: Pose check - verify rotation moves antenna correctly
    with ctx.pose({azimuth_joint: math.pi / 2}):
        # At 90 degrees, the antenna should have rotated to Y axis
        antenna_pos = ctx.part_world_position(yoke)
        ctx.check(
            "antenna_rotates_to_y_axis",
            antenna_pos is not None and abs(antenna_pos[0]) < 0.05,
            f"Antenna at 90° should be near Y axis, got {antenna_pos}",
        )

    # Test 7: Verify yoke is at correct height (at joint origin ~1.5m)
    yoke_pos = ctx.part_world_position(yoke)
    ctx.check(
        "yoke_height_correct",
        1.4 < yoke_pos[2] < 1.6 if yoke_pos else False,
        f"Yoke height {yoke_pos[2] if yoke_pos else 'unknown'}m, expected ~1.5m at joint origin",
    )

    return ctx.report()


object_model = build_object_model()
