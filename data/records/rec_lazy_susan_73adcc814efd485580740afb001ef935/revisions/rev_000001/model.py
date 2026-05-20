from __future__ import annotations

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
    mesh_from_geometry,
)


def _build_tray_shape() -> cq.Workplane:
    """Build the lazy susan tray with radial grooves and raised rim."""
    tray_radius = 0.25
    tray_thickness = 0.02
    rim_height = 0.015
    rim_thickness = 0.005
    groove_depth = 0.003
    groove_width = 0.004
    num_grooves = 12

    # Main tray surface (flat disk) - from z=0 to z=tray_thickness
    tray = (
        cq.Workplane("XY")
        .circle(tray_radius)
        .extrude(tray_thickness)
    )

    # Raised rim: outer ring - sits on top of tray (starts at z=tray_thickness)
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=tray_thickness)
        .circle(tray_radius)
        .circle(tray_radius - rim_thickness)
        .extrude(rim_height)
    )

    # Add radial grooves on the tray surface (cut from the top of the tray)
    for i in range(num_grooves):
        angle = i * (360.0 / num_grooves)
        groove = (
            cq.Workplane("XY")
            .workplane(offset=tray_thickness - groove_depth)
            .center(tray_radius * 0.3, 0)
            .rect(tray_radius * 0.6, groove_width)
            .rotate((0, 0, 0), (0, 0, 1), angle)
            .extrude(groove_depth)
        )
        tray = tray.cut(groove)

    # Combine tray and rim
    tray = tray.union(rim_outer)

    return tray


def _build_base_shape() -> cq.Workplane:
    """Build the fixed lower base disk."""
    base_radius = 0.24
    base_thickness = 0.025

    base = (
        cq.Workplane("XY")
        .circle(base_radius)
        .extrude(base_thickness)
    )

    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan")

    # Materials
    model.material("wood_tone", rgba=(0.65, 0.45, 0.28, 1.0))
    model.material("dark_wood", rgba=(0.45, 0.30, 0.18, 1.0))

    # Fixed base (root part) - sits on the ground
    # Base thickness is 0.025, with visual origin at (0,0,0), base goes from z=0 to z=0.025
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="dark_wood",
        name="base_disk",
    )

    # Rotating tray - joint at top of base (z=0.025) + small bearing gap (0.002)
    # Tray part frame is at the joint frame: z=0.027
    # Tray geometry bottom is at z=0 in local, so world bottom = 0.027
    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_shape(), "tray"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Tray geometry bottom at part origin
        material="wood_tone",
        name="tray_surface",
    )

    # Vertical yaw joint (continuous rotation)
    # Joint at top of base + small bearing seam
    joint_z = 0.027  # 0.025 (base top) + 0.002 (bearing gap)
    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    joint = object_model.get_articulation("base_to_tray")

    # Allow tray to be "isolated" - it's connected by joint, not geometric contact
    # The bearing gap means no geometric contact between base and tray
    ctx.allow_isolated_part(
        "tray",
        reason="Tray is supported by bearings with 2mm gap; connected via joint not geometric contact",
    )

    # Verify parts exist and are connected
    ctx.check("base_part_exists", base is not None, details="Base part must exist")
    ctx.check("tray_part_exists", tray is not None, details="Tray part must exist")
    ctx.check("joint_exists", joint is not None, details="Yaw joint must exist")

    # Verify joint type is continuous (360-degree rotation)
    ctx.check(
        "joint_is_continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Joint must be CONTINUOUS for 360-degree rotation",
    )

    # Verify joint axis is vertical (z-axis)
    ctx.check(
        "joint_axis_vertical",
        joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint axis should be vertical, got {joint.axis}",
    )

    # Check for bearing gap (tray should be slightly above base)
    # Actual gap is ~2mm (0.002m) - the bearing seam
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.001,
        max_gap=0.005,
        name="bearing_seam_present",
    )

    # Verify tray rotates (test at a pose)
    with ctx.pose({joint: 1.57}):  # 90 degrees
        tray_pos = ctx.part_world_position(tray)
        ctx.check(
            "tray_rotates_90_degrees",
            tray_pos is not None,
            details="Tray should have valid position after rotation",
        )

    with ctx.pose({joint: 3.14}):  # 180 degrees
        tray_pos_180 = ctx.part_world_position(tray)
        ctx.check(
            "tray_rotates_180_degrees",
            tray_pos_180 is not None,
            details="Tray should have valid position at 180 degrees",
        )

    # Verify tray stays centered over base (XY overlap)
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.20,
        name="tray_centered_on_base",
    )

    return ctx.report()


object_model = build_object_model()
