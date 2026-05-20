from __future__ import annotations
from typing import Tuple
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Origin,
    MotionLimits,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_pan_base")

    # --------------------------
    # Fixed pedestal (root part)
    # --------------------------
    pedestal = model.part("pedestal")

    # Main pedestal body: dark gray cylinder, 0.15m diameter, 0.25m tall
    pedestal.visual(
        Cylinder(radius=0.075, height=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),  # Center at z=0.125 → bottom at z=0, top at z=0.25
        material=Material(name="pedestal_material", color=(0.3, 0.3, 0.3)),
        name="pedestal_body",
    )

    # Cable gland: small black cylinder on the +x side of the pedestal
    pedestal.visual(
        Cylinder(radius=0.01, height=0.03),
        origin=Origin(xyz=(0.085, 0.0, 0.1)),  # 0.085m from center (0.075 + 0.01), halfway up
        material=Material(name="cable_gland_material", color=(0.1, 0.1, 0.1)),
        name="cable_gland",
    )

    # --------------------------
    # Rotating top assembly
    # --------------------------
    top_assembly = model.part("top_assembly")

    # Top disk: light gray cylinder with filleted edges, 0.12m diameter, 0.02m thick
    # Sits flush on pedestal: bottom at z=0 in top_assembly frame → world z=0.25
    top_disk_geom = (
        cq.Workplane("XY")
        .circle(0.06)  # Radius 0.06m (0.12m diameter)
        .extrude(0.02)  # Thickness 0.02m
        .faces(">Z")    # Top face
        .edges()
        .fillet(0.002)  # 2mm fillet on top edge
        .faces("<Z")    # Bottom face
        .edges()
        .fillet(0.002)  # 2mm fillet on bottom edge
    )
    top_assembly.visual(
        mesh_from_cadquery(top_disk_geom, "top_disk"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Bottom of disk at z=0 in top_assembly → world z=0.25 (flush with pedestal)
        material=Material(name="top_disk_material", color=(0.7, 0.7, 0.7)),
        name="top_disk",
    )

    # Central bearing collar: black cylinder, 0.04m diameter, 0.03m tall
    # Sits on top disk: bottom at z=0.02 in top_assembly frame
    top_assembly.visual(
        Cylinder(radius=0.02, height=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),  # Center at z=0.035 → bottom at z=0.02 (top of top_disk)
        material=Material(name="bearing_collar_material", color=(0.1, 0.1, 0.1)),
        name="bearing_collar",
    )

    # Bolt ring: silver hollow cylinder (ring) around bearing collar
    # Sits on top disk, outside bearing collar
    bolt_ring_geom = (
        cq.Workplane("XY")
        .circle(0.025)  # Outer radius
        .circle(0.02)   # Inner radius (matches bearing collar)
        .extrude(0.005)  # Thickness
    )
    top_assembly.visual(
        mesh_from_cadquery(bolt_ring_geom, "bolt_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),  # Center at z=0.0225 → bottom at z=0.02 (top of top_disk)
        material=Material(name="bolt_ring_material", color=(0.8, 0.8, 0.8)),
        name="bolt_ring",
    )

    # Camera mounting plate: dark gray rectangular plate on top of bearing collar and bolt ring
    # Bearing collar top at z=0.05 in top_assembly frame, so camera plate bottom at z=0.05
    top_assembly.visual(
        Box((0.08, 0.06, 0.01)),  # 80mm wide, 60mm deep, 10mm thick
        origin=Origin(xyz=(0.0, 0.0, 0.055)),  # Center at z=0.055 → bottom at z=0.05 (on top of bearing collar)
        material=Material(name="camera_plate_material", color=(0.3, 0.3, 0.3)),
        name="camera_plate",
    )

    # --------------------------
    # Yaw articulation (vertical axis)
    # --------------------------
    model.articulation(
        name="yaw_joint",
        articulation_type=ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=top_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),  # Joint at center of pedestal top
        axis=(0.0, 0.0, 1.0),  # Vertical z-axis, positive = counter-clockwise yaw
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),  # 10Nm effort, 5rad/s max speed
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    top_assembly = object_model.get_part("top_assembly")
    yaw_joint = object_model.get_articulation("yaw_joint")

    # --------------------------
    # Main mechanism tests
    # --------------------------
    ctx.check(
        name="yaw_joint_is_continuous",
        ok=yaw_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected CONTINUOUS, got {yaw_joint.articulation_type}",
    )

    ctx.check(
        name="yaw_joint_axis_is_vertical",
        ok=yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"Expected (0,0,1), got {yaw_joint.axis}",
    )

    # --------------------------
    # Support and contact tests
    # --------------------------
    with ctx.pose({yaw_joint: 0.0}):
        # Top assembly sits flush on pedestal (z-gap ~0)
        ctx.expect_gap(
            positive_link=top_assembly,
            negative_link=pedestal,
            axis="z",
            max_penetration=0.001,
            min_gap=-0.001,
            max_gap=0.001,
            name="top_assembly_flushes_with_pedestal",
        )

        # Top disk overlaps with pedestal footprint in XY
        ctx.expect_overlap(
            link_a=top_assembly,
            link_b=pedestal,
            axes="xy",
            min_overlap=0.05,
            name="top_disk_overlaps_pedestal_footprint",
        )

    # Rotation test: top assembly origin stays centered during yaw
    with ctx.pose({yaw_joint: math.pi / 2}):  # 90 degrees
        top_pos = ctx.part_world_position(top_assembly)
        ctx.check(
            name="top_assembly_centered_after_rotation",
            ok=abs(top_pos[0]) < 0.001 and abs(top_pos[1]) < 0.001,
            details=f"Top assembly position {top_pos[:2]} not centered after rotation",
        )

    # --------------------------
    # Visible details tests
    # --------------------------
    # Check all required visuals exist
    pedestal_visual_names = {v.name for v in pedestal.visuals}
    ctx.check(
        name="pedestal_has_required_visuals",
        ok={"pedestal_body", "cable_gland"} <= pedestal_visual_names,
        details=f"Missing pedestal visuals: {pedestal_visual_names}",
    )

    top_visual_names = {v.name for v in top_assembly.visuals}
    ctx.check(
        name="top_assembly_has_required_visuals",
        ok={"top_disk", "bearing_collar", "bolt_ring", "camera_plate"} <= top_visual_names,
        details=f"Missing top assembly visuals: {top_visual_names}",
    )

    # Proportion checks (real-world sizes)
    # Pedestal height check
    pedestal_aabb = ctx.part_world_aabb(pedestal)
    pedestal_height = pedestal_aabb[1][2] - pedestal_aabb[0][2]
    ctx.check(
        name="pedestal_height_realistic",
        ok=abs(pedestal_height - 0.25) < 0.01,
        details=f"Pedestal height {pedestal_height:.3f}m not ~0.25m",
    )

    # Top disk diameter check (width in x-axis = diameter)
    top_disk_aabb = ctx.part_element_world_aabb(top_assembly, elem="top_disk")
    top_disk_diameter = top_disk_aabb[1][0] - top_disk_aabb[0][0]
    ctx.check(
        name="top_disk_diameter_realistic",
        ok=abs(top_disk_diameter - 0.12) < 0.01,
        details=f"Top disk diameter {top_disk_diameter:.3f}m not ~0.12m",
    )

    # Camera plate sits on bearing collar (z-gap check)
    with ctx.pose({yaw_joint: 0.0}):
        # Check camera plate is above bearing collar
        camera_plate_aabb = ctx.part_element_world_aabb(top_assembly, elem="camera_plate")
        bearing_collar_aabb = ctx.part_element_world_aabb(top_assembly, elem="bearing_collar")
        camera_plate_bottom = camera_plate_aabb[0][2]
        bearing_collar_top = bearing_collar_aabb[1][2]
        ctx.check(
            name="camera_plate_sits_on_bearing_collar",
            ok=abs(camera_plate_bottom - bearing_collar_top) < 0.001,
            details=f"Camera plate bottom {camera_plate_bottom:.3f} not flush with bearing collar top {bearing_collar_top:.3f}",
        )

        # Bolt ring contained within bearing collar footprint in XY
        bolt_ring_aabb = ctx.part_element_world_aabb(top_assembly, elem="bolt_ring")
        # Bolt ring outer radius is 0.025, bearing collar radius is 0.02
        # So bolt ring extends beyond bearing collar - this is correct for a bolt ring
        # Check that bolt ring is centered on bearing collar
        bolt_ring_center_x = (bolt_ring_aabb[0][0] + bolt_ring_aabb[1][0]) / 2
        bolt_ring_center_y = (bolt_ring_aabb[0][1] + bolt_ring_aabb[1][1]) / 2
        ctx.check(
            name="bolt_ring_centered_on_bearing_collar",
            ok=abs(bolt_ring_center_x) < 0.001 and abs(bolt_ring_center_y) < 0.001,
            details=f"Bolt ring center ({bolt_ring_center_x:.3f}, {bolt_ring_center_y:.3f}) not at origin",
        )

    # Cable gland positioned on the side of pedestal
    cable_gland_aabb = ctx.part_element_world_aabb(pedestal, elem="cable_gland")
    cable_gland_center_x = (cable_gland_aabb[0][0] + cable_gland_aabb[1][0]) / 2
    ctx.check(
        name="cable_gland_on_pedestal_side",
        ok=cable_gland_center_x > 0.08,  # Should be on the +x side
        details=f"Cable gland x-center {cable_gland_center_x:.3f} not on pedestal side",
    )

    return ctx.report()


object_model = build_object_model()
