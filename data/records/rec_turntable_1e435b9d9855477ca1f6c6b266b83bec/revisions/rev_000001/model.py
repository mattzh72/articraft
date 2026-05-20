from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_cylinder(radius: float, height: float) -> cq.Workplane:
    """Create a simple cylinder using CadQuery."""
    return cq.Workplane("XY").cylinder(height, radius)


def make_beveled_cylinder(radius: float, height: float, chamfer: float = 0.002) -> cq.Workplane:
    """Create a cylinder with chamfered top and bottom edges."""
    wp = make_cylinder(radius, height)
    # Chamfer top and bottom circumferential edges
    wp = wp.faces(">Z").edges().chamfer(chamfer)
    wp = wp.faces("<Z").edges().chamfer(chamfer)
    return wp


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_turntable")

    # --- Root part: Low round base ---
    base = model.part("base")
    # Base: 30cm diameter, 5cm tall, dark gray matte plastic
    base_geom = make_cylinder(radius=0.15, height=0.05)
    base.visual(
        mesh_from_cadquery(base_geom, "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),  # Center of cylinder at z=0.025 (bottom at 0, top at 0.05)
        name="base_shell",
        color=(0.2, 0.2, 0.2),  # Dark gray
    )
    # Power switch: small red box on front of base
    base.visual(
        Box((0.01, 0.005, 0.01)),  # 1cm deep, 0.5cm wide, 1cm tall
        origin=Origin(xyz=(0.15, 0.0, 0.025)),  # Front side (x+), center height of base
        name="power_switch",
        color=(1.0, 0.0, 0.0),  # Red
    )

    # --- Rotating part: Top disk with beveled edge ---
    top_disk = model.part("top_disk")
    # Top disk: 29.6cm diameter (2mm smaller than base), 3cm tall, light gray matte plastic
    top_disk_geom = make_beveled_cylinder(radius=0.148, height=0.03, chamfer=0.002)
    top_disk.visual(
        mesh_from_cadquery(top_disk_geom, "top_disk_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),  # Center of disk 1.5cm above joint frame (bottom at 0.05, top at 0.08)
        name="top_disk_shell",
        color=(0.8, 0.8, 0.8),  # Light gray
    )
    # Anti-slip pads: 4 small black rubber cylinders on top surface
    pad_radius = 0.01
    pad_height = 0.002
    pad_z_rel = 0.03 + pad_height / 2  # Top disk height is 0.03, pad center 1mm above top surface
    pad_positions = [
        (0.1, 0.0, pad_z_rel),
        (0.0, 0.1, pad_z_rel),
        (-0.1, 0.0, pad_z_rel),
        (0.0, -0.1, pad_z_rel),
    ]
    for i, (x, y, z) in enumerate(pad_positions):
        pad_geom = make_cylinder(radius=pad_radius, height=pad_height)
        top_disk.visual(
            mesh_from_cadquery(pad_geom, f"pad_{i}"),
            origin=Origin(xyz=(x, y, z)),
            name=f"anti_slip_pad_{i}",
            color=(0.0, 0.0, 0.0),  # Black
        )

    # --- Articulation: Continuous yaw joint (rotation around z-axis) ---
    # Joint frame at center of base top surface (0.05m above ground)
    model.articulation(
        "base_to_top",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_disk,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # In world frame: center of base top surface
        axis=(0.0, 0.0, 1.0),  # Yaw axis (z-axis)
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_disk = object_model.get_part("top_disk")
    joint = object_model.get_articulation("base_to_top")

    # 1. Verify main articulation is continuous yaw
    ctx.check(
        "main_joint_is_continuous_yaw",
        joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint type: {joint.articulation_type}, axis: {joint.axis}",
    )

    # 2. Verify top disk is connected to base via articulation
    ctx.check(
        "top_disk_is_connected",
        joint.parent == "base" and joint.child == "top_disk",
        details=f"Joint parent: {joint.parent}, child: {joint.child}",
    )

    # 3. Verify contact between base and top disk at rest pose
    ctx.expect_contact(base, top_disk, name="base_top_disk_contact")

    # 4. Verify anti-slip pads exist and are positioned correctly
    for i in range(4):
        pad_name = f"anti_slip_pad_{i}"
        pad = top_disk.get_visual(pad_name)
        ctx.check(f"pad_{i}_exists", pad is not None, details=f"Pad {pad_name} not found")
    with ctx.pose({joint: 0.0}):
        for i in range(4):
            pad_aabb = ctx.part_element_world_aabb(top_disk, elem=f"anti_slip_pad_{i}")
            # Pad top surface should be ~0.082m (0.08 + 0.002/2)
            ctx.check(
                f"pad_{i}_z_position",
                abs(pad_aabb[1][2] - 0.082) < 0.005,
                details=f"Pad {i} max z: {pad_aabb[1][2]}",
            )

    # 5. Verify power switch exists and is on front of base
    switch = base.get_visual("power_switch")
    ctx.check("power_switch_exists", switch is not None, details="Power switch not found")
    with ctx.pose({joint: 0.0}):
        switch_aabb = ctx.part_element_world_aabb(base, elem="power_switch")
        ctx.check(
            "switch_front_position",
            switch_aabb[1][0] > 0.14,  # Max x should be >0.14m (front of base)
            details=f"Switch max x: {switch_aabb[1][0]}",
        )

    # 6. Verify subtle seam (top disk smaller than base)
    with ctx.pose({joint: 0.0}):
        base_aabb = ctx.part_world_aabb(base)
        top_aabb = ctx.part_world_aabb(top_disk)
        ctx.check(
            "top_disk_smaller_than_base",
            top_aabb[1][0] < base_aabb[1][0],
            details=f"Base x max: {base_aabb[1][0]}, Top x max: {top_aabb[1][0]}",
        )

    # 7. Verify continuous rotation rotates top disk correctly
    # Check that anti-slip pad_0 rotates around z-axis
    # Pad 0 is at (0.1, 0, 0.031) relative to top disk part frame (world x+ at 0 rad)
    with ctx.pose({joint: 0.0}):
        pad0_aabb = ctx.part_element_world_aabb(top_disk, elem="anti_slip_pad_0")
        pad0_center = [(pad0_aabb[0][i] + pad0_aabb[1][i])/2 for i in range(3)]
        ctx.check(
            "pad0_position_at_0deg",
            abs(pad0_center[0] - 0.1) < 0.01 and abs(pad0_center[1]) < 0.01,
            details=f"Pad 0 center at 0deg: {pad0_center}",
        )
    with ctx.pose({joint: 1.5708}):  # 90 degrees
        pad0_aabb = ctx.part_element_world_aabb(top_disk, elem="anti_slip_pad_0")
        pad0_center = [(pad0_aabb[0][i] + pad0_aabb[1][i])/2 for i in range(3)]
        ctx.check(
            "pad0_position_at_90deg",
            abs(pad0_center[0]) < 0.01 and abs(pad0_center[1] - 0.1) < 0.01,
            details=f"Pad 0 center at 90deg: {pad0_center}",
        )
    with ctx.pose({joint: 3.14159}):  # 180 degrees
        pad0_aabb = ctx.part_element_world_aabb(top_disk, elem="anti_slip_pad_0")
        pad0_center = [(pad0_aabb[0][i] + pad0_aabb[1][i])/2 for i in range(3)]
        ctx.check(
            "pad0_position_at_180deg",
            abs(pad0_center[0] + 0.1) < 0.01 and abs(pad0_center[1]) < 0.01,
            details=f"Pad 0 center at 180deg: {pad0_center}",
        )

    return ctx.report()


object_model = build_object_model()
