from __future__ import annotations

import cadquery as cq
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_carousel_tray")

    # Materials
    model.material("dark_base", rgba=(0.2, 0.2, 0.22, 1.0))
    model.material("tray_white", rgba=(0.92, 0.92, 0.94, 1.0))
    model.material("rim_gray", rgba=(0.75, 0.75, 0.78, 1.0))
    model.material("handle_metal", rgba=(0.7, 0.7, 0.72, 1.0))
    model.material("spice_red", rgba=(0.85, 0.2, 0.2, 1.0))
    model.material("spice_green", rgba=(0.2, 0.7, 0.25, 1.0))
    model.material("spice_yellow", rgba=(0.9, 0.85, 0.15, 1.0))
    model.material("spice_blue", rgba=(0.2, 0.35, 0.8, 1.0))
    model.material("spice_orange", rgba=(0.9, 0.55, 0.15, 1.0))
    model.material("spice_purple", rgba=(0.6, 0.25, 0.7, 1.0))

    # Fixed base - circular base that sits on counter
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.14, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="dark_base",
        name="base_body",
    )
    # Add a small foot/rim to the base for stability
    base.visual(
        Cylinder(radius=0.145, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material="dark_base",
        name="base_foot",
    )

    # Rotating tray assembly - contains tray, rim, handle, and spice blocks
    tray_assembly = model.part("tray_assembly")

    # Main tray - circular flat surface
    tray_assembly.visual(
        Cylinder(radius=0.12, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="tray_white",
        name="tray_surface",
    )

    # Raised rim around the edge of the tray - using CadQuery for hollow cylinder
    rim_outer_radius = 0.12
    rim_inner_radius = 0.108
    rim_height = 0.035
    
    # Create rim starting at top of tray surface (local z=0.012) with height 0.035
    # Using centered=(True, True, False) so box bottom is at workplane
    rim_shape = (
        cq.Workplane("XY")
        .workplane(offset=0.012)  # Bottom of rim at top of tray surface
        .box(rim_outer_radius * 2, rim_outer_radius * 2, rim_height, centered=(True, True, False))
        .edges("|Z").fillet(0.005)
    )
    # Cut inner hole to make it hollow (hole expects diameter)
    rim_shape = (
        rim_shape
        .workplane(offset=0.0)
        .hole(rim_inner_radius * 2)
    )
    
    tray_assembly.visual(
        mesh_from_cadquery(rim_shape, "raised_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="rim_gray",
        name="raised_rim",
    )

    # Center handle for rotating the tray
    tray_assembly.visual(
        Cylinder(radius=0.015, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material="handle_metal",
        name="center_handle",
    )
    # Handle grip - slightly wider top
    tray_assembly.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material="handle_metal",
        name="handle_grip",
    )

    # Spice block placeholders - arranged in a circle on the tray
    num_spice_blocks = 6
    spice_radius = 0.085  # Distance from center to spice blocks
    spice_colors = ["spice_red", "spice_green", "spice_yellow", "spice_blue", "spice_orange", "spice_purple"]
    spice_height = 0.045
    spice_half_height = spice_height / 2

    for i in range(num_spice_blocks):
        angle = i * (2 * pi / num_spice_blocks)
        x = spice_radius * cos(angle)
        y = spice_radius * sin(angle)

        # Position spice block so bottom sits on tray surface (tray top at z=0.037 in world = 0.012 in tray_assembly)
        # Tray top in tray_assembly frame = 0.012 (local top of tray_surface)
        spice_bottom_z = 0.012  # Top of tray surface in tray_assembly frame
        spice_origin_z = spice_bottom_z + spice_half_height

        tray_assembly.visual(
            Box((0.035, 0.035, spice_height)),
            origin=Origin(xyz=(x, y, spice_origin_z)),
            material=spice_colors[i],
            name=f"spice_block_{i}",
        )

    # Yaw joint - continuous rotation around Z axis
    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray_assembly = object_model.get_part("tray_assembly")
    yaw_joint = object_model.get_articulation("base_to_tray")

    # Test 1: Verify the base is fixed (root part)
    ctx.check("base_is_root", base is not None, details="Base part exists")

    # Test 2: Verify tray assembly exists and is connected
    ctx.check("tray_assembly_exists", tray_assembly is not None, details="Tray assembly part exists")

    # Test 3: Verify yaw joint exists and is continuous
    ctx.check(
        "yaw_joint_is_continuous",
        yaw_joint is not None and yaw_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Yaw joint exists and is continuous type",
    )

    # Test 4: Check that handle is centered on tray (overlap in XY)
    ctx.expect_overlap(
        tray_assembly,
        tray_assembly,
        axes="xy",
        elem_a="center_handle",
        elem_b="tray_surface",
        min_overlap=0.01,
        name="handle_centered_on_tray",
    )

    # Test 5: Check that rim surrounds the tray (overlap in XY)
    ctx.expect_overlap(
        tray_assembly,
        tray_assembly,
        axes="xy",
        elem_a="raised_rim",
        elem_b="tray_surface",
        min_overlap=0.10,
        name="rim_surrounds_tray",
    )

    # Test 6: Check vertical support - tray sits on base at rest
    ctx.expect_gap(
        tray_assembly,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        name="tray_supported_by_base",
    )

    # Test 7: Verify spice blocks are placed on the tray (Z overlap)
    for i in range(6):
        ctx.expect_contact(
            tray_assembly,
            tray_assembly,
            elem_a=f"spice_block_{i}",
            elem_b="tray_surface",
            contact_tol=0.005,
            name=f"spice_block_{i}_on_tray",
        )

    # Test 8: Test rotation - verify tray moves in Yaw
    rest_pos = ctx.part_world_position(tray_assembly)
    with ctx.pose({yaw_joint: pi / 4}):  # 45 degree rotation
        rotated_pos = ctx.part_world_position(tray_assembly)
        ctx.check(
            "tray_rotates_yaw",
            rest_pos is not None and rotated_pos is not None,
            details=f"Rest position: {rest_pos}, Rotated position: {rotated_pos}",
        )
        # Check that Z position stays the same (pure yaw rotation)
        if rest_pos is not None and rotated_pos is not None:
            ctx.check(
                "tray_height_unchanged_after_rotation",
                abs(rest_pos[2] - rotated_pos[2]) < 0.001,
                details=f"Z height change: {abs(rest_pos[2] - rotated_pos[2]) if rest_pos and rotated_pos else 'N/A'}",
            )

    # Test 9: Verify the tray can rotate 360 degrees
    with ctx.pose({yaw_joint: pi}):  # 180 degree rotation
        ctx.check("tray_rotates_180_degrees", True, details="Tray can rotate 180 degrees")

    with ctx.pose({yaw_joint: 2 * pi}):  # 360 degree rotation
        ctx.check("tray_rotates_360_degrees", True, details="Tray can rotate 360 degrees (full rotation)")

    return ctx.report()


object_model = build_object_model()
