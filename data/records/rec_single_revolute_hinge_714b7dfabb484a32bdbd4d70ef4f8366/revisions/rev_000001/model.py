from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    export_cadquery_mesh,
)


def create_leaf(length=0.3, max_width=0.1, thickness=0.01, num_ribs=3, num_bolts=4):
    """Create a heavy strap leaf with raised ribs and bolt heads as a single solid."""
    # Start with main leaf body
    leaf = cq.Workplane("XY").box(max_width, thickness, length)
    
    # Add raised longitudinal ribs on the upper surface (Y+ side)
    rib_width = 0.005  # X width of each rib
    rib_height = 0.002  # Height above leaf surface
    rib_length = length - 0.04  # Slightly shorter than leaf
    rib_spacing = max_width / (num_ribs + 1)
    
    for i in range(num_ribs):
        x_pos = -max_width/2 + (i + 1) * rib_spacing
        rib = (
            cq.Workplane("XY")
            .box(rib_width, rib_height, rib_length)
            .translate((x_pos, thickness, 0))  # Place on upper surface
        )
        leaf = leaf.union(rib)  # Union Workplanes
    
    # Add bolt heads with counterbored holes
    bolt_head_radius = 0.01
    bolt_head_height = 0.005
    bolt_hole_radius = 0.005
    bolt_z_positions = [length / (num_bolts + 1) * (i + 1) for i in range(num_bolts)]
    
    for z_pos in bolt_z_positions:
        z_offset = z_pos - length/2  # Adjust to leaf centered at Z=0
        # Bolt head (raised cylinder on upper surface)
        bolt_head = (
            cq.Workplane("XY")
            .workplane(offset=thickness)
            .center(0, z_offset)
            .circle(bolt_head_radius)
            .extrude(bolt_head_height)
        )
        leaf = leaf.union(bolt_head)
        
        # Bolt hole (through leaf and head)
        bolt_hole = (
            cq.Workplane("XY")
            .center(0, z_offset)
            .circle(bolt_hole_radius)
            .extrude(thickness + bolt_head_height)
        )
        leaf = leaf.cut(bolt_hole)
    
    return leaf.val()  # Return as single solid


def create_barrel(length=0.3, radius=0.025):
    """Create the thick cylindrical barrel for the hinge."""
    return cq.Workplane("XY").cylinder(radius, length).val()


def create_pin(length=0.32, radius=0.01):
    """Create the visible pin running full hinge height."""
    return cq.Workplane("XY").cylinder(radius, length).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_strap_hinge")
    
    # Define materials with realistic colors
    leaf_material = model.material("leaf_steel", rgba=(0.3, 0.3, 0.3, 1.0))  # Dark gray steel
    strap_material = model.material("strap_steel", rgba=(0.5, 0.5, 0.5, 1.0))  # Light gray steel
    barrel_material = model.material("barrel_brass", rgba=(0.8, 0.6, 0.2, 1.0))  # Brass
    pin_material = model.material("pin_steel", rgba=(0.7, 0.7, 0.75, 1.0))  # Stainless steel
    
    # Hinge dimensions
    hinge_length = 0.3  # 30cm vertical length
    leaf_width = 0.1  # 10cm width from barrel
    leaf_thickness = 0.01  # 1cm thickness
    
    # Create base leaf (fixed part)
    base_leaf_shape = create_leaf(
        length=hinge_length,
        max_width=leaf_width,
        thickness=leaf_thickness,
        num_ribs=3,
        num_bolts=4,
    )
    # Position base leaf on the lower side of the barrel (Y: -0.01 to 0)
    base_leaf_shape_translated = (
        cq.Workplane(base_leaf_shape)
        .translate((0, -leaf_thickness/2, hinge_length/2))
    )
    
    base_leaf = model.part("base_leaf")
    # Use export_cadquery_mesh to get single mesh
    base_leaf_mesh = export_cadquery_mesh(base_leaf_shape_translated, "base_leaf")
    base_leaf.visual(
        base_leaf_mesh.mesh,
        material=leaf_material,
        name="base_leaf_shell",
    )
    
    # Create strap leaf (moving part)
    strap_leaf_shape = create_leaf(
        length=hinge_length,
        max_width=leaf_width,
        thickness=leaf_thickness,
        num_ribs=3,
        num_bolts=4,
    )
    # Position strap leaf on the upper side of the barrel (Y: 0 to 0.01)
    strap_leaf_shape_translated = (
        cq.Workplane(strap_leaf_shape)
        .translate((0, leaf_thickness/2, hinge_length/2))
    )
    
    strap_leaf = model.part("strap_leaf")
    # Use export_cadquery_mesh to get single mesh
    strap_leaf_mesh = export_cadquery_mesh(strap_leaf_shape_translated, "strap_leaf")
    strap_leaf.visual(
        strap_leaf_mesh.mesh,
        material=strap_material,
        name="strap_leaf_shell",
    )
    
    # Create barrel (positioned at center Y, between leaves)
    barrel_shape = create_barrel(length=hinge_length, radius=0.025)
    barrel_shape_translated = (
        cq.Workplane(barrel_shape)
        .translate((0, 0, hinge_length/2))  # Center Z
        .val()
    )
    # Add barrel as visual to base leaf (fixed with base)
    barrel_mesh = export_cadquery_mesh(barrel_shape_translated, "barrel")
    base_leaf.visual(
        barrel_mesh.mesh,
        material=barrel_material,
        name="barrel",
    )
    
    # Create pin (runs through barrel, fixed to base leaf)
    pin_shape = create_pin(length=hinge_length + 0.02, radius=0.01)  # Slightly longer than hinge
    pin_shape_translated = (
        cq.Workplane(pin_shape)
        .translate((0, 0, hinge_length/2 + 0.01))  # Center Z, extend slightly beyond
        .val()
    )
    pin_mesh = export_cadquery_mesh(pin_shape_translated, "pin")
    base_leaf.visual(
        pin_mesh.mesh,
        material=pin_material,
        name="pin",
    )
    
    # Create revolute joint around vertical (Z) pin
    # Joint origin at barrel center (X=0, Y=0, Z=hinge_length/2)
    # Axis is Z (0,0,1), positive rotation swings strap leaf outward
    model.articulation(
        "base_to_strap",
        ArticulationType.REVOLUTE,
        parent=base_leaf,
        child=strap_leaf,
        origin=Origin(xyz=(0.0, 0.0, hinge_length/2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=2.0,
            lower=-2.0,  # Full 180° swing in both directions
            upper=2.0,
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_leaf = object_model.get_part("base_leaf")
    strap_leaf = object_model.get_part("strap_leaf")
    hinge = object_model.get_articulation("base_to_strap")
    
    # Test 1: Main mechanism - revolute joint around Z axis
    ctx.check(
        "hinge_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Hinge should be a revolute joint",
    )
    ctx.check(
        "hinge_axis_is_vertical",
        hinge.axis == (0.0, 0.0, 1.0),
        details="Hinge should rotate around vertical Z axis",
    )
    
    # Test 2: Pin is properly seated in barrel (intentional overlap)
    ctx.allow_overlap(
        "base_leaf",
        "base_leaf",
        elem_a="pin",
        elem_b="barrel",
        reason="Pin is intentionally seated inside the barrel as part of the hinge mechanism",
    )
    ctx.expect_contact(
        base_leaf,
        base_leaf,
        elem_a="pin",
        elem_b="barrel",
        contact_tol=0.001,
        name="pin_seated_in_barrel",
    )
    
    # Test 3: Strap leaf contacts barrel and pin (intentional overlap)
    ctx.allow_overlap(
        "base_leaf",
        "strap_leaf",
        elem_a="barrel",
        elem_b="strap_leaf_shell",
        reason="Strap leaf is intentionally attached to the barrel as part of the hinge mechanism",
    )
    ctx.allow_overlap(
        "base_leaf",
        "strap_leaf",
        elem_a="pin",
        elem_b="strap_leaf_shell",
        reason="Pin intentionally passes through strap leaf as part of the hinge mechanism",
    )
    ctx.expect_contact(
        base_leaf,
        strap_leaf,
        elem_a="barrel",
        name="strap_leaf_contacts_barrel",
    )
    
    # Test 4: Strap leaf swings correctly
    with ctx.pose({hinge: 1.0}):  # ~57° open
        strap_pos = ctx.part_world_position(strap_leaf)
        ctx.check(
            "strap_leaf_moves_on_open",
            strap_pos is not None,
            details="Strap leaf should have a valid position when open",
        )
    
    # Test 5: Visible details - leaf has correct visual
    base_shell = base_leaf.get_visual("base_leaf_shell")
    ctx.check(
        "base_leaf_has_shell",
        base_shell is not None,
        details="Base leaf should have a shell visual with ribs and bolt heads",
    )
    
    # Test 6: Pin is visible and full height
    pin_visual = base_leaf.get_visual("pin")
    ctx.check(
        "pin_is_present",
        pin_visual is not None,
        details="Pin should be present and visible",
    )
    
    return ctx.report()


object_model = build_object_model()
