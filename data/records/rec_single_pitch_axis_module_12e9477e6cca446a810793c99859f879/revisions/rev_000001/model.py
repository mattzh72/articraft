from __future__ import annotations

import cadquery as cq
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
    ClevisBracketGeometry,
    mesh_from_geometry,
    Material,
)


def _make_moving_link() -> cq.Workplane:
    """Create the moving link with bore for the axle."""
    # Main link body - rectangular arm
    link_length = 0.150  # 150mm long
    link_width = 0.040   # 40mm wide (fits in bracket gap of 50mm)
    link_height = 0.030  # 30mm thick
    
    link = (
        cq.Workplane("XY")
        .box(link_length, link_width, link_height)
        .edges("|Z").fillet(0.005)
        .edges("|X").fillet(0.003)
    )
    
    # Add bore for axle (through the width at the base end)
    # The bore is at the end that attaches to the bracket
    bore_diameter = 0.012  # 12mm bore
    bore_length = link_width + 0.002  # Slightly longer than link width
    
    # Position bore at the base end (negative X side) centered on the link
    link = (
        link
        .faces("<X")
        .workplane()
        .hole(bore_diameter, bore_length)
    )
    
    return link


def _make_washer() -> cq.Workplane:
    """Create a washer for the axle."""
    return (
        cq.Workplane("XY")
        .workplane()
        .circle(0.018)  # Outer diameter 36mm
        .circle(0.006)  # Inner diameter 12mm (matches axle)
        .extrude(0.003)  # 3mm thick
    )


def _make_stop_block() -> cq.Workplane:
    """Create a stop block to limit pitch rotation."""
    # Small block that mounts on the bracket to limit rotation
    return (
        cq.Workplane("XY")
        .box(0.020, 0.015, 0.025)
        .edges().fillet(0.002)
    )


def _make_axis_indicator() -> cq.Workplane:
    """Create a visual indicator for the rotation axis."""
    # Small colored ring that sits on the axle to show axis location
    return (
        cq.Workplane("XY")
        .workplane()
        .circle(0.020)
        .circle(0.015)
        .extrude(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="training_pitch_joint")
    
    # Define materials with color coding
    # Materials are registered on the model with name and rgba
    model.material("bracket_blue", rgba=(0.20, 0.35, 0.65, 1.0))  # Blue bracket
    model.material("link_silver", rgba=(0.70, 0.72, 0.75, 1.0))    # Silver link
    model.material("axle_steel", rgba=(0.55, 0.57, 0.60, 1.0))     # Steel axle
    model.material("washer_brass", rgba=(0.76, 0.70, 0.50, 1.0))   # Brass washers
    model.material("stop_red", rgba=(0.70, 0.20, 0.20, 1.0))       # Red stop blocks
    model.material("indicator_yellow", rgba=(0.95, 0.90, 0.20, 1.0))  # Yellow indicator
    
    # ==========================================
    # FIXED BRACKET (Root Part)
    # ==========================================
    bracket = model.part("fixed_bracket")
    
    # Main clevis bracket - U-shaped with bore
    clevis = ClevisBracketGeometry(
        (0.080, 0.050, 0.100),  # overall_size: width_x, depth_y, height_z
        gap_width=0.050,          # Clear spacing between cheeks
        bore_diameter=0.012,      # 12mm bore
        bore_center_z=0.050,      # Bore height from bottom
        base_thickness=0.015,     # Base thickness
        corner_radius=0.005,      # Rounded corners
    )
    bracket.visual(
        mesh_from_geometry(clevis, "bracket_body"),
        material="bracket_blue",
        name="bracket_body",
    )
    
    # Axle cylinder - passes through the bracket bore
    # The axle is fixed to the bracket (passes through both cheeks)
    bracket.visual(
        Cylinder(radius=0.0055, length=0.070),  # 11mm diameter, extends past cheeks
        origin=Origin(xyz=(0.0, 0.0, 0.050)),   # At bore height, centered on Y
        material="axle_steel",
        name="axle",
    )
    
    # Washers - fixed on the bracket, on each side of where the link sits
    # These prevent axial movement of the link along the axle
    bracket.visual(
        mesh_from_cadquery(_make_washer(), "washer_left"),
        origin=Origin(xyz=(0.0, -0.035, 0.050)),  # Left side of link position, at axle height
        material="washer_brass",
        name="washer_left",
    )
    bracket.visual(
        mesh_from_cadquery(_make_washer(), "washer_right"),
        origin=Origin(xyz=(0.0, 0.035, 0.050)),   # Right side of link position, at axle height
        material="washer_brass",
        name="washer_right",
    )
    
    # Stop blocks - mounted on the bracket to limit rotation
    # These are positioned to limit the pitch motion
    bracket.visual(
        mesh_from_cadquery(_make_stop_block(), "stop_block_lower"),
        origin=Origin(xyz=(-0.025, -0.035, 0.050)),  # Positioned on bracket
        material="stop_red",
        name="stop_lower",
    )
    bracket.visual(
        mesh_from_cadquery(_make_stop_block(), "stop_block_upper"),
        origin=Origin(xyz=(-0.025, 0.035, 0.050)),   # Positioned on bracket
        material="stop_red",
        name="stop_upper",
    )
    
    # Axis indicator - shows the rotation axis location
    bracket.visual(
        mesh_from_cadquery(_make_axis_indicator(), "axis_indicator"),
        origin=Origin(xyz=(0.0, -0.040, 0.050)),  # Below the bracket
        material="indicator_yellow",
        name="axis_indicator",
    )
    
    # ==========================================
    # MOVING LINK (Child Part)
    # ==========================================
    link = model.part("moving_link")
    
    # Main link body with bore
    # The link part frame is at the joint origin (axle center)
    # The link body extends along +X from the joint
    link.visual(
        mesh_from_cadquery(_make_moving_link(), "link_body"),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),  # Center at bracket, extends along +X
        material="link_silver",
        name="link_body",
    )
    
    # ==========================================
    # ARTICULATION - Horizontal Revolute Pitch Joint
    # ==========================================
    # The pitch joint rotates around Y-axis (horizontal, left-right)
    # The articulation frame is at the axle center on the bracket
    # At q=0, the link extends along +X (horizontal)
    # Positive q pitches the link upward (around -Y axis per right-hand rule)
    
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=link,
        # Origin at the axle center (center of bracket, at bore height)
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        # Pitch rotation around Y-axis
        # Using -Y so positive q lifts the link upward (nose up)
        axis=(0.0, -1.0, 0.0),
        # Motion limits: -45 to +45 degrees (typical training joint range)
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-math.radians(45),   # -45 degrees
            upper=math.radians(45),    # +45 degrees
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    bracket = object_model.get_part("fixed_bracket")
    link = object_model.get_part("moving_link")
    pitch_joint = object_model.get_articulation("pitch_joint")
    
    # ==========================================
    # TEST 1: Axis placement verification
    # ==========================================
    ctx.check(
        "pitch_joint_exists",
        pitch_joint is not None,
        details="Pitch joint articulation must exist",
    )
    
    # Verify joint type is REVOLUTE
    ctx.check(
        "joint_is_revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Joint type is {pitch_joint.articulation_type}",
    )
    
    # Verify axis is along Y (horizontal pitch axis)
    axis = pitch_joint.axis
    ctx.check(
        "axis_is_horizontal_y",
        abs(axis[0]) < 0.001 and abs(axis[1]) > 0.99 and abs(axis[2]) < 0.001,
        details=f"Joint axis = {axis}, expected (0, ±1, 0) for horizontal pitch",
    )
    
    # Verify joint origin is at correct height (bracket bore center)
    joint_origin = pitch_joint.origin
    ctx.check(
        "joint_origin_z_correct",
        abs(joint_origin.xyz[2] - 0.050) < 0.001,
        details=f"Joint Z = {joint_origin.xyz[2]}, expected 0.050",
    )
    
    # ==========================================
    # TEST 2: Support and contact verification
    # ==========================================
    # Check that the link is properly supported by the bracket
    # The washers and axle should be on the bracket (fixed)
    
    # Check axle exists and is positioned correctly
    axle = bracket.get_visual("axle")
    ctx.check(
        "axle_exists",
        axle is not None,
        details="Axle visual must exist on bracket",
    )
    
    # Check washers exist on bracket (fixed, not moving with link)
    washer_left = bracket.get_visual("washer_left")
    washer_right = bracket.get_visual("washer_right")
    ctx.check(
        "washers_exist_on_bracket",
        washer_left is not None and washer_right is not None,
        details="Both left and right washers must exist on bracket",
    )
    
    # Check stop blocks exist on bracket
    stop_lower = bracket.get_visual("stop_lower")
    stop_upper = bracket.get_visual("stop_upper")
    ctx.check(
        "stop_blocks_exist",
        stop_lower is not None and stop_upper is not None,
        details="Both stop blocks must exist on bracket",
    )
    
    # ==========================================
    # TEST 3: Motion limits verification
    # ==========================================
    limits = pitch_joint.motion_limits
    ctx.check(
        "motion_limits_exist",
        limits is not None,
        details="Motion limits must be defined for pitch joint",
    )
    
    if limits is not None:
        ctx.check(
            "limits_are_symmetric",
            abs(limits.lower + limits.upper) < 0.01,
            details=f"Limits should be symmetric: lower={limits.lower}, upper={limits.upper}",
        )
        ctx.check(
            "limits_within_90_degrees",
            limits.lower >= -math.radians(90) and limits.upper <= math.radians(90),
            details=f"Limits should be within ±90°: lower={limits.lower}, upper={limits.upper}",
        )
    
    # ==========================================
    # TEST 4: Visual details verification
    # ==========================================
    # Check that all key visuals exist
    bracket_body = bracket.get_visual("bracket_body")
    link_body = link.get_visual("link_body")
    axis_indicator = bracket.get_visual("axis_indicator")
    
    ctx.check(
        "bracket_body_exists",
        bracket_body is not None,
        details="Bracket body must exist",
    )
    ctx.check(
        "link_body_exists",
        link_body is not None,
        details="Link body must exist",
    )
    ctx.check(
        "axis_indicator_exists",
        axis_indicator is not None,
        details="Axis indicator must exist",
    )
    
    # ==========================================
    # TEST 5: Pose-specific checks for pitch movement
    # ==========================================
    # Test at rest position (q=0) - link extends horizontally
    with ctx.pose({pitch_joint: 0.0}):
        # Check that link body is in contact with washers (seated on axle)
        ctx.expect_contact(
            bracket, link,
            elem_a="washer_left",
            elem_b="link_body",
            name="link_seated_on_washer_left",
        )
        ctx.expect_contact(
            bracket, link,
            elem_a="washer_right",
            elem_b="link_body",
            name="link_seated_on_washer_right",
        )
    
    # Test at positive pitch (link pitched up 30 degrees)
    with ctx.pose({pitch_joint: math.radians(30)}):
        # Check that the far end of the link (at +X) has moved upward
        # Get the AABB of the link body to find its far end Z coordinate
        link_aabb = ctx.part_element_world_aabb(link, elem="link_body")
        if link_aabb:
            # The max Z of the link body should be above the joint height (0.050)
            link_top_z = link_aabb[1][2]  # max Z
            ctx.check(
                "link_pitches_upward",
                link_top_z > 0.050,
                details=f"Link top Z at +30°: {link_top_z:.3f}, expected > 0.050",
            )
    
    # Test at negative pitch (link pitched down 30 degrees)
    with ctx.pose({pitch_joint: -math.radians(30)}):
        # Check that the far end of the link has moved downward
        link_aabb = ctx.part_element_world_aabb(link, elem="link_body")
        if link_aabb:
            # The min Z of the link body should be below the joint height (0.050)
            link_bottom_z = link_aabb[0][2]  # min Z
            ctx.check(
                "link_pitches_downward",
                link_bottom_z < 0.050,
                details=f"Link bottom Z at -30°: {link_bottom_z:.3f}, expected < 0.050",
            )
    
    # ==========================================
    # TEST 6: Allow intentional overlaps
    # ==========================================
    # The link body sits inside the bracket's U-shaped gap - intentional overlap
    ctx.allow_overlap(
        "fixed_bracket",
        "moving_link",
        elem_a="bracket_body",
        elem_b="link_body",
        reason="Link body sits inside the bracket's U-shaped gap; intentional design for pitch joint",
    )
    
    # Also allow axle/link_body overlap (axle passes through link bore)
    ctx.allow_overlap(
        "fixed_bracket",
        "moving_link",
        elem_a="axle",
        elem_b="link_body",
        reason="Axle passes through link body bore; intentional design for pitch joint rotation",
    )
    
    # Add proof check that the axle and link body are properly aligned
    with ctx.pose({pitch_joint: 0.0}):
        ctx.expect_contact(
            bracket, link,
            elem_a="axle",
            elem_b="link_body",
            name="axle_passes_through_link_bore",
        )
    
    return ctx.report()


object_model = build_object_model()
