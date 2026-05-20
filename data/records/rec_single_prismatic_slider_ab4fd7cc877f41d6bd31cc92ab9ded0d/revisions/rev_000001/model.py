from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _build_base_plate(base_length=0.6, base_width=0.3, base_thickness=0.02):
    """Build the base plate with bolt holes."""
    plate = (
        cq.Workplane("XY")
        .box(base_length, base_width, base_thickness)
        .edges("|Z").fillet(0.003)
    )
    
    # Add bolt holes (M6 bolts)
    bolt_diameter = 0.006
    counterbore_diameter = 0.011
    counterbore_depth = 0.004
    
    bolt_positions = [
        (-base_length/2 + 0.03, -base_width/2 + 0.03),
        (-base_length/2 + 0.03, base_width/2 - 0.03),
        (base_length/2 - 0.03, -base_width/2 + 0.03),
        (base_length/2 - 0.03, base_width/2 - 0.03),
    ]
    
    for x, y in bolt_positions:
        plate = (
            plate.faces(">Z")
            .workplane(centerOption="CenterOfBoundBox")
            .moveTo(x, y)
            .cboreHole(bolt_diameter, counterbore_diameter, counterbore_depth)
        )
    
    return plate


def _build_rail(rail_length=0.6, rail_width=0.025, rail_height=0.025):
    """Build a simplified rail profile."""
    rail = (
        cq.Workplane("XY")
        .box(rail_length, rail_width, rail_height)
        .edges("|X").fillet(0.003)
    )
    return rail


def _build_end_cap():
    """Build an end cap for the rail."""
    cap = (
        cq.Workplane("XY")
        .box(0.03, 0.04, 0.03)
        .edges("|Z").fillet(0.002)
    )
    return cap


def _build_bolt_head():
    """Build a bolt head (hex head)."""
    head_height = 0.004
    head_radius = 0.005
    
    head = (
        cq.Workplane("XY")
        .cylinder(head_height, head_radius)
    )
    return head


def _build_carriage(carriage_length=0.15, carriage_width=0.25, carriage_height=0.05):
    """Build the sliding carriage with bearing block mounts."""
    carriage = (
        cq.Workplane("XY")
        .box(carriage_length, carriage_width, carriage_height)
        .edges("|Z").fillet(0.003)
    )
    
    # Add bolt holes for mounting (M4 bolts)
    bolt_diameter = 0.004
    counterbore_diameter = 0.008
    counterbore_depth = 0.003
    
    bolt_positions = [
        (-carriage_length/4, -carriage_width/4),
        (-carriage_length/4, carriage_width/4),
        (carriage_length/4, -carriage_width/4),
        (carriage_length/4, carriage_width/4),
    ]
    
    for x, y in bolt_positions:
        carriage = (
            carriage.faces(">Z")
            .workplane(centerOption="CenterOfBoundBox")
            .moveTo(x, y)
            .cboreHole(bolt_diameter, counterbore_diameter, counterbore_depth)
        )
    
    return carriage


def _build_bearing_block():
    """Build a bearing block that interfaces with the rail."""
    length = 0.03
    width = 0.04
    height = 0.03
    
    block = (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z").fillet(0.002)
    )
    
    # Add V-groove for rail interface (simplified as a chamfer)
    block = block.faces(">Z").chamfer(0.005)
    
    return block


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage")
    
    # Materials
    model.material("aluminum", rgba=(0.75, 0.77, 0.80, 1.0))  # Anodized aluminum
    model.material("dark_aluminum", rgba=(0.50, 0.52, 0.55, 1.0))  # Darker aluminum
    model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))  # Steel
    model.material("black", rgba=(0.10, 0.10, 0.12, 1.0))  # Black for bolts
    
    # === BASE (root part with rails) ===
    base = model.part("base")
    
    # Base plate
    base_plate_shape = _build_base_plate()
    base.visual(
        mesh_from_cadquery(base_plate_shape, "base_plate"),
        material="aluminum",
        name="base_plate",
    )
    
    # Rails (two parallel rails along X-axis, positioned at Y = ±0.1)
    rail_length = 0.6
    rail_y_positions = [-0.1, 0.1]
    rail_height = 0.025
    
    for i, y_pos in enumerate(rail_y_positions):
        rail_shape = _build_rail()
        base.visual(
            mesh_from_cadquery(rail_shape, f"rail_{i}"),
            origin=Origin(xyz=(0.0, y_pos, 0.02 + rail_height/2)),  # Center on top of base plate
            material="steel",
            name=f"rail_{i}",
        )
        
        # End caps at both ends of each rail
        for end in [-1, 1]:
            end_cap_shape = _build_end_cap()
            base.visual(
                mesh_from_cadquery(end_cap_shape, f"end_cap_{i}_{end}"),
                origin=Origin(xyz=(end * rail_length/2, y_pos, 0.02 + rail_height/2)),
                material="dark_aluminum",
                name=f"end_cap_{i}_{end}",
            )
    
    # Bolt heads on base plate (visible on top surface)
    bolt_positions = [
        (-0.27, -0.14), (-0.27, 0.14), (0.27, -0.14), (0.27, 0.14),
    ]
    for i, (x, y) in enumerate(bolt_positions):
        bolt_shape = _build_bolt_head()
        base.visual(
            mesh_from_cadquery(bolt_shape, f"base_bolt_{i}"),
            origin=Origin(xyz=(x, y, 0.02 + 0.002)),  # Slightly proud of surface
            material="black",
            name=f"base_bolt_{i}",
        )
    
    # === CARRIAGE (sliding part) ===
    # Joint origin at Z = 0.07 (center of carriage in world frame)
    # Carriage body is at local Z=0, which is world Z=0.07
    carriage = model.part("carriage")
    
    # Main carriage body
    carriage_shape = _build_carriage()
    carriage.visual(
        mesh_from_cadquery(carriage_shape, "carriage_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # At carriage part frame center (world Z=0.07)
        material="aluminum",
        name="carriage_body",
    )
    
    # Bearing blocks (4 blocks - 2 per rail)
    # Rails are at world Z=0.02 to 0.045 (center at 0.0325)
    # Bearing blocks should interface with rails at Z≈0.0325
    # Local Z = 0.0325 - 0.07 (part frame) = -0.0375
    bearing_local_z = 0.0325 - 0.07  # -0.0375
    bearing_positions = [
        # Rail 0 (Y = -0.1)
        (-0.04, -0.1, bearing_local_z),
        (0.04, -0.1, bearing_local_z),
        # Rail 1 (Y = 0.1)
        (-0.04, 0.1, bearing_local_z),
        (0.04, 0.1, bearing_local_z),
    ]
    
    for i, (x, y, z) in enumerate(bearing_positions):
        bearing_shape = _build_bearing_block()
        carriage.visual(
            mesh_from_cadquery(bearing_shape, f"bearing_{i}"),
            origin=Origin(xyz=(x, y, z)),
            material="steel",
            name=f"bearing_{i}",
        )
    
    # Bolt heads on carriage - on top of carriage body
    # Carriage body top at world Z=0.07 + 0.05/2 = 0.095
    # Bolt heads proud of carriage body top
    carriage_bolt_z = 0.025 + 0.002  # Half carriage height + proud (local coords)
    carriage_bolt_positions = [
        (-0.03, -0.05, carriage_bolt_z), (-0.03, 0.05, carriage_bolt_z),
        (0.03, -0.05, carriage_bolt_z), (0.03, 0.05, carriage_bolt_z),
    ]
    for i, (x, y, z) in enumerate(carriage_bolt_positions):
        bolt_shape = _build_bolt_head()
        carriage.visual(
            mesh_from_cadquery(bolt_shape, f"carriage_bolt_{i}"),
            origin=Origin(xyz=(x, y, z)),
            material="black",
            name=f"carriage_bolt_{i}",
        )
    
    # === PRISMATIC JOINT (along X-axis) ===
    # Joint origin at carriage body center: world Z = 0.07
    joint_z = 0.07
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(1.0, 0.0, 0.0),  # Slide along X-axis
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.5,
            lower=-0.2,
            upper=0.2,
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    prismatic_joint = object_model.get_articulation("base_to_carriage")
    
    # Allow intentional overlap between bearing blocks and rails
    # The bearing blocks are designed to interface with the rails
    rail_bearing_pairs = [
        ("rail_0", "bearing_0"),
        ("rail_0", "bearing_1"),
        ("rail_1", "bearing_2"),
        ("rail_1", "bearing_3"),
    ]
    
    for rail_name, bearing_name in rail_bearing_pairs:
        ctx.allow_overlap(
            "base",
            "carriage",
            elem_a=rail_name,
            elem_b=bearing_name,
            reason=f"Bearing block {bearing_name} intentionally interfaces with rail {rail_name}",
        )
    
    # Test 1: Verify prismatic joint exists and has correct properties
    ctx.check(
        "prismatic_joint_exists",
        prismatic_joint is not None,
        details="Prismatic joint should exist",
    )
    
    if prismatic_joint is not None:
        ctx.check(
            "joint_type_is_prismatic",
            prismatic_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"Joint type is {prismatic_joint.articulation_type}",
        )
        
        ctx.check(
            "joint_axis_along_x",
            prismatic_joint.axis == (1.0, 0.0, 0.0),
            details=f"Joint axis is {prismatic_joint.axis}, expected (1.0, 0.0, 0.0)",
        )
        
        if prismatic_joint.motion_limits is not None:
            limits = prismatic_joint.motion_limits
            ctx.check(
                "motion_limits_set",
                limits.lower is not None and limits.upper is not None,
                details="Motion limits should have lower and upper bounds",
            )
            ctx.check(
                "travel_range_positive",
                limits.upper > limits.lower if limits.upper is not None and limits.lower is not None else False,
                details=f"Upper limit {limits.upper} should be > lower limit {limits.lower}",
            )
    
    # Test 2: Check that carriage is supported (no floating parts)
    ctx.check(
        "carriage_has_parent",
        prismatic_joint is not None and prismatic_joint.parent == "base" and prismatic_joint.child == "carriage",
        details="Carriage should be child of base via prismatic joint",
    )
    
    # Test 3: Check carriage positioning at home position (q=0)
    with ctx.pose({prismatic_joint: 0.0}):
        # Carriage should be centered and above base
        carriage_pos = ctx.part_world_position(carriage)
        base_pos = ctx.part_world_position(base)
        
        ctx.check(
            "carriage_above_base_at_home",
            carriage_pos is not None and carriage_pos[2] > base_pos[2] if base_pos else False,
            details=f"Carriage Z={carriage_pos[2] if carriage_pos else None} should be > base Z={base_pos[2] if base_pos else None}",
        )
        
        # Verify bearing blocks contact rails (proof check for overlap allowance)
        for i in range(4):
            ctx.expect_contact(
                carriage,
                base,
                elem_a=f"bearing_{i}",
                elem_b=f"rail_{i//2}",  # bearing_0,1 -> rail_0; bearing_2,3 -> rail_1
                contact_tol=0.03,  # Bearing blocks intentionally overlap with rails
                name=f"bearing_{i}_contacts_rail_{i//2}",
            )
    
    # Test 4: Check carriage movement along X-axis
    with ctx.pose({prismatic_joint: 0.15}):
        pos_extended = ctx.part_world_position(carriage)
        with ctx.pose({prismatic_joint: -0.15}):
            pos_retracted = ctx.part_world_position(carriage)
            
        if pos_extended is not None and pos_retracted is not None:
            ctx.check(
                "carriage_moves_along_x",
                pos_extended[0] > pos_retracted[0],
                details=f"Extended X={pos_extended[0]}, retracted X={pos_retracted[0]}",
            )
    
    # Test 5: Verify rails exist on base
    rail_0 = base.get_visual("rail_0")
    rail_1 = base.get_visual("rail_1")
    ctx.check(
        "rails_present",
        rail_0 is not None and rail_1 is not None,
        details="Both rails should be present on base",
    )
    
    # Test 6: Verify bearing blocks exist on carriage
    for i in range(4):
        bearing = carriage.get_visual(f"bearing_{i}")
        ctx.check(
            f"bearing_{i}_present",
            bearing is not None,
            details=f"Bearing block {i} should be present on carriage",
        )
    
    # Test 7: Check no rotational freedom (prismatic joint should only allow X translation)
    if prismatic_joint is not None:
        axis = prismatic_joint.axis
        ctx.check(
            "no_rotational_freedom",
            axis == (1.0, 0.0, 0.0),
            details=f"Prismatic joint should only allow X motion, axis={axis}",
        )
    
    # Test 8: Check that carriage stays within rail bounds at limits
    if prismatic_joint is not None and prismatic_joint.motion_limits is not None:
        limits = prismatic_joint.motion_limits
        if limits.lower is not None:
            with ctx.pose({prismatic_joint: limits.lower}):
                pos = ctx.part_world_position(carriage)
                ctx.check(
                    "carriage_within_bounds_lower",
                    pos is not None and -0.3 <= pos[0] <= 0.3,
                    details=f"Carriage at lower limit: X={pos[0] if pos else None}",
                )
        if limits.upper is not None:
            with ctx.pose({prismatic_joint: limits.upper}):
                pos = ctx.part_world_position(carriage)
                ctx.check(
                    "carriage_within_bounds_upper",
                    pos is not None and -0.3 <= pos[0] <= 0.3,
                    details=f"Carriage at upper limit: X={pos[0] if pos else None}",
                )
    
    # Test 9: Verify carriage moves in correct direction (positive q = +X)
    home_pos = None
    extended_pos = None
    with ctx.pose({prismatic_joint: 0.0}):
        home_pos = ctx.part_world_position(carriage)
    with ctx.pose({prismatic_joint: 0.1}):
        extended_pos = ctx.part_world_position(carriage)
    
    if home_pos is not None and extended_pos is not None:
        ctx.check(
            "positive_q_moves_carriage_positive_x",
            extended_pos[0] > home_pos[0],
            details=f"At q=0.1, X={extended_pos[0]} should be > X at q=0: {home_pos[0]}",
        )
    
    return ctx.report()


object_model = build_object_model()
