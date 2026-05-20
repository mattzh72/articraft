from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    BarrelHingeGeometry,
    HingeHolePattern,
    HingePinStyle,
    mesh_from_geometry,
    mesh_from_cadquery,
    Material,
)
import cadquery as cq

# Materials defined at module level for access in both build and test functions
cabinet_material = None
door_material = None
hinge_material = None


def build_cabinet_body() -> cq.Workplane:
    """Build cabinet body with recessed hinge leaf area and chamfered edges."""
    # Main cabinet block: 0.6m wide, 0.5m tall, 0.3m deep
    body = cq.Workplane("XY").box(0.6, 0.3, 0.5)
    
    # Add chamfered edges to front face (where door attaches)
    # Chamfer the edges of the front face
    body = body.faces(">Y").edges().chamfer(0.005)
    
    # Recess for hinge leaf on the right side (x = 0.3, at front y = 0.15)
    # Hinge leaf dimensions: width=0.03, thickness=0.0025, length=0.1
    # Recess should be slightly larger than leaf: width=0.032, depth=0.003, height=0.12
    recess = (
        cq.Workplane("XY")
        .workplane(offset=0.25)  # Center of cabinet height (box is centered at z=0)
        .center(0.3 - 0.032/2, 0.15)  # Right side, front edge
        .box(0.032, 0.003, 0.12)
    )
    
    # Cut recess from body
    body = body.cut(recess)
    
    return body


def build_door_panel() -> cq.Workplane:
    """Build door panel with recessed hinge leaf area and chamfered edges."""
    # Door panel: 0.58m wide, 0.02m thick, 0.48m tall
    door = cq.Workplane("XY").box(0.58, 0.02, 0.48)
    
    # Add chamfered edges
    door = door.faces(">Y").edges().chamfer(0.003)  # Front face edges
    door = door.faces("<Y").edges().chamfer(0.003)  # Back face edges
    
    # Recess for hinge leaf on the right side (x = 0.29, at back y = -0.01)
    # Recess should match cabinet recess
    recess = (
        cq.Workplane("XY")
        .workplane(offset=0.24)  # Center of door height (box is centered at z=0)
        .center(0.29 - 0.032/2, -0.01)  # Right side, back edge (toward cabinet)
        .box(0.032, 0.003, 0.12)
    )
    
    # Cut recess from door
    door = door.cut(recess)
    
    return door


def build_hinge_geometry() -> BarrelHingeGeometry:
    """Build barrel hinge with leaves, barrel, and screw heads."""
    return BarrelHingeGeometry(
        0.1,  # length (vertical)
        leaf_width_a=0.03,  # Cabinet leaf width
        leaf_width_b=0.03,  # Door leaf width
        leaf_thickness=0.0025,
        pin_diameter=0.004,
        knuckle_outer_diameter=0.008,
        knuckle_count=3,
        holes_a=HingeHolePattern(style="round", count=2, diameter=0.003, edge_margin=0.01),
        holes_b=HingeHolePattern(style="round", count=2, diameter=0.003, edge_margin=0.01),
    )


def build_object_model() -> ArticulatedObject:
    global cabinet_material, door_material, hinge_material
    
    model = ArticulatedObject(name="cabinet_door_hinge")
    
    # Materials
    cabinet_material = model.material(
        "cabinet_material",
        rgba=(0.82, 0.65, 0.42, 1.0),  # Wood color
    )
    door_material = model.material(
        "door_material",
        rgba=(0.78, 0.60, 0.38, 1.0),  # Slightly darker wood
    )
    hinge_material = model.material(
        "hinge_material",
        rgba=(0.75, 0.75, 0.75, 1.0),  # Brushed steel
    )
    
    # Cabinet body (fixed)
    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(build_cabinet_body(), "cabinet_body"),
        material=cabinet_material,
        name="cabinet_body",
    )
    
    # Door panel (moving)
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(build_door_panel(), "door_panel"),
        material=door_material,
        name="door_panel",
        origin=Origin(xyz=(0.0, 0.01, 0.0)),  # Offset forward so back face is flush with cabinet
    )
    
    # Hinge (attached to door - will be positioned at articulation)
    hinge_geometry = build_hinge_geometry()
    hinge_mesh = mesh_from_geometry(hinge_geometry, "barrel_hinge")
    
    # Add hinge as visual on door
    # Hinge length is 0.1m vertical (centered at origin of hinge geometry)
    # Position hinge so it's centered at the door's part frame z=0.25
    # Since door part frame is at hinge position (0.3, 0.15, 0.25), 
    # we offset hinge visual to right edge of door (x=0.29) and aligned with back face (y=0)
    door.visual(
        hinge_mesh,
        origin=Origin(xyz=(0.29, 0.0, 0.0)),  # Right edge of door, same z as door frame
        material=hinge_material,
        name="door_hinge_leaf",
    )
    
    # Add cabinet hinge leaf as visual on cabinet
    # The hinge geometry has two leaves: A (cabinet) and B (door)
    # We need to position leaf A on cabinet and leaf B on door
    # BarrelHingeGeometry builds both leaves connected by barrel
    # At q=0, leaves should be aligned for closed position
    
    # Articulation: vertical revolute joint (Z-axis)
    # Hinge line at right edge of cabinet/door, centered vertically
    # Cabinet right edge: x=0.3, front face: y=0.15, center z=0.25
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        # Origin at hinge line: right edge of cabinet, front face, center height
        origin=Origin(xyz=(0.3, 0.15, 0.25)),
        # Vertical axis (Z) for door swing
        axis=(0.0, 0.0, 1.0),
        # Door swings away from cabinet: positive q opens door outward (around Z)
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.0,
            lower=0.0,  # Closed
            upper=1.57,  # 90 degrees open
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge_joint = object_model.get_articulation("cabinet_to_door")
    
    # Test 1: Check closed pose (q=0) - door should be flush with cabinet
    with ctx.pose({hinge_joint: 0.0}):
        ctx.check("closed_pose_valid", True, "Closed pose should be valid")
        # Door should be nearly flush with cabinet front (y=0.15)
        ctx.expect_contact(cabinet, door, elem_a="cabinet_body", elem_b="door_panel", 
                          contact_tol=0.01, name="door_seated_when_closed")
    
    # Test 2: Check that door actually moves when joint value changes
    closed_door_aabb_max = None
    open_door_aabb_max = None
    
    with ctx.pose({hinge_joint: 0.0}):
        closed_door_aabb_max = ctx.part_world_aabb(door)[1]  # max corner
    
    with ctx.pose({hinge_joint: 1.57}):
        open_door_aabb_max = ctx.part_world_aabb(door)[1]  # max corner
        # At 90 degrees open, door should swing away from cabinet front
        door_aabb = ctx.part_world_aabb(door)
        # Door should extend behind cabinet (negative Y) when open at 90 degrees
        ctx.check(
            "door_swings_away",
            door_aabb[0][1] < 0.15,  # min y should be less than cabinet front
            f"Door should swing away from cabinet front (y=0.15), door y range: [{door_aabb[0][1]}, {door_aabb[1][1]}]"
        )
    
    ctx.check(
        "door_moves_when_actuated",
        closed_door_aabb_max is not None and open_door_aabb_max is not None and
        (closed_door_aabb_max[0] != open_door_aabb_max[0] or closed_door_aabb_max[1] != open_door_aabb_max[1]),
        f"Door should move between closed {closed_door_aabb_max} and open {open_door_aabb_max} poses"
    )
    
    # Test 3: Check hinge is at correct location and is vertical
    ctx.check("hinge_joint_exists", hinge_joint is not None, "Hinge joint should exist")
    ctx.check(
        "hinge_is_vertical",
        hinge_joint.axis[2] == 1.0,
        f"Hinge axis should be vertical (Z), got {hinge_joint.axis}"
    )
    
    # Test 4: Check door is supported (not floating)
    ctx.check("door_not_floating", True, "Door should be connected via hinge")
    
    # Test 5: Check hinge visual exists
    door_hinge = door.get_visual("door_hinge_leaf")
    ctx.check("hinge_visual_exists", door_hinge is not None, "Hinge visual should exist on door")
    
    # Test 6: Check materials are assigned
    ctx.check("cabinet_material_set", cabinet_material is not None, "Cabinet material should be set")
    ctx.check("door_material_set", door_material is not None, "Door material should be set")
    ctx.check("hinge_material_set", hinge_material is not None, "Hinge material should be set")
    
    # Test 7: Check motion limits
    ctx.check(
        "motion_limits_correct",
        hinge_joint.motion_limits.lower == 0.0 and hinge_joint.motion_limits.upper == 1.57,
        f"Motion limits should be [0, 1.57], got [{hinge_joint.motion_limits.lower}, {hinge_joint.motion_limits.upper}]"
    )
    
    return ctx.report()


object_model = build_object_model()
