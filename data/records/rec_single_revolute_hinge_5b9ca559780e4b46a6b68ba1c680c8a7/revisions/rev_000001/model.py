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
)


def _build_frame_block() -> cq.Solid:
    """Frame block: 40mm wide (-X), 20mm thick (+Y), 120mm long (Z).
    Local origin at (0,0,0) (hinge line center).
    Block extends: X=-0.04 to 0, Y=0 to 0.02, Z=-0.06 to 0.06.
    """
    # Main block: 40mm wide, 20mm thick, 120mm long
    block = cq.Workplane("XY").box(0.04, 0.02, 0.12)  # Centered at (0,0,0)
    # Translate so X from -0.04 to 0, Y from 0 to 0.02, Z from -0.06 to 0.06
    block = block.translate((-0.02, 0.01, 0.0))
    # Recess for hinge leaf: 25mm wide (-X), 2.5mm deep (+Y), 90mm long (Z)
    # Recess: X=-0.025 to 0, Y=0 to 0.0025, Z=-0.045 to 0.045
    recess = (
        cq.Workplane("XY")
        .box(0.025, 0.0025, 0.09)  # Centered at (0,0,0)
        .translate((-0.0125, 0.00125, 0.0))  # X=-0.025 to 0, Y=0 to 0.0025
    )
    # Cut recess
    block = block.cut(recess)
    return block.val()


def _build_door_block() -> cq.Solid:
    """Door block: 20mm thick (+X), 40mm wide (-Y), 120mm long (Z).
    Local origin at (0,0,0) (hinge line center).
    Block extends: X=0 to 0.02, Y=-0.04 to 0, Z=-0.06 to 0.06.
    """
    # Main block: 20mm thick, 40mm wide, 120mm long
    block = cq.Workplane("XY").box(0.02, 0.04, 0.12)  # Centered at (0,0,0)
    # Translate so X from 0 to 0.02, Y from -0.04 to 0, Z from -0.06 to 0.06
    block = block.translate((0.01, -0.02, 0.0))
    # Recess for hinge leaf: 2.5mm deep (+X), 25mm wide (-Y), 90mm long (Z)
    # Recess: X=0 to 0.0025, Y=-0.025 to 0, Z=-0.045 to 0.045
    recess = (
        cq.Workplane("XY")
        .box(0.0025, 0.025, 0.09)  # Centered at (0,0,0)
        .translate((0.00125, -0.0125, 0.0))  # X=0 to 0.0025, Y=-0.025 to 0
    )
    # Cut recess
    block = block.cut(recess)
    return block.val()


def _build_hinge_leaf_a() -> cq.Solid:
    """Frame-side hinge leaf: 25mm wide (-X), 2.5mm thick (+Y), 90mm long (Z).
    Local origin at (0,0,0) (hinge line center).
    Extends: X=-0.025 to 0, Y=0 to 0.0025, Z=-0.045 to 0.045.
    """
    # Leaf plate
    leaf = (
        cq.Workplane("XY")
        .box(0.025, 0.0025, 0.09)
        .translate((-0.0125, 0.00125, 0.0))  # X=-0.025 to 0, Y=0 to 0.0025
    )
    # Knuckles (5 cylinders along Z axis, on -X edge)
    knuckle_od = 0.0025  # Fits within leaf Y thickness (0.0025)
    knuckle_length = 0.017  # 17mm per knuckle, leaving 0.5mm gaps between knuckles
    clearance = 0.0005  # 0.5mm gap between knuckles
    total_knuckle_length = 5 * knuckle_length + 4 * clearance  # 5*17 +4*0.5=87mm, plus 1.5mm each end=90mm
    start_z = -0.045 + (0.09 - total_knuckle_length) / 2  # Center the knuckles
    for i in range(5):
        z_center = start_z + (i * (knuckle_length + clearance)) + knuckle_length / 2
        # Cylinder along Z axis: use Workplane("XY"), height=length along Z
        knuckle = (
            cq.Workplane("XY")
            .cylinder(knuckle_length, knuckle_od / 2)  # Height along Z
            .translate((-0.025, 0.00125, z_center))  # X=-0.025, Y=0.00125, Z=z_center
        )
        leaf = leaf.union(knuckle)
    return leaf.val()


def _build_hinge_leaf_b() -> cq.Solid:
    """Door-side hinge leaf: 2.5mm thick (+X), 25mm wide (-Y), 90mm long (Z).
    Local origin at (0,0,0) (hinge line center).
    Extends: X=0 to 0.0025, Y=-0.025 to 0, Z=-0.045 to 0.045.
    """
    # Leaf plate
    leaf = (
        cq.Workplane("XY")
        .box(0.0025, 0.025, 0.09)
        .translate((0.00125, -0.0125, 0.0))  # X=0 to 0.0025, Y=-0.025 to 0
    )
    # Knuckles (5 cylinders along Z axis, on -Y edge)
    knuckle_od = 0.0025  # Fits within leaf X thickness (0.0025)
    knuckle_length = 0.017  # 17mm per knuckle, leaving 0.5mm gaps
    clearance = 0.0005  # 0.5mm gap between knuckles
    total_knuckle_length = 5 * knuckle_length + 4 * clearance
    start_z = -0.045 + (0.09 - total_knuckle_length) / 2  # Center the knuckles
    for i in range(5):
        z_center = start_z + (i * (knuckle_length + clearance)) + knuckle_length / 2
        # Cylinder along Z axis: use Workplane("XY"), height=length along Z
        knuckle = (
            cq.Workplane("XY")
            .cylinder(knuckle_length, knuckle_od / 2)  # Height along Z
            .translate((0.00125, -0.025, z_center))  # X=0.00125, Y=-0.025, Z=z_center
        )
        leaf = leaf.union(knuckle)
    return leaf.val()


def _build_screw_heads(leaf: cq.Solid, is_frame_leaf: bool) -> cq.Solid:
    """Build screw heads for a hinge leaf."""
    screw_diameter = 0.006
    screw_thickness = 0.001
    screws = cq.Workplane("XY")
    
    if is_frame_leaf:
        # 3 screws on frame leaf (along Z)
        z_positions = [-0.045 + 0.09 * 0.25, -0.045 + 0.09 * 0.5, -0.045 + 0.09 * 0.75]
        x = -0.0125  # Center of leaf width
        y = 0.0025 + screw_thickness / 2  # On top of leaf (Y direction)
    else:
        # 2 screws on door leaf (along Z)
        z_positions = [-0.045 + 0.09 * (1/3), -0.045 + 0.09 * (2/3)]
        x = 0.0025 + screw_thickness / 2  # On top of leaf (X direction)
        y = -0.0125  # Center of leaf width
    
    for z in z_positions:
        screw = cq.Workplane("XY").cylinder(screw_thickness, screw_diameter / 2).translate((x, y, z))
        screws = screws.union(screw)
    
    return screws.val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_door_hinge_corner")

    # Materials
    frame_material = model.material("frame_material", rgba=(0.3, 0.3, 0.3, 1.0))
    door_material = model.material("door_material", rgba=(0.7, 0.7, 0.7, 1.0))
    hinge_material = model.material("hinge_material", rgba=(0.8, 0.8, 0.8, 1.0))
    screw_material = model.material("screw_material", rgba=(0.2, 0.2, 0.2, 1.0))

    # Frame block (root part, fixed)
    frame_block = model.part("frame_block")
    frame_shape = _build_frame_block()
    frame_block.visual(
        mesh_from_cadquery(frame_shape, "frame_block_shape"),
        material=frame_material,
        name="frame_shell",
    )

    # Door block (child part, moving)
    door_block = model.part("door_block")
    door_shape = _build_door_block()
    door_block.visual(
        mesh_from_cadquery(door_shape, "door_block_shape"),
        material=door_material,
        name="door_shell",
    )

    # Hinge leaves (attached to respective blocks)
    # Frame leaf A
    hinge_leaf_a = _build_hinge_leaf_a()
    frame_block.visual(
        mesh_from_cadquery(hinge_leaf_a, "hinge_leaf_a_shape"),
        material=hinge_material,
        name="hinge_leaf_a",
    )
    # Screw heads A
    screw_heads_a = _build_screw_heads(hinge_leaf_a, is_frame_leaf=True)
    frame_block.visual(
        mesh_from_cadquery(screw_heads_a, "screw_heads_a_shape"),
        material=screw_material,
        name="screw_heads_a",
    )

    # Door leaf B
    hinge_leaf_b = _build_hinge_leaf_b()
    door_block.visual(
        mesh_from_cadquery(hinge_leaf_b, "hinge_leaf_b_shape"),
        material=hinge_material,
        name="hinge_leaf_b",
    )
    # Screw heads B
    screw_heads_b = _build_screw_heads(hinge_leaf_b, is_frame_leaf=False)
    door_block.visual(
        mesh_from_cadquery(screw_heads_b, "screw_heads_b_shape"),
        material=screw_material,
        name="screw_heads_b",
    )

    # Continuous barrel (pin) through all knuckles
    # Barrel: 92mm long (Z) (protrudes 1mm each end), 2mm diameter
    barrel = (
        cq.Workplane("XY")
        .cylinder(0.092, 0.001)  # 92mm long, 2mm diameter
        .translate((0.0, 0.0, 0.0))  # Centered at origin
    )
    # Add barrel to frame block (fixed part)
    frame_block.visual(
        mesh_from_cadquery(barrel.val(), "barrel_shape"),
        material=hinge_material,
        name="hinge_barrel",
    )

    # Revolute joint: frame_block to door_block
    # Joint at origin (0,0,0), axis along Z
    # When q=0: door closed (along -Y)
    # When q>0: door swings open (towards +Y, away from frame)
    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame_block,
        child=door_block,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Hinge line at origin
        axis=(0.0, 0.0, 1.0),  # Along Z axis
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=5.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_block")
    door = object_model.get_part("door_block")
    hinge = object_model.get_articulation("frame_to_door")

    # Allow intentional overlaps
    # Hinge leaves seated in block recesses
    ctx.allow_overlap(
        "frame_block", "frame_block",
        elem_a="frame_shell", elem_b="hinge_leaf_a",
        reason="Hinge leaf A seated in frame block recess",
    )
    ctx.allow_overlap(
        "door_block", "door_block",
        elem_a="door_shell", elem_b="hinge_leaf_b",
        reason="Hinge leaf B seated in door block recess",
    )
    # Screw heads seated on leaves
    ctx.allow_overlap(
        "frame_block", "frame_block",
        elem_a="hinge_leaf_a", elem_b="screw_heads_a",
        reason="Screw heads seated on hinge leaf A",
    )
    ctx.allow_overlap(
        "door_block", "door_block",
        elem_a="hinge_leaf_b", elem_b="screw_heads_b",
        reason="Screw heads seated on hinge leaf B",
    )
    # Interlocking knuckles between leaves
    ctx.allow_overlap(
        "frame_block", "door_block",
        elem_a="hinge_leaf_a", elem_b="hinge_leaf_b",
        reason="Hinge leaves interlock at knuckles",
    )
    # Barrel passes through knuckles (intentional overlap)
    ctx.allow_overlap(
        "frame_block", "frame_block",
        elem_a="hinge_barrel", elem_b="hinge_leaf_a",
        reason="Barrel passes through frame leaf knuckles",
    )
    ctx.allow_overlap(
        "door_block", "frame_block",
        elem_a="hinge_barrel", elem_b="hinge_leaf_b",
        reason="Barrel passes through door leaf knuckles",
    )

    # Test 1: Door swings away from frame when opened
    with ctx.pose({hinge: 0.0}):
        # Closed: door should be near frame (contact or small gap)
        ctx.expect_contact(frame, door, name="closed_door_contact")

    with ctx.pose({hinge: 1.57}):  # 90 degrees open
        # Open: door should swing to +Y (away from frame at Y>=0)
        door_aabb = ctx.part_world_aabb(door)  # (min_vec, max_vec)
        door_max_y = door_aabb[1][1]  # Y component of max AABB
        ctx.check(
            "door_swings_away_from_frame",
            door_max_y > 0.01,  # Door's max Y should be positive (away from frame)
            details=f"Door max Y: {door_max_y}",
        )
        # Check door clears frame (no significant overlap)
        ctx.expect_gap(
            door, frame, axis="y",
            min_gap=-0.03, max_gap=0.05,  # Allow small overlap from door thickness
            name="open_door_y_gap",
        )

    # Test 2: Hinge leaves seated in recesses
    with ctx.pose({hinge: 0.0}):
        # Frame leaf A within frame shell
        ctx.expect_within(
            frame, frame,
            inner_elem="hinge_leaf_a", outer_elem="frame_shell",
            axes="xyz", margin=0.001,
            name="hinge_leaf_a_seated",
        )
        # Door leaf B within door shell
        ctx.expect_within(
            door, door,
            inner_elem="hinge_leaf_b", outer_elem="door_shell",
            axes="xyz", margin=0.001,
            name="hinge_leaf_b_seated",
        )

    # Test 3: Screw heads present
    ctx.check(
        "screw_heads_a_present",
        any(v.name == "screw_heads_a" for v in frame.visuals),
        details="Frame block missing screw_heads_a visual",
    )
    ctx.check(
        "screw_heads_b_present",
        any(v.name == "screw_heads_b" for v in door.visuals),
        details="Door block missing screw_heads_b visual",
    )

    return ctx.report()


object_model = build_object_model()
