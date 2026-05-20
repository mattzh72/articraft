from __future__ import annotations

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
    mesh_from_geometry,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gate_strap_hinge")

    # Materials
    steel = Material(name="steel", rgba=(0.70, 0.70, 0.72, 1.0))
    dark_steel = Material(name="dark_steel", rgba=(0.45, 0.45, 0.48, 1.0))
    zinc = Material(name="zinc", rgba=(0.80, 0.82, 0.85, 1.0))

    # Wall-side plate (fixed base) - sits at z=0
    wall_plate = model.part("wall_plate")
    # Main plate: 150mm x 50mm x 6mm with chamfered edges
    wall_wp = (
        cq.Workplane("XY")
        .box(0.150, 0.050, 0.006)
        .edges("|Z")
        .chamfer(0.002)
    )
    wall_mesh = mesh_from_cadquery(wall_wp, "wall_plate_chamfered")
    wall_plate.visual(
        wall_mesh,
        origin=Origin(xyz=(0.075, 0.0, 0.003)),
        material=steel,
        name="wall_plate_shell",
    )
    # Strap gussets for structural support - with chamfered edges
    # Top gusset - welded to back of plate
    gusset_wp = (
        cq.Workplane("XY")
        .box(0.050, 0.050, 0.006)
        .edges("|Z")
        .chamfer(0.001)
    )
    gusset_mesh = mesh_from_cadquery(gusset_wp, "top_gusset_chamfered")
    wall_plate.visual(
        gusset_mesh,
        origin=Origin(xyz=(0.075, 0.028, 0.006)),
        material=dark_steel,
        name="top_gusset",
    )
    # Bottom gusset
    wall_plate.visual(
        gusset_mesh,
        origin=Origin(xyz=(0.075, -0.028, 0.006)),
        material=dark_steel,
        name="bottom_gusset",
    )

    # Vertical pin on wall plate - extends from z=0.006 to z=0.066 (60mm length)
    # Pin is fixed to wall plate, strap rotates on it
    wall_plate.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(xyz=(0.075, 0.0, 0.036)),
        material=zinc,
        name="hinge_pin",
    )

    # Bottom washer (between wall plate and strap)
    wall_plate.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.075, 0.0, 0.006)),
        material=zinc,
        name="bottom_washer",
    )

    # Top bolt head (captures strap on pin)
    wall_plate.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.075, 0.0, 0.066)),
        material=dark_steel,
        name="top_bolt_head",
    )

    # Mounting bolt heads for wall plate (4 bolts)
    bolt_positions = [
        (0.020, 0.018), (0.020, -0.018),
        (0.130, 0.018), (0.130, -0.018),
    ]
    for i, (x, y) in enumerate(bolt_positions):
        wall_plate.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, y, 0.0)),
            material=dark_steel,
            name=f"wall_bolt_{i}",
        )

    # Strap plate (moving part) - rotates on the pin
    strap_plate = model.part("strap_plate")
    # Create strap with hole using CadQuery and chamfered edges
    # Strap: 300mm x 40mm x 5mm with 12mm diameter hole at pin location
    strap_wp = (
        cq.Workplane("XY")
        .box(0.300, 0.040, 0.005)
        .faces(">Z")
        .workplane()
        .center(-0.150, 0)  # Move to where pin will be (at origin of part frame)
        .hole(0.012)
        .edges("|Z")
        .chamfer(0.0015)
    )
    strap_mesh = mesh_from_cadquery(strap_wp, "strap_plate_with_hole")
    strap_plate.visual(
        strap_mesh,
        # Part frame is at pin axis. Strap extends along +X from pin.
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=steel,
        name="strap_shell",
    )

    # Gusset on strap plate for rigidity at far end - with chamfered edges
    strap_gusset_wp = (
        cq.Workplane("XY")
        .box(0.040, 0.040, 0.005)
        .edges("|Z")
        .chamfer(0.001)
    )
    strap_gusset_mesh = mesh_from_cadquery(strap_gusset_wp, "strap_gusset_chamfered")
    strap_plate.visual(
        strap_gusset_mesh,
        origin=Origin(xyz=(0.300, 0.0, 0.0025)),
        material=dark_steel,
        name="strap_gusset",
    )

    # Washer between strap and top bolt (sits on top of strap)
    strap_plate.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=zinc,
        name="strap_washer",
    )

    # Mounting bolt heads for strap plate (4 bolts to attach to gate)
    strap_bolt_positions = [
        (0.120, 0.015), (0.120, -0.015),
        (0.210, 0.015), (0.210, -0.015),
    ]
    for i, (x, y) in enumerate(strap_bolt_positions):
        strap_plate.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, y, 0.0025)),
            material=dark_steel,
            name=f"strap_bolt_{i}",
        )

    # The articulation: vertical pin joint
    # Joint at pin axis: (0.075, 0, 0.036) in world coords
    # At q=0, strap extends along +X from pin (toward the gate)
    # Positive rotation around Z opens the gate (right-hand rule)
    model.articulation(
        "wall_to_strap",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=strap_plate,
        # Joint frame at pin axis, at mid-height of pin
        origin=Origin(xyz=(0.075, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),  # Vertical axis
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=1.57),  # 0 to ~90 degrees
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    strap_plate = object_model.get_part("strap_plate")
    hinge = object_model.get_articulation("wall_to_strap")

    # Allow the pin to overlap with strap plate (pin goes through strap hole)
    ctx.allow_overlap(
        "wall_plate", "strap_plate",
        elem_a="hinge_pin",
        elem_b="strap_shell",
        reason="Pin passes through strap plate hole; modeled as overlap for simplicity",
    )

    # Allow top bolt head to overlap with strap (bolt captures strap)
    ctx.allow_overlap(
        "wall_plate", "strap_plate",
        elem_a="top_bolt_head",
        elem_b="strap_shell",
        reason="Top bolt head sits on strap washer, overlapping in projection",
    )

    # Allow bottom washer to overlap with strap (washer sits between wall plate and strap)
    ctx.allow_overlap(
        "wall_plate", "strap_plate",
        elem_a="bottom_washer",
        elem_b="strap_shell",
        reason="Bottom washer sits between wall plate and strap plate",
    )

    # Allow strap washer to overlap with strap shell
    ctx.allow_overlap(
        "strap_plate", "strap_plate",
        elem_a="strap_washer",
        elem_b="strap_shell",
        reason="Washer sits on top of strap plate",
    )

    # Proof checks for the overlaps
    ctx.expect_contact(
        wall_plate, strap_plate,
        elem_a="hinge_pin",
        elem_b="strap_shell",
        contact_tol=0.002,
        name="pin contacts strap plate through hole",
    )

    # At rest (q=0), strap should extend along +X from pin
    with ctx.pose({hinge: 0.0}):
        # Check strap extends forward from wall plate
        wall_aabb = ctx.part_world_aabb(wall_plate)
        strap_aabb = ctx.part_world_aabb(strap_plate)
        ctx.check(
            "strap extends forward from wall plate at rest",
            wall_aabb is not None and strap_aabb is not None and strap_aabb[1][0] > wall_aabb[1][0],
            details=f"wall_max_x={wall_aabb[1][0] if wall_aabb else None}, strap_max_x={strap_aabb[1][0] if strap_aabb else None}",
        )

    # At open position (q=1.57 ~ 90 degrees), strap should swing to +Y
    with ctx.pose({hinge: 1.57}):
        # Check that the far end of the strap has moved to +Y
        strap_aabb = ctx.part_world_aabb(strap_plate)
        ctx.check(
            "strap swings to side at 90 degrees",
            strap_aabb is not None and strap_aabb[1][1] > 0.05,  # max Y should be positive
            details=f"strap_aabb={strap_aabb}",
        )

    # Check bolt heads are mounted on plates
    for i in range(4):
        ctx.expect_contact(
            wall_plate, wall_plate,
            elem_a=f"wall_bolt_{i}",
            elem_b="wall_plate_shell",
            contact_tol=0.002,
            name=f"wall bolt {i} mounted on plate",
        )

    for i in range(4):
        ctx.expect_contact(
            strap_plate, strap_plate,
            elem_a=f"strap_bolt_{i}",
            elem_b="strap_shell",
            contact_tol=0.002,
            name=f"strap bolt {i} mounted on strap",
        )

    # Verify gussets provide support
    ctx.expect_contact(
        wall_plate, wall_plate,
        elem_a="top_gusset",
        elem_b="wall_plate_shell",
        contact_tol=0.002,
        name="top gusset supports wall plate",
    )
    ctx.expect_contact(
        strap_plate, strap_plate,
        elem_a="strap_gusset",
        elem_b="strap_shell",
        contact_tol=0.002,
        name="strap gusset supports strap plate",
    )

    return ctx.report()


object_model = build_object_model()
