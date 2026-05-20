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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cabinet_hinge")

    # Fixed cabinet-side plate (root part)
    cabinet_plate = model.part("cabinet_plate")

    # --- Cabinet plate main body (60x40x2mm) with rounded corners ---
    plate_body = (
        cq.Workplane("XZ")
        .box(0.06, 0.04, 0.002)
        .edges("|Y")
        .fillet(0.002)  # Round edges parallel to Y axis (thickness)
    )
    # Cut 3mm diameter screw holes
    screw_positions = [(-0.02, -0.01), (0.02, 0.01)]
    for x, z in screw_positions:
        hole = cq.Workplane("XZ").workplane(offset=0).center(x, z).circle(0.0015)
        plate_body = plate_body.cut(hole.extrude(0.002, both=True))
    cabinet_plate.visual(
        mesh_from_cadquery(plate_body, "cabinet_plate_body"),
        origin=Origin(xyz=(0, 0, 0)),
        name="cabinet_plate_body",
        color=(0.75, 0.75, 0.75),  # Brushed nickel
    )

    # --- Cabinet barrel loops (knuckles 0 and 2) ---
    # All knuckles share same center at hinge point for interpenetration
    def make_knuckle():
        return (
            cq.Workplane("XY")
            .workplane()
            .circle(0.004)
            .extrude(0.003, both=True)
            .circle(0.002)
            .cut(cq.Workplane("XY").workplane().circle(0.002).extrude(0.004, both=True))
        )
    
    k0 = make_knuckle()
    cabinet_plate.visual(
        mesh_from_cadquery(k0, "knuckle_0"),
        origin=Origin(xyz=(0.03, 0, 0)),
        name="knuckle_0",
        color=(0.72, 0.53, 0.04),  # Brass
    )

    k2 = make_knuckle()
    cabinet_plate.visual(
        mesh_from_cadquery(k2, "knuckle_2"),
        origin=Origin(xyz=(0.03, 0, 0)),
        name="knuckle_2",
        color=(0.72, 0.53, 0.04),  # Brass,
    )

    # --- Spacer washer (thin, between knuckles) ---
    # Hollow cylinder: 4mm outer, 2.5mm inner, 1mm thick, at hinge point z=0
    washer = (
        cq.Workplane("XY")
        .workplane()
        .circle(0.004)
        .extrude(0.0005, both=True)
        .circle(0.0025)
        .cut(cq.Workplane("XY").workplane().circle(0.0025).extrude(0.001, both=True))
    )
    cabinet_plate.visual(
        mesh_from_cadquery(washer, "spacer_washer"),
        origin=Origin(xyz=(0.03, 0, 0)),
        name="spacer_washer",
        color=(0, 0, 0),  # Black plastic
    )

    # --- Hinge pin (goes through all knuckles) ---
    # Solid cylinder: 2mm radius, 16mm long (extends beyond knuckles)
    pin = cq.Workplane("XY").center(0.03, 0).circle(0.002).extrude(0.008, both=True)
    cabinet_plate.visual(
        mesh_from_cadquery(pin, "hinge_pin"),
        origin=Origin(xyz=(0, 0, 0)),  # Pin centered at origin relative to cabinet_plate
        name="hinge_pin",
        color=(0.9, 0.9, 0.9),  # Silver
    )

    # Moving door-side plate
    door_plate = model.part("door_plate")

    # --- Door plate main body (same dimensions as cabinet plate) ---
    door_body = (
        cq.Workplane("XZ")
        .box(0.06, 0.04, 0.002)
        .edges("#Y")
        .fillet(0.0005)  # Round corners (0.5mm radius)
    )
    for x, z in screw_positions:
        hole = cq.Workplane("XZ").workplane(offset=0).center(x, z).circle(0.0015)
        door_body = door_body.cut(hole.extrude(0.002, both=True))
    door_plate.visual(
        mesh_from_cadquery(door_body, "door_plate_body"),
        origin=Origin(xyz=(0, 0, 0)),
        name="door_plate_body",
        color=(0.2, 0.2, 0.2),  # Dark gray
    )

    # --- Door barrel loop (knuckle 1, middle) ---
    # At door plate origin (0,0,0) so it aligns with hinge point at (0.03, 0, 0) world
    k1 = (
        cq.Workplane("XY")
        .workplane()
        .circle(0.004)
        .extrude(0.003, both=True)
        .circle(0.002)
        .cut(cq.Workplane("XY").workplane().circle(0.002).extrude(0.004, both=True))
    )
    door_plate.visual(
        mesh_from_cadquery(k1, "knuckle_1"),
        origin=Origin(xyz=(0, 0, 0)),
        name="knuckle_1",
        color=(0.72, 0.53, 0.04),  # Brass
    )

    # --- Revolute hinge articulation ---
    # Hinge at (0.03, 0, 0) - center of knuckle assembly
    # Door plate origin is at hinge point
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_plate,
        child=door_plate,
        origin=Origin(xyz=(0.03, 0, 0)),
        axis=(0, 0, 1),  # Hinge pin along Z-axis
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,  # Fully closed
            upper=1.5708,  # ~90° open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_plate = object_model.get_part("cabinet_plate")
    door_plate = object_model.get_part("door_plate")
    hinge = object_model.get_articulation("cabinet_to_door")

    # Allow intentional overlap: knuckles interpenetrate at hinge pin
    ctx.allow_overlap(
        "cabinet_plate",
        "door_plate",
        elem_a="knuckle_0",
        elem_b="knuckle_1",
        reason="Knuckles interpenetrate at the hinge pin (captured barrel)",
    )
    ctx.allow_overlap(
        "cabinet_plate",
        "door_plate",
        elem_a="knuckle_2",
        elem_b="knuckle_1",
        reason="Knuckles interpenetrate at the hinge pin (captured barrel)",
    )

    # Rest pose (q=0) checks
    with ctx.pose({hinge: 0.0}):
        # Knuckles interpenetrate at hinge pin (correct behavior for barrel hinge)
        ctx.expect_overlap(
            cabinet_plate,
            door_plate,
            axes="z",
            elem_a="knuckle_0",
            elem_b="knuckle_1",
            min_overlap=0.005,
            name="knuckle_0_knuckle_1_overlap_rest",
        )
        ctx.expect_overlap(
            door_plate,
            cabinet_plate,
            axes="z",
            elem_a="knuckle_1",
            elem_b="knuckle_2",
            min_overlap=0.005,
            name="knuckle_1_knuckle_2_overlap_rest",
        )
        # Door is closed (aligned with cabinet plate in Y direction)
        door_aabb = ctx.part_world_aabb(door_plate)
        ctx.check(
            "door_closed_at_rest",
            door_aabb is not None and abs(door_aabb[0][1]) < 0.05,  # Check Y coordinate of AABB min
            details=f"Door AABB at rest: {door_aabb}",
        )

    # Open pose (q=90°) check - verify door swings open
    with ctx.pose({hinge: 1.5708}):
        door_aabb_open = ctx.part_world_aabb(door_plate)
        # At 90° open, door should extend in +Y direction
        ctx.check(
            "door_opens_90_degrees",
            door_aabb_open is not None and door_aabb_open[1][1] > 0.01,
            details=f"Door AABB open: {door_aabb_open}",
        )

    # Validate motion limit
    ctx.check(
        "hinge_upper_limit_correct",
        hinge.motion_limits.upper == 1.5708,
        details=f"Hinge upper limit: {hinge.motion_limits.upper}",
    )

    # Visible detail checks
    ctx.check(
        "correct_barrel_loop_count",
        len([v for v in cabinet_plate.visuals if "knuckle" in v.name]) == 2
        and len([v for v in door_plate.visuals if "knuckle" in v.name]) == 1,
        details="Incorrect number of barrel loops per plate",
    )
    ctx.check(
        "cabinet_plate_has_valid_visuals",
        len(cabinet_plate.visuals) >= 1,
        details="Cabinet plate missing visuals",
    )

    return ctx.report()


object_model = build_object_model()
