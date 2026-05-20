from __future__ import annotations

import cadquery as cq
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_mirror_bracket")

    # --- Root Part: Wall Base ---
    wall_base = model.part("wall_base")

    # Wall base geometry: main plate + side ears + pivot rod + bumper
    # Main wall plate: 0.3m W (X) x 0.1m H (Y) x 0.02m D (Z), mounted to wall at Z=0
    main_plate = (
        cq.Workplane("XY")
        .rect(0.3, 0.1)  # X: -0.15..0.15, Y: -0.05..0.05
        .extrude(0.02)  # Z: 0..0.02
    )

    # Left side ear: 0.02m W (X) x 0.1m H (Y) x 0.06m D (Z), attached to left edge of main plate
    left_ear = (
        cq.Workplane("XY")
        .workplane(offset=0.02)  # Start at front face of main plate (Z=0.02)
        .rect(0.02, 0.1)  # X: -0.01..0.01, Y: -0.05..0.05
        .extrude(0.06)  # Z: 0.02..0.08
        .translate((-0.15, 0, 0))  # Move to left edge (X: -0.16..-0.14)
    )

    # Right side ear (explicit copy of left, translated to right edge)
    right_ear = (
        cq.Workplane("XY")
        .workplane(offset=0.02)  # Start at front face of main plate (Z=0.02)
        .rect(0.02, 0.1)  # X: -0.01..0.01, Y: -0.05..0.05
        .extrude(0.06)  # Z: 0.02..0.08
        .translate((0.15, 0, 0))  # Move to right edge (X: 0.14..0.16)
    )

    # Horizontal pivot rod: 0.005m radius, 0.34m long, along X-axis at Y=0, Z=0.05
    pivot_rod = (
        cq.Workplane("YZ")
        .center(0, 0.05)  # Y=0, Z=0.05 (midpoint of side ears)
        .circle(0.005)
        .extrude(0.34, both=True)  # X: -0.17..0.17
    )

    # Lower stop bumper: small rubber bumper below mirror position
    bumper = (
        cq.Workplane("XY")
        .workplane(offset=0.06)  # Z=0.06 (on front face of wall base)
        .center(0, -0.08)  # X=0, Y=-0.08 (below mirror center)
        .circle(0.01)  # 1cm radius
        .extrude(0.01)  # Z: 0.06..0.07
    )

    # Combine all wall base geometry with chamfered edges
    wall_geom = (
        main_plate
        .union(left_ear)
        .union(right_ear)
        .union(pivot_rod)
        .union(bumper)
        .chamfer(0.002)  # 2mm chamfer on all edges
    )

    # Add wall base visual (white plastic)
    wall_base.visual(
        mesh_from_cadquery(wall_geom, "wall_base_shell"),
        origin=Origin(xyz=(0, 0, 0)),
        name="wall_base_shell",
        material=Material(
            name="white_plastic",
            color=(0.95, 0.95, 0.95)
        )
    )

    # --- Moving Part: Framed Mirror Plate ---
    mirror_plate = model.part("mirror_plate")

    # Mirror frame: 0.27m W x 0.22m H x 0.015m D, with pivot rod holes and glass cutout
    frame = (
        cq.Workplane("XY")  # Part-local X-Y plane, Z is depth
        .rect(0.27, 0.22)  # Outer dimensions
        .extrude(0.015)  # Total thickness
        .translate((0, 0, -0.0075))  # Center Z at part origin (pivot point)
        # Cut pivot rod holes at left/right edges (X=±0.135, Y=0, Z=0 part-local)
        .faces(">Z")  # Front face
        .workplane()
        .center(-0.135, 0)  # Left edge hole
        .circle(0.005)  # Match pivot rod radius
        .cutThruAll()
        .faces(">Z")
        .workplane()
        .center(0.135, 0)  # Right edge hole
        .circle(0.005)
        .cutThruAll()
        # Cut inner rectangle for glass
        .faces(">Z")
        .workplane()
        .rect(0.25, 0.20)  # Inner cutout for glass
        .cutThruAll()
        .chamfer(0.001)  # 1mm chamfer on frame edges
    )

    # Mirror glass: 0.25m W x 0.20m H x 0.005m D, flush with front of frame
    glass = (
        cq.Workplane("XY")
        .rect(0.25, 0.20)
        .extrude(0.005)
        .translate((0, 0, 0.0075 - 0.005))  # Front face aligned with frame
    )

    # Add mirror frame visual (black plastic)
    mirror_plate.visual(
        mesh_from_cadquery(frame, "mirror_frame"),
        origin=Origin(xyz=(0, 0, 0)),
        name="mirror_frame",
        material=Material(
            name="black_plastic",
            color=(0.1, 0.1, 0.1)
        )
    )

    # Add mirror glass visual (reflective)
    mirror_plate.visual(
        mesh_from_cadquery(glass, "mirror_glass"),
        origin=Origin(xyz=(0, 0, 0)),
        name="mirror_glass",
        material=Material(
            name="mirror_glass_mat",
            color=(0.8, 0.8, 0.8)
        )
    )

    # --- Main Articulation: Pitch Joint (Tilt) ---
    model.articulation(
        "wall_to_mirror",
        ArticulationType.REVOLUTE,
        parent=wall_base,
        child=mirror_plate,
        origin=Origin(xyz=(0, 0, 0.05)),  # Pivot point at center of rod
        axis=(1, 0, 0),  # Horizontal left-right axis (pitch)
        motion_limits=MotionLimits(
            lower=-0.3,  # ~-17° tilt down (toward bumper)
            upper=0.5,   # ~28° tilt up
            effort=1.0,
            velocity=1.0
        )
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_base = object_model.get_part("wall_base")
    mirror_plate = object_model.get_part("mirror_plate")
    joint = object_model.get_articulation("wall_to_mirror")

    # --- Articulation Checks ---
    ctx.check(
        "main joint is revolute pitch (X-axis)",
        joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (1, 0, 0),
        details=f"Joint type: {joint.articulation_type}, axis: {joint.axis}"
    )
    ctx.check(
        "tilt range is -0.3 to 0.5 rad (~-17° to +28°)",
        joint.motion_limits.lower == -0.3 and joint.motion_limits.upper == 0.5,
        details=f"Limits: {joint.motion_limits.lower} to {joint.motion_limits.upper}"
    )

    # --- Support/Contact Checks ---
    # Mirror plate is connected via joint (no floating parts)
    ctx.check(
        "mirror plate is supported by joint",
        joint.parent == "wall_base" and joint.child == "mirror_plate",
        details=f"Joint parent: {joint.parent}, child: {joint.child}"
    )

    # Pivot rod passes through mirror frame holes (intentional captured shaft fit)
    ctx.allow_overlap(
        "wall_base", "mirror_plate",
        reason="Pivot rod passes through mirror frame holes (captured shaft fit)",
        elem_a="wall_base_shell",
        elem_b="mirror_frame"
    )
    ctx.expect_contact(
        wall_base, mirror_plate,
        elem_a="wall_base_shell",
        elem_b="mirror_frame",
        name="pivot rod contacts mirror frame"
    )

    # Lower stop bumper contact at full down tilt
    with ctx.pose({joint: -0.3}):
        ctx.expect_contact(
            mirror_plate, wall_base,
            elem_b="wall_base_shell",
            name="mirror contacts lower bumper at full down tilt"
        )

    # --- Visible Details Checks ---
    # Mirror frame and glass overlap (glass seated in frame)
    ctx.expect_overlap(
        mirror_plate, mirror_plate,
        axes="xy",
        elem_a="mirror_frame",
        elem_b="mirror_glass",
        min_overlap=0.20,  # Matches glass Y-dimension (0.20m)
        name="mirror glass overlaps frame (seated)"
    )

    # Mirror centered on pivot rod (X-axis only, Y is intentionally taller)
    ctx.expect_within(
        mirror_plate, wall_base,
        axes="x",
        margin=0.005,
        name="mirror centered on pivot rod X"
    )

    # Material/color contrast: wall base (white) vs mirror frame (black)
    # (This is validated by design, but we can check visual names exist)
    ctx.check(
        "all visible details have named visuals",
        len(mirror_plate.visuals) == 2 and len(wall_base.visuals) == 1,
        details=f"Wall visuals: {len(wall_base.visuals)}, Mirror visuals: {len(mirror_plate.visuals)}"
    )

    return ctx.report()


object_model = build_object_model()
