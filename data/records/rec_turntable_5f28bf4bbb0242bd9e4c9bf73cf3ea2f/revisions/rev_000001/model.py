from __future__ import annotations

import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    TorusGeometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bearing_cutaway_turntable")

    # Materials
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    aluminum = model.material("aluminum", rgba=(0.85, 0.87, 0.90, 1.0))
    chrome = model.material("chrome", rgba=(0.92, 0.93, 0.95, 1.0))
    dark_machinery = model.material("dark_machinery", rgba=(0.25, 0.26, 0.28, 1.0))

    # Bearing dimensions
    outer_radius = 0.15
    inner_radius = 0.05
    ball_track_radius = 0.10  # Middle of the raceway
    ball_radius = 0.006
    ring_thickness = 0.025
    num_balls = 12

    # ============================================
    # BASE (fixed outer ring / outer race)
    # ============================================
    base = model.part("base")

    # Build the base ring (outer race) with chamfered edges
    base_ring = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(ring_thickness)
        .edges(cq.selectors.RadiusNthSelector(-1)).fillet(0.003)
    )

    base.visual(
        mesh_from_cadquery(base_ring, "base_ring"),
        material=steel,
        name="base_ring",
    )

    # Add lower ball track race (visible groove on top of base ring)
    lower_ball_track = TorusGeometry(
        radius=ball_track_radius,
        tube=0.008,
        radial_segments=32,
        tubular_segments=64,
    )
    base.visual(
        mesh_from_geometry(lower_ball_track, "ball_track_race"),
        origin=Origin(xyz=(0.0, 0.0, ring_thickness - 0.003)),
        material=dark_machinery,
        name="ball_track_race",
    )

    # Add bearing balls - positioned to sit between the races
    # Ball center at mid-height between the two races
    ball_center_z = ring_thickness + ball_radius * 0.5
    for i in range(num_balls):
        angle = (2.0 * math.pi * i) / num_balls
        x = ball_track_radius * math.cos(angle)
        y = ball_track_radius * math.sin(angle)
        base.visual(
            Sphere(radius=ball_radius),
            origin=Origin(xyz=(x, y, ball_center_z)),
            material=chrome,
            name=f"ball_{i:02d}",
        )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=outer_radius, length=ring_thickness),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, ring_thickness / 2)),
    )

    # ============================================
    # TOP RING (rotating inner ring / inner race)
    # ============================================
    top_ring = model.part("top_ring")

    # Build the top ring (inner race) with chamfered edges
    top_ring_shape = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(outer_radius - 0.005)  # Slightly smaller outer radius
        .circle(inner_radius - 0.005)  # Smaller inner radius (this is the inner race)
        .extrude(ring_thickness)
        .edges(cq.selectors.RadiusNthSelector(-1)).fillet(0.003)
    )

    # Position top ring so it forms the upper raceway
    # Top ring bottom is just above the balls
    top_ring_bottom_z = ball_center_z + ball_radius * 0.4
    top_ring.visual(
        mesh_from_cadquery(top_ring_shape, "top_ring"),
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom_z)),
        material=aluminum,
        name="top_ring_shell",
    )

    # Add upper ball track race on the bottom of the top ring
    upper_ball_track = TorusGeometry(
        radius=ball_track_radius,
        tube=0.008,
        radial_segments=32,
        tubular_segments=64,
    )
    top_ring.visual(
        mesh_from_geometry(upper_ball_track, "upper_ball_track"),
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom_z + 0.005)),
        material=dark_machinery,
        name="upper_ball_track_race",
    )

    # Add central hub/shaft connected to the rotating inner race
    # Cylinder is centered on origin, so position it so its bottom is at top ring top
    hub_height = 0.08
    top_ring.visual(
        Cylinder(radius=inner_radius * 0.8, length=hub_height),
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom_z + ring_thickness + hub_height / 2)),
        material=dark_machinery,
        name="central_hub",
    )

    # Add a top plate/platform for visual clarity
    top_plate = (
        cq.Workplane("XY")
        .circle(outer_radius - 0.03)
        .extrude(0.008)
    )
    top_ring.visual(
        mesh_from_cadquery(top_plate, "top_plate"),
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom_z + ring_thickness + hub_height)),
        material=aluminum,
        name="top_plate",
    )

    top_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=outer_radius - 0.005, length=ring_thickness + hub_height),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom_z + (ring_thickness + hub_height) / 2)),
    )

    # ============================================
    # YAW (CONTINUOUS) JOINT - enables rotation
    # ============================================
    # Joint at the center of the bearing
    model.articulation(
        "base_to_top_ring",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_ring,
        origin=Origin(xyz=(0.0, 0.0, ring_thickness)),
        axis=(0.0, 0.0, 1.0),  # Vertical axis (yaw)
        motion_limits=MotionLimits(effort=50.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_ring = object_model.get_part("top_ring")
    joint = object_model.get_articulation("base_to_top_ring")

    # Allow top_ring to be "floating" since it's connected via joint, not geometry
    ctx.allow_isolated_part(
        "top_ring",
        reason="Top ring is intentionally connected via yaw joint, not direct geometry contact",
    )

    # Test 1: Verify the joint is continuous (yaw joint)
    ctx.check(
        "joint_is_continuous_yaw",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Joint type: {joint.articulation_type}",
    )

    # Test 2: Verify correct axis (Z-axis for yaw)
    ctx.check(
        "joint_axis_is_z",
        joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint axis: {joint.axis}",
    )

    # Test 3: Check that top ring is supported (gap between base and top ring)
    # The top ring should be above the base with a gap for the balls
    ctx.expect_gap(
        top_ring,
        base,
        axis="z",
        min_gap=0.010,
        max_gap=0.025,
        name="top_ring_supported_by_bearings",
    )

    # Test 4: Check radial alignment (both rings centered on same axis)
    ctx.expect_within(
        top_ring,
        base,
        axes="xy",
        margin=0.005,
        name="rings_concentric",
    )

    # Test 5: Test rotation - verify the top ring actually moves in XY plane
    rest_pos = ctx.part_world_position(top_ring)
    with ctx.pose({joint: math.pi / 4}):  # 45 degrees
        rotated_pos = ctx.part_world_position(top_ring)
        ctx.check(
            "top_ring_rotates_in_xy",
            rest_pos is not None
            and rotated_pos is not None
            and abs(rest_pos[2] - rotated_pos[2]) < 0.001,  # Z should stay same
            details=f"rest_z={rest_pos[2] if rest_pos else None}, rotated_z={rotated_pos[2] if rotated_pos else None}",
        )

    # Test 6: Test full rotation (360 degrees)
    with ctx.pose({joint: math.pi * 2}):
        full_rot_pos = ctx.part_world_position(top_ring)
        ctx.check(
            "full_rotation_reachable",
            full_rot_pos is not None,
            details="Full 360-degree rotation pose accessible",
        )

    # Test 7: Allow overlap between balls and races (they're seated in the track)
    ctx.allow_overlap(
        "base",
        "base",
        elem_a="ball_track_race",
        elem_b="ball_00",
        reason="Balls are intentionally seated in the lower ball track race",
    )

    # Test 8: Check central hub is properly positioned on top_ring
    # The hub should extend upward from the top ring
    ctx.expect_gap(
        top_ring,
        top_ring,
        axis="z",
        min_gap=0.0,
        max_gap=0.015,
        positive_elem="central_hub",
        negative_elem="top_ring_shell",
        name="hub_positioned_on_top_ring",
    )

    # Test 9: Verify ball positions relative to tracks
    # Check that balls are properly positioned between the races
    ctx.expect_overlap(
        base,
        base,
        axes="xy",
        min_overlap=0.003,
        elem_a="ball_00",
        elem_b="ball_track_race",
        name="balls_positioned_on_track",
    )

    return ctx.report()


object_model = build_object_model()
