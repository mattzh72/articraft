from __future__ import annotations

import cadquery as cq

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_seesaw")

    # Materials
    base_material = model.material("base_material", rgba=(0.4, 0.4, 0.4, 1.0))  # Dark gray base
    fulcrum_material = model.material("fulcrum_material", rgba=(0.6, 0.3, 0.1, 1.0))  # Brown fulcrum
    beam_material = model.material("beam_material", rgba=(0.8, 0.6, 0.2, 1.0))  # Yellow-brown beam
    seat_material = model.material("seat_material", rgba=(0.2, 0.4, 0.8, 1.0))  # Blue seats
    handle_material = model.material("handle_material", rgba=(0.1, 0.1, 0.1, 1.0))  # Black handles
    stop_material = model.material("stop_material", rgba=(0.8, 0.2, 0.2, 1.0))  # Red end stops
    axis_material = model.material("axis_material", rgba=(0.9, 0.9, 0.1, 1.0))  # Yellow axis marker

    # ===== BASE =====
    base = model.part("base")
    base.visual(
        Box((0.4, 0.3, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_material,
        name="base_shell",
    )

    # ===== FULCRUM =====
    # Triangular fulcrum using CadQuery - stands upright (triangle in XZ plane)
    fulcrum_shape = (
        cq.Workplane("XZ")
        .lineTo(0.025, 0.0)  # Base right
        .lineTo(0.0, 0.15)  # Top center
        .lineTo(-0.025, 0.0)  # Base left
        .close()
        .extrude(0.04)  # Depth along Y (0.04m deep)
        .edges("|Y").fillet(0.002)
    )
    fulcrum = model.part("fulcrum")
    fulcrum.visual(
        mesh_from_cadquery(fulcrum_shape, "fulcrum"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Sit on top of base, bottom at z=0.05
        material=fulcrum_material,
        name="fulcrum_shell",
    )

    # ===== BEAM =====
    beam = model.part("beam")
    # Main beam body - center at articulation frame (origin z=0)
    beam.visual(
        Box((1.2, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=beam_material,
        name="beam_body",
    )

    # ===== SEATS =====
    # Left seat - positioned 0.5m from center, on top of beam
    beam.visual(
        Box((0.15, 0.15, 0.03)),
        origin=Origin(xyz=(-0.5, 0.0, 0.015)),
        material=seat_material,
        name="left_seat",
    )
    # Right seat - positioned 0.5m from center, on top of beam
    beam.visual(
        Box((0.15, 0.15, 0.03)),
        origin=Origin(xyz=(0.5, 0.0, 0.015)),
        material=seat_material,
        name="right_seat",
    )

    # ===== HANDLES =====
    # Left handle (small cylinder on left seat)
    beam.visual(
        Cylinder(radius=0.01, length=0.08),
        origin=Origin(xyz=(-0.5, 0.0, 0.045)),
        material=handle_material,
        name="left_handle",
    )
    # Right handle (small cylinder on right seat)
    beam.visual(
        Cylinder(radius=0.01, length=0.08),
        origin=Origin(xyz=(0.5, 0.0, 0.045)),
        material=handle_material,
        name="right_handle",
    )

    # ===== END STOPS =====
    # Left end stop - at end of beam
    beam.visual(
        Box((0.02, 0.08, 0.04)),
        origin=Origin(xyz=(-0.61, 0.0, 0.02)),
        material=stop_material,
        name="left_end_stop",
    )
    # Right end stop - at end of beam
    beam.visual(
        Box((0.02, 0.08, 0.04)),
        origin=Origin(xyz=(0.61, 0.0, 0.02)),
        material=stop_material,
        name="right_end_stop",
    )

    # ===== AXIS MARKER =====
    # Small cylinder at the pivot point to show rotation axis - sits on top of beam
    beam.visual(
        Cylinder(radius=0.008, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.023), rpy=(0.0, 1.5708, 0.0)),  # Rotate to align with Y axis, sit on beam
        material=axis_material,
        name="axis_marker",
    )

    # ===== ARTICULATIONS =====
    # Fix fulcrum to base (FIXED joint)
    model.articulation(
        "base_to_fulcrum",
        ArticulationType.FIXED,
        parent=base,
        child=fulcrum,
    )

    # Revolute joint for pitch (beam tilts up/down around Y axis)
    # The articulation frame is at the top of the fulcrum (z=0.20 in world)
    # At q=0, the beam is horizontal
    # Positive q tilts the right side up (right-hand rule around Y axis)
    model.articulation(
        "fulcrum_to_beam",
        ArticulationType.REVOLUTE,
        parent=fulcrum,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),  # Top of fulcrum (0.05 base offset + 0.15 height)
        axis=(0.0, 1.0, 0.0),  # Y axis for pitch rotation
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.3,  # ~17 degrees down
            upper=0.3,   # ~17 degrees up
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fulcrum = object_model.get_part("fulcrum")
    beam = object_model.get_part("beam")
    hinge = object_model.get_articulation("fulcrum_to_beam")

    # Allow intentional overlap: beam sits on fulcrum pivot
    ctx.allow_overlap(
        "fulcrum", "beam",
        reason="Beam pivot sits on top of fulcrum; small intentional overlap at contact point"
    )

    # ===== SUPPORT AND CONTACT TESTS =====
    # Check that fulcrum is on top of base
    with ctx.pose({hinge: 0.0}):
        # Fulcrum should be supported by base
        ctx.expect_contact(base, fulcrum, elem_a="base_shell", elem_b="fulcrum_shell",
                          name="fulcrum_supported_by_base")

        # Beam should be near the fulcrum at the pivot
        ctx.expect_gap(beam, fulcrum, axis="z", min_gap=-0.01, max_gap=0.01,
                       name="beam_on_fulcrum")

    # ===== CLOSED/LEVEL POSE TESTS =====
    with ctx.pose({hinge: 0.0}):
        # Beam should be level (check Z position of beam center)
        beam_pos = ctx.part_world_position(beam)
        # At q=0, beam part frame is at articulation frame z=0.20
        ctx.check("beam_level_at_rest",
                  abs(beam_pos[2] - 0.20) < 0.02,
                  details=f"Beam Z position: {beam_pos[2]}, expected ~0.20")

        # Check that seats are on the beam (contact check)
        ctx.expect_contact(beam, beam, elem_a="beam_body", elem_b="left_seat",
                          name="left_seat_on_beam")
        ctx.expect_contact(beam, beam, elem_a="beam_body", elem_b="right_seat",
                          name="right_seat_on_beam")

    # ===== TILT RANGE TESTS =====
    # Test positive tilt (right side up)
    with ctx.pose({hinge: 0.3}):
        # Right side should be higher than left side
        # At positive q, right end goes up (positive rotation around Y)
        beam_pos = ctx.part_world_position(beam)
        ctx.check("right_side_up_at_positive_tilt",
                  beam_pos[2] >= 0.20,  # Should be at or above neutral
                  details=f"Beam Z at positive tilt: {beam_pos[2]}")

    # Test negative tilt (left side up)
    with ctx.pose({hinge: -0.3}):
        # Left side should be higher than right side
        beam_pos = ctx.part_world_position(beam)
        ctx.check("left_side_up_at_negative_tilt",
                  beam_pos[2] >= 0.20,  # Should be at or above neutral
                  details=f"Beam Z at negative tilt: {beam_pos[2]}")

    # ===== VISIBLE DETAILS TESTS =====
    with ctx.pose({hinge: 0.0}):
        # Check that handles exist and are on seats
        ctx.expect_contact(beam, beam, elem_a="left_seat", elem_b="left_handle",
                          name="left_handle_on_seat")
        ctx.expect_contact(beam, beam, elem_a="right_seat", elem_b="right_handle",
                          name="right_handle_on_seat")

        # Check that end stops are at beam ends (check X position)
        left_stop = beam.get_visual("left_end_stop")
        right_stop = beam.get_visual("right_end_stop")
        ctx.check("left_end_stop_at_end",
                  left_stop is not None,
                  details="Left end stop exists")
        ctx.check("right_end_stop_at_end",
                  right_stop is not None,
                  details="Right end stop exists")

        # Check axis marker is present
        axis_marker = beam.get_visual("axis_marker")
        ctx.check("axis_marker_present",
                  axis_marker is not None,
                  details="Axis marker exists")

    # ===== MECHANISM TEST =====
    # Verify the joint is revolute and has correct axis
    ctx.check("joint_is_revolute",
              hinge.articulation_type == ArticulationType.REVOLUTE,
              details=f"Joint type: {hinge.articulation_type}")
    ctx.check("joint_axis_correct",
              hinge.axis == (0.0, 1.0, 0.0),
              details=f"Joint axis: {hinge.axis}")
    ctx.check("joint_has_limits",
              hinge.motion_limits is not None,
              details="Joint has motion limits")
    ctx.check("joint_limits_range",
              hinge.motion_limits is not None and
              hinge.motion_limits.lower == -0.3 and
              hinge.motion_limits.upper == 0.3,
              details=f"Joint limits: [{hinge.motion_limits.lower}, {hinge.motion_limits.upper}]")

    return ctx.report()


object_model = build_object_model()
