from __future__ import annotations

import math
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
    model = ArticulatedObject(name="wrist")

    # 1. Rear Support (Bridge)
    # A bridge with two pillars and a crossbeam.
    rear_support_cq = (
        cq.Workplane("XY")
        # Base plates
        .box(0.06, 0.04, 0.02)
        .translate((0, 0.06, 0.01))
        .union(
            cq.Workplane("XY").box(0.06, 0.04, 0.02).translate((0, -0.06, 0.01))
        )
        # Pillars
        .union(
            cq.Workplane("XY").box(0.04, 0.04, 0.08).translate((0, 0.06, 0.06))
        )
        .union(
            cq.Workplane("XY").box(0.04, 0.04, 0.08).translate((0, -0.06, 0.06))
        )
        # Crossbeam
        .union(
            cq.Workplane("XY").box(0.06, 0.16, 0.02).translate((0, 0, 0.11))
        )
    )
    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(rear_support_cq, "rear_support_mesh"),
        name="rear_support_visual"
    )

    # 2. Yaw Stage
    # Rotates around Z on top of the crossbeam (Z=0.12).
    yaw_stage_cq = (
        cq.Workplane("XY")
        # Base cylinder
        .cylinder(0.01, 0.03)
        .translate((0, 0, 0.005))
        # Support plate
        .union(
            cq.Workplane("XY").box(0.04, 0.08, 0.01).translate((0, 0, 0.015))
        )
        # Left arm
        .union(
            cq.Workplane("XY").box(0.04, 0.01, 0.06).translate((0, 0.035, 0.05))
        )
        # Right arm
        .union(
            cq.Workplane("XY").box(0.04, 0.01, 0.06).translate((0, -0.035, 0.05))
        )
    )
    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(yaw_stage_cq, "yaw_stage_mesh"),
        name="yaw_stage_visual"
    )
    
    yaw_pin_cq = cq.Workplane("XY").cylinder(0.02, 0.005).translate((0, 0, -0.01))
    yaw_stage.visual(
        mesh_from_cadquery(yaw_pin_cq, "yaw_pin_mesh"),
        name="yaw_pin"
    )

    model.articulation(
        name="support_to_yaw",
        articulation_type=ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0, 0, 0.12)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=2.0),
    )

    # 3. Pitch Frame
    # Rotates around Y between the arms.
    # Yaw arms go up to Z=0.08 (relative to yaw stage).
    # Pitch joint at Z=0.06 relative to yaw stage.
    pitch_frame_cq = (
        cq.Workplane("XY")
        .box(0.04, 0.058, 0.04)
        # Add a small protrusion for the roll joint
        .union(
            cq.Workplane("YZ").cylinder(0.02, 0.02).translate((0.03, 0, 0))
        )
    )
    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(pitch_frame_cq, "pitch_frame_mesh"),
        name="pitch_frame_visual"
    )
    
    pitch_pin_cq = cq.Workplane("XZ").cylinder(0.08, 0.005)
    pitch_frame.visual(
        mesh_from_cadquery(pitch_pin_cq, "pitch_pin_mesh"),
        name="pitch_pin"
    )

    model.articulation(
        name="yaw_to_pitch",
        articulation_type=ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0, 0, 0.06)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(lower=-math.pi/2, upper=math.pi/2, effort=10.0, velocity=2.0),
    )

    # 4. Roll Spindle
    # Rotates around X at the tip of the pitch frame.
    # Protrusion ends at X=0.04 relative to pitch frame.
    roll_spindle_cq = (
        cq.Workplane("YZ")
        # Base flange
        .cylinder(0.005, 0.018)
        .translate((0.0025, 0, 0))
        # Spindle shaft
        .union(
            cq.Workplane("YZ").cylinder(0.03, 0.01).translate((0.02, 0, 0))
        )
    )
    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(roll_spindle_cq, "roll_spindle_mesh"),
        name="roll_spindle_visual"
    )
    
    roll_pin_cq = cq.Workplane("YZ").cylinder(0.01, 0.005).translate((-0.005, 0, 0))
    roll_spindle.visual(
        mesh_from_cadquery(roll_pin_cq, "roll_pin_mesh"),
        name="roll_pin"
    )

    model.articulation(
        name="pitch_to_roll",
        articulation_type=ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(xyz=(0.04, 0, 0)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=5.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")

    ctx.allow_overlap(rear_support, yaw_stage, elem_a="rear_support_visual", elem_b="yaw_pin", reason="Yaw pin is captured in the support crossbeam.")
    ctx.allow_overlap(pitch_frame, yaw_stage, elem_a="pitch_pin", elem_b="yaw_stage_visual", reason="Pitch pin is captured in the yaw stage arms.")
    ctx.allow_overlap(pitch_frame, roll_spindle, elem_a="pitch_frame_visual", elem_b="roll_pin", reason="Roll pin is captured in the pitch frame.")

    ctx.expect_gap(yaw_stage, rear_support, axis="z", positive_elem="yaw_stage_visual", max_penetration=0.001)
    ctx.expect_overlap(yaw_stage, rear_support, axes="xy", min_overlap=0.01)

    ctx.expect_within(pitch_frame, yaw_stage, axes="y", margin=0.002)
    ctx.expect_gap(roll_spindle, pitch_frame, axis="x", positive_elem="roll_spindle_visual", max_penetration=0.001)

    return ctx.report()


object_model = build_object_model()
