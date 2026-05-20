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
    model = ArticulatedObject(name="underslung_rotary_module")

    # 1. Top support bracket
    # Base XY plane, extrude up for plate, extrude down for column
    bracket_cq = (
        cq.Workplane("XY")
        .rect(0.12, 0.12)
        .extrude(0.015)  # Plate: Z 0 to 0.015
        .edges("|Z")
        .fillet(0.01)
        .faces("<Z")
        .workplane()
        .circle(0.04)
        .extrude(0.05)  # Column: Z 0 down to -0.05
    )
    # Mounting holes on top plate
    bracket_cq = (
        bracket_cq.faces(">Z")
        .workplane()
        .rect(0.09, 0.09, forConstruction=True)
        .vertices()
        .hole(0.006)
    )

    bracket = model.part("support_bracket")
    bracket.visual(
        mesh_from_cadquery(bracket_cq, "bracket_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),  # Shift up so column bottom is at Z=0.05
        name="bracket_visual",
    )

    # 2. Moving member (rotor)
    # Start at XY plane, extrude down for hub, extrude down further for flange
    rotor_cq = (
        cq.Workplane("XY")
        .circle(0.035)
        .extrude(-0.03)  # Hub: Z 0 down to -0.03
        .faces("<Z")
        .workplane()
        .rect(0.10, 0.10)
        .extrude(0.015)  # Flange: Z -0.03 down to -0.045
        .edges("|Z")
        .fillet(0.01)
    )
    # Output holes on flange
    rotor_cq = (
        rotor_cq.faces("<Z")
        .workplane()
        .rect(0.07, 0.07, forConstruction=True)
        .vertices()
        .hole(0.005)
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_cadquery(rotor_cq, "rotor_mesh"),
        origin=Origin(),  # Frame is already at Z=0.049
        name="rotor_visual",
    )

    # 3. Articulation
    # Revolute joint around Z axis
    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("support_bracket")
    rotor = object_model.get_part("rotor")

    # Check that the rotor is mounted flush against the bracket column
    ctx.expect_contact(
        bracket,
        rotor,
        name="rotor_contacts_bracket",
    )

    # Check that they are aligned on XY
    ctx.expect_within(
        rotor,
        bracket,
        axes="xy",
        margin=0.0,
        name="rotor_is_centered_under_bracket",
    )

    return ctx.report()


object_model = build_object_model()
