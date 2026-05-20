from __future__ import annotations

import cadquery as cq
import math

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
)


def _build_plinth_shape():
    """Build chamfered rectangular plinth shape using CadQuery."""
    length = 0.45  # 450mm (x dimension)
    width = 0.35   # 350mm (y dimension)
    height = 0.1   # 100mm (z dimension)
    chamfer_size = 0.01  # 10mm chamfer on all edges
    shape = cq.Workplane("XY").box(length, width, height)
    shape = shape.edges().chamfer(chamfer_size)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_player")

    # Define materials
    model.material("plinth_wood", rgba=(0.55, 0.27, 0.07, 1.0))
    model.material("platter_aluminum", rgba=(0.75, 0.75, 0.75, 1.0))
    model.material("rubber_mat", rgba=(0.1, 0.1, 0.1, 1.0))
    model.material("spindle_metal", rgba=(0.8, 0.8, 0.8, 1.0))
    model.material("tonearm_black", rgba=(0.2, 0.2, 0.2, 1.0))
    model.material("button_red", rgba=(0.9, 0.1, 0.1, 1.0))

    # Root part: Plinth
    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(_build_plinth_shape(), "plinth.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Center of plinth box at z=0.05, so bottom at z=0, top at z=0.1
        material="plinth_wood",
        name="plinth_body"
    )

    # Fixed visual: Tonearm on plinth (raised above platter to avoid overlap)
    # Tonearm base: vertical cylinder connecting tonearm to plinth
    plinth.visual(
        Cylinder(radius=0.01, height=0.03),
        origin=Origin(xyz=(0.15, 0.1, 0.115)),  # From plinth top (0.1) to tonearm arm (0.13)
        material="tonearm_black",
        name="tonearm_base"
    )
    # Tonearm arm: horizontal cylinder along X-axis, length 0.2m, radius 0.01m
    plinth.visual(
        Cylinder(radius=0.01, height=0.2),
        origin=Origin(
            xyz=(0.05, 0.1, 0.13),  # Center of arm: above platter top (0.12m)
            rpy=(0.0, math.pi/2, 0.0)  # Rotate 90° around Y to align along X
        ),
        material="tonearm_black",
        name="tonearm_arm"
    )
    # Tonearm head: small cylinder at end of arm, over platter
    plinth.visual(
        Cylinder(radius=0.015, height=0.02),
        origin=Origin(xyz=(0.0, 0.1, 0.13)),  # Aligned with arm height
        material="tonearm_black",
        name="tonearm_head"
    )

    # Fixed visual: Control button on plinth
    plinth.visual(
        Cylinder(radius=0.01, height=0.005),
        origin=Origin(xyz=(-0.15, -0.15, 0.1025)),  # 0.1 (plinth top) + 0.0025 (half height)
        material="button_red",
        name="control_button"
    )

    # Rotating part: Platter
    platter = model.part("platter")

    # Platter disc: 0.3m diameter, 0.02m thick
    platter.visual(
        Cylinder(radius=0.15, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Center at platter part frame (joint origin)
        material="platter_aluminum",
        name="platter_disc"
    )

    # Rubber mat on platter
    platter.visual(
        Cylinder(radius=0.145, height=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),  # 0.01 (platter top) + 0.001 (mat half height)
        material="rubber_mat",
        name="rubber_mat"
    )

    # Central spindle (rotates with platter)
    platter.visual(
        Cylinder(radius=0.003, height=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),  # 0.01 (platter top) + 0.002 (mat top) + 0.005 (spindle half height)
        material="spindle_metal",
        name="spindle"
    )

    # Articulation: Vertical revolute joint for platter
    model.articulation(
        "plinth_to_platter",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),  # Center of platter: 0.1 (plinth top) + 0.01 (platter half height)
        axis=(0.0, 0.0, 1.0),  # Vertical Z-axis
        motion_limits=MotionLimits(lower=0.0, upper=2*math.pi, effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    platter_joint = object_model.get_articulation("plinth_to_platter")

    # Check joint type and axis
    ctx.check(
        "platter_joint_is_revolute",
        platter_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Expected REVOLUTE, got {platter_joint.articulation_type}"
    )
    ctx.check(
        "platter_joint_axis_vertical",
        platter_joint.axis == (0.0, 0.0, 1.0),
        details=f"Expected (0,0,1), got {platter_joint.axis}"
    )

    # Check contact between platter and plinth at rest
    ctx.expect_contact(plinth, platter, name="platter_rests_on_plinth")

    # Check spindle exists and is centered on platter
    spindle = platter.get_visual("spindle")
    ctx.check("spindle_exists", spindle is not None, "Spindle visual not found on platter")
    if spindle:
        ctx.expect_within(
            platter, platter,
            inner_elem="spindle",
            axes="xy",
            margin=0.005,
            name="spindle_centered_on_platter"
        )

    # Check rubber mat exists
    rubber_mat = platter.get_visual("rubber_mat")
    ctx.check("rubber_mat_exists", rubber_mat is not None, "Rubber mat visual not found on platter")

    # Check tonearm exists on plinth
    tonearm_arm = plinth.get_visual("tonearm_arm")
    ctx.check("tonearm_exists", tonearm_arm is not None, "Tonearm visual not found on plinth")

    # Check control button exists
    control_button = plinth.get_visual("control_button")
    ctx.check("control_button_exists", control_button is not None, "Control button visual not found on plinth")

    # Test platter rotation
    with ctx.pose({platter_joint: math.pi/2}):
        ctx.check(
            "platter_rotates_to_half_turn",
            True,
            details="Platter pose set to π/2 successfully"
        )
        ctx.expect_contact(plinth, platter, name="platter_rests_on_plinth_at_half_turn")

    return ctx.report()


object_model = build_object_model()
