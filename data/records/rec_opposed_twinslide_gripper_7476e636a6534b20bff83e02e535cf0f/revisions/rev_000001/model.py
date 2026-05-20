from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gripper")

    # Materials
    mat_body = Material(name="body_metal", rgba=(0.25, 0.25, 0.28, 1.0))
    mat_rail = Material(name="rail_steel", rgba=(0.75, 0.75, 0.8, 1.0))
    mat_jaw = Material(name="jaw_metal", rgba=(0.15, 0.15, 0.15, 1.0))

    # Body geometry
    base_block = (
        cq.Workplane("XY")
        .box(0.08, 0.04, 0.04)
        .edges("|Z")
        .chamfer(0.004)
    )
    boss = cq.Workplane("XY").cylinder(0.005, 0.015).translate((0, 0, -0.0225))
    base_block = base_block.union(boss)

    slot = cq.Workplane("XY").box(0.09, 0.004, 0.004).translate((0, 0.015, 0.01))
    slot2 = cq.Workplane("XY").box(0.09, 0.004, 0.004).translate((0, -0.015, 0.01))
    base_block = base_block.cut(slot).cut(slot2)

    # Rail geometry
    rail_base = cq.Workplane("XY").workplane(offset=0.0215).box(0.084, 0.010, 0.003)
    rail_top = cq.Workplane("XY").workplane(offset=0.0245).box(0.084, 0.016, 0.003)
    rail_top = rail_top.edges(">Z").chamfer(0.001)
    rail = rail_base.union(rail_top)
    
    # Jaw geometry (defined in its local frame, inner edge at +X)
    jaw_profile = (
        cq.Workplane("XZ")
        .moveTo(-0.0125, -0.005)
        .lineTo(0.0125, -0.005)
        .lineTo(0.0125, 0.006)
        .lineTo(-0.0025, 0.006)
        .lineTo(-0.0025, 0.026)
        .lineTo(0.0125, 0.026)
        .lineTo(0.0125, 0.036)
        .lineTo(-0.0125, 0.036)
        .close()
        .extrude(0.012, both=True)
    )
    jaw_profile = jaw_profile.edges("|Y").fillet(0.002)

    chan_top = cq.Workplane("XY").box(0.03, 0.016, 0.003).translate((0, 0, -0.0015))
    chan_base = cq.Workplane("XY").box(0.03, 0.010, 0.003).translate((0, 0, -0.0045))
    jaw_cq = jaw_profile.cut(chan_top).cut(chan_base)

    groove = (
        cq.Workplane("XY")
        .workplane(offset=0.031)
        .moveTo(0.0125, 0.003)
        .lineTo(0.0095, 0)
        .lineTo(0.0125, -0.003)
        .close()
        .extrude(0.02, both=True)
    )
    jaw_cq = jaw_cq.cut(groove)

    # Add parts
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(base_block, "body_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=mat_body,
        name="body_base_vis",
    )
    body.visual(
        mesh_from_cadquery(rail, "body_rail"),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=mat_rail,
        name="body_rail_vis",
    )

    jaw_0 = model.part("jaw_0")
    jaw_0.visual(
        mesh_from_cadquery(jaw_cq, "jaw_0_mesh"),
        origin=Origin(),
        material=mat_jaw,
        name="jaw_0_vis",
    )

    jaw_1 = model.part("jaw_1")
    jaw_1.visual(
        mesh_from_cadquery(jaw_cq, "jaw_1_mesh"),
        origin=Origin(),
        material=mat_jaw,
        name="jaw_1_vis",
    )

    # Articulations
    # The rail top surface is at Z=0.046 in world frame.
    model.articulation(
        "jaw_0_joint",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_0,
        origin=Origin(xyz=(-0.0125, 0.0, 0.046)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=0.0, upper=0.015),
    )

    model.articulation(
        "jaw_1_joint",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_1,
        origin=Origin(xyz=(0.0125, 0.0, 0.046), rpy=(0.0, 0.0, math.pi)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=0.0, upper=0.015),
        mimic=Mimic(joint="jaw_0_joint", multiplier=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    with ctx.pose(jaw_0_joint=0.0):
        ctx.expect_contact("jaw_0", "jaw_1", name="jaws_meet_when_closed")
        
        ctx.expect_within("jaw_0", "body", axes="y", name="jaw_0_centered_on_y")
        ctx.expect_within("jaw_1", "body", axes="y", name="jaw_1_centered_on_y")
        
        ctx.expect_overlap("jaw_0", "body", axes="x", min_overlap=0.02, name="jaw_0_on_rail_x")
        ctx.expect_overlap("jaw_1", "body", axes="x", min_overlap=0.02, name="jaw_1_on_rail_x")

    with ctx.pose(jaw_0_joint=0.015):
        # At max open, they should still be on the rail
        ctx.expect_overlap("jaw_0", "body", axes="x", min_overlap=0.005, name="jaw_0_on_rail_open")
        ctx.expect_overlap("jaw_1", "body", axes="x", min_overlap=0.005, name="jaw_1_on_rail_open")
        
        # And they should be separated
        ctx.expect_gap("jaw_1", "jaw_0", axis="x", min_gap=0.025, name="jaws_separated_when_open")

    return ctx.report()


object_model = build_object_model()