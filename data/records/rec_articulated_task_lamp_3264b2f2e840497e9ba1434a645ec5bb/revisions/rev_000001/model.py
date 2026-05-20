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


def build_clamp() -> cq.Workplane:
    # C-clamp profile
    clamp = (
        cq.Workplane("XZ")
        .moveTo(-0.03, 0)
        .lineTo(0.05, 0)
        .lineTo(0.05, 0.015)
        .lineTo(0, 0.015)
        .lineTo(0, 0.085)
        .lineTo(0.05, 0.085)
        .lineTo(0.05, 0.10)
        .lineTo(-0.03, 0.10)
        .close()
        .extrude(0.02, both=True)
    )
    
    # Screw and handle
    screw = (
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .center(0.025, 0)
        .cylinder(0.05, 0.004, centered=(True, True, False))
    )
    handle = (
        cq.Workplane("XY")
        .workplane(offset=-0.015)
        .center(0.025, 0)
        .cylinder(0.005, 0.015, centered=(True, True, False))
    )
    clamp = clamp.union(screw).union(handle)
    
    # Boss on top
    clamp = clamp.union(
        cq.Workplane("XY")
        .workplane(offset=0.10)
        .center(0.01, 0)
        .cylinder(0.03, 0.015, centered=(True, True, False))
    )
    
    # Clevis ears
    clamp = clamp.union(
        cq.Workplane("XY")
        .workplane(offset=0.13)
        .center(0.01, 0.01)
        .box(0.02, 0.005, 0.02, centered=(True, True, False))
    ).union(
        cq.Workplane("XY")
        .workplane(offset=0.13)
        .center(0.01, -0.01)
        .box(0.02, 0.005, 0.02, centered=(True, True, False))
    )
    
    # Rounded tops for ears
    clamp = clamp.union(
        cq.Workplane("XZ")
        .workplane(offset=0.01)
        .center(0.01, 0.15)
        .cylinder(0.005, 0.01, centered=(True, True, True))
    ).union(
        cq.Workplane("XZ")
        .workplane(offset=-0.01)
        .center(0.01, 0.15)
        .cylinder(0.005, 0.01, centered=(True, True, True))
    )
    
    return clamp


def build_lower_arm() -> cq.Workplane:
    # Main rod
    arm = (
        cq.Workplane("XY")
        .box(0.015, 0.01, 0.35, centered=(True, True, False))
    )
    
    # Rounded bottom
    arm = arm.union(
        cq.Workplane("XZ")
        .cylinder(0.01, 0.0075, centered=(True, True, True))
    )
    
    # Bottom hinge pin
    arm = arm.union(
        cq.Workplane("XZ")
        .cylinder(0.025, 0.004, centered=(True, True, True))
    )
    
    # Top clevis
    ear1 = (
        cq.Workplane("XY")
        .workplane(offset=0.35)
        .center(0, 0.0075)
        .box(0.015, 0.005, 0.02, centered=(True, True, False))
    )
    ear2 = (
        cq.Workplane("XY")
        .workplane(offset=0.35)
        .center(0, -0.0075)
        .box(0.015, 0.005, 0.02, centered=(True, True, False))
    )
    arm = arm.union(ear1).union(ear2)
    
    # Rounded tops for top clevis
    arm = arm.union(
        cq.Workplane("XZ")
        .workplane(offset=0.0075)
        .center(0, 0.37)
        .cylinder(0.005, 0.0075, centered=(True, True, True))
    ).union(
        cq.Workplane("XZ")
        .workplane(offset=-0.0075)
        .center(0, 0.37)
        .cylinder(0.005, 0.0075, centered=(True, True, True))
    )
    
    # Spring decoration
    spring = (
        cq.Workplane("XY")
        .workplane(offset=0.05)
        .center(-0.015, 0)
        .cylinder(0.25, 0.003, centered=(True, True, False))
    )
    pin1 = (
        cq.Workplane("YZ")
        .workplane(offset=-0.015)
        .center(0, 0.05)
        .cylinder(0.015, 0.002, centered=(True, True, False))
    )
    pin2 = (
        cq.Workplane("YZ")
        .workplane(offset=-0.015)
        .center(0, 0.30)
        .cylinder(0.015, 0.002, centered=(True, True, False))
    )
    
    return arm.union(spring).union(pin1).union(pin2)


def build_upper_arm() -> cq.Workplane:
    # Main rod
    arm = (
        cq.Workplane("XY")
        .box(0.015, 0.008, 0.35, centered=(True, True, False))
    )
    
    # Rounded bottom
    arm = arm.union(
        cq.Workplane("XZ")
        .cylinder(0.008, 0.0075, centered=(True, True, True))
    )
    
    # Bottom hinge pin
    arm = arm.union(
        cq.Workplane("XZ")
        .cylinder(0.02, 0.004, centered=(True, True, True))
    )
    
    # Top clevis
    ear1 = (
        cq.Workplane("XY")
        .workplane(offset=0.35)
        .center(0, 0.006)
        .box(0.015, 0.004, 0.02, centered=(True, True, False))
    )
    ear2 = (
        cq.Workplane("XY")
        .workplane(offset=0.35)
        .center(0, -0.006)
        .box(0.015, 0.004, 0.02, centered=(True, True, False))
    )
    arm = arm.union(ear1).union(ear2)
    
    # Rounded tops
    arm = arm.union(
        cq.Workplane("XZ")
        .workplane(offset=0.006)
        .center(0, 0.37)
        .cylinder(0.004, 0.0075, centered=(True, True, True))
    ).union(
        cq.Workplane("XZ")
        .workplane(offset=-0.006)
        .center(0, 0.37)
        .cylinder(0.004, 0.0075, centered=(True, True, True))
    )
    
    # Spring decoration
    spring = (
        cq.Workplane("XY")
        .workplane(offset=0.05)
        .center(-0.015, 0)
        .cylinder(0.25, 0.003, centered=(True, True, False))
    )
    pin1 = (
        cq.Workplane("YZ")
        .workplane(offset=-0.015)
        .center(0, 0.05)
        .cylinder(0.015, 0.002, centered=(True, True, False))
    )
    pin2 = (
        cq.Workplane("YZ")
        .workplane(offset=-0.015)
        .center(0, 0.30)
        .cylinder(0.015, 0.002, centered=(True, True, False))
    )
    
    return arm.union(spring).union(pin1).union(pin2)


def build_shade() -> cq.Workplane:
    # Hollow cylinder body
    shade = (
        cq.Workplane("XY")
        .workplane(offset=-0.10)
        .center(0.11, 0)
        .cylinder(0.12, 0.05, centered=(True, True, False))
        .faces("<Z")
        .hole(0.096, 0.11)
    )
    
    # Mounting tab
    tab = (
        cq.Workplane("YZ")
        .workplane(offset=0)
        .center(0, 0)
        .box(0.006, 0.015, 0.11, centered=(True, True, False))
    )
    
    # Rounded top for tab
    tab_top = (
        cq.Workplane("XZ")
        .workplane(offset=0)
        .center(0, 0)
        .cylinder(0.006, 0.0075, centered=(True, True, True))
    )
    
    # Hinge pin
    tab_pin = (
        cq.Workplane("XZ")
        .workplane(offset=0)
        .center(0, 0)
        .cylinder(0.016, 0.004, centered=(True, True, True))
    )
    
    return shade.union(tab).union(tab_top).union(tab_pin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="task_lamp")
    
    clamp_mesh = mesh_from_cadquery(build_clamp(), "clamp_mesh")
    lower_arm_mesh = mesh_from_cadquery(build_lower_arm(), "lower_arm_mesh")
    upper_arm_mesh = mesh_from_cadquery(build_upper_arm(), "upper_arm_mesh")
    shade_mesh = mesh_from_cadquery(build_shade(), "shade_mesh")
    
    clamp = model.part("clamp")
    clamp.visual(clamp_mesh, name="clamp_visual")
    
    lower_arm = model.part("lower_arm")
    lower_arm.visual(lower_arm_mesh, name="lower_arm_visual")
    
    upper_arm = model.part("upper_arm")
    upper_arm.visual(upper_arm_mesh, name="upper_arm_visual")
    
    shade = model.part("shade")
    shade.visual(shade_mesh, name="shade_visual")
    
    # Articulations
    # 1. Clamp to lower arm
    model.articulation(
        "clamp_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=lower_arm,
        origin=Origin(xyz=(0.01, 0.0, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=1.0),
    )
    
    # 2. Lower arm to upper arm
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-2.0, upper=2.0),
    )
    
    # 3. Upper arm to shade
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.5, upper=1.5),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    clamp = object_model.get_part("clamp")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    
    # Allow hinge pins inside clevis ears
    ctx.allow_overlap(lower_arm, clamp, reason="Hinge pin inside clevis ears")
    ctx.allow_overlap(upper_arm, lower_arm, reason="Hinge pin inside clevis ears")
    ctx.allow_overlap(shade, upper_arm, reason="Hinge pin inside clevis ears")
    
    # Check that arms are properly contained in their respective clevis gaps at rest
    ctx.expect_within(lower_arm, clamp, axes="y", margin=0.005)
    ctx.expect_within(upper_arm, lower_arm, axes="y", margin=0.005)
    
    # Verify the lamp can fold
    joint1 = object_model.get_articulation("clamp_to_lower_arm")
    joint2 = object_model.get_articulation("lower_arm_to_upper_arm")
    joint3 = object_model.get_articulation("upper_arm_to_shade")
    
    with ctx.pose({joint1: 0.5, joint2: -1.0, joint3: 0.5}):
        ctx.expect_within(lower_arm, clamp, axes="y", margin=0.005)
        ctx.expect_within(upper_arm, lower_arm, axes="y", margin=0.005)

    return ctx.report()


object_model = build_object_model()
