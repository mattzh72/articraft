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


def create_tripod_base() -> cq.Workplane:
    # Hub spans Z from 0.105 to 0.145
    hub = cq.Workplane("XY").cylinder(0.04, 0.025).translate((0, 0, 0.125))
    
    # Azimuth lock knob on the side of the hub
    # YZ workplane -> cylinder axis along X
    az_knob = cq.Workplane("YZ").cylinder(0.01, 0.005).translate((0.028, 0, 0.125))
    hub = hub.union(az_knob)
    
    # Legs
    leg = (
        cq.Workplane("XY")
        .cylinder(0.15, 0.008)
        .rotate((0, 0, 0), (0, 1, 0), -30)
        .translate((0.03, 0, 0.065))
    )
    
    legs = leg
    legs = legs.union(leg.rotate((0, 0, 0), (0, 0, 1), 120))
    legs = legs.union(leg.rotate((0, 0, 0), (0, 0, 1), 240))
    
    return hub.union(legs)


def create_tripod_head() -> cq.Workplane:
    # Base spans Z from 0 to 0.01
    base = cq.Workplane("XY").cylinder(0.01, 0.025).translate((0, 0, 0.005))
    
    # Left arm
    left_arm = (
        cq.Workplane("XY")
        .box(0.03, 0.01, 0.16)
        .translate((0, 0.02, 0.09))
    )
    
    # Right arm
    right_arm = (
        cq.Workplane("XY")
        .box(0.03, 0.01, 0.16)
        .translate((0, -0.02, 0.09))
    )
    
    # Altitude lock knob on the right arm
    # XZ workplane -> cylinder axis along Y
    alt_knob = cq.Workplane("XZ").cylinder(0.01, 0.006).translate((0, -0.03, 0.15))
    right_arm = right_arm.union(alt_knob)
    
    return base.union(left_arm).union(right_arm)


def create_telescope_tube() -> cq.Workplane:
    # Main tube
    tube = cq.Workplane("YZ").cylinder(0.20, 0.014).translate((0, 0, 0))
    
    # Altitude trunnions to mount in the yoke
    trunnions = cq.Workplane("XZ").cylinder(0.03, 0.005).translate((0, 0, 0))
    
    # Dew shield at the front (+X)
    dew_shield = cq.Workplane("YZ").cylinder(0.06, 0.018).translate((0.10, 0, 0))
    
    # Eyepiece assembly at the back (-X)
    eyepiece_tube = cq.Workplane("YZ").cylinder(0.04, 0.005).translate((-0.12, 0, 0))
    eyepiece = cq.Workplane("YZ").cylinder(0.015, 0.008).translate((-0.1475, 0, 0))
    
    # Finder scope
    finder_tube = cq.Workplane("YZ").cylinder(0.06, 0.004).translate((0.02, 0, 0.02))
    finder_eyepiece = cq.Workplane("YZ").cylinder(0.01, 0.006).translate((-0.015, 0, 0.02))
    finder_stalk = cq.Workplane("XY").box(0.01, 0.002, 0.01).translate((0.02, 0, 0.015))
    
    scope = tube.union(trunnions).union(dew_shield).union(eyepiece_tube).union(eyepiece)
    scope = scope.union(finder_tube).union(finder_eyepiece).union(finder_stalk)
    return scope


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_telescope")
    
    # 1. Base
    base = model.part("tripod_base")
    base.visual(
        mesh_from_cadquery(create_tripod_base(), "tripod_base_mesh"),
        name="base_visual"
    )
    
    # 2. Head (Azimuth)
    head = model.part("tripod_head")
    head.visual(
        mesh_from_cadquery(create_tripod_head(), "tripod_head_mesh"),
        name="head_visual"
    )
    
    model.articulation(
        name="azimuth_joint",
        articulation_type=ArticulationType.CONTINUOUS,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    
    # 3. Telescope Tube (Altitude)
    tube = model.part("telescope_tube")
    tube.visual(
        mesh_from_cadquery(create_telescope_tube(), "telescope_tube_mesh"),
        name="tube_visual"
    )
    
    model.articulation(
        name="altitude_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=head,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,  # ~ -20 degrees (pointing down slightly)
            upper=1.05,   # ~ 60 degrees (pointing up)
            effort=1.0,
            velocity=1.0
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("tripod_base")
    head = object_model.get_part("tripod_head")
    tube = object_model.get_part("telescope_tube")
    
    # Allow intentional overlap for the trunnions mounting into the yoke arms
    ctx.allow_overlap(tube, head, reason="The trunnions mount into the yoke arms")
    
    # Basic containment/clearance checks
    ctx.expect_contact(head, base, name="head rests on base")
    
    # Check that tube stays between the arms
    ctx.expect_within(
        tube, head, axes="y", margin=0.001,
        name="tube fits between the yoke arms"
    )
    
    # Test altitude motion
    alt_joint = object_model.get_articulation("altitude_joint")
    with ctx.pose({alt_joint: 1.0}):
        # Pointed up, there should be no collision
        ctx.expect_gap(tube, base, axis="z", min_gap=0.01, name="tube clears base when pointing up")
    
    return ctx.report()


object_model = build_object_model()