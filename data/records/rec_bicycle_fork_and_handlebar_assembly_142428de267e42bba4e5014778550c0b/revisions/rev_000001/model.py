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
import cadquery as cq
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fat_bike_front_end")

    # The head tube is the parent/root
    head_tube = model.part("head_tube")
    
    # Tapered head tube
    # Length: 0.12m
    # Bottom (Z=0): Outer R=0.028, Inner R=0.020
    # Top (Z=0.12): Outer R=0.024, Inner R=0.017
    ht_outer = (
        cq.Workplane("XY")
        .circle(0.028)
        .workplane(offset=0.12)
        .circle(0.024)
        .loft()
    )
    ht_inner = (
        cq.Workplane("XY")
        .circle(0.020)
        .workplane(offset=0.12)
        .circle(0.017)
        .loft()
    )
    head_tube_shape = ht_outer.cut(ht_inner)
    
    head_tube.visual(
        mesh_from_cadquery(head_tube_shape, "head_tube_mesh"),
        name="head_tube_shell"
    )

    fork = model.part("fork")
    
    # Steerer tube
    # Tapered from R=0.019 (1.5") at bottom to R=0.0143 (1.125") at Z=0.05, then straight to Z=0.25
    steerer_taper = (
        cq.Workplane("XY")
        .circle(0.019)
        .workplane(offset=0.05)
        .circle(0.0143)
        .loft()
    )
    steerer_straight = (
        cq.Workplane("XY", origin=(0, 0, 0.05))
        .circle(0.0143)
        .extrude(0.20)
    )
    steerer_shape = steerer_taper.union(steerer_straight)
    
    fork.visual(
        mesh_from_cadquery(steerer_shape, "steerer_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="steerer"
    )

    # Crown
    # Wide alloy crown. Center distance for blades is 0.15m (+/- 0.075m)
    # Width: 0.20m, Depth: 0.05m, Height: 0.04m
    crown_shape = (
        cq.Workplane("XY")
        .box(0.20, 0.05, 0.04)
        .edges("|Z")
        .fillet(0.02)
    )
    # The steerer sits on top of the crown, so crown is below Z=0.
    fork.visual(
        mesh_from_cadquery(crown_shape, "crown_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.02)), 
        name="crown"
    )

    # Blades
    # Oversized blades for fat bike.
    blade_length = 0.45
    blade_shape = (
        cq.Workplane("XY")
        .circle(0.018) # 36mm diameter at top
        .workplane(offset=blade_length)
        .circle(0.012) # 24mm diameter at bottom
        .loft()
    )
    # Rotate to point down. Top of blade is at Z=0 in its local frame.
    # We will position it at Z=-0.04 (bottom of crown) pointing down (-Z).
    blade_shape = blade_shape.rotate((0, 0, 0), (1, 0, 0), 180)
    
    fork.visual(
        mesh_from_cadquery(blade_shape, "left_blade_mesh"),
        origin=Origin(xyz=(-0.075, 0.0, -0.04)),
        name="left_blade"
    )

    fork.visual(
        mesh_from_cadquery(blade_shape, "right_blade_mesh"),
        origin=Origin(xyz=(0.075, 0.0, -0.04)),
        name="right_blade"
    )
    
    # Dropouts
    fork.visual(
        Box((0.01, 0.03, 0.04)),
        origin=Origin(xyz=(-0.075, 0.0, -0.04 - blade_length - 0.02)),
        name="left_dropout"
    )
    fork.visual(
        Box((0.01, 0.03, 0.04)),
        origin=Origin(xyz=(0.075, 0.0, -0.04 - blade_length - 0.02)),
        name="right_dropout"
    )

    # Stem
    stem_clamp_z = 0.22
    stem_length = 0.06
    # Stem base clamping to steerer
    fork.visual(
        Cylinder(radius=0.02, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, stem_clamp_z)),
        name="stem_steerer_clamp"
    )
    # Stem extension
    fork.visual(
        Box((0.03, stem_length, 0.03)),
        origin=Origin(xyz=(0.0, stem_length/2 + 0.01, stem_clamp_z)),
        name="stem_extension"
    )
    # Stem handlebar clamp
    fork.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(0.0, stem_length + 0.01, stem_clamp_z), rpy=(0.0, math.pi / 2, 0.0)),
        name="stem_handlebar_clamp"
    )

    # Handlebars
    # Wide flat handlebars, e.g., 760mm wide
    fork.visual(
        Cylinder(radius=0.0159, length=0.76), # 31.8mm diameter
        origin=Origin(xyz=(0.0, stem_length + 0.01, stem_clamp_z), rpy=(0.0, math.pi / 2, 0.0)),
        name="handlebars"
    )

    # Steering joint
    model.articulation(
        "steering_joint",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-math.pi/2, upper=math.pi/2)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("fork")
    
    ctx.expect_within(
        fork, head_tube,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube_shell",
        margin=0.005,
        name="steerer centered in head tube"
    )
    
    ctx.expect_overlap(
        fork, head_tube,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube_shell",
        min_overlap=0.10,
        name="steerer passes through head tube"
    )
    
    ctx.allow_overlap(
        fork, head_tube,
        elem_a="steerer",
        elem_b="head_tube_shell",
        reason="The steerer tube intentionally passes through the hollow head tube."
    )
    
    return ctx.report()

object_model = build_object_model()