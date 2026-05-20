import math
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
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_air_pump")

    # BASE
    base = model.part("base")
    
    # Base guide for stabilizer
    # Box 0.2 x 0.1 x 0.05, shelled to create a tube along X
    guide_cq = (
        cq.Workplane("XY")
        .box(0.2, 0.1, 0.05)
        .faces(">X or <X")
        .shell(-0.01)
    ).translate((0, 0, 0.025))
    
    base.visual(
        mesh_from_cadquery(guide_cq, "base_guide"),
        name="guide",
    )

    # Cylinder
    # Tube overlapping the guide slightly to ensure connectivity
    cylinder_cq = (
        cq.Workplane("XY")
        .circle(0.04)
        .circle(0.016)
        .extrude(0.58)
    ).translate((0, 0, 0.049))
    
    base.visual(
        mesh_from_cadquery(cylinder_cq, "cylinder"),
        name="cylinder",
    )

    # STABILIZER BAR
    stabilizer = model.part("stabilizer_bar")
    stabilizer.visual(
        Box((0.4, 0.07, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        name="bar",
    )
    
    model.articulation(
        "stabilizer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stabilizer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    # PISTON
    piston = model.part("piston")
    piston.visual(
        Cylinder(radius=0.015, height=0.61),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        name="rod",
    )
    piston.visual(
        Cylinder(radius=0.02, height=0.3),
        origin=Origin(xyz=(0.0, 0.0, 0.65), rpy=(math.pi/2, 0.0, 0.0)),
        name="t_bar",
    )
    
    model.articulation(
        "piston_pump",
        ArticulationType.PRISMATIC,
        parent=base,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=2.0, lower=0.0, upper=0.5),
    )

    # GRIP
    grip = model.part("grip")
    
    # Sleeve with a folding handle lever
    grip_cq = (
        cq.Workplane("XY")
        .circle(0.03)
        .circle(0.021)
        .extrude(0.1)
    ).translate((0, 0, -0.05))
    
    lever = (
        cq.Workplane("XY")
        .rect(0.08, 0.02)
        .extrude(0.1)
    ).translate((0.04, 0, -0.05))
    
    grip_cq = grip_cq.union(lever)
    
    grip.visual(
        mesh_from_cadquery(grip_cq, "grip_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="sleeve",
    )
    
    model.articulation(
        "grip_spin",
        ArticulationType.CONTINUOUS,
        parent=piston,
        child=grip,
        origin=Origin(xyz=(0.0, 0.1, 0.65)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    stabilizer = object_model.get_part("stabilizer_bar")
    piston = object_model.get_part("piston")
    grip = object_model.get_part("grip")
    
    ctx.allow_overlap(stabilizer, base, elem_a="bar", elem_b="guide", reason="stabilizer bar slides inside guide proxy")
    ctx.allow_overlap(grip, piston, elem_a="sleeve", elem_b="t_bar", reason="grip sleeve wraps around the crossbar proxy")

    
    # Stabilizer fits inside the base guide
    ctx.expect_within(stabilizer, base, axes="yz", inner_elem="bar", outer_elem="guide", margin=0.01)
    ctx.expect_overlap(stabilizer, base, axes="x", elem_a="bar", elem_b="guide", min_overlap=0.1)
    
    # Piston rod fits inside the cylinder
    ctx.expect_within(piston, base, axes="xy", inner_elem="rod", outer_elem="cylinder", margin=0.01)
    ctx.expect_overlap(piston, base, axes="z", elem_a="rod", elem_b="cylinder", min_overlap=0.1)
    
    # Grip fits around the T-bar
    ctx.expect_within(piston, grip, axes="xz", inner_elem="t_bar", outer_elem="sleeve", margin=0.01)
    ctx.expect_overlap(grip, piston, axes="y", elem_a="sleeve", elem_b="t_bar", min_overlap=0.05)

    with ctx.pose(piston_pump=0.4):
        ctx.expect_overlap(piston, base, axes="z", elem_a="rod", elem_b="cylinder", min_overlap=0.05)
        
    with ctx.pose(stabilizer_slide=0.15):
        ctx.expect_overlap(stabilizer, base, axes="x", elem_a="bar", elem_b="guide", min_overlap=0.05)

    return ctx.report()

object_model = build_object_model()
