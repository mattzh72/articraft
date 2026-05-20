from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_machine")
    
    mat_cast_iron = Material(name="cast_iron", rgba=(0.2, 0.25, 0.25, 1.0))
    mat_steel = Material(name="steel", rgba=(0.7, 0.7, 0.75, 1.0))
    mat_warning = Material(name="warning", rgba=(0.9, 0.6, 0.1, 1.0))
    
    # Base
    base = model.part("base")
    base_cq = (
        cq.Workplane("XY")
        .box(1.0, 0.8, 0.1, centered=(True, True, False))
        .edges("|Z").fillet(0.05)
    )
    base.visual(mesh_from_cadquery(base_cq, "base_mesh"), material=mat_cast_iron, name="base_vis")
    
    # Column
    column = model.part("column")
    column_cq = (
        cq.Workplane("XY")
        .circle(0.15)
        .extrude(1.2)
    )
    column.visual(
        mesh_from_cadquery(column_cq, "column_mesh"),
        origin=Origin(xyz=(0, 0, 0.1)),
        material=mat_steel,
        name="column_vis"
    )
    
    # Fixed base to column
    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin()
    )
    
    # Arm
    arm = model.part("arm")
    collar = cq.Workplane("XY").circle(0.22).extrude(0.3).faces(">Z").hole(0.298)
    beam = cq.Workplane("XY").box(1.3, 0.15, 0.25, centered=(True, True, False)).translate((0.85, 0, 0.025))
    arm_cq = collar.union(beam)
    
    arm.visual(
        mesh_from_cadquery(arm_cq, "arm_mesh"),
        material=mat_cast_iron,
        name="arm_vis"
    )
    
    # Carriage
    carriage = model.part("carriage")
    carriage_cq = (
        cq.Workplane("XY")
        .box(0.3, 0.25, 0.35, centered=(True, True, False))
        .faces("<Z")
        .workplane()
        .rect(0.4, 0.148)
        .cutBlind(-0.248) # Beam height is 0.25, so cut is slightly smaller to overlap
    )
    carriage.visual(
        mesh_from_cadquery(carriage_cq, "carriage_mesh"),
        material=mat_warning,
        name="carriage_vis"
    )
    
    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=arm,
        origin=Origin(xyz=(0, 0, 0.9)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57, effort=100.0, velocity=1.0)
    )
    
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.4, 0, 0.025)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.9, effort=100.0, velocity=1.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "column", "arm",
        reason="The arm collar is intentionally modeled as slightly overlapping the column for a tight fit."
    )
    ctx.allow_overlap(
        "arm", "carriage",
        reason="The carriage block wraps around the arm beam and slightly overlaps it for captured sliding."
    )
    
    ctx.expect_overlap("arm", "carriage", axes="x", min_overlap=0.2)
    with ctx.pose(arm_to_carriage=0.9):
        ctx.expect_overlap("arm", "carriage", axes="x", min_overlap=0.2)
    
    return ctx.report()

object_model = build_object_model()
