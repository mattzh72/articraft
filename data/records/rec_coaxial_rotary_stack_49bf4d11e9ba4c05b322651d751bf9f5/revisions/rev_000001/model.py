import cadquery as cq
import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def make_stage(radius: float, height: float, hole_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(height, radius)
        .faces(">Z").workplane()
        .hole(hole_radius * 2)
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_coaxial_rotary_stack")

    base = model.part("base")
    # Base plate
    base.visual(Cylinder(radius=0.15, height=0.05), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="plate")
    # Central shaft
    base.visual(Cylinder(radius=0.02, height=0.18), origin=Origin(xyz=(0.0, 0.0, 0.140)), name="shaft")
    # Collars to support the stages
    base.visual(Cylinder(radius=0.03, height=0.01), origin=Origin(xyz=(0.0, 0.0, 0.055)), name="collar_1")
    base.visual(Cylinder(radius=0.03, height=0.02), origin=Origin(xyz=(0.0, 0.0, 0.110)), name="collar_2")
    base.visual(Cylinder(radius=0.03, height=0.02), origin=Origin(xyz=(0.0, 0.0, 0.170)), name="collar_3")
    base.visual(Cylinder(radius=0.03, height=0.01), origin=Origin(xyz=(0.0, 0.0, 0.225)), name="top_cap")

    # Stage 1
    stage_1 = model.part("stage_1")
    stage_1_shape = make_stage(0.12, 0.039, 0.021)
    stage_1.visual(
        mesh_from_cadquery(stage_1_shape, "stage_1_mesh"),
        origin=Origin(),
        name="body"
    )
    model.articulation(
        "stage_1_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0795)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0)
    )

    # Stage 2
    stage_2 = model.part("stage_2")
    stage_2_shape = make_stage(0.09, 0.039, 0.021)
    stage_2.visual(
        mesh_from_cadquery(stage_2_shape, "stage_2_mesh"),
        origin=Origin(),
        name="body"
    )
    model.articulation(
        "stage_2_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.1395)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0)
    )

    # Stage 3
    stage_3 = model.part("stage_3")
    stage_3_shape = make_stage(0.06, 0.039, 0.021)
    stage_3.visual(
        mesh_from_cadquery(stage_3_shape, "stage_3_mesh"),
        origin=Origin(),
        name="body"
    )
    model.articulation(
        "stage_3_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.1995)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    # Verify that the stages are contained within the base plate footprint
    ctx.expect_within(stage_1, base, axes="xy", inner_elem="body", outer_elem="plate")
    ctx.expect_within(stage_2, base, axes="xy", inner_elem="body", outer_elem="plate")
    ctx.expect_within(stage_3, base, axes="xy", inner_elem="body", outer_elem="plate")

    # Verify they are separated by gap
    ctx.expect_gap(stage_2, stage_1, axis="z", min_gap=0.02, max_gap=0.025)
    ctx.expect_gap(stage_3, stage_2, axis="z", min_gap=0.02, max_gap=0.025)

    # Verify gaps to collars
    # Stage 1 sits exactly on collar 1
    ctx.expect_contact(stage_1, base, elem_a="body", elem_b="collar_1", contact_tol=1e-5)
    # Stage 1 has a gap to collar 2
    ctx.expect_gap(base, stage_1, axis="z", positive_elem="collar_2", negative_elem="body", min_gap=0.0005, max_gap=0.002)
    
    # Stage 2 sits exactly on collar 2
    ctx.expect_contact(stage_2, base, elem_a="body", elem_b="collar_2", contact_tol=1e-5)
    # Stage 2 has a gap to collar 3
    ctx.expect_gap(base, stage_2, axis="z", positive_elem="collar_3", negative_elem="body", min_gap=0.0005, max_gap=0.002)

    # Stage 3 sits exactly on collar 3
    ctx.expect_contact(stage_3, base, elem_a="body", elem_b="collar_3", contact_tol=1e-5)
    # Stage 3 has a gap to top cap
    ctx.expect_gap(base, stage_3, axis="z", positive_elem="top_cap", negative_elem="body", min_gap=0.0005, max_gap=0.002)
    
    return ctx.report()

object_model = build_object_model()