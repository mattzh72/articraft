from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_slide")

    # Sleeve (Outer Stage)
    # 0.04 x 0.04 outer, 0.036 x 0.036 inner, length 0.4
    sleeve = model.part("sleeve")
    sleeve_shape = (
        cq.Workplane("XY")
        .box(0.04, 0.04, 0.4)
        .faces(">Z")
        .shell(-0.002)
    )
    sleeve.visual(
        mesh_from_cadquery(sleeve_shape, "sleeve_mesh"),
        name="sleeve_body"
    )

    # Stage 1 (Intermediate Member)
    # 0.034 x 0.034 outer, 0.030 x 0.030 inner, length 0.4
    stage_1 = model.part("stage_1")
    stage_1_shape = (
        cq.Workplane("XY")
        .box(0.034, 0.034, 0.4)
        .faces(">Z")
        .shell(-0.002)
    )
    stage_1.visual(
        mesh_from_cadquery(stage_1_shape, "stage_1_mesh"),
        name="stage_1_body"
    )

    # Stage 2 (Inner Member)
    # 0.028 x 0.028 outer, solid, length 0.4
    stage_2 = model.part("stage_2")
    stage_2_shape = (
        cq.Workplane("XY")
        .box(0.028, 0.028, 0.4)
    )
    stage_2.visual(
        mesh_from_cadquery(stage_2_shape, "stage_2_body_mesh"),
        name="stage_2_body"
    )
    
    end_plate_shape = (
        cq.Workplane("XY")
        .box(0.04, 0.04, 0.005)
        .translate((0, 0, 0.2025))
    )
    stage_2.visual(
        mesh_from_cadquery(end_plate_shape, "stage_2_end_plate_mesh"),
        name="stage_2_end_plate"
    )

    # Joint 1: Sleeve to Stage 1
    # Placed at the bottom of the sleeve, sliding along +Z
    model.articulation(
        "sleeve_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=stage_1,
        origin=Origin(xyz=(0, 0, 0.005)), # Slightly offset to avoid Z-fighting at the bottom
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.3)
    )

    # Joint 2: Stage 1 to Stage 2
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0, 0, 0.005)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.3)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    sleeve = object_model.get_part("sleeve")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    
    j1 = object_model.get_articulation("sleeve_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")

    # Allow them to be isolated since they have clearance fits
    ctx.allow_isolated_part(stage_1, reason="Sliding member with clearance fit inside sleeve")
    ctx.allow_isolated_part(stage_2, reason="Sliding member with clearance fit inside stage_1")

    # Rest pose checks
    ctx.expect_within(
        stage_1, sleeve, axes="xy", margin=0.002,
        inner_elem="stage_1_body", outer_elem="sleeve_body",
        name="stage_1 centered in sleeve"
    )
    ctx.expect_overlap(
        stage_1, sleeve, axes="z", min_overlap=0.3,
        elem_a="stage_1_body", elem_b="sleeve_body",
        name="stage_1 inserted in sleeve at rest"
    )

    ctx.expect_within(
        stage_2, stage_1, axes="xy", margin=0.002,
        inner_elem="stage_2_body", outer_elem="stage_1_body",
        name="stage_2 centered in stage_1"
    )
    ctx.expect_overlap(
        stage_2, stage_1, axes="z", min_overlap=0.3,
        elem_a="stage_2_body", elem_b="stage_1_body",
        name="stage_2 inserted in stage_1 at rest"
    )

    # Extended pose checks
    with ctx.pose({j1: 0.3, j2: 0.3}):
        ctx.expect_within(
            stage_1, sleeve, axes="xy", margin=0.002,
            inner_elem="stage_1_body", outer_elem="sleeve_body",
            name="extended stage_1 centered in sleeve"
        )
        ctx.expect_overlap(
            stage_1, sleeve, axes="z", min_overlap=0.08,
            elem_a="stage_1_body", elem_b="sleeve_body",
            name="extended stage_1 retains insertion"
        )

        ctx.expect_within(
            stage_2, stage_1, axes="xy", margin=0.002,
            inner_elem="stage_2_body", outer_elem="stage_1_body",
            name="extended stage_2 centered in stage_1"
        )
        ctx.expect_overlap(
            stage_2, stage_1, axes="z", min_overlap=0.08,
            elem_a="stage_2_body", elem_b="stage_1_body",
            name="extended stage_2 retains insertion"
        )

    return ctx.report()

object_model = build_object_model()
