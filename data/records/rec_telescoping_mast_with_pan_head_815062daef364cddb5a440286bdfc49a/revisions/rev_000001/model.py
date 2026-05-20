from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_pan_head")
    
    mast_sleeve = model.part("mast_sleeve")
    
    mast_sleeve.visual(
        Cylinder(radius=0.05, height=0.48),
        origin=Origin(xyz=(0, 0, 0.24)),
        name="tube"
    )
    mast_sleeve.visual(
        Cylinder(radius=0.055, height=0.02),
        origin=Origin(xyz=(0, 0, 0.49)),
        name="collar"
    )
    mast_sleeve.visual(
        Cylinder(radius=0.08, height=0.02),
        origin=Origin(xyz=(0, 0, -0.01)),
        name="base_plate"
    )
    
    stage_1 = model.part("stage_1")
    stage_1.visual(
        Cylinder(radius=0.038, height=0.48),
        origin=Origin(xyz=(0, 0, 0.24)),
        name="tube"
    )
    stage_1.visual(
        Cylinder(radius=0.043, height=0.02),
        origin=Origin(xyz=(0, 0, 0.49)),
        name="collar"
    )
    
    stage_2 = model.part("stage_2")
    stage_2.visual(
        Cylinder(radius=0.026, height=0.48),
        origin=Origin(xyz=(0, 0, 0.24)),
        name="rod"
    )
    stage_2.visual(
        Cylinder(radius=0.031, height=0.02),
        origin=Origin(xyz=(0, 0, 0.49)),
        name="collar"
    )
    
    pan_plate = model.part("pan_plate")
    pan_plate.visual(
        Cylinder(radius=0.06, height=0.02),
        origin=Origin(xyz=(0, 0, 0.01)),
        name="plate"
    )
    pan_plate.visual(
        Cylinder(radius=0.003, height=0.01),
        origin=Origin(xyz=(0, 0, 0.025)),
        name="mount_screw"
    )
    
    model.articulation(
        "sleeve_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=mast_sleeve,
        child=stage_1,
        origin=Origin(xyz=(0, 0, 0.02)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.4, effort=100.0, velocity=1.0)
    )
    
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0, 0, 0.02)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.4, effort=100.0, velocity=1.0)
    )
    
    model.articulation(
        "stage_2_to_pan_plate",
        ArticulationType.REVOLUTE,
        parent=stage_2,
        child=pan_plate,
        origin=Origin(xyz=(0, 0, 0.5)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-3.14159, upper=3.14159, effort=10.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    mast_sleeve = object_model.get_part("mast_sleeve")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    pan_plate = object_model.get_part("pan_plate")
    
    # Allow overlaps for nested proxy fits
    ctx.allow_overlap(mast_sleeve, stage_1, elem_a="tube", elem_b="tube", reason="Nested proxy fit for telescoping stage")
    ctx.allow_overlap(mast_sleeve, stage_1, elem_a="collar", elem_b="tube", reason="Nested proxy fit for telescoping stage")
    ctx.allow_overlap(stage_1, stage_2, elem_a="tube", elem_b="rod", reason="Nested proxy fit for telescoping stage")
    ctx.allow_overlap(stage_1, stage_2, elem_a="collar", elem_b="rod", reason="Nested proxy fit for telescoping stage")
    ctx.allow_overlap(mast_sleeve, stage_2, elem_a="tube", elem_b="rod", reason="Nested proxy fit for telescoping stage")
    ctx.allow_overlap(mast_sleeve, stage_2, elem_a="collar", elem_b="rod", reason="Nested proxy fit for telescoping stage")
    
    ctx.expect_within(
        stage_1,
        mast_sleeve,
        axes="xy",
        inner_elem="tube",
        outer_elem="tube",
        margin=0.002,
        name="stage 1 stays centered in mast sleeve"
    )
    ctx.expect_overlap(
        stage_1,
        mast_sleeve,
        axes="z",
        elem_a="tube",
        elem_b="tube",
        min_overlap=0.1,
        name="collapsed stage 1 remains inserted in mast sleeve"
    )
    
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="xy",
        inner_elem="rod",
        outer_elem="tube",
        margin=0.002,
        name="stage 2 stays centered in stage 1"
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="z",
        elem_a="rod",
        elem_b="tube",
        min_overlap=0.1,
        name="collapsed stage 2 remains inserted in stage 1"
    )
    
    ctx.expect_gap(
        pan_plate,
        stage_2,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="plate",
        negative_elem="collar",
        name="pan plate sits on stage 2 collar"
    )
    
    sleeve_to_stage_1 = object_model.get_articulation("sleeve_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    
    with ctx.pose({sleeve_to_stage_1: 0.4, stage_1_to_stage_2: 0.4}):
        ctx.expect_within(
            stage_1,
            mast_sleeve,
            axes="xy",
            inner_elem="tube",
            outer_elem="tube",
            margin=0.002,
            name="extended stage 1 stays centered in mast sleeve"
        )
        ctx.expect_overlap(
            stage_1,
            mast_sleeve,
            axes="z",
            elem_a="tube",
            elem_b="tube",
            min_overlap=0.05,
            name="extended stage 1 retains insertion in mast sleeve"
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="xy",
            inner_elem="rod",
            outer_elem="tube",
            margin=0.002,
            name="extended stage 2 stays centered in stage 1"
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="z",
            elem_a="rod",
            elem_b="tube",
            min_overlap=0.05,
            name="extended stage 2 retains insertion in stage 1"
        )
        
        stage_2_pos = ctx.part_world_position(stage_2)
        mast_sleeve_pos = ctx.part_world_position(mast_sleeve)
        ctx.check(
            "stage 2 extends upward relative to base",
            stage_2_pos is not None and mast_sleeve_pos is not None and stage_2_pos[2] > mast_sleeve_pos[2] + 0.7,
            details=f"stage_2={stage_2_pos}, mast_sleeve={mast_sleeve_pos}"
        )

    return ctx.report()

object_model = build_object_model()