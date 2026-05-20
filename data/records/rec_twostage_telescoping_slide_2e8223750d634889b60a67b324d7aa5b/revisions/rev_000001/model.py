import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    Mimic,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)

def make_c_channel(length: float, width: float, height: float, thickness: float, open_dir: int) -> cq.Workplane:
    w2 = width / 2
    h2 = height / 2
    t = thickness
    
    if open_dir == 1:
        pts = [
            (-w2, -h2), (w2, -h2), (w2, -h2 + t), (-w2 + t, -h2 + t),
            (-w2 + t, h2 - t), (w2, h2 - t), (w2, h2), (-w2, h2)
        ]
    else:
        pts = [
            (w2, -h2), (-w2, -h2), (-w2, -h2 + t), (w2 - t, -h2 + t),
            (w2 - t, h2 - t), (-w2, h2 - t), (-w2, h2), (w2, h2)
        ]
        
    profile = cq.Workplane("YZ").polyline(pts).close()
    return profile.extrude(length / 2, both=True)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_slide")
    
    steel_mat = Material(name="steel", rgba=(0.7, 0.7, 0.75, 1.0))
    dark_mat = Material(name="dark_metal", rgba=(0.2, 0.2, 0.2, 1.0))
    
    L = 0.40
    t = 0.002
    gap = 0.001
    
    # Base stage
    base = model.part("base")
    W0 = 0.020
    H0 = 0.040
    base_rail_cq = make_c_channel(L, W0, H0, t, 1)
    base.visual(
        mesh_from_cadquery(base_rail_cq, "base_rail_mesh"),
        name="base_rail",
        material=steel_mat,
    )
    
    # Mounting plate
    base.visual(
        Box((L, 0.005, 0.060)),
        origin=Origin(xyz=(0.0, -W0/2 - 0.0025, 0.0)),
        name="mounting_plate",
        material=dark_mat,
    )
    
    # Stage 1
    stage_1 = model.part("stage_1")
    W1 = W0 - 2*t - 2*gap
    H1 = H0 - 2*t - 2*gap
    stage_1_cq = make_c_channel(L, W1, H1, t, -1)
    stage_1.visual(
        mesh_from_cadquery(stage_1_cq, "stage_1_mesh"),
        name="mid_rail",
        material=steel_mat,
    )
    
    # Stage 2
    stage_2 = model.part("stage_2")
    W2 = W1 - 2*t - 2*gap
    H2 = H1 - 2*t - 2*gap
    stage_2_cq = make_c_channel(L, W2, H2, t, 1)
    stage_2.visual(
        mesh_from_cadquery(stage_2_cq, "stage_2_mesh"),
        name="inner_rail",
        material=steel_mat,
    )
    
    # Articulations
    # Stage 1 slides along X relative to base
    # Max extension = L - 0.05 = 0.35
    model.articulation(
        name="base_to_stage_1",
        articulation_type=ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=10.0, velocity=1.0),
    )
    
    # Stage 2 slides along X relative to stage 1
    model.articulation(
        name="stage_1_to_stage_2",
        articulation_type=ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=10.0, velocity=1.0),
        mimic=Mimic(joint="base_to_stage_1", multiplier=1.0),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    
    ctx.allow_isolated_part(
        stage_1,
        reason="Stage 1 is a nested slider with intentional clearance, proxying bearing support."
    )
    ctx.allow_isolated_part(
        stage_2,
        reason="Stage 2 is a nested slider with intentional clearance, proxying bearing support."
    )
    
    # Test centering and nesting
    ctx.expect_within(
        stage_1, base,
        axes="yz",
        inner_elem="mid_rail",
        outer_elem="base_rail",
        margin=0.002,
        name="stage_1 fits in base"
    )
    
    ctx.expect_within(
        stage_2, stage_1,
        axes="yz",
        inner_elem="inner_rail",
        outer_elem="mid_rail",
        margin=0.002,
        name="stage_2 fits in stage_1"
    )
    
    # Test retained insertion at rest
    ctx.expect_overlap(
        stage_1, base,
        axes="x",
        elem_a="mid_rail",
        elem_b="base_rail",
        min_overlap=0.39,
        name="stage_1 fully inserted at rest"
    )
    
    ctx.expect_overlap(
        stage_2, stage_1,
        axes="x",
        elem_a="inner_rail",
        elem_b="mid_rail",
        min_overlap=0.39,
        name="stage_2 fully inserted at rest"
    )
    
    # Test extension
    with ctx.pose({"base_to_stage_1": 0.35}):
        ctx.expect_overlap(
            stage_1, base,
            axes="x",
            elem_a="mid_rail",
            elem_b="base_rail",
            min_overlap=0.04,
            name="stage_1 retains insertion at max extension"
        )
        
        ctx.expect_overlap(
            stage_2, stage_1,
            axes="x",
            elem_a="inner_rail",
            elem_b="mid_rail",
            min_overlap=0.04,
            name="stage_2 retains insertion at max extension"
        )
        
        # Verify the total extension is 0.70
        base_pos = ctx.part_world_position(base)
        stage_2_pos = ctx.part_world_position(stage_2)
        if base_pos and stage_2_pos:
            ctx.check(
                "total_extension_correct",
                stage_2_pos[0] - base_pos[0] > 0.69,
                f"Expected > 0.69, got {stage_2_pos[0] - base_pos[0]}"
            )

    return ctx.report()

object_model = build_object_model()
