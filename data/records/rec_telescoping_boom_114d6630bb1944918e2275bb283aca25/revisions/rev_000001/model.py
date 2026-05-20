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

def make_bracket(length: float, inner_w: float, inner_h: float, thickness: float) -> cq.Workplane:
    # A sleeve bracket that holds the first stage
    return (
        cq.Workplane("YZ")
        .rect(inner_w + 2 * thickness, inner_h + 2 * thickness)
        .rect(inner_w, inner_h)
        .extrude(length)
    )

def make_tube(length: float, width: float, height: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .rect(width - 2 * thickness, height - 2 * thickness)
        .extrude(length)
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_reach_boom")

    # Base bracket
    base_bracket = model.part("base_bracket")
    bracket_geom = make_bracket(length=0.3, inner_w=0.15, inner_h=0.15, thickness=0.01)
    base_bracket.visual(mesh_from_cadquery(bracket_geom, "base_bracket"), name="bracket_geom")

    # Stage 0
    stage_0 = model.part("stage_0")
    stage_0_geom = make_tube(length=1.0, width=0.15, height=0.15, thickness=0.005)
    stage_0.visual(mesh_from_cadquery(stage_0_geom, "stage_0"), name="stage_0_geom")

    model.articulation(
        "bracket_to_stage_0",
        ArticulationType.FIXED,
        parent=base_bracket,
        child=stage_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # Stage 1
    stage_1 = model.part("stage_1")
    stage_1_geom = make_tube(length=1.0, width=0.14, height=0.14, thickness=0.005)
    stage_1.visual(mesh_from_cadquery(stage_1_geom, "stage_1"), name="stage_1_geom")

    model.articulation(
        "stage_0_to_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.8, effort=1000.0, velocity=1.0)
    )

    # Stage 2
    stage_2 = model.part("stage_2")
    stage_2_geom = make_tube(length=1.0, width=0.13, height=0.13, thickness=0.005)
    stage_2.visual(mesh_from_cadquery(stage_2_geom, "stage_2"), name="stage_2_geom")

    model.articulation(
        "stage_1_to_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.8, effort=1000.0, velocity=1.0)
    )

    # Stage 3
    stage_3 = model.part("stage_3")
    stage_3_geom = make_tube(length=1.0, width=0.12, height=0.12, thickness=0.005)
    stage_3.visual(mesh_from_cadquery(stage_3_geom, "stage_3"), name="stage_3_geom")

    model.articulation(
        "stage_2_to_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.8, effort=1000.0, velocity=1.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allowances for exact proxy fits (coplanar surfaces treated as overlap)
    ctx.allow_overlap("base_bracket", "stage_0", reason="stage 0 is exactly fitted inside the base bracket sleeve")
    ctx.allow_overlap("stage_0", "stage_1", reason="stage 1 is exactly fitted inside stage 0")
    ctx.allow_overlap("stage_1", "stage_2", reason="stage 2 is exactly fitted inside stage 1")
    ctx.allow_overlap("stage_2", "stage_3", reason="stage 3 is exactly fitted inside stage 2")

    # Stage 1 in Stage 0
    ctx.expect_within(
        "stage_1", "stage_0",
        axes="yz",
        margin=0.001,
        name="stage_1 centered in stage_0"
    )
    ctx.expect_overlap(
        "stage_1", "stage_0",
        axes="x",
        min_overlap=0.15,
        name="stage_1 retains insertion in stage_0 at rest"
    )

    # Stage 2 in Stage 1
    ctx.expect_within(
        "stage_2", "stage_1",
        axes="yz",
        margin=0.001,
        name="stage_2 centered in stage_1"
    )
    ctx.expect_overlap(
        "stage_2", "stage_1",
        axes="x",
        min_overlap=0.15,
        name="stage_2 retains insertion in stage_1 at rest"
    )

    # Stage 3 in Stage 2
    ctx.expect_within(
        "stage_3", "stage_2",
        axes="yz",
        margin=0.001,
        name="stage_3 centered in stage_2"
    )
    ctx.expect_overlap(
        "stage_3", "stage_2",
        axes="x",
        min_overlap=0.15,
        name="stage_3 retains insertion in stage_2 at rest"
    )

    # Extended pose checks
    with ctx.pose(stage_0_to_1=0.8, stage_1_to_2=0.8, stage_2_to_3=0.8):
        ctx.expect_overlap(
            "stage_1", "stage_0",
            axes="x",
            min_overlap=0.15,
            name="stage_1 retains insertion when extended"
        )
        ctx.expect_overlap(
            "stage_2", "stage_1",
            axes="x",
            min_overlap=0.15,
            name="stage_2 retains insertion when extended"
        )
        ctx.expect_overlap(
            "stage_3", "stage_2",
            axes="x",
            min_overlap=0.15,
            name="stage_3 retains insertion when extended"
        )
        
        stage_0_pos = ctx.part_world_position("stage_0")
        stage_3_pos = ctx.part_world_position("stage_3")
        if stage_0_pos is not None and stage_3_pos is not None:
            ctx.check(
                "boom extends correctly",
                stage_3_pos[0] > stage_0_pos[0] + 2.0,
                details=f"stage_3 X: {stage_3_pos[0]}, stage_0 X: {stage_0_pos[0]}"
            )

    return ctx.report()

object_model = build_object_model()