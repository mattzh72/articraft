import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject("telescoping_mast")

    base_box_cq = (
        cq.Workplane("XY")
        .box(0.4, 0.4, 0.2, centered=(True, True, False))
        .edges("|Z").chamfer(0.05)
        .faces(">Z").circle(0.05).cutBlind(-0.15)
    )

    base_sleeve_cq = (
        cq.Workplane("XY").workplane(offset=0.2)
        .circle(0.06).extrude(0.8)
        .faces(">Z").circle(0.05).cutBlind(-0.8)
    )

    middle_cq = (
        cq.Workplane("XY")
        .circle(0.0475).extrude(0.8)
        .faces(">Z").workplane().circle(0.055).extrude(0.05)
        .faces(">Z").circle(0.0375).cutBlind(-0.85)
    )

    inner_cq = (
        cq.Workplane("XY")
        .circle(0.035).extrude(0.85)
        .faces(">Z").workplane().circle(0.045).extrude(0.05)
        .faces(">Z").circle(0.025).cutBlind(-0.9)
    )

    pan_head_base_cq = (
        cq.Workplane("XY")
        .circle(0.045).extrude(0.02)
    )

    pan_head_body_cq = (
        cq.Workplane("XY").workplane(offset=0.02)
        .box(0.12, 0.12, 0.10, centered=(True, True, False))
        .edges("|Z").chamfer(0.02)
    )

    pan_head_lens_cq = (
        cq.Workplane("XZ").workplane(offset=0.06)
        .center(0, 0.07)
        .circle(0.03).extrude(0.02)
    )

    mat_yellow = Material(name="yellow", color=(0.9, 0.6, 0.1))
    mat_dark = Material(name="dark", color=(0.2, 0.2, 0.2))
    mat_silver = Material(name="silver", color=(0.8, 0.8, 0.8))
    mat_white = Material(name="white", color=(0.9, 0.9, 0.9))
    mat_black = Material(name="black", color=(0.1, 0.1, 0.1))

    base = model.part("base")
    base.visual(mesh_from_cadquery(base_box_cq, "base_box"), material=mat_yellow, name="base_box")
    base.visual(mesh_from_cadquery(base_sleeve_cq, "base_sleeve"), material=mat_dark, name="base_sleeve")

    middle = model.part("middle_section")
    middle.visual(mesh_from_cadquery(middle_cq, "middle_section"), material=mat_silver, name="middle_tube")

    inner = model.part("inner_section")
    inner.visual(mesh_from_cadquery(inner_cq, "inner_section"), material=mat_silver, name="inner_tube")

    pan_head = model.part("pan_head")
    pan_head.visual(mesh_from_cadquery(pan_head_base_cq, "pan_head_base"), material=mat_dark, name="pan_head_base")
    pan_head.visual(mesh_from_cadquery(pan_head_body_cq, "pan_head_body"), material=mat_white, name="pan_head_body")
    pan_head.visual(mesh_from_cadquery(pan_head_lens_cq, "pan_head_lens"), material=mat_black, name="pan_head_lens")

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.7, effort=50.0, velocity=0.5),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.75, effort=50.0, velocity=0.5),
    )

    model.articulation(
        "inner_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=inner,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.9)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14, effort=5.0, velocity=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.expect_within("middle_section", "base", axes="xy", name="middle section centered in base")
    ctx.expect_within("inner_section", "middle_section", axes="xy", name="inner section centered in middle")
    ctx.expect_within("pan_head", "inner_section", axes="xy", inner_elem="pan_head_base", outer_elem="inner_tube", name="pan head base centered on inner")
    
    ctx.expect_overlap("middle_section", "base", axes="z", min_overlap=0.1, name="middle section retained in base")
    ctx.expect_overlap("inner_section", "middle_section", axes="z", min_overlap=0.1, name="inner section retained in middle")
    
    ctx.allow_overlap(
        "middle_section", "base",
        elem_a="middle_tube", elem_b="base_sleeve",
        reason="The middle section is intentionally represented as sliding inside the base sleeve."
    )
    ctx.allow_overlap(
        "inner_section", "middle_section",
        elem_a="inner_tube", elem_b="middle_tube",
        reason="The inner section is intentionally represented as sliding inside the middle section."
    )
    
    with ctx.pose(base_to_middle=0.7, middle_to_inner=0.75):
        ctx.expect_overlap("middle_section", "base", axes="z", min_overlap=0.05, name="extended middle retains insertion")
        ctx.expect_overlap("inner_section", "middle_section", axes="z", min_overlap=0.05, name="extended inner retains insertion")
        
        base_pos = ctx.part_world_position("base")
        mid_pos = ctx.part_world_position("middle_section")
        in_pos = ctx.part_world_position("inner_section")
        
        if base_pos and mid_pos and in_pos:
            ctx.check("middle extends upward", mid_pos[2] > base_pos[2] + 0.6)
            ctx.check("inner extends upward", in_pos[2] > mid_pos[2] + 0.6)

    return ctx.report()

object_model = build_object_model()