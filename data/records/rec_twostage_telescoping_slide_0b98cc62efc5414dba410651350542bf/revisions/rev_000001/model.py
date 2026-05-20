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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_slide")

    L = 0.500
    
    # Outer sleeve (base)
    base = model.part("base")
    outer_w = 0.050
    outer_t = 0.002
    outer_shape = cq.Workplane("XY").box(outer_w, L, outer_w).faces("+Y or -Y").shell(-outer_t)
    base.visual(mesh_from_cadquery(outer_shape, "outer_sleeve"), name="outer_sleeve")

    # Intermediate stage
    intermediate = model.part("intermediate")
    inter_w = 0.044
    inter_t = 0.002
    inter_shape = cq.Workplane("XY").box(inter_w, L, inter_w).faces("+Y or -Y").shell(-inter_t)
    intermediate.visual(mesh_from_cadquery(inter_shape, "inter_sleeve"), name="inter_sleeve")

    # Output stage
    output = model.part("output")
    out_w = 0.038
    out_shape = cq.Workplane("XY").box(out_w, L, out_w)
    output.visual(mesh_from_cadquery(out_shape, "out_member"), name="out_member")
    
    # End plate
    end_plate_shape = cq.Workplane("XY").box(0.050, 0.005, 0.050)
    output.visual(mesh_from_cadquery(end_plate_shape, "end_plate"), origin=Origin(xyz=(0.0, L/2 + 0.0025, 0.0)), name="end_plate")

    # Articulations
    # We want them to slide along the Y axis (+Y direction)
    model.articulation(
        "stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=intermediate,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=L * 0.8),
    )

    model.articulation(
        "stage_2",
        ArticulationType.PRISMATIC,
        parent=intermediate,
        child=output,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=L * 0.8),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    intermediate = object_model.get_part("intermediate")
    output = object_model.get_part("output")

    # Prove fit at rest
    ctx.expect_within(
        intermediate,
        base,
        axes="xz",
        inner_elem="inter_sleeve",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="intermediate stays centered in base"
    )
    ctx.expect_overlap(
        intermediate,
        base,
        axes="y",
        elem_a="inter_sleeve",
        elem_b="outer_sleeve",
        min_overlap=0.490,
        name="intermediate remains inserted in base at rest"
    )
    
    ctx.expect_within(
        output,
        intermediate,
        axes="xz",
        inner_elem="out_member",
        outer_elem="inter_sleeve",
        margin=0.002,
        name="output stays centered in intermediate"
    )
    ctx.expect_overlap(
        output,
        intermediate,
        axes="y",
        elem_a="out_member",
        elem_b="inter_sleeve",
        min_overlap=0.490,
        name="output remains inserted in intermediate at rest"
    )

    rest_inter_pos = ctx.part_world_position(intermediate)
    rest_out_pos = ctx.part_world_position(output)

    # Prove fit at extension
    with ctx.pose(stage_1=0.4, stage_2=0.4):
        ctx.expect_within(
            intermediate,
            base,
            axes="xz",
            inner_elem="inter_sleeve",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended intermediate stays centered in base"
        )
        ctx.expect_overlap(
            intermediate,
            base,
            axes="y",
            elem_a="inter_sleeve",
            elem_b="outer_sleeve",
            min_overlap=0.080,
            name="extended intermediate retains insertion in base"
        )

        ctx.expect_within(
            output,
            intermediate,
            axes="xz",
            inner_elem="out_member",
            outer_elem="inter_sleeve",
            margin=0.002,
            name="extended output stays centered in intermediate"
        )
        ctx.expect_overlap(
            output,
            intermediate,
            axes="y",
            elem_a="out_member",
            elem_b="inter_sleeve",
            min_overlap=0.080,
            name="extended output retains insertion in intermediate"
        )
        
        ext_inter_pos = ctx.part_world_position(intermediate)
        ext_out_pos = ctx.part_world_position(output)

    ctx.check(
        "intermediate extends along +Y",
        rest_inter_pos is not None and ext_inter_pos is not None and ext_inter_pos[1] > rest_inter_pos[1] + 0.3,
    )
    ctx.check(
        "output extends along +Y",
        rest_out_pos is not None and ext_out_pos is not None and ext_out_pos[1] > rest_out_pos[1] + 0.6,
    )

    return ctx.report()

object_model = build_object_model()