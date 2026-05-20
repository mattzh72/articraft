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
    model = ArticulatedObject("sliding_window")

    # Outer frame: 1.2m wide, 1.0m high, 0.1m deep.
    # Opening: 1.1m wide, 0.9m high.
    outer_frame_shape = (
        cq.Workplane("XY")
        .box(1.2, 0.1, 1.0)
        .cut(cq.Workplane("XY").box(1.1, 0.1, 0.9))
    )
    outer_frame = model.part("outer_frame")
    outer_frame.visual(mesh_from_cadquery(outer_frame_shape, "outer_frame_mesh"))

    # Fixed sash (left side)
    # Width 0.58, height 0.92, depth 0.03
    fixed_sash_shape = (
        cq.Workplane("XY")
        .box(0.58, 0.03, 0.92)
        .cut(cq.Workplane("XY").box(0.48, 0.03, 0.82))
    )
    fixed_glass = cq.Workplane("XY").box(0.48, 0.01, 0.82)
    fixed_sash_shape = fixed_sash_shape.union(fixed_glass)
    
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(mesh_from_cadquery(fixed_sash_shape, "fixed_sash_mesh"))
    
    # Fixed sash center at X=-0.27, Y=0.035, Z=0.0
    # This embeds it 0.01m into the left, top, and bottom frame members.
    model.articulation(
        "outer_to_fixed",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_sash,
        origin=Origin(xyz=(-0.27, 0.035, 0.0))
    )

    # Sliding sash (right side)
    # Width 0.58, height 0.92, depth 0.03
    sliding_sash_shape = (
        cq.Workplane("XY")
        .box(0.58, 0.03, 0.92)
        .cut(cq.Workplane("XY").box(0.48, 0.03, 0.82))
    )
    sliding_glass = cq.Workplane("XY").box(0.48, 0.01, 0.82)
    sliding_sash_shape = sliding_sash_shape.union(sliding_glass)

    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(mesh_from_cadquery(sliding_sash_shape, "sliding_sash_mesh"))

    # Sliding sash joint at X=0.27, Y=0.0, Z=0.0
    # This centers the sliding sash on the Y-axis (main body axis) and embeds it into the right, top, and bottom.
    model.articulation(
        "sliding_joint",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.27, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0), # Positive q opens to the left
        motion_limits=MotionLimits(lower=0.0, upper=0.54, effort=50.0, velocity=1.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    outer_frame = object_model.get_part("outer_frame")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    sliding_joint = object_model.get_articulation("sliding_joint")

    # Sashes are embedded into the outer frame tracks
    ctx.allow_overlap(fixed_sash, outer_frame, reason="Fixed sash is captured in the outer frame")
    ctx.allow_overlap(sliding_sash, outer_frame, reason="Sliding sash is captured in the outer frame tracks")
    
    # The two sashes should not overlap each other
    ctx.expect_gap(fixed_sash, sliding_sash, axis="y", min_gap=0.001)

    # Check that sliding sash retains insertion in the frame
    ctx.expect_overlap(sliding_sash, outer_frame, axes="z", min_overlap=0.01)

    # Check that sliding sash opens correctly
    with ctx.pose({sliding_joint: 0.54}):
        # When fully open, sliding sash moves to the left and overlaps with fixed sash in X
        ctx.expect_overlap(sliding_sash, fixed_sash, axes="x", min_overlap=0.5)

    return ctx.report()


object_model = build_object_model()
