from __future__ import annotations

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
    model = ArticulatedObject(name="sunroof_cassette")

    # 1. Cassette Frame
    # Outer dimensions: 1.0m (X) x 0.8m (Y) x 0.06m (Z)
    frame_cq = cq.Workplane("XY").box(1.0, 0.8, 0.06)
    
    # Main opening (through all)
    # 0.9m (X) x 0.7m (Y)
    frame_cq = frame_cq.cut(cq.Workplane("XY").box(0.9, 0.7, 0.1))
    
    # Rail slots for sliding vent
    # Slots cut into the inner side walls
    # Z center = -0.01, height = 0.022 (Z from -0.021 to 0.001)
    frame_cq = frame_cq.cut(cq.Workplane("XY").workplane(offset=-0.01).box(0.94, 0.74, 0.022))
    
    # Ledge for fixed glass at the rear
    # Z center = 0.025, height = 0.01 (Z from 0.02 to 0.03)
    frame_cq = frame_cq.cut(cq.Workplane("XY").workplane(offset=0.025).center(0.3, 0).box(0.34, 0.74, 0.01))
    
    # Crossbar to support the front edge of the fixed glass
    # Z center = 0.0125, height = 0.015 (Z from 0.005 to 0.02)
    crossbar = cq.Workplane("XY").workplane(offset=0.0125).center(0.14, 0).box(0.02, 0.74, 0.015)
    frame_cq = frame_cq.union(crossbar)

    frame_part = model.part("cassette_frame")
    frame_part.visual(
        mesh_from_cadquery(frame_cq, "frame_mesh"),
        name="frame_visual"
    )

    # 2. Fixed Glass Panel
    # Dimensions: X=0.342, Y=0.742, Z=0.01
    # Rubber seal frame
    fixed_seal_cq = cq.Workplane("XY").box(0.342, 0.742, 0.01)
    fixed_seal_cq = fixed_seal_cq.cut(cq.Workplane("XY").box(0.3, 0.7, 0.02))
    
    # Glass pane
    fixed_glass_cq = cq.Workplane("XY").box(0.31, 0.71, 0.005)
    
    fixed_part = model.part("fixed_glass")
    fixed_part.visual(
        mesh_from_cadquery(fixed_seal_cq, "fixed_seal_mesh"),
        origin=Origin(xyz=(0.3, 0.0, 0.025)),
        name="fixed_seal"
    )
    fixed_part.visual(
        mesh_from_cadquery(fixed_glass_cq, "fixed_glass_pane_mesh"),
        origin=Origin(xyz=(0.3, 0.0, 0.025)),
        name="fixed_glass_pane"
    )
    
    # Rigidly mount fixed glass to frame
    model.articulation(
        "frame_to_fixed_glass",
        ArticulationType.FIXED,
        parent=frame_part,
        child=fixed_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # 3. Sliding Vent
    # Dimensions: X=0.6, Y=0.742, Z=0.022
    # Frame part of the sliding vent
    vent_frame_cq = cq.Workplane("XY").box(0.6, 0.742, 0.022)
    vent_frame_cq = vent_frame_cq.cut(cq.Workplane("XY").box(0.5, 0.66, 0.03))
    
    # Glass part of the sliding vent
    vent_glass_cq = cq.Workplane("XY").box(0.52, 0.68, 0.006)
    
    vent_part = model.part("sliding_vent")
    vent_part.visual(
        mesh_from_cadquery(vent_frame_cq, "vent_frame_mesh"),
        origin=Origin(xyz=(-0.16, 0.0, -0.01)),
        name="vent_frame"
    )
    vent_part.visual(
        mesh_from_cadquery(vent_glass_cq, "vent_glass_mesh"),
        origin=Origin(xyz=(-0.16, 0.0, -0.01)),
        name="vent_glass"
    )

    # Articulation: Prismatic
    # Slides rearward (+X) under the fixed glass
    model.articulation(
        "vent_slide",
        ArticulationType.PRISMATIC,
        parent=frame_part,
        child=vent_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.3, effort=50.0, velocity=0.2)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("cassette_frame")
    fixed = object_model.get_part("fixed_glass")
    vent = object_model.get_part("sliding_vent")
    slide_joint = object_model.get_articulation("vent_slide")
    
    # Allowances for seated/proxy fits
    ctx.allow_overlap(frame, fixed, elem_a="frame_visual", elem_b="fixed_seal", reason="Fixed glass seal is seated and sealed into the frame ledge.")
    ctx.allow_overlap(frame, vent, elem_a="frame_visual", elem_b="vent_frame", reason="Sliding vent frame sits inside the frame tracks.")
    
    # Check that fixed glass is within the frame in XY
    ctx.expect_within(fixed, frame, axes="xy", margin=0.01, name="fixed glass is contained in frame footprint")
    
    # Check that vent is within the frame in XY at rest
    ctx.expect_within(vent, frame, axes="xy", margin=0.01, name="closed vent is contained in frame footprint")
    
    # Check that vent is below fixed glass
    ctx.expect_gap(fixed, vent, axis="z", min_gap=0.0, name="vent sits below fixed glass level")
    
    with ctx.pose({slide_joint: 0.3}):
        # When open, it should still be within the frame
        ctx.expect_within(vent, frame, axes="xy", margin=0.01, name="open vent is contained in frame footprint")
        
        # It should overlap with the fixed glass in XY, proving it slid under it
        ctx.expect_overlap(vent, fixed, axes="xy", min_overlap=0.1, name="open vent slides under fixed glass")

    return ctx.report()


object_model = build_object_model()
