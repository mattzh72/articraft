import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_freezer")
    
    # Cabinet
    cabinet = model.part("cabinet")
    cab_shape = (
        cq.Workplane("XY")
        .box(1.5, 0.8, 0.94)
        .translate((0, 0, 0.47))
        .cut(cq.Workplane("XY").box(1.4, 0.7, 0.85).translate((0, 0, 0.475)))
        .cut(cq.Workplane("XY").box(1.44, 0.72, 0.02).translate((0, 0, 0.91)))
        .cut(cq.Workplane("XY").box(1.44, 0.74, 0.02).translate((0, 0, 0.93)))
    )
    cabinet.visual(mesh_from_cadquery(cab_shape, "cabinet_shell"), name="shell")

    # Left lid
    lid_left = model.part("lid_left")
    lid_left.visual(Box((0.73, 0.71, 0.018)), origin=Origin(xyz=(0, 0, 0.009)), name="panel")
    
    model.articulation(
        "slide_left",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_left,
        origin=Origin(xyz=(-0.35, 0, 0.90)),
        axis=(1.0, 0.0, 0.0), # Slides right
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.65)
    )

    # Right lid
    lid_right = model.part("lid_right")
    lid_right_shape = (
        cq.Workplane("XY")
        .box(0.73, 0.73, 0.018)
        .translate((0, 0, 0.009))
        .cut(cq.Workplane("XY").box(0.3, 0.3, 0.018).translate((0, 0, 0.009)))
    )
    lid_right.visual(mesh_from_cadquery(lid_right_shape, "lid_right_shell"), name="panel")
    
    model.articulation(
        "slide_right",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_right,
        origin=Origin(xyz=(0.35, 0, 0.92)),
        axis=(-1.0, 0.0, 0.0), # Slides left
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.65)
    )

    # Service hatch
    service_hatch = model.part("service_hatch")
    service_hatch.visual(Box((0.34, 0.34, 0.018)), origin=Origin(xyz=(0, -0.17, 0.009)), name="hatch_panel")
    
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=lid_right,
        child=service_hatch,
        origin=Origin(xyz=(0, 0.17, 0.018)),
        axis=(-1.0, 0.0, 0.0), # Opens upward
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=2.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    cabinet = object_model.get_part("cabinet")
    lid_left = object_model.get_part("lid_left")
    lid_right = object_model.get_part("lid_right")
    service_hatch = object_model.get_part("service_hatch")
    
    ctx.allow_overlap(cabinet, lid_left, reason="Lid slides inside the recessed cabinet track.")
    ctx.allow_overlap(cabinet, lid_right, reason="Lid slides inside the recessed cabinet track.")
    
    ctx.expect_within(lid_left, cabinet, axes="xy")
    ctx.expect_within(lid_right, cabinet, axes="xy")
    
    ctx.expect_contact(service_hatch, lid_right)
    
    slide_left = object_model.get_articulation("slide_left")
    with ctx.pose({slide_left: 0.6}):
        ctx.expect_overlap(lid_left, lid_right, axes="xy", min_overlap=0.4)
        
    return ctx.report()

object_model = build_object_model()
