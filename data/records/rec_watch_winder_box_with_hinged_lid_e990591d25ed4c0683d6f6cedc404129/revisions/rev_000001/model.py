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
    model = ArticulatedObject(name="watch_winder")

    # Base: 0.15 x 0.15 x 0.10
    # Centered at origin. Z goes from -0.05 to 0.05.
    base_cq = (
        cq.Workplane("XY")
        .box(0.15, 0.15, 0.10)
        .faces(">Z")
        .shell(-0.01)
    )
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        name="base_shell",
    )

    # Lid: 0.15 x 0.15 x 0.04
    # Centered at Z=0.07 in world space when closed. Z goes from 0.05 to 0.09.
    lid_cq = (
        cq.Workplane("XY")
        .box(0.15, 0.15, 0.04)
        .faces("<Z")
        .shell(-0.01)
        .faces(">Z")
        .workplane()
        .rect(0.11, 0.11)
        .cutThruAll()
    )
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(lid_cq, "lid_mesh"),
        origin=Origin(xyz=(0.0, -0.075, 0.02)),
        name="lid_shell",
    )
    
    # Glass window
    glass_cq = cq.Workplane("XY").box(0.12, 0.12, 0.005)
    lid.visual(
        mesh_from_cadquery(glass_cq, "glass_mesh"),
        origin=Origin(xyz=(0.0, -0.075, 0.035)),
        name="glass_pane",
    )

    # Cradle: cylinder holding the watch, with a spindle on the back
    # Centered at origin in world space.
    cradle_cq = (
        cq.Workplane("XZ")
        .cylinder(0.06, 0.04) # height=0.06, radius=0.04
        .faces(">Y")
        .workplane()
        .circle(0.005)
        .extrude(0.045) # spindle reaching to Y=0.075
        .faces("<Y")
        .workplane()
        .rect(0.04, 0.04)
        .cutBlind(0.02) # small recess for watch
    )
    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(cradle_cq, "cradle_mesh"),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        name="cradle_body",
    )

    # Articulations
    # Lid hinge: top back edge of the base
    # Base is 0.15x0.15x0.10. Top is Z=0.05, Back is Y=0.075
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.075, 0.05)),
        axis=(-1.0, 0.0, 0.0), # positive opens upward/backward
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=2.0),
    )

    # Cradle spindle: rotates around Y axis
    # Cradle spindle is mounted into the back wall at Y=0.065
    model.articulation(
        "cradle_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.065, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")

    # Spindle is intentionally embedded in the back wall hole
    ctx.allow_overlap(
        base, cradle,
        elem_a="base_shell",
        elem_b="cradle_body",
        reason="Spindle is mounted into the back wall"
    )
    
    # Check gap between base and lid when closed
    ctx.expect_gap(lid, base, axis="z", min_gap=-0.001, max_gap=0.001, positive_elem="lid_shell", negative_elem="base_shell")
    
    # Check lid opens
    with ctx.pose(lid_hinge=1.5):
        aabb = ctx.part_world_aabb(lid)
        ctx.check("lid_opens", aabb is not None and aabb[1][2] > 0.15, "Lid should move up when opened")

    return ctx.report()

object_model = build_object_model()
