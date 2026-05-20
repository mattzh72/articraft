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
    model = ArticulatedObject(name="microscope")

    # --- Base ---
    # Horseshoe base
    base_u = (
        cq.Workplane("XY")
        .box(0.15, 0.20, 0.03, centered=(True, True, False))
        .faces(">Z").workplane()
        .moveTo(0, 0.05)
        .rect(0.08, 0.15)
        .cutBlind(-0.03)
    )
    # Curved arm
    arm = (
        cq.Workplane("YZ")
        .workplane(offset=-0.025)
        .moveTo(-0.075, 0.03)
        .lineTo(-0.075, 0.3)
        .lineTo(0.05, 0.35)
        .lineTo(0.05, 0.28)
        .lineTo(-0.025, 0.25)
        .lineTo(-0.025, 0.03)
        .close()
        .extrude(0.05)
    )
    # Head / Tube mount
    head = (
        cq.Workplane("XY")
        .workplane(offset=0.23)
        .moveTo(0, 0.05)
        .cylinder(0.12, 0.03, centered=(True, True, False))
    )
    # Inclined tube
    tube = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0.05, 0.3), rotate=(-45, 0, 0))
        .circle(0.015)
        .extrude(0.15)
    )
    # Carrier support
    support = (
        cq.Workplane("XY")
        .workplane(offset=0.12)
        .moveTo(0, 0.02)
        .rect(0.08, 0.09, centered=(True, True))
        .extrude(0.0201)
    )
    
    base_body = base_u.union(arm).union(head).union(tube).union(support)
    
    base_part = model.part("base")
    base_part.visual(mesh_from_cadquery(base_body, "base_mesh"))

    # --- Nosepiece ---
    nosepiece_body = (
        cq.Workplane("XY")
        .cylinder(0.01, 0.025, centered=(True, True, False))
    )
    for angle in [0, 120, 240]:
        obj = (
            cq.Workplane("XY")
            .workplane(offset=-0.04)
            .transformed(rotate=(0, 0, angle))
            .moveTo(0, 0.015)
            .cylinder(0.04, 0.008, centered=(True, True, False))
        )
        nosepiece_body = nosepiece_body.union(obj)

    nosepiece_part = model.part("nosepiece")
    nosepiece_part.visual(mesh_from_cadquery(nosepiece_body, "nosepiece_mesh"))

    model.articulation(
        "nosepiece_joint",
        ArticulationType.REVOLUTE,
        parent=base_part,
        child=nosepiece_part,
        origin=Origin(xyz=(0.0, 0.05, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14, effort=1.0, velocity=1.0)
    )

    # --- Y Stage (Full stage assembly) ---
    y_stage_body = (
        cq.Workplane("XY")
        .rect(0.10, 0.10)
        .circle(0.01)
        .extrude(0.01)
    )
    y_stage_part = model.part("y_stage")
    y_stage_part.visual(mesh_from_cadquery(y_stage_body, "y_stage_mesh"))

    model.articulation(
        "stage_y_slide",
        ArticulationType.PRISMATIC,
        parent=base_part,
        child=y_stage_part,
        origin=Origin(xyz=(0.0, 0.065, 0.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.02, upper=0.02, effort=10.0, velocity=0.1)
    )

    # --- X Stage (Carrier) ---
    x_stage_body = (
        cq.Workplane("XY")
        .rect(0.08, 0.02)
        .extrude(0.005)
    )
    x_stage_part = model.part("x_stage")
    x_stage_part.visual(mesh_from_cadquery(x_stage_body, "x_stage_mesh"))

    model.articulation(
        "stage_x_slide",
        ArticulationType.PRISMATIC,
        parent=y_stage_part,
        child=x_stage_part,
        origin=Origin(xyz=(0.0, 0.025, 0.0099)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.02, upper=0.02, effort=10.0, velocity=0.1)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    nosepiece = object_model.get_part("nosepiece")
    y_stage = object_model.get_part("y_stage")
    x_stage = object_model.get_part("x_stage")

    ctx.allow_overlap(base, nosepiece, reason="Nosepiece is mounted to the arm.")
    ctx.allow_overlap(y_stage, base, reason="Stage rests on carrier support.")
    ctx.allow_overlap(x_stage, y_stage, reason="Carrier slides on top of stage.")

    # We can check that the stage remains within its expected slide range.
    ctx.expect_overlap(y_stage, base, axes="xy", min_overlap=0.04)
    ctx.expect_within(x_stage, y_stage, axes="xy", margin=0.01)

    return ctx.report()

object_model = build_object_model()
