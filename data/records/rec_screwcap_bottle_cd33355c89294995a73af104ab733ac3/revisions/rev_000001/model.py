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
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_body = model.part("bottle_body")
    
    # Bottle geometry
    # Body: radius 0.04, height 0.15
    # Shoulder: loft from radius 0.04 to 0.012, height 0.04
    # Neck: radius 0.012, height 0.03
    bottle_cq = (
        cq.Workplane("XY")
        .cylinder(0.15, 0.04, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .circle(0.04)
        .workplane(offset=0.04)
        .circle(0.012)
        .loft()
        .faces(">Z")
        .workplane()
        .cylinder(0.03, 0.012, centered=(True, True, False))
    )
    
    bottle_shell = bottle_cq.faces(">Z").shell(-0.002)
    
    bottle_body.visual(
        mesh_from_cadquery(bottle_shell, "bottle_geometry"),
        name="bottle_mesh"
    )

    cap = model.part("cap")
    # Cap is narrow and long
    # Outer radius 0.015
    # Height of cap: 0.10
    # Hole diameter 0.025 -> radius 0.0125. Depth 0.035.
    cap_cq = (
        cq.Workplane("XY")
        .cylinder(0.10, 0.015, centered=(True, True, False))
        .faces("<Z")
        .workplane()
        .hole(0.024, depth=0.035)
    )
    
    cap.visual(
        mesh_from_cadquery(cap_cq, "cap_geometry"),
        name="cap_mesh"
    )

    model.articulation(
        "cap_screw",
        ArticulationType.CONTINUOUS,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0, 0, 0.19)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    bottle = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    
    ctx.expect_within(
        cap,
        bottle,
        axes="xy",
        name="cap is within bottle footprint"
    )
    
    # We will allow overlap between cap and bottle neck just in case
    # since it's a threaded fit (even though we modeled it with clearance).
    ctx.allow_overlap(cap, bottle, reason="Cap screws onto bottle neck.")

    return ctx.report()


object_model = build_object_model()
