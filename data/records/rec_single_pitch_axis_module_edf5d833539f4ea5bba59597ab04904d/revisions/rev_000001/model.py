import math
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
    model = ArticulatedObject(name="trunnion_module")

    # Base part: top support bracket with side cheeks
    base = model.part("base")
    
    # Construct the base bracket
    base_cq = (
        cq.Workplane("XY")
        .box(0.16, 0.10, 0.02)  # Top plate
        .faces("<Z")
        .workplane()
        .pushPoints([(-0.07, 0), (0.07, 0)])
        .rect(0.02, 0.08)
        .extrude(0.08)          # Side cheeks extending downwards
        .translate((0, 0, 0.07)) # Shift up so the pitch axis is at Z=0
    )
    
    # Cut holes for the trunnion pins along the X axis at Z=0
    # Hole radius is 0.0105 (0.5mm radial clearance for the 0.0100 pins)
    hole_cutter = (
        cq.Workplane("YZ")
        .workplane(offset=-0.1)
        .circle(0.0105)
        .extrude(0.2)
    )
    base_cq = base_cq.cut(hole_cutter)

    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        name="base_visual"
    )

    # Carrier part: the under-slung moving member
    carrier = model.part("carrier")
    
    # Construct the carrier block, trunnion pins, and bottom mounting face
    carrier_cq = (
        cq.Workplane("XY").box(0.11, 0.06, 0.06) # Main central block
        # Right pin
        .union(cq.Workplane("YZ").workplane(offset=0.055).circle(0.01).extrude(0.025))
        # Left pin
        .union(cq.Workplane("YZ").workplane(offset=-0.055).circle(0.01).extrude(-0.025))
        # Bottom carried face / flange
        .union(cq.Workplane("XY").workplane(offset=-0.03).box(0.08, 0.08, 0.01))
    )

    carrier.visual(
        mesh_from_cadquery(carrier_cq, "carrier_mesh"),
        name="carrier_visual"
    )

    # Pitch articulation
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-math.pi/4, upper=math.pi/4),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carrier = object_model.get_part("carrier")
    pitch_joint = object_model.get_articulation("pitch_joint")

    ctx.allow_isolated_part(carrier, reason="The carrier is suspended via trunnion pins inside clearanced base holes.")

    # The carrier should be bounded by the base cheeks along the X axis
    ctx.expect_within(carrier, base, axes="x", margin=0.001, name="carrier is bounded by base cheeks")

    # Verify that the carrier remains bounded and clears the base when tilted to its limit
    with ctx.pose({pitch_joint: math.pi/4}):
        ctx.expect_within(carrier, base, axes="x", margin=0.001, name="carrier remains bounded when tilted")

    return ctx.report()

object_model = build_object_model()
