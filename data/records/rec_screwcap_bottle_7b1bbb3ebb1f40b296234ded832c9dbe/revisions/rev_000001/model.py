import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def make_bottle():
    bottle_radius = 0.035
    bottle_height = 0.10
    shoulder_height = 0.03
    neck_radius = 0.012
    neck_height = 0.02
    
    # Body
    body = cq.Workplane("XY").circle(bottle_radius).extrude(bottle_height)
    body = body.edges("<Z").fillet(0.005)
    
    # Shoulder
    shoulder = (
        body.faces(">Z")
        .workplane()
        .circle(bottle_radius)
        .workplane(offset=shoulder_height)
        .circle(neck_radius)
        .loft(combine=True)
    )
    
    # Neck
    neck = shoulder.faces(">Z").workplane().circle(neck_radius).extrude(neck_height)
    
    # Shell it to make it hollow
    bottle_hollow = neck.faces(">Z").shell(-0.0015)
    
    bottle_final = bottle_hollow
    
    # Threads (represented as horizontal rings)
    for offset in [0.004, 0.007, 0.010]:
        z = bottle_height + shoulder_height + offset
        thread = (
            cq.Workplane("XY").workplane(offset=z)
            .circle(neck_radius + 0.001)
            .circle(neck_radius - 0.0005)
            .extrude(0.0015)
        )
        bottle_final = bottle_final.union(thread)
    
    # Neck ring / transfer bead
    z_ring = bottle_height + shoulder_height + 0.001
    neck_ring = (
        cq.Workplane("XY").workplane(offset=z_ring)
        .circle(neck_radius + 0.002)
        .circle(neck_radius - 0.0005)
        .extrude(0.002)
    )
    bottle_final = bottle_final.union(neck_ring)
    
    return bottle_final

def make_cap():
    cap_radius = 0.014
    cap_height = 0.022
    
    # Main cap shell
    cap = (
        cq.Workplane("XY")
        .circle(cap_radius)
        .extrude(cap_height)
        .edges(">Z")
        .fillet(0.002)
        .faces("<Z")
        .shell(-0.0015)
    )
    
    # Ridges for grip
    ridges = (
        cq.Workplane("XY")
        .polarArray(cap_radius, 0, 360, 36)
        .circle(0.0005)
        .extrude(cap_height)
    )
    
    cap_final = cap.union(ridges)
    return cap_final


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")
    
    glass_mat = model.material("glass", rgba=(0.8, 0.9, 0.9, 0.5))
    plastic_mat = model.material("plastic", rgba=(0.1, 0.1, 0.8, 1.0))
    
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(make_bottle(), "bottle_mesh"),
        material=glass_mat,
        name="bottle_body"
    )
    
    # Invisible carriage to hold the prismatic lift motion
    cap_carriage = model.part("cap_carriage")
    
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(make_cap(), "cap_mesh"),
        material=plastic_mat,
        name="cap_body"
    )
    
    # Articulations
    bottle_height = 0.10
    shoulder_height = 0.03
    neck_ring_top = bottle_height + shoulder_height + 0.003 # 0.133
    
    # The cap turns (REVOLUTE)
    turn_joint = model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=cap_carriage,
        origin=Origin(xyz=(0, 0, neck_ring_top)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0, lower=0.0, upper=4 * math.pi)
    )
    
    # The cap lifts (PRISMATIC)
    lift_joint = model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=cap_carriage,
        child=cap,
        origin=Origin(),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.03)
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    turn_joint = object_model.get_articulation("cap_turn")
    lift_joint = object_model.get_articulation("cap_lift")
    
    # At rest, cap is closed and overlaps with the neck threads
    ctx.allow_overlap(
        cap,
        bottle,
        elem_a="cap_body",
        elem_b="bottle_body",
        reason="The cap intentionally overlaps the bottle's neck threads to represent a seated screw closure."
    )
    
    # Check that cap is centered on bottle
    ctx.expect_within(
        cap,
        bottle,
        axes="xy",
        inner_elem="cap_body",
        outer_elem="bottle_body",
        margin=0.0,
        name="cap is centered on bottle"
    )
    
    # Check open pose
    with ctx.pose({turn_joint: 4 * math.pi, lift_joint: 0.025}):
        # At fully open, the cap should clear the bottle
        ctx.expect_gap(
            cap,
            bottle,
            axis="z",
            min_gap=0.001,
            name="cap clears bottle when fully unscrewed"
        )
    
    return ctx.report()

object_model = build_object_model()
