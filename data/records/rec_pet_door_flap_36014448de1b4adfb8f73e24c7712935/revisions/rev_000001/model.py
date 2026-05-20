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
    model = ArticulatedObject(name="pet_door")

    # Materials
    frame_mat = Material("frame_mat", rgba=(0.9, 0.9, 0.92, 1.0))
    flap_mat = Material("flap_mat", rgba=(0.3, 0.3, 0.35, 0.8))
    screw_mat = Material("screw_mat", rgba=(0.5, 0.5, 0.5, 1.0))

    # Dimensions
    frame_w = 0.30
    frame_h = 0.40
    hole_w = 0.22
    hole_h = 0.32
    corner_r = 0.02
    hole_r = 0.015
    tunnel_depth = 0.04
    bezel_depth = 0.01

    # Frame part
    frame = model.part("frame")
    
    s_inner = (
        cq.Sketch()
        .rect(frame_w, frame_h)
        .vertices()
        .fillet(corner_r)
        .rect(hole_w, hole_h, mode="s")
        .vertices()
        .fillet(hole_r)
    )
    inner_bezel = cq.Workplane("XY").placeSketch(s_inner).extrude(bezel_depth)
    
    s_tunnel = (
        cq.Sketch()
        .rect(hole_w + 0.01, hole_h + 0.01)
        .vertices()
        .fillet(hole_r + 0.005)
        .rect(hole_w, hole_h, mode="s")
        .vertices()
        .fillet(hole_r)
    )
    tunnel = cq.Workplane("XY").workplane(offset=bezel_depth).placeSketch(s_tunnel).extrude(tunnel_depth)
    
    outer_bezel = cq.Workplane("XY").workplane(offset=bezel_depth + tunnel_depth).placeSketch(s_inner).extrude(bezel_depth)
    
    frame_cq = inner_bezel.union(tunnel).union(outer_bezel)
    
    # Add screw indents
    screw_offset_x = frame_w / 2 - 0.02
    screw_offset_y = frame_h / 2 - 0.02
    screw_r = 0.004
    
    frame_cq = (
        cq.Workplane("XY")
        .add(frame_cq)
        .faces(">Z").workplane()
        .pushPoints([
            (screw_offset_x, screw_offset_y),
            (screw_offset_x, -screw_offset_y),
            (-screw_offset_x, screw_offset_y),
            (-screw_offset_x, -screw_offset_y),
        ])
        .circle(screw_r)
        .cutBlind(-0.003)
    )
    
    frame_cq = (
        cq.Workplane("XY")
        .add(frame_cq)
        .faces("<Z").workplane()
        .pushPoints([
            (screw_offset_x, screw_offset_y),
            (screw_offset_x, -screw_offset_y),
            (-screw_offset_x, screw_offset_y),
            (-screw_offset_x, -screw_offset_y),
        ])
        .circle(screw_r)
        .cutBlind(-0.003)
    )

    frame.visual(
        mesh_from_cadquery(frame_cq, "frame_mesh"),
        origin=Origin(),
        material=frame_mat,
        name="frame_visual",
    )

    # Flap part
    flap_w = hole_w - 0.01
    flap_h = hole_h - 0.01
    flap_depth = 0.004
    
    flap_cq = (
        cq.Workplane("XY")
        .rect(flap_w, flap_h)
        .extrude(flap_depth / 2, both=True)
        .edges("|Z").fillet(hole_r - 0.002)
    )
    
    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(flap_cq, "flap_mesh"),
        origin=Origin(xyz=(0, -flap_h / 2, 0)),
        material=flap_mat,
        name="flap_visual",
    )
    
    hinge_y = hole_h / 2 - 0.005
    hinge_z = bezel_depth + tunnel_depth / 2
    
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.5, upper=1.5),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")
    
    ctx.allow_overlap(flap, frame, reason="Flap sits and swings inside the frame hole")
    
    # At rest, flap is inside the tunnel
    ctx.expect_within(
        flap,
        frame,
        axes="xyz",
        margin=0.001,
        name="flap rests inside the frame bounds"
    )
    
    # Flap swings cleanly
    with ctx.pose({hinge: 1.0}):
        ctx.expect_within(flap, frame, axes="xy", margin=0.02)
        
    with ctx.pose({hinge: -1.0}):
        ctx.expect_within(flap, frame, axes="xy", margin=0.02)

    return ctx.report()

object_model = build_object_model()
