from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_base():
    return (
        cq.Workplane("XY")
        .circle(0.12).extrude(0.02)
        .faces(">Z").workplane()
        .circle(0.08).extrude(0.10)
        .edges(">Z").fillet(0.005)
    )

def make_pan_fork():
    base = cq.Workplane("XY").circle(0.08).extrude(0.04)
    
    fork = (
        base.faces(">Z").workplane()
        .rect(0.10, 0.16)
        .extrude(0.14)
    )
    
    fork = (
        fork.faces(">Z").workplane()
        .rect(0.12, 0.12)
        .cutBlind(-0.12)
    )
    
    hole_cutter = (
        cq.Workplane("YZ").workplane(offset=-0.1)
        .center(0, 0.14)
        .circle(0.015)
        .extrude(0.2)
    )
    fork = fork.cut(hole_cutter)
    
    return fork

def make_tilt_head():
    drum = (
        cq.Workplane("XZ").workplane(offset=-0.055)
        .circle(0.035)
        .extrude(0.11)
    )
    
    platform = (
        cq.Workplane("XY").workplane(offset=0.02)
        .rect(0.08, 0.08)
        .extrude(0.03)
    )
    
    shafts = (
        cq.Workplane("XZ").workplane(offset=-0.07)
        .circle(0.014)
        .extrude(0.14)
    )
    
    return drum.union(platform).union(shafts)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_tilt_head")

    mat_dark = (0.2, 0.2, 0.2, 1.0)
    mat_light = (0.7, 0.7, 0.7, 1.0)
    mat_lens = (0.1, 0.1, 0.15, 1.0)

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base(), "base_mesh"),
        name="base_visual",
        color=mat_dark
    )

    pan_fork = model.part("pan_fork")
    pan_fork.visual(
        mesh_from_cadquery(make_pan_fork(), "pan_fork_mesh"),
        name="pan_fork_visual",
        color=mat_light
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        mesh_from_cadquery(make_tilt_head(), "tilt_head_mesh"),
        name="tilt_head_visual",
        color=mat_dark
    )

    left_bracket = model.part("left_bracket")
    left_bracket.visual(
        Box((0.08, 0.01, 0.08)),
        name="left_bracket_visual",
        color=mat_light
    )

    right_bracket = model.part("right_bracket")
    right_bracket.visual(
        Box((0.08, 0.01, 0.08)),
        name="right_bracket_visual",
        color=mat_light
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.12, 0.062, 0.06)),
        name="camera_body",
        color=mat_dark
    )
    camera.visual(
        Cylinder(radius=0.025, length=0.05),
        origin=Origin(xyz=(0.085, 0, 0), rpy=(0, 1.5708, 0)),
        name="camera_lens",
        color=mat_lens
    )

    model.articulation(
        "base_to_pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_fork,
        origin=Origin(xyz=(0, 0, 0.12)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-3.14, upper=3.14)
    )

    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_fork,
        child=tilt_head,
        origin=Origin(xyz=(0, 0, 0.14)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5)
    )

    model.articulation(
        "tilt_to_left_bracket",
        ArticulationType.FIXED,
        parent=tilt_head,
        child=left_bracket,
        origin=Origin(xyz=(0, -0.035, 0.089))
    )

    model.articulation(
        "tilt_to_right_bracket",
        ArticulationType.FIXED,
        parent=tilt_head,
        child=right_bracket,
        origin=Origin(xyz=(0, 0.035, 0.089))
    )

    model.articulation(
        "tilt_to_camera",
        ArticulationType.FIXED,
        parent=tilt_head,
        child=camera,
        origin=Origin(xyz=(0, 0, 0.09))
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "tilt_head", "pan_fork",
        reason="Tilt head shafts are captured in the pan fork holes."
    )
    ctx.allow_overlap(
        "left_bracket", "tilt_head",
        reason="Bracket is bolted into the platform, slight overlap for exact seating."
    )
    ctx.allow_overlap(
        "right_bracket", "tilt_head",
        reason="Bracket is bolted into the platform, slight overlap for exact seating."
    )
    ctx.allow_overlap(
        "camera", "left_bracket",
        reason="Camera is clamped between brackets."
    )
    ctx.allow_overlap(
        "camera", "right_bracket",
        reason="Camera is clamped between brackets."
    )

    ctx.expect_contact("pan_fork", "base")
    
    with ctx.pose(pan_to_tilt=1.0):
        ctx.expect_overlap("tilt_head", "pan_fork", axes="y", min_overlap=0.005)

    return ctx.report()

object_model = build_object_model()
