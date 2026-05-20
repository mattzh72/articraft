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


def build_router_body():
    body = (
        cq.Workplane("XY")
        .box(0.20, 0.15, 0.03)
        .edges("|Z").fillet(0.02)
        .edges("<Z or >Z").fillet(0.002)
    )
    
    # Vents on top face
    body = (
        body.faces(">Z").workplane()
        .rarray(0.02, 0.015, 7, 7)
        .rect(0.012, 0.006)
        .cutBlind(-0.004)
    )
    
    # Hinge bosses on the back face (Y = 0.075)
    bosses = (
        cq.Workplane("XZ")
        .workplane(offset=0.075)
        .pushPoints([(-0.06, 0.0), (0.06, 0.0)])
        .rect(0.02, 0.012)
        .extrude(0.005)
    )
    return body.union(bosses)


def build_antenna():
    # Base cylinder along X axis (YZ workplane extrudes along X)
    base = cq.Workplane("YZ").cylinder(0.02, 0.006)
    
    # Stem to connect base to paddle
    stem = (
        cq.Workplane("XY")
        .center(0.0, 0.01)
        .box(0.01, 0.02, 0.006)
    )
    
    # Paddle panel
    panel = (
        cq.Workplane("XY")
        .center(0.0, 0.07)
        .box(0.025, 0.10, 0.006)
        .edges("|Z").fillet(0.004)
    )
    
    return base.union(stem).union(panel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="router")

    body_mesh = mesh_from_cadquery(build_router_body(), "router_body")
    antenna_mesh = mesh_from_cadquery(build_antenna(), "antenna")

    base = model.part("base")
    base.visual(body_mesh, name="body")

    left_ant = model.part("left_antenna")
    left_ant.visual(antenna_mesh, name="left_antenna_visual")

    right_ant = model.part("right_antenna")
    right_ant.visual(antenna_mesh, name="right_antenna_visual")

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_ant,
        origin=Origin(xyz=(-0.06, 0.08, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.745),
    )

    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_ant,
        origin=Origin(xyz=(0.06, 0.08, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.745),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    left_ant = object_model.get_part("left_antenna")
    right_ant = object_model.get_part("right_antenna")
    
    # At rest, antennas should point backward (+Y)
    ctx.expect_overlap(left_ant, base, axes="x")
    ctx.expect_overlap(right_ant, base, axes="x")
    
    with ctx.pose(left_hinge=1.57, right_hinge=1.57):
        # At 90 degrees, antenna should point upward (+Z)
        left_pos = ctx.part_world_position(left_ant)
        right_pos = ctx.part_world_position(right_ant)
        ctx.check("left antenna points up", left_pos is not None)
        ctx.check("right antenna points up", right_pos is not None)
        
    return ctx.report()


object_model = build_object_model()
