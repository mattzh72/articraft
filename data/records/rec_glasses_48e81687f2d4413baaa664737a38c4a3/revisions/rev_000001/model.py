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

def make_front_frame():
    rim_radius = 0.022
    rim_thickness = 0.002
    bridge_width = 0.02
    center_dist = rim_radius * 2 + bridge_width # 0.064
    
    left_rim = (
        cq.Workplane("XZ")
        .center(-center_dist/2, 0)
        .circle(rim_radius)
        .circle(rim_radius - rim_thickness)
        .extrude(rim_thickness)
        .translate((0, -rim_thickness/2, 0))
    )
    
    right_rim = (
        cq.Workplane("XZ")
        .center(center_dist/2, 0)
        .circle(rim_radius)
        .circle(rim_radius - rim_thickness)
        .extrude(rim_thickness)
        .translate((0, -rim_thickness/2, 0))
    )
    
    bridge = (
        cq.Workplane("XZ")
        .center(0, rim_radius * 0.3)
        .box(bridge_width + 0.004, rim_thickness, rim_thickness)
    )
    
    left_pad = (
        cq.Workplane("XZ")
        .center(-0.012, -0.005)
        .box(0.004, 0.004, 0.008)
    )
    right_pad = (
        cq.Workplane("XZ")
        .center(0.012, -0.005)
        .box(0.004, 0.004, 0.008)
    )
    
    left_hinge_x = -0.058
    right_hinge_x = 0.058
    right_hinge_z = 0.006
    
    left_bracket_base = (
        cq.Workplane("XZ")
        .center(-0.056, 0)
        .box(0.006, 0.002, 0.008)
    )
    left_bracket_cutout = (
        cq.Workplane("XY")
        .center(left_hinge_x, 0)
        .cylinder(0.0042, 0.002)
    )
    left_bracket = left_bracket_base.cut(left_bracket_cutout)
    
    right_bracket_base = (
        cq.Workplane("XZ")
        .center(0.056, right_hinge_z)
        .box(0.006, 0.002, 0.008)
    )
    right_bracket_cutout = (
        cq.Workplane("XY")
        .center(right_hinge_x, 0)
        .workplane(offset=right_hinge_z)
        .cylinder(0.0042, 0.002)
    )
    right_bracket = right_bracket_base.cut(right_bracket_cutout)
    
    return (
        left_rim.union(right_rim).union(bridge)
        .union(left_pad).union(right_pad)
        .union(left_bracket).union(right_bracket)
    )

def make_temple():
    temple_length = 0.110
    drop_length = 0.030
    temple_thickness = 0.002
    knuckle_radius = 0.002
    
    knuckle = cq.Workplane("XY").cylinder(0.0038, knuckle_radius)
    
    L = knuckle_radius + temple_length
    D = drop_length
    mid_Y = L + D * 0.7071
    mid_Z = -D * 0.2929
    end_Y = L + D
    end_Z = -D
    
    path = (
        cq.Workplane("YZ")
        .moveTo(knuckle_radius, 0)
        .lineTo(L, 0)
        .threePointArc((mid_Y, mid_Z), (end_Y, end_Z))
    )
    
    arm = (
        cq.Workplane("XZ")
        .workplane(offset=knuckle_radius)
        .center(0, 0)
        .circle(temple_thickness / 2)
        .sweep(path)
    )
    
    return knuckle.union(arm)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glasses")
    
    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_cadquery(make_front_frame(), "front_frame_mesh"),
        name="front_frame_visual"
    )
    
    left_temple = model.part("left_temple")
    left_temple.visual(
        mesh_from_cadquery(make_temple(), "left_temple_mesh"),
        name="left_temple_visual"
    )
    
    right_temple = model.part("right_temple")
    right_temple.visual(
        mesh_from_cadquery(make_temple(), "right_temple_mesh"),
        name="right_temple_visual"
    )
    
    left_hinge_x = -0.058
    right_hinge_x = 0.058
    right_hinge_z = 0.006
    
    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(left_hinge_x, 0, 0)),
        axis=(0, 0, -1),
        motion_limits=MotionLimits(lower=0.0, upper=1.6, effort=1.0, velocity=1.0)
    )
    
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(right_hinge_x, 0, right_hinge_z)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=1.6, effort=1.0, velocity=1.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    front = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    
    ctx.allow_overlap(front, left_temple, reason="Hinge knuckle fit")
    ctx.allow_overlap(front, right_temple, reason="Hinge knuckle fit")
    
    ctx.expect_overlap(left_temple, right_temple, axes="y", min_overlap=0.1, name="temples overlap in Y when open")
    
    with ctx.pose({left_hinge: 1.57, right_hinge: 1.57}):
        ctx.expect_overlap(left_temple, right_temple, axes="x", min_overlap=0.05, name="temples overlap in X when folded")
        
    return ctx.report()

object_model = build_object_model()
